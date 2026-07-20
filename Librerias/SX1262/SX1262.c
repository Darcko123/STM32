/**
 * @file SX1262.c
 * @brief Librería para la gestión de un módulo LoRa SX1262 (Semtech) en STM32.
 *
 * @details Implementa la abstracción SPI, la secuencia de inicialización, las
 *          rutinas de transmisión y recepción LoRa (bloqueantes y por IRQ), la
 *          aplicación de configuración de red/modulación y la telemetría RSSI/SNR.
 *
 * @author Daniel Ruiz
 * @date Abril 27, 2026
 * @version 1.6.0
 */

#include "SX1262.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static SPI_HandleTypeDef *SX1262_hspi = NULL; /**< Manejador de la interfaz SPI utilizada para cominucarse con el módulo*/

static GPIO_TypeDef *NSS_GPIO_Port    = NULL; /**< Puerto GPIO del pin de datos del SX1262 */
static uint16_t NSS_GPIO_Pin          = 0;    /**< Pin GPIO del pin de datos del SX1262 */

static GPIO_TypeDef *BUSY_GPIO_Port   = NULL;
static uint16_t BUSY_GPIO_Pin         = 0;

static GPIO_TypeDef *DIO_GPIO_Port    = NULL;
static uint16_t DIO_GPIO_Pin          = 0;

static GPIO_TypeDef *RST_GPIO_Port    = NULL;
static uint16_t RST_GPIO_Pin          = 0;

static uint8_t SX1262_Initialized     = 0;    /**< Bandera para verificar si el módulo está inicializado */

static lora_config_t  SX1262_LoRa_CurrentConfig;   /**< Configuración actual aplicada al módulo */
static fsk_config_t   SX1262_FSK_CurrentConfig;    /**< Configuración FSK actual (aplicada o cacheada por defecto) */

/**
 * @brief Último registro IRQ leído durante el consumo de un evento RX.
 *        Solo para diagnóstico: permite al llamante saber qué bits levantó el
 *        chip cuando GetReceivedPacket no devuelve SX1262_OK. Se expone con
 *        SX1262_GetLastIrqStatus().
 */
static uint16_t SX1262_LastIrqStatus = 0;

/**
 * @brief Bandera de recepción no bloqueante (productor: ISR, consumidor: main
 * loop). SX1262_IRQ_Handler() la activa a 1 cuando DIO1 sube y TxActive == 0.
 *        El main loop la lee, la pone a 0 y llama SX1262_LoRa_GetReceivedPacket().
 */
volatile uint8_t SX1262_LoRa_RxDoneFlag = 0;

/**
 * @brief Bandera de transmisión no bloqueante (productor: ISR, consumidor: main
 * loop). SX1262_IRQ_Handler() la activa a 1 cuando DIO1 sube y TxActive == 1.
 *        El main loop la lee, la pone a 0 y llama SX1262_LoRa_GetTransmitStatus().
 */
volatile uint8_t SX1262_LoRa_TxDoneFlag = 0;

// ============================================================================
// PRESETS MESHTASTIC (US, 915 MHz)
// ============================================================================

/**
 * @brief Inicializador de preset Meshtastic. Los presets solo difieren en SF, BW
 *        y CR; el resto de campos son comunes a la región US (915 MHz).
 */
#define MESHTASTIC_PRESET(sf, bw, cr)            \
{                                                \
    .frequency        = MESHTASTIC_US_CH0_FREQ,  \
    .spreading_factor = (sf),                    \
    .bandwidth        = (bw),                    \
    .coding_rate      = (cr),                    \
    .tx_power         = 20,                      \
    .preamble_len     = 16,                      \
    .iq_inverted      = false,                   \
    .network_mode     = LORA_NETWORK_MESHTASTIC, \
    .lora_sync_word   = 0,                       \
    .config_pending   = true                     \
}

const lora_config_t LongSlow   = MESHTASTIC_PRESET(12, BW_125_KHZ, CR_4_8);
const lora_config_t LongFast   = MESHTASTIC_PRESET(11, BW_250_KHZ, CR_4_5);
const lora_config_t MediumSlow = MESHTASTIC_PRESET(10, BW_250_KHZ, CR_4_5);
const lora_config_t MediumFast = MESHTASTIC_PRESET(9,  BW_250_KHZ, CR_4_5);
const lora_config_t ShortSlow  = MESHTASTIC_PRESET(8,  BW_250_KHZ, CR_4_5);
const lora_config_t ShortFast  = MESHTASTIC_PRESET(7,  BW_250_KHZ, CR_4_5);

/**
 * @brief Semáforo binario privado: 1 si hay una transmisión IT en curso.
 *        Escrito desde el main loop (StartTransmitIT / GetTransmitStatus /
 * AbortTransmit). Leido desde el ISR (SX1262_IRQ_Handler) para despachar la
 * bandera correcta. Declarado volatile porque el ISR lo lee desde contexto de
 * interrupción.
 */
static volatile uint8_t SX1262_TxActive = 0;

/**
 * @brief Semáforo binario privado: 1 si hay una recepción IT en curso (modo RX
 *        continuo armado). Escrito desde el main loop (StartReceiveIT arma,
 *        AbortReceive libera, StartTransmitIT lo libera al forzar Standby).
 *        Se usa únicamente como guardia de reentrada; el dispatcher del ISR
 *        decide TX/RX con SX1262_TxActive (TX y RX son mutuamente excluyentes).
 *        Declarado volatile por coherencia con el resto de banderas compartidas.
 */
static volatile uint8_t SX1262_RxActive = 0;

// ============================================================================
// FUNCIONES PRIVADAS - ABSTRACCIÓN SPI
// ============================================================================
/**
 * @brief Espera a que el pin BUSY del SX1262 se libere, indicando que el módulo
 * está listo para la siguiente operación.
 *
 * @return SX1262_Status_t
 */
static SX1262_Status_t sx1262_WaitBusy(void)
{
  uint32_t tickstart = HAL_GetTick();
  while (HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_GPIO_Pin) == GPIO_PIN_SET)
  {
    if ((HAL_GetTick() - tickstart) > SX1262_MAX_BUSY_TIMEOUT)
    {
      return SX1262_TIMEOUT;
    }
  }
  return SX1262_OK;
}

/**
 * @brief Escribe un comando al SX1262 a través de SPI, manejando la señal NSS y
 * el pin BUSY.
 *
 * @param cmd Comando a enviar
 * @param buffer Puntero a los datos asociados al comando (puede ser NULL si no
 * se requieren datos)
 * @param size Tamaño de los datos a enviar (0 si no se requieren datos)
 * @return SX1262_Status_t
 */
static SX1262_Status_t sx1262_WriteCommand(uint8_t cmd, uint8_t *buffer, uint16_t size)
{
    if (sx1262_WaitBusy() != SX1262_OK)
    {
        return SX1262_TIMEOUT;
    }

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(SX1262_hspi, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }

    if (size > 0 && buffer != NULL)
    {
        if (HAL_SPI_Transmit(SX1262_hspi, buffer, size, HAL_MAX_DELAY) != HAL_OK)
        {
            HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
            return SX1262_ERROR;
        }
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    return SX1262_OK;
}

/**
 * @brief Lee datos del SX1262 a través de SPI, manejando la señal NSS y el pin
 * BUSY.
 *
 * @param cmd Comando de lectura a enviar
 * @param buffer Puntero al buffer donde se almacenarán los datos leídos
 * @param size Tamaño de los datos a leer
 * @return SX1262_Status_t
 */
static SX1262_Status_t sx1262_ReadCommand(uint8_t cmd, uint8_t *buffer, uint16_t size)
{
    if (sx1262_WaitBusy() != SX1262_OK)
    {
        return SX1262_TIMEOUT;
    }

    uint8_t nop = 0x00;
    uint8_t dump; // Variable "basurero" para volcar el RX FIFO y prevenir el flag OVR

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);

    // TransmitReceive sincroniza el bus limpiando datos entrantes
    if (HAL_SPI_TransmitReceive(SX1262_hspi, &cmd, &dump, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }

    if (HAL_SPI_TransmitReceive(SX1262_hspi, &nop, &dump, 1, HAL_MAX_DELAY) != HAL_OK) // El SX1262 retorna un STATUS primero
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }

    if (size > 0 && buffer != NULL)
    {
        // En este paso, el buffer del STM32 está limpio. El RX de datos útiles es seguro sin riesgo de OVR.
        if (HAL_SPI_Receive(SX1262_hspi, buffer, size, HAL_MAX_DELAY) != HAL_OK)
        {
            HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
            return SX1262_ERROR;
        }
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    return SX1262_OK;
}

/**
 * @brief Escribe datos en el buffer interno del SX1262 a través de SPI,
 * manejando la señal NSS y el pin BUSY.
 *
 * @param offset Offset dentro del buffer interno del SX1262 donde se escribirán
 * los datos
 * @param data Puntero al buffer de datos a escribir
 * @param length Longitud de los datos a escribir (máximo 255 bytes)
 * @return SX1262_Status_t
 */
static SX1262_Status_t sx1262_WriteBuffer(uint8_t offset, uint8_t *data, uint8_t length)
{
    if (sx1262_WaitBusy() != SX1262_OK)
    {
        return SX1262_TIMEOUT;
    }

    uint8_t cmd[2] = {SX126X_CMD_WRITE_BUFFER, offset};

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(SX1262_hspi, cmd, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }

    if (HAL_SPI_Transmit(SX1262_hspi, data, length, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    return SX1262_OK;
}

/**
 * @brief Lee datos del buffer interno del SX1262 a través de SPI, manejando la
 * señal NSS y el pin BUSY.
 *
 * @param offset Offset dentro del buffer interno del SX1262 desde donde se
 * leerán los datos
 * @param data Puntero al buffer donde se almacenarán los datos leídos
 * @param length Longitud de los datos a leer (máximo 255 bytes)
 * @return SX1262_Status_t
 */
static SX1262_Status_t sx1262_ReadBuffer(uint8_t offset, uint8_t *data, uint8_t length)
{
    if (sx1262_WaitBusy() != SX1262_OK)
    {
        return SX1262_TIMEOUT;
    }

    uint8_t cmd[3] = {SX126X_CMD_READ_BUFFER, offset, 0x00}; // NOP byte automatically handled here
    uint8_t dump[3];                                         // Buffer basura para capturar las respuestas ignoradas al transmitir

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);

    // Usamos TransmitReceive para evadir la condición de error Overrun
    if (HAL_SPI_TransmitReceive(SX1262_hspi, cmd, dump, 3, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }

    // El buffer RX está ahora limpio internamente, procedemos con lectura segura
    if (HAL_SPI_Receive(SX1262_hspi, data, length, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    return SX1262_OK;
}

/**
 * @brief Realiza un reset hardware del SX1262 utilizando el pin RST, siguiendo
 * la secuencia recomendada por el datasheet.
 */
static void sx1262_Reset(void)
{
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);

    HAL_GPIO_WritePin(RST_GPIO_Port, RST_GPIO_Pin, GPIO_PIN_SET);
    HAL_Delay(10); // Permitir inicialización
}

/**
 * @brief Limpia todas las alertas pendientes del chip (IRQ mask 0x03FF).
 *
 * @return SX1262_Status_t Resultado de la escritura del comando.
 */
static SX1262_Status_t sx1262_ClearIrq(void)
{
    uint8_t buf[2] = { 0x03, 0xFF }; // Limpiar todo (0x03FF)
    return sx1262_WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2);
}

/**
 * @brief Coloca el chip en modo Standby RC.
 *
 * @return SX1262_Status_t Resultado de la escritura del comando.
 */
static SX1262_Status_t sx1262_Standby(void)
{
    uint8_t buf[1] = { SX126X_STANDBY_RC };
    return sx1262_WriteCommand(SX126X_CMD_SET_STANDBY, buf, 1);
}

/**
 * @brief Convierte el valor de ancho de banda (BW) del enum a su equivalente en
 * Hz, necesario para cálculos internos como LDRO.
 *
 * @param bw Ancho de banda representado por el enum lora_signal_bandwidth_t
 * @return uint32_t
 */
static uint32_t sx1262_BandwidthToHz(lora_signal_bandwidth_t bw)
{
    static const struct
    {
        lora_signal_bandwidth_t bw;
        uint32_t hz;
    } bw_table[] = {
        { BW_7_8_KHZ,    7800UL   },
        { BW_10_4_KHZ,   10400UL  },
        { BW_15_6_KHZ,   15600UL  },
        { BW_20_8_KHZ,   20800UL  },
        { BW_31_25_KHZ,  31250UL  },
        { BW_41_7_KHZ,   41700UL  },
        { BW_62_5_KHZ,   62500UL  },
        { BW_125_KHZ,    125000UL },
        { BW_250_KHZ,    250000UL },
        { BW_500_KHZ,    500000UL },
    };

    for (uint8_t i = 0; i < sizeof(bw_table) / sizeof(bw_table[0]); i++)
    {
        if (bw_table[i].bw == bw)
        {
            return bw_table[i].hz;
        }
    }

    return 0UL; // BW desconocido
}

/**
 * @brief Calcula si se debe activar LowDataRateOptimize (LDRO).
 *        El datasheet exige LDRO cuando el tiempo de símbolo >= 16.38 ms
 *        (datasheet SX1262, sección 6.1.1.4).
 *        Fórmula: T_sym_us = (2^SF * 1000000) / BW_Hz
 *        Se usa aritmética de 64 bits para evitar desbordamiento con SF12/BW
 * bajo.
 *
 * @param sf   Spreading Factor (5–12)
 * @param bw   Bandwidth enum
 * @return     1 si LDRO debe activarse, 0 en caso contrario
 */
static uint8_t sx1262_ComputeLDRO(uint8_t sf, lora_signal_bandwidth_t bw)
{
    uint32_t bw_hz = sx1262_BandwidthToHz(bw);

    if (bw_hz == 0)
    {
        return 0; // BW desconocido, no activar LDRO
    }

    // T_sym_us = (1 << SF) * 1_000_000 / BW_Hz
    // Umbral: 16380 us  (≈ 16.38 ms, según datasheet)
    uint64_t t_sym_us = ((uint64_t)1 << sf) * 1000000ULL / (uint64_t)bw_hz;

    return (t_sym_us >= 16380) ? 1U : 0U;
}

/**
 * @brief Devuelve el cadDetPeak recomendado para el Spreading Factor activo,
 *        asumiendo un CAD de 2 símbolos.
 *
 *        cadDetPeak fija el umbral de correlación con el preámbulo LoRa: un valor
 *        bajo dispara falsas detecciones sobre el ruido y uno alto hace que el
 *        chip pase por alto señales débiles. El datasheet (§13.4.7) delega la
 *        elección en la nota de aplicación de Semtech AN1200.48, de donde sale
 *        esta tabla. Si en un entorno concreto abundan las falsas detecciones,
 *        subir el valor una o dos unidades.
 *
 * @param sf Spreading Factor (5–12)
 * @return uint8_t Valor de cadDetPeak
 */
static uint8_t sx1262_CadDetPeak(uint8_t sf)
{
    static const uint8_t det_peak[] = { 21, 21, 22, 22, 23, 24, 25, 28 }; // SF5..SF12

    if (sf < 5 || sf > 12)
    {
        return 22; // SF fuera de rango: valor intermedio de la tabla
    }

    return det_peak[sf - 5];
}

/**
 * @brief Calcula el Time on Air (ToA) aproximado de un paquete LoRa en
 * milisegundos.
 *
 *  La fórmula sigue la nota de aplicación de Semtech AN1200.13:
 *
 *  N_sym_payload = 8 + max( ceil( (8*n - 4*SF + 28 + 16*crc - 20*ih) /
 *                                 (4*(SF - 2*ldro)) ) * (CR+4), 0 )
 *  ToA = (N_preamble + 4.25 + N_sym_payload) * T_sym
 *
 * @param payload_len  Bytes del payload
 * @param config       Configuración LoRa activa
 * @return uint32_t    ToA en milisegundos (mínimo 1 ms)
 */
static uint32_t sx1262_ComputeToA_ms(uint8_t payload_len, const lora_config_t *config)
{
    uint32_t bw_hz = sx1262_BandwidthToHz(config->bandwidth);
    if (bw_hz == 0)
    {
        return 5000U; // Valor de seguridad si BW es inválido
    }

    uint8_t sf = config->spreading_factor;
    uint8_t cr = (uint8_t)config->coding_rate; // 1=CR4/5 .. 4=CR4/8
    uint8_t ldro = sx1262_ComputeLDRO(sf, config->bandwidth);

    // T_sym en microsegundos: (2^SF * 1_000_000) / BW_Hz
    uint64_t t_sym_us = ((uint64_t)1 << sf) * 1000000ULL / (uint64_t)bw_hz;

    // Número de símbolos del payload (header explícito, CRC on)
    // Denominador: 4 * (SF - 2*LDRO)
    int32_t denom = 4 * ((int32_t)sf - 2 * (int32_t)ldro);
    if (denom <= 0)
    {
        denom = 1; // Protección contra divisiones por cero o negativas
    }

    // Numerador: 8*payload - 4*SF + 28 + 16 (CRC on) - 0 (header explícito => ih=0)
    int32_t numer = 8 * (int32_t)payload_len - 4 * (int32_t)sf + 44;

    // ceil(numer / denom) usando división entera con redondeo hacia arriba
    int32_t ceil_val = (numer > 0) ? ((numer + denom - 1) / denom) : 0;
    int32_t n_sym_payload = 8 + ceil_val * ((int32_t)cr + 4);
    if (n_sym_payload < 8)
    {
        n_sym_payload = 8;
    }

    // Número total de símbolos: preamble + 4.25 (inicio) + payload
    // Multiplicamos por 4 para evitar fracciones: (preamble + payload + 4)*4 + 1
    // (el +1 es 0.25*4)
    uint64_t n_sym_total_x4 = ((uint64_t)config->preamble_len + (uint64_t)n_sym_payload + 4ULL) * 4ULL + 1ULL;

    // ToA en microsegundos: n_sym_total_x4 * t_sym_us / 4
    uint64_t toa_us = (n_sym_total_x4 * t_sym_us) / 4ULL;

    // Convertir a ms (redondeo hacia arriba) y garantizar mínimo 1 ms
    uint32_t toa_ms = (uint32_t)((toa_us + 999ULL) / 1000ULL);

    return (toa_ms < 1U) ? 1U : toa_ms;
}

/**
 * @brief Calcula el Time on Air (ToA) aproximado de un paquete FSK/GFSK en
 * milisegundos.
 *
 *  ToA = (preamble_bytes + sync_word_bytes + length_byte(si variable) +
 *         payload_bytes + crc_bytes) * 8 / bitrate
 *
 * @param payload_len  Bytes del payload
 * @param config       Configuración FSK activa
 * @return uint32_t    ToA en milisegundos (mínimo 1 ms)
 */
static uint32_t sx1262_ComputeFskToA_ms(uint8_t payload_len, const fsk_config_t *config)
{
    if (config->bitrate == 0)
    {
        return 5000U; // Valor de seguridad si el bitrate es inválido
    }

    uint8_t crc_bytes;
    switch (config->crc_type)
    {
        case FSK_CRC_1_BYTE:
        case FSK_CRC_1_BYTE_INV:
            crc_bytes = 1U;
            break;
        case FSK_CRC_2_BYTE:
        case FSK_CRC_2_BYTE_INV:
            crc_bytes = 2U;
            break;
        default: // FSK_CRC_OFF
            crc_bytes = 0U;
            break;
    }

    uint32_t total_bytes = (uint32_t)config->preamble_len + config->sync_word_len +
                           (config->fixed_length ? 0U : 1U) + payload_len + crc_bytes;

    uint64_t toa_us = ((uint64_t)total_bytes * 8ULL * 1000000ULL) / config->bitrate;
    uint32_t toa_ms = (uint32_t)((toa_us + 999ULL) / 1000ULL);

    return (toa_ms < 1U) ? 1U : toa_ms;
}

/**
 * @brief Espera activa a que DIO1 suba. Usado por las rutas bloqueantes de TX
 *        (que son la ruta IT con esta espera intercalada entre StartTransmitIT y
 *        GetTransmitStatus) y por el CAD. Es agnóstico al evento: quién decide
 *        qué significa el flanco es el llamante, al leer el registro IRQ.
 *
 * @param timeout_ms Timeout de software para el bucle de espera en DIO1
 * @return SX1262_Status_t SX1262_OK si DIO1 subió, SX1262_TIMEOUT si expiró el
 *                         plazo sin evento.
 */
static SX1262_Status_t sx1262_WaitDio1(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(DIO_GPIO_Port, DIO_GPIO_Pin) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            return SX1262_TIMEOUT;
        }
    }

    return SX1262_OK;
}

/**
 * @brief Consume un evento RX: lee el registro IRQ, evalúa los bits de error,
 *        obtiene offset y longitud con GetRxBufferStatus y vuelca el payload del
 *        buffer interno. Común a LoRa y GFSK: el registro IRQ, el estado del
 *        buffer y la lectura del payload no dependen del modo de modulación
 *        (el bit HEADER_ERR nunca se activa en GFSK, por lo que su chequeo es
 *        inofensivo). Los llamantes públicos aportan la validación de estado.
 *
 * @param data       Buffer destino del payload recibido.
 * @param max_length Capacidad de `data` en bytes. El chip puede reportar hasta 255
 *                   bytes y el tamaño lo decide el emisor, así que este límite es la
 *                   única protección contra desbordar el buffer del llamante.
 * @param length     Puntero donde se escribe la longitud del payload (bytes).
 * @return SX1262_Status_t SX1262_OK si el paquete es válido, SX1262_TIMEOUT si el
 *                         IRQ indica timeout de chip, SX1262_RX_BUFFER_TOO_SMALL si
 *                         el payload no cabe en `data`, SX1262_ERROR ante CRC/header
 *                         inválido, ausencia de RX_DONE o fallo SPI.
 */
static SX1262_Status_t sx1262_GetReceivedPacket(uint8_t *data, uint8_t max_length, uint8_t *length)
{
    // 1. Leer registro IRQ del chip
    uint8_t irqStatus[2];
    SX1262_Status_t st = sx1262_ReadCommand(SX126X_CMD_GET_IRQ_STATUS, irqStatus, 2);
    if (st != SX1262_OK)
    {
        return st;
    }
    uint16_t irqReg = ((uint16_t)irqStatus[0] << 8) | irqStatus[1];
    SX1262_LastIrqStatus = irqReg; // Cachear para diagnóstico (SX1262_GetLastIrqStatus)

    // 2. Limpiar IRQ siempre (independientemente del resultado)
    sx1262_ClearIrq();

    // 3. Evaluar bits de error con prioridad:
    //    TIMEOUT > CRC_ERR > HEADER_ERR > ausencia de RX_DONE
    if (irqReg & SX126X_IRQ_TIMEOUT)
    {
        return SX1262_TIMEOUT;
    }

    if (irqReg & (SX126X_IRQ_CRC_ERR | SX126X_IRQ_HEADER_ERR))
    {
        return SX1262_ERROR;
    }

    if ((irqReg & SX126X_IRQ_RX_DONE) == 0)
    {
        // DIO1 subió sin RX_DONE ni bits de error. No es un fallo del enlace:
        // suele ser un flanco espurio o un falso sync detectado en el ruido
        // (probable con sync words cortos). El chip sigue en RX continuo, así
        // que el llamante solo debe ignorar el evento y seguir escuchando.
        return SX1262_RX_NO_PACKET;
    }

    // 4. Obtener offset y tamaño del paquete en el buffer interno
    uint8_t rxBufferStatus[2];
    st = sx1262_ReadCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, rxBufferStatus, 2);
    if (st != SX1262_OK)
    {
        return st;
    }

    *length = rxBufferStatus[0];        // Número de bytes del payload
    uint8_t offset = rxBufferStatus[1]; // Offset base en el buffer del chip

    // 5. El tamaño del payload lo decide el emisor (hasta 255 bytes), no la
    //    configuración local: sin este chequeo un paquete grande desbordaría el
    //    buffer del llamante. Se rechaza el paquete entero en lugar de truncarlo
    //    —un payload parcial es indistinguible de uno íntegro para el llamante—
    //    y *length ya trae el tamaño real para que pueda redimensionar su buffer.
    //    El IRQ ya se limpió en el paso 2: el chip sigue en RX continuo.
    if (*length > max_length)
    {
        return SX1262_RX_BUFFER_TOO_SMALL;
    }

    // 6. Leer payload desde el buffer interno del SX1262
    return sx1262_ReadBuffer(offset, data, *length);
}

/**
 * @brief Consume el evento de TX: lee el registro IRQ, lo limpia y libera el
 *        semáforo SX1262_TxActive. Común a LoRa y FSK: el registro IRQ y el
 *        semáforo no dependen del modo de modulación.
 *
 * @return SX1262_Status_t SX1262_OK si TX_DONE confirmado, SX1262_TIMEOUT si el
 *                         chip reporta timeout interno, SX1262_ERROR si DIO1
 *                         subió sin TX_DONE válido o si falla el SPI.
 */
static SX1262_Status_t sx1262_GetTransmitStatus(void)
{
    // 1. Leer registro IRQ del chip
    uint8_t irqStatus[2];

    SX1262_Status_t st = sx1262_ReadCommand(SX126X_CMD_GET_IRQ_STATUS, irqStatus, 2);
    if (st != SX1262_OK)
    {
        SX1262_TxActive = 0; // Liberar semáforo aunque haya fallo SPI
        return st;
    }

    uint16_t irqReg = ((uint16_t)irqStatus[0] << 8) | irqStatus[1];

    // 2. Limpiar IRQ siempre (independientemente del resultado)
    sx1262_ClearIrq();

    // 3. Liberar semáforo — TX ya terminó (con éxito o error)
    SX1262_TxActive = 0;

    // 4. Evaluar resultado: TIMEOUT tiene prioridad sobre TX_DONE ausente
    if (irqReg & SX126X_IRQ_TIMEOUT)
    {
        sx1262_Standby();
        return SX1262_TIMEOUT;
    }

    if ((irqReg & SX126X_IRQ_TX_DONE) == 0)
    {
        // DIO1 subió pero TX_DONE no está activo: condición inesperada
        return SX1262_ERROR;
    }

    return SX1262_OK;
}

/**
 * @brief Arma el chip en modo TX y lanza SetTx: Standby RC, base addresses,
 *        escritura del payload, parámetros de paquete, limpieza de IRQ y ruteo
 *        de TX_DONE/TIMEOUT a DIO1. Compartido por LoRa y FSK; lo único que
 *        distingue a cada modo es el payload de SetPacketParams, que arma el
 *        llamante.
 *
 *        Gestiona los semáforos: libera SX1262_RxActive (el Standby cancela
 *        cualquier RX continuo previo) y arma SX1262_TxActive antes de SetTx,
 *        para no perder el IRQ si el paquete es tan corto que DIO1 sube antes
 *        de que el main loop lo compruebe. Hace rollback si SetTx falla.
 *
 * @param data       Puntero al buffer de datos a transmitir
 * @param length     Longitud de los datos (máximo 255 bytes)
 * @param pkt_params Payload del comando SetPacketParams ya formateado por el
 *                   llamante (6 bytes en LoRa, 9 en GFSK), con PayloadLength
 *                   ajustado a `length`
 * @param pp_len     Número de bytes válidos en pkt_params
 * @return SX1262_Status_t SX1262_OK si el chip entró en modo TX.
 */
static SX1262_Status_t sx1262_ArmTx(uint8_t *data, uint8_t length,
                                    uint8_t *pkt_params, uint8_t pp_len)
{
    uint8_t buf[8];
    SX1262_Status_t st = SX1262_OK;

    // 1. Standby RC (chip debe estar en Standby antes de configurar TX)
    st = st ? st : sx1262_Standby();

    // 2. Fijar base addresses del buffer interno
    buf[0] = 0x00; // TX base en offset 0
    buf[1] = 0x00; // RX base en offset 0
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2);

    // 3. Escribir payload en el buffer interno del chip
    st = st ? st : sx1262_WriteBuffer(0x00, data, length);

    // 4. Parámetros de paquete propios del modo (los arma el llamante)
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, pkt_params, pp_len);

    // 5. Limpiar IRQ pendientes
    st = st ? st : sx1262_ClearIrq();

    // 6. Enrutar TX_DONE y TIMEOUT a DIO1
    uint16_t irqMask = SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT;
    buf[0] = (irqMask >> 8) & 0xFF;
    buf[1] = irqMask & 0xFF; // IRQ global mask
    buf[2] = (irqMask >> 8) & 0xFF;
    buf[3] = irqMask & 0xFF; // DIO1 mask
    buf[4] = 0x00;
    buf[5] = 0x00; // DIO2 (no usado)
    buf[6] = 0x00;
    buf[7] = 0x00; // DIO3 (no usado)
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8);

    // Abortar antes de armar TX si algún comando de configuración falló
    if (st != SX1262_OK)
    {
        return st;
    }

    // 7. Armar el semáforo ANTES de SetTx para evitar perder el IRQ si el
    //    paquete es muy corto y DIO1 sube antes de que el main loop lo compruebe.
    //    La TX ha forzado Standby (paso 1), por lo que cualquier RX continuo
    //    previo queda cancelado: liberar SX1262_RxActive para no dejar estado obsoleto.
    SX1262_RxActive = 0;
    SX1262_TxActive = 1;

    // 8. Iniciar TX (timeout = 0x000000 => sin timeout de chip; soft-timeout en
    // main loop)
    buf[0] = 0x00;
    buf[1] = 0x00;
    buf[2] = 0x00;
    st = sx1262_WriteCommand(SX126X_CMD_SET_TX, buf, 3);
    if (st != SX1262_OK)
    {
        SX1262_TxActive = 0; // Rollback del semáforo si el comando falla
        return st;
    }

    return SX1262_OK;
}

/**
 * @brief Arma el chip en modo RX: Standby RC, limpia IRQ pendientes, habilita las
 *        fuentes de IRQ en DIO1 y lanza SetRx con el timeout de chip indicado.
 *        Compartido por la ruta bloqueante (SX1262_LoRa_Receive) y la ruta por
 *        interrupción (SX1262_LoRa_StartReceiveIT); la única diferencia entre
 *        ambas es el timeout y el manejo del semáforo SX1262_RxActive, que queda
 *        en manos del llamante.
 *
 * @param chipTimeout Timeout interno del chip en ticks de 15.625 us (24 bits).
 *                    0xFFFFFF => RX continuo, sin timeout de chip.
 * @return SX1262_Status_t SX1262_OK si toda la secuencia de armado tuvo éxito.
 */
static SX1262_Status_t sx1262_ArmRx(uint32_t chipTimeout)
{
    uint8_t buf[8];
    SX1262_Status_t st = SX1262_OK;

    // Set Standby
    st = st ? st : sx1262_Standby();

    // Limpiar IRQ pendientes
    st = st ? st : sx1262_ClearIrq();

    // Habilitar IRQs en DIO1: RxDone | Timeout | CRC_ERR | HeaderErr
    // HeaderErr (bit 4) se incluye para detectar paquetes con header inválido.
    uint16_t irqMask = SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT |
                       SX126X_IRQ_CRC_ERR | SX126X_IRQ_HEADER_ERR;
    buf[0] = (irqMask >> 8) & 0xFF;
    buf[1] = irqMask & 0xFF; // IRQ global mask
    buf[2] = (irqMask >> 8) & 0xFF;
    buf[3] = irqMask & 0xFF; // DIO1 mask
    buf[4] = 0x00;
    buf[5] = 0x00; // DIO2 (no usado)
    buf[6] = 0x00;
    buf[7] = 0x00; // DIO3 (no usado)
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8);

    // Iniciar recepción
    buf[0] = (chipTimeout >> 16) & 0xFF;
    buf[1] = (chipTimeout >> 8) & 0xFF;
    buf[2] = chipTimeout & 0xFF;
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_RX, buf, 3);

    return st;
}

/**
 * @brief Cancela la TX en curso: fuerza Standby RC, limpia los IRQ residuales y
 *        libera el semáforo SX1262_TxActive. Común a LoRa y FSK.
 *
 * @return SX1262_Status_t SX1262_OK si el chip volvió a Standby.
 */
static SX1262_Status_t sx1262_AbortTransmit(void)
{
    // Regresar a Standby RC para detener la transmisión
    SX1262_Status_t st = sx1262_Standby();
    if (st != SX1262_OK)
    {
        SX1262_TxActive = 0; // Liberar semáforo aunque falle el comando
        return st;
    }

    // Limpiar IRQ residuales
    sx1262_ClearIrq();

    // Liberar semáforo
    SX1262_TxActive = 0;

    return SX1262_OK;
}

/**
 * @brief Cancela la RX en curso: libera el semáforo SX1262_RxActive, fuerza
 *        Standby RC y limpia los IRQ residuales. Común a LoRa y FSK.
 *
 * @return SX1262_Status_t SX1262_OK si el chip volvió a Standby.
 */
static SX1262_Status_t sx1262_AbortReceive(void)
{
    // Liberar el semáforo de RX aunque el comando falle: la intención es
    // abandonar el modo RX, y dejarlo en 1 impediría rearmar RX más tarde.
    SX1262_RxActive = 0;

    // Regresar a Standby RC para detener la escucha
    SX1262_Status_t st = sx1262_Standby();
    if (st != SX1262_OK)
    {
        return st;
    }

    // Limpiar IRQ residuales
    sx1262_ClearIrq();

    return SX1262_OK;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el módulo con los pines y parámetros por defecto. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_Init(SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef *nss_port, uint16_t nss_pin,
                            GPIO_TypeDef *busy_port, uint16_t busy_pin,
                            GPIO_TypeDef *dio_port, uint16_t dio_pin,
                            GPIO_TypeDef *rst_port, uint16_t rst_pin)
{
    // Validar parámetros de entrada
    if (hspi == NULL || nss_port == NULL || busy_port == NULL ||
        dio_port == NULL || rst_port == NULL)
    {
        return SX1262_INVALID_PARAM;
    }

    // Almacenar configuración para uso en funciones posteriores
    SX1262_hspi = hspi;
    NSS_GPIO_Port = nss_port;
    NSS_GPIO_Pin = nss_pin;
    BUSY_GPIO_Port = busy_port;
    BUSY_GPIO_Pin = busy_pin;
    DIO_GPIO_Port = dio_port;
    DIO_GPIO_Pin = dio_pin;
    RST_GPIO_Port = rst_port;
    RST_GPIO_Pin = rst_pin;
    SX1262_Initialized = 0; // Marcar como no inicializada hasta que se termine el proceso

    sx1262_Reset();
    uint8_t buf[8];
    SX1262_Status_t st = SX1262_OK;

    // 0. Si se usara Sleep, idealmente usaríamos Wakeup.
    // Llamamos Wakeup explícitamente para emular la robustez de RadioLib
    SX1262_Wakeup();

    // 1. Standby RC mode
    st = st ? st : sx1262_Standby();

    // 2. Set Packet Type (LORA)
    buf[0] = SX126X_PACKET_TYPE_LORA;
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_PACKET_TYPE, buf, 1);

    if (st != SX1262_OK)
    {
        return st;
    }

    // Configuración base por defecto
    lora_config_t default_lora_config =
    {
        .frequency = 915000000,
        .spreading_factor = 7,
        .bandwidth = BW_125_KHZ,
        .coding_rate = CR_4_5,
        .tx_power = 22,
        .preamble_len = 12,
        .iq_inverted = false,
        .network_mode = LORA_NETWORK_PRIVATE,
        .lora_sync_word = 0,
        .config_pending = false
    };


    fsk_config_t default_fsk_config =
    {
        .frequency = 915000000,
        .bitrate = 50000,
        .freq_dev = 25.0f,
        .shaping = LORA_FSK_SHAPING_NONE,
        .rx_bandwidth = FSK_RXBW_156_2_KHZ,
        .tx_power = 22,
        .preamble_len = 16,
        .fsk_sync_word = { 0x12, 0xAD },
        .sync_word_len = 2,
        .fixed_length = false,
        .payload_len = 0,
        .crc_type = FSK_CRC_2_BYTE,
        .whitening = true,
        .config_pending = false
    };

    // Cachear los valores por defecto de FSK sin enviarlos al chip: el chip
    // arranca en modo LoRa (ver ApplyConfig de abajo) y solo cambia a GFSK
    // cuando el usuario llama explícitamente a SX1262_FSK_ApplyConfig().
    SX1262_FSK_CurrentConfig = default_fsk_config;

    // Habilitar marca para permitir comandos internos
    SX1262_Initialized = 1;
    st = SX1262_LoRa_ApplyConfig(&default_lora_config);
    if (st != SX1262_OK)
    {
        SX1262_Initialized = 0;
        return st;
    }

    // Configurar DIO2 como RF Switch (si el layout usa el sw de semtech)
    buf[0] = 0x01; // enable
    st = sx1262_WriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, buf, 1);
    if (st != SX1262_OK)
    {
        SX1262_Initialized = 0;
        return st;
    }

    return SX1262_OK;
}

/**
 * @brief Transmite datos en modo LoRa (bloqueante). Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_Transmit(uint8_t *data, uint8_t length)
{
    // La versión bloqueante es la versión IT con una espera activa de DIO1
    // intercalada: StartTransmitIT valida los parámetros y arma la TX,
    // GetTransmitStatus evalúa el registro IRQ y libera el semáforo.
    SX1262_Status_t st = SX1262_LoRa_StartTransmitIT(data, length);
    if (st != SX1262_OK)
    {
        return st;
    }

    // Timeout de software basado en el ToA real del paquete + 50 % de margen.
    // Esto evita el hardcode de 5 segundos y adapta el timeout a los parámetros
    // LoRa.
    uint32_t toa_ms = sx1262_ComputeToA_ms(length, &SX1262_LoRa_CurrentConfig);
    uint32_t timeout_ms = toa_ms + toa_ms / 2U + 100U;

    // Esperar IRQ (DIO1 en alto => TxDone o TxTimeout)
    if (sx1262_WaitDio1(timeout_ms) != SX1262_OK)
    {
        // Timeout de software: DIO1 nunca subió, así que GetTransmitStatus
        // no llegará a consumir el evento ni a liberar el semáforo. Abort
        // devuelve el chip a Standby, limpia los IRQ y libera SX1262_TxActive
        // (best-effort: el evento real es un timeout y se reporta como tal
        // aunque la limpieza falle).
        sx1262_AbortTransmit();
        return SX1262_TIMEOUT;
    }

    return sx1262_GetTransmitStatus();
}

// ============================================================================
// TRANSMISIÓN NO BLOQUEANTE (IT) — LoRa
// ============================================================================

/**
 * @brief Inicia una transmisión LoRa y retorna inmediatamente. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_StartTransmitIT(uint8_t *data, uint8_t length)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    if (SX1262_LoRa_CurrentConfig.config_pending)
    {
        return SX1262_ERROR; // Llamar a SX1262_LoRa_ApplyConfig() antes de transmitir
    }

    if (SX1262_TxActive)
    {
        return SX1262_TX_BUSY; // Ya hay una transmisión IT en curso
    }

    if (data == NULL || length == 0)
    {
        return SX1262_INVALID_PARAM;
    }

    // Parámetros de paquete LoRa con la longitud real del payload
    uint8_t pkt[6];
    pkt[0] = (SX1262_LoRa_CurrentConfig.preamble_len >> 8) & 0xFF; // Preamble MSB
    pkt[1] = (SX1262_LoRa_CurrentConfig.preamble_len) & 0xFF;      // Preamble LSB
    pkt[2] = 0x00;                                                 // Explicit Header
    pkt[3] = length;                                               // PayloadLength real
    pkt[4] = 0x01;                                                 // CRC On
    pkt[5] = SX1262_LoRa_CurrentConfig.iq_inverted ? 0x01 : 0x00;  // Invert IQ

    // Armado de TX y retorno inmediato — el CPU queda libre
    return sx1262_ArmTx(data, length, pkt, 6);
}

/**
 * @brief Verifica y consume el evento TX_DONE. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_GetTransmitStatus(void)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    return sx1262_GetTransmitStatus();
}

/**
 * @brief Cancela la transmisión LoRa en curso y vuelve a Standby RC. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_AbortTransmit(void)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    return sx1262_AbortTransmit();
}

/**
 * @brief Recibe datos en modo LoRa (bloqueante). Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_Receive(uint8_t *data, uint8_t max_length, uint8_t *length, uint32_t timeout_ms)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    if (data == NULL || length == NULL || max_length == 0)
    {
        return SX1262_INVALID_PARAM;
    }

    // Verificar si hay una configuración pendiente sin aplicar.
    // Recibir con parámetros obsoletos (frecuencia, BW, SF, etc.) puede causar
    // que el chip nunca detecte un paquete válido.
    if (SX1262_LoRa_CurrentConfig.config_pending)
    {
        return SX1262_ERROR; // Llamar a SX1262_LoRa_ApplyConfig() antes de recibir
    }

    // Pasamos de ms a ticks (timeout interno del chip). 1 tick = 15.625 us,
    // 1 ms = 64 ticks (1000 / 15.625 = 64). Evitamos cálculo en punto flotante
    // usando * 64. timeout_ms == 0 => 0xFFFFFF: sin timeout de chip.
    uint32_t chipTimeout = (timeout_ms == 0) ? 0xFFFFFF : (timeout_ms * 64);

    // Abortar si algún comando de la secuencia de armado de RX falló
    SX1262_Status_t st = sx1262_ArmRx(chipTimeout);
    if (st != SX1262_OK)
    {
        return st;
    }

    // Bloquear hasta interrupción
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(DIO_GPIO_Port, DIO_GPIO_Pin) == GPIO_PIN_RESET)
    {
        if (timeout_ms != 0 && (HAL_GetTick() - start) > (timeout_ms + 100))
        {
            // Timeout de software: volver a Standby (best-effort) y reportar timeout
            sx1262_Standby();
            return SX1262_TIMEOUT; // Timeout de soft-check
        }
    }

    // DIO1 en alto: evaluar el registro IRQ, leer el payload y limpiar. Es
    // exactamente el trabajo de la ruta IT, sin estado propio de la RX continua.
    return SX1262_LoRa_GetReceivedPacket(data, max_length, length);
}

/**
 * @brief Arma el modo RX LoRa continuo y retorna inmediatamente. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_StartReceiveIT(void)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    if (SX1262_LoRa_CurrentConfig.config_pending)
    {
        return SX1262_ERROR; // Llamar a SX1262_LoRa_ApplyConfig() antes de recibir
    }

    if (SX1262_TxActive)
    {
        // Hay una TX IT en vuelo. Armar RX ahora forzaría Standby sobre la TX y
        // el dispatcher del ISR (SX1262_IRQ_Handler) despacharía mal el flanco DIO1.
        return SX1262_TX_BUSY;
    }

    if (SX1262_RxActive)
    {
        return SX1262_RX_BUSY; // Ya hay una recepción IT (RX continuo) en curso
    }

    // RX continuo: timeout de chip 0xFFFFFF => el chip nunca abandona la escucha
    SX1262_Status_t st = sx1262_ArmRx(0xFFFFFF);

    // Marcar RX activa solo si toda la secuencia de configuración tuvo éxito.
    // RX es continuo (timeout 0xFFFFFF): la bandera se mantiene tras cada RxDone
    // y solo se libera en AbortReceive o al iniciar una TX (StartTransmitIT).
    if (st == SX1262_OK)
    {
        SX1262_RxActive = 1;
    }

    // Retorno inmediato — sin polling en DIO1
    return st;
}

/**
 * @brief Lee el payload del paquete LoRa recibido. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_GetReceivedPacket(uint8_t *data, uint8_t max_length, uint8_t *length)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }
    if (data == NULL || length == NULL || max_length == 0)
    {
        return SX1262_INVALID_PARAM;
    }

    return sx1262_GetReceivedPacket(data, max_length, length);
}

/**
 * @brief Cancela la recepción LoRa en curso y vuelve a Standby RC. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_AbortReceive(void)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    return sx1262_AbortReceive();
}

/**
 * @brief Ejecuta una detección de actividad en el canal (CAD). Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_ChannelActivityDetection(bool *activity_detected)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    if (activity_detected == NULL)
    {
        return SX1262_INVALID_PARAM;
    }

    // El CAD depende del SF y el BW activos para su sensibilidad y su duración:
    // con configuración sin aplicar, el resultado no describiría el canal real.
    if (SX1262_LoRa_CurrentConfig.config_pending)
    {
        return SX1262_ERROR; // Llamar a SX1262_LoRa_ApplyConfig() antes del CAD
    }

    if (SX1262_TxActive)
    {
        // Hay una TX IT en vuelo. Lanzar un CAD ahora forzaría Standby sobre la TX
        // y el dispatcher del ISR despacharía mal el flanco DIO1.
        return SX1262_TX_BUSY;
    }

    *activity_detected = false;

    uint8_t buf[8];
    SX1262_Status_t st = SX1262_OK;

    // 1. Standby RC (el chip debe estar en Standby antes de configurar el CAD).
    //    Igual que en la ruta TX, el Standby cancela cualquier RX continuo previo:
    //    liberar el semáforo para no dejar estado obsoleto.
    st = st ? st : sx1262_Standby();
    SX1262_RxActive = 0;

    // 2. Limpiar IRQ pendientes
    st = st ? st : sx1262_ClearIrq();

    // 3. Parámetros del CAD (datasheet §13.4.7)
    buf[0] = 0x01;                                                            // cadSymbolNum = CAD_ON_2_SYMB
    buf[1] = sx1262_CadDetPeak(SX1262_LoRa_CurrentConfig.spreading_factor);   // cadDetPeak
    buf[2] = 10;                                                              // cadDetMin (valor de AN1200.48, común a todos los SF)
    buf[3] = 0x00;                                                            // cadExitMode = CAD_ONLY: al terminar vuelve a Standby RC
    buf[4] = 0x00;                                                            // cadTimeout(23:0): solo se usa con CAD_RX
    buf[5] = 0x00;
    buf[6] = 0x00;
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_CAD_PARAMS, buf, 7);

    // 4. Enrutar CAD_DONE y CAD_DETECTED a DIO1
    uint16_t irqMask = SX126X_IRQ_CAD_DONE | SX126X_IRQ_CAD_DETECTED;
    buf[0] = (irqMask >> 8) & 0xFF;
    buf[1] = irqMask & 0xFF; // IRQ global mask
    buf[2] = (irqMask >> 8) & 0xFF;
    buf[3] = irqMask & 0xFF; // DIO1 mask
    buf[4] = 0x00;
    buf[5] = 0x00; // DIO2 (no usado)
    buf[6] = 0x00;
    buf[7] = 0x00; // DIO3 (no usado)
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8);

    // 5. Lanzar el CAD (comando sin argumentos)
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_CAD, NULL, 0);

    if (st != SX1262_OK)
    {
        return st;
    }

    // 6. Timeout de software derivado de la modulación activa: el CAD dura poco
    //    más de los 2 símbolos analizados, así que 8 símbolos más un margen fijo
    //    cubren cualquier combinación SF/BW sin colgar el bucle si DIO1 no sube.
    uint32_t bw_hz = sx1262_BandwidthToHz(SX1262_LoRa_CurrentConfig.bandwidth);
    uint32_t timeout_ms = 100U; // Valor de seguridad si el BW es inválido
    if (bw_hz != 0)
    {
        // T_sym_us = (1 << SF) * 1_000_000 / BW_Hz  (64 bits: SF12/BW7.8k desborda 32)
        uint64_t t_sym_us = ((uint64_t)1 << SX1262_LoRa_CurrentConfig.spreading_factor) * 1000000ULL / (uint64_t)bw_hz;
        timeout_ms = (uint32_t)((t_sym_us * 8ULL) / 1000ULL) + 10U;
    }

    if (sx1262_WaitDio1(timeout_ms) != SX1262_OK)
    {
        // Sacar el chip del modo CAD antes de reportar (best-effort)
        sx1262_Standby();
        sx1262_ClearIrq();
        return SX1262_TIMEOUT;
    }

    // 7. Leer el registro IRQ y limpiarlo
    uint8_t irqStatus[2];
    st = sx1262_ReadCommand(SX126X_CMD_GET_IRQ_STATUS, irqStatus, 2);
    if (st != SX1262_OK)
    {
        return st;
    }

    uint16_t irqReg = ((uint16_t)irqStatus[0] << 8) | irqStatus[1];
    SX1262_LastIrqStatus = irqReg; // Cachear para diagnóstico (SX1262_GetLastIrqStatus)

    sx1262_ClearIrq();

    // El flanco de DIO1 también despertó al ISR, que al no haber TX en curso marcó
    // SX1262_LoRa_RxDoneFlag. El evento ya se consumió aquí: limpiarla para que el
    // main loop no procese una recepción que nunca ocurrió.
    SX1262_LoRa_RxDoneFlag = 0;

    if ((irqReg & SX126X_IRQ_CAD_DONE) == 0)
    {
        return SX1262_ERROR; // DIO1 subió sin CAD_DONE: condición inesperada
    }

    // Tras CAD_ONLY el chip ya está de vuelta en Standby RC por sí mismo
    *activity_detected = (irqReg & SX126X_IRQ_CAD_DETECTED) != 0;

    return SX1262_OK;
}

/**
 * @brief Dispatcher TX / RX del IRQ. Contrato en SX1262.h.
 *
 *        Corre en contexto ISR: sin SPI, sin HAL bloqueante, sin printf.
 */
void SX1262_IRQ_Handler(void)
{
    if (SX1262_TxActive)
    {
        SX1262_LoRa_TxDoneFlag = 1; // Evento TX: señal a SX1262_LoRa_GetTransmitStatus()
    }
    else
    {
        SX1262_LoRa_RxDoneFlag = 1; // Evento RX: señal a SX1262_LoRa_GetReceivedPacket()
    }
}

/**
 * @brief Aplica la configuración de red y modulación LoRa al chip. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_ApplyConfig(const lora_config_t *config)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    if (config == NULL)
    {
        return SX1262_INVALID_PARAM;
    }

    uint8_t buf[8];
    SX1262_Status_t st = SX1262_OK;

    // Modo Standby RC necesario para configurar
    st = st ? st : sx1262_Standby();

    // --- 1. FRECUENCIA ---
    uint32_t frf = (uint32_t)(((uint64_t)config->frequency * 16384ULL) / 15625ULL);
    buf[0] = (frf >> 24) & 0xFF;
    buf[1] = (frf >> 16) & 0xFF;
    buf[2] = (frf >> 8) & 0xFF;
    buf[3] = (frf & 0xFF);
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4);

    // --- 2. POTENCIA TX ---
    // Configuración PA por defecto para transceptores SX1262 (+22dBm Max)
    buf[0] = 0x04;
    buf[1] = 0x07;
    buf[2] = 0x00;
    buf[3] = 0x01;
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4);

    buf[0] = config->tx_power; // power
    buf[1] = 0x02;             // rampTime 40us
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2);

    // --- 3. MODULACIÓN ---
    // Calcular LDRO dinámicamente: obligatorio cuando T_símbolo >= 16.38 ms
    // Combinaciones que lo activan: SF11/BW62.5, SF12/BW62.5, SF12/BW125, y
    // anchos menores con SF altos.
    uint8_t ldro = sx1262_ComputeLDRO(config->spreading_factor, config->bandwidth);

    buf[0] = config->spreading_factor;
    buf[1] = config->bandwidth;
    buf[2] = config->coding_rate;
    buf[3] = ldro; // LowDataRateOptimize: calculado automáticamente
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, buf, 4);

    // --- 4. SYNC WORD ---
    // El campo lora_sync_word (distinto de 0) tiene prioridad absoluta sobre
    // network_mode. En caso contrario, se selecciona según el modo de red
    // configurado. Los registros 0x0740–0x0741 almacenan dos nibbles del sync
    // word según la nota de aplicación de Semtech: reg[0x0740] = (SW & 0xF0) |
    // 0x04,
    //                                reg[0x0741] = (SW << 4)   | 0x04
    uint8_t sync_msb, sync_lsb;
    if (config->lora_sync_word != 0)
    {
        // Sync word personalizado: aplicar fórmula de nibbles de Semtech
        sync_msb = (config->lora_sync_word & 0xF0) | 0x04;
        sync_lsb = (config->lora_sync_word << 4) | 0x04;
    }
    else
    {
        switch (config->network_mode)
        {
            case LORA_NETWORK_PUBLIC:
                sync_msb = 0x34;
                sync_lsb = 0x44; // SW lógico 0x34 (LoRaWAN)
                break;
            case LORA_NETWORK_MESHTASTIC:
                sync_msb = 0x24;
                sync_lsb = 0xB4; // SW lógico 0x2B (Meshtastic): (0x2B & 0xF0)|0x04=0x24, ((0x2B&0x0F)<<4)|0x04=0xB4
                break;
            case LORA_NETWORK_PRIVATE:
            default:
                sync_msb = 0x14;
                sync_lsb = 0x24; // SW lógico 0x12 (LoRa privado)
                break;
        }
    }
    buf[0] = 0x07; // Dirección de registro MSB
    buf[1] = 0x40; // Dirección de registro MSB
    buf[2] = sync_msb;
    buf[3] = sync_lsb;
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_WRITE_REGISTER, buf, 4);

    // --- 5. PARÁMETROS DEL PAQUETE ---
    buf[0] = (config->preamble_len >> 8) & 0xFF; // Preamble MSB
    buf[1] = config->preamble_len & 0xFF;        // Preamble LSB
    buf[2] = 0x00;                               // Explicit Header
    buf[3] = 0xFF;                               // PayloadLength (Dummy)
    buf[4] = 0x01;                               // CRC On
    buf[5] = config->iq_inverted ? 0x01 : 0x00;  // Invert IQ
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, buf, 6);

    // Solo persistir la configuración si toda la secuencia se aplicó con éxito
    if (st != SX1262_OK)
    {
        return st;
    }

    // Guardar estado actual
    SX1262_LoRa_CurrentConfig = *config;
    SX1262_LoRa_CurrentConfig.config_pending = false;

    return SX1262_OK;
}

/**
 * @brief Obtiene RSSI y SNR del último paquete LoRa recibido. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_GetPacketStatus(int16_t *rssi_dbm, int8_t *snr_db)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }
    if (rssi_dbm == NULL && snr_db == NULL)
    {
        return SX1262_INVALID_PARAM;
    }

    // GetPacketStatus devuelve 3 bytes en LoRa:
    //   [0] RssiPkt  → RSSI = -RssiPkt/2  (dBm)
    //   [1] SnrPkt   → SNR  =  SnrPkt/4   (dB, con signo)
    //   [2] SignalRssiPkt (no utilizado aquí)
    uint8_t status[3];
    SX1262_Status_t st = sx1262_ReadCommand(SX126X_CMD_GET_PACKET_STATUS, status, 3);
    if (st != SX1262_OK)
    {
        return st;
    }

    if (rssi_dbm != NULL)
    {
        // RssiPkt es un valor sin signo; el resultado en dBm es siempre <= 0
        *rssi_dbm = -(int16_t)status[0] / 2;
    }
    if (snr_db != NULL)
    {
        // SnrPkt está en el byte [1] como int8_t (complemento a dos).
        // SNR [dB] = (int8_t)SnrPkt / 4  →  resolución de 0.25 dB
        // Se devuelve redondeado a dB enteros para mayor simplicidad de uso.
        *snr_db = (int8_t)status[1] / 4;
    }

    return SX1262_OK;
}

/**
 * @brief Obtiene el RSSI del último paquete LoRa recibido. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_GetRSSI(int16_t *rssi_dbm)
{
    return SX1262_LoRa_GetPacketStatus(rssi_dbm, NULL);
}

/**
 * @brief Obtiene el SNR del último paquete LoRa recibido. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_GetSNR(int8_t *snr_db)
{
    return SX1262_LoRa_GetPacketStatus(NULL, snr_db);
}

/**
 * @brief Retorna una copia de la configuración LoRa aplicada. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_LoRa_GetConfig(lora_config_t *config)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }
    if (config == NULL)
    {
        return SX1262_INVALID_PARAM;
    }

    *config = SX1262_LoRa_CurrentConfig;

    return SX1262_OK;
}

// ============================================================================
// TRANSMISIÓN FSK — Bloqueante
// ============================================================================

/**
 * @brief Transmite datos en modo FSK (bloqueante). Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_FSK_Transmit(uint8_t *data, uint8_t length)
{
    // La versión bloqueante es la versión IT con una espera activa de DIO1
    // intercalada: StartTransmitIT valida los parámetros y arma la TX,
    // GetTransmitStatus evalúa el registro IRQ y libera el semáforo.
    SX1262_Status_t st = SX1262_FSK_StartTransmitIT(data, length);
    if (st != SX1262_OK)
    {
        return st;
    }

    // Timeout de software basado en el ToA real del paquete + 50 % de margen.
    uint32_t toa_ms = sx1262_ComputeFskToA_ms(length, &SX1262_FSK_CurrentConfig);
    uint32_t timeout_ms = toa_ms + toa_ms / 2U + 100U;

    // Esperar IRQ (DIO1 en alto => TxDone o TxTimeout)
    if (sx1262_WaitDio1(timeout_ms) != SX1262_OK)
    {
        // Timeout de software: DIO1 nunca subió, así que GetTransmitStatus no
        // llegará a consumir el evento ni a liberar el semáforo. Abort devuelve
        // el chip a Standby, limpia los IRQ y libera SX1262_TxActive
        // (best-effort: el evento real es un timeout y se reporta como tal
        // aunque la limpieza falle).
        sx1262_AbortTransmit();
        return SX1262_TIMEOUT;
    }

    return sx1262_GetTransmitStatus();
}

// ============================================================================
// TRANSMISIÓN NO BLOQUEANTE (IT) — FSK
// ============================================================================

/**
 * @brief Inicia una transmisión FSK y retorna inmediatamente. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_FSK_StartTransmitIT(uint8_t *data, uint8_t length)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    // Verificar si hay una configuración FSK pendiente sin aplicar.
    if (SX1262_FSK_CurrentConfig.config_pending)
    {
        return SX1262_ERROR; // Llamar a SX1262_FSK_ApplyConfig() antes de transmitir
    }

    if (SX1262_TxActive)
    {
        return SX1262_TX_BUSY; // Ya hay una transmisión IT en curso
    }

    if (data == NULL || length == 0)
    {
        return SX1262_INVALID_PARAM;
    }

    // Parámetros de paquete GFSK con la longitud real del payload.
    // PreambleLength y SyncWordLength se expresan en bits en el comando GFSK.
    uint16_t preamble_bits = (uint16_t)SX1262_FSK_CurrentConfig.preamble_len * 8U;
    uint8_t pkt[9];
    pkt[0] = (preamble_bits >> 8) & 0xFF;
    pkt[1] = preamble_bits & 0xFF;
    pkt[2] = 0x04; // PreambleDetectorLength: 8 bits
    pkt[3] = SX1262_FSK_CurrentConfig.sync_word_len * 8U; // SyncWordLength en bits
    pkt[4] = 0x00; // AddrComp: off
    pkt[5] = SX1262_FSK_CurrentConfig.fixed_length ? 0x00 : 0x01; // HeaderType: fijo/variable
    pkt[6] = length;
    pkt[7] = SX1262_FSK_CurrentConfig.crc_type;
    pkt[8] = SX1262_FSK_CurrentConfig.whitening ? 0x01 : 0x00;

    // Armado de TX y retorno inmediato — el CPU queda libre
    return sx1262_ArmTx(data, length, pkt, 9);
}

/**
 * @brief Verifica y consume el evento TX_DONE de una TX FSK. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_FSK_GetTransmitStatus(void)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    return sx1262_GetTransmitStatus();
}

/**
 * @brief Cancela la transmisión FSK en curso y vuelve a Standby RC. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_FSK_AbortTransmit(void)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    return sx1262_AbortTransmit();
}

// ============================================================================
// RECEPCIÓN FSK — Bloqueante
// ============================================================================

/**
 * @brief Recibe datos en modo FSK (bloqueante). Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_FSK_Receive(uint8_t *data, uint8_t max_length, uint8_t *length, uint32_t timeout_ms)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    if (data == NULL || length == NULL || max_length == 0)
    {
        return SX1262_INVALID_PARAM;
    }

    // Verificar si hay una configuración FSK pendiente sin aplicar.
    // Recibir con parámetros obsoletos (frecuencia, bitrate, sync word, etc.)
    // puede causar que el chip nunca detecte un paquete válido.
    if (SX1262_FSK_CurrentConfig.config_pending)
    {
        return SX1262_ERROR; // Llamar a SX1262_FSK_ApplyConfig() antes de recibir
    }

    // Pasamos de ms a ticks (timeout interno del chip). 1 tick = 15.625 us,
    // 1 ms = 64 ticks (1000 / 15.625 = 64). Evitamos cálculo en punto flotante
    // usando * 64. timeout_ms == 0 => 0xFFFFFF: sin timeout de chip.
    uint32_t chipTimeout = (timeout_ms == 0) ? 0xFFFFFF : (timeout_ms * 64);

    // Abortar si algún comando de la secuencia de armado de RX falló. sx1262_ArmRx
    // es agnóstico al modo de modulación: la secuencia Standby/ClearIrq/SetDioIrq/
    // SetRx es idéntica en LoRa y GFSK (el chip ya está en GFSK tras ApplyConfig).
    SX1262_Status_t st = sx1262_ArmRx(chipTimeout);
    if (st != SX1262_OK)
    {
        return st;
    }

    // Bloquear hasta interrupción
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(DIO_GPIO_Port, DIO_GPIO_Pin) == GPIO_PIN_RESET)
    {
        if (timeout_ms != 0 && (HAL_GetTick() - start) > (timeout_ms + 100))
        {
            // Timeout de software: volver a Standby (best-effort) y reportar timeout
            sx1262_Standby();
            return SX1262_TIMEOUT; // Timeout de soft-check
        }
    }

    // DIO1 en alto: evaluar el registro IRQ, leer el payload y limpiar.
    return sx1262_GetReceivedPacket(data, max_length, length);
}

// ----------------------------------------------------------------------------
// Recepción FSK — No Bloqueante (basada en interrupciones EXTI en DIO1)
// ----------------------------------------------------------------------------

/**
 * @brief Arma el modo RX FSK continuo y retorna inmediatamente. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_FSK_StartReceiveIT(void)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    if (SX1262_FSK_CurrentConfig.config_pending)
    {
        return SX1262_ERROR; // Llamar a SX1262_FSK_ApplyConfig() antes de recibir
    }

    if (SX1262_TxActive)
    {
        // Hay una TX IT en vuelo. Armar RX ahora forzaría Standby sobre la TX y
        // el dispatcher del ISR (SX1262_IRQ_Handler) despacharía mal el flanco DIO1.
        return SX1262_TX_BUSY;
    }

    if (SX1262_RxActive)
    {
        return SX1262_RX_BUSY; // Ya hay una recepción IT (RX continuo) en curso
    }

    // RX continuo: timeout de chip 0xFFFFFF => el chip nunca abandona la escucha.
    // sx1262_ArmRx es agnóstico al modo de modulación: la secuencia Standby/
    // ClearIrq/SetDioIrq/SetRx es idéntica en LoRa y GFSK (el chip ya está en
    // GFSK tras ApplyConfig).
    SX1262_Status_t st = sx1262_ArmRx(0xFFFFFF);

    // Marcar RX activa solo si toda la secuencia de configuración tuvo éxito.
    // RX es continuo (timeout 0xFFFFFF): la bandera se mantiene tras cada RxDone
    // y solo se libera en AbortReceive o al iniciar una TX (StartTransmitIT).
    if (st == SX1262_OK)
    {
        SX1262_RxActive = 1;
    }

    // Retorno inmediato — sin polling en DIO1
    return st;
}

/**
 * @brief Lee el payload del paquete FSK recibido. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_FSK_GetReceivedPacket(uint8_t *data, uint8_t max_length, uint8_t *length)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }
    if (data == NULL || length == NULL || max_length == 0)
    {
        return SX1262_INVALID_PARAM;
    }

    return sx1262_GetReceivedPacket(data, max_length, length);
}

/**
 * @brief Cancela la recepción FSK en curso y vuelve a Standby RC. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_FSK_AbortReceive(void)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    return sx1262_AbortReceive();
}

// ============================================================================
// CONFIGURACIÓN FSK/GFSK
// ============================================================================

/**
 * @brief Aplica la configuración de modulación FSK/GFSK al chip. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_FSK_ApplyConfig(fsk_config_t *config)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    if (config == NULL)
    {
        return SX1262_INVALID_PARAM;
    }

    if (config->bitrate == 0 || config->sync_word_len > 8)
    {
        return SX1262_INVALID_PARAM;
    }

    uint8_t buf[16];
    SX1262_Status_t st = SX1262_OK;

    // Modo Standby RC necesario para configurar
    st = st ? st : sx1262_Standby();

    // --- 1. TIPO DE PAQUETE: GFSK ---
    buf[0] = SX126X_PACKET_TYPE_GFSK;
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_PACKET_TYPE, buf, 1);

    // --- 2. FRECUENCIA ---
    uint32_t frf = (uint32_t)(((uint64_t)config->frequency * 16384ULL) / 15625ULL);
    buf[0] = (frf >> 24) & 0xFF;
    buf[1] = (frf >> 16) & 0xFF;
    buf[2] = (frf >> 8) & 0xFF;
    buf[3] = (frf & 0xFF);
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4);

    // --- 3. POTENCIA TX ---
    // Configuración PA por defecto para transceptores SX1262 (+22dBm Max)
    buf[0] = 0x04;
    buf[1] = 0x07;
    buf[2] = 0x00;
    buf[3] = 0x01;
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4);

    buf[0] = config->tx_power; // power
    buf[1] = 0x02;              // rampTime 40us
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2);

    // --- 4. MODULACIÓN GFSK ---
    // Fórmulas de conversión (datasheet SX1262 §13.4.5, Fxtal = 32 MHz):
    //   BitRate reg = 32 * Fxtal / bitrate(bps)
    //   Fdev    reg = Fdev(Hz) * 2^25 / Fxtal
    uint32_t br_reg = (uint32_t)(((uint64_t)32U * 32000000ULL) / config->bitrate);

    uint32_t fdev_hz = (uint32_t)(config->freq_dev * 1000.0f);
    uint32_t fdev_reg = (uint32_t)(((uint64_t)fdev_hz * 33554432ULL) / 32000000ULL);

    buf[0] = (br_reg >> 16) & 0xFF;
    buf[1] = (br_reg >> 8) & 0xFF;
    buf[2] = br_reg & 0xFF;
    buf[3] = config->shaping;
    buf[4] = config->rx_bandwidth;
    buf[5] = (fdev_reg >> 16) & 0xFF;
    buf[6] = (fdev_reg >> 8) & 0xFF;
    buf[7] = fdev_reg & 0xFF;
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, buf, 8);

    // --- 5. SYNC WORD ---
    // Escribe sync_word_len bytes en el registro base 0x06C0 (WriteRegister
    // toma dirección de 2 bytes seguida de los datos).
    buf[0] = (SX126X_REG_SYNC_WORD_BASE >> 8) & 0xFF;
    buf[1] = SX126X_REG_SYNC_WORD_BASE & 0xFF;
    for (uint8_t i = 0; i < config->sync_word_len; i++)
    {
        buf[2 + i] = config->fsk_sync_word[i];
    }
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_WRITE_REGISTER, buf, 2U + config->sync_word_len);

    // --- 6. PARÁMETROS DEL PAQUETE ---
    // PreambleLength y SyncWordLength se expresan en bits en el comando GFSK
    // (los campos de la struct están en bytes).
    uint16_t preamble_bits = (uint16_t)config->preamble_len * 8U;
    buf[0] = (preamble_bits >> 8) & 0xFF;
    buf[1] = preamble_bits & 0xFF;
    buf[2] = 0x04; // PreambleDetectorLength: 8 bits
    buf[3] = config->sync_word_len * 8U; // SyncWordLength en bits
    buf[4] = 0x00; // AddrComp: off
    buf[5] = config->fixed_length ? 0x00 : 0x01; // HeaderType: fijo/variable
    // PayloadLength tiene doble significado (datasheet SX1262 §13.4.6): tamaño
    // exacto del paquete en longitud fija, y tamaño MÁXIMO que el receptor
    // acepta en longitud variable. Enviar 0 en modo variable haría que el chip
    // descartara todo paquete entrante. La ruta de TX sobrescribe este campo con
    // la longitud real del payload (ver SX1262_FSK_StartTransmitIT).
    buf[6] = config->fixed_length ? config->payload_len : 255U;
    buf[7] = config->crc_type;
    buf[8] = config->whitening ? 0x01 : 0x00;
    st = st ? st : sx1262_WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, buf, 9);

    // Solo persistir la configuración si toda la secuencia se aplicó con éxito
    if (st != SX1262_OK)
    {
        return st;
    }

    // Guardar estado actual
    SX1262_FSK_CurrentConfig = *config;
    SX1262_FSK_CurrentConfig.config_pending = false;

    return SX1262_OK;
}

/**
 * @brief Retorna una copia de la configuración FSK aplicada. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_FSK_GetConfig(fsk_config_t *config)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }
    if (config == NULL)
    {
        return SX1262_INVALID_PARAM;
    }

    *config = SX1262_FSK_CurrentConfig;

    return SX1262_OK;
}

/**
 * @brief Retorna el último registro IRQ leído en una RX. Contrato en SX1262.h.
 */
uint16_t SX1262_GetLastIrqStatus(void)
{
    return SX1262_LastIrqStatus;
}

/**
 * @brief Pone el módulo en modo Sleep. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_SetSleep(uint8_t sleep_config)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    // El comando SetSleep toma un byte de configuración.
    // SX126X_CMD_SET_SLEEP (0x84) + sleep_config
    SX1262_Status_t st = sx1262_WriteCommand(SX126X_CMD_SET_SLEEP, &sleep_config, 1);
    if (st != SX1262_OK)
    {
        return st;
    }

    // NOTA: Tras entrar en Sleep, el chip no responde a comandos SPI.
    // Para volver a operar, se debe llamar a SX1262_Wakeup().

    return SX1262_OK;
}

/**
 * @brief Despierta el chip desde el modo Sleep. Contrato en SX1262.h.
 */
SX1262_Status_t SX1262_Wakeup(void)
{
    // Generar flanco de bajada y subida en NSS para despertar al chip
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(1); // Pequeño retardo
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    // Esperar a que el chip esté listo
    return sx1262_WaitBusy();
}
