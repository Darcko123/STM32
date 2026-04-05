/**
 * @file SX1262.c
 * @brief Librería para la gestión de un módulo LoRa
 * 
 * Esta librería permite inicializar y controlar
 * 
 * @author Daniel Ruiz
 * @date April 3, 2026
 * @version 1.2.0
 */

#include "SX1262.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static SPI_HandleTypeDef*   SX1262_hspi         = NULL;         /**< Manejador de la interfaz SPI utilizada para cominucarse con el módulo*/
static GPIO_TypeDef*        NSS_GPIO_Port       = NULL;         /**< Puerto GPIO del pin de datos del SX1262 */
static uint16_t             NSS_GPIO_Pin        = 0;            /**< Pin GPIO del pin de datos del SX1262 */
static GPIO_TypeDef*        BUSY_GPIO_Port      = NULL;
static uint16_t             BUSY_GPIO_Pin       = 0;
static GPIO_TypeDef*        DIO_GPIO_Port      = NULL;
static uint16_t             DIO_GPIO_Pin       = 0;
static GPIO_TypeDef*        RST_GPIO_Port       = NULL;
static uint16_t             RST_GPIO_Pin        = 0;
static uint8_t              SX1262_Initialized  = 0;            /**< Bandera para verificar si el módulo está inicializado */
static lora_config_t        SX1262_CurrentConfig;               /**< Configuración actual aplicada al módulo */

/**
 * @brief Bandera de recepción no bloqueante (productor: ISR, consumidor: main loop).
 *        SX1262_IRQ_Handler() la activa a 1 cuando DIO1 sube (EXTI).
 *        El main loop la lee, la pone a 0 y llama SX1262_GetReceivedPacket().
 */
volatile uint8_t SX1262_RxDoneFlag = 0;

// ============================================================================
// FUNCIONES PRIVADAS - ABSTRACCIÓN SPI
// ============================================================================
/**
 * @brief Espera a que el pin BUSY del SX1262 se libere, indicando que el módulo está listo para la siguiente operación.
 * 
 * @return SX1262_Status_t 
 */
static SX1262_Status_t SX1262_WaitBusy(void)
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
 * @brief Escribe un comando al SX1262 a través de SPI, manejando la señal NSS y el pin BUSY.
 * 
 * @param cmd Comando a enviar
 * @param buffer Puntero a los datos asociados al comando (puede ser NULL si no se requieren datos)
 * @param size Tamaño de los datos a enviar (0 si no se requieren datos)
 * @return SX1262_Status_t 
 */
static SX1262_Status_t SX1262_WriteCommand(uint8_t cmd, uint8_t* buffer, uint16_t size)
{
    if (SX1262_WaitBusy() != SX1262_OK) return SX1262_TIMEOUT;

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(SX1262_hspi, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }
    if (size > 0 && buffer != NULL)
    {
        if(HAL_SPI_Transmit(SX1262_hspi, buffer, size, HAL_MAX_DELAY) != HAL_OK)
        {
            HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
            return SX1262_ERROR;
        }
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    return SX1262_OK;
}

/**
 * @brief Lee datos del SX1262 a través de SPI, manejando la señal NSS y el pin BUSY.
 * 
 * @param cmd Comando de lectura a enviar
 * @param buffer Puntero al buffer donde se almacenarán los datos leídos
 * @param size Tamaño de los datos a leer
 * @return SX1262_Status_t 
 */
static SX1262_Status_t SX1262_ReadCommand(uint8_t cmd, uint8_t* buffer, uint16_t size)
{
    if (SX1262_WaitBusy() != SX1262_OK) return SX1262_TIMEOUT;

    uint8_t nop = 0x00;
    uint8_t dump; // Variable "basurero" para volcar el RX FIFO y prevenir el flag OVR

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);

    // TransmitReceive sincroniza el bus limpiando datos entrantes
    if(HAL_SPI_TransmitReceive(SX1262_hspi, &cmd, &dump, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }
    if(HAL_SPI_TransmitReceive(SX1262_hspi, &nop, &dump, 1, HAL_MAX_DELAY) != HAL_OK) // El SX1262 retorna un STATUS primero
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    } 

    if (size > 0 && buffer != NULL)
    {
        // En este paso, el buffer del STM32 está limpio. El RX de datos útiles es seguro.
        if(HAL_SPI_Receive(SX1262_hspi, buffer, size, HAL_MAX_DELAY) != HAL_OK)
        {
            HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
            return SX1262_ERROR;
        }
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    return SX1262_OK;
}

/**
 * @brief Escribe datos en el buffer interno del SX1262 a través de SPI, manejando la señal NSS y el pin BUSY.
 * 
 * @param offset Offset dentro del buffer interno del SX1262 donde se escribirán los datos
 * @param data Puntero al buffer de datos a escribir
 * @param length Longitud de los datos a escribir (máximo 255 bytes) 
 * @return SX1262_Status_t 
 */
static SX1262_Status_t SX1262_WriteBuffer(uint8_t offset, uint8_t* data, uint8_t length)
{
    if (SX1262_WaitBusy() != SX1262_OK) return SX1262_TIMEOUT;

    uint8_t cmd[2] = {SX126X_CMD_WRITE_BUFFER, offset};
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(SX1262_hspi, cmd, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }
    if(HAL_SPI_Transmit(SX1262_hspi, data, length, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
    
    return SX1262_OK;
}

/**
 * @brief Lee datos del buffer interno del SX1262 a través de SPI, manejando la señal NSS y el pin BUSY.
 * 
 * @param offset Offset dentro del buffer interno del SX1262 desde donde se leerán los datos
 * @param data Puntero al buffer donde se almacenarán los datos leídos
 * @param length Longitud de los datos a leer (máximo 255 bytes)
 * @return SX1262_Status_t 
 */
static SX1262_Status_t SX1262_ReadBuffer(uint8_t offset, uint8_t* data, uint8_t length)
{
    if (SX1262_WaitBusy() != SX1262_OK) return SX1262_TIMEOUT;

    uint8_t cmd[3] = {SX126X_CMD_READ_BUFFER, offset, 0x00}; // NOP byte automatically handled here
    uint8_t dump[3]; // Buffer basura para capturar las respuestas ignoradas al transmitir

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);

    // Usamos TransmitReceive para evadir la condición de error Overrun
    if(HAL_SPI_TransmitReceive(SX1262_hspi, cmd, dump, 3, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }

    // El buffer RX está ahora limpio internamente, procedemos con lectura segura
    if(HAL_SPI_Receive(SX1262_hspi, data, length, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    return SX1262_OK;
}

/**
 * @brief Realiza un reset hardware del SX1262 utilizando el pin RST, siguiendo la secuencia recomendada por el datasheet.
 */
static void SX1262_Reset(void)
{
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_GPIO_Pin, GPIO_PIN_SET);
    HAL_Delay(10); // Permitir inicialización
}

/**
 * @brief Despierta el chip desde el modo Sleep, según el comportamiento de RadioLib 
 *        (Falling edge en NSS activa el chip seguido de WaitBusy).
 */
static SX1262_Status_t SX1262_Wakeup(void)
{
    // Generar flanco de bajada y subida en NSS para despertar al chip
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(1); // Pequeño retardo
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
    
    // Esperar a que el chip esté listo
    return SX1262_WaitBusy();
}

/**
 * @brief Convierte el valor de ancho de banda (BW) del enum a su equivalente en Hz, necesario para cálculos internos como LDRO.
 * 
 * @param bw Ancho de banda representado por el enum lora_signal_bandwidth_t
 * @return uint32_t 
 */
static uint32_t SX1262_BandwidthToHz(lora_signal_bandwidth_t bw)
{
    switch (bw)
    {
        case BW_7_8_KHZ:   return 7800UL;
        case BW_10_4_KHZ:  return 10400UL;
        case BW_15_6_KHZ:  return 15600UL;
        case BW_20_8_KHZ:  return 20800UL;
        case BW_31_25_KHZ: return 31250UL;
        case BW_41_7_KHZ:  return 41700UL;
        case BW_62_5_KHZ:  return 62500UL;
        case BW_125_KHZ:   return 125000UL;
        case BW_250_KHZ:   return 250000UL;
        case BW_500_KHZ:   return 500000UL;
        default:           return 0UL;
    }
}

/**
 * @brief Calcula si se debe activar LowDataRateOptimize (LDRO).
 *        El datasheet exige LDRO cuando el tiempo de símbolo >= 16.38 ms
 *        (datasheet SX1262, sección 6.1.1.4).
 *        Fórmula: T_sym_us = (2^SF * 1000000) / BW_Hz
 *        Se usa aritmética de 64 bits para evitar desbordamiento con SF12/BW bajo.
 *
 * @param sf   Spreading Factor (5–12)
 * @param bw   Bandwidth enum
 * @return     1 si LDRO debe activarse, 0 en caso contrario
 */
static uint8_t SX1262_ComputeLDRO(uint8_t sf, lora_signal_bandwidth_t bw)
{
    uint32_t bw_hz = SX1262_BandwidthToHz(bw);
    if (bw_hz == 0) return 0; // BW desconocido, no activar LDRO

    // T_sym_us = (1 << SF) * 1_000_000 / BW_Hz
    // Umbral: 16380 us  (≈ 16.38 ms, según datasheet)
    uint64_t t_sym_us = ((uint64_t)1 << sf) * 1000000ULL / (uint64_t)bw_hz;

    return (t_sym_us >= 16380) ? 1U : 0U;
}

/**
 * @brief Calcula el Time on Air (ToA) aproximado de un paquete LoRa en milisegundos.
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
static uint32_t SX1262_ComputeToA_ms(uint8_t payload_len, const lora_config_t *config)
{
    uint32_t bw_hz = SX1262_BandwidthToHz(config->bandwidth);
    if (bw_hz == 0) return 5000U; // Valor de seguridad si BW es inválido

    uint8_t sf   = config->spreading_factor;
    uint8_t cr   = (uint8_t)config->coding_rate; // 1=CR4/5 .. 4=CR4/8
    uint8_t ldro = SX1262_ComputeLDRO(sf, config->bandwidth);

    // T_sym en microsegundos: (2^SF * 1_000_000) / BW_Hz
    uint64_t t_sym_us = ((uint64_t)1 << sf) * 1000000ULL / (uint64_t)bw_hz;

    // Número de símbolos del payload (header explícito, CRC on)
    // Denominador: 4 * (SF - 2*LDRO)
    int32_t denom = 4 * ((int32_t)sf - 2 * (int32_t)ldro);
    if (denom <= 0) denom = 1; // Protección contra división por cero

    // Numerador: 8*payload - 4*SF + 28 + 16 (CRC on) - 0 (header explícito => ih=0)
    int32_t numer = 8 * (int32_t)payload_len - 4 * (int32_t)sf + 44;

    // ceil(numer / denom) usando división entera con redondeo hacia arriba
    int32_t ceil_val = (numer > 0) ? ((numer + denom - 1) / denom) : 0;
    int32_t n_sym_payload = 8 + ceil_val * ((int32_t)cr + 4);
    if (n_sym_payload < 8) n_sym_payload = 8;

    // Número total de símbolos: preamble + 4.25 (inicio) + payload
    // Multiplicamos por 4 para evitar fracciones: (preamble + payload + 4)*4 + 1  (el +1 es 0.25*4)
    uint64_t n_sym_total_x4 = ((uint64_t)config->preamble_len + (uint64_t)n_sym_payload + 4ULL) * 4ULL + 1ULL;

    // ToA en microsegundos: n_sym_total_x4 * t_sym_us / 4
    uint64_t toa_us = (n_sym_total_x4 * t_sym_us) / 4ULL;

    // Convertir a ms (redondeo hacia arriba) y garantizar mínimo 1 ms
    uint32_t toa_ms = (uint32_t)((toa_us + 999ULL) / 1000ULL);
    return (toa_ms < 1U) ? 1U : toa_ms;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el módulo SX1262 con la configuración de pines y parámetros por defecto.
 * 
 * @param hspi Puntero al handle del SPI utilizado para comunicarse con el SX1262.
 * @param nss_port Puerto GPIO del pin NSS (Chip Select).
 * @param nss_pin Número de pin GPIO para NSS.
 * @param busy_port Puerto GPIO del pin BUSY.
 * @param busy_pin Número de pin GPIO para BUSY.
 * @param dio_port Puerto GPIO del pin DIO1 (interrupción).
 * @param dio_pin Número de pin GPIO para DIO1.
 * @param rst_port Puerto GPIO del pin RST (Reset).
 * @param rst_pin Número de pin GPIO para RST.
 * @return SX1262_Status_t Estado de la inicialización (OK, ERROR, etc.)
 */
SX1262_Status_t SX1262_Init(
	SPI_HandleTypeDef* hspi,
    GPIO_TypeDef*      nss_port,
    uint16_t           nss_pin,
    GPIO_TypeDef*      busy_port,
    uint16_t           busy_pin,
    GPIO_TypeDef*      dio_port,
    uint16_t           dio_pin,
    GPIO_TypeDef*      rst_port,
    uint16_t           rst_pin
)
{
    // Validar parámetros de entrada
    if(hspi == NULL || nss_port  == NULL
                    || busy_port == NULL
                    || dio_port == NULL
                    || rst_port  == NULL
    )
    {
        return SX1262_ERROR;
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
    SX1262_Initialized = 0;    // Marcar como no inicializada hasta que se termine el proceso

    SX1262_Reset();
    uint8_t buf[8];

    // 0. Si se usara Sleep, idealmente usaríamos Wakeup. 
    // Llamamos Wakeup explícitamente para emular la robustez de RadioLib
    SX1262_Wakeup();

    // 1. Standby RC mode
    buf[0] = RADIOLIB_SX126X_STANDBY_RC;
    if(SX1262_WriteCommand(SX126X_CMD_SET_STANDBY, buf, 1) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // 2. Set Packet Type (LORA)
    buf[0] = RADIOLIB_SX126X_PACKET_TYPE_LORA;
    if(SX1262_WriteCommand(SX126X_CMD_SET_PACKET_TYPE, buf, 1) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Configuración base por defecto
    lora_config_t default_config = {
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

    // Habilitar marca para permitir comandos internos
    SX1262_Initialized = 1;
    if(SX1262_ApplyConfig(&default_config) != SX1262_OK)
    {
        SX1262_Initialized = 0;
        return SX1262_ERROR;
    }

    // Configurar DIO2 como RF Switch (si el layout usa el sw de semtech)
    buf[0] = 0x01; // enable
    if(SX1262_WriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, buf, 1) != SX1262_OK)
    {
        SX1262_Initialized = 0;
        return SX1262_ERROR;
    }
    return SX1262_OK;
}

/**
 * @brief Transmite datos a través del módulo SX1262
 * 
 * @param data 
 * @param length 
 * @return SX1262_Status_t 
 */
SX1262_Status_t SX1262_Transmit(uint8_t* data, uint8_t length)
{
    if(SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    // Verificar si hay una configuración pendiente sin aplicar.
    // Transmitir con parámetros obsoletos puede causar fallos silenciosos.
    if(SX1262_CurrentConfig.config_pending)
    {
        return SX1262_ERROR; // Llamar a SX1262_ApplyConfig() antes de transmitir
    }

    uint8_t buf[8];

    // Set Standby
    buf[0] = RADIOLIB_SX126X_STANDBY_RC;
    if(SX1262_WriteCommand(SX126X_CMD_SET_STANDBY, buf, 1) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Buffer base address
    buf[0] = 0x00; // TX Base
    buf[1] = 0x00; // RX Base
    if(SX1262_WriteCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Escribir datos
    if(SX1262_WriteBuffer(0x00, data, length) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Actualizar longitud de payload (requerido antes de Tx)
    buf[0] = (SX1262_CurrentConfig.preamble_len >> 8) & 0xFF;
    buf[1] = (SX1262_CurrentConfig.preamble_len) & 0xFF;
    buf[2] = 0x00; // Explicit Header
    buf[3] = length;
    buf[4] = 0x01; // CRC On
    buf[5] = SX1262_CurrentConfig.iq_inverted ? 0x01 : 0x00;
    if(SX1262_WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, buf, 6) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Limpiar alertas (Clear IRQ)
    buf[0] = 0x03; buf[1] = 0xFF; // Limpiar todo (0x03FF)
    if(SX1262_WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Habilitar DIO1 para TxDone y TxTimeout
    uint16_t irqMask = SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT;
    buf[0] = (irqMask >> 8) & 0xFF;   buf[1] = irqMask & 0xFF;
    buf[2] = (irqMask >> 8) & 0xFF;   buf[3] = irqMask & 0xFF; // DIO1 mask
    buf[4] = 0x00; buf[5] = 0x00; // DIO2
    buf[6] = 0x00; buf[7] = 0x00; // DIO3
    if(SX1262_WriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Iniciar Transmisión (timeout de chip desactivado: el soft-timeout lo controla)
    buf[0] = 0x00; buf[1] = 0x00; buf[2] = 0x00;
    if(SX1262_WriteCommand(SX126X_CMD_SET_TX, buf, 3) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Calcular timeout de software basado en el ToA real del paquete + 50 % de margen.
    // Esto evita el hardcode de 5 segundos y adapta el timeout a los parámetros LoRa.
    uint32_t toa_ms      = SX1262_ComputeToA_ms(length, &SX1262_CurrentConfig);
    uint32_t tx_timeout  = toa_ms + toa_ms / 2U + 100U; // ToA * 1.5 + 100 ms de margen

    // Esperar IRQ (DIO1 en alto => TxDone o TxTimeout)
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(DIO_GPIO_Port, DIO_GPIO_Pin) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - start) > tx_timeout)
        {
            // Timeout de software: volver a Standby y abortar
            buf[0] = RADIOLIB_SX126X_STANDBY_RC;
            if(SX1262_WriteCommand(SX126X_CMD_SET_STANDBY, buf, 1) != SX1262_OK)
            {
                return SX1262_ERROR;
            }
            buf[0] = 0x03; buf[1] = 0xFF;
            if(SX1262_WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2) != SX1262_OK)
            {
                return SX1262_ERROR;
            }
            return SX1262_TIMEOUT;
        }
    }

    // Leer y verificar los bits del registro IRQ
    uint8_t irqStatus[2];
    if(SX1262_ReadCommand(SX126X_CMD_GET_IRQ_STATUS, irqStatus, 2) != SX1262_OK)
    {
        return SX1262_ERROR;
    }
    uint16_t irqReg = ((uint16_t)irqStatus[0] << 8) | irqStatus[1];

    // Limpiar IRQ siempre antes de retornar
    buf[0] = 0x03; buf[1] = 0xFF;
    SX1262_WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2);

    // Evaluar resultado: TIMEOUT tiene prioridad sobre TX_DONE ausente
    if (irqReg & SX126X_IRQ_TIMEOUT)
    {
        buf[0] = RADIOLIB_SX126X_STANDBY_RC;
        SX1262_WriteCommand(SX126X_CMD_SET_STANDBY, buf, 1);
        return SX1262_TIMEOUT;
    }
    if ((irqReg & SX126X_IRQ_TX_DONE) == 0)
    {
        // DIO1 se levantó pero TX_DONE no está activo: condición inesperada
        return SX1262_ERROR;
    }

    return SX1262_OK;
}

/**
 * @brief Recibe datos a través del módulo SX1262
 * 
 * @param data Puntero al buffer donde se almacenarán los datos recibidos
 * @param length Puntero a una variable donde se almacenará la longitud de los datos recibidos
 * @param timeout_ms Tiempo máximo de espera para recibir datos (en milisegundos). Si es 0, espera indefinidamente.
 * @return SX1262_Status_t 
 */
SX1262_Status_t SX1262_Receive(uint8_t* data, uint8_t* length, uint32_t timeout_ms)
{
    if(SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    // Verificar si hay una configuración pendiente sin aplicar.
    // Recibir con parámetros obsoletos (frecuencia, BW, SF, etc.) puede causar
    // que el chip nunca detecte un paquete válido.
    if(SX1262_CurrentConfig.config_pending)
    {
        return SX1262_ERROR; // Llamar a SX1262_ApplyConfig() antes de recibir
    }

    uint8_t buf[8];

    // Set Standby
    buf[0] = RADIOLIB_SX126X_STANDBY_RC;
    if(SX1262_WriteCommand(SX126X_CMD_SET_STANDBY, buf, 1) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Clear IRQ
    buf[0] = 0x03; buf[1] = 0xFF;
    if(SX1262_WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Habilitar DIO1 para RxDone y RxTimeout (y CRC Err)
    uint16_t irqMask = SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR;
    buf[0] = (irqMask >> 8) & 0xFF;   buf[1] = irqMask & 0xFF; 
    buf[2] = (irqMask >> 8) & 0xFF;   buf[3] = irqMask & 0xFF; // DIO1 mask
    buf[4] = 0x00; buf[5] = 0x00; buf[6] = 0x00; buf[7] = 0x00;
    if(SX1262_WriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Iniciar Recepción
    // Pasamos de MS a Ticks (Timeout interno de chip). 1 tick = 15.625 us
    // 1 ms = 64 ticks (1000 / 15.625 = 64). Evitamos cálculo en punto flotante usando * 64.
    uint32_t timeoutBytes = (timeout_ms == 0) ? 0xFFFFFF : (timeout_ms * 64);
    buf[0] = (timeoutBytes >> 16) & 0xFF;
    buf[1] = (timeoutBytes >> 8) & 0xFF;
    buf[2] = (timeoutBytes & 0xFF);
    if(SX1262_WriteCommand(SX126X_CMD_SET_RX, buf, 3) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Bloquear hasta interrupción
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(DIO_GPIO_Port, DIO_GPIO_Pin) == GPIO_PIN_RESET)
    {
        if (timeout_ms != 0 && (HAL_GetTick() - start) > (timeout_ms + 100))
        {
            buf[0] = RADIOLIB_SX126X_STANDBY_RC;
            if(SX1262_WriteCommand(SX126X_CMD_SET_STANDBY, buf, 1) != SX1262_OK)
            {
                return SX1262_ERROR;
            }
            return SX1262_TIMEOUT; // Timeout de soft-check
        }
    }

    // Leemos status de la interrupción 
    uint8_t irqStatus[2];
    if(SX1262_ReadCommand(0x12 /* GET_IRQ_STATUS */, irqStatus, 2) != SX1262_OK)
    {
        return SX1262_ERROR;
    }
    uint16_t irqReg = (irqStatus[0] << 8) | irqStatus[1];

    if ((irqReg & SX126X_IRQ_RX_DONE) == 0 || (irqReg & SX126X_IRQ_TIMEOUT) || (irqReg & SX126X_IRQ_CRC_ERR))
    {
        buf[0] = 0x03; buf[1] = 0xFF;
        SX1262_WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2);
        return SX1262_ERROR;
    }

    // Obtener información del buffer (offset base de recepción y tamaño)
    uint8_t rxBufferStatus[2];
    if(SX1262_ReadCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, rxBufferStatus, 2) != SX1262_OK)
    {
        return SX1262_ERROR;
    }
    
    *length = rxBufferStatus[0];       // Tamaño del paquete
    uint8_t offset = rxBufferStatus[1]; // Offset en memoria interna

    // Leer payload del buffer
    if(SX1262_ReadBuffer(offset, data, *length) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Clear IRQ Status final
    buf[0] = 0x03; buf[1] = 0xFF;
    if(SX1262_WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    return SX1262_OK;
}

/**
 * @brief Configura el chip SX1262 en modo RX continuo y retorna inmediatamente.
 *
 *        No realiza ningún polling. El evento de recepción se señaliza
 *        mediante SX1262_RxDoneFlag activada desde el ISR (EXTI en DIO1).
 *
 * Flujo:
 *  1. Standby RC
 *  2. Clear IRQ
 *  3. SetDioIrqParams: habilita RxDone | Timeout | CRC_ERR | HeaderErr en DIO1
 *  4. SetRx con timeout 0xFFFFFF (RX continuo)
 *  5. Retorno inmediato
 *
 * @return SX1262_Status_t
 */
SX1262_Status_t SX1262_StartReceiveIT(void)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }
    if (SX1262_CurrentConfig.config_pending)
    {
        return SX1262_ERROR; // Llamar a SX1262_ApplyConfig() antes de recibir
    }

    uint8_t buf[8];

    // 1. Standby RC
    buf[0] = RADIOLIB_SX126X_STANDBY_RC;
    if (SX1262_WriteCommand(SX126X_CMD_SET_STANDBY, buf, 1) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // 2. Limpiar IRQ pendientes
    buf[0] = 0x03; buf[1] = 0xFF;
    if (SX1262_WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // 3. Habilitar IRQs en DIO1: RxDone | Timeout | CRC_ERR | HeaderErr
    //    HeaderErr (bit 4) se incluye para detectar paquetes con header inválido.
    uint16_t irqMask = SX126X_IRQ_RX_DONE    |
                       SX126X_IRQ_TIMEOUT    |
                       SX126X_IRQ_CRC_ERR    |
                       SX126X_IRQ_HEADER_ERR;
    buf[0] = (irqMask >> 8) & 0xFF;   buf[1] = irqMask & 0xFF; // IRQ global mask
    buf[2] = (irqMask >> 8) & 0xFF;   buf[3] = irqMask & 0xFF; // DIO1 mask
    buf[4] = 0x00; buf[5] = 0x00;                               // DIO2 (no usado)
    buf[6] = 0x00; buf[7] = 0x00;                               // DIO3 (no usado)
    if (SX1262_WriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // 4. Iniciar RX continuo (timeout = 0xFFFFFF => sin timeout de chip)
    buf[0] = 0xFF; buf[1] = 0xFF; buf[2] = 0xFF;
    if (SX1262_WriteCommand(SX126X_CMD_SET_RX, buf, 3) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Retorno inmediato — sin polling en DIO1
    return SX1262_OK;
}

/**
 * @brief Lee el payload del paquete recibido después de que SX1262_RxDoneFlag se active.
 *
 *        SOLO debe llamarse desde el main loop, nunca desde el ISR.
 *
 * @param data   Buffer de destino para el payload.
 * @param length Longitud del paquete recibido (bytes).
 * @return SX1262_Status_t
 */
SX1262_Status_t SX1262_GetReceivedPacket(uint8_t* data, uint8_t* length)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }
    if (data == NULL || length == NULL)
    {
        return SX1262_ERROR;
    }

    uint8_t buf[2];

    // 1. Leer registro IRQ del chip
    uint8_t irqStatus[2];
    if (SX1262_ReadCommand(SX126X_CMD_GET_IRQ_STATUS, irqStatus, 2) != SX1262_OK)
    {
        return SX1262_ERROR;
    }
    uint16_t irqReg = ((uint16_t)irqStatus[0] << 8) | irqStatus[1];

    // 2. Limpiar IRQ siempre (independientemente del resultado)
    buf[0] = 0x03; buf[1] = 0xFF;
    SX1262_WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2);

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
        // DIO1 subió pero RX_DONE no está activo: condición inesperada
        return SX1262_ERROR;
    }

    // 4. Obtener offset y tamaño del paquete en el buffer interno
    uint8_t rxBufferStatus[2];
    if (SX1262_ReadCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, rxBufferStatus, 2) != SX1262_OK)
    {
        return SX1262_ERROR;
    }
    *length          = rxBufferStatus[0]; // Número de bytes del payload
    uint8_t offset   = rxBufferStatus[1]; // Offset base en el buffer del chip

    // 5. Leer payload desde el buffer interno del SX1262
    if (SX1262_ReadBuffer(offset, data, *length) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    return SX1262_OK;
}

/**
 * @brief Cancela la recepción en curso y regresa el chip a modo Standby RC.
 *
 *        Útil para timeouts de software: el main loop llama esta función
 *        si SX1262_RxDoneFlag no se activó en el tiempo esperado.
 *
 * @return SX1262_Status_t
 */
SX1262_Status_t SX1262_AbortReceive(void)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }

    uint8_t buf[2];

    // Regresar a Standby RC para detener la escucha
    buf[0] = RADIOLIB_SX126X_STANDBY_RC;
    if (SX1262_WriteCommand(SX126X_CMD_SET_STANDBY, buf, 1) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Limpiar IRQ residuales
    buf[0] = 0x03; buf[1] = 0xFF;
    SX1262_WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2);

    return SX1262_OK;
}

/**
 * @brief Manejador de IRQ del SX1262 para el contexto de interrupción.
 *
 *        SOLO activa SX1262_RxDoneFlag. Sin SPI, sin HAL calls bloqueantes.
 *        Llamar desde HAL_GPIO_EXTI_Callback() cuando GPIO_Pin == DIO_Pin.
 */
void SX1262_IRQ_Handler(void)
{
    SX1262_RxDoneFlag = 1;
}

/**
 * @brief Aplica la configuración de red y modulación LoRa al chip
 * 
 * @param config Puntero a la estructura de configuración (lora_config_t)
 * @return SX1262_Status_t
 */
SX1262_Status_t SX1262_ApplyConfig(lora_config_t *config)
{
    if(SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }
    if(!config) return SX1262_ERROR;

    uint8_t buf[8];

    // Modo Standby RC necesario para configurar
    buf[0] = RADIOLIB_SX126X_STANDBY_RC;
    if(SX1262_WriteCommand(SX126X_CMD_SET_STANDBY, buf, 1) != SX1262_OK) return SX1262_ERROR;

    // --- 1. FRECUENCIA ---
    uint32_t frf = (uint32_t)(((uint64_t)config->frequency * 16384ULL) / 15625ULL);
    buf[0] = (frf >> 24) & 0xFF; 
    buf[1] = (frf >> 16) & 0xFF; 
    buf[2] = (frf >> 8) & 0xFF; 
    buf[3] = (frf & 0xFF);

    if(SX1262_WriteCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // --- 2. POTENCIA TX ---
    // Configuración PA por defecto para transceptores SX1262 (+22dBm Max)
    buf[0] = 0x04; 
    buf[1] = 0x07; 
    buf[2] = 0x00; 
    buf[3] = 0x01;

    if(SX1262_WriteCommand(0x95 /* SET_PA_CONFIG */, buf, 4) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    buf[0] = config->tx_power; // power
    buf[1] = 0x02; // rampTime 40us
    
    if(SX1262_WriteCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // --- 3. MODULACIÓN ---
    // Calcular LDRO dinámicamente: obligatorio cuando T_símbolo >= 16.38 ms
    // Combinaciones que lo activan: SF11/BW62.5, SF12/BW62.5, SF12/BW125, y anchos menores con SF altos.
    uint8_t ldro = SX1262_ComputeLDRO(config->spreading_factor, config->bandwidth);

    buf[0] = config->spreading_factor;
    buf[1] = config->bandwidth;
    buf[2] = config->coding_rate;
    buf[3] = ldro; // LowDataRateOptimize: calculado automáticamente
    if(SX1262_WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, buf, 4) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // --- 4. SYNC WORD ---
    // El campo lora_sync_word (distinto de 0) tiene prioridad absoluta sobre network_mode.
    // En caso contrario, se selecciona según el modo de red configurado.
    // Los registros 0x0740–0x0741 almacenan dos nibbles del sync word según la
    // nota de aplicación de Semtech: reg[0x0740] = (SW & 0xF0) | 0x04,
    //                                reg[0x0741] = (SW << 4)   | 0x04
    uint8_t sync_msb, sync_lsb;
    if (config->lora_sync_word != 0)
    {
        // Sync word personalizado: aplicar fórmula de nibbles de Semtech
        sync_msb = (config->lora_sync_word & 0xF0) | 0x04;
        sync_lsb = (config->lora_sync_word << 4)   | 0x04;
    }
    else
    {
        switch (config->network_mode)
        {
            case LORA_NETWORK_PUBLIC:
                sync_msb = 0x34; sync_lsb = 0x44; // SW lógico 0x34 (LoRaWAN)
                break;
            case LORA_NETWORK_MESHTASTIC:
                sync_msb = 0x2B; sync_lsb = 0xB4; // SW lógico 0x2B (Meshtastic)
                break;
            case LORA_NETWORK_PRIVATE:
            default:
                sync_msb = 0x14; sync_lsb = 0x24; // SW lógico 0x12 (LoRa privado)
                break;
        }
    }
    buf[0] = 0x07; // Dirección de registro MSB
    buf[1] = 0x40; // Dirección de registro MSB
    buf[2] = sync_msb;
    buf[3] = sync_lsb;

    if(SX1262_WriteCommand(SX126X_CMD_WRITE_REGISTER, buf, 4) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // --- 5. PARÁMETROS DEL PAQUETE ---
    buf[0] = (config->preamble_len >> 8) & 0xFF; // Preamble MSB
    buf[1] = config->preamble_len & 0xFF;        // Preamble LSB
    buf[2] = 0x00;                               // Explicit Header
    buf[3] = 0xFF;                               // PayloadLength (Dummy)
    buf[4] = 0x01;                               // CRC On
    buf[5] = config->iq_inverted ? 0x01 : 0x00;  // Invert IQ
    if(SX1262_WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, buf, 6) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Guardar estado actual
    SX1262_CurrentConfig = *config;
    SX1262_CurrentConfig.config_pending = false;

    return SX1262_OK;
}

/**
 * @brief Obtiene el RSSI del último paquete recibido.
 *
 *        Internamente emite el comando GetPacketStatus (0x14) y calcula:
 *          RSSI_pkt [dBm] = -RssiPkt / 2
 *        según el datasheet SX1262 §13.5.3.
 *
 *        Debe llamarse después de una recepción exitosa (SX1262_Receive == SX1262_OK).
 *        Si se llama fuera de ese contexto, el chip devuelve el último valor
 *        almacenado en su registro, que puede no ser válido.
 *
 * @param rssi_dbm  Puntero donde se almacenará el RSSI en dBm (valor negativo típico).
 * @return SX1262_Status_t
 */
SX1262_Status_t SX1262_GetRSSI(int16_t *rssi_dbm)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }
    if (rssi_dbm == NULL)
    {
        return SX1262_ERROR;
    }

    // GetPacketStatus devuelve 3 bytes en LoRa:
    //   [0] RssiPkt  → RSSI = -RssiPkt/2  (dBm)
    //   [1] SnrPkt   → SNR  =  SnrPkt/4   (dB, con signo)
    //   [2] SignalRssiPkt (no utilizado aquí)
    uint8_t status[3];
    if (SX1262_ReadCommand(SX126X_CMD_GET_PACKET_STATUS, status, 3) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // RssiPkt es un valor sin signo; el resultado en dBm es siempre <= 0
    *rssi_dbm = -(int16_t)status[0] / 2;

    return SX1262_OK;
}

/**
 * @brief Obtiene el SNR del último paquete recibido.
 *
 *        Internamente emite el comando GetPacketStatus (0x14) y calcula:
 *          SNR [dB] = SnrPkt / 4
 *        El byte SnrPkt es un entero con signo en complemento a 2 (int8_t),
 *        por lo que el SNR puede ser negativo (señal por debajo del nivel de ruido).
 *
 *        Debe llamarse después de una recepción exitosa (SX1262_Receive == SX1262_OK).
 *
 * @param snr_db    Puntero donde se almacenará el SNR en dB (puede ser negativo).
 * @return SX1262_Status_t
 */
SX1262_Status_t SX1262_GetSNR(int8_t *snr_db)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }
    if (snr_db == NULL)
    {
        return SX1262_ERROR;
    }

    uint8_t status[3];
    if (SX1262_ReadCommand(SX126X_CMD_GET_PACKET_STATUS, status, 3) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // SnrPkt está en el byte [1] como int8_t (complemento a dos).
    // SNR [dB] = (int8_t)SnrPkt / 4  →  resolución de 0.25 dB
    // Se devuelve redondeado a dB enteros para mayor simplicidad de uso.
    *snr_db = (int8_t)status[1] / 4;

    return SX1262_OK;
}

/**
 * @brief Retorna una copia de la configuración LoRa actualmente aplicada al chip.
 *
 *        La copia refleja el estado que fue enviado al SX1262 en la última llamada
 *        exitosa a SX1262_ApplyConfig(). El campo config_pending de la copia
 *        siempre será false, ya que solo se guarda tras una aplicación correcta.
 *
 *        Esta función es de solo lectura: no modifica el estado interno del módulo
 *        ni realiza ninguna comunicación SPI.
 *
 * @param config    Puntero a la estructura donde se copiará la configuración actual.
 * @return SX1262_Status_t
 */
SX1262_Status_t SX1262_GetConfig(lora_config_t *config)
{
    if (SX1262_Initialized != 1)
    {
        return SX1262_NOT_INITIALIZED;
    }
    if (config == NULL)
    {
        return SX1262_ERROR;
    }

    *config = SX1262_CurrentConfig;

    return SX1262_OK;
}