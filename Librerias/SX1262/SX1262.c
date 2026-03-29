/**
 * @file SX1262.h
 * @brief Librería para la gestión de un módulo LoRa
 * 
 * Esta librería permite inicializar y controlar
 * 
 * @author Daniel Ruiz
 * @date March 28, 2026
 * @version 1.0.0
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
    
    SX1262_Initialized = 1;
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

    // Actualizar longitud de payload (Requerido antes de Tx)
    // Usamos los mismos parámetros configurados, pero actualizamos `length`
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

    // Iniciar Transmisión
    buf[0] = 0x00; buf[1] = 0x00; buf[2] = 0x00; // 0=Timeout desactivado
    if(SX1262_WriteCommand(SX126X_CMD_SET_TX, buf, 3) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // Esperar IRQ (DIO1 en alto => TxDone) o un timeout artificial
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(DIO_GPIO_Port, DIO_GPIO_Pin) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - start) > 5000)  // Timeout software de 5 seg
        {
            buf[0] = RADIOLIB_SX126X_STANDBY_RC;
            if(SX1262_WriteCommand(SX126X_CMD_SET_STANDBY, buf, 1) != SX1262_OK)
            {
                return SX1262_ERROR;
            }
            return SX1262_TIMEOUT;
        }
    }

    // Comprobar IRQ Status Real
    uint8_t irqStatus[2];
    if(SX1262_ReadCommand(0x12 /* GET_IRQ_STATUS */, irqStatus, 2) != SX1262_OK)
    {
        return SX1262_ERROR;
    }
    
    // Clear IRQ
    buf[0] = 0x03; buf[1] = 0xFF;
    if(SX1262_WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2) != SX1262_OK)
    {
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