/**
 * @file SX1262.h
 * @brief Librería para la gestión de un módulo LoRa
 * 
 * Esta librería permite inicializar y controlar
 * 
 * @author Daniel Ruiz
 * @date March 16, 2026
 * @version 1.0
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

static SX1262_Status_t SX1262_ReadCommand(uint8_t cmd, uint8_t* buffer, uint16_t size)
{
    if (SX1262_WaitBusy() != SX1262_OK) return SX1262_TIMEOUT;

    uint8_t nop = 0x00;
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(SX1262_hspi, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }
    if(HAL_SPI_Transmit(SX1262_hspi, &nop, 1, HAL_MAX_DELAY) != HAL_OK) // El SX1262 retorna un STATUS primero
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    } 
    if (size > 0 && buffer != NULL)
    {
        if(HAL_SPI_Receive(SX1262_hspi, buffer, size, HAL_MAX_DELAY) != HAL_OK)
        {
            HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
            return SX1262_ERROR;
        }
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    return SX1262_OK;
}

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

static SX1262_Status_t SX1262_ReadBuffer(uint8_t offset, uint8_t* data, uint8_t length)
{
    if (SX1262_WaitBusy() != SX1262_OK) return SX1262_TIMEOUT;

    uint8_t cmd[3] = {SX126X_CMD_READ_BUFFER, offset, 0x00}; // NOP byte automatically handled here
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(SX1262_hspi, cmd, 3, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }
    if(HAL_SPI_Receive(SX1262_hspi, data, length, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
        return SX1262_ERROR;
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    return SX1262_OK;
}

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

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el módulo LoRa.
 * 
 * @param hspi Puntero al manejador de la interfaz SPI utilizada para comunicarse con el módulo.
 * @param GPIOx Puerto GPIO del pin CS del SX1262.
 * @param GPIO_PIN Pin GPIO del pin CS del SX1262.
 * 
 * @return SX1262_Status_t Estado de la operación
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
        .public_network = false,
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
 * @param data 
 * @param length 
 * @param timeout_ms 
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
    buf[0] = config->spreading_factor;
    buf[1] = config->bandwidth;
    buf[2] = config->coding_rate;
    buf[3] = 0x00; // LowDataRateOptimize off
    if(SX1262_WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, buf, 4) != SX1262_OK)
    {
        return SX1262_ERROR;
    }

    // --- 4. SYNC WORD ---
    uint8_t sync_msb, sync_lsb;
    if (config->lora_sync_word != 0)
    {
        sync_msb = config->lora_sync_word;
        sync_lsb = 0x44;
    } else if (config->public_network)
    {
        sync_msb = 0x34;
        sync_lsb = 0x44; // Red Pública
    } else
    {
        sync_msb = 0x14;
        sync_lsb = 0x24; // Red Privada
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
