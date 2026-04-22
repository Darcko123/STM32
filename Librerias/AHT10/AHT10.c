/**
 * @file AHT10.c
 * 
 * @brief Librería para el sensor de temperatura y humedad AHT10 utilizando I2C en STM32
 * 
 * @author Daniel Ruiz
 * @date Abril 10, 2026
 * @version 0.1
 */

 #include "AHT10.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static I2C_HandleTypeDef* AHT10_hi2c   =   NULL;   /**< Manejador de la interfaz I2C utilizado para comunicarse con el módulo */
static uint8_t      AHT10_Initialized  =   0;      /**< Bandera para verificar si el módulo está inicializado */

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el sensor AHT10.
 *
 * Envía un comando de reset al sensor a través de I2C para asegurar que está en un estado conocido.
 *
 * @param hi2c Puntero al manejador de la interfaz I2C previamente configurado.
 * 
 * @note Esta función debe ser llamada antes de realizar lecturas de temperatura y humedad.
 */
AHT10_Status_t AHT10_Init(I2C_HandleTypeDef* hi2c)
{
    if(hi2c == NULL)
    {
        return AHT10_ERROR;
    }

    AHT10_hi2c = hi2c;

    HAL_Delay(AHT10_POWER_ON_DELAY); // Esperar inicialización interna del sensor

    uint8_t cmd[3];

    // --- Establecer a "Normal Mode" (one measurement + sleep) ---
    cmd[0] = AHT10_CMD_NORMAL;
    cmd[1] = AHT10_DATA_NOP;
    cmd[2] = AHT10_DATA_NOP;

    if(HAL_I2C_Master_Transmit(AHT10_hi2c, AHT10_ADDRESS, cmd, 3, AHT10_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return AHT10_ERROR;
    }

    HAL_Delay(AHT10_CMD_DELAY); // Esperar a que pase a modo normal

    // --- Cargar "Factory Calibration Coeff" ---
    cmd[0] = AHT10_CMD_INIT;
    cmd[1] = AHT10_INIT_CAL_ENABLE;
    cmd[2] = AHT10_DATA_NOP;

    if(HAL_I2C_Master_Transmit(AHT10_hi2c, AHT10_ADDRESS, cmd, 3, AHT10_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return AHT10_ERROR;
    }

    HAL_Delay(AHT10_CMD_DELAY); // Esperar que se aplique

    // --- Verificar bit de calibración usando status ---
    uint8_t status_byte = 0;
    if(HAL_I2C_Master_Receive(AHT10_hi2c, AHT10_ADDRESS, &status_byte, 1, AHT10_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return AHT10_ERROR;
    }

    if((status_byte & 0x08) == 0x00) // Revisar si el bit 3 está a 1
    {
        return AHT10_ERROR;
    }

    AHT10_Initialized = 1;

    return AHT10_OK;
}

/**
 * @brief Obtiene los valores de temperatura y humedad del sensor AHT10.
 *
 * @param[out] temp  Puntero para almacenar el valor de la temperatura (°C).
 * @param[out] humid Puntero para almacenar el valor de la humedad (%HR).
 *
 * @note La fórmula de conversión está basada en el datasheet del AHT10.
 */
AHT10_Status_t AHT10_Get(aht10_data_t *environment)
{
    if(AHT10_Initialized != 1)
    {
        return AHT10_NOT_INITIALIZED;
    }

    if(environment == NULL)
    {
        return AHT10_ERROR;
    }

    uint8_t cmd[3];
    cmd[0] = AHT10_CMD_START_MEASUREMENT;
    cmd[1] = AHT10_DATA_MEASURMENT_CMD;
    cmd[2] = AHT10_DATA_NOP;

    // Iniciar medición
    if(HAL_I2C_Master_Transmit(AHT10_hi2c, AHT10_ADDRESS, cmd, 3, AHT10_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return AHT10_ERROR;
    }

    HAL_Delay(AHT10_MEASURMENT_DELAY); // Retraso de tiempo de conversión

    uint8_t rxBuffer[6];
    
    // Leer los 6 bytes
    if(HAL_I2C_Master_Receive(AHT10_hi2c, AHT10_ADDRESS, rxBuffer, 6, AHT10_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return AHT10_ERROR;
    }

    // Opcional: Revisar bit busy del status (rxBuffer[0] bit 7) o bit calibración (bxBuffer[0] bit 3)
    if((rxBuffer[0] & 0x08) == 0x00) // Ver si perdimos calibración
    {
        return AHT10_ERROR;
    }

    // Construir dato 20-bit crudo de humedad
    uint32_t rawHumidity = (((uint32_t)rxBuffer[1] << 16) | ((uint16_t)rxBuffer[2] << 8) | (rxBuffer[3])) >> 4;
    
    // Construir dato 20-bit crudo de temperatura
    uint32_t rawTemperature = ((uint32_t)(rxBuffer[3] & 0x0F) << 16) | ((uint16_t)rxBuffer[4] << 8) | rxBuffer[5];

    // Conversión a valores reales siguiendo el datasheet AHT10
    // formula Humedad = (rawHumedad / 2^20) * 100
    // formula Temp    = (rawTemp / 2^20) * 200 - 50

    float humidity = (float)rawHumidity * 0.00009536743f; // 100 / (2^20) que es ~0.00009536743
    float temperature = ((float)rawTemperature * 0.00019073486f) - 50.0f; // 200 / (2^20) que es ~0.00019073486

    if(humidity < 0.0f) humidity = 0.0f;
    if(humidity > 100.0f) humidity = 100.0f;

    environment->humedad = humidity;
    environment->temperatura = temperature;

    return AHT10_OK;
}