/**
 * @file SI7021.c
 *
 * @brief Implementación de la librería para el sensor de temperatura y humedad SI7021 utilizando I2C en STM32.
 *
 * @author Daniel Ruiz
 * @date Abril 10, 2026
 * @version 2.0.0
 */

#include "SI7021.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static I2C_HandleTypeDef* SI7021_hi2c   =   NULL;   /**< Manejador de la interfaz I2C utilizado para comunicarse con el módulo */
static uint8_t      SI7021_Initialized  =   0;      /**< Bandera para verificar si el módulo está inicializado */


static uint8_t dataTemp[3];      // Buffer para datos recibidos de temperatura
static uint8_t dataHume[3];      // Buffer para datos recibidos de humedad

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================


/**
 * @brief Inicializa el sensor SI7021.
 *
 * Envía un comando de reset al sensor a través de I2C para asegurar que está en un estado conocido.
 *
 * @param hi2c Puntero al manejador de la interfaz I2C previamente configurado.
 * 
 * @note Esta función debe ser llamada antes de realizar lecturas de temperatura y humedad.
 */
SI7021_Status_t SI7021_Init(I2C_HandleTypeDef* hi2c)
{
    if(hi2c == NULL)
    {
        return SI7021_ERROR;
    }

    // Asigna el handler de I2C proporcionado a la variable estática
    SI7021_hi2c = hi2c;

    // Envía el comando de reset (SI7021_REG_RESET) al sensor SI7021
    uint8_t cmd = SI7021_REG_RESET;
    if(HAL_I2C_Master_Transmit(SI7021_hi2c, SI7021_ADDRESS, &cmd, 1, SI7021_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return SI7021_ERROR;
    }

    HAL_Delay(20); // Espera 20 ms para completar el reset

    SI7021_Initialized = 1;

    return SI7021_OK;
}

/**
 * @brief Obtiene los valores de temperatura y humedad del sensor SI7021.
 *
 * @param[out] temp  Puntero para almacenar el valor de la temperatura (°C).
 * @param[out] humid Puntero para almacenar el valor de la humedad (%HR).
 *
 * @note La fórmula de conversión está basada en el datasheet del SI7021.
 */
SI7021_Status_t SI7021_Get(si7021_data_t *environment)
{
    if(SI7021_Initialized != 1)
    {
        return SI7021_NOT_INITIALIZED;
    }

    if(environment == NULL)
    {
        return SI7021_ERROR;
    }

    uint8_t cmd;
    uint16_t Temp_Code;
    uint16_t RH_Code;

    // ----- Lectura de Temperatura -----
    cmd = SI7021_REG_TEMP;
    if(HAL_I2C_Master_Transmit(SI7021_hi2c, SI7021_ADDRESS, &cmd, 1, SI7021_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return SI7021_ERROR;
    }
    
    HAL_Delay(25); // Espera para la conversión de temperatura (Máx 25ms para 14-bit)

    if(HAL_I2C_Master_Receive(SI7021_hi2c, SI7021_ADDRESS, dataTemp, 3, SI7021_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return SI7021_ERROR;
    }

    // Combina los datos recibidos en un entero de 16 bits
    Temp_Code = (dataTemp[0] << 8) | dataTemp[1];

    // Convierte el valor recibido a grados Celsius usando la fórmula del datasheet
    // Formula: Temp = (175.72 * Temp_Code / 65536) - 46.85
    environment->temperatura = (((175.72f * (float)Temp_Code) / 65536.0f) - 46.85f);

    // ----- Lectura de Humedad -----
    cmd = SI7021_REG_HUM;
    if(HAL_I2C_Master_Transmit(SI7021_hi2c, SI7021_ADDRESS, &cmd, 1, SI7021_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return SI7021_ERROR;
    }

    HAL_Delay(25); // Espera para la conversión de humedad (Máx 25ms para 12-bit)

    if(HAL_I2C_Master_Receive(SI7021_hi2c, SI7021_ADDRESS, dataHume, 3, SI7021_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return SI7021_ERROR;
    }

    // Combina los datos recibidos en un entero de 16 bits
    RH_Code = (dataHume[0] << 8) | dataHume[1];

    // Convierte el valor recibido a porcentaje de humedad usando la fórmula del datasheet
    // Formula: RH = (125 * RH_Code / 65536) - 6
    environment->humedad = (((125.0f * (float)RH_Code) / 65536.0f) - 6.0f);

    return SI7021_OK;
}
