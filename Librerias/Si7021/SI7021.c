/**
 * @file SI7021.c
 *
 * @brief Implementación de la librería para el sensor de temperatura y humedad SI7021 utilizando I2C en STM32.
 *
 * @author Daniel Ruiz
 * @date Dec 28, 2024
 * @version 1.2.0
 */

#include "SI7021.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static I2C_HandleTypeDef* SI7021_hi2c   =   NULL;   /**< Manejador de la interfaz I2C utilizado para comunicarse con el módulo */
static uint8_t      SI7021_Initialized  =   0;      /**< Bandera para verificar si el módulo está inicializado */

/**
 * @brief Buffers de datos para comunicación I2C.
 * 
 * - data: Almacena datos recibidos del sensor (temperatura).
 * - da: Comando de reinicio (0xFE).
 * - read: Comandos para medir temperatura (0xE3) y humedad (0xE5).
 * - data1: Almacena datos recibidos del sensor (humedad).
 */
static uint8_t data[3];       // Buffer para datos recibidos de temperatura
static uint8_t da[1] = {0xFE}; // Comando para resetear el sensor
static uint8_t read[2] = {0xE3, 0xE5}; // Comandos: 0xE3 (medir temperatura), 0xE5 (medir humedad)
static uint8_t data1[3];      // Buffer para datos recibidos de humedad

/**
 * @brief Variables locales para almacenar los valores de temperatura y humedad.
 * 
 * - sum_temp: Almacena el valor combinado de temperatura.
 * - sum_humid: Almacena el valor combinado de humedad.
 * - tempp: Valor temporal de temperatura en formato flotante.
 * - hu: Valor temporal de humedad en formato flotante.
 */
static uint16_t sum_temp, sum_humid;
static float tempp, hu;

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

    // Envía el comando de reset (0xFE) al sensor SI7021
    if(HAL_I2C_Master_Transmit(SI7021_hi2c, 0x80, da, 1, 100) != HAL_OK)
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
    if(SI7021_Initialized)
    {
        return SI7021_NOT_INITIALIZED;
    }

    // ----- Lectura de Temperatura -----
    if(HAL_I2C_Master_Transmit(SI7021_hi2c, 0x80, &read[0], 1, 100) != HAL_OK)  // Envía comando para medir temperatura (0xE3)
    {
        return SI7021_ERROR;
    }
    if(HAL_I2C_Master_Receive(SI7021_hi2c, 0x80, data, 3, 1000) != HAL_OK)  // Recibe 3 bytes de datos de temperatura
    {
        return SI7021_ERROR;
    }

    // Combina los datos recibidos en un entero de 16 bits
    sum_temp = (data[0] << 8) | data[1];
    tempp = sum_temp;

    // Convierte el valor recibido a grados Celsius usando la fórmula del datasheet
    environment->temperatura = (-46.85 + (175.72 * (tempp / 65536)));
    HAL_Delay(20);

    // ----- Lectura de Humedad -----
    if(HAL_I2C_Master_Transmit(SI7021_hi2c, 0x80, &read[1], 1, 100) != HAL_OK) // Envía comando para medir humedad (0xE5)
    {
        return SI7021_ERROR;
    }

    if(HAL_I2C_Master_Receive(SI7021_hi2c, 0x80, data1, 3, 1000) != HAL_OK) // Recibe 3 bytes de datos de humedad
    {
        return SI7021_ERROR;
    }

    // Combina los datos recibidos en un entero de 16 bits
    sum_humid = (data1[0] << 8) | data1[1];
    hu = sum_humid;

    // Convierte el valor recibido a porcentaje de humedad usando la fórmula del datasheet
    environment->humedad = (-6 + (125 * (hu / 65536)));
    HAL_Delay(20);

    return SI7021_OK;
}
