/**
 * @file MPU6050.c
 * @brief Implementación de funciones para el manejo del acelerómetro MPU6050
 * 
 * @details Este archivo contiene las definiciones de las funciones para inicializar
 *          y leer datos del acelerómetro y giroscopio MPU6050 mediante la interfaz 
 *          I2C en un microcontrolador STM32.
 * 
 * @author   Daniel Ruiz
 * @date     Junio 17, 2026
 * @version  2.1.0
 */

#include "MPU6050.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static I2C_HandleTypeDef*   MPU6050_hi2c        = NULL; /**< Manejador de la interfaz I2C utilizado para comunicarse con el MPU6050 */
static uint8_t              MPU6050_Initialized = 0U;   /**< Bandera para verificar si el sensor está inicializado */

/** @brief Almacenan valores RAW del acelerómetro. */
static int16_t Accel_X_RAW = 0, Accel_Y_RAW = 0, Accel_Z_RAW = 0;

/** @brief Almacenan valores RAW del giroscopio. */
static int16_t Gyro_X_RAW = 0, Gyro_Y_RAW = 0, Gyro_Z_RAW = 0;

/** @brief Almacena el valor RAW de la temperatura. */
static int16_t Temp_RAW = 0;

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief  Inicializa el sensor MPU6050.
 * 
 * @param[in] hi2c Puntero al manejador I2C.
 * 
 * @details  
 * - Verifica la conexión con el dispositivo leyendo el registro WHO_AM_I.
 * - Despierta el sensor escribiendo en el registro PWR_MGMT_1.
 * - Configura el acelerómetro, giroscopio y la tasa de muestreo.
 * 
 * @return MPU6050_Status_t Estado de la operación
 */
MPU6050_Status_t MPU6050_Init(I2C_HandleTypeDef* hi2c)
{
    HAL_StatusTypeDef hal_status;
    uint8_t check, data;

    // Validar parámetros de entrada
    if(hi2c == NULL)
    {
        return MPU6050_INVALID_PARAM;
    }

    // Almacenar configuración para uso en funciones posteriores
    MPU6050_hi2c = hi2c;

    // Comprobar conexión con el MPU6050 mediante el registro WHO_AM_I
    hal_status = HAL_I2C_Mem_Read(MPU6050_hi2c, MPU6050_ADDRESS, WHO_AM_I_REG, 1, &check, 1, 1000);

    if(hal_status != HAL_OK)
    {
        return MPU6050_TIMEOUT;
    }

    if (check != 104) // El valor esperado para MPU6050 es 104 (0x68)
    {
        return MPU6050_ERROR;
    }

    // Despertar el sensor escribiendo 0 en PWR_MGMT_1
    data = 0x00;
    hal_status = HAL_I2C_Mem_Write(MPU6050_hi2c, MPU6050_ADDRESS, PWR_MGMT_1_REG, 1, &data, 1, 1000);
    if(hal_status != HAL_OK)
    {
        return MPU6050_TIMEOUT;
    }

    // Configurar la tasa de muestreo (Sample Rate Divider) a 1 kHz
    data = 0x07;
    hal_status = HAL_I2C_Mem_Write(MPU6050_hi2c, MPU6050_ADDRESS, SMPLRT_DIV_REG, 1, &data, 1, 1000);
    if(hal_status != HAL_OK)
    {
        return MPU6050_TIMEOUT;
    }

    // Configurar acelerómetro con rango ±2g
    data = 0x00;
    hal_status = HAL_I2C_Mem_Write(MPU6050_hi2c, MPU6050_ADDRESS, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
    if(hal_status != HAL_OK)
    {
        return MPU6050_TIMEOUT;
    }

    // Configurar giroscopio con rango ±250 dps
    data = 0x00;
    hal_status = HAL_I2C_Mem_Write(MPU6050_hi2c, MPU6050_ADDRESS, GYRO_CONFIG_REG, 1, &data, 1, 1000);
    if(hal_status != HAL_OK)
    {
        return MPU6050_TIMEOUT;
    }

    // Marcar como inicializado exitosamente
    MPU6050_Initialized = 1U;

    return MPU6050_OK;
}

/**
 * @brief Desinicializa el sensor MPU6050, liberando recursos y marcando el módulo como no inicializado.
 * 
 * @return MPU6050_Status_t Siempre retorna MPU6050_OK
 */
MPU6050_Status_t MPU6050_DeInit(void)
{
    MPU6050_hi2c         = NULL;    // Limpia el handler de I2C
    MPU6050_Initialized  = 0U;      // Marca el módulo como no inicializado
    
    return MPU6050_OK;
}

/**
 * @brief Lee los valores del acelerómetro y los convierte a unidades 'g'.
 * 
 * @param[out] accel Puntero a la estructura donde se almacenarán los valores de aceleración en 'g'.
 * 
 * @return MPU6050_Status_t Estado de la operación
 * 
 * @details  
 * - Se leen 6 bytes correspondientes a los valores de los ejes X, Y y Z.
 * - Se convierten los valores RAW a aceleración usando el factor de escala (FS_SEL = 0 → 16384 LSB/g).
 */
MPU6050_Status_t MPU6050_Read_Accel(MPU6050_Accel_t *accel)
{
    HAL_StatusTypeDef hal_status;
    uint8_t Rec_Data[6];

    // Verificación de inicialización
    if(MPU6050_Initialized == 0) { return MPU6050_NOT_INITIALIZED; }

    // Validar puntero de salida
    if(accel == NULL) { return MPU6050_INVALID_PARAM; }

    // Leer 6 bytes de datos del acelerómetro
    hal_status = HAL_I2C_Mem_Read(MPU6050_hi2c, MPU6050_ADDRESS, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    if(hal_status == HAL_TIMEOUT)
    {
        return MPU6050_TIMEOUT;
    }
    else if (hal_status != HAL_OK)
    {
        return MPU6050_ERROR;
    }

    // Convertir datos RAW a valores enteros de 16 bits
    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Convertir valores RAW a aceleración en 'g'
    accel->x = Accel_X_RAW / 16384.0;
    accel->y = Accel_Y_RAW / 16384.0;
    accel->z = Accel_Z_RAW / 16384.0;

    return MPU6050_OK;
}

/**
 * @brief Lee los valores del giroscopio y los convierte a unidades 'dps'.
 * 
 * @param[out] gyro Puntero a la estructura donde se almacenarán los valores de velocidad angular en 'dps'.
 * 
 * @return MPU6050_Status_t Estado de la operación
 * 
 * @details  
 * - Se leen 6 bytes correspondientes a los valores de los ejes X, Y y Z.
 * - Se convierten los valores RAW a velocidad angular usando el factor de escala (FS_SEL = 0 → 131 LSB/dps).
 */
MPU6050_Status_t MPU6050_Read_Gyro(MPU6050_Gyro_t *gyro)
{
    HAL_StatusTypeDef hal_status;
    uint8_t Rec_Data[6];

    // Verificación de inicialización
    if(MPU6050_Initialized == 0) { return MPU6050_NOT_INITIALIZED; }

    // Validar puntero de salida
    if(gyro == NULL) { return MPU6050_INVALID_PARAM; }

    // Leer 6 bytes de datos del giroscopio
    hal_status = HAL_I2C_Mem_Read(MPU6050_hi2c, MPU6050_ADDRESS, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    if(hal_status == HAL_TIMEOUT)
    {
        return MPU6050_TIMEOUT;
    }
    else if (hal_status != HAL_OK)
    {
        return MPU6050_ERROR;
    }

    // Convertir datos RAW a valores enteros de 16 bits
    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Convertir valores RAW a velocidad angular en 'dps'
    gyro->x = Gyro_X_RAW / 131.0;
    gyro->y = Gyro_Y_RAW / 131.0;
    gyro->z = Gyro_Z_RAW / 131.0;

    return MPU6050_OK;
}

/**
 * @brief Lee la temperatura del sensor y la convierte a grados Celsius.
 *
 * @param[out] temp Puntero a la variable donde se almacenará la temperatura en grados Celsius.
 *
 * @return MPU6050_Status_t Estado de la operación
 *
 * @details
 * - Se leen 2 bytes correspondientes al registro de temperatura.
 * - Se convierte el valor RAW a grados Celsius según la fórmula del datasheet: Temp = RAW/340 + 36.53.
 */
MPU6050_Status_t MPU6050_Read_Temp(float *temp)
{
    HAL_StatusTypeDef hal_status;
    uint8_t Rec_Data[2];

    // Verificación de inicialización
    if(MPU6050_Initialized == 0) { return MPU6050_NOT_INITIALIZED; }

    // Validar puntero de salida
    if(temp == NULL) { return MPU6050_INVALID_PARAM; }

    // Leer 2 bytes de datos de temperatura
    hal_status = HAL_I2C_Mem_Read(MPU6050_hi2c, MPU6050_ADDRESS, TEMP_OUT_H_REG, 1, Rec_Data, 2, 1000);

    if(hal_status == HAL_TIMEOUT)
    {
        return MPU6050_TIMEOUT;
    }
    else if (hal_status != HAL_OK)
    {
        return MPU6050_ERROR;
    }

    // Convertir datos RAW a valor entero de 16 bits
    Temp_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

    // Convertir valor RAW a grados Celsius
    *temp = Temp_RAW / 340.0 + 36.53;

    return MPU6050_OK;
}

/**
 * @brief Activa o desactiva el modo de bajo consumo (sleep) del MPU6050.
 *
 * @param[in] enable 1 para activar el modo sleep, 0 para despertar el sensor.
 *
 * @return MPU6050_Status_t Estado de la operación
 *
 * @details
 * - Lee el registro PWR_MGMT_1 y modifica únicamente el bit SLEEP,
 *   preservando el resto de la configuración (clock source, etc.).
 */
MPU6050_Status_t MPU6050_Set_Sleep_Mode(uint8_t enable)
{
    HAL_StatusTypeDef hal_status;
    uint8_t data;

    // Verificación de inicialización
    if(MPU6050_Initialized == 0)     { return MPU6050_NOT_INITIALIZED; }
    if((enable < 0) || (enable > 1)) { return MPU6050_INVALID_PARAM; }

    // Leer valor actual del registro PWR_MGMT_1
    hal_status = HAL_I2C_Mem_Read(MPU6050_hi2c, MPU6050_ADDRESS, PWR_MGMT_1_REG, 1, &data, 1, 1000);
    if(hal_status == HAL_TIMEOUT)
    {
        return MPU6050_TIMEOUT;
    }
    else if (hal_status != HAL_OK)
    {
        return MPU6050_ERROR;
    }

    // Modificar únicamente el bit SLEEP, preservando el resto de la configuración
    if(enable)
    {
        data |= PWR_MGMT_1_SLEEP_BIT;
    }
    else
    {
        data &= ~PWR_MGMT_1_SLEEP_BIT;
    }

    // Escribir el valor actualizado en el registro PWR_MGMT_1
    hal_status = HAL_I2C_Mem_Write(MPU6050_hi2c, MPU6050_ADDRESS, PWR_MGMT_1_REG, 1, &data, 1, 1000);
    if(hal_status == HAL_TIMEOUT)
    {
        return MPU6050_TIMEOUT;
    }
    else if (hal_status != HAL_OK)
    {
        return MPU6050_ERROR;
    }

    return MPU6050_OK;
}
