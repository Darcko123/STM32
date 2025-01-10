/*
 * MPU6050.c
 *
 *  @brief    Implementación de funciones para el manejo del acelerómetro MPU6050
 *  @author   Daniel Ruiz
 *  @date     23 de septiembre de 2024
 *
 *  @details  
 *  Este archivo contiene las definiciones de las funciones para inicializar y leer datos 
 *  del acelerómetro y giroscopio MPU6050 mediante la interfaz I2C en un microcontrolador STM32F4.
 */

#include "MPU6050.h"

extern I2C_HandleTypeDef hi2c1;

/*-------------------- Variables Globales --------------------*/
/** @brief Almacenan valores RAW del acelerómetro. */
int16_t Accel_X_RAW = 0, Accel_Y_RAW = 0, Accel_Z_RAW = 0;

/** @brief Almacenan valores RAW del giroscopio. */
int16_t Gyro_X_RAW = 0, Gyro_Y_RAW = 0, Gyro_Z_RAW = 0;

/** @brief Almacenan valores convertidos de aceleración. */
float AxF = 0, AyF = 0, AzF = 0;

/** @brief Almacenan valores convertidos de giroscopio. */
float GxF = 0, GyF = 0, GzF = 0;

/*-----------------------------------------------------------*/

/**
 * @brief  Inicializa el sensor MPU6050.
 * 
 * @details  
 * - Verifica la conexión con el dispositivo leyendo el registro WHO_AM_I.
 * - Despierta el sensor escribiendo en el registro PWR_MGMT_1.
 * - Configura el acelerómetro, giroscopio y la tasa de muestreo.
 */
void MPU6050_Init(void)
{
    uint8_t check, data;

    // Comprobar conexión con el MPU6050 mediante el registro WHO_AM_I
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 104) // El valor esperado para MPU6050 es 104 (0x68)
    {
        // Despertar el sensor escribiendo 0 en PWR_MGMT_1
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, PWR_MGMT_1_REG, 1, &data, 1, 1000);

        // Configurar la tasa de muestreo (Sample Rate Divider) a 1 kHz
        data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, SMPLRT_DIV_REG, 1, &data, 1, 1000);

        // Configurar acelerómetro con rango ±2g
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

        // Configurar giroscopio con rango ±250 dps
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, GYRO_CONFIG_REG, 1, &data, 1, 1000);
    }
}

/**
 * @brief Lee los valores del acelerómetro y los convierte a unidades 'g'.
 * 
 * @param[out] Ax Puntero para almacenar la aceleración en el eje X (en 'g').
 * @param[out] Ay Puntero para almacenar la aceleración en el eje Y (en 'g').
 * @param[out] Az Puntero para almacenar la aceleración en el eje Z (en 'g').
 * 
 * @details  
 * - Se leen 6 bytes correspondientes a los valores de los ejes X, Y y Z.
 * - Se convierten los valores RAW a aceleración usando el factor de escala (FS_SEL = 0 → 16384 LSB/g).
 */
void MPU6050_Read_Accel(float *Ax, float *Ay, float *Az)
{
    uint8_t Rec_Data[6];

    // Leer 6 bytes de datos del acelerómetro
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    // Convertir datos RAW a valores enteros de 16 bits
    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Convertir valores RAW a aceleración en 'g'
    *Ax = Accel_X_RAW / 16384.0;
    *Ay = Accel_Y_RAW / 16384.0;
    *Az = Accel_Z_RAW / 16384.0;
}

/**
 * @brief Lee los valores del giroscopio y los convierte a unidades 'dps'.
 * 
 * @param[out] Gx Puntero para almacenar la velocidad angular en el eje X (en 'dps').
 * @param[out] Gy Puntero para almacenar la velocidad angular en el eje Y (en 'dps').
 * @param[out] Gz Puntero para almacenar la velocidad angular en el eje Z (en 'dps').
 * 
 * @details  
 * - Se leen 6 bytes correspondientes a los valores de los ejes X, Y y Z.
 * - Se convierten los valores RAW a velocidad angular usando el factor de escala (FS_SEL = 0 → 131 LSB/dps).
 */
void MPU6050_Read_Gyro(float *Gx, float *Gy, float *Gz)
{
    uint8_t Rec_Data[6];

    // Leer 6 bytes de datos del giroscopio
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    // Convertir datos RAW a valores enteros de 16 bits
    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Convertir valores RAW a velocidad angular en 'dps'
    *Gx = Gyro_X_RAW / 131.0;
    *Gy = Gyro_Y_RAW / 131.0;
    *Gz = Gyro_Z_RAW / 131.0;
}
