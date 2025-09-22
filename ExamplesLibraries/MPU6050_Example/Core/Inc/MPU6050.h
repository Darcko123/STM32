/*
 * MPU6050.h
 *
 *  @brief    Librería para la comunicación y lectura de datos de un acelerómetro MPU6050
 *  @author   Daniel Ruiz
 *  @date     23 de septiembre de 2024
 *  
 *  @details  
 *  Esta librería proporciona funciones para inicializar y leer datos del acelerómetro 
 *  MPU6050 mediante la interfaz I2C en un microcontrolador STM32F4. 
 *  Permite obtener valores de aceleración y giroscopio en los tres ejes (X, Y, Z).
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f1xx_hal.h"

/*---------------- Definiciones del MPU6050 ----------------*/

/**
 * @brief Dirección I2C del MPU6050.
 * @note La dirección puede cambiar dependiendo del estado del pin AD0 (0xD0 si está a GND, 0xD1 si está a VCC).
 */
#define MPU6050_ADDRESS 0xD0

/** @brief Registro de división de muestreo. */
#define SMPLRT_DIV_REG 0x19

/** @brief Registro de configuración del giroscopio. */
#define GYRO_CONFIG_REG 0x1B

/** @brief Registro de configuración del acelerómetro. */
#define ACCEL_CONFIG_REG 0x1C

/** @brief Registro de datos de aceleración en el eje X (parte alta). */
#define ACCEL_XOUT_H_REG 0x3B

/** @brief Registro de datos de temperatura (parte alta). */
#define TEMP_OUT_H_REG 0x41

/** @brief Registro de datos del giroscopio en el eje X (parte alta). */
#define GYRO_XOUT_H_REG 0x43

/** @brief Registro de gestión de energía 1. */
#define PWR_MGMT_1_REG 0x6B

/** @brief Registro de identificación del dispositivo. */
#define WHO_AM_I_REG 0x75

/*---------------------------------------------------------*/

/*---------------- Prototipos de Funciones ----------------*/

/**
 * @brief Inicializa el MPU6050.
 *
 * @param[in] hi2c Puntero al manejador I2C.
 *
 * @details Configura los registros básicos del MPU6050 para activar el dispositivo
 *          y establecer parámetros predeterminados de muestreo.
 */
void MPU6050_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief Lee los valores de aceleración en los tres ejes.
 * 
 * @param[out] Ax Puntero donde se almacenará el valor de aceleración en el eje X.
 * @param[out] Ay Puntero donde se almacenará el valor de aceleración en el eje Y.
 * @param[out] Az Puntero donde se almacenará el valor de aceleración en el eje Z.
 * 
 * @note Los valores se devuelven en unidades dependientes de la configuración del rango.
 */
void MPU6050_Read_Accel(float *Ax, float *Ay, float *Az);

/**
 * @brief Lee los valores del giroscopio en los tres ejes.
 * 
 * @param[out] Gx Puntero donde se almacenará el valor del giroscopio en el eje X.
 * @param[out] Gy Puntero donde se almacenará el valor del giroscopio en el eje Y.
 * @param[out] Gz Puntero donde se almacenará el valor del giroscopio en el eje Z.
 * 
 * @note Los valores se devuelven en unidades dependientes de la configuración del rango.
 */
void MPU6050_Read_Gyro(float *Gx, float *Gy, float *Gz);

/*---------------------------------------------------------*/

#endif /* MPU6050_H_ */
