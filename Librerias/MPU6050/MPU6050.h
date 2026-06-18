/**
 * @file MPU6050.h
 * @brief    Librería para la comunicación y lectura de datos de un acelerómetro MPU6050
 * 
 * @details Esta librería proporciona funciones para inicializar y leer datos del 
 *  acelerómetro MPU6050 mediante la interfaz I2C en un microcontrolador STM32F4. 
 *  Permite obtener valores de aceleración y giroscopio en los tres ejes (X, Y, Z).
 *  
 * @author   Daniel Ruiz
 * @date     Junio 17, 2026
 * @version  2.1.0
 *  
 */

#ifndef MPU6050_H_
#define MPU6050_H_

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdint.h>

// ============================================================================
// DEFINICIONES DE CONSTANTES
// ============================================================================

/**
 * @brief Dirección I2C del MPU6050.
 * @note La dirección puede cambiar dependiendo del estado del pin AD0 (0xD0 si está a GND, 0xD1 si está a VCC).
 */
#define MPU6050_ADDRESS     0xD0

#define SMPLRT_DIV_REG      0x19    /**< Registro de división de muestreo. */
#define GYRO_CONFIG_REG     0x1B    /**< Registro de configuración del giroscopio. */
#define ACCEL_CONFIG_REG    0x1C    /**< Registro de configuración del acelerómetro. */
#define ACCEL_XOUT_H_REG    0x3B    /**< Registro de datos del acelerómetro en el eje X (parte alta). */
#define TEMP_OUT_H_REG      0x41    /**< Registro de datos de temperatura (parte alta). */
#define GYRO_XOUT_H_REG     0x43    /**< Registro de datos del giroscopio en el eje X (parte alta). */
#define PWR_MGMT_1_REG      0x6B    /**< Registro de gestión de energía 1. */
#define PWR_MGMT_1_SLEEP_BIT (1U << 6)  /**< Bit SLEEP del registro PWR_MGMT_1. */

#define WHO_AM_I_REG        0x75    /**< Registro de identificación del dispositivo. */

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del MPU6050.
 */
typedef enum {
    MPU6050_OK              = 0,    /**< Operación exitosa */
    MPU6050_ERROR           = 1,    /**< Error en la operación */
    MPU6050_TIMEOUT         = 2,    /**< Timeout en la operación */
    MPU6050_NOT_INITIALIZED = 3,	/**< Módulo no inicializado */
    MPU6050_INVALID_PARAM   = 4     /**< Parámetro inválido */
}MPU6050_Status_t;

/**
 * @brief Datos de medición del acelerómetro del MPU6050.
 */
typedef struct {
    float x;    /**< Valor de aceleración en el eje X. */
    float y;    /**< Valor de aceleración en el eje Y. */
    float z;    /**< Valor de aceleración en el eje Z. */
} MPU6050_Accel_t;

/**
 * @brief Datos de medición del giroscopio del MPU6050.
 */
typedef struct {
    float x;    /**< Valor de velocidad angular en el eje X. */
    float y;    /**< Valor de velocidad angular en el eje Y. */
    float z;    /**< Valor de velocidad angular en el eje Z. */
} MPU6050_Gyro_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el MPU6050.
 * @details Configura los registros básicos del MPU6050 para activar el dispositivo
 *          y establecer parámetros predeterminados de muestreo.
 * 
 * @param[in] hi2c Puntero al manejador I2C.
 *
 * @return MPU6050_Status_t Estado de la operación
 */
MPU6050_Status_t MPU6050_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief Desinicializa el sensor MPU6050, liberando recursos y marcando el módulo como no inicializado.
 * 
 * @return MPU6050_Status_t Siempre retorna MPU6050_OK
 */
MPU6050_Status_t MPU6050_DeInit(void);

/**
 * @brief Lee los valores de aceleración en los tres ejes.
 * 
 * @param[out] accel Puntero a la estructura donde se almacenarán los valores de aceleración.
 * 
 * @return MPU6050_Status_t Estado de la operación
 */
MPU6050_Status_t MPU6050_Read_Accel(MPU6050_Accel_t *accel);

/**
 * @brief Lee los valores del giroscopio en los tres ejes.
 * 
 * @param[out] gyro Puntero a la estructura donde se almacenarán los valores de velocidad angular.
 * 
 * @return MPU6050_Status_t Estado de la operación
 */
MPU6050_Status_t MPU6050_Read_Gyro(MPU6050_Gyro_t *gyro);

/**
 * @brief Lee la temperatura del sensor MPU6050.
 *
 * @param[out] temp Puntero a la variable donde se almacenará la temperatura en grados Celsius.
 *
 * @return MPU6050_Status_t Estado de la operación
 */
MPU6050_Status_t MPU6050_Read_Temp(float *temp);

/**
 * @brief Activa o desactiva el modo de bajo consumo (sleep) del MPU6050 sin reinicializar el sensor.
 *
 * @param[in] enable 1 para activar el modo sleep, 0 para despertar el sensor.
 *
 * @return MPU6050_Status_t Estado de la operación
 */
MPU6050_Status_t MPU6050_Set_Sleep_Mode(uint8_t enable);

/*---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H_ */
