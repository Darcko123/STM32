/**
 * @file MPU6050.h
 * @brief    Librería para la comunicación y lectura de datos de un acelerómetro MPU6050
 * 
 * @details Esta librería proporciona funciones para inicializar y leer datos del 
 *  acelerómetro MPU6050 mediante la interfaz I2C en un microcontrolador STM32F4. 
 *  Permite obtener valores de aceleración y giroscopio en los tres ejes (X, Y, Z).
 *  
 * @author   Daniel Ruiz
 * @date     23 de septiembre de 2024
 * @version  2.0.1
 *  
 */

#ifndef MPU6050_H_
#define MPU6050_H_

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"

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
 * @brief Lee los valores de aceleración en los tres ejes.
 * 
 * @param[out] Ax Puntero donde se almacenará el valor de aceleración en el eje X.
 * @param[out] Ay Puntero donde se almacenará el valor de aceleración en el eje Y.
 * @param[out] Az Puntero donde se almacenará el valor de aceleración en el eje Z.
 * 
 * @return MPU6050_Status_t Estado de la operación
 * 
 * @note Los valores se devuelven en unidades dependientes de la configuración del rango.
 */
MPU6050_Status_t MPU6050_Read_Accel(float *Ax, float *Ay, float *Az);

/**
 * @brief Lee los valores del giroscopio en los tres ejes.
 * 
 * @param[out] Gx Puntero donde se almacenará el valor del giroscopio en el eje X.
 * @param[out] Gy Puntero donde se almacenará el valor del giroscopio en el eje Y.
 * @param[out] Gz Puntero donde se almacenará el valor del giroscopio en el eje Z.
 * 
 * @return MPU6050_Status_t Estado de la operación
 * 
 * @note Los valores se devuelven en unidades dependientes de la configuración del rango.
 */
MPU6050_Status_t MPU6050_Read_Gyro(float *Gx, float *Gy, float *Gz);

/*---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H_ */
