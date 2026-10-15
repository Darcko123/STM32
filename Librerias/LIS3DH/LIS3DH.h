/**
 * @file LIS3DH.h
 * @brief    Librería para la comunicación y lectura de datos de un acelerómetro LIS3DH
 * 
 * @details Esta librería proporciona funciones para inicializar y leer datos del 
 *  acelerómetro LIS3DH mediante la interfaz I2C en un microcontrolador STM32.
 *  Permite obtener valores de aceleración y giroscopio en los tres ejes (X, Y, Z).
 *  
 * @author   Daniel Ruiz
 * @date     Octubre 15, 2026
 * @version  1.0.0
 *  
 */

#ifndef LIS3DH_H_
#define LIS3DH_H_

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdint.h>

// ============================================================================
// DEFINICIONES DE CONSTANTES
// ============================================================================

#define WHO_AM_I_REG        0x18    /**< Registro de identificación del dispositivo. */

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del LIS3DH.
 */
typedef enum {
    LIS3DH_OK              = 0,    /**< Operación exitosa */
    LIS3DH_ERROR           = 1,    /**< Error en la operación */
    LIS3DH_TIMEOUT         = 2,    /**< Timeout en la operación */
    LIS3DH_NOT_INITIALIZED = 3,	/**< Módulo no inicializado */
    LIS3DH_INVALID_PARAM   = 4     /**< Parámetro inválido */
}LIS3DH_Status_t;

/**
 * @brief Datos de medición del acelerómetro del LIS3DH.
 */
typedef struct {
    float x;    /**< Valor de aceleración en el eje X. */
    float y;    /**< Valor de aceleración en el eje Y. */
    float z;    /**< Valor de aceleración en el eje Z. */
} LIS3DH_Accel_t;

/**
 * @brief Datos de medición del giroscopio del LIS3DH.
 */
typedef struct {
    float x;    /**< Valor de velocidad angular en el eje X. */
    float y;    /**< Valor de velocidad angular en el eje Y. */
    float z;    /**< Valor de velocidad angular en el eje Z. */
} LIS3DH_Gyro_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* LIS3DH_H_ */
