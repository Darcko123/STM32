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

#define LIS3DH_ADDRESS          (0x18 << 1) /**< Dirección I2C (SA0 = GND). Usar (0x19 << 1) si SA0 = VDD. */
#define LIS3DH_WHO_AM_I_CONTENT 0x33        /**< Valor esperado del registro WHO_AM_I. */

#define TEMP_CFG_REG        0x1F    /**< Configuración del ADC auxiliar y sensor de temperatura. */
#define WHO_AM_I_REG        0x0F    /**< Registro de identificación del dispositivo. */
#define CTRL_REG1           0x20    /**< ODR, modo bajo consumo y habilitación de ejes. */
#define CTRL_REG3           0x22    /**< Enrutamiento de interrupciones al pin INT1. */
#define CTRL_REG4           0x23    /**< BDU, escala completa y modo alta resolución. */

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

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el LIS3DH.
 * @details Configura los registros básicos del LIS3DH para activar el dispositivo
 *          y establecer parámetros predeterminados de muestreo.
 * 
 * @param[in] hi2c Puntero al manejador I2C.
 *
 * @return LIS3DH_Status_t Estado de la operación
 */
LIS3DH_Status_t LIS3DH_Init(I2C_HandleTypeDef* hi2c);

#ifdef __cplusplus
}
#endif

#endif /* LIS3DH_H_ */
