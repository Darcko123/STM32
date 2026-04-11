/**
 * @file SI7021.h
 *
 * @brief Librería para el sensor de temperatura y humedad SI7021 utilizando I2C en STM32.
 *
 * @author Daniel Ruiz
 * @date Abril 10, 2026
 * @version 2.0.0
 */

#ifndef __SI7021_H
#define __SI7021_H

/**
 * @brief Incluir el encabezado adecuado según la familia STM32 utilizada.
 * Por ejemplo:
 * - Para STM32F1xx: "stm32f1xx_hal.h"
 * - Para STM32F4xx: "stm32f4xx_hal.h"
 */
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// MACROS Y CONSTANTES DE COMANDOS Si7021
// ============================================================================ 

/**
 * @brief Dirección I2C del módulo SI7021.
 */
#define SI7021_ADDRESS		0x80

#define SI7021_REG_RESET	0xFE
#define SI7021_REG_TEMP		0xE3
#define SI7021_REG_HUM		0xE5

// Timeout
#define SI7021_MAX_BUSY_TIMEOUT 500     /**< milisegundos*/

// ============================================================================
// CONFIGURACIÓN ENTORNO (ESTRUCTURAS)
// ============================================================================

/**
 * @brief Estructura para almacenar los datos de temperatura y humedad.
 */
typedef struct
{
    float humedad;      /**< Humedad relativa (%RH) */
    float temperatura;  /**< Temperatura en grados Celsius (°C) */
} si7021_data_t;

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================
/**
 * @brief Enumeración para estados de retorno del SI7021.
 */
typedef enum {
	SI7021_OK = 0,				/**< Operación exitosa */
	SI7021_ERROR = 1,			/**< Error en la operación */
	SI7021_TIMEOUT = 2,			/**< Timeout en la operación */
	SI7021_NOT_INITIALIZED = 3,	/**< Módulo no inicializado */
}SI7021_Status_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el sensor SI7021.
 * 
 * @param hi2c Puntero al handñe de I2C utilizado para comunicarse con el módulo SI7021.
 * @return SI7021_Status_t Estado de la inicialización (OK, ERROR, etc.)
 */
SI7021_Status_t SI7021_Init(I2C_HandleTypeDef* hi2c);

 /**
 * @brief Lee los valores de temperatura y humedad del sensor SI7021.
  * 
  * @param environment Puntero a una estructura `si7021_data_t` donde se almacenarán los valores de temperatura y humedad leídos del sensor.
  * @return SI7021_Status_t 
  */
SI7021_Status_t SI7021_Get(si7021_data_t *environment);

#ifdef __cplusplus
}
#endif

#endif /* __SI7021_H */