/**
 * @file SI7021.h
 *
 * @brief Librería para el sensor de temperatura y humedad SI7021 utilizando I2C en STM32.
 *
 * @author Daniel Ruiz
 * @date Dec 28, 2024
 * @version 1.2.0
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

// ============================================================================
// MACROS Y CONSTANTES DE COMANDOS DS3231
// ============================================================================ 

/**
 * @brief Dirección I2C del módulo RTC DS3231.
 */
#define DS3231_ADDRESS

// Timeout
#define SI7021_MAX_BUSY_TIMEOUT 500     /**< milisegundos*/

// ============================================================================
// CONFIGURACIÓN ENTORNO (ESTRUCTURAS)
// ============================================================================

/**
 * @brief 
 * 
 */
typedef struct
{
    uint8_t humedad;
    uint8_t temperatura;
}si7021_data_t;

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
 * Envía un comando de reset al sensor a través de I2C para asegurar que está en un estado conocido.
 *
 * @param hi2c Puntero al manejador de la interfaz I2C previamente configurado.
 * 
 * @note Esta función debe ser llamada antes de realizar lecturas de temperatura y humedad.
 */
SI7021_Status_t SI7021_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief Lee los valores de temperatura y humedad del sensor SI7021.
 *
 * @param[out] temp  Puntero a una variable donde se almacenará la temperatura en grados Celsius (°C).
 * @param[out] humid Puntero a una variable donde se almacenará la humedad relativa (%HR).
 *
 * @note Esta función realiza la lectura secuencial de los datos del sensor.
 *
 * @attention Asegúrate de llamar a `si7021_init` antes de usar esta función.
 */
SI7021_Status_t SI7021_Get(si7021_data_t *environment);

#ifdef __cplusplus
}
#endif

#endif /* __SI7021_H */
