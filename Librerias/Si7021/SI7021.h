/**
 * @file SI7021.h
 *
 * @brief Librería para el sensor de temperatura y humedad SI7021 utilizando I2C en STM32.
 *
 * @author Daniel Ruiz
 * @date Junio 16, 2026
 * @version 2.1.0
 */

#ifndef __SI7021_H
#define __SI7021_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"

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
} SI7021_Data_t;

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
    SI7021_INVALID_PARAM = 4,	/**< Parámetro inválido */
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
 * @param hi2c Puntero al handle de I2C utilizado para comunicarse con el módulo SI7021.
 * @return SI7021_Status_t Estado de la inicialización (OK, ERROR, etc.)
 */
SI7021_Status_t SI7021_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief Desinicializa el sensor SI7021, liberando recursos y marcando el módulo como no inicializado.
 * 
 * @return SI7021_Status_t Siempre retorna SI7021_OK
 */
SI7021_Status_t SI7021_DeInit(void);

 /**
 * @brief Lee los valores de temperatura y humedad del sensor SI7021.
  * 
  * @param environment Puntero a una estructura `SI7021_Data_t` donde se almacenarán los valores de temperatura y humedad leídos del sensor.
  * @return SI7021_Status_t 
  */
SI7021_Status_t SI7021_Get(SI7021_Data_t *environment);

#ifdef __cplusplus
}
#endif

#endif /* __SI7021_H */