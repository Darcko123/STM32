/**
 * @file AHT10.h
 * 
 * @brief Librería para el sensor de temperatura y humedad AHT10 utilizando I2C en STM32
 * 
 * @author Daniel Ruiz
 * @date Abril 10, 2026
 * @version 0.1
 */

 #ifndef AHT10_H
 #define AHT10_H

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
// MACROS Y CONSTANTES DE COMANDOS AHT10
// ============================================================================ 

/**
 * @brief Dirección I2C del módulo AHT10.
 */
#define AHT10_ADDRESS		        0x38

#define AHT10_CMD_INIT              0xE1  // Comando de Inicialización
#define AHT10_CMD_START_MEASUREMENT 0xAC
#define AHT10_CMD_RESET             0xBA

// Timeout
#define AHT10_MAX_BUSY_TIMEOUT 500     /**< milisegundos*/

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
} aht10_data_t;

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================
/**
 * @brief Enumeración para estados de retorno del AHT10.
 */
typedef enum {
	AHT10_OK = 0,				/**< Operación exitosa */
	AHT10_ERROR = 1,			/**< Error en la operación */
	AHT10_TIMEOUT = 2,			/**< Timeout en la operación */
	AHT10_NOT_INITIALIZED = 3,	/**< Módulo no inicializado */
}AHT10_Status_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el sensor AHT10.
 * 
 * @param hi2c Puntero al handñe de I2C utilizado para comunicarse con el módulo AHT10.
 * @return AHT10_Status_t Estado de la inicialización (OK, ERROR, etc.)
 */
AHT10_Status_t AHT10_Init(I2C_HandleTypeDef* hi2c);

 /**
 * @brief Lee los valores de temperatura y humedad del sensor AHT10.
  * 
  * @param environment Puntero a una estructura `aht10_data_t` donde se almacenarán los valores de temperatura y humedad leídos del sensor.
  * @return AHT10_Status_t 
  */
AHT10_Status_t AHT10_Get(si7021_data_t *environment);

#ifdef __cplusplus
}
#endif

 #endif /* AHT10_H */