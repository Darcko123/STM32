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
#define AHT10_ADDRESS		                0x70   /**< Dirección I2C desplazada 1 bit (0x38 << 1) */

#define AHT10_CMD_INIT                  0xE1   /**< Comando de Inicialización*/
#define AHT10_CMD_START_MEASUREMENT     0xAC   /**< Comando de Inicio de Medición*/
#define AHT10_CMD_NORMAL                0xA8   /**< Comando a modo normal*/
#define AHT10_CMD_SOFT_RESET            0xBA   /**< Comando de Reset*/

#define AHT10_INIT_CAL_ENABLE           0x08   /**< Habilitar coeficientes de calibración de fábrica*/
#define AHT10_DATA_MEASURMENT_CMD       0x33   /**< Parámetro de medición*/
#define AHT10_DATA_NOP                  0x00   /**< Parametro NOP*/

// Tiempos de espera (ms)
#define AHT10_MAX_BUSY_TIMEOUT          500    /**< Timeout para I2C ocupado*/
#define AHT10_POWER_ON_DELAY            40     /**< Retardo tras encendido*/
#define AHT10_CMD_DELAY                 350    /**< Retardo tras de enviar comando*/
#define AHT10_MEASURMENT_DELAY          80     /**< Retardo para conversión / medición*/

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
AHT10_Status_t AHT10_Get(aht10_data_t *environment);

#ifdef __cplusplus
}
#endif

 #endif /* AHT10_H */