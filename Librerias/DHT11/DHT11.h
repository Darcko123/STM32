/**
 * @file DHT11.h
 * @brief Definiciones y prototipos para el control del módulo DHT11 mediante Timer y comunicación parasita en STM32.
 * @author Daniel Ruiz
 * @date Ene 20, 2026
 * @version 1.0.0
 */

#ifndef DHT11_H_
#define DHT11_H_

/**
 * @brief Incluir el encabezado adecuado según la familia STM32 utilizada.
 * Por ejemplo:
 * - Para STM32F1xx: "stm32f1xx_hal.h"
 * - Para STM32F4xx: "stm32f4xx_hal.h"
 */
#include "stm32f4xx_hal.h"
#include "stdio.h"

// ============================================================================
// DEFINICIONES DE CONSTANTES
// ============================================================================

#define DHT11_START_SIGNAL_US       18000   /**< Señal de inicio en microsegundos */
#define DHT11_START_WAIT_US         20      /**< Espera después de señal de inicio */
#define DHT11_RESPONSE_WAIT_US      40      /**< Espera para verificar respuesta */
#define DHT11_BIT_READ_DELAY_US     40      /**< Retardo para lectura de bit */
#define DHT11_RESPONSE_TIMEOUT_US   100     /**< Timeout para respuesta del sensor */
#define DHT11_BIT_TIMEOUT_US        150     /**< Timeout para lectura de bits */

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del DHT11.
 */
typedef enum {
	DHT11_OK = 0,				/** Operación exitosa */
	DHT11_ERROR = 1,			/** Error en la operación */
	DHT11_TIMEOUT = 2,			/** Timeout en la operación */
	DHT11_CHECKSUM_ERROR = 3,	/** Error en el checksum */
	DHT11_NOT_INITIALIZED = 4	/** Sensor no inicializado */
}DHT11_Status_t;

/**
 * @brief Estructura para almacenar los valores de temperatura y humedad leídos del sensor DHT11.
 * @param Temperature Valor de temperatura en grados Celsius.
 * @param Humidity Valor de humedad relativa en porcentaje.
 */
typedef struct
{
	float Temperature;
	float Humidity;
} DHT11_Handles_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el sensor DHT11.
 * 
 * @param htim Puntero al manejador del timer utilizado para retardos en microsegundos.
 * @param GPIOx Puerto GPIO del pin de datos del DHT11.
 * @param GPIO_PIN Pin GPIO del pin de datos del DHT11.
 * @return DHT11_Status_t 
 */
DHT11_Status_t DHT11_Init(TIM_HandleTypeDef* htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN);

/**
 * @brief Lee los valores de temperatura y humedad del sensor DHT11.
 * 
 * @param values Puntero a la estructura donde se almacenarán los valores leídos.
 * @return DHT11_Status_t 
 */
DHT11_Status_t DHT11_Read_Data(DHT11_Handles_t* values);

#ifdef __cplusplus
}
#endif

#endif /* DHT11_H_ */
