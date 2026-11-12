/**
 * @file DHT11.h
 *
 * @brief Librería para el sensor de temperatura y humedad DHT11 utilizando un Timer y protocolo single-wire en STM32.
 *
 * @author Daniel Ruiz
 * @date Ene 20, 2026
 * @version 1.0.0
 */

#ifndef DHT11_H_
#define DHT11_H_

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdint.h>

// ============================================================================
// MACROS Y CONSTANTES DE TIEMPO DHT11
// ============================================================================

#define DHT11_START_SIGNAL_US       18000   /**< Señal de inicio en microsegundos */
#define DHT11_START_WAIT_US         20      /**< Espera después de señal de inicio */
#define DHT11_RESPONSE_WAIT_US      40      /**< Espera para verificar respuesta */
#define DHT11_BIT_READ_DELAY_US     40      /**< Retardo para lectura de bit */
#define DHT11_RESPONSE_TIMEOUT_US   100     /**< Timeout para respuesta del sensor */
#define DHT11_BIT_TIMEOUT_US        150     /**< Timeout para lectura de bits */
#define DHT11_MIN_INTERVAL_MS       2000U   /**< Intervalo mínimo entre lecturas reales del sensor */

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del DHT11.
 */
typedef enum {
	DHT11_OK = 0,				/**< Operación exitosa */
	DHT11_ERROR = 1,			/**< Error en la operación */
	DHT11_TIMEOUT = 2,			/**< Timeout en la operación */
	DHT11_NOT_INITIALIZED = 3,	/**< Módulo no inicializado */
	DHT11_INVALID_PARAM = 4,	/**< Parámetro inválido */
	DHT11_CHECKSUM_ERROR = 5	/**< Error en el checksum */
}DHT11_Status_t;

/**
 * @brief Estructura para almacenar los datos de temperatura y humedad.
 */
typedef struct
{
	float humedad;		/**< Humedad relativa (%RH) */
	float temperatura;	/**< Temperatura en grados Celsius (°C) */
} DHT11_Data_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el sensor DHT11.
 *
 * @note  El timer debe contar a 1 MHz y tener el Counter Period (ARR) al máximo:
 *        0xFFFF en timers de 16 bits o 0xFFFFFFFF en timers de 32 bits. Las diferencias
 *        de tiempo se calculan en 16 bits, así que cualquier otro ARR rompe el manejo
 *        del desbordamiento.
 *
 * @param[in] htim     Puntero al handle del timer utilizado para retardos en microsegundos (1 MHz).
 * @param[in] GPIOx    Puerto GPIO del pin de datos del DHT11.
 * @param[in] GPIO_PIN Pin GPIO del pin de datos del DHT11.
 * @return DHT11_Status_t
 *         - DHT11_OK            si la inicialización fue exitosa.
 *         - DHT11_INVALID_PARAM si @p htim, su instancia o @p GPIOx es NULL, o @p GPIO_PIN es 0.
 *         - DHT11_ERROR         si no se pudo arrancar el timer.
 */
DHT11_Status_t DHT11_Init(TIM_HandleTypeDef* htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN);

/**
 * @brief Desinicializa el sensor DHT11, liberando recursos y marcando el módulo como no inicializado.
 *
 * @return DHT11_Status_t Siempre retorna DHT11_OK.
 */
DHT11_Status_t DHT11_DeInit(void);

/**
 * @brief Lee los valores de temperatura y humedad del sensor DHT11.
 *
 * @param[out] environment Puntero a una estructura `DHT11_Data_t` donde se almacenarán los valores leídos.
 * @return DHT11_Status_t
 *         - DHT11_OK              si la lectura fue exitosa.
 *         - DHT11_NOT_INITIALIZED si no se llamó a DHT11_Init().
 *         - DHT11_INVALID_PARAM   si @p environment es NULL.
 *         - DHT11_TIMEOUT         si el sensor no responde a tiempo.
 *         - DHT11_CHECKSUM_ERROR  si la trama recibida está corrupta.
 *         - DHT11_ERROR           si la respuesta del sensor es inválida.
 *
 * @note Si no han pasado DHT11_MIN_INTERVAL_MS desde la última lectura válida,
 *       no se consulta al sensor: se devuelve la medición guardada con DHT11_OK.
 */
DHT11_Status_t DHT11_Get(DHT11_Data_t *environment);

#ifdef __cplusplus
}
#endif

#endif /* DHT11_H_ */
