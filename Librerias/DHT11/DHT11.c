/**
 * @file DHT11.c
 *
 * @brief Implementación de la librería para el sensor de temperatura y humedad DHT11 utilizando un Timer y protocolo single-wire en STM32.
 *
 * @author Daniel Ruiz
 * @date Ene 20, 2026
 * @version 1.0.0
 */

#include "DHT11.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static TIM_HandleTypeDef*	DHT11_htim			= NULL;	/**< Manejador de la interfaz Timer utilizado para retardos en microsegundos */
static GPIO_TypeDef*		DHT11_Port			= NULL;	/**< Puerto GPIO del pin de datos del DHT11 */
static uint16_t				DHT11_Pin			= 0U;	/**< Pin GPIO del pin de datos del DHT11 */
static uint8_t				DHT11_Initialized	= 0U;	/**< Bandera para verificar si el módulo está inicializado */

static uint8_t humedad_entero, humedad_decimal, temp_entero, temp_decimal, checksum;

static DHT11_Data_t			DHT11_LastData;				/**< Última medición válida */
static uint32_t				DHT11_LastTick		= 0U;	/**< HAL_GetTick() de la última medición válida */
static uint8_t				DHT11_HasData		= 0U;	/**< Indica si DHT11_LastData es válida */

// ============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
// ============================================================================

static void           dht11_DelayUs(uint16_t time);
static void           dht11_SetPinOutput(void);
static void           dht11_SetPinInput(void);
static DHT11_Status_t dht11_CheckResponse(void);
static DHT11_Status_t dht11_ReadByte(uint8_t* byte);
static DHT11_Status_t dht11_StartCommunication(void);

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Genera un retardo preciso en microsegundos utilizando el timer configurado.
 *
 * @param time Tiempo en microsegundos a esperar.
 */
static void dht11_DelayUs(uint16_t time)
{
	uint32_t start = __HAL_TIM_GET_COUNTER(DHT11_htim);

	// Esperar hasta que el contador avance la cantidad especificada
	// La resta se trunca a 16 bits: maneja el desbordamiento con timers de 16 y 32 bits (ARR al máximo)
	while((uint16_t)(__HAL_TIM_GET_COUNTER(DHT11_htim) - start) < time);
}

/**
 * @brief Configura el pin del DHT11 como salida push-pull.
 */
static void dht11_SetPinOutput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DHT11_Port, &GPIO_InitStruct);
}

/**
 * @brief Configura el pin del DHT11 como entrada con pull-up interno.
 */
static void dht11_SetPinInput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT11_Port, &GPIO_InitStruct);
}

/**
 * @brief Verifica la secuencia de respuesta del sensor DHT11.
 *
 * El sensor responde con una señal de bajo durante ~80μs seguida de un alto ~80μs.
 *
 * @return DHT11_Status_t Estado de la verificación
 */
static DHT11_Status_t dht11_CheckResponse(void)
{
	uint32_t start = 0;

	// Esperar señal de bajo inicial (sensor toma control del bus)
	start = __HAL_TIM_GET_COUNTER(DHT11_htim);
	while(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))		// Esperar mientras esté ALTO
	{
		if((uint16_t)(__HAL_TIM_GET_COUNTER(DHT11_htim) - start) > DHT11_RESPONSE_TIMEOUT_US)
			return DHT11_TIMEOUT;
	}

	// Verificar primer pulso bajo (~80μs)
	if(!(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin)))
	{
		// Esperar transición a alto (~80μs)
		start = __HAL_TIM_GET_COUNTER(DHT11_htim);
		while(!HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))
		{
			if((uint16_t)(__HAL_TIM_GET_COUNTER(DHT11_htim) - start) > DHT11_RESPONSE_TIMEOUT_US)
				return DHT11_TIMEOUT;
		}

		// Esperar segundo pulso bajo (~80μs)
		start = __HAL_TIM_GET_COUNTER(DHT11_htim);
		while(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))
		{
			if((uint16_t)(__HAL_TIM_GET_COUNTER(DHT11_htim) - start) > DHT11_RESPONSE_TIMEOUT_US)
				return DHT11_TIMEOUT;
		}

		return DHT11_OK;
	}

	return DHT11_ERROR;
}

/**
 * @brief Lee un byte completo (8 bits) del sensor DHT11.
 *
 * Cada bit comienza con un pulso bajo de ~50μs seguido de:
 * - 26-28μs para bit '0'
 * - 70μs para bit '1'
 *
 * @param byte Puntero donde se almacenará el byte leído.
 * @return DHT11_Status_t Estado de la lectura
 */
static DHT11_Status_t dht11_ReadByte(uint8_t* byte)
{
	uint8_t i = 0;
	uint32_t start;

	if (byte == NULL)
		return DHT11_ERROR;

	for (uint8_t j = 0; j < 8; j++)
	{
		// Esperar inicio del bit (transición a alto)
		start = __HAL_TIM_GET_COUNTER(DHT11_htim);
		while(!(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin)))
		{
			if((uint16_t)(__HAL_TIM_GET_COUNTER(DHT11_htim) - start) > DHT11_BIT_TIMEOUT_US)
				return DHT11_TIMEOUT;
		}

		// Esperar tiempo crítico para diferenciar 0 de 1
		dht11_DelayUs(DHT11_BIT_READ_DELAY_US);

		// Determinar valor del bit según nivel del pin
		if(!(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin)))
		{
			i &= ~(1 << (7 - j));	// Escribe 0
		}
		else
		{
			i |=  (1 << (7 - j));   // Escribe 1
		}

		// Esperar fin del bit (transición a bajo)
		start = __HAL_TIM_GET_COUNTER(DHT11_htim);
		while(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))
		{
			if((uint16_t)(__HAL_TIM_GET_COUNTER(DHT11_htim) - start) > DHT11_BIT_TIMEOUT_US)
				return DHT11_TIMEOUT;
		}
	}

	*byte = i;
	return DHT11_OK;
}

/**
 * @brief Inicia la secuencia de comunicación con el sensor DHT11.
 *
 * La secuencia consiste en:
 * 1. Bajar el pin por 18ms
 * 2. Subir el pin por 20μs
 * 3. Cambiar a modo entrada
 *
 * @return DHT11_Status_t Estado de la operación
 */
static DHT11_Status_t dht11_StartCommunication(void)
{
	dht11_SetPinOutput();									// Configurar el pin como salida
	HAL_GPIO_WritePin(DHT11_Port, DHT11_Pin, GPIO_PIN_RESET);	// Pin bajo
	dht11_DelayUs(DHT11_START_SIGNAL_US);					// Esperar por 18 ms
	HAL_GPIO_WritePin(DHT11_Port, DHT11_Pin, GPIO_PIN_SET);	// Pin alto
	dht11_DelayUs(DHT11_START_WAIT_US);						// Esperar 20 us
	dht11_SetPinInput();									// Configurar el pin como entrada

	dht11_DelayUs(2);		// Pequeña espera para estabilización del pin

	return DHT11_OK;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el sensor DHT11 configurando timer y GPIO.
 *
 * @param[in] htim     Puntero al handle del timer para retardos en microsegundos.
 * @param[in] GPIOx    Puerto GPIO del pin de datos del DHT11.
 * @param[in] GPIO_PIN Pin GPIO del pin de datos del DHT11.
 * @return DHT11_Status_t
 *         - DHT11_OK            si la inicialización fue exitosa.
 *         - DHT11_INVALID_PARAM si @p htim, su instancia o @p GPIOx es NULL, o @p GPIO_PIN es 0.
 *         - DHT11_ERROR         si no se pudo arrancar el timer.
 *
 * @note Si el timer no está en ejecución, esta función lo arranca con HAL_TIM_Base_Start().
 */
DHT11_Status_t DHT11_Init(TIM_HandleTypeDef* htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN)
{
	// Validar parámetros de entrada
	if (htim == NULL || htim->Instance == NULL || GPIOx == NULL || GPIO_PIN == 0U)
	{
		return DHT11_INVALID_PARAM;
	}

	// Asegurar que el timer esté en ejecución
	if (!(htim->Instance->CR1 & TIM_CR1_CEN))
	{
		if (HAL_TIM_Base_Start(htim) != HAL_OK)
		{
			return DHT11_ERROR;
		}
	}

	// Almacenar configuración para uso en funciones posteriores
	DHT11_htim	= htim;
	DHT11_Port	= GPIOx;
	DHT11_Pin	= GPIO_PIN;

	DHT11_HasData = 0U;		// Descartar mediciones de una configuración anterior
	DHT11_Initialized = 1U;

	return DHT11_OK;
}

/**
 * @brief Desinicializa el sensor DHT11, liberando recursos y marcando el módulo como no inicializado.
 *
 * @return DHT11_Status_t Siempre retorna DHT11_OK.
 *
 * @note El timer no se detiene: puede estar compartido con otros módulos.
 */
DHT11_Status_t DHT11_DeInit(void)
{
	DHT11_htim			= NULL;	// Limpia el handler del timer
	DHT11_Port			= NULL;	// Limpia el puerto GPIO
	DHT11_Pin			= 0U;	// Limpia el pin GPIO
	DHT11_Initialized	= 0U;	// Marca el módulo como no inicializado

	return DHT11_OK;
}

/**
 * @brief Lee temperatura y humedad del sensor DHT11.
 *
 * Realiza la secuencia completa de comunicación:
 * 1. Inicio de comunicación
 * 2. Verificación de respuesta
 * 3. Lectura de 5 bytes de datos
 * 4. Validación de checksum
 * 5. Conversión a valores flotantes
 *
 * @param[out] environment Puntero a estructura donde se almacenarán los valores leídos.
 * @return DHT11_Status_t
 *         - DHT11_OK              si la lectura fue exitosa.
 *         - DHT11_NOT_INITIALIZED si no se llamó a DHT11_Init().
 *         - DHT11_INVALID_PARAM   si @p environment es NULL.
 *         - DHT11_TIMEOUT         si el sensor no responde a tiempo.
 *         - DHT11_CHECKSUM_ERROR  si la trama recibida está corrupta.
 *         - DHT11_ERROR           si la respuesta del sensor es inválida.
 */
DHT11_Status_t DHT11_Get(DHT11_Data_t *environment)
{
	DHT11_Status_t status;

	// Verificar inicialización previa
	if (DHT11_Initialized != 1U)
	{
		return DHT11_NOT_INITIALIZED;
	}

	// Validar puntero de salida
	if (environment == NULL)
	{
		return DHT11_INVALID_PARAM;
	}

	// El DHT11 necesita ~2 s entre lecturas: antes de eso, devolver la última medición válida
	if (DHT11_HasData && (HAL_GetTick() - DHT11_LastTick) < DHT11_MIN_INTERVAL_MS)
	{
		*environment = DHT11_LastData;
		return DHT11_OK;
	}

	// --- SECUENCIA DE COMUNICACIÓN ---

	// 1. Iniciar comunicación
	status = dht11_StartCommunication();
	if (status != DHT11_OK)
		return status;

	// 2. Verificar respuesta del sensor
	status = dht11_CheckResponse();
	if (status != DHT11_OK)
		return status;

	// 3. Leer los 5 bytes de datos
	status = dht11_ReadByte(&humedad_entero);
	if (status != DHT11_OK)
		return status;

	status = dht11_ReadByte(&humedad_decimal);
	if (status != DHT11_OK)
		return status;

	status = dht11_ReadByte(&temp_entero);
	if (status != DHT11_OK)
		return status;

	status = dht11_ReadByte(&temp_decimal);
	if (status != DHT11_OK)
		return status;

	status = dht11_ReadByte(&checksum);
	if (status != DHT11_OK)
		return status;

	// 4. Verificar integridad de datos mediante checksum
	if ((uint8_t)(humedad_entero + humedad_decimal + temp_entero + temp_decimal) != checksum)
		return DHT11_CHECKSUM_ERROR;

	// 5. Convertir bytes a valores flotantes
	environment->humedad = (float)humedad_entero + ((float)humedad_decimal / 10.0f);
	environment->temperatura = (float)temp_entero + ((float)temp_decimal / 10.0f);

	// 6. Guardar medición y marca de tiempo para respetar el intervalo mínimo
	DHT11_LastData	= *environment;
	DHT11_LastTick	= HAL_GetTick();
	DHT11_HasData	= 1U;

	return DHT11_OK;
}
