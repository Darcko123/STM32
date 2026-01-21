/**
 * @file DHT11.c
 * @brief Funciones para el control del módulo DHT11 mediante Timer y comunicación parasita en STM32.
 * @author Daniel Ruiz
 * @date Ene 20, 2026
 * @version 1.0.0
 */

#include "DHT11.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static TIM_HandleTypeDef*	DHT11_tim = NULL;	/**< Manejador de la interfaz Timer utilizado para retardos en microsegundos */
static GPIO_TypeDef*		DHT11_Port = NULL;	/**< Puerto GPIO del pin de datos del DHT11 */
static uint16_t				DHT11_Pin = 0;		/**< Pin GPIO del pin de datos del DHT11 */

static uint8_t humedad_entero, humedad_decimal, temp_entero, temp_decimal, checksum;

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Genera un retardo preciso en microsegundos utilizando el timer configurado.
 * 
 * @param time Tiempo en microsegundos a esperar.
 */
static void delayUs(uint16_t time)
{
	uint32_t start = __HAL_TIM_GET_COUNTER(DHT11_tim);

	// Esperar hasta que el contador avance la cantidad especificada
	// Se utiliza aritmética modular para manejar correctamente el desbordamiento del timer
	while((__HAL_TIM_GET_COUNTER(DHT11_tim) - start) < time);
}

/**
 * @brief Configura el pin del DHT11 como salida push-pull.
 */
void Set_DHT11Pin_Output (void)
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
void Set_DHT11Pin_Input(void)
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
static DHT11_Status_t DHT11_Check_Response(void)
{   
	uint32_t start = 0;

	// Esperar señal de bajo inicial (sensor toma control del bus)
	start = __HAL_TIM_GET_COUNTER(DHT11_tim);
	while(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))		// Esperar mientras esté ALTO
	{
		if((__HAL_TIM_GET_COUNTER(DHT11_tim) - start) > DHT11_RESPONSE_TIMEOUT_US)
			return DHT11_TIMEOUT;
	}

	// Verificar primer pulso bajo (~80μs)
	if(!(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin)))
	{
		// Esperar transición a alto (~80μs)
		start = __HAL_TIM_GET_COUNTER(DHT11_tim);
		while(!HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))
		{
			if((__HAL_TIM_GET_COUNTER(DHT11_tim) - start) > DHT11_RESPONSE_TIMEOUT_US)
				return DHT11_TIMEOUT;
		}

		// Esperar segundo pulso bajo (~80μs)
		start = __HAL_TIM_GET_COUNTER(DHT11_tim);
		while(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))
		{
			if((__HAL_TIM_GET_COUNTER(DHT11_tim) - start) > DHT11_RESPONSE_TIMEOUT_US)
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
static DHT11_Status_t DHT11_Read_Byte(uint8_t* byte)
{
	uint8_t i = 0;
	uint32_t start;

	if (byte == NULL)
		return DHT11_ERROR;

	for (uint8_t j = 0; j < 8; j++)
	{
		// Esperar inicio del bit (transición a alto)
		start = __HAL_TIM_GET_COUNTER(DHT11_tim);
		while(!(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin)))
		{
			if((__HAL_TIM_GET_COUNTER(DHT11_tim) - start) > DHT11_BIT_TIMEOUT_US)
				return DHT11_TIMEOUT;
		}

		// Esperar tiempo crítico para diferenciar 0 de 1
		delayUs(DHT11_BIT_READ_DELAY_US);

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
		start = __HAL_TIM_GET_COUNTER(DHT11_tim);
		while(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))
		{
			if((__HAL_TIM_GET_COUNTER(DHT11_tim) - start) > DHT11_BIT_TIMEOUT_US)
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
static DHT11_Status_t DHT11_Start_Communication(void)
{
	Set_DHT11Pin_Output();							// Configurar el pin como salida
	HAL_GPIO_WritePin(DHT11_Port, DHT11_Pin, 0);	// Pin bajo
	delayUs(DHT11_START_SIGNAL_US);					// Esperar por 18 ms		
	HAL_GPIO_WritePin(DHT11_Port, DHT11_Pin, 1);	// Pin alto
	delayUs(DHT11_START_WAIT_US);					// Esperar 20 us
	Set_DHT11Pin_Input();							// Configurar el pin como entrada

	delayUs(2);		// Pequeña espera para estabilización del pin

	return DHT11_OK;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el sensor DHT11 configurando timer y GPIO.
 * 
 * @param htim Puntero al manejador del timer para retardos en microsegundos.
 * @param GPIOx Puerto GPIO del pin de datos del DHT11.
 * @param GPIO_PIN Pin GPIO del pin de datos del DHT11.
 * @return DHT11_Status_t Estado de la operación
 */
DHT11_Status_t DHT11_Init(TIM_HandleTypeDef* htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN)
{
	// Validar parámetros de entrada
	if (htim == NULL || GPIOx == NULL)
	{
		return DHT11_ERROR;
	}

	// Asegurar que el timer esté en ejecución
	if (!(htim->Instance->CR1 & TIM_CR1_CEN))
	{
		HAL_TIM_Base_Start(htim);
	}

	// Almacenar configuración para uso en funciones posteriores
	DHT11_tim 	= htim;
	DHT11_Port 	= GPIOx;
	DHT11_Pin	= GPIO_PIN;

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
 * @param values Puntero a estructura donde se almacenarán los valores leídos.
 * @return DHT11_Status_t Estado de la operación
 */
DHT11_Status_t DHT11_Read_Data(DHT11_Handles_t* values)
{
	DHT11_Status_t status;

	// Verificar inicialización previa
	if (DHT11_tim == NULL || DHT11_Port == NULL)
	{
		return DHT11_NOT_INITIALIZED;
	}

	// Validar puntero de salida
	if (values == NULL)
	{
		return DHT11_ERROR;
	}

	// --- SECUENCIA DE COMUNICACIÓN ---

	// 1. Iniciar comunicación
	status = DHT11_Start_Communication();
	if (status != DHT11_OK)
		return status;

	// 2. Verificar respuesta del sensor
	status = DHT11_Check_Response();
	if (status != DHT11_OK)
		return status;

	// 3. Leer los 5 bytes de datos
	status = DHT11_Read_Byte(&humedad_entero);
	if (status != DHT11_OK)
		return status;

	status = DHT11_Read_Byte(&humedad_decimal);
	if (status != DHT11_OK)
		return status;

	status = DHT11_Read_Byte(&temp_entero);
	if (status != DHT11_OK)
		return status;

	status = DHT11_Read_Byte(&temp_decimal);
	if (status != DHT11_OK)
		return status;

	status = DHT11_Read_Byte(&checksum);
	if (status != DHT11_OK)
		return status;

	// 4. Verificar integridad de datos mediante checksum
	if ((uint8_t)(humedad_entero + humedad_decimal + temp_entero + temp_decimal) != checksum)
		return DHT11_CHECKSUM_ERROR;

	// 5. Convertir bytes a valores flotantes
	values->Humidity = (float)humedad_entero + ((float)humedad_decimal / 10.0f);
	values->Temperature = (float)temp_entero + ((float)temp_decimal / 10.0f);

	return DHT11_OK;
}
