/*
 * HC05.h
 *
 * @brief Implementación de la librería para el módulo HC-05(Bluetooth) utilizando UART en STM32.
 * 
 * @author Daniel Ruiz
 * @date Sep 24, 2025
 * @version 1.0
 */

#include "HC05.h"

#include "stdio.h"
#include "string.h"

// Variables estáticas
static UART_HandleTypeDef* HC05_HUART = NULL;     /**< Manejador de la interfaz UART utilizado para comunicarse con el módulo */
static GPIO_TypeDef* HC05_En_Port = NULL;
static uint16_t HC05_En_Pin = 0;
static bool HC05_AT_Mode = false;

// Funciones privadas
static HC05_Status_t HC05_SendCommand(const char* command, char* response, uint16_t response_size);
static HC05_Status_t HC05_EnterATMode(void);
static HC05_Status_t HC05_ExitATMode(void);

/**
 * @brief Inicializa el módulo HC-05
 * 
 * @param[in] huart Puntero al manejador de la interfaz UART
 * @param[in] en_pin Pin para controlar el modo AT (opcional, puede ser NULL)
 * 
 * @return HC05_Status_t Estado de la inicialización
 * 
 * @note La función configura la UART para modo AT y verifica la comunicación
 */
HC05_Status_t HC05_Init(UART_HandleTypeDef *huart, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	if(huart == NULL)
	{
		return HC05_ERROR;
	}

	HC05_HUART = huart;
	HC05_En_Port = GPIOx;
	HC05_En_Pin = GPIO_Pin;

	//Configurar UART para modo AT
	HC05_HUART->Init.BaudRate = 115200;
	HC05_HUART->Init.WordLength = UART_WORDLENGTH_8B;
  	HC05_HUART->Init.StopBits = UART_STOPBITS_1;
	HC05_HUART->Init.Parity = UART_PARITY_NONE;
 	HC05_HUART->Init.Mode = UART_MODE_TX_RX;
  	HC05_HUART->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  	HC05_HUART->Init.OverSampling = UART_OVERSAMPLING_16;

	if(HAL_UART_Init(HC05_HUART) != HAL_OK)
	{
		return HC05_ERROR;
	}

	//Entrar en modo AT
	if(HC05_EnterATMode() != HC05_OK)
	{
		return HC05_ERROR;
	}

	if(HC05_Test() != HC05_OK)
	{
		return HC05_ERROR;
	}

	return HC05_OK;
}

/**
 * @brief Verifica si el módulo responde a comandos AT
 * 
 * @return HC05_Status_t HC05_OK si responde correctamente
 */
HC05_Status_t HC05_Test(void)
{
	char response[16];

	if(HC05_SendCommand("AT", response, sizeof(response)) == HC05_OK)
	{
		if(strstr(response, "OK") != NULL)
		{
			return HC05_OK;
		}
	}

	return HC05_ERROR;
}

/**
 * @brief Configura el rol a desempeñar por el módulo HC-05
 * 
 * @param[in] role Rol a desempeñar por el módulo (HC05_Role_t)
 *          - 0: Slave
 *          - 1: Master
 *          - 2: Slave-loop
 * 
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetRole(HC05_ROLE_t role)
{
	char command[20];
	char response[16];

	if(role > HC05_ROLE_SLAVE_LOOP)
	{
		return HC05_ERROR;
	}

	snprintf(command, sizeof(command), "AT+ROLE=%d", (int)role);

	if(HC05_SendCommand(command, response, sizeof(response)) == HC05_OK)
	{
		if(strstr(response, "OK") != NULL)
		{
			return HC05_OK;
		}
	}

	return HC05_ERROR;
}

/**
 * @brief Establece el nombre del módulo HC-05
 * 
 * @param[in] name Nombre del módulo (máximo 20 caracteres)
 * 
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetName(const char* name)
{
	char command[40];
	char response[16];

	if(name == NULL || strlen(name) > 20)
	{
		return HC05_ERROR;
	}

	snprintf(command, sizeof(command), "AT+NAME=%s", name);

	if(HC05_SendCommand(command, response, sizeof(response)) == HC05_OK)
	{
		if(strstr(response, "OK") != NULL)
		{
			return HC05_OK;
		}
	}

	return HC05_ERROR;
}

/**
 * @brief Establece el PIN de emparejamiento
 * 
 * @param[in] pin PIN de 4 dígitos
 * 
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetPin(const char* pin)
{
	char command[40];
	char response[16];

	if(pin == NULL || strlen(pin) != 4)
	{
		return HC05_ERROR;
	}

	snprintf(command, sizeof(command), "AT+PSWD=%s", pin);

	if(HC05_SendCommand(command, response, sizeof(response)) == HC05_OK)
	{
		if(strstr(response, "OK") != NULL)
		{
			return HC05_OK;
		}
	}

	return HC05_ERROR;

}

/**
 * @brief Configura el baudrate para modo de datos
 * 
 * @param[in] baudrate Velocidad de transmisión deseada
 * 
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetDataBaudrate(uint32_t baudrate)
{
	char command[30];
	char response[16];

	if(baudrate != 1200 && baudrate != 2400 && baudrate != 4800 &&
	   baudrate != 9600 && baudrate != 19200 && baudrate != 38400 &&
	   baudrate != 57600 && baudrate != 115200)
	{
		return HC05_ERROR;
	}

	snprintf(command, sizeof(command), "AT+UART=%lu,0,0", baudrate);
	
	if(HC05_SendCommand(command, response, sizeof(response)) == HC05_OK)
	{
		if(strstr(response, "OK") != NULL)
		{
			return HC05_OK;
		}
	}

	return HC05_ERROR;
}

/**
 * @brief Reinicia el módulo HC-05
 * 
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_Reset(void)
{
	char response[16];

	if(HC05_SendCommand("AT+RESET", response, sizeof(response)) == HC05_OK)
	{
		HAL_Delay(1000);	//Esperar a que el módulo reinicie
		return HC05_OK;
	}
	return HC05_ERROR;
}

/**
 * @brief Cambia al modo de datos (comunicación normal)
 * 
 * @return HC05_Status_t Estado de la operación
 * 
 * @note Después de llamar esta función, el módulo estará listo para 
 *       comunicación Bluetooth normal
 */
HC05_Status_t HC05_EnterDataMode(void)
{
	if(HC05_ExitATMode() != HC05_OK)
	{
		return HC05_ERROR;
	}

	// Reconfigurar UART para modo de datos (Por defecto 115200)
	HC05_HUART ->Init.BaudRate = HC05_DATA_BAUDRATE;
	if(HAL_UART_Init(HC05_HUART) != HAL_OK)
	{
		return HC05_ERROR;
	}

	HC05_AT_Mode = false;
	return HC05_OK;
}

/**
 * @brief Envía datos a través de Bluetooth
 * 
 * @param[in] data Puntero a los datos a enviar
 * 
 * @return HC05_Status_t Estado de la operación
 * 
 * @note Solo funciona en modo de datos
 */
HC05_Status_t HC05_SendData(uint8_t* data, uint16_t size)
{
    if (HC05_AT_Mode || data == NULL || size == 0)
    {
        return HC05_ERROR;
    }

    if (HAL_UART_Transmit(HC05_HUART, data, size, HC05_TIMEOUT_MS) == HAL_OK)
    {
        return HC05_OK;
    }

    return HC05_ERROR;
}

/**
 * @brief Recibe datos a través de Bluetooth
 * 
 * @param[out] data Buffer para almacenar los datos recibidos
 * @param[in] size Tamaño máximo a recibir
 * @param[in] timeout Timeout en milisegundos
 * 
 * @return HC05_Status_t Estado de la operación
 * 
 * @note Solo funciona en modo de datos
 */
HC05_Status_t HC05_ReceiveData(uint8_t* data, uint16_t size, uint32_t timeout)
{
	if ( HC05_AT_Mode || data == NULL || size == 0)
	{
		return HC05_ERROR;
	}

	HAL_StatusTypeDef status = HAL_UART_Receive(HC05_HUART, data, size, timeout);

	if(status == HAL_OK)
	{
		return HC05_OK;
	}else if(status == HAL_TIMEOUT)
	{
		return HC05_TIMEOUT;
	}

	return HC05_ERROR;
}

// =======================================
// Funciones privadas
// =======================================

/**
 * @brief Envía datos a través de Bluetooth
 * 
 * @param[in] data Puntero a los datos a enviar
 * @param[in] size Tamaño de los datos
 * 
 * @return HC05_Status_t Estado de la operación
 * 
 * @note Solo funciona en modo de datos
 */
HC05_Status_t HC05_SendCommand(const char* command, char* response, uint16_t response_size)
{
	if((command == NULL) || (response == NULL) || (HC05_HUART == NULL))
	{
		return HC05_ERROR;
	}

	char full_command[HC05_BUFFER_SIZE];
	snprintf(full_command, sizeof(full_command), "%s\r\n", command);

	// Limpiar el buffer de respuesta
	memset(response, 0, response_size);

	// Enviar comando
	if(HAL_UART_Transmit(HC05_HUART, (uint8_t*)full_command, strlen(full_command), HC05_TIMEOUT_MS) != HAL_OK)
	{
		return HC05_ERROR;
	}

	// Pequeña pausa para que el módulo procese
	HAL_Delay(100);

	HAL_StatusTypeDef status = HAL_UART_Receive(HC05_HUART, (uint8_t*)response, response_size - 1, HC05_TIMEOUT_MS);

	if(status == HAL_OK)
	{
		return HC05_OK;
	} else if (status == HAL_TIMEOUT)
	{
		return HC05_TIMEOUT;
	}

	return HC05_ERROR;

}


/**
 * @brief Entra en modo AT
 */
static HC05_Status_t HC05_EnterATMode(void)
{
	if(HC05_En_Port != NULL && HC05_En_Pin != 0)
	{
		//Poner el pin en HIGH para entrar en modo AT
		HAL_GPIO_WritePin(HC05_En_Port, HC05_En_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
	}

	HC05_AT_Mode = true;
	return HC05_OK;
}

/**
 * @brief Sale de modo AT
 */
static HC05_Status_t HC05_ExitATMode(void)
{
	if(HC05_En_Port != NULL && HC05_En_Pin != 0)
	{
		//Poner el pin en HIGH para entrar en modo AT
		HAL_GPIO_WritePin(HC05_En_Port, HC05_En_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
	}

	HC05_AT_Mode = false;
	return HC05_OK;
}
