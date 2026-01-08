/*
 * HC05.h
 *
 * @brief Implementación de la librería para el módulo HC-05(Bluetooth) utilizando UART en STM32.
 *
 * @author Daniel Ruiz
 * @date Sep 24, 2025
 * @version 1.3
 */

#include "HC05.h"

#include "stdio.h"
#include "string.h"

// Variables estáticas
static UART_HandleTypeDef* HC05_HUART = NULL;	/**< Manejador de la interfaz UART utilizado para comunicarse con el módulo */
static GPIO_TypeDef* HC05_En_Port = NULL;
static uint16_t HC05_En_Pin = 0;
static GPIO_TypeDef* HC05_Power_Port = NULL;  // Para controlar alimentación
static uint16_t HC05_Power_Pin = 0;
static bool HC05_AT_Mode = false;

// Funciones privadas
static HC05_Status_t HC05_SendCommand(const char* command, char* response, uint16_t response_size);
static HC05_Status_t HC05_EnterATMode(void);
static HC05_Status_t HC05_ExitATMode(void);
static void HC05_FlushUART(void);

/**
 * @brief Inicializa el módulo HC-05
 *
 * @param[in] huart Puntero al manejador UART
 * @param[in] GPIOx Puerto GPIO del pin EN
 * @param[in] GPIO_Pin Pin EN del HC-05
 * @param[in] Power_GPIOx Puerto GPIO para control de alimentación (NULL si no disponible)
 * @param[in] Power_Pin Pin para control de alimentación (0 si no disponible)
 *
 * @return HC05_Status_t Estado de la inicialización
 * @note La función configura la UART para modo AT y verifica la comunicación
 */
HC05_Status_t HC05_Init(UART_HandleTypeDef *huart,
                        GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,
                        GPIO_TypeDef* Power_GPIOx, uint16_t Power_Pin)
{
	if(huart == NULL)
	{
		return HC05_ERROR;
	}

	HC05_HUART = huart;
	HC05_En_Port = GPIOx;
	HC05_En_Pin = GPIO_Pin;
	HC05_Power_Port = Power_GPIOx;
	HC05_Power_Pin = Power_Pin;

	// Verificar si tenemos control de alimentación
	bool has_power_control = (HC05_Power_Port != NULL && HC05_Power_Pin != 0);

	if(HC05_En_Port != NULL && HC05_En_Pin != 0)
	{
		// PASO 1: Poner EN en HIGH ANTES de encender
		HC05_EnterATMode();

		if(has_power_control)
		{
			// PASO 2: Apagar el módulo (si estaba encendido)
			HAL_GPIO_WritePin(HC05_Power_Port, HC05_Power_Pin, GPIO_PIN_RESET);
			HAL_Delay(500);

			// PASO 3: Encender el módulo con EN ya en HIGH
			HAL_GPIO_WritePin(HC05_Power_Port, HC05_Power_Pin, GPIO_PIN_SET);
			HAL_Delay(1500);  // Esperar a que entre en modo AT
		}
		else
		{
			HAL_Delay(2000);  // Dar tiempo al usuario para reconectar VCC
		}
	}

	// Intentar con diferentes baudrates comunes para modo AT
	uint32_t at_baudrates[] = {HC05_AT_BAUDRATE, 9600};	// 38400 es más común en muchos HC-05

	for(int baud_idx = 0; baud_idx < 2; baud_idx++)
	{
		// Configurar UART para modo AT
		HC05_HUART->Init.BaudRate = at_baudrates[baud_idx];
		HC05_HUART->Init.WordLength = UART_WORDLENGTH_8B;
		HC05_HUART->Init.StopBits = UART_STOPBITS_1;
		HC05_HUART->Init.Parity = UART_PARITY_NONE;
		HC05_HUART->Init.Mode = UART_MODE_TX_RX;
		HC05_HUART->Init.HwFlowCtl = UART_HWCONTROL_NONE;
		HC05_HUART->Init.OverSampling = UART_OVERSAMPLING_16;

		if(HAL_UART_Init(HC05_HUART) != HAL_OK)
		{
			continue;  // Intentar con siguiente baudrate
		}

		// Limpiar cualquier basura en el buffer después de reconfigurar
		HAL_Delay(100);
		HC05_FlushUART();

		HC05_AT_Mode = true;

		// Intentar comunicación varias veces con este baudrate
		for(int i = 0; i < 5; i++)  // 5 intentos
		{
			HAL_Delay(300);
			if(HC05_Test() == HC05_OK)
			{
				return HC05_OK;
			}
		}
	}

	return HC05_ERROR;
}

/**
 * @brief Verifica si el módulo responde a comandos AT
 *
 * @return HC05_Status_t HC05_OK si responde correctamente
 */
HC05_Status_t HC05_Test(void)
{
    char response[64];
    memset(response, 0, sizeof(response));

    HC05_Status_t result = HC05_SendCommand("AT", response, sizeof(response));

    // Debug - puedes habilitar esto temporalmente
    // HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), 1000);

    if(result == HC05_OK)
    {
        // Verificar "OK" ignorando espacios, \r y \n
        char* ok_ptr = strstr(response, "OK");
        if(ok_ptr != NULL)
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
	char response[32];

	if(HC05_SendCommand("AT+RESET", response, sizeof(response)) == HC05_OK)
	{
		HAL_Delay(1000);	//Esperar a que el módulo reinicie
		return HC05_OK;
	}
	return HC05_ERROR;
}

/**
 * @brief Dejar el módulo en modo de factory reset
 *
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_FactoryReset(void)
{
	char response[32];

	if(HC05_SendCommand("AT+ORGL", response, sizeof(response)) == HC05_OK)
	{
		HAL_Delay(1000);	//Esperar a que el módulo aplique el factory reset
		return HC05_OK;
	}
	return HC05_ERROR;
}

/**
 * @brief Obtener la dirección del módulo HC-05
 *
 * @param[out] address Buffer para almacenar la dirección (mínimo 13 bytes)
 */
HC05_Status_t HC05_GetAddress(char* address)
{
	char response[32];
	memset(response, 0, sizeof(response));

	if(HC05_SendCommand("AT+ADDR?", response, sizeof(response)) == HC05_OK)
    {
        char* addr_ptr = strstr(response, "+ADDR=");
        if(addr_ptr != NULL)
        {
            addr_ptr += 6; // Saltar "+ADDR:"

            // Copiar la dirección completa
            int i = 0;
            while(addr_ptr[i] != '\0' && addr_ptr[i] != '\r' &&
                  addr_ptr[i] != '\n' && addr_ptr[i] != ' ' && i < 20)
            {
                address[i] = addr_ptr[i];
                i++;
            }
            address[i] = '\0';
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
	char response[32];

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
 * @brief Obtiene el nombre actual del módulo HC-05
 *
 * @param[out] name Buffer para almacenar el nombre (mínimo 21 bytes)
 */
HC05_Status_t HC05_GetName(char* name)
{
	char response[64];

	if(name == NULL)
	{
		return HC05_ERROR;
	}

	memset(response, 0, sizeof(response));

	if(HC05_SendCommand("AT+NAME?", response, sizeof(response)) == HC05_OK)
	{
		char* name_ptr = strstr(response, "+NAME:");
		if(name_ptr != NULL)
		{
			// Avanzar después de "+NAME:"
			name_ptr += 6;

			// Copiar hasta encontrar \r o \n
			int i = 0;
			while(i < 20 && name_ptr[i] != '\0' &&
			      name_ptr[i] != '\r' && name_ptr[i] != '\n')
			{
				name[i] = name_ptr[i];
				i++;
			}
			name[i] = '\0';
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
	char response[32];

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
 * @brief Obtiene el rol actual del módulo HC-05
 *
 * @param[out] role Puntero para almacenar el rol actual
 *          - 0: Slave
 *          - 1: Master
 *          - 2: Slave-loop
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_GetRole(HC05_ROLE_t* role)
{
	char response[64];

	if(role == NULL)
	{
		return HC05_ERROR;
	}

	memset(response, 0, sizeof(response));

	if(HC05_SendCommand("AT+ROLE?", response, sizeof(response)) == HC05_OK)
	{
		char* role_ptr = strstr(response, "+ROLE:");
		if(role_ptr != NULL)
		{
			// Avanzar después de "+ROLE:"
			role_ptr += 6;

			// Extraer el número (puede estar seguido de \r\n)
			int role_value = atoi(role_ptr);
			if(role_value >= 0 && role_value <= 2)
			{
				*role = (HC05_ROLE_t)role_value;
				return HC05_OK;
			}
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
    char response[64];  // Aumentado el tamaño

    if(pin == NULL || strlen(pin) != 4)
    {
        return HC05_ERROR;
    }

    // Verificar que todos los caracteres sean dígitos
    for(int i = 0; i < 4; i++)
    {
        if(pin[i] < '0' || pin[i] > '9')
        {
            return HC05_ERROR;
        }
    }

    // Intentar con el formato estándar (con comillas)
    snprintf(command, sizeof(command), "AT+PSWD=\"%s\"", pin);

    // Limpiar el buffer de respuesta
    memset(response, 0, sizeof(response));

    // Enviar comando y esperar respuesta
    HC05_Status_t result = HC05_SendCommand(command, response, sizeof(response));

    if(result == HC05_OK)
    {
        // Buscar "OK" en la respuesta
        if(strstr(response, "OK") != NULL)
        {
            return HC05_OK;
        }
    }

    // Si falló, intentar sin comillas (algunos módulos lo requieren así)
    memset(response, 0, sizeof(response));
    snprintf(command, sizeof(command), "AT+PSWD=%s", pin);

    result = HC05_SendCommand(command, response, sizeof(response));

    if(result == HC05_OK)
    {
        if(strstr(response, "OK") != NULL)
        {
            return HC05_OK;
        }
    }

    return HC05_ERROR;
}

HC05_Status_t HC05_GetPin(char* pin)
{
	char response[64];

	if(pin == NULL)
	{
		return HC05_ERROR;
	}

	memset(response, 0, sizeof(response));

	if(HC05_SendCommand("AT+PSWD?", response, sizeof(response)) == HC05_OK)
    {
        char* pin_ptr = strstr(response, "+PSWD:");
        if(pin_ptr != NULL)
        {
            pin_ptr += 6; // Saltar "+PSWD:"

            // Saltar comillas si existen
            if(*pin_ptr == '"')
                pin_ptr++;

            // Copiar 4 dígitos
            int i = 0;
            while(i < 4 && pin_ptr[i] >= '0' && pin_ptr[i] <= '9')
            {
                pin[i] = pin_ptr[i];
                i++;
            }

            if(i == 4)
            {
                pin[4] = '\0';
                return HC05_OK;
            }
        }
    }

	return HC05_ERROR;
}

/**
 * @brief Borra la lista de todos los dispositivos emparejados
 */
HC05_Status_t HC05_ClearPairedDevices(void)
{
	char response[32];

	if(HC05_SendCommand("AT+RMAAD", response, sizeof(response)) == HC05_OK)
	{
		if(strstr(response, "OK") != NULL)
		{
			HAL_Delay(500);
			return HC05_OK;
		}
	}

	return HC05_ERROR;
}

/**
 * @brief Configura el modo de conexión del módulo HC-05
 *
 * @param[in] mode Modo de conexión (0: Conectar a dirección fija, 1: Conectar a cualquier dirección, 2: Slave Mode)
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetConnectMode(uint8_t mode)
{
	char command[30];
	char response[32];

	if(mode > 2)
	{
		return HC05_ERROR;
	}

	snprintf(command, sizeof(command), "AT+CMODE=%u", mode);

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
 * @brief Obtiene el modo de conexión actual del módulo HC-05
 *
 * @return HC05_Status_t
 */
HC05_Status_t HC05_GetConnectMode(HC05_Connected_Mode_t* mode)
{
	char response[64];

	if(mode == NULL)
	{
		return HC05_ERROR;
	}

	memset(response, 0, sizeof(response));

	if(HC05_SendCommand("AT+CMODE?", response, sizeof(response)) == HC05_OK)
	{
		char* mode_ptr = strstr(response, "+CMODE:");
		if(mode_ptr != NULL)
		{
			// Avanzar después de "+CMODE:"
			mode_ptr += 7;

			// Extraer el número
			int mode_value = atoi(mode_ptr);
			if(mode_value >= 0 && mode_value <= 2)
			{
				*mode = (HC05_Connected_Mode_t)mode_value;
				return HC05_OK;
			}
		}
	}

	return HC05_ERROR;
}

/**
 * @brief Establece la dirección fija a la que se conectará el módulo HC-05
 *
 * @param address Dirección Bluetooth a conectar (formato: "XX:XX:XX:XX:XX:XX")
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetFixedAddressToConnect(const char* address)
{
	char command[40];
	char response[32];

	if(address == NULL || strlen(address) != 17)
	{
		return HC05_ERROR;
	}

	snprintf(command, sizeof(command), "AT+BIND=%s", address);

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
 * @brief Obtiene la dirección fija a la que se conecta el módulo HC-05
 *
 * @param address
 * @return HC05_Status_t
 */
HC05_Status_t HC05_GetFixedAddressConnected(char* address)
{
	char response[64];

	if(address == NULL)
	{
		return HC05_ERROR;
	}

	memset(response, 0, sizeof(response));

	if(HC05_SendCommand("AT+BIND?", response, sizeof(response)) == HC05_OK)
	{
		char* addr_ptr = strstr(response, "+BIND:");
		if(addr_ptr != NULL)
		{
			// Avanzar después de "+BIND:"
			addr_ptr += 6;

			// Copiar la dirección completa
			int i = 0;
			while(addr_ptr[i] != '\0' && addr_ptr[i] != '\r' &&
				  addr_ptr[i] != '\n' && addr_ptr[i] != ' ' && i < 20)
			{
				address[i] = addr_ptr[i];
				i++;
			}
			address[i] = '\0';
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
	char response[32];

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
 * @brief Cambia al modo de datos (comunicación normal)
 *
 * @return HC05_Status_t Estado de la operación
 *
 * @note Después de llamar esta función, el módulo estará listo para
 *       comunicación Bluetooth normal
 */
HC05_Status_t HC05_EnterDataMode(uint32_t baudrate)
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

	// Limpiar buffer después de reconfigurar
	HAL_Delay(100);
	HC05_FlushUART();

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
 * @brief Envía comando AT y recibe respuesta
 *
 * @param[in] command Comando AT a enviar
 * @param[out] response Buffer para almacenar la respuesta
 * @param[in] response_size Tamaño del buffer de respuesta
 *
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SendCommand(const char* command, char* response, uint16_t response_size)
{
	if((command == NULL) || (response == NULL) || (HC05_HUART == NULL))
	{
		return HC05_ERROR;
	}

	char full_command[HC05_BUFFER_SIZE];
	memset(full_command, 0, sizeof(full_command));
	snprintf(full_command, sizeof(full_command), "%s\r\n", command);

	// Limpiar el buffer de respuesta
	memset(response, 0, response_size);

	// Limpiar buffer UART antes de enviar
	HC05_FlushUART();

	// Enviar comando
	if(HAL_UART_Transmit(HC05_HUART, (uint8_t*)full_command, strlen(full_command), HC05_TIMEOUT_MS) != HAL_OK)
	{
		return HC05_ERROR;
	}

    uint16_t idx = 0;
    uint8_t ch;
    uint32_t start = HAL_GetTick();
	uint8_t newline_count = 0;  // Contador de líneas recibidas

	// Leer hasta obtener respuesta completa o timeout
    while ((HAL_GetTick() - start) < 1000)
    {
        if (HAL_UART_Receive(HC05_HUART, &ch, 1, 50) == HAL_OK)
        {
            if (idx < response_size - 1)
            {
                response[idx++] = ch;
                response[idx] = '\0';
            }

			// Contar saltos de línea
			if (ch == '\n')
			{
				newline_count++;
				// Para comandos de consulta (con '?'), esperamos 2 líneas: datos + OK
				// Para comandos de configuración, esperamos 1 línea: OK
				if ((strstr(command, "?") != NULL && newline_count >= 2) ||
				    (strstr(command, "?") == NULL && newline_count >= 1))
				{
					break;
				}
			}
        }
    }

    if (strstr(response, "OK"))
        return HC05_OK;

    if (strstr(response, "ERROR"))
        return HC05_ERROR;

    return HC05_TIMEOUT;
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
		HAL_Delay(500);
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
		//Poner el pin en LOW para salir en modo AT
		HAL_GPIO_WritePin(HC05_En_Port, HC05_En_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
	}

	HC05_AT_Mode = false;
	return HC05_OK;
}

/**
 * @brief Limpia el buffer UART de datos residuales
 */
static void HC05_FlushUART(void)
{
	uint8_t dummy;
	// Leer hasta que no haya más datos o timeout
	while(HAL_UART_Receive(HC05_HUART, &dummy, 1, 10) == HAL_OK)
	{
		// Descartar datos
	}
}
