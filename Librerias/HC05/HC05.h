/*
 * HC05.h
 *
 * @brief Implementación de la librería para el módulo HC-05(Bluetooth) utilizando UART en STM32.
 * 
 * @author Daniel Ruiz
 * @date Sep 24, 2025
 * @version 1.3
 */

#ifndef INC_HC05_H_
#define INC_HC05_H_

#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include "stdlib.h"

// Definiciones constantes
#define HC05_TIMEOUT_MS     2000        /** Timeout para comandos AT en ms */
#define HC05_BUFFER_SIZE    64          /** Tamaño de buffer para comandos AT */
#define HC05_AT_BAUDRATE    38400       /** Baudrate para comando AT */
#define HC05_DATA_BAUDRATE  115200      /** Baudrate para modo datos por defecto */

// Enumeración para roles del HC-05
typedef enum {
    HC05_ROLE_SLAVE = 0,        /** Modo esclavo */
    HC05_ROLE_MASTER = 1,       /** Modo maestro */
    HC05_ROLE_SLAVE_LOOP = 2    /** Modo esclavo en bucle */
}HC05_ROLE_t;

typedef enum {
    HC05_CONNECT_MODE_FIXED = 0,    /** Conectar a dirección fija */
    HC05_CONNECT_MODE_ANY = 1,      /** Conectar a cualquier dirección */
    HC05_CONNECT_MODE_SLAVE = 2     /** Modo esclavo */
}HC05_Connected_Mode_t;

// Enumeración para estados de retorno
typedef enum {
    HC05_OK = 0,        /** Operación exitosa */
    HC05_ERROR = 1,     /** Error en la operación */
    HC05_TIMEOUT = 2    /** Timeout en la operación */
}HC05_Status_t;

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
                        GPIO_TypeDef* Power_GPIOx, uint16_t Power_Pin);

/**
 * @brief Verifica si el módulo responde a comandos AT
 * 
 * @return HC05_Status_t HC05_OK si responde correctamente
 */
HC05_Status_t HC05_Test(void);

/**
 * @brief Reinicia el módulo HC-05
 *
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_Reset(void);

/**
 * @brief Dejar el módulo en modo de factory reset
 *
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_FactoryReset(void);

/**
 * @brief Obtener la dirección del módulo HC-05
 *
 * @param[out] address Buffer para almacenar la dirección (mínimo 13 bytes)
 */
HC05_Status_t HC05_GetAddress(char* address);

/**
 * @brief Establece el nombre del módulo HC-05
 *
 * @param[in] name Nombre del módulo (máximo 20 caracteres)
 *
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetName(const char* name);

/**
 * @brief Obtiene el nombre actual del módulo HC-05
 *
 * @param[out] name Buffer para almacenar el nombre (mínimo 21 bytes)
 */
HC05_Status_t HC05_GetName(char* name);

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
HC05_Status_t HC05_SetRole(HC05_ROLE_t role);

/**
 * @brief Obtiene el rol actual del módulo HC-05
 *
 * @param[out] role Puntero para almacenar el rol actual
 *          - 0: Slave
 *          - 1: Master
 *          - 2: Slave-loop
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_GetRole(HC05_ROLE_t* role);

/**
 * @brief Establece el PIN de emparejamiento
 * 
 * @param[in] pin PIN de 4 dígitos
 * 
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetPin(const char* pin);

/**
 * @brief Obtiene el PIN actual del módulo HC-05
 *
 * @param[out] pin Buffer para almacenar el PIN (mínimo 5 bytes)
 */
HC05_Status_t HC05_GetPin(char* pin);

/**
 * @brief Borra la lista de todos los dispositivos emparejados
 */
HC05_Status_t HC05_ClearPairedDevices(void);

/**
 * @brief Configura el modo de conexión del módulo HC-05
 *
 * @param[in] mode Modo de conexión (0: Conectar a dirección fija, 1: Conectar a cualquier dirección, 2: Slave Mode)
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetConnectMode(uint8_t mode);

/**
 * @brief Obtiene el modo de conexión actual del módulo HC-05
 *
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_GetConnectMode(HC05_Connected_Mode_t* mode);

/**
 * @brief Establece la dirección fija a la que se conectará el módulo HC-05
 *
 * @param address Dirección Bluetooth a conectar (formato: "XX:XX:XX:XX:XX:XX")
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetFixedAddressToConnect(const char* address);

/**
 * @brief Obtiene la dirección fija a la que se conecta el módulo HC-05
 *
 * @param address
 * @return HC05_Status_t
 */
HC05_Status_t HC05_GetFixedAddressConnected(char* address);

/**
 * @brief Configura el baudrate para modo de datos
 * 
 * @param[in] baudrate Velocidad de transmisión deseada
 * 
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetDataBaudrate(uint32_t baudrate);

/**
 * @brief Cambia al modo de datos (comunicación normal)
 * 
 * @return HC05_Status_t Estado de la operación
 * 
 * @note Después de llamar esta función, el módulo estará listo para 
 *       comunicación Bluetooth normal
 */
HC05_Status_t HC05_EnterDataMode(uint32_t baudrate);

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
HC05_Status_t HC05_SendData(uint8_t* data, uint16_t size);

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
HC05_Status_t HC05_ReceiveData(uint8_t* data, uint16_t size, uint32_t timeout);

#endif /* INC_HC05_H_ */
