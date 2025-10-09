/*
 * HC05.h
 *
 * @brief Implementación de la librería para el módulo HC-05(Bluetooth) utilizando UART en STM32.
 * 
 * @author Daniel Ruiz
 * @date Sep 24, 2025
 * @version 1.0
 */

#ifndef INC_HC05_H_
#define INC_HC05_H_

#include "stm32f1xx_hal.h"
#include "stdbool.h"

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

// Enumeración para estados de retorno
typedef enum {
    HC05_OK = 0,        /** Operación exitosa */
    HC05_ERROR = 1,     /** Error en la operación */
    HC05_TIMEOUT = 2    /** Timeout en la operación */
}HC05_Status_t;

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
HC05_Status_t HC05_Init(UART_HandleTypeDef *huart, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/**
 * @brief Verifica si el módulo responde a comandos AT
 * 
 * @return HC05_Status_t HC05_OK si responde correctamente
 */
HC05_Status_t HC05_Test(void);

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
 * @brief Establece el nombre del módulo HC-05
 * 
 * @param[in] name Nombre del módulo (máximo 20 caracteres)
 * 
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetName(const char* name);

/**
 * @brief Establece el PIN de emparejamiento
 * 
 * @param[in] pin PIN de 4 dígitos
 * 
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetPin(const char* pin);

/**
 * @brief Configura el baudrate para modo de datos
 * 
 * @param[in] baudrate Velocidad de transmisión deseada
 * 
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_SetDataBaudrate(uint32_t baudrate);

/**
 * @brief Reinicia el módulo HC-05
 * 
 * @return HC05_Status_t Estado de la operación
 */
HC05_Status_t HC05_Reset(void);

/**
 * @brief Cambia al modo de datos (comunicación normal)
 * 
 * @return HC05_Status_t Estado de la operación
 * 
 * @note Después de llamar esta función, el módulo estará listo para 
 *       comunicación Bluetooth normal
 */
HC05_Status_t HC05_EnterDataMode(void);

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
