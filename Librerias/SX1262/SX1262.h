/**
 * @file SX1262.h
 * @brief Librería para la gestión de un
 * 
 * Esta librería permite inicializar y controlar
 * 
 * @author Daniel Ruiz
 * @date March 16, 2026
 * @version 1.0
 */

#ifndef SX1262_H
#define SX1262_H

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
// MACROS Y CONSTANTES DE COMANDOS SX1262
// ============================================================================ 
#define SX126X_CMD_SET_SLEEP                  0x84
#define SX126X_CMD_SET_STANDBY                0x80
#define SX126X_CMD_SET_PACKET_TYPE            0x8A
#define SX126X_CMD_SET_RF_FREQUENCY           0x86
#define SX126X_CMD_SET_MODULATION_PARAMS      0x8B
#define SX126X_CMD_SET_PACKET_PARAMS          0x8C
#define SX126X_CMD_SET_TX_PARAMS              0x8E
#define SX126X_CMD_SET_BUFFER_BASE_ADDRESS    0x8F
#define SX126X_CMD_SET_DIO_IRQ_PARAMS         0x08
#define SX126X_CMD_SET_TX                     0x83
#define SX126X_CMD_SET_RX                     0x82
#define SX126X_CMD_CLEAR_IRQ_STATUS           0x02
#define SX126X_CMD_GET_IRQ_STATUS             0x12
#define SX126X_CMD_GET_PACKET_STATUS          0x14
#define SX126X_CMD_WRITE_REGISTER             0x0D
#define SX126X_CMD_READ_REGISTER              0x1D
#define SX126X_CMD_WRITE_BUFFER               0x0E
#define SX126X_CMD_READ_BUFFER                0x1E
#define SX126X_CMD_GET_RX_BUFFER_STATUS       0x13
#define SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL 0x9D
#define SX126X_CMD_SET_REGULATOR_MODE         0x96

// Valores comunes
#define RADIOLIB_SX126X_PACKET_TYPE_LORA      0x01
#define RADIOLIB_SX126X_STANDBY_RC            0x00

// IRQ Flags 
#define SX126X_IRQ_TX_DONE                    (1 << 0)
#define SX126X_IRQ_RX_DONE                    (1 << 1)
#define SX126X_IRQ_PREAMBLE_DETECTED          (1 << 2)
#define SX126X_IRQ_SYNC_WORD_VALID            (1 << 3)
#define SX126X_IRQ_HEADER_VALID               (1 << 4)
#define SX126X_IRQ_HEADER_ERR                 (1 << 5)
#define SX126X_IRQ_CRC_ERR                    (1 << 6)
#define SX126X_IRQ_TIMEOUT                    (1 << 9)

// Timeout
#define SX1262_MAX_BUSY_TIMEOUT 500     /**< milisegundos*/

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================
/**
 * @brief Enumeración para estados de retorno del SX1262.
 */
typedef enum {
	SX1262_OK = 0,				/** Operación exitosa */
	SX1262_ERROR = 1,			/** Error en la operación */
	SX1262_TIMEOUT = 2,			/** Timeout en la operación */
	SX1262_NOT_INITIALIZED = 3	/** Sensor no inicializado */
}SX1262_Status_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa la el módulo SX1262.
 * 
 * @param hspi Puntero al manejador de la interfaz SPI utilizada para comunicarse con el módulo.
 * 
 * @param GPIOx Puerto GPIO del pin de datos del SX1262.
 * @param GPIO_PIN Pin GPIO del pin de datos del SX1262.
 */
SX1262_Status_t SX1262_Init(
	SPI_HandleTypeDef* hspi,
    GPIO_TypeDef*      NSS_Port,
    uint16_t           NSS_Pin,
    GPIO_TypeDef*      BUSY_Port,
    uint16_t           BUSY_Pin,
    GPIO_TypeDef*      DIO1_Port,
    uint16_t           DIO1_Pin,
    GPIO_TypeDef*      RST_Port,
    uint16_t           RST_Pin
);

/**
 * @brief Transmite datos a través del módulo SX1262
 * 
 * @param data 
 * @param length 
 * @return SX1262_Status_t 
 */
SX1262_Status_t SX1262_Transmit(uint8_t* data, uint8_t length);

/**
 * @brief Recibe datos a través del módulo SX1262
 * 
 * @param data 
 * @param length 
 * @param timeout_ms 
 * @return SX1262_Status_t 
 */
SX1262_Status_t SX1262_Receive(uint8_t* data, uint8_t* length, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* SX1262_H */
