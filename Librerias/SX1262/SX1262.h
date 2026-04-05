/**
 * @file SX1262.h
 * @brief Librería para la gestión de un módulo LoRa
 * 
 * Esta librería permite inicializar y controlar
 * 
 * @author Daniel Ruiz
 * @date April 3, 2026
 * @version 1.2.0
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
// CONFIGURACIÓN LORA (ESTRUCTURAS)
// ============================================================================

/**
 * @brief Enumeración para valores de ancho de banda (BW) en modulación LoRa.
 */
typedef enum {
	BW_7_8_KHZ    = 0x00,
	BW_10_4_KHZ   = 0x08,
	BW_15_6_KHZ   = 0x01,
	BW_20_8_KHZ   = 0x09,
	BW_31_25_KHZ  = 0x02,
	BW_41_7_KHZ   = 0x0A,
	BW_62_5_KHZ   = 0x03,
	BW_125_KHZ    = 0x04,
	BW_250_KHZ    = 0x05,
	BW_500_KHZ    = 0x06
} lora_signal_bandwidth_t;

// ============================================================================
// MODO DE RED LORA (SYNC WORD)
// ============================================================================
/**
 * @brief Selecciona el modo de red LoRa, que determina el Sync Word enviado al chip.
 *
 * | Modo                   | Registro 0x0740 | Registro 0x0741 | Sync Word lógico |
 * |------------------------|-----------------|-----------------|------------------|
 * | LORA_NETWORK_PRIVATE   |      0x14       |      0x24       |      0x12        |
 * | LORA_NETWORK_PUBLIC    |      0x34       |      0x44       |      0x34        |
 * | LORA_NETWORK_MESHTASTIC|      0x2B       |      0xB4       |      0x2B        |
 *
 * Nota: los valores de registro se calculan con la fórmula de Semtech:
 *   reg[0x0740] = (SW & 0xF0) | 0x04
 *   reg[0x0741] = (SW << 4)   | 0x04
 * Para SW=0x2B: reg[0x0740]=0x2B, reg[0x0741]=(0x2B<<4)|0x04=0xB4.
 *
 * Meshtastic usa el sync word 0x2B (definido en su firmware como LORA_PREAMBLE_LENGTH
 * con sync 0x2B). Permite interoperar directamente con nodos Meshtastic sin
 * modificar ningún otro parámetro de modulación.
 *
 * Si se necesita un sync word completamente personalizado, usar el campo
 * `lora_sync_word` de lora_config_t (distinto de 0 tiene prioridad sobre network_mode).
 */
typedef enum {
    LORA_NETWORK_PRIVATE    = 0,  /**< Sync Word 0x12 — red privada (por defecto LoRa) */
    LORA_NETWORK_PUBLIC     = 1,  /**< Sync Word 0x34 — red pública (LoRaWAN)          */
    LORA_NETWORK_MESHTASTIC = 2,  /**< Sync Word 0x2B — compatible con Meshtastic       */
} lora_network_mode_t;

/**
 * @brief Enumeración para valores de coding rate (CR) en modulación LoRa.
 */
typedef enum {
	CR_4_5 = 0x01,
	CR_4_6 = 0x02,
	CR_4_7 = 0x03,
	CR_4_8 = 0x04
}lora_coding_rate_t;

/**
 * @brief Estructura para almacenar la configuración de modulación y red LoRa.
 *        El campo `config_pending` se establece en true si se han realizado cambios
 *        en la configuración que aún no se han aplicado al chip. Esto permite a las
 *        funciones de transmisión/recepción verificar si es necesario aplicar la configuración
 *        antes de operar.
 */
typedef struct {
	uint32_t frequency;		            // Hz (default: 915000000)
	uint8_t spreading_factor;	        // 5 to 12 (default: 7)
	lora_signal_bandwidth_t bandwidth;  // BW_125_KHZ, BW_250_KHZ, BW_500_KHZ...
	lora_coding_rate_t coding_rate;		// CR_4_5, CR_4_6, CR_4_7, CR_4_8 (default: CR_4_5)
	int8_t tx_power;                    // -9 to 22 dBm (default: 20)
	uint16_t preamble_len;	            // Default: 12
	bool iq_inverted;	                // IQ inversion (default: false/normal)
	lora_network_mode_t network_mode;   // Sync word: LORA_NETWORK_PRIVATE / PUBLIC / MESHTASTIC
	uint8_t lora_sync_word;             // Custom sync word (distinto de 0 tiene prioridad sobre network_mode)
	bool config_pending;	            // true if changes not yet applied
} lora_config_t;

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================
/**
 * @brief Enumeración para estados de retorno del SX1262.
 */
typedef enum {
	SX1262_OK = 0,				/**< Operación exitosa */
	SX1262_ERROR = 1,			/**< Error en la operación */
	SX1262_TIMEOUT = 2,			/**< Timeout en la operación */
	SX1262_NOT_INITIALIZED = 3,	/**< Módulo no inicializado */
	SX1262_RX_BUSY = 4			/**< El módulo está en modo RX, esperando paquete (modo IT) */
}SX1262_Status_t;

// ============================================================================
// BANDERA DE RECEPCIÓN (modo no bloqueante)
// ============================================================================
/**
 * @brief Bandera activada por el ISR cuando DIO1 genera un flanco de subida.
 *
 *        El ISR (vía SX1262_IRQ_Handler) la pone en 1.
 *        El main loop debe ponerla en 0 después de consumirla.
 *        Declarada volatile para forzar lectura directa desde RAM y evitar
 *        optimizaciones del compilador que omitan la actualización del ISR.
 *
 * Patrón de uso:
 * @code
 *   if (SX1262_RxDoneFlag) {
 *       SX1262_RxDoneFlag = 0;
 *       SX1262_GetReceivedPacket(buf, &len);
 *   }
 * @endcode
 */
extern volatile uint8_t SX1262_RxDoneFlag;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el módulo SX1262 con la configuración de pines y parámetros por defecto.
 * 
 * @param hspi Puntero al handle del SPI utilizado para comunicarse con el SX1262.
 * @param nss_port Puerto GPIO del pin NSS (Chip Select).
 * @param nss_pin Número de pin GPIO para NSS.
 * @param busy_port Puerto GPIO del pin BUSY.
 * @param busy_pin Número de pin GPIO para BUSY.
 * @param dio_port Puerto GPIO del pin DIO1 (interrupción).
 * @param dio_pin Número de pin GPIO para DIO1.
 * @param rst_port Puerto GPIO del pin RST (Reset).
 * @param rst_pin Número de pin GPIO para RST.
 * @return SX1262_Status_t Estado de la inicialización (OK, ERROR, etc.)
 */
SX1262_Status_t SX1262_Init(
	SPI_HandleTypeDef* hspi,
    GPIO_TypeDef*      nss_port,
    uint16_t           nss_pin,
    GPIO_TypeDef*      busy_port,
    uint16_t           busy_pin,
    GPIO_TypeDef*      dio_port,
    uint16_t           dio_pin,
    GPIO_TypeDef*      rst_port,
    uint16_t           rst_pin
);

/**
 * @brief Transmite datos a través del módulo SX1262
 * 
 * @param data Puntero al buffer de datos a transmitir
 * @param length Longitud de los datos a transmitir (máximo 255 bytes)
 * @return SX1262_Status_t 
 */
SX1262_Status_t SX1262_Transmit(uint8_t* data, uint8_t length);

/**
 * @brief Recibe datos a través del módulo SX1262 (modo bloqueante)
 * 
 * @param data Puntero al buffer donde se almacenarán los datos recibidos
 * @param length Puntero a una variable donde se almacenará la longitud de los datos recibidos
 * @param timeout_ms Tiempo máximo de espera para recibir datos (en milisegundos). Si es 0, espera indefinidamente.
 * @return SX1262_Status_t 
 */
SX1262_Status_t SX1262_Receive(uint8_t* data, uint8_t* length, uint32_t timeout_ms);

// ----------------------------------------------------------------------------
// Recepción No Bloqueante (basada en interrupciones EXTI en DIO1)
// ----------------------------------------------------------------------------

/**
 * @brief Configura el chip en modo RX continuo y retorna inmediatamente.
 *
 *        El evento de recepción se señaliza mediante SX1262_RxDoneFlag,
 *        que es activada desde el ISR a través de SX1262_IRQ_Handler().
 *
 *        Requiere que DIO1 esté configurado como EXTI flanco de subida
 *        en STM32CubeMX y que HAL_GPIO_EXTI_Callback llame a SX1262_IRQ_Handler().
 *
 * @return SX1262_Status_t SX1262_OK si el chip entró en modo RX,
 *                         SX1262_ERROR si falla SPI,
 *                         SX1262_NOT_INITIALIZED si no se inicializó.
 */
SX1262_Status_t SX1262_StartReceiveIT(void);

/**
 * @brief Lee el payload del paquete recibido.
 *
 *        Debe llamarse SOLO desde el main loop cuando SX1262_RxDoneFlag == 1.
 *        NO llamar desde el ISR.
 *
 *        Lee el registro IRQ del chip, verifica RX_DONE vs TIMEOUT/CRC_ERR,
 *        obtiene el offset y tamaño del paquete con GetRxBufferStatus,
 *        lee el payload con ReadBuffer y limpia el registro IRQ.
 *
 * @param data   Puntero al buffer donde se almacenarán los datos recibidos.
 * @param length Puntero donde se escribirá la longitud del paquete (bytes).
 * @return SX1262_Status_t SX1262_OK si el paquete es válido,
 *                         SX1262_TIMEOUT si el IRQ indica timeout interno del chip,
 *                         SX1262_ERROR si hay CRC, header inválido o fallo SPI.
 */
SX1262_Status_t SX1262_GetReceivedPacket(uint8_t* data, uint8_t* length);

/**
 * @brief Cancela la recepción en curso y regresa el chip al modo Standby RC.
 *
 *        Útil para implementar timeouts de software sin bloquear el CPU:
 *        el main loop puede llamar esta función si SX1262_RxDoneFlag no se
 *        activa en el tiempo esperado.
 *
 * @return SX1262_Status_t SX1262_OK si el chip volvió a Standby,
 *                         SX1262_ERROR si falla la escritura SPI.
 */
SX1262_Status_t SX1262_AbortReceive(void);

/**
 * @brief Manejador de IRQ del SX1262. Llamar desde HAL_GPIO_EXTI_Callback().
 *
 *        Esta función está diseñada para ejecutarse en contexto de
 *        interrupción (ISR). Única responsabilidad: activar SX1262_RxDoneFlag.
 *        NO realiza ninguna comunicación SPI.
 *
 * Ejemplo de integración en main.c:
 * @code
 *   void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 *       if (GPIO_Pin == DIO_Pin) {
 *           SX1262_IRQ_Handler();
 *       }
 *   }
 * @endcode
 */
void SX1262_IRQ_Handler(void);

/**
 * @brief Aplica la configuración de red y modulación LoRa al chip
 * 
 * @param config Puntero a la estructura de configuración (lora_config_t)
 * @return SX1262_Status_t
 */
SX1262_Status_t SX1262_ApplyConfig(lora_config_t *config);

/**
 * @brief Obtiene el RSSI del último paquete recibido.
 *
 *        Usa el comando GetPacketStatus (0x14). El resultado se calcula como
 *        RSSI [dBm] = -RssiPkt / 2, según el datasheet SX1262 §13.5.3.
 *        Debe llamarse justo después de una recepción exitosa.
 *
 * @param rssi_dbm  Puntero donde se almacenará el RSSI en dBm (valor negativo típico).
 * @return SX1262_Status_t
 */
SX1262_Status_t SX1262_GetRSSI(int16_t *rssi_dbm);

/**
 * @brief Obtiene el SNR del último paquete recibido.
 *
 *        Usa el comando GetPacketStatus (0x14). El resultado se calcula como
 *        SNR [dB] = SnrPkt / 4, donde SnrPkt es un int8_t en complemento a dos.
 *        El valor puede ser negativo si la señal está por debajo del nivel de ruido.
 *        Debe llamarse justo después de una recepción exitosa.
 *
 * @param snr_db    Puntero donde se almacenará el SNR en dB (puede ser negativo).
 * @return SX1262_Status_t
 */
SX1262_Status_t SX1262_GetSNR(int8_t *snr_db);

/**
 * @brief Retorna una copia de la configuración LoRa actualmente aplicada al chip.
 *
 *        No realiza ninguna comunicación SPI. Refleja el estado enviado en la
 *        última llamada exitosa a SX1262_ApplyConfig(). El campo config_pending
 *        de la copia siempre será false.
 *
 * @param config    Puntero a la estructura donde se copiará la configuración actual.
 * @return SX1262_Status_t
 */
SX1262_Status_t SX1262_GetConfig(lora_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* SX1262_H */