/**
 * @file CME6005.h
 *
 * @brief Librería para decodificación de la señal WWVB (60 kHz) mediante el
 *        receptor CME6005 en microcontroladores STM32 con HAL.
 *
 *        El módulo WVB-0860N-03A expone cuatro pines:
 *          - GND  : Tierra
 *          - VCC  : Alimentación (1.1 - 3.3 V)
 *          - PON  : Power ON activo en LOW (LOW = encendido, HIGH = apagado)
 *          - NTCO : Salida demodulada invertida (HIGH = portadora caída,
 *                   LOW = portadora en nivel normal)
 *
 *        Esquema de modulación WWVB (NIST, Fort Collins CO, 60 kHz):
 *          - Pulso 200 ms → bit "0"
 *          - Pulso 500 ms → bit "1"
 *          - Pulso 800 ms → marcador de posición / referencia de trama
 *
 *        La decodificación se basa en interrupciones GPIO (ambos flancos de
 *        NTCO) y HAL_GetTick() para la medición de tiempo. El usuario debe
 *        llamar a CME6005_NTCO_RisingEdgeCallback() y
 *        CME6005_NTCO_FallingEdgeCallback() desde su manejador de interrupción.
 *
 * @author Daniel Ruiz
 * @date   Abril 20, 2026
 * @version 1.0.0
 */

#ifndef CME6005_H
#define CME6005_H

/**
 * @brief Incluir el encabezado adecuado según la familia STM32 utilizada.
 * Por ejemplo:
 * - Para STM32F1xx: "stm32f1xx_hal.h"
 * - Para STM32F4xx: "stm32f4xx_hal.h"
 */
#include "stm32f4xx_hal.h"
#include <stdint.h>

// ============================================================================
// MACROS Y CONSTANTES DE PROTOCOLO WWVB
// ============================================================================

/** @brief Número de bits por trama WWVB (1 trama = 1 minuto). */
#define WWVB_FRAME_BITS 60U

/** @brief Umbral superior para clasificar un bit "0" (nominal 200 ms). */
#define WWVB_BIT0_THRESHOLD_MS 350U

/** @brief Umbral superior para clasificar un bit "1" (nominal 500 ms). */
#define WWVB_BIT1_THRESHOLD_MS 650U

/**
 * @brief Tiempo mínimo de espera tras activar PON antes de que la recepción
 *        sea confiable (típico 0.5 s, máximo 2 s según datasheet CME6005).
 */
#define CME6005_POWER_ON_DELAY_MS 2000U

// ============================================================================
// CONFIGURACIÓN TIEMPO (ESTRUCTURAS)
// ============================================================================

/**
 * @brief Estructura que representa la fecha y hora actuales.
 */
typedef struct {
  uint8_t minutos;            /**< Minutos (0 - 59)                        */
  uint8_t horas;              /**< Horas en UTC (0 - 23)                   */
  uint16_t dia_del_anio;      /**< Día del año (1 - 366)                   */
  uint8_t anio;               /**< Últimos dos dígitos del año (00 - 99)   */
  uint8_t dst;                /**< Bits DST: bit1=DST vigente, bit0=cambio */
  uint8_t anio_bisiesto;      /**< 1 = año bisiesto                        */
  uint8_t segundo_intercalar; /**< 1 = segundo intercalar al final del mes */
  int8_t correccion_uti;      /**< Corrección UTI en décimas de segundo    */
} WWVB_Tiempo_t;

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Códigos de retorno de las funciones de la librería.
 */
typedef enum {
  CME6005_OK = 0,          /**< Operación exitosa                          */
  CME6005_ERROR = 1,       /**< Error genérico                             */
  CME6005_NOT_READY = 2,   /**< Trama aún no disponible                    */
  CME6005_FRAME_ERROR = 3, /**< Error de sincronización o marcador inválido */
  CME6005_NOT_INITIALIZED = 4, /**< Módulo no inicializado */
} CME6005_Status_t;

/**
 * @brief Estados internos de la máquina de estados del decodificador WWVB.
 */
typedef enum {
  WWVB_STATE_POWER_DOWN = 0, /**< Módulo apagado (PON = HIGH)                */
  WWVB_STATE_SYNCING = 1,    /**< Buscando dos marcadores 800 ms consecutivos */
  WWVB_STATE_RECEIVING = 2,  /**< Recibiendo bits de la trama                */
  WWVB_STATE_FRAME_READY = 3, /**< Trama completa lista para leer             */
} WWVB_State_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa y configura los pines GPIO del módulo.
 *
 *        Los pines deben estar previamente habilitados con MX_GPIO_Init() o
 *        equivalente. Esta función solo almacena los punteros y pone el
 *        módulo en modo Power Down (PON = HIGH).
 *
 * @param pon_port  Puerto GPIO conectado a PON (ej. GPIOB).
 * @param pon_pin   Pin GPIO conectado a PON (ej. GPIO_PIN_0).
 * @param ntco_port Puerto GPIO conectado a NTCO (ej. GPIOA).
 * @param ntco_pin  Pin GPIO conectado a NTCO (ej. GPIO_PIN_1).
 * @return CME6005_Status_t Estado de la inicialización
 */
CME6005_Status_t CME6005_Init(GPIO_TypeDef *pon_port, uint16_t pon_pin,
                              GPIO_TypeDef *ntco_port, uint16_t ntco_pin);

/**
 * @brief Enciende el módulo receptor (PON = LOW) e inicia la sincronización.
 *
 *        Espera CME6005_POWER_ON_DELAY_MS con HAL_Delay() para garantizar
 *        la estabilización interna del CME6005.
 *
 * @return CME6005_Status_t
 */
CME6005_Status_t CME6005_PowerOn(void);

/**
 * @brief Apaga el módulo receptor (PON = HIGH).
 *
 * @return CME6005_Status_t
 */
CME6005_Status_t CME6005_PowerDown(void);

/**
 * @brief Reinicia el estado del decodificador sin apagar el módulo.
 *
 *        Útil para forzar una nueva sincronización tras un error.
 *
 * @return CME6005_Status_t
 */
CME6005_Status_t CME6005_Reset(void);

/**
 * @brief Callback de flanco de subida de NTCO (portadora cae).
 *
 *        Llamar desde HAL_GPIO_EXTI_Callback() cuando el pin NTCO genera
 *        una interrupción por flanco de subida.
 */
void CME6005_NTCO_RisingEdgeCallback(void);

/**
 * @brief Callback de flanco de bajada de NTCO (portadora se restaura).
 *
 *        Llamar desde HAL_GPIO_EXTI_Callback() cuando el pin NTCO genera
 *        una interrupción por flanco de bajada.
 */
void CME6005_NTCO_FallingEdgeCallback(void);

/**
 * @brief Consulta si hay una trama decodificada disponible.
 *
 * @retval 1  Trama lista (llamar a CME6005_GetTime() para obtenerla).
 * @retval 0  Aún no hay trama disponible.
 */
uint8_t CME6005_IsFrameReady(void);

/**
 * @brief Copia el tiempo decodificado al buffer del usuario.
 *
 *        Tras la lectura, el estado regresa a WWVB_STATE_SYNCING para
 *        comenzar a recibir la siguiente trama automáticamente.
 *
 * @param tiempo_out Puntero a la estructura donde se copiará el tiempo.
 * @return CME6005_OK       si la trama estaba lista.
 * @return CME6005_NOT_READY si aún no hay trama decodificada.
 * @return CME6005_NOT_INITIALIZED si no se inicializó.
 */
CME6005_Status_t CME6005_GetTime(WWVB_Tiempo_t *tiempo_out);

#ifdef __cplusplus
}
#endif

#endif /* CME6005_H */
