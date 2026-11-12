/**
 * @file WS2812.h
 * @brief Driver del controlador WS2812 sobre interfaz SPI.
 *
 * @details 
 *
 * @author Daniel Ruiz
 * @date Noviembre 12, 2026
 * @version 0.1.0
 */

#ifndef WS2812_H
#define WS2812_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"

// ============================================================================
// MACROS Y CONSTANTES WS2812
// ============================================================================

#ifndef WS2812_NUM_LEDS
#define WS2812_NUM_LEDS         8U      /**< Número de LEDs de la tira (redefinible antes de incluir) */
#endif

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del WS2812.
 */
typedef enum {
    WS2812_OK               = 0,    /**< Operación exitosa */
    WS2812_ERROR            = 1,    /**< Error en la operación */
    WS2812_TIMEOUT          = 2,    /**< Timeout en la operación */
    WS2812_NOT_INITIALIZED  = 3,    /**< Módulo no inicializado */
    WS2812_INVALID_PARAM    = 4     /**< Parámetro inválido */
} WS2812_Status_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el módulo WS2812.
 *
 * Valida la configuración del SPI, guarda el handle, apaga todos los píxeles
 * del buffer y fija el brillo al máximo. No transmite nada a la tira.
 *
 * @param hspi Handle del SPI ya inicializado por CubeMX.
 *
 * @return WS2812_OK si la inicialización fue exitosa.
 * @return WS2812_INVALID_PARAM si hspi es NULL o el SPI no es Master, 8 bits, MSB first.
 *
 * @note El SPI debe configurarse como Master (Transmit Only), 8 bits, MSB first,
 *       CPOL Low, a ~2.4 MHz (3 bits SPI por bit WS2812). Solo se usa MOSI.
 */
WS2812_Status_t WS2812_Init(SPI_HandleTypeDef* hspi);

/**
 * @brief Desinicializa el módulo WS2812 liberando la configuración almacenada.
 *
 * @return WS2812_Status_t Estado de la operación.
 */
WS2812_Status_t WS2812_DeInit(void);

/**
 * @brief Establece el color de un píxel específico.
 * 
 * @param n Índice del píxel.
 * @param r Valor de rojo.
 * @param g Valor de verde.
 * @param b Valor de azul.
 * 
 * @return WS2812_Status_t Estado de la operación.
 */
WS2812_Status_t WS2812_SetPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Rellena un rango de píxeles con un mismo color.
 *
 * @param r     Valor de rojo.
 * @param g     Valor de verde.
 * @param b     Valor de azul.
 * @param first Índice del primer píxel a rellenar.
 * @param count Cantidad de píxeles a rellenar (0 o excedente = hasta el final de la tira).
 *
 * @return WS2812_OK si se rellenó el rango.
 * @return WS2812_INVALID_PARAM si first está fuera de la tira.
 */
WS2812_Status_t WS2812_Fill(uint8_t r, uint8_t g, uint8_t b, uint16_t first, uint16_t count);

/**
 * @brief Establece el brillo del módulo WS2812.
 *
 * @param brightness Valor de brillo (0-255).
 *
 * @return WS2812_Status_t Estado de la operación.
 */
WS2812_Status_t WS2812_SetBrightness(uint8_t brightness);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_H */