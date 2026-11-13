/**
 * @file WS2812.h
 *
 * @brief Librería para el control de tiras de LEDs WS2812 mediante SPI.
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
#include <stdint.h>

// ============================================================================
// MACROS Y CONSTANTES WS2812
// ============================================================================

#define WS2812_MAX_LEDS         7270U   /**< Máximo de LEDs: HAL_SPI_Transmit acepta 65535 bytes = LEDs x 9 + 100 de reset */

#ifndef WS2812_SPI_TIMEOUT_MS
#define WS2812_SPI_TIMEOUT_MS   100U    /**< Timeout de la transmisión SPI en ms (redefinible antes de incluir) */
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
 * Valida la configuración del SPI, guarda el handle, reserva dinámicamente los
 * buffers (calloc/malloc), apaga todos los píxeles y fija el brillo al máximo.
 * No transmite nada a la tira. Si ya estaba inicializado, libera y reinicia.
 *
 * @param hspi     Handle del SPI ya inicializado por CubeMX.
 * @param num_leds Cantidad de LEDs de la tira (1 a WS2812_MAX_LEDS).
 *
 * @return WS2812_OK si la inicialización fue exitosa.
 * @return WS2812_INVALID_PARAM si num_leds es 0 o excede WS2812_MAX_LEDS, hspi es NULL,
 *         el SPI no está inicializado, es solo-RX o no es Master, 8 bits, MSB first.
 * @return WS2812_ERROR si no hay memoria suficiente para los buffers.
 *
 * @note El SPI debe configurarse como Master (Transmit Only), 8 bits, MSB first,
 *       CPOL Low, a ~2.4 MHz (3 bits SPI por bit WS2812). Solo se usa MOSI.
 */
WS2812_Status_t WS2812_Init(SPI_HandleTypeDef* hspi, uint16_t num_leds);

/**
 * @brief Desinicializa el módulo WS2812 liberando los buffers y la configuración almacenada.
 *
 * @return WS2812_OK si se desinicializó correctamente.
 * @return WS2812_NOT_INITIALIZED si el módulo no está inicializado.
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
 * @return WS2812_OK si se estableció el color.
 * @return WS2812_NOT_INITIALIZED si el módulo no está inicializado.
 * @return WS2812_INVALID_PARAM si n está fuera de la tira.
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
 * @return WS2812_NOT_INITIALIZED si el módulo no está inicializado.
 * @return WS2812_INVALID_PARAM si first está fuera de la tira.
 */
WS2812_Status_t WS2812_Fill(uint8_t r, uint8_t g, uint8_t b, uint16_t first, uint16_t count);

/**
 * @brief Establece el brillo del módulo WS2812.
 *
 * @param brightness Valor de brillo (0-255).
 *
 * @return WS2812_OK si se aplicó el brillo.
 * @return WS2812_NOT_INITIALIZED si el módulo no está inicializado.
 */
WS2812_Status_t WS2812_SetBrightness(uint8_t brightness);

/**
 * @brief Obtiene el brillo global actual del módulo WS2812.
 *
 * @return Valor de brillo (0-255). Vale 255 si el módulo no se ha inicializado.
 */
uint8_t WS2812_GetBrightness(void);

/**
 * @brief Convierte un color HSV a RGB empaquetado (equivalente a Adafruit_NeoPixel::ColorHSV).
 *
 * Función pura: no requiere que el módulo esté inicializado ni aplica el brillo global.
 *
 * @param hue Matiz de 16 bits (0-65535 = una vuelta completa: rojo, verde, azul, rojo).
 * @param sat Saturación (0 = blanco, 255 = color puro).
 * @param val Valor/luminosidad (0 = negro, 255 = máximo).
 *
 * @return Color empaquetado 0x00RRGGBB. Se usa con SetPixelColor así:
 *         c >> 16 (R), c >> 8 (G), c (B), tomando los 8 bits bajos de cada uno.
 */
uint32_t WS2812_ColorHSV(uint16_t hue, uint8_t sat, uint8_t val);

/**
 * @brief Apaga todos los píxeles del buffer (no transmite, se requiere show).
 *
 * @return WS2812_OK si se limpió el buffer.
 * @return WS2812_NOT_INITIALIZED si el módulo no está inicializado.
 */
WS2812_Status_t WS2812_Clear(void);

/**
 * @brief Transmite el buffer de píxeles a la tira por SPI (bloqueante).
 *
 * Codifica cada bit WS2812 como 3 bits SPI (1 -> 110, 0 -> 100) y añade un
 * silencio de ~300 us al final para que la tira haga latch.
 *
 * @return WS2812_OK si se transmitió correctamente.
 * @return WS2812_NOT_INITIALIZED si el módulo no está inicializado.
 * @return WS2812_TIMEOUT si la transmisión excedió WS2812_SPI_TIMEOUT_MS.
 * @return WS2812_ERROR para cualquier otro fallo del HAL.
 */
WS2812_Status_t WS2812_Show(void);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_H */
