/**
 * @file WS2812.c
 * @brief Driver del controlador WS2812 sobre interfaz SPI.
 *
 * @details
 *
 * @author Daniel Ruiz
 * @date Noviembre 12, 2026
 * @version 0.1.0
 */

#include "WS2812.h"
#include <string.h>

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static SPI_HandleTypeDef*   WS2812_hspi                         = NULL; /**< Handle de SPI utilizado para comunicarse con el WS2812 */
static uint8_t              WS2812_Initialized                  = 0U;   /**< Bandera para verificar si el módulo está inicializado */
static uint8_t              WS2812_Brightness                   = 255U; /**< Brillo global (0-255) */
static uint8_t              WS2812_Pixels[WS2812_NUM_LEDS][3]   = {0};  /**< Color de cada píxel en orden G, R, B */

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el módulo WS2812.
 *
 * @param hspi Handle del SPI ya inicializado por CubeMX.
 *
 * @return WS2812_Status_t Estado de la operación.
 */
WS2812_Status_t WS2812_Init(SPI_HandleTypeDef* hspi)
{
    // Validar parámetros de entrada y configuración del SPI
    if (hspi == NULL || hspi->Instance == NULL ||
        hspi->Init.Mode != SPI_MODE_MASTER ||
        hspi->Init.DataSize != SPI_DATASIZE_8BIT ||
        hspi->Init.FirstBit != SPI_FIRSTBIT_MSB)
    {
        return WS2812_INVALID_PARAM;
    }

    WS2812_hspi = hspi;

    memset(WS2812_Pixels, 0, sizeof(WS2812_Pixels));   // Todos los píxeles apagados
    WS2812_Brightness = 255U;

    WS2812_Initialized = 1U;

    return WS2812_OK;
}

/**
 * @brief Desinicializa el módulo WS2812 liberando la configuración almacenada.
 *
 * @return WS2812_Status_t Estado de la operación.
 */
WS2812_Status_t WS2812_DeInit(void)
{
    if (WS2812_Initialized != 1U)
    {
        return WS2812_NOT_INITIALIZED;
    }

    memset(WS2812_Pixels, 0, sizeof(WS2812_Pixels));

    WS2812_hspi = NULL;
    WS2812_Initialized = 0U;

    return WS2812_OK;
}

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
WS2812_Status_t WS2812_SetPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
{
    if (!WS2812_Initialized)
    {
        return WS2812_NOT_INITIALIZED;
    }

    if (n >= WS2812_NUM_LEDS)
    {
        return WS2812_INVALID_PARAM;
    }

    // Escalado por brillo como en Adafruit: (c * (brillo + 1)) >> 8 -> 255 deja el color intacto, 0 lo apaga
    uint16_t scale = (uint16_t)WS2812_Brightness + 1U;

    // Orden de transmisión del WS2812: G, R, B
    WS2812_Pixels[n][0] = (uint8_t)((g * scale) >> 8);
    WS2812_Pixels[n][1] = (uint8_t)((r * scale) >> 8);
    WS2812_Pixels[n][2] = (uint8_t)((b * scale) >> 8);

    return WS2812_OK;
}

/**
 * @brief Rellena un rango de píxeles con un mismo color.
 *
 * @param r     Valor de rojo.
 * @param g     Valor de verde.
 * @param b     Valor de azul.
 * @param first Índice del primer píxel a rellenar.
 * @param count Cantidad de píxeles a rellenar (0 o excedente = hasta el final de la tira).
 *
 * @return WS2812_Status_t Estado de la operación.
 */
WS2812_Status_t WS2812_Fill(uint8_t r, uint8_t g, uint8_t b, uint16_t first, uint16_t count)
{
    if (!WS2812_Initialized)
    {
        return WS2812_NOT_INITIALIZED;
    }

    if (first >= WS2812_NUM_LEDS)
    {
        return WS2812_INVALID_PARAM;
    }

    // Como en Adafruit: count = 0 o fuera de rango -> hasta el último píxel
    uint16_t end = (count == 0U || count > (uint16_t)(WS2812_NUM_LEDS - first)) ? WS2812_NUM_LEDS : (uint16_t)(first + count);

    for (uint16_t i = first; i < end; i++)
    {
        WS2812_SetPixelColor(i, r, g, b);
    }

    return WS2812_OK;
}

/**
 * @brief Establece el brillo del módulo WS2812.
 * 
 * @param brightness Valor de brillo (0-255).
 * 
 * @return WS2812_Status_t Estado de la operación.
 */
WS2812_Status_t WS2812_SetBrightness(uint8_t brightness)
{
    if (!WS2812_Initialized)
    {
        return WS2812_NOT_INITIALIZED;
    }

    if (brightness == WS2812_Brightness)
    {
        return WS2812_OK;
    }

    // Como en Adafruit, se reescalan los píxeles ya almacenados (proceso con pérdida al subir el brillo).
    // Los datos en RAM ya están escalados por (WS2812_Brightness + 1), se re-escalan a (brightness + 1).
    // Con brillo = 255 -> (256 << 8) - 1 = 65535, sin overflow porque la aritmética se promueve a int.
    uint8_t  old = WS2812_Brightness;
    uint16_t scale = (old == 0U) ? 0U : (uint16_t)(((((uint16_t)brightness + 1U) << 8) - 1U) / old); // Evita /0

    uint8_t* ptr = &WS2812_Pixels[0][0];
    for (uint16_t i = 0; i < sizeof(WS2812_Pixels); i++)
    {
        ptr[i] = (uint8_t)((ptr[i] * scale) >> 8);
    }

    WS2812_Brightness = brightness;

    return WS2812_OK;
}

/**
 * @brief Apaga todos los píxeles del buffer.
 *
 * @return WS2812_Status_t Estado de la operación.
 */
WS2812_Status_t WS2812_Clear(void)
{
    if (!WS2812_Initialized)
    {
        return WS2812_NOT_INITIALIZED;
    }

    memset(WS2812_Pixels, 0, sizeof(WS2812_Pixels));

    return WS2812_OK;
}