/**
 * @file WS2812.c
 *
 * @brief Implementación de la librería para el control de tiras de LEDs WS2812 en STM32 mediante SPI.
 *
 * @author Daniel Ruiz
 * @date Noviembre 12, 2026
 * @version 0.1.0
 */

#include "WS2812.h"
#include <stdlib.h>
#include <string.h>

// ============================================================================
// MACROS PRIVADAS
// ============================================================================

#define WS2812_SPI_BYTES_PER_LED    9U      /**< 24 bits WS2812 x 3 bits SPI = 72 bits = 9 bytes */
#define WS2812_RESET_BYTES          100U    /**< Silencio final: 100 bytes x 3.33 us (2.4 MHz) ~ 333 us >= 300 us de latch */

_Static_assert((size_t)WS2812_MAX_LEDS * WS2812_SPI_BYTES_PER_LED + WS2812_RESET_BYTES <= 0xFFFFU,
               "WS2812_MAX_LEDS demasiado grande: HAL_SPI_Transmit acepta como maximo 65535 bytes");

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static SPI_HandleTypeDef*   WS2812_hspi                         = NULL; /**< Handle de SPI utilizado para comunicarse con el WS2812 */
static uint8_t              WS2812_Initialized                  = 0U;   /**< Bandera para verificar si el módulo está inicializado */
static uint8_t              WS2812_Brightness                   = 255U; /**< Brillo global (0-255) */
static uint16_t             WS2812_NumLeds                      = 0U;   /**< Cantidad de LEDs de la tira (fijada en Init) */
static uint8_t              (*WS2812_Pixels)[3]                 = NULL; /**< Color de cada píxel en orden G, R, B (reservado en Init) */
static uint8_t*             WS2812_SpiBuf                       = NULL; /**< Buffer codificado para SPI (reservado en Init) */
static size_t               WS2812_SpiBufSize                   = 0U;   /**< Tamaño en bytes de WS2812_SpiBuf */

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el módulo WS2812.
 *
 * @param hspi     Handle del SPI ya inicializado por CubeMX.
 * @param num_leds Cantidad de LEDs de la tira (1 a WS2812_MAX_LEDS).
 *
 * @return WS2812_Status_t Estado de la operación.
 */
WS2812_Status_t WS2812_Init(SPI_HandleTypeDef* hspi, uint16_t num_leds)
{
    // Validar parámetros de entrada y configuración del SPI
    if (num_leds == 0U || num_leds > WS2812_MAX_LEDS ||
        hspi == NULL || hspi->Instance == NULL ||
        hspi->State == HAL_SPI_STATE_RESET ||
        hspi->Init.Mode != SPI_MODE_MASTER ||
        hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY ||
        hspi->Init.DataSize != SPI_DATASIZE_8BIT ||
        hspi->Init.FirstBit != SPI_FIRSTBIT_MSB)
    {
        return WS2812_INVALID_PARAM;
    }

    if (WS2812_Initialized)
    {
        (void)WS2812_DeInit();   // Re-Init: libera los buffers anteriores
    }

    WS2812_Pixels = calloc(num_leds, 3U);   // calloc: todos los píxeles apagados
    WS2812_SpiBufSize = (size_t)num_leds * WS2812_SPI_BYTES_PER_LED + WS2812_RESET_BYTES;
    WS2812_SpiBuf = malloc(WS2812_SpiBufSize);

    if (WS2812_Pixels == NULL || WS2812_SpiBuf == NULL)
    {
        free(WS2812_Pixels);
        free(WS2812_SpiBuf);
        WS2812_Pixels = NULL;
        WS2812_SpiBuf = NULL;
        return WS2812_ERROR;
    }

    WS2812_hspi = hspi;
    WS2812_NumLeds = num_leds;
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

    free(WS2812_Pixels);
    free(WS2812_SpiBuf);
    WS2812_Pixels = NULL;
    WS2812_SpiBuf = NULL;
    WS2812_SpiBufSize = 0U;
    WS2812_NumLeds = 0U;

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

    if (n >= WS2812_NumLeds)
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
 * @brief Variante de WS2812_SetPixelColor con color empaquetado 0x00RRGGBB.
 *
 * @param n Índice del píxel.
 * @param c Color empaquetado 0x00RRGGBB.
 *
 * @return WS2812_Status_t Estado de la operación.
 */
WS2812_Status_t WS2812_SetPixelColorPacked(uint16_t n, uint32_t c)
{
    return WS2812_SetPixelColor(n, (uint8_t)(c >> 16), (uint8_t)(c >> 8), (uint8_t)c);
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

    if (first >= WS2812_NumLeds)
    {
        return WS2812_INVALID_PARAM;
    }

    // Como en Adafruit: count = 0 o fuera de rango -> hasta el último píxel
    uint16_t end = (count == 0U || count > (uint16_t)(WS2812_NumLeds - first)) ? WS2812_NumLeds : (uint16_t)(first + count);

    for (uint16_t i = first; i < end; i++)
    {
        (void)WS2812_SetPixelColor(i, r, g, b);   // No puede fallar: init e índice ya validados
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
    for (uint32_t i = 0; i < (uint32_t)WS2812_NumLeds * 3U; i++)
    {
        ptr[i] = (uint8_t)((ptr[i] * scale) >> 8);
    }

    WS2812_Brightness = brightness;

    return WS2812_OK;
}

/**
 * @brief Obtiene el brillo global actual del módulo WS2812.
 *
 * @return uint8_t Valor de brillo (0-255).
 */
uint8_t WS2812_GetBrightness(void)
{
    return WS2812_Brightness;
}

/**
 * @brief Convierte un color HSV a RGB empaquetado (0x00RRGGBB).
 *
 * @param hue Matiz de 16 bits (0-65535 = una vuelta completa).
 * @param sat Saturación (0-255).
 * @param val Valor/luminosidad (0-255).
 *
 * @return uint32_t Color empaquetado 0x00RRGGBB.
 */
uint32_t WS2812_ColorHSV(uint16_t hue, uint8_t sat, uint8_t val)
{
    uint8_t r, g, b;

    // Matiz 0-65535 -> 0-1529 (6 tramos de 255), redondeado
    uint32_t h = ((uint32_t)hue * 1530U + 32768U) / 65536U;

    if (h < 510U)           // Rojo -> Verde
    {
        b = 0U;
        if (h < 255U) { r = 255U;                 g = (uint8_t)h; }
        else          { r = (uint8_t)(510U - h);  g = 255U; }
    }
    else if (h < 1020U)     // Verde -> Azul
    {
        r = 0U;
        if (h < 765U) { g = 255U;                  b = (uint8_t)(h - 510U); }
        else          { g = (uint8_t)(1020U - h);  b = 255U; }
    }
    else if (h < 1530U)     // Azul -> Rojo
    {
        g = 0U;
        if (h < 1275U) { r = (uint8_t)(h - 1020U); b = 255U; }
        else           { r = 255U;                 b = (uint8_t)(1530U - h); }
    }
    else                    // hue cercano a 65535 redondea a 1530 = rojo
    {
        r = 255U; g = 0U; b = 0U;
    }

    // Saturación (mezcla con blanco) y valor (escala), como en Adafruit
    uint32_t v1 = 1U + val;
    uint32_t s1 = 1U + sat;
    uint32_t s2 = 255U - sat;

    return (((((r * s1) >> 8) + s2) * v1) & 0xFF00U) << 8 |
           (((((g * s1) >> 8) + s2) * v1) & 0xFF00U)      |
           (((((b * s1) >> 8) + s2) * v1) >> 8);
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

    memset(WS2812_Pixels, 0, (size_t)WS2812_NumLeds * 3U);

    return WS2812_OK;
}

/**
 * @brief Transmite el buffer de píxeles a la tira por SPI (bloqueante).
 *
 * @return WS2812_Status_t Estado de la operación.
 */
WS2812_Status_t WS2812_Show(void)
{
    if (!WS2812_Initialized)
    {
        return WS2812_NOT_INITIALIZED;
    }

    // Cada byte de color (8 bits) -> 24 bits SPI (3 bytes): 1 -> 110, 0 -> 100
    uint8_t* out = WS2812_SpiBuf;
    const uint8_t* in = &WS2812_Pixels[0][0];

    for (uint32_t i = 0; i < (uint32_t)WS2812_NumLeds * 3U; i++)
    {
        uint32_t v = 0U;
        for (int8_t bit = 7; bit >= 0; bit--)
        {
            v = (v << 3) | (((in[i] >> bit) & 1U) ? 0x6U : 0x4U);
        }
        *out++ = (uint8_t)(v >> 16);
        *out++ = (uint8_t)(v >> 8);
        *out++ = (uint8_t)v;
    }

    memset(out, 0, WS2812_RESET_BYTES);   // Línea en bajo para el latch

    switch (HAL_SPI_Transmit(WS2812_hspi, WS2812_SpiBuf, (uint16_t)WS2812_SpiBufSize, WS2812_SPI_TIMEOUT_MS))
    {
        case HAL_OK:      return WS2812_OK;
        case HAL_TIMEOUT: return WS2812_TIMEOUT;
        default:          return WS2812_ERROR;
    }
}
