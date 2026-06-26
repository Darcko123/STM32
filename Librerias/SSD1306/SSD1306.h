/**
 * original author:  Tilen Majerle<tilen@majerle.eu>
 * modification for STM32f10x: Alexander Lutsai<s.lyra@ya.ru>

   ----------------------------------------------------------------------
   	Copyright (C) Alexander Lutsai, 2016
    Copyright (C) Tilen Majerle, 2015

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */

/**
 * @file SSD1306.h
 * @brief Librería para la gestión del controlador de pantalla OLED SSD1306 mediante comunicación I2C.
 *
 * @details Driver para pantallas OLED monocromáticas basadas en el controlador
 *          SSD1306, soporta inicialización, primitivas de dibujo (píxeles, líneas,
 *          rectángulos, triángulos, círculos, bitmaps), texto con fuentes
 *          personalizadas y efectos de scroll por hardware.
 *
 * @author Tilen Majerle, Alexander Lutsai - Daniel Ruiz
 * @date Junio 24, 2026
 * @version 1.0.0
 */

#ifndef SSD1306_H
#define SSD1306_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdint.h>
#include <string.h>

// ============================================================================
// MACROS Y CONSTANTES SSD1306
// ============================================================================

/* -- Dimensiones de la pantalla -- */
#define SSD1306_WIDTH        128U  /**< Ancho de la pantalla en píxeles */
#define SSD1306_HEIGHT       64U   /**< Alto de la pantalla en píxeles  */

/* -- Dirección I2C del módulo -- */
#define SSD1306_I2C_ADDR      0x78U /**< Dirección I2C del SSD1306 (7 bits desplazada) */

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Códigos de estado retornados por todas las funciones públicas del SSD1306.
 */
typedef enum {
    SSD1306_OK              = 0,    /**< Operación exitosa */
    SSD1306_ERROR           = 1,    /**< Error en la operación */
    SSD1306_TIMEOUT         = 2,    /**< Timeout en la operación */
    SSD1306_NOT_INITIALIZED = 3,    /**< Módulo no inicializado */
    SSD1306_INVALID_PARAM   = 4,    /**< Parámetro inválido */
} SSD1306_Status_t;

/**
 * @brief Color de un píxel en el buffer del SSD1306.
 */
typedef enum {
	SSD1306_COLOR_BLACK = 0x00, /**< Píxel apagado (negro) */
	SSD1306_COLOR_WHITE = 0x01  /**< Píxel encendido (blanco) */
} SSD1306_COLOR_t;

/**
 * @brief Definición de una fuente tipográfica utilizada por SSD1306_Putc()/SSD1306_Puts().
 */
typedef struct {
	uint8_t  FontWidth;    /**< Ancho de cada carácter en píxeles */
	uint8_t  FontHeight;   /**< Alto de cada carácter en píxeles  */
	const uint16_t* data;  /**< Datos de la fuente (bitmap por carácter) */
} FontDef_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el controlador SSD1306 y lo deja listo para dibujar.
 * @return uint8_t 1 si la inicialización fue exitosa, 0 si el dispositivo no respondió por I2C.
 */
uint8_t SSD1306_Init(void);

/**
 * @brief Envía el contenido del buffer interno a la pantalla física.
 */
void SSD1306_UpdateScreen(void);

/**
 * @brief Invierte el color de todos los píxeles del buffer (negro<->blanco).
 */
void SSD1306_ToggleInvert(void);

/**
 * @brief Rellena todo el buffer con un color sólido.
 * @param[in] color Color de relleno (SSD1306_COLOR_BLACK o SSD1306_COLOR_WHITE).
 */
void SSD1306_Fill(SSD1306_COLOR_t color);

/**
 * @brief Dibuja un píxel individual en el buffer.
 * @param[in] x     Coordenada X (0..SSD1306_WIDTH-1).
 * @param[in] y     Coordenada Y (0..SSD1306_HEIGHT-1).
 * @param[in] color Color del píxel.
 */
void SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color);

/**
 * @brief Posiciona el cursor de escritura de texto.
 * @param[in] x Coordenada X de inicio.
 * @param[in] y Coordenada Y de inicio.
 */
void SSD1306_GotoXY(uint16_t x, uint16_t y);

/**
 * @brief Dibuja un carácter en la posición actual del cursor.
 * @param[in] ch    Carácter a dibujar.
 * @param[in] Font  Fuente a utilizar.
 * @param[in] color Color del carácter.
 * @return char El mismo carácter si se dibujó correctamente, 0 si no había espacio en pantalla.
 */
char SSD1306_Putc(char ch, FontDef_t* Font, SSD1306_COLOR_t color);

/**
 * @brief Dibuja una cadena de texto a partir de la posición actual del cursor.
 * @param[in] str   Cadena a dibujar.
 * @param[in] Font  Fuente a utilizar.
 * @param[in] color Color del texto.
 * @return char 0 si toda la cadena se dibujó correctamente, el carácter que falló en caso contrario.
 */
char SSD1306_Puts(char* str, FontDef_t* Font, SSD1306_COLOR_t color);

/**
 * @brief Dibuja una línea entre dos puntos.
 */
void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR_t c);

/**
 * @brief Dibuja un rectángulo (solo bordes).
 */
void SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c);

/**
 * @brief Dibuja un rectángulo relleno.
 */
void SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c);

/**
 * @brief Dibuja un triángulo (solo bordes) a partir de sus 3 vértices.
 */
void SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color);

/**
 * @brief Dibuja un triángulo relleno a partir de sus 3 vértices.
 */
void SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color);

/**
 * @brief Dibuja un círculo (solo borde).
 * @param[in] x0 Coordenada X del centro.
 * @param[in] y0 Coordenada Y del centro.
 * @param[in] r  Radio del círculo.
 */
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c);

/**
 * @brief Dibuja un círculo relleno.
 * @param[in] x0 Coordenada X del centro.
 * @param[in] y0 Coordenada Y del centro.
 * @param[in] r  Radio del círculo.
 */
void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c);

/**
 * @brief Dibuja un bitmap monocromático en la posición indicada.
 * @param[in] x      Coordenada X de inicio.
 * @param[in] y      Coordenada Y de inicio.
 * @param[in] bitmap Datos del bitmap (1 bit por píxel, empaquetado por byte).
 * @param[in] w      Ancho del bitmap en píxeles.
 * @param[in] h      Alto del bitmap en píxeles.
 * @param[in] color  Color de los píxeles activos.
 */
void SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color);

/**
 * @brief Limpia el buffer y actualiza la pantalla.
 */
void SSD1306_Clear(void);

/**
 * @brief Enciende la pantalla (sale de modo de bajo consumo).
 */
void SSD1306_ON(void);

/**
 * @brief Apaga la pantalla (entra en modo de bajo consumo).
 */
void SSD1306_OFF(void);

/**
 * @brief Invierte los colores de la pantalla a nivel de controlador.
 * @param[in] i Distinto de 0 para invertir, 0 para modo normal.
 */
void SSD1306_InvertDisplay(int i);

/**
 * @brief Inicia un scroll horizontal hacia la derecha.
 * @param[in] start_row Página de inicio (0-7).
 * @param[in] end_row   Página de fin (0-7).
 */
void SSD1306_ScrollRight(uint8_t start_row, uint8_t end_row);

/**
 * @brief Inicia un scroll horizontal hacia la izquierda.
 * @param[in] start_row Página de inicio (0-7).
 * @param[in] end_row   Página de fin (0-7).
 */
void SSD1306_ScrollLeft(uint8_t start_row, uint8_t end_row);

/**
 * @brief Inicia un scroll diagonal hacia la derecha (vertical + horizontal).
 * @param[in] start_row Página de inicio (0-7).
 * @param[in] end_row   Página de fin (0-7).
 */
void SSD1306_Scrolldiagright(uint8_t start_row, uint8_t end_row);

/**
 * @brief Inicia un scroll diagonal hacia la izquierda (vertical + horizontal).
 * @param[in] start_row Página de inicio (0-7).
 * @param[in] end_row   Página de fin (0-7).
 */
void SSD1306_Scrolldiagleft(uint8_t start_row, uint8_t end_row);

/**
 * @brief Detiene cualquier efecto de scroll activo.
 */
void SSD1306_Stopscroll(void);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H */
