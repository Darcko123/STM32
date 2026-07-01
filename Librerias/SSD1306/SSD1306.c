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
 * @file SSD1306.c
 * @brief Librería para la gestión del controlador de pantalla OLED SSD1306 mediante comunicación I2C.
 *
 * @details Driver para pantallas OLED monocromáticas basadas en el controlador
 *          SSD1306, soporta inicialización, primitivas de dibujo (píxeles, líneas,
 *          rectángulos, triángulos, círculos, bitmaps), texto con fuentes
 *          personalizadas y efectos de scroll por hardware.
 *
 * @author Tilen Majerle, Alexander Lutsai - Daniel Ruiz
 * @date Junio 29, 2026
 * @version 2.0.0
 */

#include "SSD1306.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static I2C_HandleTypeDef* SSD1306_hi2c        = NULL; /**< Handle de I2C utilizado para comunicarse con el SSD1306 */
static uint8_t             SSD1306_Initialized = 0U;  /**< Bandera para verificar si el módulo está inicializado */

/** Buffer de la pantalla en RAM (1 bit por píxel), dimensionado al tamaño máximo soportado */
static uint8_t SSD1306_Buffer[SSD1306_MAX_WIDTH * SSD1306_MAX_HEIGHT / 8];

/** Estado interno del cursor de escritura, dimensiones reales y de la inversión de color */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint16_t Width;
	uint16_t Height;
	uint8_t  Inverted;
} SSD1306_State_t;

static SSD1306_State_t SSD1306_State;

// ============================================================================
// MACROS Y CONSTANTES PRIVADAS
// ============================================================================

/* Write command */
#define SSD1306_WRITECOMMAND(command)      ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, (command))
/* Write data */
#define SSD1306_WRITEDATA(data)            ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x40, (data))

#define SSD1306_RIGHT_HORIZONTAL_SCROLL              0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL               0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A
#define SSD1306_DEACTIVATE_SCROLL                    0x2E /* Stop scroll */
#define SSD1306_ACTIVATE_SCROLL                      0x2F /* Start scroll */
#define SSD1306_SET_VERTICAL_SCROLL_AREA             0xA3 /* Set scroll range */

#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_INVERTDISPLAY       0xA7

/* Factor de conversión de grados a radianes, usado por SSD1306_DrawArc */
#define SSD1306_DEG_TO_RAD          0.017453292519943295769236907684886f

// ============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
// ============================================================================

static SSD1306_Status_t ssd1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t data);
static SSD1306_Status_t ssd1306_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count);
static SSD1306_Status_t ssd1306_FillHLine(int16_t x0, int16_t x1, int16_t y, SSD1306_COLOR_t color);
static SSD1306_Status_t ssd1306_FillVLine(int16_t x, int16_t y0, int16_t y1, SSD1306_COLOR_t color);
static SSD1306_Status_t ssd1306_FillArcHelper(int16_t cx, int16_t cy, int16_t oradius, int16_t iradius, float start, float end, SSD1306_COLOR_t color);

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Escribe un único byte de comando o dato al SSD1306 vía I2C.
 *
 * @param address Dirección I2C del dispositivo (ya desplazada 7 bits).
 * @param reg     Byte de control (0x00 para comando, 0x40 para dato).
 * @param data    Byte a escribir.
 *
 * @return SSD1306_Status_t Estado de la operación.
 */
static SSD1306_Status_t ssd1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t data)
{
	HAL_StatusTypeDef hal_status;
	uint8_t dt[2];
	dt[0] = reg;
	dt[1] = data;

	hal_status = HAL_I2C_Master_Transmit(SSD1306_hi2c, address, dt, 2, 10);
	if (hal_status == HAL_TIMEOUT)
	{
		return SSD1306_TIMEOUT;
	}
	else if (hal_status != HAL_OK)
	{
		return SSD1306_ERROR;
	}

	return SSD1306_OK;
}

/**
 * @brief Escribe múltiples bytes de datos al SSD1306 vía I2C.
 *
 * @param address Dirección I2C del dispositivo (ya desplazada 7 bits).
 * @param reg     Byte de control (0x00 para comando, 0x40 para dato).
 * @param data    Puntero al bloque de datos a escribir.
 * @param count   Cantidad de bytes a escribir.
 *
 * @return SSD1306_Status_t Estado de la operación.
 */
static SSD1306_Status_t ssd1306_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count)
{
	HAL_StatusTypeDef hal_status;
	uint8_t dt[256];
	uint16_t i;

	dt[0] = reg;
	for (i = 0; i < count; i++)
	{
		dt[i + 1] = data[i];
	}

	hal_status = HAL_I2C_Master_Transmit(SSD1306_hi2c, address, dt, count + 1, 10);
	if (hal_status == HAL_TIMEOUT)
	{
		return SSD1306_TIMEOUT;
	}
	else if (hal_status != HAL_OK)
	{
		return SSD1306_ERROR;
	}

	return SSD1306_OK;
}

/**
 * @brief Rellena un segmento horizontal directamente en el buffer de pantalla.
 *
 * @details Evita el cómputo de Bresenham de SSD1306_DrawLine para el caso
 *          (ya conocido) de una línea horizontal, usado por las rutinas de
 *          relleno de círculos y triángulos.
 *
 * @param x0    Coordenada X inicial (puede exceder los límites, se recorta).
 * @param x1    Coordenada X final (puede exceder los límites, se recorta).
 * @param y     Coordenada Y del segmento.
 * @param color Color a utilizar.
 *
 * @return SSD1306_Status_t Estado de la operación.
 */
static SSD1306_Status_t ssd1306_FillHLine(int16_t x0, int16_t x1, int16_t y, SSD1306_COLOR_t color)
{
	SSD1306_Status_t st;
	int16_t x, tmp;

	if (y < 0 || y >= (int16_t)SSD1306_State.Height)
	{
		return SSD1306_OK;
	}

	if (x0 > x1) { tmp = x0; x0 = x1; x1 = tmp; }
	if (x0 < 0) { x0 = 0; }
	if (x1 >= (int16_t)SSD1306_State.Width) { x1 = (int16_t)SSD1306_State.Width - 1; }

	for (x = x0; x <= x1; x++)
	{
		st = SSD1306_DrawPixel((uint16_t)x, (uint16_t)y, color);
		if (st != SSD1306_OK)
		{
			return st;
		}
	}

	return SSD1306_OK;
}

/**
 * @brief Rellena un segmento vertical directamente en el buffer de pantalla.
 *
 * @details Análogo a ssd1306_FillHLine, usado por SSD1306_DrawFastVLine.
 *
 * @param x     Coordenada X del segmento.
 * @param y0    Coordenada Y inicial (puede exceder los límites, se recorta).
 * @param y1    Coordenada Y final (puede exceder los límites, se recorta).
 * @param color Color a utilizar.
 *
 * @return SSD1306_Status_t Estado de la operación.
 */
static SSD1306_Status_t ssd1306_FillVLine(int16_t x, int16_t y0, int16_t y1, SSD1306_COLOR_t color)
{
	SSD1306_Status_t st;
	int16_t y, tmp;

	if (x < 0 || x >= (int16_t)SSD1306_State.Width)
	{
		return SSD1306_OK;
	}

	if (y0 > y1) { tmp = y0; y0 = y1; y1 = tmp; }
	if (y0 < 0) { y0 = 0; }
	if (y1 >= (int16_t)SSD1306_State.Height) { y1 = (int16_t)SSD1306_State.Height - 1; }

	for (y = y0; y <= y1; y++)
	{
		st = SSD1306_DrawPixel((uint16_t)x, (uint16_t)y, color);
		if (st != SSD1306_OK)
		{
			return st;
		}
	}

	return SSD1306_OK;
}

/**
 * @brief Rellena un segmento angular (arco) entre dos ángulos y dos radios.
 *
 * @details Recorre el cuadro delimitador del arco fila por fila y dibuja,
 *          en cada una, los segmentos horizontales que caen dentro de la
 *          corona circular [iradius, oradius] y del sector angular
 *          [start, end]. Usado por SSD1306_DrawArc tanto para los bordes
 *          rectos del arco como para sus bordes curvos.
 *
 * @param cx      Coordenada X del centro.
 * @param cy      Coordenada Y del centro.
 * @param oradius Radio exterior del arco.
 * @param iradius Radio interior del arco.
 * @param start   Ángulo inicial en grados (0 a 360).
 * @param end     Ángulo final en grados (0 a 360).
 * @param color   Color a utilizar.
 *
 * @return SSD1306_Status_t Estado de la operación.
 */
static SSD1306_Status_t ssd1306_FillArcHelper(int16_t cx, int16_t cy, int16_t oradius, int16_t iradius, float start, float end, SSD1306_COLOR_t color)
{
	SSD1306_Status_t st;
	float s_cos, e_cos, sslope, eslope, swidth, ewidth;
	int32_t ir2, or2, xs, y, ye, xe;
	bool start180, end180, reversed;

	if ((start == 90.0f) || (start == 180.0f) || (start == 270.0f) || (start == 360.0f))
	{
		start -= 0.1f;
	}
	if ((end == 90.0f) || (end == 180.0f) || (end == 270.0f) || (end == 360.0f))
	{
		end -= 0.1f;
	}

	s_cos  = cosf(start * SSD1306_DEG_TO_RAD);
	e_cos  = cosf(end * SSD1306_DEG_TO_RAD);
	sslope = s_cos / sinf(start * SSD1306_DEG_TO_RAD);
	eslope = e_cos / sinf(end * SSD1306_DEG_TO_RAD);
	swidth = 0.5f / s_cos;
	ewidth = -0.5f / e_cos;
	--iradius;
	ir2 = (int32_t)iradius * iradius + iradius;
	or2 = (int32_t)oradius * oradius + oradius;

	start180 = !(start < 180.0f);
	end180   = end < 180.0f;
	reversed = (start + 180.0f < end) || (end < start && start < end + 180.0f);

	xs = -oradius;
	y  = -oradius;
	ye = oradius;
	xe = oradius + 1;
	if (!reversed)
	{
		if ((end >= 270.0f || end < 90.0f) && (start >= 270.0f || start < 90.0f))
		{
			xs = 0;
		}
		else if (end < 270.0f && end >= 90.0f && start < 270.0f && start >= 90.0f)
		{
			xe = 1;
		}
		if (end >= 180.0f && start >= 180.0f)
		{
			ye = 0;
		}
		else if (end < 180.0f && start < 180.0f)
		{
			y = 0;
		}
	}

	do
	{
		int32_t y2 = y * y;
		int32_t x  = xs;
		int32_t len = 0;
		float ysslope, yeslope;

		if (x < 0)
		{
			while (x * x + y2 >= or2)
			{
				++x;
			}
			if (xe != 1)
			{
				xe = 1 - x;
			}
		}

		ysslope = ((float)y + swidth) * sslope;
		yeslope = ((float)y + ewidth) * eslope;

		do
		{
			bool flg1 = start180 != (x <= ysslope);
			bool flg2 = end180 != (x <= yeslope);
			int32_t distance = x * x + y2;

			if (distance >= ir2 && ((flg1 && flg2) || (reversed && (flg1 || flg2))) && x != xe && distance < or2)
			{
				++len;
			}
			else
			{
				if (len)
				{
					st = ssd1306_FillHLine(cx + x - len, cx + x - 1, cy + y, color);
					if (st != SSD1306_OK)
					{
						return st;
					}
					len = 0;
				}
				if (distance >= or2)
				{
					break;
				}
				if (x < 0 && distance < ir2)
				{
					x = -x;
				}
			}
		} while (++x <= xe);
	} while (++y <= ye);

	return SSD1306_OK;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

SSD1306_Status_t SSD1306_Init(I2C_HandleTypeDef* hi2c, uint8_t width, uint8_t height)
{
	SSD1306_Status_t st;
	uint32_t p;

	if (hi2c == NULL || width == 0 || height == 0 ||
		width > SSD1306_MAX_WIDTH || height > SSD1306_MAX_HEIGHT)
	{
		return SSD1306_INVALID_PARAM;
	}

	SSD1306_hi2c = hi2c;
	SSD1306_State.Width  = width;
	SSD1306_State.Height = height;

	/* Revisar si OLED está conectado a I2C */
	if (HAL_I2C_IsDeviceReady(SSD1306_hi2c, SSD1306_I2C_ADDR, 1, 20000) != HAL_OK)
	{
		return SSD1306_ERROR;
	}

	/* A little delay */
	p = 2500;
	while (p > 0)
		p--;

	/* Inicialización OLED */
	st  = SSD1306_WRITECOMMAND(0xAE); 			//display off
	st  = st ? st : SSD1306_WRITECOMMAND(0x20); //Set Memory Addressing Mode
	st  = st ? st : SSD1306_WRITECOMMAND(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	st  = st ? st : SSD1306_WRITECOMMAND(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	st  = st ? st : SSD1306_WRITECOMMAND(0xC8); //Set COM Output Scan Direction
	st  = st ? st : SSD1306_WRITECOMMAND(0x00); //---set low column address
	st  = st ? st : SSD1306_WRITECOMMAND(0x10); //---set high column address
	st  = st ? st : SSD1306_WRITECOMMAND(0x40); //--set start line address
	st  = st ? st : SSD1306_WRITECOMMAND(0x81); //--set contrast control register
	st  = st ? st : SSD1306_WRITECOMMAND(0xFF);
	st  = st ? st : SSD1306_WRITECOMMAND(0xA1); //--set segment re-map 0 to 127
	st  = st ? st : SSD1306_WRITECOMMAND(0xA6); //--set normal display
	st  = st ? st : SSD1306_WRITECOMMAND(0xA8); //--set multiplex ratio(1 to 64)
	st  = st ? st : SSD1306_WRITECOMMAND((uint8_t)(height - 1));
	st  = st ? st : SSD1306_WRITECOMMAND(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	st  = st ? st : SSD1306_WRITECOMMAND(0xD3); //-set display offset
	st  = st ? st : SSD1306_WRITECOMMAND(0x00); //-not offset
	st  = st ? st : SSD1306_WRITECOMMAND(0xD5); //--set display clock divide ratio/oscillator frequency
	st  = st ? st : SSD1306_WRITECOMMAND(0xF0); //--set divide ratio
	st  = st ? st : SSD1306_WRITECOMMAND(0xD9); //--set pre-charge period
	st  = st ? st : SSD1306_WRITECOMMAND(0x22);
	st  = st ? st : SSD1306_WRITECOMMAND(0xDA); //--set com pins hardware configuration
	st  = st ? st : SSD1306_WRITECOMMAND((height == 64U) ? 0x12 : 0x02);
	st  = st ? st : SSD1306_WRITECOMMAND(0xDB); //--set vcomh
	st  = st ? st : SSD1306_WRITECOMMAND(0x20); //0x20,0.77xVcc
	st  = st ? st : SSD1306_WRITECOMMAND(0x8D); //--set DC-DC enable
	st  = st ? st : SSD1306_WRITECOMMAND(0x14);
	st  = st ? st : SSD1306_WRITECOMMAND(0xAF); //--turn on SSD1306 panel
	st  = st ? st : SSD1306_WRITECOMMAND(SSD1306_DEACTIVATE_SCROLL);
	if (st != SSD1306_OK)
	{
		return st;
	}

	/* Set default values */
	SSD1306_State.CurrentX = 0;
	SSD1306_State.CurrentY = 0;
	SSD1306_State.Inverted = 0;

	SSD1306_Initialized = 1U;

	/* Limpuar Pantalla */
	st = SSD1306_Fill(SSD1306_COLOR_BLACK);
	if (st != SSD1306_OK)
	{
		return st;
	}

	/* Actualizar Pantalla */
	return SSD1306_UpdateScreen();
}

/**
 * @brief Obtiene el ancho de pantalla configurado en SSD1306_Init.
 *
 * @return uint16_t Ancho en píxeles, o 0 si el módulo no ha sido inicializado.
 */
uint16_t SSD1306_GetWidth(void)
{
	if (SSD1306_Initialized != 1U) { return 0U; }

	return SSD1306_State.Width;
}

/**
 * @brief Obtiene el alto de pantalla configurado en SSD1306_Init.
 *
 * @return uint16_t Alto en píxeles, o 0 si el módulo no ha sido inicializado.
 */
uint16_t SSD1306_GetHeight(void)
{
	if (SSD1306_Initialized != 1U) { return 0U; }

	return SSD1306_State.Height;
}

SSD1306_Status_t SSD1306_UpdateScreen(void)
{
	SSD1306_Status_t st;
	uint8_t m;
	uint8_t pages;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	pages = (uint8_t)(SSD1306_State.Height / 8U);

	for (m = 0; m < pages; m++)
	{
		st  = SSD1306_WRITECOMMAND(0xB0 + m);
		st  = st ? st : SSD1306_WRITECOMMAND(0x00);
		st  = st ? st : SSD1306_WRITECOMMAND(0x10);
		st  = st ? st : ssd1306_I2C_WriteMulti(SSD1306_I2C_ADDR, 0x40, &SSD1306_Buffer[SSD1306_State.Width * m], SSD1306_State.Width);
		if (st != SSD1306_OK)
		{
			return st;
		}
	}

	return SSD1306_OK;
}

SSD1306_Status_t SSD1306_ToggleInvert(void)
{
	uint16_t i;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	/* Toggle invert */
	SSD1306_State.Inverted = !SSD1306_State.Inverted;

	/* Do memory toggle */
	for (i = 0; i < sizeof(SSD1306_Buffer); i++)
	{
		SSD1306_Buffer[i] = ~SSD1306_Buffer[i];
	}

	return SSD1306_OK;
}

SSD1306_Status_t SSD1306_Fill(SSD1306_COLOR_t color)
{
	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	/* Set memory */
	memset(SSD1306_Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF,
		(size_t)SSD1306_State.Width * SSD1306_State.Height / 8U);

	return SSD1306_OK;
}

SSD1306_Status_t SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color)
{
	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	if (x >= SSD1306_State.Width || y >= SSD1306_State.Height)
	{
		return SSD1306_INVALID_PARAM;
	}

	/* Check if pixels are inverted */
	if (SSD1306_State.Inverted)
	{
		color = (SSD1306_COLOR_t)!color;
	}

	/* Set color */
	if (color == SSD1306_COLOR_WHITE)
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_State.Width] |= 1 << (y % 8);
	}
	else
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_State.Width] &= ~(1 << (y % 8));
	}

	return SSD1306_OK;
}

SSD1306_Status_t SSD1306_GotoXY(uint16_t x, uint16_t y)
{
	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	if (x >= SSD1306_State.Width || y >= SSD1306_State.Height)
	{
		return SSD1306_INVALID_PARAM;
	}

	/* Set write pointers */
	SSD1306_State.CurrentX = x;
	SSD1306_State.CurrentY = y;

	return SSD1306_OK;
}

SSD1306_Status_t SSD1306_Putc(char ch, FontDef_t* Font, SSD1306_COLOR_t color)
{
	SSD1306_Status_t st;
	uint32_t i, b, j;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }
	if (Font == NULL)              { return SSD1306_INVALID_PARAM;   }

	/* Check available space in LCD */
	if (
		SSD1306_State.Width <= (SSD1306_State.CurrentX + Font->FontWidth) ||
		SSD1306_State.Height <= (SSD1306_State.CurrentY + Font->FontHeight)
	) {
		return SSD1306_INVALID_PARAM;
	}

	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++)
	{
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++)
		{
			if ((b << j) & 0x8000)
			{
				st = SSD1306_DrawPixel(SSD1306_State.CurrentX + j, SSD1306_State.CurrentY + i, color);
			}
			else
			{
				st = SSD1306_DrawPixel(SSD1306_State.CurrentX + j, SSD1306_State.CurrentY + i, (SSD1306_COLOR_t)!color);
			}

			if (st != SSD1306_OK)
			{
				return st;
			}
		}
	}

	/* Increase pointer */
	SSD1306_State.CurrentX += Font->FontWidth;

	return SSD1306_OK;
}

SSD1306_Status_t SSD1306_Puts(char* str, FontDef_t* Font, SSD1306_COLOR_t color)
{
	SSD1306_Status_t st;

	if (str == NULL) { return SSD1306_INVALID_PARAM; }

	/* Write characters */
	while (*str)
	{
		st = SSD1306_Putc(*str, Font, color);
		if (st != SSD1306_OK)
		{
			return st;
		}

		str++;
	}

	return SSD1306_OK;
}

SSD1306_Status_t SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR_t c)
{
	SSD1306_Status_t st;
	int16_t dx, dy, sx, sy, err, e2, i, tmp;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	/* Check for overflow */
	if (x0 >= SSD1306_State.Width) { x0 = SSD1306_State.Width - 1; }
	if (x1 >= SSD1306_State.Width) { x1 = SSD1306_State.Width - 1; }
	if (y0 >= SSD1306_State.Height) { y0 = SSD1306_State.Height - 1; }
	if (y1 >= SSD1306_State.Height) { y1 = SSD1306_State.Height - 1; }

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	if (dx == 0)
	{
		if (y1 < y0) { tmp = y1; y1 = y0; y0 = tmp; }
		if (x1 < x0) { tmp = x1; x1 = x0; x0 = tmp; }

		/* Vertical line */
		for (i = y0; i <= y1; i++)
		{
			st = SSD1306_DrawPixel(x0, i, c);
			if (st != SSD1306_OK)
			{
				return st;
			}
		}

		return SSD1306_OK;
	}

	if (dy == 0)
	{
		if (y1 < y0) { tmp = y1; y1 = y0; y0 = tmp; }
		if (x1 < x0) { tmp = x1; x1 = x0; x0 = tmp; }

		/* Horizontal line */
		for (i = x0; i <= x1; i++)
		{
			st = SSD1306_DrawPixel(i, y0, c);
			if (st != SSD1306_OK)
			{
				return st;
			}
		}

		return SSD1306_OK;
	}

	while (1)
	{
		st = SSD1306_DrawPixel(x0, y0, c);
		if (st != SSD1306_OK)
		{
			return st;
		}
		if (x0 == x1 && y0 == y1)
		{
			break;
		}
		e2 = err;
		if (e2 > -dx) { err -= dy; x0 += sx; }
		if (e2 < dy)  { err += dx; y0 += sy; }
	}

	return SSD1306_OK;
}

/**
 * @brief Dibuja una línea vertical de forma optimizada (sin Bresenham).
 *
 * @param x     Coordenada X de la línea.
 * @param y     Coordenada Y inicial.
 * @param h     Alto de la línea (puede ser negativo, se normaliza).
 * @param color Color de la línea.
 * @return SSD1306_Status_t
 */
SSD1306_Status_t SSD1306_DrawFastVLine(int16_t x, int16_t y, int16_t h, SSD1306_COLOR_t color)
{
	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	if (h == 0)
	{
		return SSD1306_OK;
	}

	if (h < 0) { y += h + 1; h = -h; }

	return ssd1306_FillVLine(x, y, y + h - 1, color);
}

/**
 * @brief Dibuja una línea horizontal de forma optimizada (sin Bresenham).
 *
 * @param x     Coordenada X inicial.
 * @param y     Coordenada Y de la línea.
 * @param w     Ancho de la línea (puede ser negativo, se normaliza).
 * @param color Color de la línea.
 * @return SSD1306_Status_t
 */
SSD1306_Status_t SSD1306_DrawFastHLine(int16_t x, int16_t y, int16_t w, SSD1306_COLOR_t color)
{
	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	if (w == 0)
	{
		return SSD1306_OK;
	}

	if (w < 0) { x += w + 1; w = -w; }

	return ssd1306_FillHLine(x, x + w - 1, y, color);
}

SSD1306_Status_t SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c)
{
	SSD1306_Status_t st;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	if (x >= SSD1306_State.Width || y >= SSD1306_State.Height)
	{
		return SSD1306_INVALID_PARAM;
	}

	/* Check width and height */
	if ((x + w) >= SSD1306_State.Width)  { w = SSD1306_State.Width - x;  }
	if ((y + h) >= SSD1306_State.Height) { h = SSD1306_State.Height - y; }

	/* Draw 4 lines */
	st  = SSD1306_DrawLine(x, y, x + w, y, c);                 /* Top line */
	st  = st ? st : SSD1306_DrawLine(x, y + h, x + w, y + h, c); /* Bottom line */
	st  = st ? st : SSD1306_DrawLine(x, y, x, y + h, c);         /* Left line */
	st  = st ? st : SSD1306_DrawLine(x + w, y, x + w, y + h, c); /* Right line */

	return st;
}

SSD1306_Status_t SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c)
{
	SSD1306_Status_t st;
	uint8_t i;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	if (x >= SSD1306_State.Width || y >= SSD1306_State.Height)
	{
		return SSD1306_INVALID_PARAM;
	}

	/* Check width and height */
	if ((x + w) >= SSD1306_State.Width)  { w = SSD1306_State.Width - x;  }
	if ((y + h) >= SSD1306_State.Height) { h = SSD1306_State.Height - y; }

	/* Draw lines */
	for (i = 0; i <= h; i++)
	{
		st = SSD1306_DrawLine(x, y + i, x + w, y + i, c);
		if (st != SSD1306_OK)
		{
			return st;
		}
	}

	return SSD1306_OK;
}

SSD1306_Status_t SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color)
{
	SSD1306_Status_t st;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	/* Draw lines */
	st  = SSD1306_DrawLine(x1, y1, x2, y2, color);
	st  = st ? st : SSD1306_DrawLine(x2, y2, x3, y3, color);
	st  = st ? st : SSD1306_DrawLine(x3, y3, x1, y1, color);

	return st;
}

SSD1306_Status_t SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color)
{
	SSD1306_Status_t st;
	int16_t ax = (int16_t)x1, ay = (int16_t)y1;
	int16_t bx = (int16_t)x2, by = (int16_t)y2;
	int16_t cx = (int16_t)x3, cy = (int16_t)y3;
	int16_t a, b, y, last, tmp;
	int16_t dxab, dyab, dxac, dyac, dxbc, dybc;
	int32_t sa, sb;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	/* Ordenar vértices por Y ascendente: ay <= by <= cy */
	if (ay > by) { tmp = ay; ay = by; by = tmp; tmp = ax; ax = bx; bx = tmp; }
	if (by > cy) { tmp = by; by = cy; cy = tmp; tmp = bx; bx = cx; cx = tmp; }
	if (ay > by) { tmp = ay; ay = by; by = tmp; tmp = ax; ax = bx; bx = tmp; }

	if (ay == cy)
	{
		/* Triángulo degenerado: los 3 vértices están en la misma fila */
		a = b = ax;
		if (bx < a)      { a = bx; }
		else if (bx > b) { b = bx; }
		if (cx < a)      { a = cx; }
		else if (cx > b) { b = cx; }

		return ssd1306_FillHLine(a, b, ay, color);
	}

	dxab = bx - ax; dyab = by - ay;
	dxac = cx - ax; dyac = cy - ay;
	dxbc = cx - bx; dybc = cy - by;
	sa = 0;
	sb = 0;

	/* Mitad superior: bordes A-B y A-C (se omite la fila by si A-B no es plano) */
	last = (by == cy) ? by : (int16_t)(by - 1);
	for (y = ay; y <= last; y++)
	{
		a = ax + (int16_t)(sa / dyab);
		b = ax + (int16_t)(sb / dyac);
		sa += dxab;
		sb += dxac;

		st = ssd1306_FillHLine(a, b, y, color);
		if (st != SSD1306_OK)
		{
			return st;
		}
	}

	/* Mitad inferior: bordes B-C y A-C */
	sa = (int32_t)dxbc * (y - by);
	sb = (int32_t)dxac * (y - ay);
	for (; y <= cy; y++)
	{
		a = bx + (int16_t)(sa / dybc);
		b = ax + (int16_t)(sb / dyac);
		sa += dxbc;
		sb += dxac;

		st = ssd1306_FillHLine(a, b, y, color);
		if (st != SSD1306_OK)
		{
			return st;
		}
	}

	return SSD1306_OK;
}

SSD1306_Status_t SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c)
{
	SSD1306_Status_t st;
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	st  = SSD1306_DrawPixel(x0, y0 + r, c);
	st  = st ? st : SSD1306_DrawPixel(x0, y0 - r, c);
	st  = st ? st : SSD1306_DrawPixel(x0 + r, y0, c);
	st  = st ? st : SSD1306_DrawPixel(x0 - r, y0, c);
	if (st != SSD1306_OK)
	{
		return st;
	}

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		st  = SSD1306_DrawPixel(x0 + x, y0 + y, c);
		st  = st ? st : SSD1306_DrawPixel(x0 - x, y0 + y, c);
		st  = st ? st : SSD1306_DrawPixel(x0 + x, y0 - y, c);
		st  = st ? st : SSD1306_DrawPixel(x0 - x, y0 - y, c);

		st  = st ? st : SSD1306_DrawPixel(x0 + y, y0 + x, c);
		st  = st ? st : SSD1306_DrawPixel(x0 - y, y0 + x, c);
		st  = st ? st : SSD1306_DrawPixel(x0 + y, y0 - x, c);
		st  = st ? st : SSD1306_DrawPixel(x0 - y, y0 - x, c);
		if (st != SSD1306_OK)
		{
			return st;
		}
	}

	return SSD1306_OK;
}

SSD1306_Status_t SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c)
{
	SSD1306_Status_t st;
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	st  = SSD1306_DrawPixel(x0, y0 + r, c);
	st  = st ? st : SSD1306_DrawPixel(x0, y0 - r, c);
	st  = st ? st : SSD1306_DrawPixel(x0 + r, y0, c);
	st  = st ? st : SSD1306_DrawPixel(x0 - r, y0, c);
	st  = st ? st : ssd1306_FillHLine(x0 - r, x0 + r, y0, c);
	if (st != SSD1306_OK)
	{
		return st;
	}

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		st  = ssd1306_FillHLine(x0 - x, x0 + x, y0 + y, c);
		st  = st ? st : ssd1306_FillHLine(x0 - x, x0 + x, y0 - y, c);

		st  = st ? st : ssd1306_FillHLine(x0 - y, x0 + y, y0 + x, c);
		st  = st ? st : ssd1306_FillHLine(x0 - y, x0 + y, y0 - x, c);
		if (st != SSD1306_OK)
		{
			return st;
		}
	}

	return SSD1306_OK;
}

SSD1306_Status_t SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color)
{
	SSD1306_Status_t st;
	int16_t byteWidth = (w + 7) / 8; /* Bitmap scanline pad = whole byte */
	uint8_t byte = 0;
	int16_t i, j;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }
	if (bitmap == NULL)            { return SSD1306_INVALID_PARAM;   }

	for (j = 0; j < h; j++, y++)
	{
		for (i = 0; i < w; i++)
		{
			if (i & 7)
			{
				byte <<= 1;
			}
			else
			{
				byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
			}
			if (byte & 0x80)
			{
				st = SSD1306_DrawPixel(x + i, y, (SSD1306_COLOR_t)color);
				if (st != SSD1306_OK)
				{
					return st;
				}
			}
		}
	}

	return SSD1306_OK;
}

SSD1306_Status_t SSD1306_Clear(void)
{
	SSD1306_Status_t st = SSD1306_Fill(SSD1306_COLOR_BLACK);
	return (st != SSD1306_OK) ? st : SSD1306_UpdateScreen();
}

SSD1306_Status_t SSD1306_ON(void)
{
	SSD1306_Status_t st;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	st  = SSD1306_WRITECOMMAND(0x8D);
	st  = st ? st : SSD1306_WRITECOMMAND(0x14);
	st  = st ? st : SSD1306_WRITECOMMAND(0xAF);

	return st;
}

SSD1306_Status_t SSD1306_OFF(void)
{
	SSD1306_Status_t st;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	st  = SSD1306_WRITECOMMAND(0x8D);
	st  = st ? st : SSD1306_WRITECOMMAND(0x10);
	st  = st ? st : SSD1306_WRITECOMMAND(0xAE);

	return st;
}

SSD1306_Status_t SSD1306_InvertDisplay(int i)
{
	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	return i ? SSD1306_WRITECOMMAND(SSD1306_INVERTDISPLAY) : SSD1306_WRITECOMMAND(SSD1306_NORMALDISPLAY);
}

SSD1306_Status_t SSD1306_ScrollRight(uint8_t start_row, uint8_t end_row)
{
	SSD1306_Status_t st;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	st  = SSD1306_WRITECOMMAND(SSD1306_RIGHT_HORIZONTAL_SCROLL); // send 0x26
	st  = st ? st : SSD1306_WRITECOMMAND(0x00);        // send dummy
	st  = st ? st : SSD1306_WRITECOMMAND(start_row);   // start page address
	st  = st ? st : SSD1306_WRITECOMMAND(0X00);        // time interval 5 frames
	st  = st ? st : SSD1306_WRITECOMMAND(end_row);     // end page address
	st  = st ? st : SSD1306_WRITECOMMAND(0X00);
	st  = st ? st : SSD1306_WRITECOMMAND(0XFF);
	st  = st ? st : SSD1306_WRITECOMMAND(SSD1306_ACTIVATE_SCROLL); // start scroll

	return st;
}

SSD1306_Status_t SSD1306_ScrollLeft(uint8_t start_row, uint8_t end_row)
{
	SSD1306_Status_t st;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	st  = SSD1306_WRITECOMMAND(SSD1306_LEFT_HORIZONTAL_SCROLL); // send 0x27
	st  = st ? st : SSD1306_WRITECOMMAND(0x00);        // send dummy
	st  = st ? st : SSD1306_WRITECOMMAND(start_row);   // start page address
	st  = st ? st : SSD1306_WRITECOMMAND(0X00);        // time interval 5 frames
	st  = st ? st : SSD1306_WRITECOMMAND(end_row);     // end page address
	st  = st ? st : SSD1306_WRITECOMMAND(0X00);
	st  = st ? st : SSD1306_WRITECOMMAND(0XFF);
	st  = st ? st : SSD1306_WRITECOMMAND(SSD1306_ACTIVATE_SCROLL); // start scroll

	return st;
}

SSD1306_Status_t SSD1306_Scrolldiagright(uint8_t start_row, uint8_t end_row)
{
	SSD1306_Status_t st;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	st  = SSD1306_WRITECOMMAND(SSD1306_SET_VERTICAL_SCROLL_AREA); // select the area
	st  = st ? st : SSD1306_WRITECOMMAND(0x00);   // write dummy
	st  = st ? st : SSD1306_WRITECOMMAND(SSD1306_State.Height);

	st  = st ? st : SSD1306_WRITECOMMAND(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
	st  = st ? st : SSD1306_WRITECOMMAND(0x00);
	st  = st ? st : SSD1306_WRITECOMMAND(start_row);
	st  = st ? st : SSD1306_WRITECOMMAND(0X00);
	st  = st ? st : SSD1306_WRITECOMMAND(end_row);
	st  = st ? st : SSD1306_WRITECOMMAND(0x01);
	st  = st ? st : SSD1306_WRITECOMMAND(SSD1306_ACTIVATE_SCROLL);

	return st;
}

SSD1306_Status_t SSD1306_Scrolldiagleft(uint8_t start_row, uint8_t end_row)
{
	SSD1306_Status_t st;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	st  = SSD1306_WRITECOMMAND(SSD1306_SET_VERTICAL_SCROLL_AREA); // select the area
	st  = st ? st : SSD1306_WRITECOMMAND(0x00);   // write dummy
	st  = st ? st : SSD1306_WRITECOMMAND(SSD1306_State.Height);

	st  = st ? st : SSD1306_WRITECOMMAND(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
	st  = st ? st : SSD1306_WRITECOMMAND(0x00);
	st  = st ? st : SSD1306_WRITECOMMAND(start_row);
	st  = st ? st : SSD1306_WRITECOMMAND(0X00);
	st  = st ? st : SSD1306_WRITECOMMAND(end_row);
	st  = st ? st : SSD1306_WRITECOMMAND(0x01);
	st  = st ? st : SSD1306_WRITECOMMAND(SSD1306_ACTIVATE_SCROLL);

	return st;
}

SSD1306_Status_t SSD1306_Stopscroll(void)
{
	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }

	return SSD1306_WRITECOMMAND(SSD1306_DEACTIVATE_SCROLL);
}

/**
 * @brief Dibuja una elipse (solo borde) mediante el algoritmo de punto medio.
 *
 * @param x0    Coordenada X del centro.
 * @param y0    Coordenada Y del centro.
 * @param rx    Radio horizontal.
 * @param ry    Radio vertical.
 * @param color Color del borde.
 * @return SSD1306_Status_t
 */
SSD1306_Status_t SSD1306_DrawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, SSD1306_COLOR_t color)
{
	SSD1306_Status_t st;
	int32_t a2, b2, fa2, fb2, sigma, x, y;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }
	if (rx < 0 || ry < 0)          { return SSD1306_INVALID_PARAM;   }

	if (rx == 0)
	{
		return ssd1306_FillVLine(x0, y0 - ry, y0 + ry, color);
	}
	if (ry == 0)
	{
		return ssd1306_FillHLine(x0 - rx, x0 + rx, y0, color);
	}

	a2 = (int32_t)rx * rx;
	b2 = (int32_t)ry * ry;
	fa2 = 4 * a2;
	fb2 = 4 * b2;

	/* Región 1: pendiente |dy/dx| <= 1 */
	for (x = 0, y = ry, sigma = 2 * b2 + a2 * (1 - 2 * ry); b2 * x <= a2 * y; x++)
	{
		st  = SSD1306_DrawPixel(x0 + x, y0 + y, color);
		st  = st ? st : SSD1306_DrawPixel(x0 - x, y0 + y, color);
		st  = st ? st : SSD1306_DrawPixel(x0 + x, y0 - y, color);
		st  = st ? st : SSD1306_DrawPixel(x0 - x, y0 - y, color);
		if (st != SSD1306_OK)
		{
			return st;
		}

		if (sigma >= 0)
		{
			sigma += fa2 * (1 - y);
			y--;
		}
		sigma += b2 * (4 * x + 6);
	}

	/* Región 2: pendiente |dy/dx| > 1 */
	for (x = rx, y = 0, sigma = 2 * a2 + b2 * (1 - 2 * rx); a2 * y <= b2 * x; y++)
	{
		st  = SSD1306_DrawPixel(x0 + x, y0 + y, color);
		st  = st ? st : SSD1306_DrawPixel(x0 - x, y0 + y, color);
		st  = st ? st : SSD1306_DrawPixel(x0 + x, y0 - y, color);
		st  = st ? st : SSD1306_DrawPixel(x0 - x, y0 - y, color);
		if (st != SSD1306_OK)
		{
			return st;
		}

		if (sigma >= 0)
		{
			sigma += fb2 * (1 - x);
			x--;
		}
		sigma += a2 * (4 * y + 6);
	}

	return SSD1306_OK;
}

/**
 * @brief Dibuja una elipse
 * 
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] rx    Radio horizontal.
 * @param[in] ry    Radio vertical.
 * @param[in] color Color del relleno.
 * @return SSD1306_Status_t 
 */
SSD1306_Status_t SSD1306_DrawFilledEllipse(int16_t x, int16_t y, int16_t rx, int16_t ry, uint16_t color)
{
	SSD1306_Status_t st;
	int32_t a2, b2, fa2, fb2, sigma, dx, dy;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }
	if (rx < 0 || ry < 0)          { return SSD1306_INVALID_PARAM;   }

	if (rx == 0)
	{
		return ssd1306_FillVLine(x, y - ry, y + ry, (SSD1306_COLOR_t)color);
	}
	if (ry == 0)
	{
		return ssd1306_FillHLine(x - rx, x + rx, y, (SSD1306_COLOR_t)color);
	}

	a2 = (int32_t)rx * rx;
	b2 = (int32_t)ry * ry;
	fa2 = 4 * a2;
	fb2 = 4 * b2;

	/* Región 1: pendiente |dy/dx| <= 1 */
	for (dx = 0, dy = ry, sigma = 2 * b2 + a2 * (1 - 2 * ry); b2 * dx <= a2 * dy; dx++)
	{
		st  = ssd1306_FillHLine(x - dx, x + dx, y + dy, (SSD1306_COLOR_t)color);
		st  = st ? st : ssd1306_FillHLine(x - dx, x + dx, y - dy, (SSD1306_COLOR_t)color);
		if (st != SSD1306_OK)
		{
			return st;
		}

		if (sigma >= 0)
		{
			sigma += fa2 * (1 - dy);
			dy--;
		}
		sigma += b2 * (4 * dx + 6);
	}

	/* Región 2: pendiente |dy/dx| > 1 */
	for (dx = rx, dy = 0, sigma = 2 * a2 + b2 * (1 - 2 * rx); a2 * dy <= b2 * dx; dy++)
	{
		st  = ssd1306_FillHLine(x - dx, x + dx, y + dy, (SSD1306_COLOR_t)color);
		st  = st ? st : ssd1306_FillHLine(x - dx, x + dx, y - dy, (SSD1306_COLOR_t)color);
		if (st != SSD1306_OK)
		{
			return st;
		}

		if (sigma >= 0)
		{
			sigma += fb2 * (1 - dx);
			dx--;
		}
		sigma += a2 * (4 * dy + 6);
	}

	return SSD1306_OK;
}

/**
 * @brief Dibuja el contorno de un arco (sector de anillo) entre dos ángulos.
 *
 * @param[in] x     Coordenada X del centro.
 * @param[in] y     Coordenada Y del centro.
 * @param[in] r1    Radio exterior del arco.
 * @param[in] r2    Radio interior del arco.
 * @param[in] start Ángulo inicial en grados (0° = derecha, sentido horario).
 * @param[in] end   Ángulo final en grados.
 * @param[in] color Color del contorno.
 * @return SSD1306_Status_t
 */
SSD1306_Status_t SSD1306_DrawArc(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, SSD1306_COLOR_t color)
{
	SSD1306_Status_t st;
	bool equal;
	int16_t tmp;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }
	if (r1 < 0 || r2 < 0)          { return SSD1306_INVALID_PARAM;   }

	if (r1 < r2) { tmp = r1; r1 = r2; r2 = tmp; }
	if (r1 < 1)  { r1 = 1; }
	if (r2 < 1)  { r2 = 1; }

	equal = fabsf(start - end) < FLT_EPSILON;
	start = fmodf(start, 360.0f);
	end   = fmodf(end, 360.0f);
	if (start < 0) { start += 360.0f; }
	if (end < 0)   { end   += 360.0f; }

	/* Bordes rectos del arco (en start y en end) */
	st  = ssd1306_FillArcHelper(x, y, r1, r2, start, start, color);
	st  = st ? st : ssd1306_FillArcHelper(x, y, r1, r2, end, end, color);
	if (st != SSD1306_OK)
	{
		return st;
	}

	if (!equal && (fabsf(start - end) <= 0.0001f))
	{
		start = 0.0f;
		end   = 360.0f;
	}

	/* Bordes curvos del arco (radio exterior e interior) */
	st  = ssd1306_FillArcHelper(x, y, r1, r1, start, end, color);
	st  = st ? st : ssd1306_FillArcHelper(x, y, r2, r2, start, end, color);

	return st;
}

/**
 * @brief Dibuja un arco (sector de anillo) entre dos ángulos.
 * 
 * @param[in] x     Coordenada X del centro.
 * @param[in] y     Coordenada Y del centro.
 * @param[in] r1    Radio exterior del arco.
 * @param[in] r2    Radio interior del arco.
 * @param[in] start Ángulo inicial en grados (0° = derecha, sentido horario).
 * @param[in] end   Ángulo final en grados.
 * @param color     Color del relleno
 * @return SSD1306_Status_t 
 */
SSD1306_Status_t SSD1306_DrawFilledArc(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, uint16_t color)
{
	bool equal;
	int16_t tmp;

	if (SSD1306_Initialized != 1U) { return SSD1306_NOT_INITIALIZED; }
	if (r1 < 0 || r2 < 0)          { return SSD1306_INVALID_PARAM;   }

	if (r1 < r2) { tmp = r1; r1 = r2; r2 = tmp; }
	if (r1 < 1)  { r1 = 1; }
	if (r2 < 1)  { r2 = 1; }

	equal = fabsf(start - end) < FLT_EPSILON;
	start = fmodf(start, 360.0f);
	end   = fmodf(end, 360.0f);
	if (start < 0) { start += 360.0f; }
	if (end < 0)   { end   += 360.0f; }

	if (!equal && (fabsf(start - end) <= 0.0001f))
	{
		start = 0.0f;
		end   = 360.0f;
	}

	return ssd1306_FillArcHelper(x, y, r1, r2, start, end, (SSD1306_COLOR_t)color);
}