 /**
 * @file SSD1681.c
 * @brief Librería para la gestión de pantallas e-paper (SSD1680/SSD1681/SSD1683) mediante comunicación SPI.
 *
 * @details Driver para pantallas e-paper monocromáticas, soporta inicialización,
 *          primitivas de dibujo (píxeles, líneas, rectángulos, triángulos,
 *          círculos, bitmaps) y texto con fuentes personalizadas.
 *
 * @author Daniel Ruiz
 * @date Octubre 29, 2026
 * @version 0.1.0
 */

#include "SSD1681.h"

// ============================================================================
// COMANDOS DEL CONTROLADOR (datasheet SSD1681 de Solomon Systech)
// ============================================================================

#define SSD1681_CMD_DRIVER_OUTPUT   0x01U /**< Número de compuertas (filas) y orden de barrido */
#define SSD1681_CMD_DATA_ENTRY      0x11U /**< Dirección de avance de los contadores de RAM    */
#define SSD1681_CMD_SW_RESET        0x12U /**< Reset por software                              */
#define SSD1681_CMD_TEMP_SENSOR     0x18U /**< Selección del sensor de temperatura             */
#define SSD1681_CMD_BORDER          0x3CU /**< Forma de onda del borde                         */
#define SSD1681_CMD_RAM_X_RANGE     0x44U /**< Ventana de RAM en X (en bytes)                  */
#define SSD1681_CMD_RAM_Y_RANGE     0x45U /**< Ventana de RAM en Y (en filas)                  */
#define SSD1681_CMD_RAM_X_COUNTER   0x4EU /**< Contador de dirección X                         */
#define SSD1681_CMD_RAM_Y_COUNTER   0x4FU /**< Contador de dirección Y                         */
#define SSD1681_CMD_MASTER_ACTIVATE 0x20U /**< Ejecuta la secuencia de actualización          */
#define SSD1681_CMD_UPDATE_CTRL2    0x22U /**< Secuencia de actualización del panel            */
#define SSD1681_CMD_WRITE_RAM_BW    0x24U /**< Escritura en la RAM blanco/negro                */

#define SSD1681_DATA_ENTRY_XY_INC   0x03U /**< X+ y Y+, avanza en X (por filas)                */
#define SSD1681_BORDER_WHITE        0x05U /**< El borde sigue a la LUT del blanco              */
#define SSD1681_TEMP_INTERNAL       0x80U /**< Sensor de temperatura interno                   */
#define SSD1681_UPDATE_FULL         0xF7U /**< Clock+analog on, carga temp+LUT, display, off   */

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static SPI_HandleTypeDef* SSD1681_hspi        = NULL; /**< Handle de SPI utilizado para comunicarse con el SSD1681 */
static uint8_t            SSD1681_Initialized = 0U;   /**< Bandera para verificar si el módulo está inicializado */

/* -- Pines de control (parametrizados en SSD1681_Init) -- */
static GPIO_TypeDef* SSD1681_CS_Port   = NULL; /**< Puerto GPIO del pin CS   */
static uint16_t      SSD1681_CS_Pin    = 0U;   /**< Pin GPIO del pin CS      */
static GPIO_TypeDef* SSD1681_DC_Port   = NULL; /**< Puerto GPIO del pin D/C  */
static uint16_t      SSD1681_DC_Pin    = 0U;   /**< Pin GPIO del pin D/C     */
static GPIO_TypeDef* SSD1681_RST_Port  = NULL; /**< Puerto GPIO del pin RST  */
static uint16_t      SSD1681_RST_Pin   = 0U;   /**< Pin GPIO del pin RST     */
static GPIO_TypeDef* SSD1681_BUSY_Port = NULL; /**< Puerto GPIO del pin BUSY */
static uint16_t      SSD1681_BUSY_Pin  = 0U;   /**< Pin GPIO del pin BUSY    */

static uint16_t SSD1681_Width  = 0U; /**< Ancho activo en píxeles */
static uint16_t SSD1681_Height = 0U; /**< Alto activo en píxeles  */

/** Buffer 1 bpp, por filas, MSB = píxel de la izquierda (1 = blanco, 0 = negro) */
static uint8_t SSD1681_Buffer[((SSD1681_MAX_WIDTH + 7U) / 8U) * SSD1681_MAX_HEIGHT];

/** Estado interno del cursor de escritura y de la inversión de color */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t  Inverted;
} SSD1681_State_t;

static SSD1681_State_t SSD1681_State = {0}; /**< Cursor de escritura y estado de inversión */

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Envía un comando y, opcionalmente, sus datos en una sola transacción SPI.
 *
 * @param[in] cmd    Comando a enviar (D/C en bajo).
 * @param[in] data   Datos del comando (D/C en alto); puede ser NULL si @p size es 0.
 * @param[in] size   Número de bytes de datos.
 * @return SSD1681_OK si la transmisión fue exitosa; SSD1681_ERROR en caso contrario.
 */
static SSD1681_Status_t ssd1681_SendCommand(uint8_t cmd, const uint8_t* data, uint16_t size)
{
	HAL_StatusTypeDef hst;

	HAL_GPIO_WritePin(SSD1681_DC_Port, SSD1681_DC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SSD1681_CS_Port, SSD1681_CS_Pin, GPIO_PIN_RESET);
	hst = HAL_SPI_Transmit(SSD1681_hspi, &cmd, 1U, SSD1681_SPI_TIMEOUT_MS);

	if (hst == HAL_OK && size > 0U && data != NULL)
	{
		HAL_GPIO_WritePin(SSD1681_DC_Port, SSD1681_DC_Pin, GPIO_PIN_SET);
		hst = HAL_SPI_Transmit(SSD1681_hspi, (uint8_t*)data, size, SSD1681_SPI_TIMEOUT_MS);
	}

	HAL_GPIO_WritePin(SSD1681_CS_Port, SSD1681_CS_Pin, GPIO_PIN_SET);
	return (hst == HAL_OK) ? SSD1681_OK : SSD1681_ERROR;
}

/**
 * @brief Espera a que el controlador libere el pin BUSY.
 *
 * @param[in] timeout_ms Tiempo máximo de espera en milisegundos.
 * @return SSD1681_OK si BUSY se liberó; SSD1681_TIMEOUT si se agotó el tiempo.
 */
static SSD1681_Status_t ssd1681_WaitBusy(uint32_t timeout_ms)
{
	uint32_t tickstart = HAL_GetTick();
	while (HAL_GPIO_ReadPin(SSD1681_BUSY_Port, SSD1681_BUSY_Pin) == SSD1681_BUSY_ACTIVE_LEVEL)
	{
		if ((HAL_GetTick() - tickstart) >= timeout_ms)
		{
			return SSD1681_TIMEOUT;
		}
	}
	return SSD1681_OK;
}

/**
 * @brief Pulso de reset por hardware en el pin RST (activo en bajo).
 */
static void ssd1681_HardwareReset(void)
{
	HAL_GPIO_WritePin(SSD1681_RST_Port, SSD1681_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(SSD1681_RESET_PULSE_MS);
	HAL_GPIO_WritePin(SSD1681_RST_Port, SSD1681_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(SSD1681_RESET_PULSE_MS);
}

/**
 * @brief Rellena un segmento vertical directamente en el buffer de pantalla.
 *
 * @details Usado por SSD1681_DrawFastVLine.
 *
 * @param x     Coordenada X del segmento.
 * @param y0    Coordenada Y inicial (puede exceder los límites, se recorta).
 * @param y1    Coordenada Y final (puede exceder los límites, se recorta).
 * @param color Color a utilizar.
 *
 * @return SSD1681_Status_t Estado de la operación.
 */
static SSD1681_Status_t ssd1681_FillVLine(int16_t x, int16_t y0, int16_t y1, SSD1681_COLOR_t color)
{
	SSD1681_Status_t st;
	int16_t y, tmp;

	if (x < 0 || x >= (int16_t)SSD1681_Width)
	{
		return SSD1681_OK;
	}

	if (y0 > y1) { tmp = y0; y0 = y1; y1 = tmp; }
	if (y0 < 0) { y0 = 0; }
	if (y1 >= (int16_t)SSD1681_Height) { y1 = (int16_t)SSD1681_Height - 1; }

	for (y = y0; y <= y1; y++)
	{
		st = SSD1681_DrawPixel((uint16_t)x, (uint16_t)y, color);
		if (st != SSD1681_OK)
		{
			return st;
		}
	}

	return SSD1681_OK;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

SSD1681_Status_t SSD1681_Init(SPI_HandleTypeDef* hspi,
                            GPIO_TypeDef* csPort, uint16_t csPin,
                            GPIO_TypeDef* dcPort, uint16_t dcPin,
                            GPIO_TypeDef* rstPort, uint16_t rstPin,
                            GPIO_TypeDef* busyPort, uint16_t busyPin,
                            uint16_t width, uint16_t height)
{
	SSD1681_Status_t st;

	if (hspi == NULL || csPort == NULL || dcPort == NULL || rstPort == NULL || busyPort == NULL ||
		width == 0U || height == 0U || width > SSD1681_MAX_WIDTH || height > SSD1681_MAX_HEIGHT)
	{
		return SSD1681_INVALID_PARAM;
	}

	SSD1681_hspi = hspi;
	SSD1681_CS_Port   = csPort;   SSD1681_CS_Pin   = csPin;
	SSD1681_DC_Port   = dcPort;   SSD1681_DC_Pin   = dcPin;
	SSD1681_RST_Port  = rstPort;  SSD1681_RST_Pin  = rstPin;
	SSD1681_BUSY_Port = busyPort; SSD1681_BUSY_Pin = busyPin;
	SSD1681_Width  = width;
	SSD1681_Height = height;
	SSD1681_Initialized = 0U;

	HAL_GPIO_WritePin(SSD1681_CS_Port, SSD1681_CS_Pin, GPIO_PIN_SET);

	ssd1681_HardwareReset();
	st = ssd1681_WaitBusy(SSD1681_BUSY_TIMEOUT_MS);
	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_SW_RESET, NULL, 0U);
	st = st ? st : ssd1681_WaitBusy(SSD1681_BUSY_TIMEOUT_MS);

	/* Compuertas = filas del panel y ventana de RAM del panel completo,
	 * recorrida por filas desde la esquina superior izquierda. */
	const uint16_t lastRow  = height - 1U;
	const uint8_t  lastByte = (uint8_t)(((width + 7U) / 8U) - 1U);
	const uint8_t  driverOutput[] = { (uint8_t)(lastRow & 0xFFU), (uint8_t)(lastRow >> 8), 0x00U };
	const uint8_t  dataEntry      = SSD1681_DATA_ENTRY_XY_INC;
	const uint8_t  xRange[]       = { 0x00U, lastByte };
	const uint8_t  yRange[]       = { 0x00U, 0x00U, (uint8_t)(lastRow & 0xFFU), (uint8_t)(lastRow >> 8) };
	const uint8_t  border         = SSD1681_BORDER_WHITE;
	const uint8_t  tempSensor     = SSD1681_TEMP_INTERNAL;
	const uint8_t  origin[]       = { 0x00U, 0x00U };

	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_DRIVER_OUTPUT, driverOutput, sizeof(driverOutput));
	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_DATA_ENTRY,    &dataEntry,   1U);
	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_RAM_X_RANGE,   xRange,       sizeof(xRange));
	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_RAM_Y_RANGE,   yRange,       sizeof(yRange));
	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_BORDER,        &border,      1U);
	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_TEMP_SENSOR,   &tempSensor,  1U);
	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_RAM_X_COUNTER, origin,       1U);
	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_RAM_Y_COUNTER, origin,       2U);
	st = st ? st : ssd1681_WaitBusy(SSD1681_BUSY_TIMEOUT_MS);

	if (st == SSD1681_OK)
	{
		SSD1681_Initialized = 1U;
	}
	return st;
}

/**
 * @brief Obtiene el ancho de pantalla configurado en SSD1681_Init.
 *
 * @return uint16_t Ancho en píxeles, o 0 si el módulo no ha sido inicializado.
 */
uint16_t SSD1681_GetWidth(void)
{
	if (SSD1681_Initialized != 1U) { return 0U; }

	return SSD1681_Width;
}

/**
 * @brief Obtiene el alto de pantalla configurado en SSD1681_Init.
 *
 * @return uint16_t Alto en píxeles, o 0 si el módulo no ha sido inicializado.
 */
uint16_t SSD1681_GetHeight(void)
{
	if (SSD1681_Initialized != 1U) { return 0U; }

	return SSD1681_Height;
}

/**
 * @brief Envía el contenido del buffer interno a la pantalla física.
 * 
 * @return SSD1681_Status_t 
 */
SSD1681_Status_t SSD1681_UpdateScreen(void)
{
	SSD1681_Status_t st;
	const uint8_t  origin[] = { 0x00U, 0x00U };
	const uint8_t  update   = SSD1681_UPDATE_FULL;
	const uint16_t size     = (uint16_t)(((SSD1681_Width + 7U) / 8U) * SSD1681_Height);

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	st = ssd1681_SendCommand(SSD1681_CMD_RAM_X_COUNTER, origin, 1U);
	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_RAM_Y_COUNTER, origin, 2U);
	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_WRITE_RAM_BW, SSD1681_Buffer, size);
	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_UPDATE_CTRL2, &update, 1U);
	st = st ? st : ssd1681_SendCommand(SSD1681_CMD_MASTER_ACTIVATE, NULL, 0U);
	st = st ? st : ssd1681_WaitBusy(SSD1681_BUSY_TIMEOUT_MS);

	return st;
}

/**
 * @brief Invierte el color de todos los píxeles del buffer (negro<->blanco).
 * 
 * @return SSD1681_Status_t 
 */
SSD1681_Status_t SSD1681_ToggleInvert(void)
{
	uint32_t i;
	const uint32_t size = ((SSD1681_Width + 7U) / 8U) * (uint32_t)SSD1681_Height;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	/* Invertir toggle */
	SSD1681_State.Inverted = (uint8_t)!SSD1681_State.Inverted;

	/* Hacer toggle de memoria */
	for (i = 0U; i < size; i++)
	{
		SSD1681_Buffer[i] = (uint8_t)~SSD1681_Buffer[i];
	}

	return SSD1681_OK;
}

/**
 * @brief Rellena todo el buffer con un color sólido.
 * 
 * @param[in] color Color de relleno (SSD1681_COLOR_BLACK o SSD1681_COLOR_WHITE).
 * @return SSD1681_Status_t
 */
SSD1681_Status_t SSD1681_Fill(SSD1681_COLOR_t color)
{
	const uint32_t size = ((SSD1681_Width + 7U) / 8U) * (uint32_t)SSD1681_Height;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	/* Respetar la inversión activa */
	if (SSD1681_State.Inverted) { color = (color == SSD1681_COLOR_BLACK) ? SSD1681_COLOR_WHITE : SSD1681_COLOR_BLACK; }

	/* Configurar memoria */
	memset(SSD1681_Buffer, (color == SSD1681_COLOR_BLACK) ? 0x00 : 0xFF, size);

	return SSD1681_OK;
}

/**
 * @brief Dibuja un píxel individual en el buffer.
 * 
 * @param[in] x     Coordenada X (0..ancho configurado en SSD1681_Init - 1).
 * @param[in] y     Coordenada Y (0..alto configurado en SSD1681_Init - 1).
 * @param[in] color Color del píxel.
 * @return SSD1681_Status_t
 */
SSD1681_Status_t SSD1681_DrawPixel(uint16_t x, uint16_t y, SSD1681_COLOR_t color)
{
	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	if (x >= SSD1681_Width || y >= SSD1681_Height)
	{
		return SSD1681_INVALID_PARAM;
	}

	/* Respetar la inversión activa */
	if (SSD1681_State.Inverted) { color = (color == SSD1681_COLOR_BLACK) ? SSD1681_COLOR_WHITE : SSD1681_COLOR_BLACK; }

	/* Buffer por filas: byte = x/8 dentro de la fila y, bit 7 = píxel de la izquierda */
	const uint32_t index = (uint32_t)y * ((SSD1681_Width + 7U) / 8U) + (x / 8U);
	const uint8_t  mask  = (uint8_t)(0x80U >> (x % 8U));

	if (color == SSD1681_COLOR_WHITE)
	{
		SSD1681_Buffer[index] |= mask;
	}
	else
	{
		SSD1681_Buffer[index] &= (uint8_t)~mask;
	}

	return SSD1681_OK;
}

/**
 * @brief Posiciona el cursor de escritura de texto.
 * 
 * @param[in] x Coordenada X de inicio.
 * @param[in] y Coordenada Y de inicio.
 * @return SSD1681_Status_t
 */
SSD1681_Status_t SSD1681_GotoXY(uint16_t x, uint16_t y)
{
	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	if (x >= SSD1681_Width || y >= SSD1681_Height)
	{
		return SSD1681_INVALID_PARAM;
	}

	/* Configurar puntos */
	SSD1681_State.CurrentX = x;
	SSD1681_State.CurrentY = y;

	return SSD1681_OK;
}

/**
 * @brief Dibuja un carácter en la posición actual del cursor.
 * 
 * @param[in] ch    Carácter a dibujar.
 * @param[in] Font  Fuente a utilizar.
 * @param[in] color Color del carácter.
 * @return SSD1681_Status_t
 */
SSD1681_Status_t SSD1681_Putc(char ch, FontDef_t* Font, SSD1681_COLOR_t color)
{
	SSD1681_Status_t st;
	uint32_t i, b, j;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }
	if (Font == NULL)              { return SSD1681_INVALID_PARAM;   }

	/* Solo ASCII imprimible (la tabla de la fuente inicia en 32) */
	if (ch < 32 || ch > 126)       { return SSD1681_INVALID_PARAM;   }

	/* Revisar espacio disponible */
	if (
		SSD1681_Width  < (SSD1681_State.CurrentX + Font->FontWidth) ||
		SSD1681_Height < (SSD1681_State.CurrentY + Font->FontHeight)
	) {
		return SSD1681_INVALID_PARAM;
	}

	/* Dibujar cada línea de la fuente */
	for (i = 0; i < Font->FontHeight; i++)
	{
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++)
		{
			if ((b << j) & 0x8000)
			{
				st = SSD1681_DrawPixel(SSD1681_State.CurrentX + j, SSD1681_State.CurrentY + i, color);
			}
			else
			{
				st = SSD1681_DrawPixel(SSD1681_State.CurrentX + j, SSD1681_State.CurrentY + i, (SSD1681_COLOR_t)!color);
			}

			if (st != SSD1681_OK)
			{
				return st;
			}
		}
	}

	/* Incrementar posición del cursor */
	SSD1681_State.CurrentX += Font->FontWidth;

	return SSD1681_OK;
}

/**
 * @brief Dibuja una cadena de texto a partir de la posición actual del cursor.
 * 
 * @param[in] str   Cadena a dibujar.
 * @param[in] Font  Fuente a utilizar.
 * @param[in] color Color del texto.
 * @return SSD1681_Status_t
 */
SSD1681_Status_t SSD1681_Puts(const char* str, FontDef_t* Font, SSD1681_COLOR_t color)
{
	SSD1681_Status_t st;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }
	if (str == NULL)               { return SSD1681_INVALID_PARAM;   }

	/* Escribir caracteres */
	while (*str)
	{
		st = SSD1681_Putc(*str, Font, color);
		if (st != SSD1681_OK)
		{
			return st;
		}

		str++;
	}

	return SSD1681_OK;
}

/**
 * @brief Dibuja una línea entre dos puntos.
 * 
 * @param[in] x0  Coordenada X de inicio.
 * @param[in] y0  Coordenada Y de inicio.
 * @param[in] x1  Coordenada X de fin.
 * @param[in] y1  Coordenada Y de fin.
 * @param[in] c   Color de la línea
 * @return SSD1681_Status_t 
 */
SSD1681_Status_t SSD1681_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1681_COLOR_t c)
{
	SSD1681_Status_t st;
	int16_t dx, dy, sx, sy, err, e2;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	/* Extremos fuera de pantalla: mismo criterio que SSD1681_DrawPixel */
	if (x0 >= SSD1681_Width  || x1 >= SSD1681_Width ||
		y0 >= SSD1681_Height || y1 >= SSD1681_Height)
	{
		return SSD1681_INVALID_PARAM;
	}

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	/* Bresenham cubre también líneas horizontales, verticales y un punto */
	while (1)
	{
		st = SSD1681_DrawPixel(x0, y0, c);
		if (st != SSD1681_OK)
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

	return SSD1681_OK;
}

/**
 * @brief Dibuja una línea vertical de forma optimizada (sin Bresenham).
 *
 * @param[in] x     Coordenada X de la línea.
 * @param[in] y     Coordenada Y inicial.
 * @param[in] h     Alto de la línea (puede ser negativo, se normaliza).
 * @param[in] color Color de la línea.
 * @return SSD1681_Status_t
 */
SSD1681_Status_t SSD1681_DrawFastVLine(int16_t x, int16_t y, int16_t h, SSD1681_COLOR_t color)
{
	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	if (h == 0)
	{
		return SSD1681_OK;
	}

	if (h < 0) { y += h + 1; h = -h; }

	return ssd1681_FillVLine(x, y, y + h - 1, color);
}