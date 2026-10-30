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

/* Factor de conversión de grados a radianes, usado por ssd1681_ArcSinCos */
#define SSD1681_DEG_TO_RAD          0.017453292519943295769236907684886f

/* Tolerancia (en píxeles) de la distancia con signo a los rayos del sector:
 * absorbe el error de redondeo de sinf()/cosf() sin desplazar el borde */
#define SSD1681_ARC_EPS             1e-3f

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

/**
 * @brief Rellena un segmento horizontal directamente en el buffer de pantalla.
 *
 * @details Evita el cómputo de Bresenham de SSD1681_DrawLine para el caso
 *          (ya conocido) de una línea horizontal, usado por las rutinas de
 *          relleno de círculos, triángulos y elipses.
 *
 * @param x0    Coordenada X inicial (puede exceder los límites, se recorta).
 * @param x1    Coordenada X final (puede exceder los límites, se recorta).
 * @param y     Coordenada Y del segmento.
 * @param color Color a utilizar.
 *
 * @return SSD1681_Status_t Estado de la operación.
 */
static SSD1681_Status_t ssd1681_FillHLine(int16_t x0, int16_t x1, int16_t y, SSD1681_COLOR_t color)
{
	SSD1681_Status_t st;
	int16_t x, tmp;

	if (y < 0 || y >= (int16_t)SSD1681_Height)
	{
		return SSD1681_OK;
	}

	if (x0 > x1) { tmp = x0; x0 = x1; x1 = tmp; }
	if (x0 < 0) { x0 = 0; }
	if (x1 >= (int16_t)SSD1681_Width) { x1 = (int16_t)SSD1681_Width - 1; }

	for (x = x0; x <= x1; x++)
	{
		st = SSD1681_DrawPixel((uint16_t)x, (uint16_t)y, color);
		if (st != SSD1681_OK)
		{
			return st;
		}
	}

	return SSD1681_OK;
}

/**
 * @brief Dibuja un píxel, ignorando en silencio los que caen fuera de pantalla.
 *
 * @details Usado por SSD1681_DrawCircle y SSD1681_DrawEllipse para recortar figuras parcialmente
 *          visibles, con el mismo criterio que ssd1681_FillHLine.
 *
 * @param x     Coordenada X (puede exceder los límites).
 * @param y     Coordenada Y (puede exceder los límites).
 * @param color Color a utilizar.
 *
 * @return SSD1681_Status_t Estado de la operación.
 */
static SSD1681_Status_t ssd1681_PlotPixel(int16_t x, int16_t y, SSD1681_COLOR_t color)
{
	if (x < 0 || y < 0 || x >= (int16_t)SSD1681_Width || y >= (int16_t)SSD1681_Height)
	{
		return SSD1681_OK;
	}

	return SSD1681_DrawPixel((uint16_t)x, (uint16_t)y, color);
}

/**
 * @brief Dibuja un tramo horizontal de un arco, recortado a la pantalla.
 *
 * @details Recorta en int32_t antes de pasar a int16_t: el cast truncaría
 *          coordenadas fuera de rango y ssd1681_FillHLine no lo detectaría.
 *
 * @param xL    Extremo izquierdo del tramo.
 * @param xR    Extremo derecho del tramo.
 * @param y     Fila del tramo.
 * @param color Color a utilizar.
 *
 * @return SSD1681_Status_t Estado de la operación.
 */
static SSD1681_Status_t ssd1681_ArcSpan(int32_t xL, int32_t xR, int32_t y, SSD1681_COLOR_t color)
{
	if (y < 0 || y >= (int32_t)SSD1681_Height || xR < 0 || xL >= (int32_t)SSD1681_Width)
	{
		return SSD1681_OK;
	}

	if (xL < 0) { xL = 0; }
	if (xR >= (int32_t)SSD1681_Width) { xR = (int32_t)SSD1681_Width - 1; }

	return ssd1681_FillHLine((int16_t)xL, (int16_t)xR, (int16_t)y, color);
}

/**
 * @brief Calcula el seno y el coseno de un ángulo expresado en grados.
 *
 * @details En los múltiplos de 90° se devuelven valores exactos: sinf()/cosf()
 *          dejan un residuo del orden de 1e-7 que, al dividir por el seno para
 *          obtener el borde del sector, desplaza el corte un píxel hacia un
 *          lado. El redondeo a los valores exactos elimina ese desplazamiento.
 *
 * @param deg   Ángulo en grados (cualquier valor; se normaliza a [0,360)).
 * @param sin_t Seno del ángulo.
 * @param cos_t Coseno del ángulo.
 */
static void ssd1681_ArcSinCos(float deg, float* sin_t, float* cos_t)
{
	float r = fmodf(deg, 360.0f);
	if (r < 0.0f) { r += 360.0f; }

	if      ((r < 0.001f)   || (r > 359.999f)) { *sin_t =  0.0f; *cos_t =  1.0f; }
	else if ((r > 89.999f)  && (r < 90.001f))  { *sin_t =  1.0f; *cos_t =  0.0f; }
	else if ((r > 179.999f) && (r < 180.001f)) { *sin_t =  0.0f; *cos_t = -1.0f; }
	else if ((r > 269.999f) && (r < 270.001f)) { *sin_t = -1.0f; *cos_t =  0.0f; }
	else
	{
		*sin_t = sinf(r * SSD1681_DEG_TO_RAD);
		*cos_t = cosf(r * SSD1681_DEG_TO_RAD);
	}
}

/**
 * @brief Raíz cuadrada entera exacta: floor(sqrt(v)) para v >= 0.
 *
 * @details Con radios int16_t, v < 2^30 y los cuadrados caben en int32_t.
 *
 * @param v Valor del que se quiere la raíz cuadrada.
 *
 * @return La mayor raíz cuadrada entera cuyo cuadrado no supera @p v.
 */
static int32_t ssd1681_Isqrt(int32_t v)
{
	int32_t r = (int32_t)sqrtf((float)v);

	/* Corrige el posible error de redondeo de la raíz en coma flotante */
	while ((r * r) > v) { r--; }
	while (((r + 1) * (r + 1)) <= v) { r++; }

	return r;
}

/**
 * @brief Normaliza el par de ángulos de un arco.
 *
 * @details Devuelve el ángulo inicial normalizado a [0,360) y el barrido en
 *          sentido horario. Si los ángulos normalizados coinciden pero los
 *          originales no (caso start=0, end=360, o start=90, end=450), el
 *          barrido es de 360°: el arco completo.
 *
 * @param start Ángulo inicial en grados.
 * @param end   Ángulo final en grados.
 * @param a0    Ángulo inicial normalizado en grados.
 * @param sweep Barrido angular en grados, en sentido horario.
 */
static void ssd1681_ArcAngles(float start, float end, float* a0, float* sweep)
{
	float s = fmodf(start, 360.0f);
	float e = fmodf(end, 360.0f);

	if (s < 0.0f) { s += 360.0f; }
	if (e < 0.0f) { e += 360.0f; }

	*sweep = e - s;
	if (*sweep < 0.0f) { *sweep += 360.0f; }
	if ((*sweep <= 0.001f) && (fabsf(start - end) > 0.001f)) { *sweep = 360.0f; }

	*a0 = s;
}

/**
 * @brief Rellena el sector anular (banda de radios) comprendido entre los ángulos
 *        @p a0 y @p a0 + @p sweep.
 *
 * @details Recorre el sector fila a fila. Para cada fila calcula:
 *          -# el rango radial: los píxeles cuyo cuadrado de distancia al centro
 *             está en [r_in²-r_in, r_out²+r_out), es decir, entre los radios
 *             r_in-0.5 y r_out+0.5. Con r_in == r_out la banda resultante es de
 *             un píxel de ancho, que es lo que usa SSD1681_DrawArc para trazar
 *             los bordes curvos.
 *          -# el rango angular: la intersección de los dos semiplanos que
 *             delimitan el sector, que se resuelve como una cota para X.
 *
 *          El barrido angular se trocea en trozos de 180° como máximo, porque
 *          un sector de más de 180° no es convexo y no se puede describir con
 *          dos semiplanos: la unión de dos sectores convexos consecutivos
 *          cubre el mismo barrido.
 *
 * @param cx     Coordenada X del centro.
 * @param cy     Coordenada Y del centro.
 * @param r_out  Radio exterior de la banda.
 * @param r_in   Radio interior de la banda.
 * @param a0     Ángulo inicial en grados (0° = derecha, sentido horario).
 * @param sweep  Barrido angular en grados, en sentido horario.
 * @param color  Color a utilizar.
 *
 * @return SSD1681_Status_t Estado de la operación.
 */
static SSD1681_Status_t ssd1681_FillArcHelper(int32_t cx, int32_t cy, int32_t r_out, int32_t r_in, float a0, float sweep, SSD1681_COLOR_t color)
{
	SSD1681_Status_t st;
	int32_t hi2, lo2, dy;
	float a = a0;

	if (r_in < 0) { r_in = 0; }

	hi2 = (r_out * r_out) + r_out; /* d² < hi2  → radio < r_out+0.5 */
	lo2 = (r_in * r_in) - r_in;    /* d² >= lo2 → radio >= r_in-0.5 */

	while (sweep > 0.001f)
	{
		float chunk = (sweep > 180.0f) ? 180.0f : sweep; /* Trozo convexo del sector */
		float sA, cA, sB, cB;

		ssd1681_ArcSinCos(a, &sA, &cA);
		ssd1681_ArcSinCos(a + chunk, &sB, &cB);

		for (dy = -r_out; dy <= r_out; dy++)
		{
			int32_t rem  = hi2 - 1 - (dy * dy); /* x² <= rem  → |x| <= x_max */
			int32_t need = lo2 - (dy * dy);     /* x² >= need → |x| >= x_min */
			int32_t x_max, x_min, blo, bhi, k;
			bool empty = false;

			if (rem < 0) { continue; }

			x_max = ssd1681_Isqrt(rem);
			x_min = (need > 0) ? (ssd1681_Isqrt(need - 1) + 1) : 0;
			if (x_min > x_max) { continue; }

			/* Cota angular: por defecto, toda la fila de la banda */
			blo = -x_max;
			bhi =  x_max;

			/* Límite del rayo inicial: sin(φ-A) >= 0  →  cA*dy - sA*x >= -eps */
			if (fabsf(sA) < 1e-6f)
			{
				if ((cA * (float)dy) < -SSD1681_ARC_EPS) { empty = true; }
			}
			else if (sA > 0.0f)
			{
				int32_t v = (int32_t)floorf((cA * (float)dy + SSD1681_ARC_EPS) / sA);
				if (v < bhi) { bhi = v; }
			}
			else
			{
				int32_t v = (int32_t)ceilf((cA * (float)dy + SSD1681_ARC_EPS) / sA);
				if (v > blo) { blo = v; }
			}

			/* Límite del rayo final: sin(φ-B) <= 0  →  sB*x - cB*dy >= -eps */
			if (!empty)
			{
				if (fabsf(sB) < 1e-6f)
				{
					if ((-cB * (float)dy) < -SSD1681_ARC_EPS) { empty = true; }
				}
				else if (sB > 0.0f)
				{
					int32_t v = (int32_t)ceilf((cB * (float)dy - SSD1681_ARC_EPS) / sB);
					if (v > blo) { blo = v; }
				}
				else
				{
					int32_t v = (int32_t)floorf((cB * (float)dy - SSD1681_ARC_EPS) / sB);
					if (v < bhi) { bhi = v; }
				}
			}

			if (empty || (blo > bhi)) { continue; }

			/* La banda radial deja un tramo por fila si el hueco interior no
			 * llega a esa altura, o dos si la fila atraviesa el disco hueco.
			 * Cada tramo se recorta contra la cota angular y se dibuja. */
			for (k = 0; k < ((x_min == 0) ? 1 : 2); k++)
			{
				int32_t px0 = (x_min == 0) ? -x_max : ((k == 0) ? -x_max : x_min);
				int32_t px1 = (x_min == 0) ?  x_max : ((k == 0) ? -x_min : x_max);
				int32_t sx0 = (px0 > blo) ? px0 : blo;
				int32_t sx1 = (px1 < bhi) ? px1 : bhi;

				if (sx0 > sx1) { continue; }

				st = ssd1681_ArcSpan(cx + sx0, cx + sx1, cy + dy, color);
				if (st != SSD1681_OK)
				{
					return st;
				}
			}
		}

		a     += chunk;
		sweep -= chunk;
	}

	return SSD1681_OK;
}

/**
 * @brief Dibuja el radio recto (la "raya" que une el borde interior con el
 *        exterior) de un sector anular, en el ángulo indicado.
 *
 * @details Recorre la arista por su eje dominante (columnas si la arista es más
 *          horizontal que vertical, filas en caso contrario) y calcula en cada
 *          paso la coordenada exacta sobre la semirrecta. Así la arista queda
 *          alineada con la circunferencia en sus dos extremos. En el recorrido
 *          por columnas los píxeles de la misma fila se agrupan en un único tramo.
 *
 * @param cx     Coordenada X del centro.
 * @param cy     Coordenada Y del centro.
 * @param r_in   Radio interior (origen de la raya).
 * @param r_out  Radio exterior (fin de la raya).
 * @param ang    Ángulo de la raya en grados (0° = derecha, sentido horario).
 * @param color  Color a utilizar.
 *
 * @return SSD1681_Status_t Estado de la operación.
 */
static SSD1681_Status_t ssd1681_DrawArcRadius(int32_t cx, int32_t cy, int32_t r_in, int32_t r_out, float ang, SSD1681_COLOR_t color)
{
	SSD1681_Status_t st = SSD1681_OK;
	float s, c;
	int32_t i, from, to, tmp;

	ssd1681_ArcSinCos(ang, &s, &c);

	if (fabsf(c) >= fabsf(s))
	{
		/* Arista más horizontal que vertical: se recorre por columnas agrupando
		 * las que caen en la misma fila */
		int32_t run_x0 = 0, run_y = 0;
		bool run = false;

		from = cx + (int32_t)lroundf((float)r_in  * c);
		to   = cx + (int32_t)lroundf((float)r_out * c);
		if (from > to) { tmp = from; from = to; to = tmp; }

		for (i = from; i <= to; i++)
		{
			int32_t y = cy + (int32_t)lroundf((float)(i - cx) * s / c);

			if (run && (y == run_y)) { continue; } /* Se alarga el tramo */

			if (run) /* Se cierra el tramo anterior */
			{
				st = ssd1681_ArcSpan(run_x0, i - 1, run_y, color);
				if (st != SSD1681_OK)
				{
					return st;
				}
			}
			run_x0 = i;
			run_y  = y;
			run    = true;
		}

		if (run) { st = ssd1681_ArcSpan(run_x0, to, run_y, color); }
	}
	else
	{
		/* Arista más vertical que horizontal: se recorre por filas */
		from = cy + (int32_t)lroundf((float)r_in  * s);
		to   = cy + (int32_t)lroundf((float)r_out * s);
		if (from > to) { tmp = from; from = to; to = tmp; }

		for (i = from; (i <= to) && (st == SSD1681_OK); i++)
		{
			int32_t x = cx + (int32_t)lroundf((float)(i - cy) * c / s);
			st = ssd1681_ArcSpan(x, x, i, color);
		}
	}

	return st;
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
 * @brief Limpia el buffer y actualiza la pantalla.
 * 
 * @return SSD1681_Status_t 
 */
SSD1681_Status_t SSD1681_Clear(void)
{
	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	SSD1681_Status_t st = SSD1681_Fill(SSD1681_COLOR_WHITE);
	return (st != SSD1681_OK) ? st : SSD1681_UpdateScreen();
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

/**
 * @brief Dibuja una línea horizontal de forma optimizada (sin Bresenham).
 *
 * @param[in] x     Coordenada X inicial.
 * @param[in] y     Coordenada Y de la línea.
 * @param[in] w     Ancho de la línea (puede ser negativo, se normaliza).
 * @param[in] color Color de la línea.
 * @return SSD1681_Status_t
 */
SSD1681_Status_t SSD1681_DrawFastHLine(int16_t x, int16_t y, int16_t w, SSD1681_COLOR_t color)
{
	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	if (w == 0)
	{
		return SSD1681_OK;
	}

	if (w < 0) { x += w + 1; w = -w; }

	return ssd1681_FillHLine(x, x + w - 1, y, color);
}

/**
 * @brief Dibuja un rectángulo (solo bordes).
 *
 * @param[in] x0  Coordenada X de una esquina.
 * @param[in] y0  Coordenada Y de una esquina.
 * @param[in] x1  Coordenada X de la esquina opuesta.
 * @param[in] y1  Coordenada Y de la esquina opuesta.
 * @param[in] c   Color de la línea
 * @return SSD1681_Status_t 
 */
SSD1681_Status_t SSD1681_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1681_COLOR_t c)
{
	SSD1681_Status_t st;
	uint16_t tmp;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	/* Extremos fuera de pantalla: mismo criterio que SSD1681_DrawLine */
	if (x0 >= SSD1681_Width  || x1 >= SSD1681_Width ||
		y0 >= SSD1681_Height || y1 >= SSD1681_Height)
	{
		return SSD1681_INVALID_PARAM;
	}

	/* Normaliza el orden de las esquinas */
	if (x0 > x1) { tmp = x0; x0 = x1; x1 = tmp; }
	if (y0 > y1) { tmp = y0; y0 = y1; y1 = tmp; }

	const int16_t w = (int16_t)(x1 - x0 + 1U);
	const int16_t h = (int16_t)(y1 - y0 + 1U);

	/* Dibuja 4 líneas */
	st  = SSD1681_DrawFastHLine((int16_t)x0, (int16_t)y0, w, c);           /* Línea superior */
	st  = st ? st : SSD1681_DrawFastHLine((int16_t)x0, (int16_t)y1, w, c); /* Línea inferior */
	st  = st ? st : SSD1681_DrawFastVLine((int16_t)x0, (int16_t)y0, h, c); /* Línea izquierda */
	st  = st ? st : SSD1681_DrawFastVLine((int16_t)x1, (int16_t)y0, h, c); /* Línea derecha */

	return st;
}

/**
 * @brief Dibuja un rectángulo relleno.
 * 
 * @param[in] x  Coordenada X superior izquierda.
 * @param[in] y  Coordenada Y superior izquierda.
 * @param[in] w  Ancho del rectángulo.
 * @param[in] h  Alto del rectángulo.
 * @param[in] c  Color de la línea
 * @return SSD1681_Status_t 
 */
SSD1681_Status_t SSD1681_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1681_COLOR_t c)
{
	SSD1681_Status_t st;
	uint16_t i;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	if (x >= SSD1681_Width || y >= SSD1681_Height)
	{
		return SSD1681_INVALID_PARAM;
	}

	if (w == 0U || h == 0U)
	{
		return SSD1681_OK;
	}

	/* Recorta al borde de la pantalla */
	if ((uint32_t)x + w > SSD1681_Width)  { w = SSD1681_Width - x;  }
	if ((uint32_t)y + h > SSD1681_Height) { h = SSD1681_Height - y; }

	/* Dibuja h filas de w píxeles */
	for (i = 0U; i < h; i++)
	{
		st = ssd1681_FillHLine((int16_t)x, (int16_t)(x + w - 1U), (int16_t)(y + i), c);
		if (st != SSD1681_OK)
		{
			return st;
		}
	}

	return SSD1681_OK;
}

/**
 * @brief Dibuja un triángulo.
 * 
 * @param[in] x1    Coordenada X del primer vértice.
 * @param[in] y1    Coordenada Y del primer vértice.
 * @param[in] x2    Coordenada X del segundo vértice.
 * @param[in] y2    Coordenada Y del segundo vértice.
 * @param[in] x3    Coordenada X del tercer vértice.
 * @param[in] y3    Coordenada Y del tercer vértice.
 * @param[in] color Color de las líneas.
 * @return SSD1681_Status_t
 */
SSD1681_Status_t SSD1681_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1681_COLOR_t color)
{
	SSD1681_Status_t st;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	/* Vértices fuera de pantalla: mismo criterio que SSD1681_DrawLine */
	if (x1 >= SSD1681_Width  || x2 >= SSD1681_Width  || x3 >= SSD1681_Width ||
		y1 >= SSD1681_Height || y2 >= SSD1681_Height || y3 >= SSD1681_Height)
	{
		return SSD1681_INVALID_PARAM;
	}

	/* Dibuja líneas */
	st  = SSD1681_DrawLine(x1, y1, x2, y2, color);
	st  = st ? st : SSD1681_DrawLine(x2, y2, x3, y3, color);
	st  = st ? st : SSD1681_DrawLine(x3, y3, x1, y1, color);

	return st;
}

/**
 * @brief Dibuja un triángulo relleno.
 * 
 * @param[in] x1    Coordenada X del primer vértice.
 * @param[in] y1    Coordenada Y del primer vértice.
 * @param[in] x2    Coordenada X del segundo vértice.
 * @param[in] y2    Coordenada Y del segundo vértice.
 * @param[in] x3    Coordenada X del tercer vértice.
 * @param[in] y3    Coordenada Y del tercer vértice.
 * @param[in] color Color de relleno.
 * @return SSD1681_Status_t
 */
SSD1681_Status_t SSD1681_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1681_COLOR_t color)
{
	SSD1681_Status_t st;
	int16_t ax = (int16_t)x1, ay = (int16_t)y1;
	int16_t bx = (int16_t)x2, by = (int16_t)y2;
	int16_t cx = (int16_t)x3, cy = (int16_t)y3;
	int16_t a, b, y, last, tmp;
	int16_t dxab, dyab, dxac, dyac, dxbc, dybc;
	int32_t sa, sb;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }

	/* Vértices fuera de pantalla: mismo criterio que SSD1681_DrawTriangle */
	if (x1 >= SSD1681_Width  || x2 >= SSD1681_Width  || x3 >= SSD1681_Width ||
		y1 >= SSD1681_Height || y2 >= SSD1681_Height || y3 >= SSD1681_Height)
	{
		return SSD1681_INVALID_PARAM;
	}

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

		return ssd1681_FillHLine(a, b, ay, color);
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

		st = ssd1681_FillHLine(a, b, y, color);
		if (st != SSD1681_OK)
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

		st = ssd1681_FillHLine(a, b, y, color);
		if (st != SSD1681_OK)
		{
			return st;
		}
	}

	return SSD1681_OK;
}

/**
 * @brief Dibuja un círculo (solo borde).
 * 
 * @param[in] x0 Coordenada X del centro.
 * @param[in] y0 Coordenada Y del centro.
 * @param[in] r  Radio del círculo (>= 0).
 * @param[in] c  Color del borde.
 * @return SSD1681_Status_t
 *
 * @note Los píxeles fuera de pantalla se recortan sin generar error.
 */
SSD1681_Status_t SSD1681_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1681_COLOR_t c)
{
	SSD1681_Status_t st;
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }
	if (r < 0)                     { return SSD1681_INVALID_PARAM;   }

	st  = ssd1681_PlotPixel(x0, y0 + r, c);
	st  = st ? st : ssd1681_PlotPixel(x0, y0 - r, c);
	st  = st ? st : ssd1681_PlotPixel(x0 + r, y0, c);
	st  = st ? st : ssd1681_PlotPixel(x0 - r, y0, c);
	if (st != SSD1681_OK)
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

		st  = ssd1681_PlotPixel(x0 + x, y0 + y, c);
		st  = st ? st : ssd1681_PlotPixel(x0 - x, y0 + y, c);
		st  = st ? st : ssd1681_PlotPixel(x0 + x, y0 - y, c);
		st  = st ? st : ssd1681_PlotPixel(x0 - x, y0 - y, c);

		st  = st ? st : ssd1681_PlotPixel(x0 + y, y0 + x, c);
		st  = st ? st : ssd1681_PlotPixel(x0 - y, y0 + x, c);
		st  = st ? st : ssd1681_PlotPixel(x0 + y, y0 - x, c);
		st  = st ? st : ssd1681_PlotPixel(x0 - y, y0 - x, c);
		if (st != SSD1681_OK)
		{
			return st;
		}
	}

	return SSD1681_OK;
}

/**
 * @brief Dibuja un círculo relleno.
 * 
 * @param[in] x0 Coordenada X del centro.
 * @param[in] y0 Coordenada Y del centro.
 * @param[in] r  Radio del círculo (>= 0).
 * @param[in] c  Color del relleno.
 * @return SSD1681_Status_t
 *
 * @note Los píxeles fuera de pantalla se recortan sin generar error.
 */
SSD1681_Status_t SSD1681_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1681_COLOR_t c)
{
	SSD1681_Status_t st;
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }
	if (r < 0)                     { return SSD1681_INVALID_PARAM;   }

	/* Línea central; el bucle cubre el resto, incluidos los extremos (x0, y0±r) */
	st = ssd1681_FillHLine(x0 - r, x0 + r, y0, c);
	if (st != SSD1681_OK)
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

		st  = ssd1681_FillHLine(x0 - x, x0 + x, y0 + y, c);
		st  = st ? st : ssd1681_FillHLine(x0 - x, x0 + x, y0 - y, c);

		st  = st ? st : ssd1681_FillHLine(x0 - y, x0 + y, y0 + x, c);
		st  = st ? st : ssd1681_FillHLine(x0 - y, x0 + y, y0 - x, c);
		if (st != SSD1681_OK)
		{
			return st;
		}
	}

	return SSD1681_OK;
}

/**
 * @brief Dibuja una elipse (solo borde).
 *
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] rx    Radio horizontal (>= 0).
 * @param[in] ry    Radio vertical (>= 0).
 * @param[in] color Color del borde.
 * @return SSD1681_Status_t
 *
 * @note Los píxeles fuera de pantalla se recortan sin generar error.
 */
SSD1681_Status_t SSD1681_DrawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, SSD1681_COLOR_t color)
{
	SSD1681_Status_t st;
	int32_t a2, b2, fa2, fb2, sigma, x, y;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }
	if (rx < 0 || ry < 0)          { return SSD1681_INVALID_PARAM;   }

	if (rx == 0)
	{
		return ssd1681_FillVLine(x0, y0 - ry, y0 + ry, color);
	}
	if (ry == 0)
	{
		return ssd1681_FillHLine(x0 - rx, x0 + rx, y0, color);
	}

	a2 = (int32_t)rx * rx;
	b2 = (int32_t)ry * ry;
	fa2 = 4 * a2;
	fb2 = 4 * b2;

	/* Región 1: pendiente |dy/dx| <= 1 */
	for (x = 0, y = ry, sigma = 2 * b2 + a2 * (1 - 2 * ry); b2 * x <= a2 * y; x++)
	{
		st  = ssd1681_PlotPixel((int16_t)(x0 + x), (int16_t)(y0 + y), color);
		st  = st ? st : ssd1681_PlotPixel((int16_t)(x0 - x), (int16_t)(y0 + y), color);
		st  = st ? st : ssd1681_PlotPixel((int16_t)(x0 + x), (int16_t)(y0 - y), color);
		st  = st ? st : ssd1681_PlotPixel((int16_t)(x0 - x), (int16_t)(y0 - y), color);
		if (st != SSD1681_OK)
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
		st  = ssd1681_PlotPixel((int16_t)(x0 + x), (int16_t)(y0 + y), color);
		st  = st ? st : ssd1681_PlotPixel((int16_t)(x0 - x), (int16_t)(y0 + y), color);
		st  = st ? st : ssd1681_PlotPixel((int16_t)(x0 + x), (int16_t)(y0 - y), color);
		st  = st ? st : ssd1681_PlotPixel((int16_t)(x0 - x), (int16_t)(y0 - y), color);
		if (st != SSD1681_OK)
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

	return SSD1681_OK;
}

/**
 * @brief Dibuja una elipse rellena
 * 
 * @param[in] x    Coordenada X del centro.
 * @param[in] y    Coordenada Y del centro.
 * @param[in] rx    Radio horizontal (>= 0).
 * @param[in] ry    Radio vertical (>= 0).
 * @param[in] color Color del relleno.
 * @return SSD1681_Status_t
 *
 * @note Los píxeles fuera de pantalla se recortan sin generar error.
 */
SSD1681_Status_t SSD1681_DrawFilledEllipse(int16_t x, int16_t y, int16_t rx, int16_t ry, SSD1681_COLOR_t color)
{
	SSD1681_Status_t st;
	int32_t a2, b2, fa2, fb2, sigma, dx, dy;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }
	if (rx < 0 || ry < 0)          { return SSD1681_INVALID_PARAM;   }

	if (rx == 0)
	{
		return ssd1681_FillVLine(x, y - ry, y + ry, color);
	}
	if (ry == 0)
	{
		return ssd1681_FillHLine(x - rx, x + rx, y, color);
	}

	a2 = (int32_t)rx * rx;
	b2 = (int32_t)ry * ry;
	fa2 = 4 * a2;
	fb2 = 4 * b2;

	/* Región 1: pendiente |dy/dx| <= 1 */
	for (dx = 0, dy = ry, sigma = 2 * b2 + a2 * (1 - 2 * ry); b2 * dx <= a2 * dy; dx++)
	{
		st  = ssd1681_FillHLine((int16_t)(x - dx), (int16_t)(x + dx), (int16_t)(y + dy), color);
		st  = st ? st : ssd1681_FillHLine((int16_t)(x - dx), (int16_t)(x + dx), (int16_t)(y - dy), color);
		if (st != SSD1681_OK)
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
		st  = ssd1681_FillHLine((int16_t)(x - dx), (int16_t)(x + dx), (int16_t)(y + dy), color);
		st  = st ? st : ssd1681_FillHLine((int16_t)(x - dx), (int16_t)(x + dx), (int16_t)(y - dy), color);
		if (st != SSD1681_OK)
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

	return SSD1681_OK;
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
 * @return SSD1681_Status_t
 *
 * @note Los píxeles fuera de pantalla se recortan sin generar error.
 */
SSD1681_Status_t SSD1681_DrawArc(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, SSD1681_COLOR_t color)
{
	SSD1681_Status_t st;
	float a0, sweep;
	int16_t tmp;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }
	if (r1 < 0 || r2 < 0)          { return SSD1681_INVALID_PARAM;   }

	if (r1 < r2) { tmp = r1; r1 = r2; r2 = tmp; }
	if (r1 < 1)  { r1 = 1; }
	if (r2 < 1)  { r2 = 1; }

	ssd1681_ArcAngles(start, end, &a0, &sweep);

	/* Bordes curvos del arco (bandas de 1 px en el radio exterior e interior) */
	st  = ssd1681_FillArcHelper(x, y, r1, r1, a0, sweep, color);
	st  = st ? st : ssd1681_FillArcHelper(x, y, r2, r2, a0, sweep, color);

	/* Bordes rectos del arco. En el barrido completo ambos extremos coinciden */
	st  = st ? st : ssd1681_DrawArcRadius(x, y, r2, r1, a0, color);
	if ((st == SSD1681_OK) && (sweep < 359.999f))
	{
		st = ssd1681_DrawArcRadius(x, y, r2, r1, a0 + sweep, color);
	}

	return st;
}

/**
 * @brief Dibuja un arco relleno (sector de anillo) entre dos ángulos.
 *
 * @param[in] x     Coordenada X del centro.
 * @param[in] y     Coordenada Y del centro.
 * @param[in] r1    Radio exterior del arco.
 * @param[in] r2    Radio interior del arco.
 * @param[in] start Ángulo inicial en grados (0° = derecha, sentido horario).
 * @param[in] end   Ángulo final en grados.
 * @param[in] color Color del relleno.
 * @return SSD1681_Status_t
 *
 * @note Los píxeles fuera de pantalla se recortan sin generar error.
 */
SSD1681_Status_t SSD1681_DrawFilledArc(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, SSD1681_COLOR_t color)
{
	float a0, sweep;
	int16_t tmp;

	if (SSD1681_Initialized != 1U) { return SSD1681_NOT_INITIALIZED; }
	if (r1 < 0 || r2 < 0)          { return SSD1681_INVALID_PARAM;   }

	if (r1 < r2) { tmp = r1; r1 = r2; r2 = tmp; }
	if (r1 < 1)  { r1 = 1; }
	if (r2 < 1)  { r2 = 1; }

	ssd1681_ArcAngles(start, end, &a0, &sweep);

	/* Barrido nulo (start == end): el sector degenera en la arista radial */
	if (sweep <= 0.001f) { return ssd1681_DrawArcRadius(x, y, r2, r1, a0, color); }

	return ssd1681_FillArcHelper(x, y, r1, r2, a0, sweep, color);
}
