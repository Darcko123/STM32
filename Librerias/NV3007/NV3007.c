/**
 * @file NV3007.c
 * @brief Driver del controlador LCD NV3007 sobre interfaz SPI, panel Estardyn 2.79" 142x428.
 *
 * @details Traducción a STM32 HAL de la librería Arduino_NV3007 (Arduino_GFX),
 *          usando un pin DC dedicado para distinguir comando/dato en el bus SPI.
 *
 *          La GRAM del controlador es de 168x428; el panel solo expone 142 columnas,
 *          por lo que CASET/RASET se desplazan con los offsets de NV3007.h.
 *
 * @author Daniel Ruiz
 * @date Junio 23, 2026
 * @version 0.1.0
 */

#include "NV3007.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static SPI_HandleTypeDef*   NV3007_hspi             = NULL; /**< Handle de SPI utilizado para comunicarse con el NV3007 */
static GPIO_TypeDef*        NV3007_CS_GPIO_Port     = NULL; /**< Puerto GPIO del pin CS del NV3007  */
static uint16_t             NV3007_CS_Pin           = 0;    /**< Pin GPIO del pin CS del NV3007     */
static GPIO_TypeDef*        NV3007_DC_GPIO_Port     = NULL; /**< Puerto GPIO del pin DC del NV3007  */
static uint16_t             NV3007_DC_Pin           = 0;    /**< Pin GPIO del pin DC del NV3007     */
static GPIO_TypeDef*        NV3007_RST_GPIO_Port    = NULL; /**< Puerto GPIO del pin RST del NV3007 */
static uint16_t             NV3007_RST_Pin          = 0;    /**< Pin GPIO del pin RST del NV3007    */
static uint8_t              NV3007_Initialized      = 0U;   /**< Bandera para verificar si el módulo está inicializado */

static uint16_t             NV3007_CurrentW         = 0U;   /**< Ancho de la ventana de direccionamiento activa */
static uint16_t             NV3007_CurrentH         = 0U;   /**< Alto de la ventana de direccionamiento activa  */
static uint16_t             NV3007_CurrentX         = 0xFFFFU; /**< Columna de inicio cacheada (0xFFFF = inválida, fuerza primer envío) */
static uint16_t             NV3007_CurrentY         = 0xFFFFU; /**< Fila de inicio cacheada (0xFFFF = inválida, fuerza primer envío)      */

static uint16_t             NV3007_Width            = NV3007_WIDTH;  /**< Ancho lógico de la pantalla, según la rotación activa */
static uint16_t             NV3007_Height           = NV3007_HEIGHT; /**< Alto lógico de la pantalla, según la rotación activa  */

static uint16_t             NV3007_XOffset          = NV3007_OFFSET_P1_X; /**< Offset de columna activo, sumado en CASET */
static uint16_t             NV3007_YOffset          = NV3007_OFFSET_P1_Y; /**< Offset de fila activo, sumado en RASET    */

/** Offsets de GRAM por orientación. El índice coincide con NV3007_Orientation_t. */
static const uint16_t NV3007_OffsetTable[4][2] = {
    { NV3007_OFFSET_P1_X, NV3007_OFFSET_P1_Y },  /* NV3007_Orientation_Portrait_1  */
    { NV3007_OFFSET_P2_X, NV3007_OFFSET_P2_Y },  /* NV3007_Orientation_Portrait_2  */
    { NV3007_OFFSET_L1_X, NV3007_OFFSET_L1_Y },  /* NV3007_Orientation_Landscape_1 */
    { NV3007_OFFSET_L2_X, NV3007_OFFSET_L2_Y },  /* NV3007_Orientation_Landscape_2 */
};

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Transmite un bloque de bytes al NV3007 por el periférico SPI configurado.
 */
static NV3007_Status_t NV3007_SPI_Send(uint8_t* data, uint16_t size)
{
    return (HAL_SPI_Transmit(NV3007_hspi, data, size, 5000U) == HAL_OK)
           ? NV3007_OK : NV3007_ERROR;
}

/**
 * @brief Inicia una transacción SPI con el NV3007 (CS bajo).
 *
 * @details Equivalente a BEGIN_WRITE de Arduino_GFX: el panel espera que CS
 *          permanezca bajo mientras dura un comando con sus datos asociados.
 */
static void NV3007_Select(void)
{
    HAL_GPIO_WritePin(NV3007_CS_GPIO_Port, NV3007_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Finaliza la transacción SPI con el NV3007 (CS alto). Equivalente a END_WRITE.
 */
static void NV3007_Unselect(void)
{
    HAL_GPIO_WritePin(NV3007_CS_GPIO_Port, NV3007_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Envía un byte de comando (DC bajo) asumiendo CS ya bajo (transacción abierta).
 */
static NV3007_Status_t NV3007_WriteCommandRaw(uint8_t cmd)
{
    HAL_GPIO_WritePin(NV3007_DC_GPIO_Port, NV3007_DC_Pin, GPIO_PIN_RESET);
    return NV3007_SPI_Send(&cmd, 1U);
}

/**
 * @brief Envía un byte de dato (DC alto) asumiendo CS ya bajo (transacción abierta).
 */
static NV3007_Status_t NV3007_WriteDataRaw(uint8_t data)
{
    HAL_GPIO_WritePin(NV3007_DC_GPIO_Port, NV3007_DC_Pin, GPIO_PIN_SET);
    return NV3007_SPI_Send(&data, 1U);
}

/**
 * @brief Envía un comando seguido de un byte de dato, con CS bajo, dentro de una
 *        transacción ya abierta (uso en la secuencia de inicialización).
 */
static NV3007_Status_t NV3007_BatchCmdData8(uint8_t cmd, uint8_t data)
{
    NV3007_Status_t status = NV3007_WriteCommandRaw(cmd);
    return (status == NV3007_OK) ? NV3007_WriteDataRaw(data) : status;
}

/**
 * @brief Envía un byte de comando al NV3007 en su propia transacción (CS bajo→alto).
 */
static NV3007_Status_t NV3007_SendCommand(uint8_t cmd)
{
    NV3007_Status_t status;
    NV3007_Select();
    status = NV3007_WriteCommandRaw(cmd);
    NV3007_Unselect();
    return status;
}

/**
 * @brief Envía un comando seguido de un byte de dato manteniendo CS bajo entre ambos.
 */
static NV3007_Status_t NV3007_WriteCmdData8(uint8_t cmd, uint8_t data)
{
    NV3007_Status_t status;
    NV3007_Select();
    status = NV3007_BatchCmdData8(cmd, data);
    NV3007_Unselect();
    return status;
}

/**
 * @brief Envía un único píxel de color (RGB565, MSB primero) tras NV3007_WriteAddrWindow().
 *
 * @note Asume que CS ya está bajo (transacción abierta por NV3007_WriteAddrWindow,
 *       que deja RAMWR activo). Es esta función quien libera CS al terminar.
 */
static NV3007_Status_t NV3007_SendColor(uint16_t color)
{
    uint8_t data[2] = { (uint8_t)(color >> 8), (uint8_t)(color & 0xFFU) };
    NV3007_Status_t status;

    HAL_GPIO_WritePin(NV3007_DC_GPIO_Port, NV3007_DC_Pin, GPIO_PIN_SET);
    status = NV3007_SPI_Send(data, 2U);
    NV3007_Unselect();
    return status;
}

/** Número de píxeles por ráfaga SPI en NV3007_FillColor() (2 bytes/píxel -> buffer de NV3007_FILL_CHUNK*2 bytes). */
#define NV3007_FILL_CHUNK   64U

/**
 * @brief Repite un color RGB565 @p count veces dentro de una única transacción SPI (CS permanece bajo).
 *
 * @details Equivalente a writeFillRectPreclipped() de Arduino_GFX, pero evitando el costo de
 *          togglear CS por cada píxel: se asume que NV3007_WriteAddrWindow() ya dejó al panel
 *          en RAMWR con CS bajo, y se mantiene la transacción abierta durante todo el volcado.
 *          Es esta función quien libera CS al terminar.
 *
 *          Para reducir el overhead por llamada de HAL_SPI_Transmit, el color se replica en un
 *          buffer de NV3007_FILL_CHUNK píxeles y se envía en ráfagas, en lugar de 2 bytes por vez.
 */
static NV3007_Status_t NV3007_FillColor(uint16_t color, uint32_t count)
{
    uint8_t buf[NV3007_FILL_CHUNK * 2U];
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFFU);
    NV3007_Status_t status = NV3007_OK;
    uint16_t i;

    /* Precargar el buffer con el color repetido */
    for (i = 0U; i < sizeof(buf); i += 2U)
    {
        buf[i]      = hi;
        buf[i + 1U] = lo;
    }

    HAL_GPIO_WritePin(NV3007_DC_GPIO_Port, NV3007_DC_Pin, GPIO_PIN_SET);
    while ((count > 0U) && (status == NV3007_OK))
    {
        uint32_t chunk = (count > NV3007_FILL_CHUNK) ? NV3007_FILL_CHUNK : count;
        status = NV3007_SPI_Send(buf, (uint16_t)(chunk * 2U));
        count -= chunk;
    }
    NV3007_Unselect();

    return status;
}

/**
 * @brief Secuencia de registros propietaria del panel 2.79" 142x428.
 *
 * @details Formato de cada registro: @c cmd, @c n, @c data[n]. Verificada byte a byte
 *          contra @c nv3007_279_init_operations (Arduino_GFX, Arduino_NV3007.h) y contra
 *          el fichero del fabricante "NV3006A1N/NV3007 + IVO2.66": 117 registros, 365 bytes.
 *
 *          Tener la secuencia como TABLA y no como cadena de llamadas permite reproducirla
 *          por un transporte alternativo (p.ej. el sondeo de 3 hilos / 9 bits de main.c)
 *          sin duplicarla, que es justo lo que hace falta para descartar el modo de bus.
 *
 *          Difiere de la secuencia estándar de 168 columnas @c nv3007_init_operations en los
 *          ajustes de charge-pump (VGH/VGL), gamma y timing; con la estándar el panel 2.79"
 *          no genera imagen aunque el backlight encienda.
 *
 *          Registros dependientes del panel, por si hay que reajustarlos:
 *            - 0x9A-0x9E, 0x8F, 0x83-0x85 : booster / VGH / VGL / oscilador
 *            - 0x60-0x7F                  : curvas gamma (positiva y negativa)
 *            - 0x50-0x56                  : VCOM y control de fuente
 *            - 0xA0-0xD1, 0xB0-0xBF       : señales GIP (gate-in-panel)
 *            - 0xE0-0xF1                  : formas de onda STV/CLK del gate driver
 *            - 0xF2                       : timing horizontal (escala con nº de columnas)
 *            - 0x3A = 0x05                : RGB565, 16 bpp (fijo, lo asume el driver)
 *
 * @note SLPOUT (0x11) y DISPON (0x29) NO están en la tabla: necesitan retardos entre medias
 *       y los emite NV3007_RunInitSequence().
 */
const uint8_t NV3007_InitTable[] = {
    0xFF, 1, 0xA5,
    0x9A, 1, 0x08,
    0x9B, 1, 0x08,
    0x9C, 1, 0xB0,
    0x9D, 1, 0x16,
    0x9E, 1, 0xC4,
    0x8F, 2, 0x55, 0x04,
    0x84, 1, 0x90,
    0x83, 1, 0x7B,
    0x85, 1, 0x33,
    0x60, 1, 0x00,
    0x70, 1, 0x00,
    0x61, 1, 0x02,
    0x71, 1, 0x02,
    0x62, 1, 0x04,
    0x72, 1, 0x04,
    0x6C, 1, 0x29,
    0x7C, 1, 0x29,
    0x6D, 1, 0x31,
    0x7D, 1, 0x31,
    0x6E, 1, 0x0F,
    0x7E, 1, 0x0F,
    0x66, 1, 0x21,
    0x76, 1, 0x21,
    0x68, 1, 0x3A,
    0x78, 1, 0x3A,
    0x63, 1, 0x07,
    0x73, 1, 0x07,
    0x64, 1, 0x05,
    0x74, 1, 0x05,
    0x65, 1, 0x02,
    0x75, 1, 0x02,
    0x67, 1, 0x23,
    0x77, 1, 0x23,
    0x69, 1, 0x08,
    0x79, 1, 0x08,
    0x6A, 1, 0x13,
    0x7A, 1, 0x13,
    0x6B, 1, 0x13,
    0x7B, 1, 0x13,
    0x6F, 1, 0x00,
    0x7F, 1, 0x00,
    0x50, 1, 0x00,
    0x52, 1, 0xD6,
    0x53, 1, 0x08,
    0x54, 1, 0x08,
    0x55, 1, 0x1E,
    0x56, 1, 0x1C,
    0xA0, 3, 0x2B, 0x24, 0x00,
    0xA1, 1, 0x87,
    0xA2, 1, 0x86,
    0xA5, 1, 0x00,
    0xA6, 1, 0x00,
    0xA7, 1, 0x00,
    0xA8, 1, 0x36,
    0xA9, 1, 0x7E,
    0xAA, 1, 0x7E,
    0xB9, 1, 0x85,
    0xBA, 1, 0x84,
    0xBB, 1, 0x83,
    0xBC, 1, 0x82,
    0xBD, 1, 0x81,
    0xBE, 1, 0x80,
    0xBF, 1, 0x01,
    0xC0, 1, 0x02,
    0xC1, 1, 0x00,
    0xC2, 1, 0x00,
    0xC3, 1, 0x00,
    0xC4, 1, 0x33,
    0xC5, 1, 0x7E,
    0xC6, 1, 0x7E,
    0xC8, 2, 0x33, 0x33,
    0xC9, 1, 0x68,
    0xCA, 1, 0x69,
    0xCB, 1, 0x6A,
    0xCC, 1, 0x6B,
    0xCD, 2, 0x33, 0x33,
    0xCE, 1, 0x6C,
    0xCF, 1, 0x6D,
    0xD0, 1, 0x6E,
    0xD1, 1, 0x6F,
    0xAB, 2, 0x03, 0x67,
    0xAC, 2, 0x03, 0x6B,
    0xAD, 2, 0x03, 0x68,
    0xAE, 2, 0x03, 0x6C,
    0xB3, 1, 0x00,
    0xB4, 1, 0x00,
    0xB5, 1, 0x00,
    0xB6, 1, 0x32,
    0xB7, 1, 0x7E,
    0xB8, 1, 0x7E,
    0xE0, 1, 0x00,
    0xE1, 2, 0x03, 0x0F,
    0xE2, 1, 0x04,
    0xE3, 1, 0x01,
    0xE4, 1, 0x0E,
    0xE5, 1, 0x01,
    0xE6, 1, 0x19,
    0xE7, 1, 0x10,
    0xE8, 1, 0x10,
    0xEA, 1, 0x12,
    0xEB, 1, 0xD0,
    0xEC, 1, 0x04,
    0xED, 1, 0x07,
    0xEE, 1, 0x07,
    0xEF, 1, 0x09,
    0xF0, 1, 0xD0,
    0xF1, 1, 0x0E,
    0xF9, 1, 0x17,
    0xF2, 4, 0x2C, 0x1B, 0x0B, 0x20,
    0xE9, 1, 0x29,
    0xEC, 1, 0x04,
    0x35, 1, 0x00,
    0x44, 2, 0x00, 0x10,
    0x46, 1, 0x10,
    0xFF, 1, 0x00,
    0x3A, 1, 0x05,
};

const uint16_t NV3007_InitTableSize = (uint16_t)sizeof(NV3007_InitTable);

/**
 * @brief Reproduce NV3007_InitTable por el bus SPI y remata con SLPOUT/DISPON.
 *
 * @details Toda la tabla viaja en una única transacción (CS bajo de principio a fin),
 *          igual que el BEGIN_WRITE/END_WRITE de la referencia.
 *
 *          Los retardos de SLPOUT/DISPON son los del fichero del fabricante (220/200 ms),
 *          no los 120/150 de Arduino_GFX: el charge-pump necesita ese tiempo para
 *          estabilizarse antes de habilitar la salida, y quedarse corto deja el panel en negro.
 */
static NV3007_Status_t NV3007_RunInitSequence(void)
{
    NV3007_Status_t st = NV3007_OK;
    uint16_t i = 0U;

    /* BEGIN_WRITE: CS permanece bajo durante todo el bloque, igual que la referencia */
    NV3007_Select();

    while ((i < NV3007_InitTableSize) && (st == NV3007_OK))
    {
        uint8_t cmd = NV3007_InitTable[i++];
        uint8_t n   = NV3007_InitTable[i++];
        uint8_t k;

        st = NV3007_WriteCommandRaw(cmd);

        /* El índice avanza siempre el registro completo, falle o no la transferencia,
         * para no desalinear la tabla si se decide continuar tras un error. */
        for (k = 0U; k < n; k++)
        {
            if (st == NV3007_OK) { st = NV3007_WriteDataRaw(NV3007_InitTable[i]); }
            i++;
        }
    }

    st  = (st == NV3007_OK) ? NV3007_WriteCommandRaw(NV3007_CMD_SLPOUT) : st;
    if (st == NV3007_OK) { HAL_Delay(NV3007_SLPOUT_DELAY); }
    st  = (st == NV3007_OK) ? NV3007_WriteCommandRaw(NV3007_CMD_DISPON) : st;

    /* END_WRITE */
    NV3007_Unselect();
    if (st != NV3007_OK) { return st; }

    HAL_Delay(NV3007_DISPON_DELAY);

    return NV3007_OK;
}

/**
 * @brief Escribe un píxel sin gestionar el estado de inicialización (uso interno por las
 *        demás funciones de dibujo, equivalente a writePixel() de Arduino_GFX).
 */
static NV3007_Status_t NV3007_WritePixelPreclipped(int16_t x, int16_t y, uint16_t color)
{
    NV3007_Status_t status = NV3007_WriteAddrWindow((uint16_t)x, (uint16_t)y, 1U, 1U);
    return (status == NV3007_OK) ? NV3007_SendColor(color) : status;
}

/**
 * @brief Dibuja una línea diagonal mediante el algoritmo de Bresenham, píxel a píxel.
 *
 * @details Traducción de Arduino_GFX::writeSlashLine(), usado internamente por
 *          NV3007_DrawLine() cuando la línea no es perfectamente horizontal ni vertical.
 */
static NV3007_Status_t NV3007_DrawSlashLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    NV3007_Status_t status = NV3007_OK;
    int16_t dx, dy, err, step, tmp;
    bool steep = ((y1 > y0) ? (y1 - y0) : (y0 - y1)) > ((x1 > x0) ? (x1 - x0) : (x0 - x1));

    if (steep)
    {
        tmp = x0; x0 = y0; y0 = tmp;
        tmp = x1; x1 = y1; y1 = tmp;
    }

    if (x0 > x1)
    {
        tmp = x0; x0 = x1; x1 = tmp;
        tmp = y0; y0 = y1; y1 = tmp;
    }

    dx = x1 - x0;
    dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    err = dx >> 1;
    step = (y0 < y1) ? 1 : -1;

    for (; (x0 <= x1) && (status == NV3007_OK); x0++)
    {
        status = steep ? NV3007_WritePixel(y0, x0, color) : NV3007_WritePixel(x0, y0, color);
        err -= dy;
        if (err < 0)
        {
            err += dx;
            y0 += step;
        }
    }

    return status;
}

/**
 * @brief Dibuja uno o más cuadrantes del contorno de un círculo, usado por NV3007_DrawRoundRect()
 *        para trazar las cuatro esquinas (equivalente a Adafruit_GFX::drawCircleHelper()).
 */
static NV3007_Status_t NV3007_DrawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
    NV3007_Status_t status = NV3007_OK;
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    while ((x < y) && (status == NV3007_OK))
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

        if (cornername & 0x4)
        {
            status = NV3007_WritePixel(x0 + x, y0 + y, color);
            status = (status == NV3007_OK) ? NV3007_WritePixel(x0 + y, y0 + x, color) : status;
        }
        if ((cornername & 0x2) && (status == NV3007_OK))
        {
            status = NV3007_WritePixel(x0 + x, y0 - y, color);
            status = (status == NV3007_OK) ? NV3007_WritePixel(x0 + y, y0 - x, color) : status;
        }
        if ((cornername & 0x8) && (status == NV3007_OK))
        {
            status = NV3007_WritePixel(x0 - y, y0 + x, color);
            status = (status == NV3007_OK) ? NV3007_WritePixel(x0 - x, y0 + y, color) : status;
        }
        if ((cornername & 0x1) && (status == NV3007_OK))
        {
            status = NV3007_WritePixel(x0 - y, y0 - x, color);
            status = (status == NV3007_OK) ? NV3007_WritePixel(x0 - x, y0 - y, color) : status;
        }
    }

    return status;
}

/**
 * @brief Rellena uno o más cuadrantes de un círculo mediante franjas verticales, usado por
 *        NV3007_DrawFilledRoundRect() para las cuatro esquinas (equivalente a
 *        Adafruit_GFX::fillCircleHelper()).
 */
static NV3007_Status_t NV3007_FillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color)
{
    NV3007_Status_t status = NV3007_OK;
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    int16_t px = x;
    int16_t py = y;

    delta++;

    while ((x < y) && (status == NV3007_OK))
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

        if (x < (y + 1))
        {
            if (corners & 1)
            {
                status = NV3007_DrawFastVLine(x0 + x, y0 - y, (2 * y) + delta, color);
            }
            if ((corners & 2) && (status == NV3007_OK))
            {
                status = NV3007_DrawFastVLine(x0 - x, y0 - y, (2 * y) + delta, color);
            }
        }
        if ((y != py) && (status == NV3007_OK))
        {
            if (corners & 1)
            {
                status = NV3007_DrawFastVLine(x0 + py, y0 - px, (2 * px) + delta, color);
            }
            if ((corners & 2) && (status == NV3007_OK))
            {
                status = NV3007_DrawFastVLine(x0 - py, y0 - px, (2 * px) + delta, color);
            }
            py = y;
        }
        px = x;
    }

    return status;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el módulo NV3007
 *
 * @param hspi Puntero al manejador de la interfaz SPI utilizada para comunicarse con el módulo.
 * @param CS_GPIOx Puerto GPIO del pin CS del NV3007.
 * @param CS_Pin Pin GPIO del pin CS del NV3007.
 * @param DC_GPIOx Puerto GPIO del pin DC del NV3007.
 * @param DC_Pin Pin GPIO del pin DC del NV3007.
 * @param RST_GPIOx Puerto GPIO del pin RST del NV3007.
 * @param RST_Pin Pin GPIO del pin RST del NV3007.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_Init(SPI_HandleTypeDef* hspi,
                             GPIO_TypeDef* CS_GPIOx, uint16_t CS_Pin,
                             GPIO_TypeDef* DC_GPIOx, uint16_t DC_Pin,
                             GPIO_TypeDef* RST_GPIOx, uint16_t RST_Pin)
{
    NV3007_Status_t status;

    if (hspi == NULL || CS_GPIOx == NULL || CS_Pin == 0U ||
        DC_GPIOx == NULL || DC_Pin == 0U || RST_GPIOx == NULL || RST_Pin == 0U)
    {
        return NV3007_INVALID_PARAM;
    }

    /* El panel no da ninguna señal de error cuando el bus está mal configurado: se
     * queda simplemente en negro con el backlight encendido, que es indistinguible de
     * un fallo de cableado. Se validan aquí las tres condiciones que el driver asume:
     *
     *   - Tramas de 8 bits: NV3007_SPI_Send() pasa un contador de BYTES a
     *     HAL_SPI_Transmit(), que en 16 bits lo interpretaría como nº de half-words.
     *   - MSB primero: los comandos y el RGB565 viajan con el bit más significativo
     *     delante.
     *   - Reloj muestreando en flanco de subida, es decir SPI Mode 0 (CPOL=0/CPHA=0)
     *     o Mode 3 (CPOL=1/CPHA=1). Los modos 1 y 2 desplazan el muestreo medio ciclo
     *     y el controlador solo recibe basura. */
    if (hspi->Init.DataSize != SPI_DATASIZE_8BIT)
    {
        return NV3007_INVALID_PARAM;
    }
    if (hspi->Init.FirstBit != SPI_FIRSTBIT_MSB)
    {
        return NV3007_INVALID_PARAM;
    }
    if (!(((hspi->Init.CLKPolarity == SPI_POLARITY_LOW)  && (hspi->Init.CLKPhase == SPI_PHASE_1EDGE)) ||
          ((hspi->Init.CLKPolarity == SPI_POLARITY_HIGH) && (hspi->Init.CLKPhase == SPI_PHASE_2EDGE))))
    {
        return NV3007_INVALID_PARAM;
    }

    NV3007_hspi          = hspi;
    NV3007_CS_GPIO_Port  = CS_GPIOx;
    NV3007_CS_Pin        = CS_Pin;
    NV3007_DC_GPIO_Port  = DC_GPIOx;
    NV3007_DC_Pin        = DC_Pin;
    NV3007_RST_GPIO_Port = RST_GPIOx;
    NV3007_RST_Pin       = RST_Pin;
    NV3007_Initialized   = 0U;

    HAL_GPIO_WritePin(NV3007_CS_GPIO_Port, NV3007_CS_Pin, GPIO_PIN_SET);

    /* Reset físico */
    HAL_GPIO_WritePin(NV3007_RST_GPIO_Port, NV3007_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100U);
    HAL_GPIO_WritePin(NV3007_RST_GPIO_Port, NV3007_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(NV3007_RST_DELAY);
    HAL_GPIO_WritePin(NV3007_RST_GPIO_Port, NV3007_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(NV3007_RST_DELAY);

    status = NV3007_RunInitSequence();
    if (status != NV3007_OK) { return status; }

    /* NV3007_Rotate() e NV3007_InvertDisplay() exigen el módulo inicializado; se marca
     * antes y se revierte si alguna falla, para no dejar el driver medio configurado
     * y reportando NV3007_OK a futuras llamadas. */
    NV3007_Initialized = 1U;

    /* MADCTL explícito, vía NV3007_Rotate() para que ancho/alto lógicos y offsets de
     * GRAM queden coherentes con el registro que se acaba de escribir. La secuencia
     * del fabricante no incluye 0x36: sin esto el panel se queda con la orientación y
     * el orden de color por defecto del reset. */
    status = NV3007_Rotate(NV3007_Orientation_Portrait_1);
    if (status != NV3007_OK) { NV3007_Initialized = 0U; return status; }

    /* Con NV3007_IPS=1 esto envía INVON, que es lo que exige este panel IPS. */
    status = NV3007_InvertDisplay(false);
    if (status != NV3007_OK) { NV3007_Initialized = 0U; return status; }

    return NV3007_OK;
}

/**
 * @brief Desinicializa el módulo NV3007, apagando la pantalla y liberando la configuración almacenada.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DeInit(void)
{
    NV3007_Status_t status;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    status = NV3007_DisplayOff();

    NV3007_hspi          = NULL;
    NV3007_CS_GPIO_Port  = NULL;
    NV3007_CS_Pin        = 0U;
    NV3007_DC_GPIO_Port  = NULL;
    NV3007_DC_Pin        = 0U;
    NV3007_RST_GPIO_Port = NULL;
    NV3007_RST_Pin       = 0U;
    NV3007_Initialized   = 0U;

    return status;
}

/**
 * @brief Establece la ventana de direccionamiento activa para la siguiente escritura en RAM.
 *
 * @param x Columna inicial de la ventana.
 * @param y Fila inicial de la ventana.
 * @param w Ancho de la ventana en píxeles.
 * @param h Alto de la ventana en píxeles.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_WriteAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    NV3007_Status_t status = NV3007_OK;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    NV3007_Select();

    if ((x != NV3007_CurrentX) || (w != NV3007_CurrentW) || (y != NV3007_CurrentY) || (h != NV3007_CurrentH))
    {
        /* La caché guarda coordenadas lógicas; el offset de GRAM se suma solo al emitir
         * los comandos, para que el llamante siga razonando en píxeles de pantalla. */
        uint16_t xs = x + NV3007_XOffset;
        uint16_t xe = x + w - 1U + NV3007_XOffset;
        uint16_t ys = y + NV3007_YOffset;
        uint16_t ye = y + h - 1U + NV3007_YOffset;

        status = NV3007_WriteCommandRaw(NV3007_CMD_CASET);
        status = (status == NV3007_OK) ? NV3007_WriteDataRaw((uint8_t)(xs >> 8)) : status;
        status = (status == NV3007_OK) ? NV3007_WriteDataRaw((uint8_t)(xs & 0xFFU)) : status;
        status = (status == NV3007_OK) ? NV3007_WriteDataRaw((uint8_t)(xe >> 8)) : status;
        status = (status == NV3007_OK) ? NV3007_WriteDataRaw((uint8_t)(xe & 0xFFU)) : status;

        status = (status == NV3007_OK) ? NV3007_WriteCommandRaw(NV3007_CMD_RASET) : status;
        status = (status == NV3007_OK) ? NV3007_WriteDataRaw((uint8_t)(ys >> 8)) : status;
        status = (status == NV3007_OK) ? NV3007_WriteDataRaw((uint8_t)(ys & 0xFFU)) : status;
        status = (status == NV3007_OK) ? NV3007_WriteDataRaw((uint8_t)(ye >> 8)) : status;
        status = (status == NV3007_OK) ? NV3007_WriteDataRaw((uint8_t)(ye & 0xFFU)) : status;

        if (status != NV3007_OK)
        {
            NV3007_Unselect();
            return status;
        }

        NV3007_CurrentX = x;
        NV3007_CurrentY = y;
        NV3007_CurrentW = w;
        NV3007_CurrentH = h;
    }

    /* IMPORTANTE: NO se libera CS aquí en el camino exitoso. RAMWR (0x2C) y los
     * datos de píxel que siguen deben viajar en una única transacción SPI con CS
     * bajo; si CS sube entre medias, el NV3007 cierra el ciclo de escritura en RAM
     * y descarta los píxeles. Es NV3007_SendColor() / NV3007_FillColor() quien
     * libera CS tras enviar los datos.
     *
     * Si el propio RAMWR falla, el llamante no continuará con SendColor/FillColor,
     * así que hay que liberar CS aquí para no dejar el bus ocupado indefinidamente. */
    status = NV3007_WriteCommandRaw(NV3007_CMD_RAMWR);
    if (status != NV3007_OK)
    {
        NV3007_Unselect();
    }
    return status;
}

/**
 * @brief Rota la pantalla y actualiza el ancho/alto internos.
 *
 * @note La geometría interna solo se actualiza si el comando SPI es exitoso.
 *
 * @param[in] orientation Orientación deseada (NV3007_Orientation_t, en pasos de 90°).
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_Rotate(NV3007_Orientation_t orientation)
{
    NV3007_Status_t status;
    uint8_t madctl;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    /* Ahora el parámetro indexa NV3007_OffsetTable, así que hay que validarlo:
     * el default: del switch ya no puede hacer de red para valores fuera de rango. */
    if ((unsigned int)orientation > (unsigned int)NV3007_Orientation_Landscape_2)
    {
        return NV3007_INVALID_PARAM;
    }

    switch (orientation)
    {
    case NV3007_Orientation_Landscape_1:
        madctl = NV3007_MADCTL_MX | NV3007_MADCTL_MV | NV3007_MADCTL_RGB;
        break;
    case NV3007_Orientation_Portrait_2:
        madctl = NV3007_MADCTL_MX | NV3007_MADCTL_MY | NV3007_MADCTL_RGB;
        break;
    case NV3007_Orientation_Landscape_2:
        madctl = NV3007_MADCTL_MY | NV3007_MADCTL_MV | NV3007_MADCTL_RGB;
        break;
    default: /* NV3007_Orientation_Portrait_1 */
        madctl = NV3007_MADCTL_RGB;
        break;
    }

    status = NV3007_WriteCmdData8(NV3007_CMD_MADCTL, madctl);
    if (status != NV3007_OK) { return status; }

    /* Intercambiar ancho/alto lógicos cuando la orientación intercambia los ejes (MV) */
    if ((madctl & NV3007_MADCTL_MV) != 0U)
    {
        NV3007_Width  = NV3007_HEIGHT;
        NV3007_Height = NV3007_WIDTH;
    }
    else
    {
        NV3007_Width  = NV3007_WIDTH;
        NV3007_Height = NV3007_HEIGHT;
    }

    /* MX/MY/MV remapean el direccionamiento sobre la GRAM, así que el origen del área
     * visible cambia con la orientación. */
    NV3007_XOffset = NV3007_OffsetTable[orientation][0];
    NV3007_YOffset = NV3007_OffsetTable[orientation][1];

    /* Forzar reenvío de la ventana de direccionamiento tras el cambio de orientación.
     * Imprescindible: el offset acaba de cambiar y la caché guarda coordenadas lógicas. */
    NV3007_CurrentX = 0xFFFFU;
    NV3007_CurrentY = 0xFFFFU;
    NV3007_CurrentW = 0U;
    NV3007_CurrentH = 0U;

    return NV3007_OK;
}

/**
 * @brief Devuelve el ancho lógico activo, ya intercambiado según la rotación.
 *
 * @details El código de aplicación no debe usar NV3007_WIDTH/NV3007_HEIGHT directamente:
 *          esas constantes son la geometría del panel en Portrait, y quedan invertidas
 *          en cuanto se llama a NV3007_Rotate() con una orientación Landscape.
 */
uint16_t NV3007_GetWidth(void)  { return NV3007_Width; }

/**
 * @brief Devuelve el alto lógico activo, ya intercambiado según la rotación.
 */
uint16_t NV3007_GetHeight(void) { return NV3007_Height; }

/**
 * @brief Sobrescribe en tiempo de ejecución el offset de GRAM aplicado a CASET/RASET.
 *
 * @details Pensado para calibrar el panel sin recompilar: se ajustan los valores hasta
 *          que la imagen encaja con el borde físico y luego se trasladan a las constantes
 *          NV3007_OFFSET_* de NV3007.h. El valor se pierde en la siguiente llamada a
 *          NV3007_Rotate(), que lo recarga desde la tabla.
 *
 * @param x_offset Desplazamiento de columna aplicado a CASET.
 * @param y_offset Desplazamiento de fila aplicado a RASET.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_SetOffset(uint16_t x_offset, uint16_t y_offset)
{
    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    NV3007_XOffset = x_offset;
    NV3007_YOffset = y_offset;

    /* Forzar reenvío de la ventana con los nuevos offsets */
    NV3007_CurrentX = 0xFFFFU;
    NV3007_CurrentY = 0xFFFFU;
    NV3007_CurrentW = 0U;
    NV3007_CurrentH = 0U;

    return NV3007_OK;
}

/**
 * @brief Activa o desactiva la inversión de color de la pantalla.
 *
 * @param invert true para invertir los colores, false para restaurarlos.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_InvertDisplay(bool invert)
{
    /* Sin esta guarda, llamarla antes de NV3007_Init() pasa punteros NULL a
     * HAL_GPIO_WritePin() y provoca un HardFault en lugar de un código de error. */
    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

#if (NV3007_IPS != 0)
    /* En panel IPS la polaridad va al revés (ver NV3007_IPS en NV3007.h) */
    return NV3007_SendCommand(invert ? NV3007_CMD_INVOFF : NV3007_CMD_INVON);
#else
    return NV3007_SendCommand(invert ? NV3007_CMD_INVON : NV3007_CMD_INVOFF);
#endif
}

/**
 * @brief Enciende la pantalla: sale del modo Sleep (SLPOUT) y habilita la salida (DISPON).
 *
 * @details SLPOUT por sí solo no basta: si antes se llamó a NV3007_DisplayOff(), la salida
 *          del panel quedó deshabilitada con DISPOFF y hay que rehabilitarla explícitamente.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DisplayOn(void)
{
    NV3007_Status_t status;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    status = NV3007_SendCommand(NV3007_CMD_SLPOUT);
    if (status != NV3007_OK) { return status; }

    HAL_Delay(NV3007_SLPOUT_DELAY);

    return NV3007_SendCommand(NV3007_CMD_DISPON);
}

/**
 * @brief Apaga la pantalla: deshabilita la salida (DISPOFF) y entra en modo Sleep (SLPIN).
 *
 * @details DISPOFF antes de SLPIN evita el destello que produce cortar la alimentación
 *          del panel con la salida de display todavía activa.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DisplayOff(void)
{
    NV3007_Status_t status;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    status = NV3007_SendCommand(NV3007_CMD_DISPOFF);
    if (status != NV3007_OK) { return status; }

    status = NV3007_SendCommand(NV3007_CMD_SLPIN);
    if (status != NV3007_OK) { return status; }

    HAL_Delay(NV3007_SLPIN_DELAY);

    return NV3007_OK;
}

// ============================================================================
// FUNCIONES DE DIBUJO
// ============================================================================

/**
 * @brief Escribe un píxel de color, recortando contra los límites de la pantalla.
 *
 * @param x Columna del píxel.
 * @param y Fila del píxel.
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación (NV3007_OK también si el píxel cae fuera de pantalla).
 */
NV3007_Status_t NV3007_WritePixel(int16_t x, int16_t y, uint16_t color)
{
    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    if ((x < 0) || (x >= (int16_t)NV3007_Width) || (y < 0) || (y >= (int16_t)NV3007_Height))
    {
        return NV3007_OK; /* Fuera de los límites: se descarta silenciosamente, igual que Arduino_GFX */
    }

    return NV3007_WritePixelPreclipped(x, y, color);
}

/**
 * @brief Dibuja un píxel de color (alias de NV3007_WritePixel, sin transacción separada
 *        ya que este driver no agrupa varias operaciones en una sola transacción SPI).
 *
 * @param x Columna del píxel.
 * @param y Fila del píxel.
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawPixel(int16_t x, int16_t y, uint16_t color)
{
    return NV3007_WritePixel(x, y, color);
}

/**
 * @brief Rellena toda la pantalla con un color (alias de NV3007_WriteFilledRectangle
 *        sobre el área completa, equivalente a Arduino_GFX::fillScreen()).
 *
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_FillScreen(uint16_t color)
{
    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    return NV3007_WriteFilledRectangle(0, 0, (int16_t)NV3007_Width, (int16_t)NV3007_Height, color);
}

/**
 * @brief Dibuja una línea entre dos puntos, delegando en las variantes rápidas
 *        horizontal/vertical cuando es posible (equivalente a Arduino_GFX::writeLine()).
 *
 * @param x0 Columna del punto inicial.
 * @param y0 Fila del punto inicial.
 * @param x1 Columna del punto final.
 * @param y1 Fila del punto final.
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    int16_t tmp;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    if (x0 == x1)
    {
        if (y0 > y1) { tmp = y0; y0 = y1; y1 = tmp; }
        return NV3007_DrawFastVLine(x0, y0, y1 - y0 + 1, color);
    }
    else if (y0 == y1)
    {
        if (x0 > x1) { tmp = x0; x0 = x1; x1 = tmp; }
        return NV3007_DrawFastHLine(x0, y0, x1 - x0 + 1, color);
    }

    return NV3007_DrawSlashLine(x0, y0, x1, y1, color);
}

/**
 * @brief Dibuja una línea vertical, recortando contra los límites de la pantalla.
 *
 * @param x Columna de la línea.
 * @param y Fila superior de la línea.
 * @param h Alto de la línea en píxeles.
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
    NV3007_Status_t status;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    if ((x < 0) || (x >= (int16_t)NV3007_Width) || (h == 0))
    {
        return NV3007_OK;
    }

    if (h < 0) { y += h + 1; h = -h; }
    if (y < 0) { h += y; y = 0; }
    if ((y + h) > (int16_t)NV3007_Height) { h = (int16_t)NV3007_Height - y; }
    if (h <= 0)
    {
        return NV3007_OK;
    }

    status = NV3007_WriteAddrWindow((uint16_t)x, (uint16_t)y, 1U, (uint16_t)h);
    return (status == NV3007_OK) ? NV3007_FillColor(color, (uint32_t)h) : status;
}

/**
 * @brief Dibuja una línea horizontal, recortando contra los límites de la pantalla.
 *
 * @param x Columna izquierda de la línea.
 * @param y Fila de la línea.
 * @param w Ancho de la línea en píxeles.
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    NV3007_Status_t status;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    if ((y < 0) || (y >= (int16_t)NV3007_Height) || (w == 0))
    {
        return NV3007_OK;
    }

    if (w < 0) { x += w + 1; w = -w; }
    if (x < 0) { w += x; x = 0; }
    if ((x + w) > (int16_t)NV3007_Width) { w = (int16_t)NV3007_Width - x; }
    if (w <= 0)
    {
        return NV3007_OK;
    }

    status = NV3007_WriteAddrWindow((uint16_t)x, (uint16_t)y, (uint16_t)w, 1U);
    return (status == NV3007_OK) ? NV3007_FillColor(color, (uint32_t)w) : status;
}

/**
 * @brief Dibuja un rectángulo sin relleno (equivalente a Arduino_GFX::drawRect()).
 *
 * @param x Columna de la esquina superior izquierda.
 * @param y Fila de la esquina superior izquierda.
 * @param w Ancho del rectángulo.
 * @param h Alto del rectángulo.
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    NV3007_Status_t status;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    status = NV3007_DrawFastHLine(x, y, w, color);
    status = (status == NV3007_OK) ? NV3007_DrawFastHLine(x, y + h - 1, w, color) : status;
    status = (status == NV3007_OK) ? NV3007_DrawFastVLine(x, y, h, color) : status;
    status = (status == NV3007_OK) ? NV3007_DrawFastVLine(x + w - 1, y, h, color) : status;

    return status;
}

/**
 * @brief Rellena un rectángulo con un color, recortando contra los límites de la pantalla.
 *
 * @details Traducción de Arduino_GFX::writeFillRect(): acepta w/h negativos (se normalizan
 *          moviendo la esquina) y descarta rectángulos completamente fuera de pantalla.
 *
 * @param x Columna de la primera esquina.
 * @param y Fila de la primera esquina.
 * @param w Ancho del rectángulo (negativo = hacia la izquierda de la esquina).
 * @param h Alto del rectángulo (negativo = hacia arriba de la esquina).
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_WriteFilledRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    NV3007_Status_t status;
    int16_t maxX, maxY, x2, y2;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    if ((w == 0) || (h == 0))
    {
        return NV3007_OK;
    }

    if (w < 0) { x += w + 1; w = -w; }
    if (h < 0) { y += h + 1; h = -h; }

    maxX = (int16_t)NV3007_Width - 1;
    maxY = (int16_t)NV3007_Height - 1;
    x2 = x + w - 1;
    y2 = y + h - 1;

    if ((x > maxX) || (y > maxY) || (x2 < 0) || (y2 < 0))
    {
        return NV3007_OK; /* Totalmente fuera de los límites */
    }

    if (x < 0) { x = 0; w = x2 + 1; }
    if (y < 0) { y = 0; h = y2 + 1; }
    if (x2 > maxX) { w = maxX - x + 1; }
    if (y2 > maxY) { h = maxY - y + 1; }

    status = NV3007_WriteAddrWindow((uint16_t)x, (uint16_t)y, (uint16_t)w, (uint16_t)h);
    return (status == NV3007_OK) ? NV3007_FillColor(color, (uint32_t)w * (uint32_t)h) : status;
}

/**
 * @brief Dibuja (rellena) un rectángulo con un color (alias de NV3007_WriteFilledRectangle).
 *
 * @param x Columna de la primera esquina.
 * @param y Fila de la primera esquina.
 * @param w Ancho del rectángulo.
 * @param h Alto del rectángulo.
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawFilledRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    return NV3007_WriteFilledRectangle(x, y, w, h, color);
}

/**
 * @brief Dibuja un círculo sin relleno mediante el algoritmo de punto medio de Bresenham.
 *
 * @param x0 Columna del centro.
 * @param y0 Fila del centro.
 * @param r Radio del círculo.
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    NV3007_Status_t status;
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    status = NV3007_WritePixel(x0, y0 + r, color);
    status = (status == NV3007_OK) ? NV3007_WritePixel(x0, y0 - r, color) : status;
    status = (status == NV3007_OK) ? NV3007_WritePixel(x0 + r, y0, color) : status;
    status = (status == NV3007_OK) ? NV3007_WritePixel(x0 - r, y0, color) : status;

    while ((x < y) && (status == NV3007_OK))
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

        status = NV3007_WritePixel(x0 + x, y0 + y, color);
        status = (status == NV3007_OK) ? NV3007_WritePixel(x0 - x, y0 + y, color) : status;
        status = (status == NV3007_OK) ? NV3007_WritePixel(x0 + x, y0 - y, color) : status;
        status = (status == NV3007_OK) ? NV3007_WritePixel(x0 - x, y0 - y, color) : status;
        status = (status == NV3007_OK) ? NV3007_WritePixel(x0 + y, y0 + x, color) : status;
        status = (status == NV3007_OK) ? NV3007_WritePixel(x0 - y, y0 + x, color) : status;
        status = (status == NV3007_OK) ? NV3007_WritePixel(x0 + y, y0 - x, color) : status;
        status = (status == NV3007_OK) ? NV3007_WritePixel(x0 - y, y0 - x, color) : status;
    }

    return status;
}

/**
 * @brief Dibuja un círculo relleno usando líneas verticales como franjas
 *        (equivalente a Arduino_GFX::fillCircle()).
 *
 * @param x0 Columna del centro.
 * @param y0 Fila del centro.
 * @param r Radio del círculo.
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    NV3007_Status_t status;
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    status = NV3007_DrawFastVLine(x0, y0 - r, (2 * r) + 1, color);

    while ((x < y) && (status == NV3007_OK))
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

        status = NV3007_DrawFastVLine(x0 + x, y0 - y, (2 * y) + 1, color);
        status = (status == NV3007_OK) ? NV3007_DrawFastVLine(x0 - x, y0 - y, (2 * y) + 1, color) : status;
        status = (status == NV3007_OK) ? NV3007_DrawFastVLine(x0 + y, y0 - x, (2 * x) + 1, color) : status;
        status = (status == NV3007_OK) ? NV3007_DrawFastVLine(x0 - y, y0 - x, (2 * x) + 1, color) : status;
    }

    return status;
}

/**
 * @brief Dibuja un triángulo sin relleno trazando sus tres lados
 *        (equivalente a Arduino_GFX::drawTriangle()).
 *
 * @param x0 Columna del vértice 0.
 * @param y0 Fila del vértice 0.
 * @param x1 Columna del vértice 1.
 * @param y1 Fila del vértice 1.
 * @param x2 Columna del vértice 2.
 * @param y2 Fila del vértice 2.
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    NV3007_Status_t status;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    status = NV3007_DrawLine(x0, y0, x1, y1, color);
    status = (status == NV3007_OK) ? NV3007_DrawLine(x1, y1, x2, y2, color) : status;
    status = (status == NV3007_OK) ? NV3007_DrawLine(x2, y2, x0, y0, color) : status;

    return status;
}

/**
 * @brief Dibuja un triángulo relleno mediante barrido de líneas horizontales
 *        (equivalente a Arduino_GFX::fillTriangle()).
 *
 * @param x0 Columna del vértice 0.
 * @param y0 Fila del vértice 0.
 * @param x1 Columna del vértice 1.
 * @param y1 Fila del vértice 1.
 * @param x2 Columna del vértice 2.
 * @param y2 Fila del vértice 2.
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawFilledTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    NV3007_Status_t status = NV3007_OK;
    int16_t a, b, y, last, tmp;
    int16_t dx01, dy01, dx02, dy02, dx12, dy12;
    int32_t sa, sb;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    /* Ordenar los vértices por Y (y2 >= y1 >= y0) */
    if (y0 > y1) { tmp = y0; y0 = y1; y1 = tmp; tmp = x0; x0 = x1; x1 = tmp; }
    if (y1 > y2) { tmp = y2; y2 = y1; y1 = tmp; tmp = x2; x2 = x1; x1 = tmp; }
    if (y0 > y1) { tmp = y0; y0 = y1; y1 = tmp; tmp = x0; x0 = x1; x1 = tmp; }

    if (y0 == y2)
    {
        /* Triángulo degenerado: los tres vértices están en la misma fila */
        a = b = x0;
        if (x1 < a) { a = x1; } else if (x1 > b) { b = x1; }
        if (x2 < a) { a = x2; } else if (x2 > b) { b = x2; }
        return NV3007_DrawFastHLine(a, y0, b - a + 1, color);
    }

    dx01 = x1 - x0;
    dy01 = y1 - y0;
    dx02 = x2 - x0;
    dy02 = y2 - y0;
    dx12 = x2 - x1;
    dy12 = y2 - y1;
    sa = 0;
    sb = 0;

    /* Mitad superior del triángulo: segmentos 0-1 y 0-2 */
    last = (y1 == y2) ? y1 : (y1 - 1);

    for (y = y0; (y <= last) && (status == NV3007_OK); y++)
    {
        a = x0 + (int16_t)(sa / dy01);
        b = x0 + (int16_t)(sb / dy02);
        sa += dx01;
        sb += dx02;
        if (a > b) { tmp = a; a = b; b = tmp; }
        status = NV3007_DrawFastHLine(a, y, b - a + 1, color);
    }

    /* Mitad inferior del triángulo: segmentos 1-2 y 0-2 */
    sa = (int32_t)dx12 * (y - y1);
    sb = (int32_t)dx02 * (y - y0);
    for (; (y <= y2) && (status == NV3007_OK); y++)
    {
        a = x1 + (int16_t)(sa / dy12);
        b = x0 + (int16_t)(sb / dy02);
        sa += dx12;
        sb += dx02;
        if (a > b) { tmp = a; a = b; b = tmp; }
        status = NV3007_DrawFastHLine(a, y, b - a + 1, color);
    }

    return status;
}

/**
 * @brief Dibuja un rectángulo de esquinas redondeadas sin relleno, combinando los lados rectos
 *        con cuatro cuadrantes de círculo en las esquinas (equivalente a Arduino_GFX::drawRoundRect()).
 *
 * @param x0 Columna de la esquina superior izquierda.
 * @param y0 Fila de la esquina superior izquierda.
 * @param w Ancho del rectángulo.
 * @param h Alto del rectángulo.
 * @param radius Radio de las esquinas (se recorta a la mitad del lado menor).
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color)
{
    NV3007_Status_t status;
    int16_t max_radius;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    max_radius = ((w < h) ? w : h) / 2;
    if (radius > max_radius) { radius = max_radius; }

    status = NV3007_DrawFastHLine(x0 + radius, y0, w - 2 * radius, color);
    status = (status == NV3007_OK) ? NV3007_DrawFastHLine(x0 + radius, y0 + h - 1, w - 2 * radius, color) : status;
    status = (status == NV3007_OK) ? NV3007_DrawFastVLine(x0, y0 + radius, h - 2 * radius, color) : status;
    status = (status == NV3007_OK) ? NV3007_DrawFastVLine(x0 + w - 1, y0 + radius, h - 2 * radius, color) : status;

    status = (status == NV3007_OK) ? NV3007_DrawCircleHelper(x0 + radius, y0 + radius, radius, 1, color) : status;
    status = (status == NV3007_OK) ? NV3007_DrawCircleHelper(x0 + w - radius - 1, y0 + radius, radius, 2, color) : status;
    status = (status == NV3007_OK) ? NV3007_DrawCircleHelper(x0 + w - radius - 1, y0 + h - radius - 1, radius, 4, color) : status;
    status = (status == NV3007_OK) ? NV3007_DrawCircleHelper(x0 + radius, y0 + h - radius - 1, radius, 8, color) : status;

    return status;
}

/**
 * @brief Dibuja (rellena) un rectángulo de esquinas redondeadas, combinando un rectángulo central
 *        con dos cuadrantes de círculo relleno por lado (equivalente a Arduino_GFX::fillRoundRect()).
 *
 * @param x0 Columna de la esquina superior izquierda.
 * @param y0 Fila de la esquina superior izquierda.
 * @param w Ancho del rectángulo.
 * @param h Alto del rectángulo.
 * @param radius Radio de las esquinas (se recorta a la mitad del lado menor).
 * @param color Color en formato RGB565.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawFilledRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color)
{
    NV3007_Status_t status;
    int16_t max_radius;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    max_radius = ((w < h) ? w : h) / 2;
    if (radius > max_radius) { radius = max_radius; }

    status = NV3007_WriteFilledRectangle(x0 + radius, y0, w - 2 * radius, h, color);
    status = (status == NV3007_OK) ? NV3007_FillCircleHelper(x0 + w - radius - 1, y0 + radius, radius, 1, h - 2 * radius - 1, color) : status;
    status = (status == NV3007_OK) ? NV3007_FillCircleHelper(x0 + radius, y0 + radius, radius, 2, h - 2 * radius - 1, color) : status;

    return status;
}

/**
 * @brief Dibuja un bitmap monocromo (1 bit por píxel) residente en memoria, usando el color
 *        indicado para los bits activos y descartando los bits inactivos (transparentes)
 *        (equivalente a Arduino_GFX::drawBitmap()).
 *
 * @param x Columna de la esquina superior izquierda.
 * @param y Fila de la esquina superior izquierda.
 * @param bitmap Arreglo de bytes con el bitmap monocromo (MSB primero, relleno a byte completo por fila).
 * @param w Ancho del bitmap en píxeles.
 * @param h Alto del bitmap en píxeles.
 * @param color Color en formato RGB565 para los bits activos.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DrawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color)
{
    NV3007_Status_t status = NV3007_OK;
    int16_t byteWidth = (int16_t)((w + 7) / 8); /* Relleno de fila a byte completo */
    uint8_t byte = 0;
    int16_t i, j;

    if (!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    for (j = 0; (j < h) && (status == NV3007_OK); j++, y++)
    {
        for (i = 0; (i < w) && (status == NV3007_OK); i++)
        {
            if (i & 7)
            {
                byte <<= 1;
            }
            else
            {
                byte = bitmap[(j * byteWidth) + (i / 8)];
            }
            if (byte & 0x80)
            {
                status = NV3007_WritePixel(x + i, y, color);
            }
        }
    }

    return status;
}
