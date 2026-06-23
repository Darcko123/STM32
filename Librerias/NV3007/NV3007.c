/**
 * @file NV3007.c
 * @brief Driver del controlador LCD NV3007 (168x428) sobre interfaz SPI.
 *
 * @details Traducción a STM32 HAL de la librería Arduino_NV3007 (Arduino_GFX),
 *          usando un pin DC dedicado para distinguir comando/dato en el bus SPI.
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
 * @brief Envía un byte de comando al NV3007 (DC bajo).
 */
static NV3007_Status_t NV3007_SendCommand(uint8_t cmd)
{
    NV3007_Status_t status;
    HAL_GPIO_WritePin(NV3007_DC_GPIO_Port, NV3007_DC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NV3007_CS_GPIO_Port, NV3007_CS_Pin, GPIO_PIN_RESET);
    status = NV3007_SPI_Send(&cmd, 1U);
    HAL_GPIO_WritePin(NV3007_CS_GPIO_Port, NV3007_CS_Pin, GPIO_PIN_SET);
    return status;
}

/**
 * @brief Envía un byte de dato al NV3007 (DC alto).
 */
static NV3007_Status_t NV3007_SendData(uint8_t data)
{
    NV3007_Status_t status;
    HAL_GPIO_WritePin(NV3007_DC_GPIO_Port, NV3007_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(NV3007_CS_GPIO_Port, NV3007_CS_Pin, GPIO_PIN_RESET);
    status = NV3007_SPI_Send(&data, 1U);
    HAL_GPIO_WritePin(NV3007_CS_GPIO_Port, NV3007_CS_Pin, GPIO_PIN_SET);
    return status;
}

/**
 * @brief Envía un comando seguido de un byte de dato.
 */
static NV3007_Status_t NV3007_WriteCmdData8(uint8_t cmd, uint8_t data)
{
    NV3007_Status_t status = NV3007_SendCommand(cmd);
    return (status == NV3007_OK) ? NV3007_SendData(data) : status;
}

/**
 * @brief Envía un comando seguido de un dato de 16 bits (MSB primero).
 */
static NV3007_Status_t NV3007_WriteCmdData16(uint8_t cmd, uint16_t data)
{
    NV3007_Status_t status = NV3007_SendCommand(cmd);
    status = (status == NV3007_OK) ? NV3007_SendData((uint8_t)(data >> 8)) : status;
    return (status == NV3007_OK) ? NV3007_SendData((uint8_t)(data & 0xFFU)) : status;
}

/**
 * @brief Envía un único píxel de color (RGB565, MSB primero) tras NV3007_WriteAddrWindow().
 */
static NV3007_Status_t NV3007_SendColor(uint16_t color)
{
    uint8_t data[2] = { (uint8_t)(color >> 8), (uint8_t)(color & 0xFFU) };
    return NV3007_SendData(data[0]) == NV3007_OK ? NV3007_SendData(data[1]) : NV3007_ERROR;
}

/**
 * @brief Repite un color RGB565 @p count veces dentro de una única ráfaga SPI (CS permanece bajo).
 *
 * @details Equivalente a writeFillRectPreclipped() de Arduino_GFX, pero evitando el costo de
 *          togglear CS por cada píxel: se asume que NV3007_WriteAddrWindow() ya dejó al panel
 *          en RAMWR y se mantiene la transacción abierta durante todo el volcado.
 */
static NV3007_Status_t NV3007_FillColor(uint16_t color, uint32_t count)
{
    uint8_t buf[2] = { (uint8_t)(color >> 8), (uint8_t)(color & 0xFFU) };
    NV3007_Status_t status = NV3007_OK;
    uint32_t i;

    HAL_GPIO_WritePin(NV3007_DC_GPIO_Port, NV3007_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(NV3007_CS_GPIO_Port, NV3007_CS_Pin, GPIO_PIN_RESET);
    for (i = 0U; i < count; i++)
    {
        status = NV3007_SPI_Send(buf, 2U);
        if (status != NV3007_OK) { break; }
    }
    HAL_GPIO_WritePin(NV3007_CS_GPIO_Port, NV3007_CS_Pin, GPIO_PIN_SET);

    return status;
}

/**
 * @brief Ejecuta la secuencia de registros propietaria de inicialización del panel NV3007 168x428.
 *
 * @details Traducción directa de @c nv3007_init_operations (Arduino_NV3007.h).
 */
static NV3007_Status_t NV3007_RunInitSequence(void)
{
    NV3007_Status_t st;

    st  = NV3007_WriteCmdData8(0xFF, 0xA5);
    st  = (st == NV3007_OK) ? NV3007_SendCommand(NV3007_CMD_SLPOUT) : st;
    if (st != NV3007_OK) { return st; }
    HAL_Delay(120U);

    st  = NV3007_WriteCmdData8(0xFF, 0xA5);
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x9A, 0x08) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x9B, 0x08) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x9C, 0xB0) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x9D, 0x17) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x9E, 0xC2) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData16(0x8F, 0x2204) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x84, 0x90) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x83, 0x7B) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x85, 0x4F) : st;

    /* GAMMA */
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x6E, 0x0F) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x7E, 0x0F) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x60, 0x00) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x70, 0x00) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x6D, 0x39) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x7D, 0x31) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x61, 0x0A) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x71, 0x0A) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x6C, 0x35) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x7C, 0x29) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x62, 0x0F) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x72, 0x0F) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x68, 0x4F) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x78, 0x45) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x66, 0x33) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x76, 0x33) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x6B, 0x14) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x7B, 0x14) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x63, 0x09) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x73, 0x09) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x6A, 0x13) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x7A, 0x16) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x64, 0x08) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x74, 0x08) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x69, 0x07) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x79, 0x0D) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x65, 0x05) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x75, 0x05) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x67, 0x33) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x77, 0x33) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x6F, 0x00) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x7F, 0x00) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x50, 0x00) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x52, 0xD6) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x53, 0x04) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x54, 0x04) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x55, 0x1B) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x56, 0x1B) : st;

    st  = (st == NV3007_OK) ? NV3007_SendCommand(0xA0) : st;
    st  = (st == NV3007_OK) ? NV3007_SendData(0x2A) : st;
    st  = (st == NV3007_OK) ? NV3007_SendData(0x24) : st;
    st  = (st == NV3007_OK) ? NV3007_SendData(0x00) : st;

    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xA1, 0x84) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xA2, 0x85) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xA8, 0x34) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xA9, 0x80) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xAA, 0x73) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData16(0xAB, 0x0361) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData16(0xAC, 0x0365) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData16(0xAD, 0x0360) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData16(0xAE, 0x0364) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xB9, 0x82) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xBA, 0x83) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xBB, 0x80) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xBC, 0x81) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xBD, 0x02) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xBE, 0x01) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xBF, 0x04) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xC0, 0x03) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xC4, 0x33) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xC5, 0x80) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xC6, 0x73) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xC7, 0x00) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData16(0xC8, 0x3333) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xC9, 0x5B) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xCA, 0x5A) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xCB, 0x5D) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xCC, 0x5C) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData16(0xCD, 0x3333) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xCE, 0x5F) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xCF, 0x5E) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xD0, 0x61) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xD1, 0x60) : st;

    st  = (st == NV3007_OK) ? NV3007_SendCommand(0xB0) : st;
    st  = (st == NV3007_OK) ? NV3007_SendData(0x3A) : st;
    st  = (st == NV3007_OK) ? NV3007_SendData(0x3A) : st;
    st  = (st == NV3007_OK) ? NV3007_SendData(0x00) : st;
    st  = (st == NV3007_OK) ? NV3007_SendData(0x00) : st;

    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xB6, 0x32) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xB7, 0x80) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xB8, 0x73) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xE0, 0x00) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData16(0xE1, 0x030F) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xE2, 0x04) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xE3, 0x01) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xE4, 0x0E) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xE5, 0x01) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xE6, 0x19) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xE7, 0x10) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xE8, 0x10) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xE9, 0x21) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xEA, 0x12) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xEB, 0xD0) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xEC, 0x04) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xED, 0x07) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xEE, 0x07) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xEF, 0x09) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xF0, 0xD0) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xF1, 0x0E) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xF9, 0x56) : st;

    st  = (st == NV3007_OK) ? NV3007_SendCommand(0xF2) : st;
    st  = (st == NV3007_OK) ? NV3007_SendData(0x26) : st;
    st  = (st == NV3007_OK) ? NV3007_SendData(0x1B) : st;
    st  = (st == NV3007_OK) ? NV3007_SendData(0x0B) : st;
    st  = (st == NV3007_OK) ? NV3007_SendData(0x20) : st;

    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xEC, 0x04) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x35, 0x00) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData16(0x44, 0x0010) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x46, 0x10) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0xFF, 0x00) : st;
    st  = (st == NV3007_OK) ? NV3007_WriteCmdData8(0x3A, 0x05) : st;
    st  = (st == NV3007_OK) ? NV3007_SendCommand(NV3007_CMD_SLPOUT) : st;
    if (st != NV3007_OK) { return st; }

    HAL_Delay(200U);

    st = NV3007_SendCommand(NV3007_CMD_DISPON);
    if (st != NV3007_OK) { return st; }

    HAL_Delay(150U);

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

    NV3007_CurrentX = 0xFFFFU;
    NV3007_CurrentY = 0xFFFFU;
    NV3007_CurrentW = 0U;
    NV3007_CurrentH = 0U;
    NV3007_Width    = NV3007_WIDTH;
    NV3007_Height   = NV3007_HEIGHT;

    status = NV3007_InvertDisplay(false);
    if (status != NV3007_OK) { return status; }

    NV3007_Initialized = 1U;

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

    if ((x != NV3007_CurrentX) || (w != NV3007_CurrentW) || (y != NV3007_CurrentY) || (h != NV3007_CurrentH))
    {
        status = NV3007_SendCommand(NV3007_CMD_CASET);
        status = (status == NV3007_OK) ? NV3007_SendData((uint8_t)(x >> 8)) : status;
        status = (status == NV3007_OK) ? NV3007_SendData((uint8_t)(x & 0xFFU)) : status;
        status = (status == NV3007_OK) ? NV3007_SendData((uint8_t)((x + w - 1U) >> 8)) : status;
        status = (status == NV3007_OK) ? NV3007_SendData((uint8_t)((x + w - 1U) & 0xFFU)) : status;

        status = (status == NV3007_OK) ? NV3007_SendCommand(NV3007_CMD_RASET) : status;
        status = (status == NV3007_OK) ? NV3007_SendData((uint8_t)(y >> 8)) : status;
        status = (status == NV3007_OK) ? NV3007_SendData((uint8_t)(y & 0xFFU)) : status;
        status = (status == NV3007_OK) ? NV3007_SendData((uint8_t)((y + h - 1U) >> 8)) : status;
        status = (status == NV3007_OK) ? NV3007_SendData((uint8_t)((y + h - 1U) & 0xFFU)) : status;

        if (status != NV3007_OK) { return status; }

        NV3007_CurrentX = x;
        NV3007_CurrentY = y;
        NV3007_CurrentW = w;
        NV3007_CurrentH = h;
    }

    return NV3007_SendCommand(NV3007_CMD_RAMWR);
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

    /* Forzar reenvío de la ventana de direccionamiento tras el cambio de orientación */
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
    return NV3007_SendCommand(invert ? NV3007_CMD_INVON : NV3007_CMD_INVOFF);
}

/**
 * @brief Enciende la pantalla (sale del modo Sleep).
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DisplayOn(void)
{
    NV3007_Status_t status = NV3007_SendCommand(NV3007_CMD_SLPOUT);
    HAL_Delay(NV3007_SLPOUT_DELAY);
    return status;
}

/**
 * @brief Apaga la pantalla (entra en modo Sleep).
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DisplayOff(void)
{
    NV3007_Status_t status = NV3007_SendCommand(NV3007_CMD_SLPIN);
    HAL_Delay(NV3007_SLPIN_DELAY);
    return status;
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
