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
