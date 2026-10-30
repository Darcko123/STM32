 /**
 * @file Epaper.c
 * @brief Librería para la gestión de pantallas e-paper (SSD1680/SSD1681/SSD1683) mediante comunicación SPI.
 *
 * @details Driver para pantallas e-paper monocromáticas, soporta inicialización,
 *          primitivas de dibujo (píxeles, líneas, rectángulos, triángulos,
 *          círculos, bitmaps) y texto con fuentes personalizadas.
 *
 * @author Tilen Majerle, Alexander Lutsai - Daniel Ruiz
 * @date Octubre 29, 2026
 * @version 0.1.0
 */

#include "Epaper.h"

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

#define SSD1681_DATA_ENTRY_XY_INC   0x03U /**< X+ y Y+, avanza en X (por filas)                */
#define SSD1681_BORDER_WHITE        0x05U /**< El borde sigue a la LUT del blanco              */
#define SSD1681_TEMP_INTERNAL       0x80U /**< Sensor de temperatura interno                   */

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
static SSD1681_Status_t SSD1681_SendCommand(uint8_t cmd, const uint8_t* data, uint16_t size)
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
static SSD1681_Status_t SSD1681_WaitBusy(uint32_t timeout_ms)
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
static void SSD1681_HardwareReset(void)
{
	HAL_GPIO_WritePin(SSD1681_RST_Port, SSD1681_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(SSD1681_RESET_PULSE_MS);
	HAL_GPIO_WritePin(SSD1681_RST_Port, SSD1681_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(SSD1681_RESET_PULSE_MS);
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

	SSD1681_HardwareReset();
	st = SSD1681_WaitBusy(SSD1681_BUSY_TIMEOUT_MS);
	st = st ? st : SSD1681_SendCommand(SSD1681_CMD_SW_RESET, NULL, 0U);
	st = st ? st : SSD1681_WaitBusy(SSD1681_BUSY_TIMEOUT_MS);

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

	st = st ? st : SSD1681_SendCommand(SSD1681_CMD_DRIVER_OUTPUT, driverOutput, sizeof(driverOutput));
	st = st ? st : SSD1681_SendCommand(SSD1681_CMD_DATA_ENTRY,    &dataEntry,   1U);
	st = st ? st : SSD1681_SendCommand(SSD1681_CMD_RAM_X_RANGE,   xRange,       sizeof(xRange));
	st = st ? st : SSD1681_SendCommand(SSD1681_CMD_RAM_Y_RANGE,   yRange,       sizeof(yRange));
	st = st ? st : SSD1681_SendCommand(SSD1681_CMD_BORDER,        &border,      1U);
	st = st ? st : SSD1681_SendCommand(SSD1681_CMD_TEMP_SENSOR,   &tempSensor,  1U);
	st = st ? st : SSD1681_SendCommand(SSD1681_CMD_RAM_X_COUNTER, origin,       1U);
	st = st ? st : SSD1681_SendCommand(SSD1681_CMD_RAM_Y_COUNTER, origin,       2U);
	st = st ? st : SSD1681_WaitBusy(SSD1681_BUSY_TIMEOUT_MS);

	if (st == SSD1681_OK)
	{
		SSD1681_Initialized = 1U;
	}
	return st;
}
