/**
 * @file ILI9341_Disc1.h
 * @brief Driver for the ILI9341 TFT LCD display on the STM32F429-Discovery board.
 *
 * @details Controls the ILI9341 320×240 TFT display via SPI5 and the STMPE811
 *          touch-panel controller via I2C3. The SPI must be initialized at
 *          2 Mbit/s before calling LCD_ILI9341_Init(); after initialization
 *          the prescaler is raised to 45 Mbit/s automatically.
 *
 *          Pin mapping:
 *          | Signal     | Pin  | Description               |
 *          |------------|------|---------------------------|
 *          | SDO (MISO) | PF8  | LCD output (optional)     |
 *          | SCK        | PF7  | SPI5 clock                |
 *          | SDI (MOSI) | PF9  | SPI5 master output        |
 *          | D/C (WRX)  | PD13 | Data/Command select       |
 *          | RESET      | PD12 | LCD hardware reset        |
 *          | CS         | PC2  | SPI5 chip select          |
 *          | Touch SCL  | PA8  | I2C3 clock                |
 *          | Touch SDA  | PC9  | I2C3 data                 |
 *
 * @author Dr. Luis Antonio Raygoza Pérez & Ing. Daniel Ruiz
 * @date June 08, 2026
 * @version 0.1.0
 */

#ifndef ILI9341_DISC1_H
#define ILI9341_DISC1_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include "lcd_fonts.h"

// ============================================================================
// MACROS Y CONSTANTES [ILI9341]
// ============================================================================

/* -- Pines de control -- */
#define ILI9341_RST_SET     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET)
#define ILI9341_RST_RESET   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET)
#define ILI9341_CS_SET      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,  GPIO_PIN_SET)
#define ILI9341_CS_RESET    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,  GPIO_PIN_RESET)
#define ILI9341_WRX_SET     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET)
#define ILI9341_WRX_RESET   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET)

/* -- Dimensiones de la pantalla -- */
#define ILI9341_WIDTH       240U                        /**< Display width in pixels  */
#define ILI9341_HEIGHT      320U                        /**< Display height in pixels */
#define ILI9341_PIXEL       (ILI9341_WIDTH * ILI9341_HEIGHT)  /**< Total pixel count  */

/* -- Tamaño del buffer de imagen -- */
#define IMG_TOTAL_BUF32     (ILI9341_PIXEL / 2U)       /**< Frame-buffer size in 32-bit words (2 pixels per word) */

/* -- Colores predefinidos (RGB565) -- */
#define ILI9341_COLOR_WHITE     0xFFFFU
#define ILI9341_COLOR_BLACK     0x0000U
#define ILI9341_COLOR_RED       0xF800U
#define ILI9341_COLOR_GREEN     0x07E0U
#define ILI9341_COLOR_GREEN2    0xB723U
#define ILI9341_COLOR_BLUE      0x001FU
#define ILI9341_COLOR_BLUE2     0x051DU
#define ILI9341_COLOR_YELLOW    0xFFE0U
#define ILI9341_COLOR_ORANGE    0xFBE4U
#define ILI9341_COLOR_CYAN      0x07FFU
#define ILI9341_COLOR_MAGENTA   0xA254U
#define ILI9341_COLOR_GRAY      0x7BEFU
#define ILI9341_COLOR_BROWN     0xBBCAU

/* -- Comandos ILI9341 -- */
#define ILI9341_RESET           0x01U
#define ILI9341_SLEEP_OUT       0x11U
#define ILI9341_GAMMA           0x26U
#define ILI9341_DISPLAY_OFF     0x28U
#define ILI9341_DISPLAY_ON      0x29U
#define ILI9341_COLUMN_ADDR     0x2AU
#define ILI9341_PAGE_ADDR       0x2BU
#define ILI9341_GRAM            0x2CU
#define ILI9341_MAC             0x36U
#define ILI9341_PIXEL_FORMAT    0x3AU
#define ILI9341_WDB             0x51U
#define ILI9341_WCD             0x53U
#define ILI9341_RGB_INTERFACE   0xB0U
#define ILI9341_FRC             0xB1U
#define ILI9341_BPC             0xB5U
#define ILI9341_DFC             0xB6U
#define ILI9341_POWER1          0xC0U
#define ILI9341_POWER2          0xC1U
#define ILI9341_VCOM1           0xC5U
#define ILI9341_VCOM2           0xC7U
#define ILI9341_POWERA          0xCBU
#define ILI9341_POWERB          0xCFU
#define ILI9341_PGAMMA          0xE0U
#define ILI9341_NGAMMA          0xE1U
#define ILI9341_DTCA            0xE8U
#define ILI9341_DTCB            0xEAU
#define ILI9341_POWER_SEQ       0xEDU
#define ILI9341_3GAMMA_EN       0xF2U
#define ILI9341_INTERFACE       0xF6U
#define ILI9341_PRC             0xF7U

/* -- Touch panel STMPE811 -- */
#define TP_ADDR                 0x82U
#define STMPE811_ID             0x0811U

/* Identification registers */
#define TP_REG_CHP_ID           0x00U
#define TP_REG_ID_VER           0x02U

/* General Control registers */
#define TP_REG_SYS_CTRL1        0x03U
#define TP_REG_SYS_CTRL2        0x04U
#define TP_REG_SPI_CFG          0x08U

/* Interrupt Control registers */
#define TP_REG_INT_CTRL         0x09U
#define TP_REG_INT_EN           0x0AU
#define TP_REG_INT_STA          0x0BU
#define TP_REG_GPIO_INT_EN      0x0CU
#define TP_REG_GPIO_INT_STA     0x0DU

/* ADC registers */
#define TP_REG_ADC_INT_EN       0x0EU
#define TP_REG_ADC_INT_STA      0x0FU
#define TP_REG_ADC_CTRL1        0x20U
#define TP_REG_ADC_CTRL2        0x21U
#define TP_REG_ADC_CAPT         0x22U
#define TP_REG_ADC_DATA_CH0     0x30U
#define TP_REG_ADC_DATA_CH1     0x32U
#define TP_REG_ADC_DATA_CH2     0x34U
#define TP_REG_ADC_DATA_CH3     0x36U
#define TP_REG_ADC_DATA_CH4     0x38U
#define TP_REG_ADC_DATA_CH5     0x3AU
#define TP_REG_ADC_DATA_CH6     0x3BU
#define TP_REG_ADC_DATA_CH7     0x3CU

/* GPIO registers */
#define TP_REG_GPIO_SET_PIN     0x10U
#define TP_REG_GPIO_CLR_PIN     0x11U
#define TP_REG_GPIO_MP_STA      0x12U
#define TP_REG_GPIO_DIR         0x13U
#define TP_REG_GPIO_ED          0x14U
#define TP_REG_GPIO_RE          0x15U
#define TP_REG_GPIO_FE          0x16U
#define TP_REG_GPIO_AF          0x17U

/* Touch Panel registers */
#define TP_REG_TP_CTRL          0x40U
#define TP_REG_TP_CFG           0x41U
#define TP_REG_WDM_TR_X         0x42U
#define TP_REG_WDM_TR_Y         0x44U
#define TP_REG_WDM_BL_X         0x46U
#define TP_REG_WDM_BL_Y         0x48U
#define TP_REG_FIFO_TH          0x4AU
#define TP_REG_FIFO_STA         0x4BU
#define TP_REG_FIFO_SIZE        0x4CU
#define TP_REG_TP_DATA_X        0x4DU
#define TP_REG_TP_DATA_Y        0x4FU
#define TP_REG_TP_DATA_Z        0x51U
#define TP_REG_TP_DATA_XYZ      0x52U
#define TP_REG_TP_FRACT_XYZ     0x56U
#define TP_REG_TP_DATA          0x57U
#define TP_REG_TP_I_DRIVE       0x58U
#define TP_REG_TP_SHIELD        0x59U

/* IO Expander functionalities */
#define TP_ADC_FCT              0x01U
#define TP_TP_FCT               0x02U
#define TP_IO_FCT               0x04U

/* IO pins */
#define IO_Pin_0                0x01U
#define IO_Pin_1                0x02U
#define IO_Pin_2                0x04U
#define IO_Pin_3                0x08U
#define IO_Pin_4                0x10U
#define IO_Pin_5                0x20U
#define IO_Pin_6                0x40U
#define IO_Pin_7                0x80U
#define IO_Pin_ALL              0xFFU

/* Touch panel I/O pin mapping */
#define TOUCH_YD                IO_Pin_1
#define TOUCH_XD                IO_Pin_2
#define TOUCH_YU                IO_Pin_3
#define TOUCH_XU                IO_Pin_4
#define TOUCH_IO_ALL            ((uint32_t)(IO_Pin_1 | IO_Pin_2 | IO_Pin_3 | IO_Pin_4))

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Status codes returned by all ILI9341 public functions.
 */
typedef enum {
    ILI9341_OK              = 0,    /**< Operation successful          */
    ILI9341_ERROR           = 1,    /**< Operation failed              */
    ILI9341_TIMEOUT         = 2,    /**< HAL timeout occurred          */
    ILI9341_NOT_INITIALIZED = 3,    /**< Driver not initialized        */
    ILI9341_INVALID_PARAM   = 4     /**< Invalid parameter             */
} ILI9341_Status_t;

/**
 * @brief Configuration structure passed to LCD_ILI9341_Init().
 */
typedef struct {
    SPI_HandleTypeDef* hspi;    /**< HAL SPI handle (SPI5 on Discovery board)              */
    I2C_HandleTypeDef* hi2c;    /**< HAL I2C handle for the STMPE811 touch-panel controller */
} ILI9341_Config_t;

/**
 * @brief Display orientation options passed to LCD_ILI9341_Rotate().
 */
typedef enum {
    LCD_ILI9341_Orientation_Portrait_1,    /**< No rotation          */
    LCD_ILI9341_Orientation_Portrait_2,    /**< 180° rotation        */
    LCD_ILI9341_Orientation_Landscape_1,   /**< 90° rotation         */
    LCD_ILI9341_Orientation_Landscape_2    /**< 270° rotation (-90°) */
} LCD_ILI9341_Orientation_t;

/**
 * @brief Touch-panel state returned by TP_GetState().
 */
typedef struct {
    uint16_t TouchDetected;     /**< Non-zero when a touch is active */
    uint16_t X;                 /**< Calibrated X coordinate [0, 239] */
    uint16_t Y;                 /**< Calibrated Y coordinate [0, 319] */
    uint16_t Z;                 /**< Pressure index (raw ADC value)   */
} TP_STATE;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/* --- Inicialización ------------------------------------------------------- */

/**
 * @brief Initialize the ILI9341 LCD and the STMPE811 touch controller.
 *
 * @param[in] config Pointer to a populated ILI9341_Config_t structure.
 * @return ILI9341_Status_t
 *         - ILI9341_OK            if initialization succeeded.
 *         - ILI9341_INVALID_PARAM if @p config or any handle inside it is NULL.
 */
ILI9341_Status_t LCD_ILI9341_Init(const ILI9341_Config_t* config);

/* --- Primitivas de bajo nivel -------------------------------------------- */

/**
 * @brief Software delay loop (busy-wait, not milliseconds).
 *
 * @param[in] delay Number of loop iterations.
 */
void LCD_ILI9341_Delay(volatile unsigned int delay);

/**
 * @brief Send a command byte to the LCD over SPI.
 *
 * @param[in] data Command byte.
 */
void LCD_ILI9341_SendCommand(uint8_t data);

/**
 * @brief Send a data byte to the LCD over SPI.
 *
 * @param[in] data Data byte.
 */
void LCD_ILI9341_SendData(uint8_t data);

/**
 * @brief Set the active drawing window on the LCD (COLUMN_ADDR + PAGE_ADDR).
 *
 * @param[in] x1 Left column of the window.
 * @param[in] y1 Top row of the window.
 * @param[in] x2 Right column of the window.
 * @param[in] y2 Bottom row of the window.
 */
void LCD_ILI9341_SetCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

/* --- Dibujo en pantalla --------------------------------------------------- */

/**
 * @brief Fill the entire LCD with a solid color.
 *
 * @param[in] color RGB565 fill color.
 */
void LCD_ILI9341_Fill(uint16_t color);

/**
 * @brief Rotate the display and update internal width/height.
 *
 * @param[in] orientation Desired orientation (LCD_ILI9341_Orientation_t).
 */
void LCD_ILI9341_Rotate(LCD_ILI9341_Orientation_t orientation);

/**
 * @brief Draw a single pixel on the LCD.
 *
 * @param[in] x     Pixel X coordinate.
 * @param[in] y     Pixel Y coordinate.
 * @param[in] color RGB565 pixel color.
 */
void LCD_ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Draw a line on the LCD using Bresenham's algorithm.
 *
 * @param[in] x0    Start X coordinate.
 * @param[in] y0    Start Y coordinate.
 * @param[in] x1    End X coordinate.
 * @param[in] y1    End Y coordinate.
 * @param[in] color RGB565 line color.
 */
void LCD_ILI9341_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Draw a rectangle outline on the LCD.
 *
 * @param[in] x0    Top-left X coordinate.
 * @param[in] y0    Top-left Y coordinate.
 * @param[in] x1    Bottom-right X coordinate.
 * @param[in] y1    Bottom-right Y coordinate.
 * @param[in] color RGB565 line color.
 */
void LCD_ILI9341_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Draw a filled rectangle on the LCD.
 *
 * @param[in] x0    Top-left X coordinate.
 * @param[in] y0    Top-left Y coordinate.
 * @param[in] x1    Bottom-right X coordinate.
 * @param[in] y1    Bottom-right Y coordinate.
 * @param[in] color RGB565 fill color.
 */
void LCD_ILI9341_DrawFilledRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Draw a circle outline on the LCD.
 *
 * @param[in] x0    Center X coordinate.
 * @param[in] y0    Center Y coordinate.
 * @param[in] r     Radius in pixels.
 * @param[in] color RGB565 line color.
 */
void LCD_ILI9341_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

/**
 * @brief Draw a filled circle on the LCD.
 *
 * @param[in] x0    Center X coordinate.
 * @param[in] y0    Center Y coordinate.
 * @param[in] r     Radius in pixels.
 * @param[in] color RGB565 fill color.
 */
void LCD_ILI9341_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

/* --- Texto en pantalla ---------------------------------------------------- */

/**
 * @brief Render a single character on the LCD.
 *
 * @param[in] x          Top-left X of the character cell.
 * @param[in] y          Top-left Y of the character cell.
 * @param[in] c          Character to display.
 * @param[in] font       Pointer to the font definition.
 * @param[in] foreground RGB565 foreground color.
 * @param[in] background RGB565 background color.
 */
void LCD_ILI9341_Putc(uint16_t x, uint16_t y, char c, LCD_FontDef_t* font, uint16_t foreground, uint16_t background);

/**
 * @brief Render a null-terminated string on the LCD.
 *
 * @param[in] x          Top-left X of the first character.
 * @param[in] y          Top-left Y of the first character.
 * @param[in] str        Pointer to the null-terminated string.
 * @param[in] font       Pointer to the font definition.
 * @param[in] foreground RGB565 foreground color.
 * @param[in] background RGB565 background color.
 */
void LCD_ILI9341_Puts(uint16_t x, uint16_t y, char* str, LCD_FontDef_t* font, uint16_t foreground, uint16_t background);

/**
 * @brief Compute the pixel bounding-box of a string for a given font.
 *
 * @param[in]  str    Pointer to the null-terminated string.
 * @param[in]  font   Pointer to the font definition.
 * @param[out] width  Total width in pixels.
 * @param[out] height Total height in pixels.
 */
void LCD_ILI9341_GetStringSize(char* str, LCD_FontDef_t* font, uint16_t* width, uint16_t* height);

/* --- Imagen completa ------------------------------------------------------ */

/**
 * @brief Transfer a full-screen RGB565 frame buffer to the LCD via optimized SPI.
 *
 * @param[in] image Array of IMG_TOTAL_BUF32 uint32_t words (two RGB565 pixels per word).
 * @return ILI9341_Status_t
 *         - ILI9341_OK      if all pixels were sent.
 *         - ILI9341_TIMEOUT if the SPI bus stalled.
 *         - ILI9341_ERROR   if the SPI peripheral was busy.
 */
ILI9341_Status_t LCD_ILI9341_DisplayImage(uint32_t image[ILI9341_PIXEL]);

/* --- Frame buffer (escritura fuera de pantalla) --------------------------- */

/**
 * @brief Write a single pixel into an off-screen frame buffer.
 *
 * @param[in]     x      Pixel X coordinate.
 * @param[in]     y      Pixel Y coordinate.
 * @param[in]     color  RGB565 pixel color.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 uint32_t words).
 */
void LCD_ILI9341_DrawPixel_ImageBuffer(uint16_t x, uint16_t y, uint16_t color, uint32_t image[ILI9341_PIXEL]);

/**
 * @brief Render a single character into an off-screen frame buffer.
 *
 * @param[in]     x          Top-left X of the character cell.
 * @param[in]     y          Top-left Y of the character cell.
 * @param[in]     c          Character to display.
 * @param[in]     font       Pointer to the font definition.
 * @param[in]     foreground RGB565 foreground color.
 * @param[in,out] image      Frame buffer (IMG_TOTAL_BUF32 uint32_t words).
 */
void LCD_ILI9341_Putc_ImageBuffer(uint16_t x, uint16_t y, char c, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[ILI9341_PIXEL]);

/**
 * @brief Render a null-terminated string into an off-screen frame buffer.
 *
 * @param[in]     x          Top-left X of the first character.
 * @param[in]     y          Top-left Y of the first character.
 * @param[in]     str        Pointer to the null-terminated string.
 * @param[in]     font       Pointer to the font definition.
 * @param[in]     foreground RGB565 foreground color.
 * @param[in,out] image      Frame buffer (IMG_TOTAL_BUF32 uint32_t words).
 */
void LCD_ILI9341_Puts_ImageBuffer(uint16_t x, uint16_t y, char* str, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[ILI9341_PIXEL]);

/**
 * @brief Draw a line into an off-screen frame buffer.
 *
 * @param[in]     x0     Start X coordinate.
 * @param[in]     y0     Start Y coordinate.
 * @param[in]     x1     End X coordinate.
 * @param[in]     y1     End Y coordinate.
 * @param[in]     color  RGB565 line color.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 uint32_t words).
 */
void LCD_ILI9341_DrawLine_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[ILI9341_PIXEL]);

/**
 * @brief Draw a rectangle outline into an off-screen frame buffer.
 *
 * @param[in]     x0     Top-left X coordinate.
 * @param[in]     y0     Top-left Y coordinate.
 * @param[in]     x1     Bottom-right X coordinate.
 * @param[in]     y1     Bottom-right Y coordinate.
 * @param[in]     color  RGB565 line color.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 uint32_t words).
 */
void LCD_ILI9341_DrawRectangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[ILI9341_PIXEL]);

/**
 * @brief Draw a filled rectangle into an off-screen frame buffer.
 *
 * @param[in]     x0     Top-left X coordinate.
 * @param[in]     y0     Top-left Y coordinate.
 * @param[in]     x1     Bottom-right X coordinate.
 * @param[in]     y1     Bottom-right Y coordinate.
 * @param[in]     color  RGB565 fill color.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 uint32_t words).
 */
void LCD_ILI9341_DrawFilledRectangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[ILI9341_PIXEL]);

/* --- Touch panel (STMPE811) ---------------------------------------------- */

/**
 * @brief Configure the STMPE811 touch-panel controller.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK    if the device was detected and configured.
 *         - ILI9341_ERROR if the chip ID did not match STMPE811_ID.
 */
ILI9341_Status_t TP_Config(void);

/**
 * @brief Read the current touch-panel state (coordinates and touch detection).
 *
 * @return Pointer to the internal TP_STATE structure with updated values.
 */
TP_STATE* TP_GetState(void);

/**
 * @brief Read the calibrated X coordinate of the active touch point.
 *
 * @return X coordinate in the range [0, 239].
 */
uint16_t TP_Read_X(void);

/**
 * @brief Read the calibrated Y coordinate of the active touch point.
 *
 * @return Y coordinate in the range [0, 319].
 */
uint16_t TP_Read_Y(void);

/**
 * @brief Read the Z (pressure) value of the active touch point.
 *
 * @return Pressure index (raw ADC value).
 */
uint16_t TP_Read_Z(void);

/**
 * @brief Reset the STMPE811 via the SYS_CTRL1 software-reset bit.
 *
 * @return ILI9341_Status_t ILI9341_OK always.
 */
ILI9341_Status_t TP_Reset(void);

/**
 * @brief Enable or disable STMPE811 functional blocks (ADC, touch-panel, GPIO).
 *
 * @param[in] Fct      Function mask: TP_ADC_FCT, TP_TP_FCT, or TP_IO_FCT.
 * @param[in] NewState ENABLE or DISABLE.
 * @return ILI9341_Status_t ILI9341_OK always.
 */
ILI9341_Status_t TP_FnctCmd(uint8_t Fct, FunctionalState NewState);

/**
 * @brief Configure alternate-function mode for STMPE811 GPIO pins.
 *
 * @param[in] IO_Pin   Pin mask (IO_Pin_x values).
 * @param[in] NewState ENABLE or DISABLE.
 * @return ILI9341_Status_t ILI9341_OK always.
 */
ILI9341_Status_t TP_IOAFConfig(uint8_t IO_Pin, FunctionalState NewState);

/**
 * @brief Read one byte from a STMPE811 register over I2C.
 *
 * @param[in] RegisterAddr Register address (0x00–0x59).
 * @return Register value, or 0xAA on I2C error.
 */
uint8_t TP_ReadDeviceRegister(uint8_t RegisterAddr);

/**
 * @brief Write one byte to a STMPE811 register over I2C.
 *
 * @param[in] RegisterAddr  Register address.
 * @param[in] RegisterValue Byte to write.
 * @return ILI9341_Status_t
 *         - ILI9341_OK    on success.
 *         - ILI9341_ERROR on I2C failure.
 */
ILI9341_Status_t TP_WriteDeviceRegister(uint8_t RegisterAddr, uint8_t RegisterValue);

/**
 * @brief Read two bytes from a STMPE811 register (used for X/Y/Z ADC data).
 *
 * @param[in] RegisterAddr Register address.
 * @return 16-bit reconstructed value, or 0xAA on I2C error.
 */
uint16_t TP_ReadDataBuffer(uint32_t RegisterAddr);

#ifdef __cplusplus
}
#endif

#endif /* ILI9341_DISC1_H */
