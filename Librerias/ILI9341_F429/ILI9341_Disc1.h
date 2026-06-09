/**
 * @file ILI9341_Disc1.h
 * @brief Driver para la pantalla TFT LCD ILI9341 en la tarjeta STM32F429-Discovery.
 *
 * @details Controla la pantalla TFT ILI9341 de 320×240 mediante SPI5 y el
 *          controlador de panel táctil STMPE811 mediante I2C3. El SPI debe
 *          inicializarse a 2 Mbit/s antes de llamar a ILI9341_Init(); tras
 *          la inicialización el preescalador se eleva a 45 Mbit/s automáticamente.
 *
 *          Mapeo de pines:
 *          | Señal      | Pin  | Descripción               |
 *          |------------|------|---------------------------|
 *          | SDO (MISO) | PF8  | Salida LCD (opcional)     |
 *          | SCK        | PF7  | Reloj SPI5                |
 *          | SDI (MOSI) | PF9  | Salida maestra SPI5       |
 *          | D/C (WRX)  | PD13 | Selección Dato/Comando    |
 *          | RESET      | PD12 | Reset por hardware del LCD|
 *          | CS         | PC2  | Selección de chip SPI5    |
 *          | Touch SCL  | PA8  | Reloj I2C3                |
 *          | Touch SDA  | PC9  | Datos I2C3                |
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
#include <string.h>

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
#define ILI9341_WIDTH       240U                        /**< Ancho de la pantalla en píxeles  */
#define ILI9341_HEIGHT      320U                        /**< Alto de la pantalla en píxeles   */
#define ILI9341_PIXEL       (ILI9341_WIDTH * ILI9341_HEIGHT)  /**< Total de píxeles           */

/* -- Tamaño del buffer de imagen -- */
#define IMG_TOTAL_BUF32     (ILI9341_PIXEL / 2U)       /**< Tamaño del frame-buffer en palabras de 32 bits (2 píxeles por palabra) */

/* -- SDRAM (frame buffer opcional) -- */
#ifdef HAL_SDRAM_MODULE_ENABLED

#ifndef ILI9341_SDRAM_BASE
#define ILI9341_SDRAM_BASE               0xD0000000U    /**< Dirección base del banco 2 del FMC en la STM32F429-Discovery */
#endif
#define ILI9341_SDRAM_FB_SIZE            (IMG_TOTAL_BUF32 * 4U) /**< Tamaño del frame-buffer en SDRAM en bytes (153 600 B) */

/* Tamaño total del chip IS42S16400J (4 MB) */
#define IS42S16400J_SIZE                 0x400000U

/* Longitud de ráfaga */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000U)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001U)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002U)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004U)

/* Tipo de ráfaga */
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000U)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008U)

/* Latencia CAS */
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020U)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030U)

/* Modo de operación */
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000U)

/* Modo de escritura en ráfaga */
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000U)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200U)

/* Contador de refresco para reloj SDRAM a 90 MHz */
#define REFRESH_COUNT                            ((uint32_t)0x056AU)

#endif /* HAL_SDRAM_MODULE_ENABLED */

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

/* Registros de identificación */
#define TP_REG_CHP_ID           0x00U
#define TP_REG_ID_VER           0x02U

/* Registros de control general */
#define TP_REG_SYS_CTRL1        0x03U
#define TP_REG_SYS_CTRL2        0x04U
#define TP_REG_SPI_CFG          0x08U

/* Registros de control de interrupciones */
#define TP_REG_INT_CTRL         0x09U
#define TP_REG_INT_EN           0x0AU
#define TP_REG_INT_STA          0x0BU
#define TP_REG_GPIO_INT_EN      0x0CU
#define TP_REG_GPIO_INT_STA     0x0DU

/* Registros ADC */
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

/* Registros GPIO */
#define TP_REG_GPIO_SET_PIN     0x10U
#define TP_REG_GPIO_CLR_PIN     0x11U
#define TP_REG_GPIO_MP_STA      0x12U
#define TP_REG_GPIO_DIR         0x13U
#define TP_REG_GPIO_ED          0x14U
#define TP_REG_GPIO_RE          0x15U
#define TP_REG_GPIO_FE          0x16U
#define TP_REG_GPIO_AF          0x17U

/* Registros del panel táctil */
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

/* Funcionalidades del expansor de IO */
#define TP_ADC_FCT              0x01U
#define TP_TP_FCT               0x02U
#define TP_IO_FCT               0x04U

/* Pines de IO */
#define IO_Pin_0                0x01U
#define IO_Pin_1                0x02U
#define IO_Pin_2                0x04U
#define IO_Pin_3                0x08U
#define IO_Pin_4                0x10U
#define IO_Pin_5                0x20U
#define IO_Pin_6                0x40U
#define IO_Pin_7                0x80U
#define IO_Pin_ALL              0xFFU

/* Mapeo de pines I/O del panel táctil */
#define TOUCH_YD                IO_Pin_1
#define TOUCH_XD                IO_Pin_2
#define TOUCH_YU                IO_Pin_3
#define TOUCH_XU                IO_Pin_4
#define TOUCH_IO_ALL            ((uint32_t)(IO_Pin_1 | IO_Pin_2 | IO_Pin_3 | IO_Pin_4))

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Códigos de estado retornados por todas las funciones públicas del ILI9341.
 */
typedef enum {
    ILI9341_OK              = 0,    /**< Operación exitosa             */
    ILI9341_ERROR           = 1,    /**< Operación fallida             */
    ILI9341_TIMEOUT         = 2,    /**< Tiempo de espera HAL agotado  */
    ILI9341_NOT_INITIALIZED = 3,    /**< Driver no inicializado        */
    ILI9341_INVALID_PARAM   = 4     /**< Parámetro inválido            */
} ILI9341_Status_t;

/**
 * @brief Opciones de orientación de pantalla para ILI9341_Rotate().
 */
typedef enum {
    ILI9341_Orientation_Portrait_1,    /**< Sin rotación          */
    ILI9341_Orientation_Portrait_2,    /**< Rotación 180°         */
    ILI9341_Orientation_Landscape_1,   /**< Rotación 90°          */
    ILI9341_Orientation_Landscape_2    /**< Rotación 270° (-90°)  */
} ILI9341_Orientation_t;

/**
 * @brief Estado del panel táctil retornado por ILI9341_TP_GetState().
 */
typedef struct {
    uint16_t TouchDetected;     /**< Distinto de cero cuando hay toque activo */
    uint16_t X;                 /**< Coordenada X calibrada [0, 239]          */
    uint16_t Y;                 /**< Coordenada Y calibrada [0, 319]          */
    uint16_t Z;                 /**< Índice de presión (valor ADC crudo)      */
} TP_STATE;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/* --- Inicialización ------------------------------------------------------- */

/**
 * @brief Inicializa la pantalla LCD ILI9341 y el controlador táctil STMPE811.
 *
 * @param[in] hspi   Puntero al handle SPI de HAL.
 * @param[in] hi2c   Puntero al handle I2C de HAL.
 * @param[in] hsdram (Solo con HAL_SDRAM_MODULE_ENABLED) Puntero al handle SDRAM de HAL
 *                   generado por STM32CubeMX. Pasar NULL deshabilita el frame buffer en SDRAM.
 *                   Cuando no es NULL, la librería reserva los primeros ILI9341_SDRAM_FB_SIZE
 *                   bytes de ILI9341_SDRAM_BASE como frame buffer interno (153 600 B).
 * @return ILI9341_Status_t
 *         - ILI9341_OK            si la inicialización fue exitosa.
 *         - ILI9341_INVALID_PARAM si @p hspi o @p hi2c es NULL.
 *         - ILI9341_ERROR         si una transmisión SPI falló durante la inicialización.
 */
#ifdef HAL_SDRAM_MODULE_ENABLED
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, SDRAM_HandleTypeDef* hsdram);
#else
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c);
#endif

/* --- Primitivas de bajo nivel -------------------------------------------- */

/**
 * @brief Bucle de retardo por software (espera activa, no milisegundos).
 *
 * @param[in] delay Número de iteraciones del bucle.
 */
void ILI9341_Delay(volatile unsigned int delay);

/**
 * @brief Envía un byte de comando a la pantalla LCD por SPI.
 *
 * @param[in] data Byte de comando.
 * @return ILI9341_Status_t
 *         - ILI9341_OK            en caso de éxito.
 *         - ILI9341_INVALID_PARAM si el handle SPI es NULL.
 *         - ILI9341_ERROR         si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_SendCommand(uint8_t data);

/**
 * @brief Envía un byte de datos a la pantalla LCD por SPI.
 *
 * @param[in] data Byte de datos.
 * @return ILI9341_Status_t
 *         - ILI9341_OK            en caso de éxito.
 *         - ILI9341_INVALID_PARAM si el handle SPI es NULL.
 *         - ILI9341_ERROR         si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_SendData(uint8_t data);

/**
 * @brief Establece la ventana de dibujo activa en la pantalla LCD (COLUMN_ADDR + PAGE_ADDR).
 *
 * @param[in] x1 Columna izquierda de la ventana.
 * @param[in] y1 Fila superior de la ventana.
 * @param[in] x2 Columna derecha de la ventana.
 * @param[in] y2 Fila inferior de la ventana.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_SetCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

/* --- Dibujo en pantalla --------------------------------------------------- */

/**
 * @brief Rellena toda la pantalla LCD con un color sólido.
 *
 * @param[in] color Color de relleno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_Fill(uint16_t color);

/**
 * @brief Rota la pantalla y actualiza el ancho/alto internos.
 *
 * @note La geometría interna solo se actualiza si el comando SPI es exitoso.
 *
 * @param[in] orientation Orientación deseada (ILI9341_Orientation_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_Rotate(ILI9341_Orientation_t orientation);

/**
 * @brief Dibuja un píxel en la pantalla LCD.
 *
 * @param[in] x     Coordenada X del píxel.
 * @param[in] y     Coordenada Y del píxel.
 * @param[in] color Color del píxel en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Dibuja una línea en la pantalla LCD usando el algoritmo de Bresenham.
 *
 * @param[in] x0    Coordenada X de inicio.
 * @param[in] y0    Coordenada Y de inicio.
 * @param[in] x1    Coordenada X de fin.
 * @param[in] y1    Coordenada Y de fin.
 * @param[in] color Color de la línea en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Dibuja el contorno de un rectángulo en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X superior izquierda.
 * @param[in] y0    Coordenada Y superior izquierda.
 * @param[in] x1    Coordenada X inferior derecha.
 * @param[in] y1    Coordenada Y inferior derecha.
 * @param[in] color Color de la línea en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Dibuja un rectángulo relleno en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X superior izquierda.
 * @param[in] y0    Coordenada Y superior izquierda.
 * @param[in] x1    Coordenada X inferior derecha.
 * @param[in] y1    Coordenada Y inferior derecha.
 * @param[in] color Color de relleno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Dibuja el contorno de un círculo en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] r     Radio en píxeles.
 * @param[in] color Color de la línea en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

/**
 * @brief Dibuja un círculo relleno en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] r     Radio en píxeles.
 * @param[in] color Color de relleno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

/* --- Texto en pantalla ---------------------------------------------------- */

/**
 * @brief Renderiza un carácter en la pantalla LCD.
 *
 * @param[in] x          Coordenada X superior izquierda de la celda del carácter.
 * @param[in] y          Coordenada Y superior izquierda de la celda del carácter.
 * @param[in] c          Carácter a mostrar.
 * @param[in] font       Puntero a la definición de la fuente.
 * @param[in] foreground Color de primer plano en formato RGB565.
 * @param[in] background Color de fondo en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_Putc(uint16_t x, uint16_t y, char c, LCD_FontDef_t* font, uint16_t foreground, uint16_t background);

/**
 * @brief Renderiza una cadena terminada en nulo en la pantalla LCD.
 *
 * @param[in] x          Coordenada X superior izquierda del primer carácter.
 * @param[in] y          Coordenada Y superior izquierda del primer carácter.
 * @param[in] str        Puntero a la cadena terminada en nulo.
 * @param[in] font       Puntero a la definición de la fuente.
 * @param[in] foreground Color de primer plano en formato RGB565.
 * @param[in] background Color de fondo en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_Puts(uint16_t x, uint16_t y, char* str, LCD_FontDef_t* font, uint16_t foreground, uint16_t background);

/**
 * @brief Calcula el bounding-box en píxeles de una cadena para una fuente dada.
 *
 * @param[in]  str    Puntero a la cadena terminada en nulo.
 * @param[in]  font   Puntero a la definición de la fuente.
 * @param[out] width  Ancho total en píxeles.
 * @param[out] height Alto total en píxeles.
 */
void ILI9341_GetStringSize(char* str, LCD_FontDef_t* font, uint16_t* width, uint16_t* height);

/* --- Imagen completa ------------------------------------------------------ */

/**
 * @brief Transfiere un frame buffer RGB565 de pantalla completa a la LCD mediante SPI optimizado.
 *
 * @param[in] image Arreglo de IMG_TOTAL_BUF32 palabras uint32_t (dos píxeles RGB565 por palabra).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si todos los píxeles fueron enviados.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_TIMEOUT         si el bus SPI se bloqueó.
 *         - ILI9341_ERROR           si el periférico SPI estaba ocupado.
 */
ILI9341_Status_t ILI9341_DisplayImage(uint32_t image[IMG_TOTAL_BUF32]);

/* --- Frame buffer (escritura fuera de pantalla) --------------------------- */

/**
 * @brief Escribe un píxel en un frame buffer fuera de pantalla.
 *
 * @param[in]     x      Coordenada X del píxel.
 * @param[in]     y      Coordenada Y del píxel.
 * @param[in]     color  Color del píxel en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
void ILI9341_DrawPixel_ImageBuffer(uint16_t x, uint16_t y, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);

/**
 * @brief Renderiza un carácter en un frame buffer fuera de pantalla.
 *
 * @param[in]     x          Coordenada X superior izquierda de la celda del carácter.
 * @param[in]     y          Coordenada Y superior izquierda de la celda del carácter.
 * @param[in]     c          Carácter a mostrar.
 * @param[in]     font       Puntero a la definición de la fuente.
 * @param[in]     foreground Color de primer plano en formato RGB565.
 * @param[in,out] image      Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
void ILI9341_Putc_ImageBuffer(uint16_t x, uint16_t y, char c, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32]);

/**
 * @brief Renderiza una cadena terminada en nulo en un frame buffer fuera de pantalla.
 *
 * @param[in]     x          Coordenada X superior izquierda del primer carácter.
 * @param[in]     y          Coordenada Y superior izquierda del primer carácter.
 * @param[in]     str        Puntero a la cadena terminada en nulo.
 * @param[in]     font       Puntero a la definición de la fuente.
 * @param[in]     foreground Color de primer plano en formato RGB565.
 * @param[in,out] image      Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
void ILI9341_Puts_ImageBuffer(uint16_t x, uint16_t y, char* str, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32]);

/**
 * @brief Dibuja una línea en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X de inicio.
 * @param[in]     y0     Coordenada Y de inicio.
 * @param[in]     x1     Coordenada X de fin.
 * @param[in]     y1     Coordenada Y de fin.
 * @param[in]     color  Color de la línea en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
void ILI9341_DrawLine_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);

/**
 * @brief Dibuja el contorno de un rectángulo en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X superior izquierda.
 * @param[in]     y0     Coordenada Y superior izquierda.
 * @param[in]     x1     Coordenada X inferior derecha.
 * @param[in]     y1     Coordenada Y inferior derecha.
 * @param[in]     color  Color de la línea en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
void ILI9341_DrawRectangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);

/**
 * @brief Dibuja un rectángulo relleno en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X superior izquierda.
 * @param[in]     y0     Coordenada Y superior izquierda.
 * @param[in]     x1     Coordenada X inferior derecha.
 * @param[in]     y1     Coordenada Y inferior derecha.
 * @param[in]     color  Color de relleno en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
void ILI9341_DrawFilledRectangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);

/* --- Frame buffer SDRAM --------------------------------------------------- */

#ifdef HAL_SDRAM_MODULE_ENABLED

/**
 * @brief Transfiere el frame buffer interno (SDRAM) a la pantalla LCD mediante SPI.
 *
 * @details Equivalente a llamar ILI9341_DisplayImage() con el puntero interno
 *          al frame buffer en SDRAM. Requiere que ILI9341_Init() haya sido
 *          invocado con un handle SDRAM válido.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si la transferencia fue exitosa.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si el frame buffer SDRAM no está habilitado (hsdram era NULL).
 *         - ILI9341_TIMEOUT         si el bus SPI se bloqueó.
 *         - ILI9341_ERROR           si el periférico SPI estaba ocupado.
 */
ILI9341_Status_t ILI9341_Flush(void);

/**
 * @brief Retorna el puntero al frame buffer interno ubicado en SDRAM.
 *
 * @details El buffer tiene formato IMG_TOTAL_BUF32 palabras uint32_t (dos píxeles
 *          RGB565 empaquetados por palabra), compatible con todas las funciones
 *          ILI9341_*_ImageBuffer(). Retorna NULL si SDRAM no fue habilitada
 *          o si el driver no ha sido inicializado.
 *
 * @return Puntero al frame buffer en SDRAM, o NULL si no está disponible.
 */
uint32_t* ILI9341_GetFrameBuffer(void);

#endif /* HAL_SDRAM_MODULE_ENABLED */

/**
 * @brief Desinicializa el driver LCD y libera los recursos periféricos.
 *
 * @details Marca el driver como no inicializado y pone a NULL los handles
 *          internos de SPI e I2C. Si HAL_SDRAM_MODULE_ENABLED está definido y
 *          la SDRAM fue habilitada en Init(), también llama a HAL_SDRAM_DeInit()
 *          y limpia el puntero al frame buffer. Tras esta llamada es necesario
 *          invocar ILI9341_Init() antes de usar cualquier otra función.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si la desinicialización fue exitosa.
 *         - ILI9341_NOT_INITIALIZED si el driver no estaba inicializado.
 */
ILI9341_Status_t ILI9341_DeInit(void);

/* --- Touch panel (STMPE811) ---------------------------------------------- */

/**
 * @brief Configura el controlador del panel táctil STMPE811.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si el dispositivo fue detectado y configurado.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si el ID del chip no coincidió con STMPE811_ID.
 */
ILI9341_Status_t ILI9341_TP_Config(void);

/**
 * @brief Lee el estado actual del panel táctil (coordenadas y detección de toque).
 *
 * @return Puntero a la estructura TP_STATE interna con valores actualizados,
 *         o NULL si el driver no ha sido inicializado.
 */
TP_STATE* ILI9341_TP_GetState(void);

/**
 * @brief Lee la coordenada X calibrada del punto de toque activo.
 *
 * @return Coordenada X en el rango [0, 239], o 0 si no está inicializado.
 */
uint16_t ILI9341_TP_Read_X(void);

/**
 * @brief Lee la coordenada Y calibrada del punto de toque activo.
 *
 * @return Coordenada Y en el rango [0, 319], o 0 si no está inicializado.
 */
uint16_t ILI9341_TP_Read_Y(void);

/**
 * @brief Lee el valor Z (presión) del punto de toque activo.
 *
 * @return Índice de presión (valor ADC crudo), o 0 si no está inicializado.
 */
uint16_t ILI9341_TP_Read_Z(void);

/**
 * @brief Reinicia el STMPE811 mediante el bit de reset por software SYS_CTRL1.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              siempre, una vez inicializado.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 */
ILI9341_Status_t ILI9341_TP_Reset(void);

/**
 * @brief Habilita o deshabilita bloques funcionales del STMPE811 (ADC, panel táctil, GPIO).
 *
 * @param[in] Fct      Máscara de función: TP_ADC_FCT, TP_TP_FCT, o TP_IO_FCT.
 * @param[in] NewState ENABLE o DISABLE.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              siempre, una vez inicializado.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 */
ILI9341_Status_t ILI9341_TP_FnctCmd(uint8_t Fct, FunctionalState NewState);

/**
 * @brief Configura el modo de función alternativa para los pines GPIO del STMPE811.
 *
 * @param[in] IO_Pin   Máscara de pin (valores IO_Pin_x).
 * @param[in] NewState ENABLE o DISABLE.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              siempre, una vez inicializado.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 */
ILI9341_Status_t ILI9341_TP_IOAFConfig(uint8_t IO_Pin, FunctionalState NewState);

/**
 * @brief Lee un byte de un registro del STMPE811 por I2C.
 *
 * @param[in] RegisterAddr Dirección del registro (0x00–0x59).
 * @return Valor del registro, o 0xAA si hay error I2C o no está inicializado.
 */
uint8_t ILI9341_TP_ReadDeviceRegister(uint8_t RegisterAddr);

/**
 * @brief Escribe un byte en un registro del STMPE811 por I2C.
 *
 * @param[in] RegisterAddr  Dirección del registro.
 * @param[in] RegisterValue Byte a escribir.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la comunicación I2C.
 */
ILI9341_Status_t ILI9341_TP_WriteDeviceRegister(uint8_t RegisterAddr, uint8_t RegisterValue);

/**
 * @brief Lee dos bytes de un registro del STMPE811 (usado para datos ADC de X/Y/Z).
 *
 * @param[in] RegisterAddr Dirección del registro.
 * @return Valor reconstruido de 16 bits, o 0xAA si hay error I2C o no está inicializado.
 */
uint16_t ILI9341_TP_ReadDataBuffer(uint32_t RegisterAddr);

#ifdef __cplusplus
}
#endif

#endif /* ILI9341_DISC1_H */
