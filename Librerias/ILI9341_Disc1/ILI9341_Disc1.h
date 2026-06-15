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
 * @origin El código de este driver se basa en la librería Petr Machala, Tilen Majerle, 2014.
 * @author Dr. Luis Antonio Raygoza Pérez & Ing. Daniel Ruiz
 * @date June 14, 2026
 * @version 1.1.0
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
#define ILI9341_RST_SET     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET)    /**< Desactiva el reset por hardware del LCD (PD12 → HIGH).  */
#define ILI9341_RST_RESET   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET)  /**< Activa el reset por hardware del LCD (PD12 → LOW).      */
#define ILI9341_CS_SET      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,  GPIO_PIN_SET)    /**< Deselecciona el chip SPI del LCD (PC2 → HIGH).          */
#define ILI9341_CS_RESET    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,  GPIO_PIN_RESET)  /**< Selecciona el chip SPI del LCD (PC2 → LOW).             */
#define ILI9341_WRX_SET     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET)    /**< Indica modo datos en la línea D/C del LCD (PD13 → HIGH).*/
#define ILI9341_WRX_RESET   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET)  /**< Indica modo comando en la línea D/C del LCD (PD13 → LOW).*/

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
#define ILI9341_SDRAM_FB_SIZE            (IMG_TOTAL_BUF32 * 4U) /**< Tamaño de cada frame-buffer en SDRAM en bytes (153 600 B) */

#define IS42S16400J_SIZE                 0x400000U /**< Capacidad total del chip IS42S16400J: 4 MB */

/* Longitud de ráfaga */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000U) /**< Longitud de ráfaga: 1 palabra                        */
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001U) /**< Longitud de ráfaga: 2 palabras                       */
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002U) /**< Longitud de ráfaga: 4 palabras                       */
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004U) /**< Longitud de ráfaga: 8 palabras                       */

/* Tipo de ráfaga */
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000U) /**< Ráfaga secuencial                                    */
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008U) /**< Ráfaga intercalada                                   */

/* Latencia CAS */
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020U) /**< Latencia CAS de 2 ciclos de reloj                    */
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030U) /**< Latencia CAS de 3 ciclos de reloj                    */

/* Modo de operación */
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000U) /**< Modo de operación estándar (único soportado)         */

/* Modo de escritura en ráfaga */
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000U) /**< Escritura en ráfaga igual a la longitud de lectura   */
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200U) /**< Escritura en modo de acceso simple (un solo acceso)  */

/* Contador de refresco para reloj SDRAM a 90 MHz */
#define REFRESH_COUNT                            ((uint32_t)0x056AU) /**< Valor del contador de refresco para SDRAM a 90 MHz   */

#endif /* HAL_SDRAM_MODULE_ENABLED */

/* -- Colores predefinidos (RGB565) -- */
#define ILI9341_COLOR_WHITE     0xFFFFU /**< Blanco          */
#define ILI9341_COLOR_BLACK     0x0000U /**< Negro           */
#define ILI9341_COLOR_RED       0xF800U /**< Rojo puro       */
#define ILI9341_COLOR_GREEN     0x07E0U /**< Verde puro      */
#define ILI9341_COLOR_GREEN2    0xB723U /**< Verde oscuro    */
#define ILI9341_COLOR_BLUE      0x001FU /**< Azul puro       */
#define ILI9341_COLOR_BLUE2     0x051DU /**< Azul oscuro     */
#define ILI9341_COLOR_YELLOW    0xFFE0U /**< Amarillo        */
#define ILI9341_COLOR_ORANGE    0xFBE4U /**< Naranja         */
#define ILI9341_COLOR_CYAN      0x07FFU /**< Cian            */
#define ILI9341_COLOR_MAGENTA   0xA254U /**< Magenta         */
#define ILI9341_COLOR_GRAY      0x7BEFU /**< Gris medio      */
#define ILI9341_COLOR_BROWN     0xBBCAU /**< Café/marrón     */

/* -- Comandos ILI9341 -- */
#define ILI9341_RESET           0x01U /**< Reinicio por software                                  */
#define ILI9341_SLEEP_OUT       0x11U /**< Sale del modo Sleep                                    */
#define ILI9341_GAMMA           0x26U /**< Selección de curva gamma                               */
#define ILI9341_DISPLAY_OFF     0x28U /**< Apaga la pantalla                                      */
#define ILI9341_DISPLAY_ON      0x29U /**< Enciende la pantalla                                   */
#define ILI9341_COLUMN_ADDR     0x2AU /**< Establece la ventana de columna (eje X)                */
#define ILI9341_PAGE_ADDR       0x2BU /**< Establece la ventana de página (eje Y)                 */
#define ILI9341_GRAM            0x2CU /**< Escritura en GRAM (inicio de transferencia de píxeles) */
#define ILI9341_MAC             0x36U /**< Control de acceso a memoria (rotación y espejo)        */
#define ILI9341_PIXEL_FORMAT    0x3AU /**< Formato de píxel (0x55 = 16 bpp RGB565)               */
#define ILI9341_WDB             0x51U /**< Escritura de brillo de pantalla                        */
#define ILI9341_WCD             0x53U /**< Escritura de control de display                        */
#define ILI9341_RGB_INTERFACE   0xB0U /**< Control de señal de interfaz RGB                       */
#define ILI9341_FRC             0xB1U /**< Control de tasa de fotogramas (modo normal)             */
#define ILI9341_BPC             0xB5U /**< Control de umbral de retroiluminación                  */
#define ILI9341_DFC             0xB6U /**< Control de función de pantalla                         */
#define ILI9341_POWER1          0xC0U /**< Control de potencia 1                                  */
#define ILI9341_POWER2          0xC1U /**< Control de potencia 2                                  */
#define ILI9341_VCOM1           0xC5U /**< Control VCOM 1                                         */
#define ILI9341_VCOM2           0xC7U /**< Control VCOM 2                                         */
#define ILI9341_POWERA          0xCBU /**< Secuencia de control de potencia A (extendido)         */
#define ILI9341_POWERB          0xCFU /**< Secuencia de control de potencia B (extendido)         */
#define ILI9341_PGAMMA          0xE0U /**< Corrección de gamma positiva                           */
#define ILI9341_NGAMMA          0xE1U /**< Corrección de gamma negativa                           */
#define ILI9341_DTCA            0xE8U /**< Temporización del driver A                             */
#define ILI9341_DTCB            0xEAU /**< Temporización del driver B                             */
#define ILI9341_POWER_SEQ       0xEDU /**< Secuencia de encendido (Power-on sequence)             */
#define ILI9341_3GAMMA_EN       0xF2U /**< Habilitación de corrección gamma de 3 bits             */
#define ILI9341_INTERFACE       0xF6U /**< Control de interfaz                                    */
#define ILI9341_PRC             0xF7U /**< Control de bomba de carga (Pump Ratio Control)         */

#ifdef HAL_I2C_MODULE_ENABLED
/* -- Touch panel STMPE811 -- */
#define TP_ADDR                 0x82U   /**< Dirección I2C del STMPE811 (sin bit R/W)            */
#define STMPE811_ID             0x0811U /**< ID de chip esperado (registros 0x00–0x01)            */

/* Registros de identificación */
#define TP_REG_CHP_ID           0x00U  /**< ID del chip (2 bytes: 0x08, 0x11)                    */
#define TP_REG_ID_VER           0x02U  /**< Versión de revisión del chip                         */

/* Registros de control general */
#define TP_REG_SYS_CTRL1        0x03U  /**< Control del sistema 1 (reset por software, bit 1)    */
#define TP_REG_SYS_CTRL2        0x04U  /**< Control del sistema 2 (apagado de bloques ADC/TP/IO) */
#define TP_REG_SPI_CFG          0x08U  /**< Configuración de la interfaz SPI                     */

/* Registros de control de interrupciones */
#define TP_REG_INT_CTRL         0x09U  /**< Control de interrupciones (polaridad, tipo)          */
#define TP_REG_INT_EN           0x0AU  /**< Habilitación de fuentes de interrupción              */
#define TP_REG_INT_STA          0x0BU  /**< Estado de interrupciones (escribir 1 para limpiar)   */
#define TP_REG_GPIO_INT_EN      0x0CU  /**< Habilitación de interrupción por pin GPIO            */
#define TP_REG_GPIO_INT_STA     0x0DU  /**< Estado de interrupción por pin GPIO                  */

/* Registros ADC */
#define TP_REG_ADC_INT_EN       0x0EU  /**< Habilitación de interrupción ADC por canal           */
#define TP_REG_ADC_INT_STA      0x0FU  /**< Estado de interrupción ADC por canal                 */
#define TP_REG_ADC_CTRL1        0x20U  /**< Control ADC 1 (resolución y tiempo de muestreo)      */
#define TP_REG_ADC_CTRL2        0x21U  /**< Control ADC 2 (frecuencia de reloj ADC)              */
#define TP_REG_ADC_CAPT         0x22U  /**< Captura ADC (inicio de conversión por canal)         */
#define TP_REG_ADC_DATA_CH0     0x30U  /**< Resultado ADC canal 0 (2 bytes)                      */
#define TP_REG_ADC_DATA_CH1     0x32U  /**< Resultado ADC canal 1 (2 bytes)                      */
#define TP_REG_ADC_DATA_CH2     0x34U  /**< Resultado ADC canal 2 (2 bytes)                      */
#define TP_REG_ADC_DATA_CH3     0x36U  /**< Resultado ADC canal 3 (2 bytes)                      */
#define TP_REG_ADC_DATA_CH4     0x38U  /**< Resultado ADC canal 4 (2 bytes)                      */
#define TP_REG_ADC_DATA_CH5     0x3AU  /**< Resultado ADC canal 5 (2 bytes)                      */
#define TP_REG_ADC_DATA_CH6     0x3BU  /**< Resultado ADC canal 6 (2 bytes)                      */
#define TP_REG_ADC_DATA_CH7     0x3CU  /**< Resultado ADC canal 7 (2 bytes)                      */

/* Registros GPIO */
#define TP_REG_GPIO_SET_PIN     0x10U  /**< Pone en HIGH los pines indicados por máscara         */
#define TP_REG_GPIO_CLR_PIN     0x11U  /**< Pone en LOW los pines indicados por máscara          */
#define TP_REG_GPIO_MP_STA      0x12U  /**< Estado actual de los pines GPIO (lectura)            */
#define TP_REG_GPIO_DIR         0x13U  /**< Dirección de los pines GPIO (1 = salida)             */
#define TP_REG_GPIO_ED          0x14U  /**< Habilitación de detección de flanco por pin          */
#define TP_REG_GPIO_RE          0x15U  /**< Habilitación de detección de flanco de subida        */
#define TP_REG_GPIO_FE          0x16U  /**< Habilitación de detección de flanco de bajada        */
#define TP_REG_GPIO_AF          0x17U  /**< Función alternativa GPIO (1 = AF, 0 = GPIO normal)   */

/* Registros del panel táctil */
#define TP_REG_TP_CTRL          0x40U  /**< Control del panel táctil; bit 7 = toque detectado    */
#define TP_REG_TP_CFG           0x41U  /**< Configuración del panel táctil (modo, averaging)     */
#define TP_REG_WDM_TR_X         0x42U  /**< Ventana de detección: esquina superior derecha X     */
#define TP_REG_WDM_TR_Y         0x44U  /**< Ventana de detección: esquina superior derecha Y     */
#define TP_REG_WDM_BL_X         0x46U  /**< Ventana de detección: esquina inferior izquierda X   */
#define TP_REG_WDM_BL_Y         0x48U  /**< Ventana de detección: esquina inferior izquierda Y   */
#define TP_REG_FIFO_TH          0x4AU  /**< Umbral del FIFO táctil (en número de muestras)       */
#define TP_REG_FIFO_STA         0x4BU  /**< Estado del FIFO (escribir 0x01 para limpiar)         */
#define TP_REG_FIFO_SIZE        0x4CU  /**< Número de muestras disponibles en el FIFO            */
#define TP_REG_TP_DATA_X        0x4DU  /**< Coordenada X cruda del punto de toque (2 bytes)      */
#define TP_REG_TP_DATA_Y        0x4FU  /**< Coordenada Y cruda del punto de toque (2 bytes)      */
#define TP_REG_TP_DATA_Z        0x51U  /**< Presión Z cruda del punto de toque (2 bytes)         */
#define TP_REG_TP_DATA_XYZ      0x52U  /**< Datos XYZ combinados del punto de toque              */
#define TP_REG_TP_FRACT_XYZ     0x56U  /**< Control de fracción de resolución XYZ                */
#define TP_REG_TP_DATA          0x57U  /**< Registro de datos táctiles del FIFO                  */
#define TP_REG_TP_I_DRIVE       0x58U  /**< Corriente del driver del panel táctil                */
#define TP_REG_TP_SHIELD        0x59U  /**< Configuración del shield (apantallamiento)           */

/* Funcionalidades del expansor de IO */
#define TP_ADC_FCT              0x01U  /**< Máscara de función ADC en SYS_CTRL2                  */
#define TP_TP_FCT               0x02U  /**< Máscara de función panel táctil en SYS_CTRL2         */
#define TP_IO_FCT               0x04U  /**< Máscara de función GPIO en SYS_CTRL2                 */

/* Pines de IO */
#define IO_Pin_0                0x01U  /**< GPIO 0 del STMPE811 */
#define IO_Pin_1                0x02U  /**< GPIO 1 del STMPE811 */
#define IO_Pin_2                0x04U  /**< GPIO 2 del STMPE811 */
#define IO_Pin_3                0x08U  /**< GPIO 3 del STMPE811 */
#define IO_Pin_4                0x10U  /**< GPIO 4 del STMPE811 */
#define IO_Pin_5                0x20U  /**< GPIO 5 del STMPE811 */
#define IO_Pin_6                0x40U  /**< GPIO 6 del STMPE811 */
#define IO_Pin_7                0x80U  /**< GPIO 7 del STMPE811 */
#define IO_Pin_ALL              0xFFU  /**< Todos los GPIOs del STMPE811 */

/* Mapeo de pines I/O del panel táctil */
#define TOUCH_YD                IO_Pin_1 /**< Electrodo inferior del eje Y  (GPIO 1) */
#define TOUCH_XD                IO_Pin_2 /**< Electrodo derecho del eje X   (GPIO 2) */
#define TOUCH_YU                IO_Pin_3 /**< Electrodo superior del eje Y  (GPIO 3) */
#define TOUCH_XU                IO_Pin_4 /**< Electrodo izquierdo del eje X (GPIO 4) */
#define TOUCH_IO_ALL            ((uint32_t)(IO_Pin_1 | IO_Pin_2 | IO_Pin_3 | IO_Pin_4)) /**< Máscara de todos los pines táctiles */
#endif /* HAL_I2C_MODULE_ENABLED */

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

#ifdef HAL_I2C_MODULE_ENABLED
/**
 * @brief Estado del panel táctil retornado por ILI9341_TP_GetState().
 */
typedef struct {
    uint16_t TouchDetected;     /**< Distinto de cero cuando hay toque activo */
    uint16_t X;                 /**< Coordenada X calibrada [0, 239]          */
    uint16_t Y;                 /**< Coordenada Y calibrada [0, 319]          */
    uint16_t Z;                 /**< Índice de presión (valor ADC crudo)      */
} TP_STATE;
#endif /* HAL_I2C_MODULE_ENABLED */

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/* --- Inicialización ------------------------------------------------------- */

/**
 * @brief Inicializa la pantalla LCD ILI9341.
 *
 * @details La firma varía según los módulos HAL habilitados en stm32f4xx_hal_conf.h:
 *          | HAL_SDRAM_MODULE_ENABLED | HAL_I2C_MODULE_ENABLED | HAL_DMA2D_MODULE_ENABLED | Firma resultante                              |
 *          |--------------------------|------------------------|--------------------------|-----------------------------------------------|
 *          | No                       | No                     | No                       | ILI9341_Init(hspi)                            |
 *          | No                       | No                     | Sí                       | ILI9341_Init(hspi, hdma2d)                    |
 *          | No                       | Sí                     | No                       | ILI9341_Init(hspi, hi2c)                      |
 *          | No                       | Sí                     | Sí                       | ILI9341_Init(hspi, hi2c, hdma2d)              |
 *          | Sí                       | No                     | No                       | ILI9341_Init(hspi, hsdram)                    |
 *          | Sí                       | No                     | Sí                       | ILI9341_Init(hspi, hsdram, hdma2d)            |
 *          | Sí                       | Sí                     | No                       | ILI9341_Init(hspi, hi2c, hsdram)              |
 *          | Sí                       | Sí                     | Sí                       | ILI9341_Init(hspi, hi2c, hsdram, hdma2d)      |
 *
 * @param[in] hspi   Puntero al handle SPI de HAL (obligatorio).
 * @param[in] hi2c   (Solo con HAL_I2C_MODULE_ENABLED) Puntero al handle I2C de HAL.
 * @param[in] hsdram (Solo con HAL_SDRAM_MODULE_ENABLED) Puntero al handle SDRAM de HAL
 *                   generado por STM32CubeMX. Pasar NULL deshabilita el frame buffer en SDRAM.
 *                   Cuando no es NULL, la librería reserva los primeros ILI9341_SDRAM_FB_SIZE
 *                   bytes de ILI9341_SDRAM_BASE como frame buffer interno (153 600 B).
 * @param[in] hdma2d (Solo con HAL_DMA2D_MODULE_ENABLED) Puntero al handle DMA2D de HAL
 *                   generado por STM32CubeMX. Pasar NULL deshabilita la aceleración DMA2D:
 *                   los rellenos usan el camino CPU y ILI9341_BlitImage no realiza ninguna copia.
 * @return ILI9341_Status_t
 *         - ILI9341_OK            si la inicialización fue exitosa.
 *         - ILI9341_INVALID_PARAM si @p hspi es NULL, o @p hi2c es NULL cuando I2C está habilitado.
 *         - ILI9341_ERROR         si una transmisión SPI o la configuración DMA2D falló durante la inicialización.
 */
#if defined(HAL_SDRAM_MODULE_ENABLED) && defined(HAL_I2C_MODULE_ENABLED) && defined(HAL_DMA2D_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, SDRAM_HandleTypeDef* hsdram, DMA2D_HandleTypeDef* hdma2d);
#elif defined(HAL_SDRAM_MODULE_ENABLED) && defined(HAL_I2C_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, SDRAM_HandleTypeDef* hsdram);
#elif defined(HAL_SDRAM_MODULE_ENABLED) && defined(HAL_DMA2D_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi, SDRAM_HandleTypeDef* hsdram, DMA2D_HandleTypeDef* hdma2d);
#elif defined(HAL_SDRAM_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi, SDRAM_HandleTypeDef* hsdram);
#elif defined(HAL_I2C_MODULE_ENABLED) && defined(HAL_DMA2D_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, DMA2D_HandleTypeDef* hdma2d);
#elif defined(HAL_I2C_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c);
#elif defined(HAL_DMA2D_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi, DMA2D_HandleTypeDef* hdma2d);
#else
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi);
#endif

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
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito (incluye píxel fuera de límites, que se ignora).
 *         - ILI9341_INVALID_PARAM   si @p image es NULL.
 */
ILI9341_Status_t ILI9341_DrawPixel_ImageBuffer(uint16_t x, uint16_t y, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);

/**
 * @brief Renderiza un carácter en un frame buffer fuera de pantalla.
 *
 * @param[in]     x          Coordenada X superior izquierda de la celda del carácter.
 * @param[in]     y          Coordenada Y superior izquierda de la celda del carácter.
 * @param[in]     c          Carácter a mostrar.
 * @param[in]     font       Puntero a la definición de la fuente.
 * @param[in]     foreground Color de primer plano en formato RGB565.
 * @param[in,out] image      Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito (incluye carácter fuera de rango ASCII, que se ignora).
 *         - ILI9341_INVALID_PARAM   si @p font o @p image son NULL.
 */
ILI9341_Status_t ILI9341_Putc_ImageBuffer(uint16_t x, uint16_t y, char c, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32]);

/**
 * @brief Renderiza una cadena terminada en nulo en un frame buffer fuera de pantalla.
 *
 * @param[in]     x          Coordenada X superior izquierda del primer carácter.
 * @param[in]     y          Coordenada Y superior izquierda del primer carácter.
 * @param[in]     str        Puntero a la cadena terminada en nulo.
 * @param[in]     font       Puntero a la definición de la fuente.
 * @param[in]     foreground Color de primer plano en formato RGB565.
 * @param[in,out] image      Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_INVALID_PARAM   si @p str, @p font o @p image son NULL.
 */
ILI9341_Status_t ILI9341_Puts_ImageBuffer(uint16_t x, uint16_t y, char* str, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32]);

/**
 * @brief Dibuja una línea en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X de inicio.
 * @param[in]     y0     Coordenada Y de inicio.
 * @param[in]     x1     Coordenada X de fin.
 * @param[in]     y1     Coordenada Y de fin.
 * @param[in]     color  Color de la línea en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_INVALID_PARAM   si @p image es NULL.
 */
ILI9341_Status_t ILI9341_DrawLine_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);

/**
 * @brief Dibuja el contorno de un rectángulo en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X superior izquierda.
 * @param[in]     y0     Coordenada Y superior izquierda.
 * @param[in]     x1     Coordenada X inferior derecha.
 * @param[in]     y1     Coordenada Y inferior derecha.
 * @param[in]     color  Color de la línea en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_INVALID_PARAM   si @p image es NULL.
 */
ILI9341_Status_t ILI9341_DrawRectangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);

/**
 * @brief Dibuja un rectángulo relleno en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X superior izquierda.
 * @param[in]     y0     Coordenada Y superior izquierda.
 * @param[in]     x1     Coordenada X inferior derecha.
 * @param[in]     y1     Coordenada Y inferior derecha.
 * @param[in]     color  Color de relleno en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_INVALID_PARAM   si @p image es NULL.
 *         - ILI9341_ERROR           si falla la transferencia DMA2D (solo con HAL_DMA2D_MODULE_ENABLED).
 */
ILI9341_Status_t ILI9341_DrawFilledRectangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);

/**
 * @brief Dibuja un círculo relleno en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X del centro.
 * @param[in]     y0     Coordenada Y del centro.
 * @param[in]     r      Radio en píxeles.
 * @param[in]     color  Color de relleno en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_INVALID_PARAM   si @p image es NULL.
 *         - ILI9341_ERROR           si falla una transferencia DMA2D interna (solo con HAL_DMA2D_MODULE_ENABLED).
 */
ILI9341_Status_t ILI9341_DrawFilledCircle_ImageBuffer(int16_t x0, int16_t y0, int16_t r, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);

#ifdef HAL_DMA2D_MODULE_ENABLED
/**
 * @brief Copia una imagen RGB565 al frame buffer usando DMA2D (memoria a memoria).
 *
 * @param[in]     src         Puntero a la imagen fuente en formato RGB565.
 * @param[in]     x0          Coordenada X de la esquina superior izquierda en el frame buffer.
 * @param[in]     y0          Coordenada Y de la esquina superior izquierda en el frame buffer.
 * @param[in]     img_w       Ancho de la imagen fuente en píxeles.
 * @param[in]     img_h       Alto de la imagen fuente en píxeles.
 * @param[in,out] framebuffer Frame buffer destino (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito (incluye x0/y0 fuera de pantalla, que se ignora).
 *         - ILI9341_INVALID_PARAM   si @p src, @p framebuffer son NULL, o si el handle DMA2D no fue inyectado.
 *         - ILI9341_ERROR           si falla la transferencia DMA2D.
 */
ILI9341_Status_t ILI9341_BlitImage(const uint16_t* src, uint16_t x0, uint16_t y0,
                                    uint16_t img_w, uint16_t img_h,
                                    uint32_t* framebuffer);
#endif /* HAL_DMA2D_MODULE_ENABLED */

/* --- Frame buffer SDRAM --------------------------------------------------- */

#ifdef HAL_SDRAM_MODULE_ENABLED

/**
 * @brief Presenta el frame dibujado en pantalla usando doble buffer con pipelining DMA.
 *
 * @details Internamente implementa el patrón front/back buffer de forma transparente:
 *          -# Espera a que termine el DMA del frame anterior (si hay uno activo).
 *          -# Intercambia los punteros front/back.
 *          -# Inicia el DMA sobre el nuevo front buffer y retorna sin bloquear.
 *
 *          La CPU puede empezar a dibujar el siguiente frame inmediatamente después de
 *          que Flush() retorne, mientras el DMA envía el frame actual a la pantalla.
 *          El puntero devuelto por ILI9341_GetFrameBuffer() cambia tras cada llamada
 *          a Flush(), por lo que debe invocarse de nuevo para obtener el back buffer activo.
 *
 *          Uso típico:
 *          @code
 *          while (1) {
 *              uint32_t *fb = ILI9341_GetFrameBuffer();
 *              draw_scene(fb);   // dibuja mientras el DMA envía el frame anterior
 *              ILI9341_Flush();  // espera DMA, swap, arranca DMA del frame recién dibujado
 *          }
 *          @endcode
 *
 * @note No usar funciones de dibujo directo en pantalla (SPI) entre Flush() y el siguiente
 *       ILI9341_GetFrameBuffer(), ya que el DMA puede estar ocupando el bus SPI.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si el swap y el nuevo DMA se iniciaron correctamente.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si el frame buffer SDRAM no está habilitado (hsdram era NULL).
 *         - ILI9341_TIMEOUT         si el DMA anterior no terminó en 5 000 ms.
 *         - ILI9341_ERROR           si HAL_SPI_Transmit_DMA falló.
 */
ILI9341_Status_t ILI9341_Flush(void);

/**
 * @brief Espera a que concluya el DMA en curso y restaura el bus SPI al modo 8 bits.
 *
 * @details Llamar al salir del modo de doble buffer antes de usar funciones de dibujo
 *          directo en pantalla (ILI9341_Fill, ILI9341_DrawPixel, etc.).
 *          Si no hay DMA activo retorna inmediatamente sin efecto.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si el bus quedó libre correctamente.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_TIMEOUT         si el DMA no terminó en 5 000 ms.
 */
ILI9341_Status_t ILI9341_Sync(void);

/**
 * @brief Retorna el puntero al buffer de dibujo activo en SDRAM (back buffer).
 *
 * @details Devuelve siempre el buffer sobre el que debe dibujar la CPU — el back buffer
 *          en el esquema de doble buffer interno. El puntero cambia tras cada llamada a
 *          ILI9341_Flush(), por lo que hay que invocarlo de nuevo en cada frame.
 *          El buffer tiene formato IMG_TOTAL_BUF32 palabras uint32_t (dos píxeles RGB565
 *          por palabra), compatible con todas las funciones ILI9341_*_ImageBuffer().
 *          Retorna NULL si SDRAM no fue habilitada o si el driver no ha sido inicializado.
 *
 * @return Puntero al back buffer en SDRAM, o NULL si no está disponible.
 */
uint32_t* ILI9341_GetFrameBuffer(void);

#endif /* HAL_SDRAM_MODULE_ENABLED */

/**
 * @brief Desinicializa el driver LCD y libera los recursos periféricos.
 *
 * @details Marca el driver como no inicializado y pone a NULL los handles
 *          internos de SPI, I2C y DMA2D. Si HAL_SDRAM_MODULE_ENABLED está definido y
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

#ifdef HAL_I2C_MODULE_ENABLED
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
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* ILI9341_DISC1_H */
