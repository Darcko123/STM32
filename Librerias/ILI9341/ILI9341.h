/**
 * @file ILI9341.h
 * @brief Driver genérico para la pantalla TFT LCD ILI9341 (320×240, interfaz SPI).
 *
 * @details Controla la pantalla TFT ILI9341 por SPI y, opcionalmente, un panel
 *          táctil. Se soportan dos controladores de touch intercambiables:
 *          - STMPE811 (I2C), configurado con ILI9341_TP_Config().
 *          - XPT2046 (SPI, comparte el bus del LCD con un CS propio), configurado
 *            con ILI9341_TP_ConfigXPT2046().
 *          Ambos exponen el mismo estado a través de ILI9341_TP_GetState(). El SPI
 *          debe inicializarse a 2 Mbit/s antes de llamar a ILI9341_Init(); tras la
 *          inicialización el preescalador se eleva a 45 Mbit/s automáticamente.
 *
 *          Los pines SCK/MOSI/MISO del SPI (y SCL/SDA del I2C del touch, si se usa
 *          el STMPE811) se configuran junto con el periférico correspondiente
 *          (p. ej. en STM32CubeMX) antes de llamar a ILI9341_Init(). Los pines CS,
 *          RESET y D/C (WRX) del LCD son de propósito general y se indican como
 *          parámetros de ILI9341_Init(); el usuario debe configurarlos como salida
 *          push-pull antes de inicializar el driver.
 *
 * @origin El código de este driver se basa en la librería Petr Machala, Tilen Majerle, 2014.
 * @author Dr. Luis Antonio Raygoza Pérez & Ing. Daniel Ruiz
 * @date July 08, 2026
 * @version 2.0.0
 */

#ifndef ILI9341_H
#define ILI9341_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include "lcd_fonts.h"
#include <string.h>
#include <math.h>
#include <float.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>

// ============================================================================
// MACROS Y CONSTANTES [ILI9341]
// ============================================================================

/* -- Dimensiones de la pantalla -- */
#define ILI9341_WIDTH       240U                        /**< Ancho de la pantalla en píxeles  */
#define ILI9341_HEIGHT      320U                        /**< Alto de la pantalla en píxeles   */
#define ILI9341_PIXEL       (ILI9341_WIDTH * ILI9341_HEIGHT)  /**< Total de píxeles           */

/* -- Tamaño del buffer de imagen -- */
#define IMG_TOTAL_BUF32     (ILI9341_PIXEL / 2U)       /**< Tamaño del frame-buffer en palabras de 32 bits (2 píxeles por palabra) */

/* -- Colores predefinidos (RGB565) -- */
#define RGB565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
#define RGB16TO24(c) ((((uint32_t)(c) & 0xF800) << 8) | (((c) & 0x07E0) << 5) | (((c) & 0x1F) << 3))

#define ILI9341_COLOR_ALICEBLUE         RGB565(240, 248, 248)
#define ILI9341_COLOR_ANTIQUEWHITE      RGB565(248, 236, 216)
#define ILI9341_COLOR_AQUA              RGB565(0, 252, 248)
#define ILI9341_COLOR_AQUAMARINE        RGB565(128, 252, 216)
#define ILI9341_COLOR_AZURE             RGB565(240, 252, 248)
#define ILI9341_COLOR_BEIGE             RGB565(248, 244, 224)
#define ILI9341_COLOR_BISQUE            RGB565(248, 228, 200)
#define ILI9341_COLOR_BLACK             RGB565(0, 0, 0)
#define ILI9341_COLOR_BLANCHEDALMOND    RGB565(248, 236, 208)
#define ILI9341_COLOR_BLUE              RGB565(0, 0, 248)
#define ILI9341_COLOR_BLUEVIOLET        RGB565(136, 44, 224)
#define ILI9341_COLOR_BROWN             RGB565(168, 44, 40)
#define ILI9341_COLOR_BURLYWOOD         RGB565(224, 184, 136)
#define ILI9341_COLOR_CADETBLUE         RGB565(96, 160, 160)
#define ILI9341_COLOR_CHARTREUSE        RGB565(128, 252, 0)
#define ILI9341_COLOR_CHOCOLATE         RGB565(208, 104, 32)
#define ILI9341_COLOR_CORAL             RGB565(248, 128, 80)
#define ILI9341_COLOR_CORNFLOWERBLUE    RGB565(104, 148, 240)
#define ILI9341_COLOR_CORNSILK          RGB565(248, 248, 224)
#define ILI9341_COLOR_CRIMSON           RGB565(224, 20, 64)
#define ILI9341_COLOR_CYAN              RGB565(0, 252, 248)
#define ILI9341_COLOR_DARKBLUE          RGB565(0, 0, 136)
#define ILI9341_COLOR_DARKCYAN          RGB565(0, 140, 136)
#define ILI9341_COLOR_DARKGOLDENROD     RGB565(184, 136, 8)
#define ILI9341_COLOR_DARKGRAY          RGB565(168, 168, 168)
#define ILI9341_COLOR_DARKGREEN         RGB565(0, 100, 0)
#define ILI9341_COLOR_DARKGREY          RGB565(168, 168, 168)
#define ILI9341_COLOR_DARKKHAKI         RGB565(192, 184, 104)
#define ILI9341_COLOR_DARKMAGENTA       RGB565(136, 0, 136)
#define ILI9341_COLOR_DARKOLIVEGREEN    RGB565(88, 108, 48)
#define ILI9341_COLOR_DARKORANGE        RGB565(248, 140, 0)
#define ILI9341_COLOR_DARKORCHID        RGB565(152, 52, 208)
#define ILI9341_COLOR_DARKRED           RGB565(136, 0, 0)
#define ILI9341_COLOR_DARKSALMON        RGB565(232, 152, 120)
#define ILI9341_COLOR_DARKSEAGREEN      RGB565(144, 188, 144)
#define ILI9341_COLOR_DARKSLATEBLUE     RGB565(72, 60, 136)
#define ILI9341_COLOR_DARKSLATEGRAY     RGB565(48, 80, 80)
#define ILI9341_COLOR_DARKSLATEGREY     RGB565(48, 80, 80)
#define ILI9341_COLOR_DARKTURQUOISE     RGB565(0, 208, 208)
#define ILI9341_COLOR_DARKVIOLET        RGB565(152, 0, 208)
#define ILI9341_COLOR_DEEPPINK          RGB565(248, 20, 144)
#define ILI9341_COLOR_DEEPSKYBLUE       RGB565(0, 192, 248)
#define ILI9341_COLOR_DIMGRAY           RGB565(104, 104, 104)
#define ILI9341_COLOR_DIMGREY           RGB565(104, 104, 104)
#define ILI9341_COLOR_DODGERBLUE        RGB565(32, 144, 248)
#define ILI9341_COLOR_FIREBRICK         RGB565(176, 36, 32)
#define ILI9341_COLOR_FLORALWHITE       RGB565(248, 252, 240)
#define ILI9341_COLOR_FORESTGREEN       RGB565(32, 140, 32)
#define ILI9341_COLOR_FUCHSIA           RGB565(248, 0, 248)
#define ILI9341_COLOR_GAINSBORO         RGB565(224, 220, 224)
#define ILI9341_COLOR_GHOSTWHITE        RGB565(248, 248, 248)
#define ILI9341_COLOR_GOLD              RGB565(248, 216, 0)
#define ILI9341_COLOR_GOLDENROD         RGB565(216, 164, 32)
#define ILI9341_COLOR_GRAY              RGB565(128, 128, 128)
#define ILI9341_COLOR_GREEN             RGB565(0, 128, 0)
#define ILI9341_COLOR_GREENYELLOW       RGB565(176, 252, 48)
#define ILI9341_COLOR_GREY              RGB565(128, 128, 128)
#define ILI9341_COLOR_HONEYDEW          RGB565(240, 252, 240)
#define ILI9341_COLOR_HOTPINK           RGB565(248, 104, 184)
#define ILI9341_COLOR_INDIANRED         RGB565(208, 92, 96)
#define ILI9341_COLOR_INDIGO            RGB565(72, 0, 128)
#define ILI9341_COLOR_IVORY             RGB565(248, 252, 240)
#define ILI9341_COLOR_KHAKI             RGB565(240, 232, 144)
#define ILI9341_COLOR_LAVENDER          RGB565(232, 232, 248)
#define ILI9341_COLOR_LAVENDERBLUSH     RGB565(248, 240, 248)
#define ILI9341_COLOR_LAWNGREEN         RGB565(128, 252, 0)
#define ILI9341_COLOR_LEMONCHIFFON      RGB565(248, 252, 208)
#define ILI9341_COLOR_LIGHTBLUE         RGB565(176, 216, 232)
#define ILI9341_COLOR_LIGHTCORAL        RGB565(240, 128, 128)
#define ILI9341_COLOR_LIGHTCYAN         RGB565(224, 252, 248)
#define ILI9341_COLOR_LIGHTGOLDENRODYELLOW RGB565(248, 252, 208)
#define ILI9341_COLOR_LIGHTGRAY         RGB565(208, 212, 208)
#define ILI9341_COLOR_LIGHTGREEN        RGB565(144, 240, 144)
#define ILI9341_COLOR_LIGHTGREY         RGB565(208, 212, 208)
#define ILI9341_COLOR_LIGHTPINK         RGB565(248, 184, 192)
#define ILI9341_COLOR_LIGHTSALMON       RGB565(248, 160, 120)
#define ILI9341_COLOR_LIGHTSEAGREEN     RGB565(32, 180, 168)
#define ILI9341_COLOR_LIGHTSKYBLUE      RGB565(136, 208, 248)
#define ILI9341_COLOR_LIGHTSLATEGRAY    RGB565(120, 136, 152)
#define ILI9341_COLOR_LIGHTSLATEGREY    RGB565(120, 136, 152)
#define ILI9341_COLOR_LIGHTSTEELBLUE    RGB565(176, 196, 224)
#define ILI9341_COLOR_LIGHTYELLOW       RGB565(248, 252, 224)
#define ILI9341_COLOR_LIME              RGB565(0, 252, 0)
#define ILI9341_COLOR_LIMEGREEN         RGB565(48, 204, 48)
#define ILI9341_COLOR_LINEN             RGB565(248, 240, 232)
#define ILI9341_COLOR_MAGENTA           RGB565(248, 0, 248)
#define ILI9341_COLOR_MAROON            RGB565(128, 0, 0)
#define ILI9341_COLOR_MEDIUMAQUAMARINE  RGB565(104, 204, 168)
#define ILI9341_COLOR_MEDIUMBLUE        RGB565(0, 0, 208)
#define ILI9341_COLOR_MEDIUMORCHID      RGB565(184, 84, 208)
#define ILI9341_COLOR_MEDIUMPURPLE      RGB565(144, 112, 216)
#define ILI9341_COLOR_MEDIUMSEAGREEN    RGB565(64, 180, 112)
#define ILI9341_COLOR_MEDIUMSLATEBLUE   RGB565(120, 104, 240)
#define ILI9341_COLOR_MEDIUMSPRINGGREEN RGB565(0, 252, 152)
#define ILI9341_COLOR_MEDIUMTURQUOISE   RGB565(72, 208, 208)
#define ILI9341_COLOR_MEDIUMVIOLETRED   RGB565(200, 20, 136)
#define ILI9341_COLOR_MIDNIGHTBLUE      RGB565(24, 24, 112)
#define ILI9341_COLOR_MINTCREAM         RGB565(248, 252, 248)
#define ILI9341_COLOR_MISTYROSE         RGB565(248, 228, 224)
#define ILI9341_COLOR_MOCCASIN          RGB565(248, 228, 184)
#define ILI9341_COLOR_NAVAJOWHITE       RGB565(248, 224, 176)
#define ILI9341_COLOR_NAVY              RGB565(0, 0, 128)
#define ILI9341_COLOR_OLDLACE           RGB565(248, 244, 232)
#define ILI9341_COLOR_OLIVE             RGB565(128, 128, 0)
#define ILI9341_COLOR_OLIVEDRAB         RGB565(104, 144, 32)
#define ILI9341_COLOR_ORANGE            RGB565(248, 164, 0)
#define ILI9341_COLOR_ORANGERED         RGB565(248, 68, 0)
#define ILI9341_COLOR_ORCHID            RGB565(216, 112, 216)
#define ILI9341_COLOR_PALEGOLDENROD     RGB565(240, 232, 168)
#define ILI9341_COLOR_PALEGREEN         RGB565(152, 252, 152)
#define ILI9341_COLOR_PALETURQUOISE     RGB565(176, 240, 240)
#define ILI9341_COLOR_PALEVIOLETRED     RGB565(216, 112, 144)
#define ILI9341_COLOR_PAPAYAWHIP        RGB565(248, 240, 216)
#define ILI9341_COLOR_PEACHPUFF         RGB565(248, 220, 184)
#define ILI9341_COLOR_PERU              RGB565(208, 132, 64)
#define ILI9341_COLOR_PINK              RGB565(248, 192, 200)
#define ILI9341_COLOR_PLUM              RGB565(224, 160, 224)
#define ILI9341_COLOR_POWDERBLUE        RGB565(176, 224, 232)
#define ILI9341_COLOR_PURPLE            RGB565(128, 0, 128)
#define ILI9341_COLOR_RED               RGB565(248, 0, 0)
#define ILI9341_COLOR_ROSYBROWN         RGB565(192, 144, 144)
#define ILI9341_COLOR_ROYALBLUE         RGB565(64, 104, 224)
#define ILI9341_COLOR_SADDLEBROWN       RGB565(136, 68, 16)
#define ILI9341_COLOR_SALMON            RGB565(248, 128, 112)
#define ILI9341_COLOR_SANDYBROWN        RGB565(248, 164, 96)
#define ILI9341_COLOR_SEAGREEN          RGB565(48, 140, 88)
#define ILI9341_COLOR_SEASHELL          RGB565(248, 244, 240)
#define ILI9341_COLOR_SIENNA            RGB565(160, 84, 48)
#define ILI9341_COLOR_SILVER            RGB565(192, 192, 192)
#define ILI9341_COLOR_SKYBLUE           RGB565(136, 208, 232)
#define ILI9341_COLOR_SLATEBLUE         RGB565(104, 92, 208)
#define ILI9341_COLOR_SLATEGRAY         RGB565(112, 128, 144)
#define ILI9341_COLOR_SLATEGREY         RGB565(112, 128, 144)
#define ILI9341_COLOR_SNOW              RGB565(248, 252, 248)
#define ILI9341_COLOR_SPRINGGREEN       RGB565(0, 252, 128)
#define ILI9341_COLOR_STEELBLUE         RGB565(72, 132, 184)
#define ILI9341_COLOR_TAN               RGB565(208, 180, 144)
#define ILI9341_COLOR_TEAL              RGB565(0, 128, 128)
#define ILI9341_COLOR_THISTLE           RGB565(216, 192, 216)
#define ILI9341_COLOR_TOMATO            RGB565(248, 100, 72)
#define ILI9341_COLOR_TURQUOISE         RGB565(64, 224, 208)
#define ILI9341_COLOR_VIOLET            RGB565(240, 132, 240)
#define ILI9341_COLOR_WHEAT             RGB565(248, 224, 176)
#define ILI9341_COLOR_WHITE             RGB565(248, 252, 248)
#define ILI9341_COLOR_WHITESMOKE        RGB565(248, 244, 248)
#define ILI9341_COLOR_YELLOW            RGB565(248, 252, 0)
#define ILI9341_COLOR_YELLOWGREEN       RGB565(152, 204, 48)

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

/* -- Touch panel XPT2046 (SPI) -- */
/* Bytes de comando: S=1, A2-A0=canal, MODE=12 bits, SER/DFR=diferencial, PD1:0=00 */
#define XPT2046_CMD_X           0xD0U  /**< Canal X+ (posición X)                */
#define XPT2046_CMD_Y           0x90U  /**< Canal Y+ (posición Y)                */
#define XPT2046_CMD_Z1          0xB0U  /**< Canal Z1 (presión, referencia baja)  */
#define XPT2046_CMD_Z2          0xC0U  /**< Canal Z2 (presión, referencia alta)  */

/** @brief Preescalador SPI usado durante las transacciones con el XPT2046.
 *  @details El XPT2046 admite hasta ~2 MHz de reloj SPI; tras la inicialización
 *           el bus del LCD corre a ~45 Mbit/s (@ref SPI_ILI9341_BaudRateUp), por lo
 *           que debe bajarse temporalmente antes de leer el touch. Ajustar este
 *           valor según la frecuencia de reloj APB del proyecto si 2 MHz no cae
 *           dentro del preescalador por defecto. */
#ifndef XPT2046_SPI_PRESCALER
#define XPT2046_SPI_PRESCALER   SPI_BAUDRATEPRESCALER_32
#endif

/** @brief Umbral mínimo de presión (Z1 crudo) para considerar un toque válido
 *         cuando no se dispone de pin PENIRQ (ver ILI9341_TP_ConfigXPT2046). */
#ifndef XPT2046_PRESSURE_THRESHOLD
#define XPT2046_PRESSURE_THRESHOLD  50U
#endif

/* -- Calibración XPT2046: rango crudo del ADC (0-4095) mapeado a la pantalla --
 *    Estos valores varían por panel/cableado; redefinir antes de incluir el
 *    header (o como macro de proyecto) si el touch queda desalineado. */
#ifndef XPT2046_X_MIN
#define XPT2046_X_MIN   200U
#endif
#ifndef XPT2046_X_MAX
#define XPT2046_X_MAX   3900U
#endif
#ifndef XPT2046_Y_MIN
#define XPT2046_Y_MIN   200U
#endif
#ifndef XPT2046_Y_MAX
#define XPT2046_Y_MAX   3900U
#endif

/** @brief Si el cableado intercambia los ejes X/Y crudos respecto a la pantalla. */
#ifndef XPT2046_SWAP_XY
#define XPT2046_SWAP_XY 0
#endif
/** @brief Invierte la dirección del eje X calibrado. */
#ifndef XPT2046_INVERT_X
#define XPT2046_INVERT_X 0
#endif
/** @brief Invierte la dirección del eje Y calibrado. */
#ifndef XPT2046_INVERT_Y
#define XPT2046_INVERT_Y 0
#endif

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
 * @brief Alineación horizontal para ILI9341_PutsAligned()/ILI9341_PrintfAligned()
 *        y sus variantes de frame buffer.
 */
typedef enum {
    ILI9341_ALIGN_LEFT,     /**< Cadena pegada al borde izquierdo de la región   */
    ILI9341_ALIGN_CENTER,   /**< Cadena centrada dentro de la región             */
    ILI9341_ALIGN_RIGHT     /**< Cadena pegada al borde derecho de la región     */
} ILI9341_TextAlign_t;

/**
 * @brief Controlador de panel táctil activo, seleccionado por la función de
 *        configuración de touch que se haya invocado (ILI9341_TP_Config() para
 *        STMPE811, ILI9341_TP_ConfigXPT2046() para XPT2046).
 */
typedef enum {
    ILI9341_TOUCH_NONE = 0,    /**< Ningún panel táctil configurado */
    ILI9341_TOUCH_STMPE811,    /**< Controlador STMPE811 por I2C    */
    ILI9341_TOUCH_XPT2046      /**< Controlador XPT2046 por SPI     */
} ILI9341_TouchDriver_t;

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
 * @brief Inicializa la pantalla LCD ILI9341.
 *
 * @details La firma varía según los módulos HAL habilitados en stm32f4xx_hal_conf.h
 *          (o el equivalente de la familia STM32 usada):
 *          | HAL_I2C_MODULE_ENABLED | HAL_DMA2D_MODULE_ENABLED | Parámetros extra tras dcPin |
 *          |-------------------------|--------------------------|------------------------------|
 *          | No                      | No                        | (ninguno)                    |
 *          | No                      | Sí                        | hdma2d                       |
 *          | Sí                      | No                        | hi2c                         |
 *          | Sí                      | Sí                        | hi2c, hdma2d                 |
 *
 *          Firma base: ILI9341_Init(hspi, csPort, csPin, rstPort, rstPin, dcPort, dcPin, ...)
 *
 *          Los pines SCK/MOSI/MISO del SPI se configuran junto con @p hspi (fuera de esta
 *          librería); CS, RESET y D/C son pines de propósito general que el usuario debe
 *          configurar como salida push-pull y pasar aquí.
 *
 * @param[in] hspi    Puntero al handle SPI de HAL (obligatorio).
 * @param[in] csPort  Puerto GPIO del pin CS (Chip Select) del LCD.
 * @param[in] csPin   Pin GPIO del pin CS del LCD (p. ej. GPIO_PIN_2).
 * @param[in] rstPort Puerto GPIO del pin RESET del LCD.
 * @param[in] rstPin  Pin GPIO del pin RESET del LCD.
 * @param[in] dcPort  Puerto GPIO del pin D/C (WRX, Dato/Comando) del LCD.
 * @param[in] dcPin   Pin GPIO del pin D/C del LCD.
 * @param[in] hi2c    (Solo con HAL_I2C_MODULE_ENABLED) Puntero al handle I2C de HAL,
 *                    usado para el panel táctil STMPE811.
 * @param[in] hdma2d  (Solo con HAL_DMA2D_MODULE_ENABLED) Puntero al handle DMA2D de HAL
 *                    generado por STM32CubeMX. Pasar NULL deshabilita la aceleración DMA2D:
 *                    los rellenos usan el camino CPU y ILI9341_BlitImage no realiza ninguna copia.
 * @return ILI9341_Status_t
 *         - ILI9341_OK            si la inicialización fue exitosa.
 *         - ILI9341_INVALID_PARAM si @p hspi, @p csPort, @p rstPort o @p dcPort son NULL,
 *                                 o @p hi2c es NULL cuando I2C está habilitado.
 *         - ILI9341_ERROR         si una transmisión SPI o la configuración DMA2D falló durante la inicialización.
 */
#if defined(HAL_I2C_MODULE_ENABLED) && defined(HAL_DMA2D_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi,
                               GPIO_TypeDef* csPort, uint16_t csPin,
                               GPIO_TypeDef* rstPort, uint16_t rstPin,
                               GPIO_TypeDef* dcPort, uint16_t dcPin,
                               I2C_HandleTypeDef* hi2c, DMA2D_HandleTypeDef* hdma2d);
#elif defined(HAL_I2C_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi,
                               GPIO_TypeDef* csPort, uint16_t csPin,
                               GPIO_TypeDef* rstPort, uint16_t rstPin,
                               GPIO_TypeDef* dcPort, uint16_t dcPin,
                               I2C_HandleTypeDef* hi2c);
#elif defined(HAL_DMA2D_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi,
                               GPIO_TypeDef* csPort, uint16_t csPin,
                               GPIO_TypeDef* rstPort, uint16_t rstPin,
                               GPIO_TypeDef* dcPort, uint16_t dcPin,
                               DMA2D_HandleTypeDef* hdma2d);
#else
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi,
                               GPIO_TypeDef* csPort, uint16_t csPin,
                               GPIO_TypeDef* rstPort, uint16_t rstPin,
                               GPIO_TypeDef* dcPort, uint16_t dcPin);
#endif

/* --- Utilidades de color --------------------------------------------------- */

/**
 * @brief Convierte una componente de color RGB888 (8 bits por canal) a RGB565.
 *
 * @details Equivale a la macro RGB565(r, g, b), pero como función evita que el
 *          usuario tenga que calcular el empaquetado de bits manualmente y
 *          permite pasar valores calculados en tiempo de ejecución.
 *
 * @param[in] r Componente roja (0-255).
 * @param[in] g Componente verde (0-255).
 * @param[in] b Componente azul (0-255).
 * @return uint16_t Color empaquetado en formato RGB565.
 */
uint16_t ILI9341_Color565(uint8_t r, uint8_t g, uint8_t b);

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
 * @brief Dibuja una línea con grosor (ancho de trazo) en la pantalla LCD.
 *
 * @details Las líneas horizontales y verticales se rellenan con un único rectángulo
 *          (recortado a los límites de pantalla). Las líneas diagonales se aproximan
 *          trazando @p thickness líneas de Bresenham paralelas, desplazadas sobre la
 *          normal del segmento y centradas en la línea original; en ángulos muy
 *          pronunciados puede quedar un ligero aliasing entre trazos adyacentes.
 *
 * @param[in] x0        Coordenada X de inicio.
 * @param[in] y0        Coordenada Y de inicio.
 * @param[in] x1        Coordenada X de fin.
 * @param[in] y1        Coordenada Y de fin.
 * @param[in] thickness Grosor de la línea en píxeles (0 y 1 equivalen a ILI9341_DrawLine()).
 * @param[in] color     Color de la línea en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawThickLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t thickness, uint16_t color);

/**
 * @brief Dibuja una línea vertical de forma optimizada (sin Bresenham).
 *
 * @param[in] x     Coordenada X de la línea.
 * @param[in] y     Coordenada Y inicial.
 * @param[in] h     Alto de la línea (puede ser negativo, se normaliza).
 * @param[in] color Color de la línea.
 * @return ILI9341_Status_t
 */
ILI9341_Status_t ILI9341_DrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);

/**
 * @brief Dibuja una línea horizontal de forma optimizada (sin Bresenham).
 *
 * @param[in] x     Coordenada X inicial.
 * @param[in] y     Coordenada Y de la línea.
 * @param[in] w     Ancho de la línea (puede ser negativo, se normaliza).
 * @param[in] color Color de la línea.
 * @return ILI9341_Status_t
 */
ILI9341_Status_t ILI9341_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

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
 * @brief Dibuja el contorno de un rectángulo con esquinas redondeadas en la pantalla LCD.
 *
 * @details Las esquinas se trazan con arcos de cuarto de círculo de radio @p r usando el
 *          algoritmo de Bresenham. Si @p r supera la mitad del lado más corto se recorta
 *          automáticamente; si se pasa 0 es equivalente a ILI9341_DrawRectangle().
 *
 * @param[in] x0    Coordenada X superior izquierda.
 * @param[in] y0    Coordenada Y superior izquierda.
 * @param[in] x1    Coordenada X inferior derecha.
 * @param[in] y1    Coordenada Y inferior derecha.
 * @param[in] r     Radio de las esquinas en píxeles.
 * @param[in] color Color del contorno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawRoundRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t r, uint16_t color);

/**
 * @brief Dibuja un rectángulo relleno con esquinas redondeadas en la pantalla LCD.
 *
 * @details Combina una franja central (rectángulo completo entre los centros de las
 *          esquinas) con tramos horizontales generados por Bresenham para las zonas
 *          de arco superior e inferior. Si @p r supera la mitad del lado más corto
 *          se recorta automáticamente; si se pasa 0 es equivalente a
 *          ILI9341_DrawFilledRectangle().
 *
 * @param[in] x0    Coordenada X superior izquierda.
 * @param[in] y0    Coordenada Y superior izquierda.
 * @param[in] x1    Coordenada X inferior derecha.
 * @param[in] y1    Coordenada Y inferior derecha.
 * @param[in] r     Radio de las esquinas en píxeles.
 * @param[in] color Color de relleno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledRoundRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t r, uint16_t color);

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

/**
 * @brief Dibuja el contorno de un triángulo en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X del primer vértice.
 * @param[in] y0    Coordenada Y del primer vértice.
 * @param[in] x1    Coordenada X del segundo vértice.
 * @param[in] y1    Coordenada Y del segundo vértice.
 * @param[in] x2    Coordenada X del tercer vértice.
 * @param[in] y2    Coordenada Y del tercer vértice.
 * @param[in] color Color de la línea en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/**
 * @brief Dibuja un triángulo relleno en la pantalla LCD.
 *
 * @details Ordena los vértices por coordenada Y y rellena con tramos horizontales
 *          usando interpolación entera. Los tramos se recortan al borde de pantalla.
 *
 * @param[in] x0    Coordenada X del primer vértice.
 * @param[in] y0    Coordenada Y del primer vértice.
 * @param[in] x1    Coordenada X del segundo vértice.
 * @param[in] y1    Coordenada Y del segundo vértice.
 * @param[in] x2    Coordenada X del tercer vértice.
 * @param[in] y2    Coordenada Y del tercer vértice.
 * @param[in] color Color de relleno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/**
 * @brief Dibuja el contorno de una elipse en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] rx    Radio horizontal en píxeles.
 * @param[in] ry    Radio vertical en píxeles.
 * @param[in] color Color del contorno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p rx o @p ry son negativos.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color);

/**
 * @brief Dibuja una elipse rellena en la pantalla LCD.
 * 
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] rx    Radio horizontal en píxeles.
 * @param[in] ry    Radio vertical en píxeles.
 * @param[in] color Color del contorno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p rx o @p ry son negativos.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color);

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
 * @return ILI9341_Status_t
 */
ILI9341_Status_t ILI9341_DrawArc(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, uint16_t color);

/**
 * @brief Dibuja un arco relleno (sector de anillo) entre dos ángulos.
 *
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] r1    Radio exterior del arco.
 * @param[in] r2    Radio interior del arco.
 * @param[in] start Ángulo inicial en grados (0° = derecha, sentido horario).
 * @param[in] end   Ángulo final en grados.
 * @param[in] color Color del contorno.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p r1 o @p r2 son negativos.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledArc(int16_t x0, int16_t y0, int16_t r1, int16_t r2, float start, float end, uint16_t color);

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
 * @brief Tamaño (en bytes) del buffer interno usado por ILI9341_Printf().
 *
 * @details Define la longitud máxima de la cadena ya formateada, incluido el
 *          terminador nulo. Puede redefinirse antes de incluir este header si
 *          se necesitan cadenas más largas. El buffer reside en la pila.
 */
#ifndef ILI9341_PRINTF_BUF_SIZE
#define ILI9341_PRINTF_BUF_SIZE 128U
#endif

/**
 * @brief Renderiza una cadena con formato (estilo printf) en la pantalla LCD.
 *
 * @details Formatea los argumentos variádicos con vsnprintf() en un buffer
 *          interno de ILI9341_PRINTF_BUF_SIZE bytes y delega el dibujo en
 *          ILI9341_Puts(). Si el resultado excede el tamaño del buffer, la
 *          cadena se trunca de forma segura (sin desbordamiento).
 *
 * @param[in] x          Coordenada X superior izquierda del primer carácter.
 * @param[in] y          Coordenada Y superior izquierda del primer carácter.
 * @param[in] font       Puntero a la definición de la fuente.
 * @param[in] foreground Color de primer plano en formato RGB565.
 * @param[in] background Color de fondo en formato RGB565.
 * @param[in] fmt        Cadena de formato estilo printf (terminada en nulo).
 * @param[in] ...        Argumentos variádicos correspondientes a @p fmt.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p fmt o @p font son NULL, o si vsnprintf falla.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_Printf(uint16_t x, uint16_t y, LCD_FontDef_t* font, uint16_t foreground, uint16_t background, const char* fmt, ...);

/**
 * @brief Calcula el bounding-box en píxeles de una cadena para una fuente dada.
 *
 * @param[in]  str    Puntero a la cadena terminada en nulo.
 * @param[in]  font   Puntero a la definición de la fuente.
 * @param[out] width  Ancho total en píxeles.
 * @param[out] height Alto total en píxeles.
 */
void ILI9341_GetStringSize(char* str, LCD_FontDef_t* font, uint16_t* width, uint16_t* height);

/**
 * @brief Renderiza una cadena terminada en nulo alineada dentro de una región horizontal.
 *
 * @details Calcula el ancho de @p str con ILI9341_GetStringSize() y deriva la
 *          coordenada X según @p align antes de delegar en ILI9341_Puts(). Pensada
 *          para cadenas de una sola línea (títulos, etiquetas, valores); si
 *          @p str es más ancha que la región [x0, x1] se alinea contra @p x0.
 *
 * @param[in] x0         Borde izquierdo de la región de alineación.
 * @param[in] x1         Borde derecho de la región de alineación (x1 >= x0).
 * @param[in] y          Coordenada Y superior izquierda del texto.
 * @param[in] align      Alineación deseada (izquierda, centro, derecha).
 * @param[in] str        Puntero a la cadena terminada en nulo.
 * @param[in] font       Puntero a la definición de la fuente.
 * @param[in] foreground Color de primer plano en formato RGB565.
 * @param[in] background Color de fondo en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p str o @p font son NULL, o si x1 < x0.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_PutsAligned(uint16_t x0, uint16_t x1, uint16_t y, ILI9341_TextAlign_t align, char* str, LCD_FontDef_t* font, uint16_t foreground, uint16_t background);

/**
 * @brief Renderiza una cadena con formato (estilo printf) alineada dentro de una región horizontal.
 *
 * @details Formatea los argumentos variádicos con vsnprintf() en un buffer interno
 *          de ILI9341_PRINTF_BUF_SIZE bytes y delega en ILI9341_PutsAligned().
 *
 * @param[in] x0         Borde izquierdo de la región de alineación.
 * @param[in] x1         Borde derecho de la región de alineación (x1 >= x0).
 * @param[in] y          Coordenada Y superior izquierda del texto.
 * @param[in] align      Alineación deseada (izquierda, centro, derecha).
 * @param[in] font       Puntero a la definición de la fuente.
 * @param[in] foreground Color de primer plano en formato RGB565.
 * @param[in] background Color de fondo en formato RGB565.
 * @param[in] fmt        Cadena de formato estilo printf (terminada en nulo).
 * @param[in] ...        Argumentos variádicos correspondientes a @p fmt.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p fmt o @p font son NULL, si x1 < x0, o si vsnprintf falla.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_PrintfAligned(uint16_t x0, uint16_t x1, uint16_t y, ILI9341_TextAlign_t align, LCD_FontDef_t* font, uint16_t foreground, uint16_t background, const char* fmt, ...);

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

/* --- Frame buffer fuera de pantalla (ImageBuffer) -------------------------- */
/* Variantes de las funciones de dibujo/texto que escriben en un buffer provisto
 * por el usuario (IMG_TOTAL_BUF32 palabras uint32_t, en la memoria que sea: RAM
 * interna, RAM externa, etc.) en vez de enviar directamente a la LCD por SPI.
 * Útiles para componer un frame completo antes de mostrarlo (ver ILI9341_DisplayImage()
 * o el modo de doble buffer más abajo). */

ILI9341_Status_t ILI9341_Fill_ImageBuffer(uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawPixel_ImageBuffer(uint16_t x, uint16_t y, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_Putc_ImageBuffer(uint16_t x, uint16_t y, char c, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_Puts_ImageBuffer(uint16_t x, uint16_t y, char* str, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_Printf_ImageBuffer(uint16_t x, uint16_t y, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32], const char* fmt, ...);
ILI9341_Status_t ILI9341_PutsAligned_ImageBuffer(uint16_t x0, uint16_t x1, uint16_t y, ILI9341_TextAlign_t align, char* str, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_PrintfAligned_ImageBuffer(uint16_t x0, uint16_t x1, uint16_t y, ILI9341_TextAlign_t align, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32], const char* fmt, ...);
ILI9341_Status_t ILI9341_DrawLine_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawThickLine_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t thickness, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawRectangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawRoundRect_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t r, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawFilledRoundRect_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t r, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawFilledRectangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawCircle_ImageBuffer(int16_t x0, int16_t y0, int16_t r, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawFilledCircle_ImageBuffer(int16_t x0, int16_t y0, int16_t r, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawTriangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawFilledTriangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawEllipse_ImageBuffer(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawFilledEllipse_ImageBuffer(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawArc_ImageBuffer(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
ILI9341_Status_t ILI9341_DrawFilledArc_ImageBuffer(int16_t x0, int16_t y0, int16_t r1, int16_t r2, float start, float end, uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);

#ifdef HAL_DMA2D_MODULE_ENABLED
/**
 * @brief Copia una imagen RGB565 al frame buffer usando DMA2D (memoria a memoria).
 *
 * @details Requiere que ILI9341_Init() haya recibido un handle DMA2D válido.
 *
 * @param[in]     src         Puntero a la imagen fuente en formato RGB565.
 * @param[in]     x0          Coordenada X de la esquina superior izquierda en el frame buffer.
 * @param[in]     y0          Coordenada Y de la esquina superior izquierda en el frame buffer.
 * @param[in]     img_w       Ancho de la imagen fuente en píxeles.
 * @param[in]     img_h       Alto de la imagen fuente en píxeles.
 * @param[in,out] framebuffer Frame buffer destino (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_INVALID_PARAM   si @p src o @p framebuffer son NULL, o no se inyectó DMA2D en Init().
 *         - ILI9341_ERROR           si falla el DMA2D.
 */
ILI9341_Status_t ILI9341_BlitImage(const uint16_t* src, uint16_t x0, uint16_t y0,
                                    uint16_t img_w, uint16_t img_h,
                                    uint32_t* framebuffer);
#endif /* HAL_DMA2D_MODULE_ENABLED */

/* --- Doble buffer con pipelining DMA --------------------------------------- */

/**
 * @brief Registra los buffers usados por el modo de doble buffer con pipelining DMA.
 *
 * @details El usuario reserva ambos buffers (RAM interna, RAM externa, etc.), cada uno
 *          de IMG_TOTAL_BUF32 palabras uint32_t; se limpian a cero al registrarse.
 *          Pasar NULL en ambos deshabilita el modo doble buffer.
 *
 * @param[in] front Buffer que se transmite a la LCD (usado por ILI9341_Flush()/ILI9341_Sync()).
 * @param[in] back  Buffer sobre el que dibuja la CPU (ver ILI9341_GetFrameBuffer()).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si solo uno de los dos punteros es NULL.
 */
ILI9341_Status_t ILI9341_SetFrameBuffers(uint32_t* front, uint32_t* back);

/**
 * @brief Presenta el frame dibujado en pantalla usando doble buffer con pipelining DMA.
 *
 * @details Espera el DMA anterior, intercambia front/back e inicia el DMA del frame
 *          recién dibujado sin bloquear. Requiere haber llamado antes a
 *          ILI9341_SetFrameBuffers(). Patrón de uso típico:
 *          @code
 *          ILI9341_SetFrameBuffers(front, back);
 *          for (;;) {
 *              uint32_t* buf = ILI9341_GetFrameBuffer();
 *              // ... dibujar en buf con las funciones _ImageBuffer ...
 *              ILI9341_Flush();
 *          }
 *          @endcode
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si no se registraron buffers con ILI9341_SetFrameBuffers().
 */
ILI9341_Status_t ILI9341_Flush(void);

/**
 * @brief Espera a que concluya el DMA en curso y restaura el bus SPI al modo 8 bits.
 *
 * @details Debe llamarse al salir del modo de doble buffer antes de usar funciones de
 *          dibujo directo en pantalla (ILI9341_Fill, ILI9341_DrawPixel, etc.).
 *          Si no hay DMA activo retorna inmediatamente sin efecto.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si el bus quedó libre correctamente.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_TIMEOUT         si el DMA no terminó en 5 000 ms.
 */
ILI9341_Status_t ILI9341_Sync(void);

/**
 * @brief Retorna el puntero al back buffer activo registrado con ILI9341_SetFrameBuffers().
 *
 * @details Devuelve siempre el back buffer (el buffer sobre el que debe dibujar la CPU).
 *          El puntero cambia tras cada llamada a ILI9341_Flush(), por lo que hay que
 *          invocarlo de nuevo en cada frame.
 *
 * @return Puntero al back buffer activo, o NULL si no hay buffers registrados o el
 *         driver no está inicializado.
 */
uint32_t* ILI9341_GetFrameBuffer(void);

/**
 * @brief Desinicializa el driver LCD y libera los recursos periféricos.
 *
 * @details Marca el driver como no inicializado, pone a NULL los handles internos de
 *          SPI, I2C y DMA2D, y limpia los punteros de frame buffer registrados con
 *          ILI9341_SetFrameBuffers() (la memoria en sí sigue siendo responsabilidad
 *          del usuario). Tras esta llamada es necesario invocar ILI9341_Init() antes
 *          de usar cualquier otra función.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si la desinicialización fue exitosa.
 *         - ILI9341_NOT_INITIALIZED si el driver no estaba inicializado.
 */
ILI9341_Status_t ILI9341_DeInit(void);

/* --- Touch panel (STMPE811) ---------------------------------------------- */

#ifdef HAL_I2C_MODULE_ENABLED
/**
 * @brief Configura el controlador del panel táctil STMPE811 (I2C).
 *
 * @details Tras una configuración exitosa, ILI9341_TP_GetState() consulta este
 *          controlador hasta que se llame a ILI9341_TP_ConfigXPT2046() o a
 *          ILI9341_DeInit().
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si el dispositivo fue detectado y configurado.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si el ID del chip no coincidió con STMPE811_ID.
 */
ILI9341_Status_t ILI9341_TP_Config(void);
#endif /* HAL_I2C_MODULE_ENABLED */

/**
 * @brief Configura el controlador del panel táctil XPT2046 (SPI).
 *
 * @details Reutiliza el mismo periférico SPI pasado a ILI9341_Init() (debe estar
 *          en modo 0, CPOL=0/CPHA=0, igual que el LCD); el XPT2046 se selecciona
 *          con su propio pin CS, distinto del CS del LCD. Durante cada lectura de
 *          touch el preescalador SPI se reduce temporalmente a XPT2046_SPI_PRESCALER
 *          y se restaura al terminar.
 *
 *          Tras una configuración exitosa, ILI9341_TP_GetState() consulta este
 *          controlador hasta que se llame a ILI9341_TP_Config() (STMPE811) o a
 *          ILI9341_DeInit().
 *
 * @param[in] csPort  Puerto GPIO del pin CS (T_CS) del XPT2046.
 * @param[in] csPin   Pin GPIO del pin CS del XPT2046.
 * @param[in] irqPort Puerto GPIO del pin PENIRQ (T_IRQ) del XPT2046, o NULL si no
 *                    está conectado. Sin PENIRQ, la detección de toque usa el
 *                    umbral XPT2046_PRESSURE_THRESHOLD sobre la lectura de presión.
 * @param[in] irqPin  Pin GPIO del pin PENIRQ (ignorado si irqPort es NULL).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si csPort es NULL.
 */
ILI9341_Status_t ILI9341_TP_ConfigXPT2046(GPIO_TypeDef* csPort, uint16_t csPin,
                                           GPIO_TypeDef* irqPort, uint16_t irqPin);

/**
 * @brief Lee el estado actual del panel táctil (coordenadas y detección de toque).
 *
 * @details Despacha internamente al controlador configurado con ILI9341_TP_Config()
 *          o ILI9341_TP_ConfigXPT2046(); si no se configuró ninguno, retorna NULL.
 *
 * @return Puntero a la estructura TP_STATE interna con valores actualizados,
 *         o NULL si el driver no ha sido inicializado o no hay touch configurado.
 */
TP_STATE* ILI9341_TP_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* ILI9341_H */
