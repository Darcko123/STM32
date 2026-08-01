/**
 * @file NV3007.h
 * @brief 
 *
 * @details 
 *
 * @author Daniel Ruiz
 * @date Junio 23, 2026
 * @version 0.1.0
 */

#ifndef NV3007_H
#define NV3007_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// MACROS Y CONSTANTES NV3007
// ============================================================================

/* -- Dimensiones visibles del panel -- */
#define NV3007_WIDTH        142U  /**< Ancho visible del panel en píxeles (columnas) */
#define NV3007_HEIGHT       428U  /**< Alto visible del panel en píxeles (filas)     */

/* -- Origen del área visible dentro de la GRAM del controlador --
 *
 * El NV3007 direcciona una GRAM más ancha que el área visible de este panel
 * (142 columnas), por lo que CASET/RASET necesitan un desplazamiento. El valor
 * depende de cómo el fabricante alineó el panel dentro de la GRAM y cambia con
 * la orientación, porque los bits MX/MY/MV de MADCTL remapean el direccionamiento.
 *
 * Los valores por defecto son una HIPÓTESIS (panel centrado horizontalmente en una
 * GRAM de 172 columnas, alineado arriba). Calibrar con NV3007_SetOffset() y el
 * procedimiento descrito en PLAN-FIX-NV3007-142x428.md §10 antes de darlos por buenos. */
#define NV3007_OFFSET_P1_X  15U   /**< Portrait_1  (0°)   - offset X */
#define NV3007_OFFSET_P1_Y   0U   /**< Portrait_1  (0°)   - offset Y */
#define NV3007_OFFSET_P2_X  15U   /**< Portrait_2  (180°) - offset X */
#define NV3007_OFFSET_P2_Y   0U   /**< Portrait_2  (180°) - offset Y */
#define NV3007_OFFSET_L1_X   0U   /**< Landscape_1 (90°)  - offset X */
#define NV3007_OFFSET_L1_Y  15U   /**< Landscape_1 (90°)  - offset Y */
#define NV3007_OFFSET_L2_X   0U   /**< Landscape_2 (270°) - offset X */
#define NV3007_OFFSET_L2_Y  15U   /**< Landscape_2 (270°) - offset Y */

/* -- Retardos de la secuencia de inicialización (ms) -- */
#define NV3007_RST_DELAY    120U  /**< Espera tras el reset por hardware */
#define NV3007_SLPIN_DELAY  120U  /**< Espera tras entrar en modo Sleep  */
#define NV3007_SLPOUT_DELAY 120U  /**< Espera tras salir del modo Sleep  */

/* -- Comandos NV3007 -- */
#define NV3007_CMD_SLPIN     0x10U  /**< Entra en modo Sleep                           */
#define NV3007_CMD_SLPOUT    0x11U  /**< Sale del modo Sleep                           */
#define NV3007_CMD_INVOFF    0x20U  /**< Desactiva la inversión de color               */
#define NV3007_CMD_INVON     0x21U  /**< Activa la inversión de color                  */
#define NV3007_CMD_DISPOFF   0x28U  /**< Apaga la pantalla                             */
#define NV3007_CMD_DISPON    0x29U  /**< Enciende la pantalla                          */
#define NV3007_CMD_CASET     0x2AU  /**< Establece la ventana de columna (eje X)       */
#define NV3007_CMD_RASET     0x2BU  /**< Establece la ventana de fila (eje Y)          */
#define NV3007_CMD_RAMWR     0x2CU  /**< Escritura en RAM (inicio de transferencia)    */
#define NV3007_CMD_RAMRD     0x2EU  /**< Lectura de RAM                                */
#define NV3007_CMD_MADCTL    0x36U  /**< Control de acceso a memoria (rotación/espejo) */

/* -- Bits del registro MADCTL -- */
#define NV3007_MADCTL_MY     0x80U  /**< Espejo vertical (Mirror Y)    */
#define NV3007_MADCTL_MX     0x40U  /**< Espejo horizontal (Mirror X)  */
#define NV3007_MADCTL_MV     0x20U  /**< Intercambio de ejes (Swap XY) */
#define NV3007_MADCTL_ML     0x10U  /**< Orden de escaneo vertical     */
#define NV3007_MADCTL_RGB    0x00U  /**< Orden de color RGB            */

// ============================================================================
// COLORES
// ============================================================================
/* -- Colores predefinidos (RGB565) -- */
#define RGB565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
#define RGB16TO24(c) ((((uint32_t)(c) & 0xF800) << 8) | (((c) & 0x07E0) << 5) | (((c) & 0x1F) << 3))

#define NV3007_COLOR_ALICEBLUE         RGB565(240, 248, 248)
#define NV3007_COLOR_ANTIQUEWHITE      RGB565(248, 236, 216)
#define NV3007_COLOR_AQUA              RGB565(0, 252, 248)
#define NV3007_COLOR_AQUAMARINE        RGB565(128, 252, 216)
#define NV3007_COLOR_AZURE             RGB565(240, 252, 248)
#define NV3007_COLOR_BEIGE             RGB565(248, 244, 224)
#define NV3007_COLOR_BISQUE            RGB565(248, 228, 200)
#define NV3007_COLOR_BLACK             RGB565(0, 0, 0)
#define NV3007_COLOR_BLANCHEDALMOND    RGB565(248, 236, 208)
#define NV3007_COLOR_BLUE              RGB565(0, 0, 248)
#define NV3007_COLOR_BLUEVIOLET        RGB565(136, 44, 224)
#define NV3007_COLOR_BROWN             RGB565(168, 44, 40)
#define NV3007_COLOR_BURLYWOOD         RGB565(224, 184, 136)
#define NV3007_COLOR_CADETBLUE         RGB565(96, 160, 160)
#define NV3007_COLOR_CHARTREUSE        RGB565(128, 252, 0)
#define NV3007_COLOR_CHOCOLATE         RGB565(208, 104, 32)
#define NV3007_COLOR_CORAL             RGB565(248, 128, 80)
#define NV3007_COLOR_CORNFLOWERBLUE    RGB565(104, 148, 240)
#define NV3007_COLOR_CORNSILK          RGB565(248, 248, 224)
#define NV3007_COLOR_CRIMSON           RGB565(224, 20, 64)
#define NV3007_COLOR_CYAN              RGB565(0, 252, 248)
#define NV3007_COLOR_DARKBLUE          RGB565(0, 0, 136)
#define NV3007_COLOR_DARKCYAN          RGB565(0, 140, 136)
#define NV3007_COLOR_DARKGOLDENROD     RGB565(184, 136, 8)
#define NV3007_COLOR_DARKGRAY          RGB565(168, 168, 168)
#define NV3007_COLOR_DARKGREEN         RGB565(0, 100, 0)
#define NV3007_COLOR_DARKGREY          RGB565(168, 168, 168)
#define NV3007_COLOR_DARKKHAKI         RGB565(192, 184, 104)
#define NV3007_COLOR_DARKMAGENTA       RGB565(136, 0, 136)
#define NV3007_COLOR_DARKOLIVEGREEN    RGB565(88, 108, 48)
#define NV3007_COLOR_DARKORANGE        RGB565(248, 140, 0)
#define NV3007_COLOR_DARKORCHID        RGB565(152, 52, 208)
#define NV3007_COLOR_DARKRED           RGB565(136, 0, 0)
#define NV3007_COLOR_DARKSALMON        RGB565(232, 152, 120)
#define NV3007_COLOR_DARKSEAGREEN      RGB565(144, 188, 144)
#define NV3007_COLOR_DARKSLATEBLUE     RGB565(72, 60, 136)
#define NV3007_COLOR_DARKSLATEGRAY     RGB565(48, 80, 80)
#define NV3007_COLOR_DARKSLATEGREY     RGB565(48, 80, 80)
#define NV3007_COLOR_DARKTURQUOISE     RGB565(0, 208, 208)
#define NV3007_COLOR_DARKVIOLET        RGB565(152, 0, 208)
#define NV3007_COLOR_DEEPPINK          RGB565(248, 20, 144)
#define NV3007_COLOR_DEEPSKYBLUE       RGB565(0, 192, 248)
#define NV3007_COLOR_DIMGRAY           RGB565(104, 104, 104)
#define NV3007_COLOR_DIMGREY           RGB565(104, 104, 104)
#define NV3007_COLOR_DODGERBLUE        RGB565(32, 144, 248)
#define NV3007_COLOR_FIREBRICK         RGB565(176, 36, 32)
#define NV3007_COLOR_FLORALWHITE       RGB565(248, 252, 240)
#define NV3007_COLOR_FORESTGREEN       RGB565(32, 140, 32)
#define NV3007_COLOR_FUCHSIA           RGB565(248, 0, 248)
#define NV3007_COLOR_GAINSBORO         RGB565(224, 220, 224)
#define NV3007_COLOR_GHOSTWHITE        RGB565(248, 248, 248)
#define NV3007_COLOR_GOLD              RGB565(248, 216, 0)
#define NV3007_COLOR_GOLDENROD         RGB565(216, 164, 32)
#define NV3007_COLOR_GRAY              RGB565(128, 128, 128)
#define NV3007_COLOR_GREEN             RGB565(0, 128, 0)
#define NV3007_COLOR_GREENYELLOW       RGB565(176, 252, 48)
#define NV3007_COLOR_GREY              RGB565(128, 128, 128)
#define NV3007_COLOR_HONEYDEW          RGB565(240, 252, 240)
#define NV3007_COLOR_HOTPINK           RGB565(248, 104, 184)
#define NV3007_COLOR_INDIANRED         RGB565(208, 92, 96)
#define NV3007_COLOR_INDIGO            RGB565(72, 0, 128)
#define NV3007_COLOR_IVORY             RGB565(248, 252, 240)
#define NV3007_COLOR_KHAKI             RGB565(240, 232, 144)
#define NV3007_COLOR_LAVENDER          RGB565(232, 232, 248)
#define NV3007_COLOR_LAVENDERBLUSH     RGB565(248, 240, 248)
#define NV3007_COLOR_LAWNGREEN         RGB565(128, 252, 0)
#define NV3007_COLOR_LEMONCHIFFON      RGB565(248, 252, 208)
#define NV3007_COLOR_LIGHTBLUE         RGB565(176, 216, 232)
#define NV3007_COLOR_LIGHTCORAL        RGB565(240, 128, 128)
#define NV3007_COLOR_LIGHTCYAN         RGB565(224, 252, 248)
#define NV3007_COLOR_LIGHTGOLDENRODYELLOW RGB565(248, 252, 208)
#define NV3007_COLOR_LIGHTGRAY         RGB565(208, 212, 208)
#define NV3007_COLOR_LIGHTGREEN        RGB565(144, 240, 144)
#define NV3007_COLOR_LIGHTGREY         RGB565(208, 212, 208)
#define NV3007_COLOR_LIGHTPINK         RGB565(248, 184, 192)
#define NV3007_COLOR_LIGHTSALMON       RGB565(248, 160, 120)
#define NV3007_COLOR_LIGHTSEAGREEN     RGB565(32, 180, 168)
#define NV3007_COLOR_LIGHTSKYBLUE      RGB565(136, 208, 248)
#define NV3007_COLOR_LIGHTSLATEGRAY    RGB565(120, 136, 152)
#define NV3007_COLOR_LIGHTSLATEGREY    RGB565(120, 136, 152)
#define NV3007_COLOR_LIGHTSTEELBLUE    RGB565(176, 196, 224)
#define NV3007_COLOR_LIGHTYELLOW       RGB565(248, 252, 224)
#define NV3007_COLOR_LIME              RGB565(0, 252, 0)
#define NV3007_COLOR_LIMEGREEN         RGB565(48, 204, 48)
#define NV3007_COLOR_LINEN             RGB565(248, 240, 232)
#define NV3007_COLOR_MAGENTA           RGB565(248, 0, 248)
#define NV3007_COLOR_MAROON            RGB565(128, 0, 0)
#define NV3007_COLOR_MEDIUMAQUAMARINE  RGB565(104, 204, 168)
#define NV3007_COLOR_MEDIUMBLUE        RGB565(0, 0, 208)
#define NV3007_COLOR_MEDIUMORCHID      RGB565(184, 84, 208)
#define NV3007_COLOR_MEDIUMPURPLE      RGB565(144, 112, 216)
#define NV3007_COLOR_MEDIUMSEAGREEN    RGB565(64, 180, 112)
#define NV3007_COLOR_MEDIUMSLATEBLUE   RGB565(120, 104, 240)
#define NV3007_COLOR_MEDIUMSPRINGGREEN RGB565(0, 252, 152)
#define NV3007_COLOR_MEDIUMTURQUOISE   RGB565(72, 208, 208)
#define NV3007_COLOR_MEDIUMVIOLETRED   RGB565(200, 20, 136)
#define NV3007_COLOR_MIDNIGHTBLUE      RGB565(24, 24, 112)
#define NV3007_COLOR_MINTCREAM         RGB565(248, 252, 248)
#define NV3007_COLOR_MISTYROSE         RGB565(248, 228, 224)
#define NV3007_COLOR_MOCCASIN          RGB565(248, 228, 184)
#define NV3007_COLOR_NAVAJOWHITE       RGB565(248, 224, 176)
#define NV3007_COLOR_NAVY              RGB565(0, 0, 128)
#define NV3007_COLOR_OLDLACE           RGB565(248, 244, 232)
#define NV3007_COLOR_OLIVE             RGB565(128, 128, 0)
#define NV3007_COLOR_OLIVEDRAB         RGB565(104, 144, 32)
#define NV3007_COLOR_ORANGE            RGB565(248, 164, 0)
#define NV3007_COLOR_ORANGERED         RGB565(248, 68, 0)
#define NV3007_COLOR_ORCHID            RGB565(216, 112, 216)
#define NV3007_COLOR_PALEGOLDENROD     RGB565(240, 232, 168)
#define NV3007_COLOR_PALEGREEN         RGB565(152, 252, 152)
#define NV3007_COLOR_PALETURQUOISE     RGB565(176, 240, 240)
#define NV3007_COLOR_PALEVIOLETRED     RGB565(216, 112, 144)
#define NV3007_COLOR_PAPAYAWHIP        RGB565(248, 240, 216)
#define NV3007_COLOR_PEACHPUFF         RGB565(248, 220, 184)
#define NV3007_COLOR_PERU              RGB565(208, 132, 64)
#define NV3007_COLOR_PINK              RGB565(248, 192, 200)
#define NV3007_COLOR_PLUM              RGB565(224, 160, 224)
#define NV3007_COLOR_POWDERBLUE        RGB565(176, 224, 232)
#define NV3007_COLOR_PURPLE            RGB565(128, 0, 128)
#define NV3007_COLOR_RED               RGB565(248, 0, 0)
#define NV3007_COLOR_ROSYBROWN         RGB565(192, 144, 144)
#define NV3007_COLOR_ROYALBLUE         RGB565(64, 104, 224)
#define NV3007_COLOR_SADDLEBROWN       RGB565(136, 68, 16)
#define NV3007_COLOR_SALMON            RGB565(248, 128, 112)
#define NV3007_COLOR_SANDYBROWN        RGB565(248, 164, 96)
#define NV3007_COLOR_SEAGREEN          RGB565(48, 140, 88)
#define NV3007_COLOR_SEASHELL          RGB565(248, 244, 240)
#define NV3007_COLOR_SIENNA            RGB565(160, 84, 48)
#define NV3007_COLOR_SILVER            RGB565(192, 192, 192)
#define NV3007_COLOR_SKYBLUE           RGB565(136, 208, 232)
#define NV3007_COLOR_SLATEBLUE         RGB565(104, 92, 208)
#define NV3007_COLOR_SLATEGRAY         RGB565(112, 128, 144)
#define NV3007_COLOR_SLATEGREY         RGB565(112, 128, 144)
#define NV3007_COLOR_SNOW              RGB565(248, 252, 248)
#define NV3007_COLOR_SPRINGGREEN       RGB565(0, 252, 128)
#define NV3007_COLOR_STEELBLUE         RGB565(72, 132, 184)
#define NV3007_COLOR_TAN               RGB565(208, 180, 144)
#define NV3007_COLOR_TEAL              RGB565(0, 128, 128)
#define NV3007_COLOR_THISTLE           RGB565(216, 192, 216)
#define NV3007_COLOR_TOMATO            RGB565(248, 100, 72)
#define NV3007_COLOR_TURQUOISE         RGB565(64, 224, 208)
#define NV3007_COLOR_VIOLET            RGB565(240, 132, 240)
#define NV3007_COLOR_WHEAT             RGB565(248, 224, 176)
#define NV3007_COLOR_WHITE             RGB565(248, 252, 248)
#define NV3007_COLOR_WHITESMOKE        RGB565(248, 244, 248)
#define NV3007_COLOR_YELLOW            RGB565(248, 252, 0)
#define NV3007_COLOR_YELLOWGREEN       RGB565(152, 204, 48)

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del NV3007.
 */
typedef enum {
    NV3007_OK               = 0,    /**< Operación exitosa */
    NV3007_ERROR            = 1,    /**< Error en la operación */
    NV3007_TIMEOUT          = 2,    /**< Timeout en la operación */
    NV3007_NOT_INITIALIZED  = 3,    /**< Módulo no inicializado */
    NV3007_INVALID_PARAM    = 4     /**< Parámetro inválido */
} NV3007_Status_t;

/**
 * @brief Opciones de orientación de pantalla para NV3007_Rotate().
 */
typedef enum {
    NV3007_Orientation_Portrait_1,    /**< Sin rotación          */
    NV3007_Orientation_Portrait_2,    /**< Rotación 180°         */
    NV3007_Orientation_Landscape_1,   /**< Rotación 90°          */
    NV3007_Orientation_Landscape_2    /**< Rotación 270° (-90°)  */
} NV3007_Orientation_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el módulo NV3007
 *
 * @param hspi Puntero al manejador de la interfaz SPI utilizada para comunicarse con el módulo.
 * @param CS_GPIOx Puerto GPIO del pin CS (Chip Select) del NV3007.
 * @param CS_Pin Pin GPIO del pin CS del NV3007.
 * @param DC_GPIOx Puerto GPIO del pin DC (Data/Command) del NV3007.
 * @param DC_Pin Pin GPIO del pin DC del NV3007.
 * @param RST_GPIOx Puerto GPIO del pin RST (Reset) del NV3007.
 * @param RST_Pin Pin GPIO del pin RST del NV3007.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_Init(SPI_HandleTypeDef* hspi,
                             GPIO_TypeDef* CS_GPIOx, uint16_t CS_Pin,
                             GPIO_TypeDef* DC_GPIOx, uint16_t DC_Pin,
                             GPIO_TypeDef* RST_GPIOx, uint16_t RST_Pin);

/**
 * @brief Desinicializa el módulo NV3007, apagando la pantalla y liberando la configuración almacenada.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DeInit(void);

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
NV3007_Status_t NV3007_WriteAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

/**
 * @brief Rota la pantalla y actualiza el ancho/alto internos.
 * 
 * @note La geometría interna solo se actualiza si el comando SPI es exitoso.
 *
 * @param[in] orientation Orientación deseada (NV3007_Orientation_t, en pasos de 90°).
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_Rotate(NV3007_Orientation_t orientation);

/**
 * @brief Sobrescribe en tiempo de ejecución el offset de GRAM aplicado a CASET/RASET.
 *
 * @details Pensado para calibrar el panel sin recompilar: se ajustan los valores hasta
 *          que la imagen encaja con el borde físico y luego se trasladan a las constantes
 *          NV3007_OFFSET_*.
 *
 * @param x_offset Desplazamiento de columna aplicado a CASET.
 * @param y_offset Desplazamiento de fila aplicado a RASET.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_SetOffset(uint16_t x_offset, uint16_t y_offset);

/**
 * @brief Activa o desactiva la inversión de color de la pantalla.
 *
 * @param invert true para invertir los colores, false para restaurarlos.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_InvertDisplay(bool invert);

/**
 * @brief Enciende la pantalla (sale del modo DISPOFF).
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DisplayOn(void);

/**
 * @brief Apaga la pantalla (entra en modo DISPOFF).
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DisplayOff(void);

// ============================================================================
// Funciones de dibujo
// ============================================================================
NV3007_Status_t NV3007_WritePixel(int16_t x, int16_t y, uint16_t color);
NV3007_Status_t NV3007_DrawPixel(int16_t x, int16_t y, uint16_t color);
NV3007_Status_t NV3007_FillScreen(uint16_t color);
NV3007_Status_t NV3007_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
NV3007_Status_t NV3007_DrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
NV3007_Status_t NV3007_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
NV3007_Status_t NV3007_DrawRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
NV3007_Status_t NV3007_WriteFilledRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
NV3007_Status_t NV3007_DrawFilledRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
NV3007_Status_t NV3007_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
NV3007_Status_t NV3007_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
NV3007_Status_t NV3007_DrawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
NV3007_Status_t NV3007_DrawFilledTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
NV3007_Status_t NV3007_DrawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
NV3007_Status_t NV3007_DrawFilledRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
NV3007_Status_t NV3007_DrawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
#ifdef __cplusplus
}
#endif

#endif /* NV3007_H */