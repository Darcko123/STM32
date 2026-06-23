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

/* -- Dimensiones de la pantalla -- */
#define NV3007_WIDTH        168U  /**< Ancho de la pantalla en píxeles */
#define NV3007_HEIGHT       428U  /**< Alto de la pantalla en píxeles  */

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
NV3007_Status_t NV3007_DrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
NV3007_Status_t NV3007_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
NV3007_Status_t NV3007_WriteFilledRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
NV3007_Status_t NV3007_DrawFilledRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

#ifdef __cplusplus
}
#endif

#endif /* NV3007_H */