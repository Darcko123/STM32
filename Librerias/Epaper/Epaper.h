#ifndef EPAPER_H
#define EPAPER_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include "FONTS.h"
#include <stdint.h>
#include <string.h>

// ============================================================================
// MACROS Y CONSTANTES EPAPER
// ============================================================================

/* -- Dimensiones máximas soportadas -- */
/* Dimensionan el buffer interno; el ancho/alto real se definen en tiempo de
 * ejecución mediante EPAPER_Init(). Pueden redefinirse antes de incluir
 * este archivo para soportar pantallas mayores a 400x300. */
#ifndef EPAPER_MAX_WIDTH
#define EPAPER_MAX_WIDTH     400u  /**< Ancho máximo soportado en píxeles */
#endif
#ifndef EPAPER_MAX_HEIGHT
#define EPAPER_MAX_HEIGHT    300U   /**< Alto máximo soportado en píxeles  */
#endif

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Códigos de estado retornados por todas las funciones públicas del EPAPER.
 */
typedef enum {
    EPAPER_OK              = 0,    /**< Operación exitosa */
    EPAPER_ERROR           = 1,    /**< Error en la operación */
    EPAPER_TIMEOUT         = 2,    /**< Timeout en la operación */
    EPAPER_NOT_INITIALIZED = 3,    /**< Módulo no inicializado */
    EPAPER_INVALID_PARAM   = 4,    /**< Parámetro inválido */
} EPAPER_Status_t;

/**
 * @brief Color de un píxel en el buffer del EPAPER.
 */
typedef enum {
	EPAPER_COLOR_BLACK = 0x00, /**< Píxel apagado (negro) */
	EPAPER_COLOR_WHITE = 0x01  /**< Píxel encendido (blanco) */
} EPAPER_COLOR_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el controlador EPAPER y lo deja listo para dibujar.
 *
 * @param[in] hspi Puntero al handle de SPI.
 * @param[in] width Ancho de la pantalla en píxeles (máximo EPAPER_MAX_WIDTH).
 * @param[in] height Alto de la pantalla en píxeles (máximo EPAPER_MAX_HEIGHT).
 * @return EPAPER_Status_t
 *          - EPAPER_OK            si la inicialización fue exitosa.
 *          - EPAPER_ERROR         si ocurrió un error de comunicación
 *          - EPAPER_INVALID_PARAM si @p hspi es NULL, @p width o @p height son 0,
 *                                  o exceden EPAPER_MAX_WIDTH / EPAPER_MAX_HEIGHT
 */
EPAPER_Status_t EPAPER_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin, uint8_t width, uint8_t height);

#ifdef __cplusplus
}
#endif

#endif /* epaper_H */