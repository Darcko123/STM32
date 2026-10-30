#ifndef SSD1681_H
#define SSD1681_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include "FONTS.h"
#include <stdint.h>
#include <string.h>

// ============================================================================
// MACROS Y CONSTANTES SSD1681
// ============================================================================

/* -- Dimensiones máximas soportadas -- */
/* Dimensionan el buffer interno; el ancho/alto real se definen en tiempo de
 * ejecución mediante SSD1681_Init(). Pueden redefinirse antes de incluir
 * este archivo para soportar pantallas mayores a 400x300. */
#ifndef SSD1681_MAX_WIDTH
#define SSD1681_MAX_WIDTH     400U  /**< Ancho máximo soportado en píxeles */
#endif
#ifndef SSD1681_MAX_HEIGHT
#define SSD1681_MAX_HEIGHT    300U   /**< Alto máximo soportado en píxeles  */
#endif

/* -- Tiempos (pueden redefinirse antes de incluir este archivo) -- */
#ifndef SSD1681_SPI_TIMEOUT_MS
#define SSD1681_SPI_TIMEOUT_MS   100U  /**< Timeout de cada transmisión SPI          */
#endif
#ifndef SSD1681_BUSY_TIMEOUT_MS
#define SSD1681_BUSY_TIMEOUT_MS  5000U /**< Espera máxima a que se libere BUSY       */
#endif
#ifndef SSD1681_RESET_PULSE_MS
#define SSD1681_RESET_PULSE_MS   10U   /**< Duración del pulso y la espera de reset  */
#endif

/** @brief Nivel del pin BUSY mientras el controlador está ocupado (SSD168x: alto). */
#ifndef SSD1681_BUSY_ACTIVE_LEVEL
#define SSD1681_BUSY_ACTIVE_LEVEL GPIO_PIN_SET
#endif

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Códigos de estado retornados por todas las funciones públicas del SSD1681.
 */
typedef enum {
    SSD1681_OK              = 0,    /**< Operación exitosa */
    SSD1681_ERROR           = 1,    /**< Error en la operación */
    SSD1681_TIMEOUT         = 2,    /**< Timeout en la operación */
    SSD1681_NOT_INITIALIZED = 3,    /**< Módulo no inicializado */
    SSD1681_INVALID_PARAM   = 4,    /**< Parámetro inválido */
} SSD1681_Status_t;

/**
 * @brief Color de un píxel en el buffer del SSD1681.
 */
typedef enum {
	SSD1681_COLOR_BLACK = 0x00, /**< Píxel apagado (negro) */
	SSD1681_COLOR_WHITE = 0x01  /**< Píxel encendido (blanco) */
} SSD1681_COLOR_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el controlador SSD1681 y lo deja listo para dibujar.
 *
 * @details Secuencia: reset por hardware, SW reset, número de compuertas,
 *          modo de avance de RAM, ventana de RAM del panel completo, borde y
 *          sensor de temperatura interno. El SPI (modo 0, MSB primero) y los
 *          pines CS, D/C y RST (salida) y BUSY (entrada) deben configurarse
 *          antes, p. ej. en STM32CubeMX.
 *
 * @param[in] hspi     Puntero al handle de SPI.
 * @param[in] csPort   Puerto GPIO del pin CS.
 * @param[in] csPin    Pin GPIO del pin CS.
 * @param[in] dcPort   Puerto GPIO del pin D/C.
 * @param[in] dcPin    Pin GPIO del pin D/C.
 * @param[in] rstPort  Puerto GPIO del pin RST.
 * @param[in] rstPin   Pin GPIO del pin RST.
 * @param[in] busyPort Puerto GPIO del pin BUSY.
 * @param[in] busyPin  Pin GPIO del pin BUSY.
 * @param[in] width    Ancho de la pantalla en píxeles (máximo SSD1681_MAX_WIDTH).
 * @param[in] height   Alto de la pantalla en píxeles (máximo SSD1681_MAX_HEIGHT).
 * @return SSD1681_Status_t
 *          - SSD1681_OK            si la inicialización fue exitosa.
 *          - SSD1681_ERROR         si ocurrió un error de comunicación.
 *          - SSD1681_TIMEOUT       si BUSY no se liberó en SSD1681_BUSY_TIMEOUT_MS
 *                                  (¿panel desconectado?).
 *          - SSD1681_INVALID_PARAM si algún handle/puerto es NULL, @p width o
 *                                  @p height son 0, o exceden SSD1681_MAX_WIDTH /
 *                                  SSD1681_MAX_HEIGHT.
 */
SSD1681_Status_t SSD1681_Init(SPI_HandleTypeDef* hspi,
                            GPIO_TypeDef* csPort, uint16_t csPin,
                            GPIO_TypeDef* dcPort, uint16_t dcPin,
                            GPIO_TypeDef* rstPort, uint16_t rstPin,
                            GPIO_TypeDef* busyPort, uint16_t busyPin,
                            uint16_t width, uint16_t height);

/**
 * @brief Obtiene el ancho de pantalla configurado en SSD1681_Init.
 *
 * @return uint16_t Ancho en píxeles, o 0 si el módulo no ha sido inicializado.
 */
uint16_t SSD1681_GetWidth(void);

/**
 * @brief Obtiene el alto de pantalla configurado en SSD1681_Init.
 *
 * @return uint16_t Alto en píxeles, o 0 si el módulo no ha sido inicializado.
 */
uint16_t SSD1681_GetHeight(void);

/**
 * @brief Envía el contenido del buffer interno a la pantalla física.
 * 
 * @return SSD1681_Status_t 
 */
SSD1681_Status_t SSD1681_UpdateScreen(void);

#ifdef __cplusplus
}
#endif

#endif /* SSD1681_H */