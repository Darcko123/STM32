/**
 * @file MAX7219.h
 * @brief Librería para la gestión de una matriz de leds basada en el integrado MAX7219 usando comunicación SPI.
 * 
 * Esta librería permite inicializar y controlar una matriz de LEDs utilizando el controlador MAX7219.
 * 
 * @author Daniel Ruiz
 * @date Jan 22, 2026
 * @version 2.0
 */

#ifndef MAX7219_H_
#define MAX7219_H_

/**
 * @brief Incluir el encabezado adecuado según la familia STM32 utilizada.
 * Por ejemplo:
 * - Para STM32F1xx: "stm32f1xx_hal.h"
 * - Para STM32F4xx: "stm32f4xx_hal.h"
 */
#include "stm32f4xx_hal.h"
#include <stdint.h>

// ============================================================================
// DEFINICIONES DE CONSTANTES
// ============================================================================

#define NUM_DEV 4       /**< Número de dispositivos MAX7219 en cascada */
#define FONT_WIDTH 8    /**< Ancho de cada carácter en el conjunto de fuentes */

extern uint8_t bufferCol[NUM_DEV*8];	/**< Número de columnas respecto al número de dispositivos conectados en cascada */

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================
/**
 * @brief Enumeración para estados de retorno del MAX7219.
 */
typedef enum {
	MAX7219_OK = 0,				/** Operación exitosa */
	MAX7219_ERROR = 1,			/** Error en la operación */
	MAX7219_TIMEOUT = 2,		/** Timeout en la operación */
	MAX7219_NOT_INITIALIZED = 3	/** Sensor no inicializado */
}MAX7219_Status_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa la matriz de LEDs.
 * 
 * @param hspi Puntero al manejador de la interfaz SPI utilizada para comunicarse con el módulo.
 * 
 * @param GPIOx Puerto GPIO del pin de datos del DHT11.
 * @param GPIO_PIN Pin GPIO del pin de datos del DHT11.
 * 
 * 
 */
MAX7219_Status_t MAX7219_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN);

/**
 * @brief Apaga todos los LEDs de la matriz.
 */
MAX7219_Status_t MAX7219_ClearDisplay(void);

/**
 * @brief Muestra una cadena de caracteres en la matriz de LEDs con desplazamiento.
 * 
 * @param str Cadena de caracteres a mostrar.
 * @param delay Tiempo de retardo entre cada desplazamiento.
 */
MAX7219_Status_t MAX7219_ScrollString (char *str, int delay);

/**
 * @brief Muestra una cadena de caracteres en la matriz de LEDs.
 * 
 * Esta función toma una cadena de texto y la convierte en una representación de bytes
 * para visualizarla en la matriz de LEDs. Cada carácter se almacena en el buffer de
 * columnas y se muestra en la pantalla.
 * 
 * @param str Puntero a la cadena de caracteres que se mostrará.
 */
MAX7219_Status_t MAX7219_PrintString(const char *str);

/**
 * @brief Conjunto de fuentes para la pantalla de matriz de puntos MAX7219.
 *
 * Este array contiene el conjunto de fuentes de 8x8 para la pantalla de matriz de puntos MAX7219.
 * Cada carácter está representado por un array de 8 bytes, donde cada byte representa
 * una fila del carácter.
 */
extern const uint8_t MAX7219_Dot_Matrix_font[256][8];

#ifdef __cplusplus
}
#endif

#endif /* MAX7219_H_ */
