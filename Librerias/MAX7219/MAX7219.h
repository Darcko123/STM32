/**
 * @file MAX7219.c
 *
 * @brief Implementación de la librería para el control de una matriz de LEDs usando MAX7219 en STM32 mediante SPI.
 *
 * @author Daniel Ruiz
 * @date Junio 18, 2026
 * @version 2.1.0
 */


#ifndef MAX7219_H_
#define MAX7219_H_

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdint.h>

// ============================================================================
// DEFINICIONES DE CONSTANTES
// ============================================================================

#define MAX7219_MAX_DEV 4   /**< Número máximo de dispositivos MAX7219 en cascada soportados (define el tamaño del buffer estático) */
#define FONT_WIDTH 8        /**< Ancho de cada carácter en el conjunto de fuentes */

extern uint8_t bufferCol[MAX7219_MAX_DEV*8];	/**< Número de columnas respecto al número de dispositivos conectados en cascada */

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================
/**
 * @brief Enumeración para estados de retorno del MAX7219.
 */
typedef enum {
	MAX7219_OK = 0,					/** Operación exitosa */
	MAX7219_ERROR = 1,				/** Error en la operación */
	MAX7219_TIMEOUT = 2,			/** Timeout en la operación */
	MAX7219_NOT_INITIALIZED = 3,	/** Sensor no inicializado */
	MAX7219_INVALID_PARAM = 4		/** Parámetro inválido */
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
 * @param GPIOx Puerto GPIO del pin CS del MAX7219.
 * @param GPIO_PIN Pin GPIO del pin CS del MAX7219.
 * @param numDevices Número de dispositivos MAX7219 conectados en cascada (1 a MAX7219_MAX_DEV).
 *
 * @return MAX7219_Status_t Estado de la operación.
 */
MAX7219_Status_t MAX7219_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN, uint8_t numDevices);

/**
 * @brief Desinicializa el módulo MAX7219, apagando la pantalla y liberando la configuración almacenada.
 *
 * @return MAX7219_Status_t Estado de la operación.
 */
MAX7219_Status_t MAX7219_DeInit(void);

/**
 * @brief Activa o desactiva el modo de bajo consumo (shutdown) del MAX7219 sin perder la configuración almacenada.
 *
 * A diferencia de MAX7219_DeInit, esta función no libera el manejador SPI ni el pin CS, por lo que
 * el módulo puede reactivarse posteriormente sin necesidad de volver a llamar a MAX7219_Init.
 *
 * @param enable Distinto de 0 para entrar en modo de bajo consumo, 0 para volver al modo de operación normal.
 *
 * @return MAX7219_Status_t Estado de la operación.
 */
MAX7219_Status_t MAX7219_Set_Sleep(uint8_t enable);

/**
 * @brief Apaga todos los LEDs de la matriz.
 *
 * @return MAX7219_Status_t Estado de la operación.
 */
MAX7219_Status_t MAX7219_ClearDisplay(void);

/**
 * @brief Muestra una cadena de caracteres en la matriz de LEDs con desplazamiento.
 *
 * @param str Cadena de caracteres a mostrar.
 * @param delay Tiempo de retardo entre cada desplazamiento.
 *
 * @return MAX7219_Status_t Estado de la operación.
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
 *
 * @return MAX7219_Status_t Estado de la operación.
 */
MAX7219_Status_t MAX7219_PrintString(const char *str);

/**
 * @brief Enciende o apaga un píxel individual de la matriz de LEDs.
 *
 * Manipula directamente bufferCol para permitir dibujar gráficos/iconos arbitrarios,
 * en lugar de limitarse a texto. La actualización de la pantalla es inmediata.
 *
 * @param x Columna del píxel (0 a número de dispositivos configurado en MAX7219_Init multiplicado por 8, menos 1).
 * @param y Fila del píxel dentro de la columna (0 a 7), donde el bit y de bufferCol[x] representa el píxel.
 * @param state Distinto de 0 para encender el píxel, 0 para apagarlo.
 *
 * @return MAX7219_Status_t Estado de la operación.
 */
MAX7219_Status_t MAX7219_SetPixel(int x, int y, uint8_t state);

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
