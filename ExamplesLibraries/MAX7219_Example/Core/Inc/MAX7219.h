/**
 * @file MAX7219.h
 * @brief Librería para la gestión de una matriz de leds basada en el integrado MAX7219 usando comunicación SPI.
 * 
 * Esta librería permite inicializar y controlar una matriz de LEDs utilizando el controlador MAX7219.
 * 
 * @author Daniel Ruiz
 * @date Feb 5, 2025
 * @version 1.0
 */

#ifndef MAX7219_H_
#define MAX7219_H_

//Ajustar dependiendo del microcontrolador
#include "stm32f1xx_hal.h"

#include <stdint.h>

#define NUM_DEV 4 /**< Número de dispositivos MAX7219 en cascada */
#define FONT_WIDTH 8 /**< Ancho de cada carácter en el conjunto de fuentes */

extern uint8_t bufferCol[NUM_DEV*8];	// Número de columnas respecto al número de dispositivos conectados en cascada

#define CS_GPIO_Port GPIOA	 // Cambia GPIOB por el puerto correcto
#define CS_Pin GPIO_PIN_4   // Cambia el número de pin según la conexión en tu PCB

/**
 * @brief Inicializa la matriz de LEDs.
 * 
 * Configura los registros del MAX7219 para preparar la matriz de LEDs para su uso.
 */
void MAX7219_Init(void);

/**
 * @brief Escribe un byte de datos en una fila específica de la matriz de LEDs.
 * 
 * @param row La fila en la que se escribirá el dato (1-8).
 * @param data El byte de datos que se escribirá en la fila.
 */
void max7219_write(int row, uint8_t data);

/**
 * @brief Envía un comando al MAX7219.
 * 
 * @param Addr La dirección del registro del MAX7219.
 * @param data El dato que se escribirá en el registro.
 */
void max7219_cmd(uint8_t Addr, uint8_t data);

/**
 * @brief Muestra un número en la matriz de LEDs.
 * 
 * @param num Número a mostrar (0-9).
 */
void MatrixData(int num);

/**
 * @brief Limpia el buffer de la matriz de LEDs y actualiza la pantalla.
 */
void flushBuffer(void);

/**
 * @brief Apaga todos los LEDs de la matriz.
 */
void MAX7219_clearDisplay(void);

/**
 * @brief Desplaza el contenido de la matriz hacia la izquierda.
 */
void shiftLeft(void);

/**
 * @brief Desplaza el contenido de la matriz hacia la derecha.
 */
void shiftRight(void);

/**
 * @brief Desplaza un carácter a través de la matriz de LEDs con un retardo específico.
 * 
 * Convierte un carácter en su representación de bits y lo desplaza a través de la matriz
 * de LEDs, mostrando el efecto de desplazamiento con un retardo configurable.
 * 
 * @param ch Carácter que se desplazará.
 * @param delay Retardo entre cada desplazamiento en milisegundos.
 */
void shiftchar(uint8_t ch, int delay);

/**
 * @brief Muestra una cadena de caracteres en la matriz de LEDs con desplazamiento.
 * 
 * @param str Cadena de caracteres a mostrar.
 * @param delay Tiempo de retardo entre cada desplazamiento.
 */
void MAX7219_scrollString (char *str, int delay);

/**
 * @brief Muestra una cadena de caracteres en la matriz de LEDs.
 * 
 * Esta función toma una cadena de texto y la convierte en una representación de bytes
 * para visualizarla en la matriz de LEDs. Cada carácter se almacena en el buffer de
 * columnas y se muestra en la pantalla.
 * 
 * @param str Puntero a la cadena de caracteres que se mostrará.
 */
void MAX7219_printString(const char *str);

/**
 * @brief Conjunto de fuentes para la pantalla de matriz de puntos MAX7219.
 *
 * Este array contiene el conjunto de fuentes de 8x8 para la pantalla de matriz de puntos MAX7219.
 * Cada carácter está representado por un array de 8 bytes, donde cada byte representa
 * una fila del carácter.
 */
extern const uint8_t MAX7219_Dot_Matrix_font[256][8];

#endif /* MAX7219_H_ */
