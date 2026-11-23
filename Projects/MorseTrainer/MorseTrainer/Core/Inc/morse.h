/**
 * @file  morse.h
 * @brief Lógica Morse pura (sin HAL): codificación y tiempos WPM + Farnsworth.
 *        Compila igual en el STM32 y en el PC (ver pruebas/test_morse.c).
 */
#ifndef MORSE_H
#define MORSE_H

#include <stdint.h>

/** Duraciones en ms. Raya = 3 puntos; espacio entre símbolos = 1 punto. */
typedef struct
{
  uint16_t punto_ms;    /* Unidad a la velocidad de carácter */
  uint16_t letra_ms;    /* Silencio entre caracteres (alargado por Farnsworth) */
  uint16_t palabra_ms;  /* Silencio entre palabras (alargado por Farnsworth) */
} Morse_Tiempos;

/**
 * @brief  Patrón Morse de un carácter ("." punto, "-" raya).
 * @param  c  A–Z, a–z o 0–9.
 * @return Cadena constante, o NULL si el carácter no está soportado.
 */
const char *Morse_Patron(char c);

/**
 * @brief  Tiempos para una velocidad de carácter y una velocidad efectiva (Farnsworth, fórmula ARRL).
 * @param  wpm_caracter  Velocidad de los símbolos (0 se trata como 1).
 * @param  wpm_efectivo  Velocidad media con los silencios alargados. 0 o mayor que
 *                       wpm_caracter → sin Farnsworth (espaciado estándar 3/7).
 */
Morse_Tiempos Morse_CalcularTiempos(uint8_t wpm_caracter, uint8_t wpm_efectivo);

#endif /* MORSE_H */
