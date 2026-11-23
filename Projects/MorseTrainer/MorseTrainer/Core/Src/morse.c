/**
 * @file  morse.c
 * @brief Lógica Morse pura (sin HAL): codificación y tiempos WPM + Farnsworth.
 */
#include "morse.h"
#include <stddef.h>

/* Tablas reutilizadas de Projects/UARTToMorseCode */
static const char *const letras[26] = {
  ".-",   "-...", "-.-.", "-..",  ".",    "..-.", "--.",  "....", "..",   /* A–I */
  ".---", "-.-",  ".-..", "--",   "-.",   "---",  ".--.", "--.-", ".-.",  /* J–R */
  "...",  "-",    "..-",  "...-", ".--",  "-..-", "-.--", "--.."          /* S–Z */
};

static const char *const numeros[10] = {
  "-----", ".----", "..---", "...--", "....-",                            /* 0–4 */
  ".....", "-....", "--...", "---..", "----."                             /* 5–9 */
};

const char *Morse_Patron(char c)
{
  if (c >= 'a' && c <= 'z')
  {
    c = (char)(c - 'a' + 'A');
  }
  if (c >= 'A' && c <= 'Z')
  {
    return letras[c - 'A'];
  }
  if (c >= '0' && c <= '9')
  {
    return numeros[c - '0'];
  }
  return NULL;
}

/* División entera redondeada al más cercano */
static uint32_t DivRedondeo(uint32_t a, uint32_t b)
{
  return (a + b / 2U) / b;
}

Morse_Tiempos Morse_CalcularTiempos(uint8_t wpm_caracter, uint8_t wpm_efectivo)
{
  uint32_t c = wpm_caracter ? wpm_caracter : 1U;
  uint32_t s = (wpm_efectivo == 0U || wpm_efectivo > c) ? c : wpm_efectivo;

  /* "PARIS " = 50 unidades: 31 de símbolos a velocidad c y 19 de silencio
   * (4 espacios de letra × 3 + 1 de palabra × 7). Farnsworth reparte en esas
   * 19 unidades el tiempo sobrante para que la palabra dure 60/s segundos:
   *   ta = (60·c − 37,2·s) / (s·c)  [s]   →   letra = 3·ta/19, palabra = 7·ta/19
   * Con s == c se reduce a 3 y 7 unidades. */
  uint32_t ta_ms_por_c = (60000U * c - 37200U * s) / s;   /* ta en ms × c; el /c va abajo */
  Morse_Tiempos t = {
    .punto_ms   = (uint16_t)DivRedondeo(1200U, c),
    .letra_ms   = (uint16_t)DivRedondeo(3U * ta_ms_por_c, 19U * c),
    .palabra_ms = (uint16_t)DivRedondeo(7U * ta_ms_por_c, 19U * c),
  };
  return t;
}
