/**
 * @file  morse.h
 * @brief Lógica Morse pura (sin HAL): codificación, tiempos WPM + Farnsworth,
 *        reproductor y decodificador avanzados por ticks de 1 ms.
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

/* ---------------------------------------------------------------------------
 * Reproductor no bloqueante: Morse_ReprodTick() se llama cada 1 ms (ISR) y
 * dice si el tono debe sonar en ese milisegundo.
 * Desde el bucle principal, Morse_Reproducir() se llama con la interrupción
 * del tick deshabilitada y el estado se consulta con Morse_Reproduciendo().
 * ------------------------------------------------------------------------- */
typedef struct
{
  const char   *texto;    /* Siguiente carácter por reproducir; NULL = parado */
  const char   *simbolo;  /* Siguiente símbolo del carácter en curso */
  Morse_Tiempos t;
  uint16_t      ms;       /* Lo que queda del tramo actual */
  uint8_t       tono;     /* 1 si el tramo actual suena */
} Morse_Reproductor;

/**
 * @brief Empieza a reproducir un texto (descarta lo que estuviera sonando).
 *        Los espacios separan palabras; los caracteres no soportados se saltan.
 * @param texto  Debe seguir siendo válido hasta que termine la reproducción.
 */
void    Morse_Reproducir(Morse_Reproductor *r, const char *texto, Morse_Tiempos t);
uint8_t Morse_ReprodTick(Morse_Reproductor *r);              /* 1 = tono encendido */
uint8_t Morse_Reproduciendo(const Morse_Reproductor *r);

/* ---------------------------------------------------------------------------
 * Decodificador: Morse_DecodTick() se llama cada 1 ms con el estado ya sin
 * rebotes del manipulador. Umbrales:
 *   pulsación < 2 puntos → punto, si no raya; silencio > 2 puntos → fin de letra;
 *   silencio > (letra_ms + palabra_ms) / 2 → fin de palabra (5 puntos sin Farnsworth).
 * ------------------------------------------------------------------------- */
#define MORSE_MAX_SIMBOLOS 5U   /* El carácter soportado más largo (números) */

typedef struct
{
  char     patron[MORSE_MAX_SIMBOLOS + 1U]; /* Tras devolver un carácter: lo manipulado */
  uint8_t  n;          /* Símbolos de la letra en curso (satura en MAX + 1) */
  uint8_t  pulsado;
  uint8_t  palabra;    /* Hay letras desde el último espacio */
  uint16_t ms;         /* Duración de la pulsación o del silencio en curso */
  uint16_t punto_ms;
  uint16_t palabra_ms; /* Umbral de fin de palabra */
} Morse_Decodificador;

/** @param t  Velocidad esperada; se puede ajustar en hardware si el usuario manipula distinto. */
void Morse_DecodInit(Morse_Decodificador *d, Morse_Tiempos t);

/**
 * @return El carácter al cerrar una letra ('?' si el patrón no existe),
 *         ' ' al cerrar una palabra, o 0 si no hay nada nuevo.
 */
char Morse_DecodTick(Morse_Decodificador *d, uint8_t pulsado);

#endif /* MORSE_H */
