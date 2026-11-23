/**
 * @file  morse.c
 * @brief Lógica Morse pura (sin HAL): codificación, tiempos WPM + Farnsworth,
 *        reproductor y decodificador avanzados por ticks de 1 ms.
 */
#include "morse.h"
#include <stddef.h>
#include <string.h>

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

/* ======================== Reproductor ======================== */

/* Salta lo no soportado; devuelve 1 si se cruzó un espacio */
static uint8_t SaltarNoSoportados(const char **texto)
{
  uint8_t espacio = 0;
  while (**texto != '\0' && Morse_Patron(**texto) == NULL)
  {
    espacio |= (**texto == ' ');
    (*texto)++;
  }
  return espacio;
}

void Morse_Reproducir(Morse_Reproductor *r, const char *texto, Morse_Tiempos t)
{
  SaltarNoSoportados(&texto);
  r->texto   = (*texto != '\0') ? texto : NULL;
  r->simbolo = "";
  r->t       = t;
  r->ms      = 0;
  r->tono    = 0;
}

uint8_t Morse_Reproduciendo(const Morse_Reproductor *r)
{
  return r->texto != NULL;
}

/* Carga el siguiente tramo (símbolo o silencio). 0 = fin del texto. */
static uint8_t SiguienteTramo(Morse_Reproductor *r)
{
  if (r->tono)                                  /* Acabó un símbolo: toca silencio */
  {
    r->tono = 0;
    if (*r->simbolo != '\0')
    {
      r->ms = r->t.punto_ms;                    /* Entre símbolos de la letra */
      return 1;
    }
    uint8_t espacio = SaltarNoSoportados(&r->texto);
    if (*r->texto == '\0')
    {
      r->texto = NULL;                          /* Sin silencio final */
      return 0;
    }
    r->ms = espacio ? r->t.palabra_ms : r->t.letra_ms;
    return 1;
  }

  if (*r->simbolo == '\0')                      /* Acabó un silencio entre caracteres */
  {
    r->simbolo = Morse_Patron(*r->texto++);     /* texto apunta siempre a uno válido */
  }
  r->ms   = (uint16_t)((*r->simbolo++ == '.') ? r->t.punto_ms : 3U * r->t.punto_ms);
  r->tono = 1;
  return 1;
}

uint8_t Morse_ReprodTick(Morse_Reproductor *r)
{
  if (r->ms == 0U && (r->texto == NULL || !SiguienteTramo(r)))
  {
    return 0;
  }
  r->ms--;
  return r->tono;
}

/* ======================= Decodificador ======================= */

void Morse_DecodInit(Morse_Decodificador *d, Morse_Tiempos t)
{
  memset(d, 0, sizeof *d);
  d->punto_ms   = t.punto_ms;
  d->palabra_ms = (uint16_t)(((uint32_t)t.letra_ms + t.palabra_ms) / 2U);
}

static char BuscarPatron(const char *patron)
{
  for (uint8_t i = 0; i < 26U; i++)
  {
    if (strcmp(patron, letras[i]) == 0)
    {
      return (char)('A' + i);
    }
  }
  for (uint8_t i = 0; i < 10U; i++)
  {
    if (strcmp(patron, numeros[i]) == 0)
    {
      return (char)('0' + i);
    }
  }
  return '?';
}

char Morse_DecodTick(Morse_Decodificador *d, uint8_t pulsado)
{
  if (pulsado != d->pulsado)                    /* Flanco: reinicia la cuenta */
  {
    if (d->pulsado && d->n <= MORSE_MAX_SIMBOLOS)   /* Al soltar se clasifica */
    {
      if (d->n < MORSE_MAX_SIMBOLOS)
      {
        d->patron[d->n] = (d->ms < 2U * d->punto_ms) ? '.' : '-';
      }
      d->n++;
    }
    d->pulsado = pulsado;
    d->ms      = 0;
  }
  if (d->ms < UINT16_MAX)
  {
    d->ms++;
  }
  if (pulsado)
  {
    return 0;
  }

  if (d->n != 0U && d->ms > 2U * d->punto_ms)  /* Fin de letra */
  {
    uint8_t largo = (d->n <= MORSE_MAX_SIMBOLOS) ? d->n : MORSE_MAX_SIMBOLOS;
    d->patron[largo] = '\0';
    char c = (d->n <= MORSE_MAX_SIMBOLOS) ? BuscarPatron(d->patron) : '?';  /* > 5: no existe */
    d->n       = 0;
    d->palabra = 1;
    return c;
  }
  if (d->palabra && d->ms > d->palabra_ms)  /* Fin de palabra, una sola vez */
  {
    d->palabra = 0;
    return ' ';
  }
  return 0;
}
