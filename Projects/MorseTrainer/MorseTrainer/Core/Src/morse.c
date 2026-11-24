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
  d->punto_ms   = t.punto_ms;
  d->palabra_ms = (uint16_t)(((uint32_t)t.letra_ms + t.palabra_ms) / 2U);
  Morse_DecodVaciar(d);
}

void Morse_DecodVaciar(Morse_Decodificador *d)
{
  uint16_t punto   = d->punto_ms;
  uint16_t palabra = d->palabra_ms;
  memset(d, 0, sizeof *d);
  d->punto_ms   = punto;
  d->palabra_ms = palabra;
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

/* Media móvil (peso 1/4) del punto del usuario. Cada muestra se limita a [u/2, 2u]:
   una pulsación de varios segundos no descoloca la estimación de golpe. */
static void Adaptar(Morse_Decodificador *d, uint32_t muestra)
{
  uint32_t u = d->punto_ms;
  muestra = (muestra < u / 2U) ? u / 2U : (muestra > 2U * u) ? 2U * u : muestra;
  u = (3U * u + muestra + 2U) / 4U;
  d->punto_ms = (uint16_t)((u < MORSE_PUNTO_MIN_MS) ? MORSE_PUNTO_MIN_MS
                         : (u > MORSE_PUNTO_MAX_MS) ? MORSE_PUNTO_MAX_MS : u);
}

/* Clasifica las pulsaciones guardadas (ver morse.h) y, si el carácter existe,
   ajusta el punto estimado con cada símbolo (raya / 3 = un punto). */
static char CerrarLetra(Morse_Decodificador *d)
{
  uint8_t  largo = (d->n <= MORSE_MAX_SIMBOLOS) ? d->n : MORSE_MAX_SIMBOLOS;
  uint16_t corta = UINT16_MAX;
  uint16_t larga = 0;
  for (uint8_t i = 0; i < largo; i++)
  {
    corta = (d->pulso_ms[i] < corta) ? d->pulso_ms[i] : corta;
    larga = (d->pulso_ms[i] > larga) ? d->pulso_ms[i] : larga;
  }
  /* ponytail: una raya muy sobrada (O como 180/400/180 ms) pasa por mezcla → R;
     si molesta, exigir además que el umbral medio no se aleje de 2u */
  uint32_t umbral = (larga >= 2U * (uint32_t)corta) ? ((uint32_t)corta + larga) / 2U
                                                    : 2U * (uint32_t)d->punto_ms;
  for (uint8_t i = 0; i < largo; i++)
  {
    d->patron[i] = (d->pulso_ms[i] < umbral) ? '.' : '-';
  }
  d->patron[largo] = '\0';

  char c = (d->n <= MORSE_MAX_SIMBOLOS) ? BuscarPatron(d->patron) : '?';  /* > 5: no existe */
  if (c != '?')
  {
    for (uint8_t i = 0; i < largo; i++)
    {
      Adaptar(d, (d->patron[i] == '.') ? d->pulso_ms[i] : d->pulso_ms[i] / 3U);
    }
  }
  return c;
}

char Morse_DecodTick(Morse_Decodificador *d, uint8_t pulsado)
{
  if (pulsado != d->pulsado)                    /* Flanco: reinicia la cuenta */
  {
    if (d->pulsado && d->n <= MORSE_MAX_SIMBOLOS)   /* Al soltar se anota la duración */
    {
      if (d->n < MORSE_MAX_SIMBOLOS)
      {
        d->pulso_ms[d->n] = d->ms;
      }
      d->n++;
    }
    else if (pulsado && d->n != 0U && d->n < MORSE_MAX_SIMBOLOS)  /* Al pulsar, el hueco */
    {
      d->hueco_ms[d->n - 1U] = d->ms;
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
    char c = CerrarLetra(d);
    d->n       = 0;
    d->palabra = 1;
    return c;
  }
  /* Nunca por debajo del fin de letra: si no, saldría un espacio dentro de una letra lenta */
  uint32_t palabra = 5U * (uint32_t)d->punto_ms;
  if (d->palabra && d->ms > ((d->palabra_ms > palabra) ? d->palabra_ms : palabra))
  {
    d->palabra = 0;                             /* Fin de palabra, una sola vez */
    return ' ';
  }
  return 0;
}

const char *Morse_EvaluarManip(const Morse_Decodificador *d, char esperado, uint8_t nivel)
{
  const char *e  = Morse_Patron(esperado);
  size_t      n  = strlen(d->patron);
  if (e == NULL || n == 0U)
  {
    return NULL;
  }
  size_t ne = strlen(e);
  if (n < ne && strncmp(e, d->patron, n) == 0)
  {
    return "Letra partida";                     /* Una pausa larga cerró la letra antes */
  }
  if (n != ne)
  {
    return NULL;
  }

  /* Pulsaciones agrupadas según lo que debían ser, no según cómo se leyeron */
  uint32_t punto = 0, raya = 0;
  uint32_t np = 0, nr = 0;
  uint16_t punto_max = 0, raya_min = UINT16_MAX;
  for (size_t i = 0; i < n; i++)
  {
    uint16_t p = d->pulso_ms[i];
    if (e[i] == '.') { punto += p; np++; punto_max = (p > punto_max) ? p : punto_max; }
    else             { raya  += p; nr++; raya_min  = (p < raya_min)  ? p : raya_min;  }
  }

  nivel = (nivel < KOCH_MINIMO) ? KOCH_MINIMO : (nivel > KOCH_TOTAL) ? KOCH_TOTAL : nivel;
  uint32_t tol = 10U - 5U * (nivel - KOCH_MINIMO) / (KOCH_TOTAL - KOCH_MINIMO);  /* Décimas */

  if (np != 0U && nr != 0U && punto != 0U)
  {
    if (raya_min <= punto_max)
    {
      return NULL;   /* Alguna "raya" no supera a algún "punto": manipuló otro carácter */
    }
    uint32_t r = 10U * raya * np / (punto * nr);  /* Raya / punto, en décimas */
    if (r < 30U - tol)
    {
      return "Rayas cortas";
    }
    if (r > 30U + tol)
    {
      return "Rayas largas";
    }
  }

  uint32_t unidad = (np != 0U) ? punto / np : raya / (3U * nr);
  uint32_t hueco  = 0;
  for (size_t i = 0; i + 1U < n; i++)
  {
    hueco = (d->hueco_ms[i] > hueco) ? d->hueco_ms[i] : hueco;
  }
  if (10U * hueco > (10U + tol * 4U / 5U) * (unidad ? unidad : 1U))
  {
    return "Pausas largas";
  }
  return NULL;
}

/* ====================== Progresión Koch ====================== */

const char KOCH_ORDEN[KOCH_TOTAL + 1U] = "KMURESNAPTLWIJZFOYVG5Q92H38B47C1D60X";

void Koch_Init(Koch_Progreso *p, uint8_t nivel)
{
  p->nivel     = (nivel < KOCH_MINIMO) ? KOCH_MINIMO : (nivel > KOCH_TOTAL) ? KOCH_TOTAL : nivel;
  p->intentos  = 0;
  p->historial = 0;
}

uint8_t Koch_Aciertos(const Koch_Progreso *p)
{
  return (uint8_t)__builtin_popcount(p->historial);
}

uint8_t Koch_Registrar(Koch_Progreso *p, uint8_t acierto)
{
  p->historial = ((p->historial << 1) | (acierto != 0U)) & ((1UL << KOCH_VENTANA) - 1U);
  if (p->intentos < KOCH_VENTANA)
  {
    p->intentos++;
  }
  if (p->intentos < KOCH_VENTANA || Koch_Aciertos(p) < KOCH_ACIERTOS || p->nivel >= KOCH_TOTAL)
  {
    return 0;
  }
  Koch_Init(p, (uint8_t)(p->nivel + 1U));
  return 1;
}

uint8_t Koch_WpmEfectivo(uint8_t nivel, uint8_t inicial, uint8_t caracter)
{
  nivel   = (nivel < KOCH_MINIMO) ? KOCH_MINIMO : (nivel > KOCH_TOTAL) ? KOCH_TOTAL : nivel;
  inicial = (inicial > caracter) ? caracter : inicial;
  return (uint8_t)(inicial + (uint32_t)(caracter - inicial) * (nivel - KOCH_MINIMO)
                             / (KOCH_TOTAL - KOCH_MINIMO));
}

/* ================= Estadísticas de la sesión ================= */

/* Índice en KOCH_ORDEN, o -1 si no está (incluido 0, que strchr encontraría) */
static int KochIndice(char c)
{
  const char *p = (c != '\0') ? strchr(KOCH_ORDEN, c) : NULL;
  return (p != NULL) ? (int)(p - KOCH_ORDEN) : -1;
}

void Sesion_Init(Sesion *s)
{
  memset(s, 0, sizeof *s);
}

void Sesion_Registrar(Sesion *s, char oido, char respondido, uint8_t reconocido,
                      uint16_t latencia_ms)
{
  int i = KochIndice(oido);
  if (i < 0)
  {
    return;
  }
  /* Sin saturación: 65 535 intentos son más de 50 h seguidas */
  s->intentos[i]++;
  s->aciertos[i] = (uint16_t)(s->aciertos[i] + (reconocido != 0U));

  int j = KochIndice(respondido);
  if (j >= 0 && j != i && s->confusion[i][j] < UINT8_MAX)
  {
    s->confusion[i][j]++;
  }
  if (latencia_ms != SESION_SIN_LATENCIA)
  {
    s->latencia_total_ms += latencia_ms;
    s->latencias++;
  }
}

/* 1 + 4 · tasa de fallo suavizada: 3 sin intentos, → 1 si siempre acierta, → 5 si
   siempre falla. ×3 para el recién desbloqueado en sus primeros intentos. */
static uint32_t Peso(const Sesion *s, uint8_t nivel, uint8_t i)
{
  uint32_t n = s->intentos[i];
  uint32_t w = 1U + 4U * (n - s->aciertos[i] + 1U) / (n + 2U);
  return (i == nivel - 1U && n < SESION_NUEVO_INTENTOS) ? 3U * w : w;
}

char Sesion_Elegir(const Sesion *s, uint8_t nivel, uint32_t azar)
{
  uint32_t total = 0;
  for (uint8_t i = 0; i < nivel; i++)
  {
    total += Peso(s, nivel, i);
  }
  azar %= total;
  for (uint8_t i = 0; i < nivel; i++)
  {
    uint32_t w = Peso(s, nivel, i);
    if (azar < w)
    {
      return KOCH_ORDEN[i];
    }
    azar -= w;
  }
  return KOCH_ORDEN[nivel - 1U];   /* Inalcanzable */
}

uint8_t Sesion_MasFallados(const Sesion *s, char *salida, uint8_t max)
{
  /* Orden: fallos descendentes, empate por orden Koch. Cada vuelta busca el
     siguiente tras (antes_f, antes_i), así no hace falta marcar los ya elegidos. */
  uint32_t antes_f = UINT32_MAX;
  int      antes_i = -1;
  uint8_t  k = 0;
  for (; k < max; k++)
  {
    int mejor = -1;
    uint32_t mejor_f = 0;
    for (int i = 0; i < (int)KOCH_TOTAL; i++)
    {
      uint32_t f = (uint32_t)(s->intentos[i] - s->aciertos[i]);
      uint8_t despues = (f < antes_f) || (f == antes_f && i > antes_i);
      if (f > mejor_f && despues)
      {
        mejor   = i;
        mejor_f = f;
      }
    }
    if (mejor < 0)
    {
      break;
    }
    salida[k] = KOCH_ORDEN[mejor];
    antes_f   = mejor_f;
    antes_i   = mejor;
  }
  salida[k] = '\0';
  return k;
}

uint8_t Sesion_Confusiones(const Sesion *s, char *salida, uint8_t max)
{
  /* Mismo recorrido que Sesion_MasFallados sobre la matriz aplanada */
  uint32_t antes_c = UINT32_MAX;
  int      antes_i = -1;
  uint8_t  k = 0;
  for (; k < max; k++)
  {
    int mejor = -1;
    uint32_t mejor_c = 0;
    for (int i = 0; i < (int)(KOCH_TOTAL * KOCH_TOTAL); i++)
    {
      uint32_t c = s->confusion[i / KOCH_TOTAL][i % KOCH_TOTAL];
      uint8_t despues = (c < antes_c) || (c == antes_c && i > antes_i);
      if (c > mejor_c && despues)
      {
        mejor   = i;
        mejor_c = c;
      }
    }
    if (mejor < 0)
    {
      break;
    }
    salida[2 * k]      = KOCH_ORDEN[mejor / (int)KOCH_TOTAL];
    salida[2 * k + 1U] = KOCH_ORDEN[mejor % (int)KOCH_TOTAL];
    antes_c = mejor_c;
    antes_i = mejor;
  }
  salida[2 * k] = '\0';
  return k;
}
