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
 * rebotes del manipulador. Se adapta a la velocidad del usuario: 'punto_ms'
 * empieza en el punto de reproducción y sigue al punto real con cada letra válida.
 * Umbrales (u = punto estimado):
 *   silencio > 2u → fin de letra; silencio > máx((letra_ms + palabra_ms) / 2, 5u)
 *   → fin de palabra (5u sin Farnsworth).
 *   Punto o raya, al cerrar la letra: si mezcla pulsaciones cortas y largas (la más
 *   larga ≥ 2× la más corta), el umbral es el punto medio entre ambas, sea cual sea
 *   la velocidad; si todas se parecen (E, T, M, S, O…), raya = pulsación ≥ 2u.
 * ------------------------------------------------------------------------- */
#define MORSE_MAX_SIMBOLOS 5U   /* El carácter soportado más largo (números) */
#define MORSE_PUNTO_MIN_MS 20U  /* Límites del punto estimado: 60 y 3 WPM */
#define MORSE_PUNTO_MAX_MS 400U

typedef struct
{
  char     patron[MORSE_MAX_SIMBOLOS + 1U]; /* Tras devolver un carácter: lo manipulado */
  uint16_t pulso_ms[MORSE_MAX_SIMBOLOS];    /* Duraciones de la letra en curso */
  uint16_t hueco_ms[MORSE_MAX_SIMBOLOS - 1U]; /* Silencio tras cada símbolo, dentro de la letra */
  uint8_t  n;          /* Símbolos de la letra en curso (satura en MAX + 1) */
  uint8_t  pulsado;
  uint8_t  palabra;    /* Hay letras desde el último espacio */
  uint16_t ms;         /* Duración de la pulsación o del silencio en curso */
  uint16_t punto_ms;   /* Punto estimado del usuario (1200 / punto_ms = sus WPM) */
  uint16_t palabra_ms; /* Umbral de fin de palabra según la reproducción */
} Morse_Decodificador;

/** @param t  Velocidad de reproducción: punto inicial y umbral de palabra. */
void Morse_DecodInit(Morse_Decodificador *d, Morse_Tiempos t);

/** @brief Descarta la letra en curso y conserva el punto estimado. */
void Morse_DecodVaciar(Morse_Decodificador *d);

/**
 * @return El carácter al cerrar una letra ('?' si el patrón no existe),
 *         ' ' al cerrar una palabra, o 0 si no hay nada nuevo.
 */
char Morse_DecodTick(Morse_Decodificador *d, uint8_t pulsado);

/**
 * @brief  Calidad de la manipulación de la última letra cerrada, comparada con el
 *         carácter esperado. La tolerancia se estrecha con el nivel Koch: relación
 *         raya/punto 3 ± 1 en el nivel 2 → 3 ± 0,5 en el 36; hueco máximo entre
 *         símbolos 1,8 → 1,4 puntos.
 * @param  d  Decodificador justo tras devolver el carácter (pulso_ms, hueco_ms, patron).
 * @return Indicación breve para la pantalla, o NULL si el ritmo es bueno o si se
 *         manipuló otro carácter (confusión de oído, no de ritmo).
 */
const char *Morse_EvaluarManip(const Morse_Decodificador *d, char esperado, uint8_t nivel);

/* ---------------------------------------------------------------------------
 * Progresión Koch: los caracteres se desbloquean en el orden KOCH_ORDEN (LCWO);
 * el siguiente llega al acertar KOCH_ACIERTOS de los últimos KOCH_VENTANA intentos.
 * ------------------------------------------------------------------------- */
#define KOCH_TOTAL     36U
#define KOCH_MINIMO    2U
#define KOCH_VENTANA   20U
#define KOCH_ACIERTOS  18U   /* 90 % de la ventana */

extern const char KOCH_ORDEN[KOCH_TOTAL + 1U];

typedef struct
{
  uint8_t  nivel;      /* Desbloqueados: KOCH_ORDEN[0 .. nivel - 1] */
  uint8_t  intentos;   /* Intentos en la ventana (satura en KOCH_VENTANA) */
  uint32_t historial;  /* Bit 0 = último intento, 1 = acierto */
} Koch_Progreso;

/** @brief Fija el nivel (recortado a [KOCH_MINIMO, KOCH_TOTAL]) y vacía la ventana. */
void    Koch_Init(Koch_Progreso *p, uint8_t nivel);
uint8_t Koch_Aciertos(const Koch_Progreso *p);
/** @return 1 si el intento desbloqueó KOCH_ORDEN[nivel - 1] (la ventana vuelve a empezar). */
uint8_t Koch_Registrar(Koch_Progreso *p, uint8_t acierto);
/**
 * @brief  Velocidad efectiva (Farnsworth) para un nivel: 'inicial' en KOCH_MINIMO y
 *         crece en línea recta hasta 'caracter' (sin Farnsworth) en KOCH_TOTAL.
 */
uint8_t Koch_WpmEfectivo(uint8_t nivel, uint8_t inicial, uint8_t caracter);

/* ---------------------------------------------------------------------------
 * Estadísticas de la sesión (modo Identificar) y selección ponderada.
 * Índices en KOCH_ORDEN. "Reconocido" = correcto y dentro de la latencia máxima.
 * ------------------------------------------------------------------------- */
#define SESION_SIN_LATENCIA   UINT16_MAX  /* No hubo pulsación (sin respuesta) */
#define SESION_NUEVO_INTENTOS 10U         /* Intentos con peso extra del recién desbloqueado */

typedef struct
{
  uint16_t intentos[KOCH_TOTAL];
  uint16_t aciertos[KOCH_TOTAL];                /* Reconocidos */
  uint8_t  confusion[KOCH_TOTAL][KOCH_TOTAL];   /* [oído][respondido], satura en 255 */
  uint32_t latencia_total_ms;
  uint16_t latencias;                           /* Respuestas con latencia medida */
} Sesion;

void Sesion_Init(Sesion *s);
/**
 * @param respondido  Carácter manipulado; '?' o 0 (sin respuesta) no cuentan como confusión.
 * @param latencia_ms Fin de la reproducción → primera pulsación, o SESION_SIN_LATENCIA.
 */
void Sesion_Registrar(Sesion *s, char oido, char respondido, uint8_t reconocido,
                      uint16_t latencia_ms);
/**
 * @brief  Carácter aleatorio entre KOCH_ORDEN[0 .. nivel - 1], con más peso para los
 *         de menor acierto y para el recién desbloqueado en sus primeros intentos.
 * @param  azar  Cualquier número aleatorio (p. ej. rand()).
 */
char    Sesion_Elegir(const Sesion *s, uint8_t nivel, uint32_t azar);
/** @brief Hasta 'max' caracteres con fallos, de más a menos ('salida' de max + 1). */
uint8_t Sesion_MasFallados(const Sesion *s, char *salida, uint8_t max);
/** @brief Hasta 'max' pares "oído respondido" más repetidos ('salida' de 2·max + 1). */
uint8_t Sesion_Confusiones(const Sesion *s, char *salida, uint8_t max);

#endif /* MORSE_H */
