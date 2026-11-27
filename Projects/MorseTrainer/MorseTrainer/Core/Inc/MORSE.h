/**
 * @file MORSE.h
 *
 * @brief Lógica Morse pura (sin HAL): codificación, tiempos WPM + Farnsworth,
 *        reproductor y decodificador avanzados por ticks de 1 ms, progresión
 *        Koch y estadísticas de la sesión.
 *
 * Compila igual en el STM32 y en el PC (ver pruebas/test_morse.c), por eso no
 * incluye main.h. El módulo no tiene estado global: cada función trabaja sobre
 * la estructura que recibe, así que no hay MORSE_Init()/MORSE_DeInit(); cada
 * estructura tiene su propia función de arranque (MORSE_DecodInit, MORSE_KochInit…).
 *
 * Errores: las funciones de arranque y registro validan sus punteros y devuelven
 * MORSE_Status_t. Las de ticks (1 kHz, desde la ISR) y las consultas devuelven el
 * valor directamente y esperan punteros válidos.
 *
 * @author Daniel Ruiz
 * @date Noviembre 26, 2026
 * @version 1.0.0
 */

#ifndef MORSE_H
#define MORSE_H

// ============================================================================
// INCLUDES
// ============================================================================

#include <stdint.h>

// ============================================================================
// MACROS Y CONSTANTES MORSE
// ============================================================================

#define MORSE_MAX_SIMBOLOS          5U          /**< Símbolos del carácter soportado más largo (números) */

#ifndef MORSE_PUNTO_MIN_MS
#define MORSE_PUNTO_MIN_MS          20U         /**< Punto estimado mínimo del decodificador: 60 WPM (redefinible) */
#endif
#ifndef MORSE_PUNTO_MAX_MS
#define MORSE_PUNTO_MAX_MS          400U        /**< Punto estimado máximo del decodificador: 3 WPM (redefinible) */
#endif

/* Progresión Koch: los caracteres se desbloquean en el orden MORSE_KochOrden (LCWO);
 * el siguiente llega al acertar MORSE_KOCH_ACIERTOS de los últimos MORSE_KOCH_VENTANA intentos. */
#define MORSE_KOCH_TOTAL            36U         /**< Caracteres en MORSE_KochOrden */
#define MORSE_KOCH_MINIMO           2U          /**< Nivel inicial (K y M) */

#ifndef MORSE_KOCH_VENTANA
#define MORSE_KOCH_VENTANA          20U         /**< Intentos que se miran para desbloquear (≤ 32, redefinible) */
#endif
#ifndef MORSE_KOCH_ACIERTOS
#define MORSE_KOCH_ACIERTOS         18U         /**< Aciertos necesarios en la ventana: 90 % (redefinible) */
#endif

#if (MORSE_KOCH_VENTANA > 32U) || (MORSE_KOCH_ACIERTOS > MORSE_KOCH_VENTANA)
#error "MORSE_KOCH_VENTANA debe caber en 32 bits y ser >= MORSE_KOCH_ACIERTOS"
#endif

/* Estadísticas de la sesión. "Reconocido" = correcto y dentro de la latencia máxima. */
#define MORSE_SESION_SIN_LATENCIA   UINT16_MAX  /**< No hubo pulsación (sin respuesta) */
#define MORSE_SESION_REPASO_MAX     9U          /**< Caracteres máximos del repaso: 3 fallados + 3 pares */

#ifndef MORSE_SESION_NUEVO_INTENTOS
#define MORSE_SESION_NUEVO_INTENTOS 10U         /**< Intentos con peso extra del recién desbloqueado (redefinible) */
#endif

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del módulo MORSE.
 */
typedef enum {
    MORSE_OK               = 0,    /**< Operación exitosa */
    MORSE_ERROR            = 1,    /**< Error en la operación (sin uso: no hay bus) */
    MORSE_TIMEOUT          = 2,    /**< Timeout en la operación (sin uso: no hay bus) */
    MORSE_NOT_INITIALIZED  = 3,    /**< Módulo no inicializado (sin uso: no hay estado global) */
    MORSE_INVALID_PARAM    = 4     /**< Parámetro inválido */
} MORSE_Status_t;

/**
 * @brief Duraciones en ms. Raya = 3 puntos; espacio entre símbolos = 1 punto.
 */
typedef struct {
    uint16_t punto_ms;    /**< Unidad a la velocidad de carácter */
    uint16_t letra_ms;    /**< Silencio entre caracteres (alargado por Farnsworth) */
    uint16_t palabra_ms;  /**< Silencio entre palabras (alargado por Farnsworth) */
} MORSE_Tiempos_t;

/**
 * @brief Reproductor no bloqueante.
 *
 * MORSE_ReprodTick() se llama cada 1 ms (ISR) y dice si el tono debe sonar en ese
 * milisegundo. Desde el bucle principal, MORSE_Reproducir() se llama con la
 * interrupción del tick deshabilitada y el estado se consulta con MORSE_Reproduciendo().
 */
typedef struct {
    const char      *texto;    /**< Siguiente carácter por reproducir; NULL = parado */
    const char      *simbolo;  /**< Siguiente símbolo del carácter en curso */
    MORSE_Tiempos_t  t;        /**< Tiempos de la reproducción */
    uint16_t         ms;       /**< Lo que queda del tramo actual */
    uint8_t          tono;     /**< 1 si el tramo actual suena */
} MORSE_Reproductor_t;

/**
 * @brief Decodificador adaptativo.
 *
 * MORSE_DecodTick() se llama cada 1 ms con el estado ya sin rebotes del manipulador.
 * Se adapta a la velocidad del usuario: 'punto_ms' empieza en el punto de
 * reproducción y sigue al punto real con cada letra válida.
 * Umbrales (u = punto estimado):
 *   - silencio > 2u → fin de letra; silencio > máx((letra_ms + palabra_ms) / 2, 5u)
 *     → fin de palabra (5u sin Farnsworth).
 *   - Punto o raya, al cerrar la letra: si mezcla pulsaciones cortas y largas (la más
 *     larga ≥ 2× la más corta), el umbral es el punto medio entre ambas, sea cual sea
 *     la velocidad; si todas se parecen (E, T, M, S, O…), raya = pulsación ≥ 2u.
 */
typedef struct {
    char     patron[MORSE_MAX_SIMBOLOS + 1U];       /**< Tras devolver un carácter: lo manipulado */
    uint16_t pulso_ms[MORSE_MAX_SIMBOLOS];          /**< Duraciones de la letra en curso */
    uint16_t hueco_ms[MORSE_MAX_SIMBOLOS - 1U];     /**< Silencio tras cada símbolo, dentro de la letra */
    uint8_t  n;           /**< Símbolos de la letra en curso (satura en MAX + 1) */
    uint8_t  pulsado;     /**< Último estado del manipulador */
    uint8_t  palabra;     /**< Hay letras desde el último espacio */
    uint16_t ms;          /**< Duración de la pulsación o del silencio en curso */
    uint16_t punto_ms;    /**< Punto estimado del usuario (1200 / punto_ms = sus WPM) */
    uint16_t palabra_ms;  /**< Umbral de fin de palabra según la reproducción */
} MORSE_Decodificador_t;

/**
 * @brief Progreso en el método Koch.
 */
typedef struct {
    uint8_t  nivel;       /**< Desbloqueados: MORSE_KochOrden[0 .. nivel - 1] */
    uint8_t  intentos;    /**< Intentos en la ventana (satura en MORSE_KOCH_VENTANA) */
    uint32_t historial;   /**< Bit 0 = último intento, 1 = acierto */
} MORSE_KochProgreso_t;

/**
 * @brief Estadísticas de la sesión (modo Identificar). Índices en MORSE_KochOrden.
 */
typedef struct {
    uint16_t intentos[MORSE_KOCH_TOTAL];                    /**< Veces que sonó cada carácter */
    uint16_t aciertos[MORSE_KOCH_TOTAL];                    /**< Reconocidos */
    uint8_t  confusion[MORSE_KOCH_TOTAL][MORSE_KOCH_TOTAL]; /**< [oído][respondido], satura en 255 */
    uint32_t latencia_total_ms;                             /**< Suma de las latencias medidas */
    uint16_t latencias;                                     /**< Respuestas con latencia medida */
} MORSE_Sesion_t;

// ============================================================================
// VARIABLES PÚBLICAS
// ============================================================================

/** @brief Orden de desbloqueo Koch (LCWO), terminado en '\0'. */
extern const char MORSE_KochOrden[MORSE_KOCH_TOTAL + 1U];

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Codificación ----------------------------- */

/**
 * @brief Patrón Morse de un carácter ("." punto, "-" raya).
 *
 * @param c           A–Z, a–z o 0–9.
 * @param[out] patron Cadena constante con el patrón (sin cambios si falla).
 *
 * @return MORSE_OK si el carácter está soportado.
 * @return MORSE_INVALID_PARAM si patron es NULL o el carácter no está soportado.
 */
MORSE_Status_t MORSE_Patron(char c, const char **patron);

/**
 * @brief Tiempos para una velocidad de carácter y una velocidad efectiva
 *        (Farnsworth, fórmula ARRL). No puede fallar.
 *
 * @param wpm_caracter Velocidad de los símbolos (0 se trata como 1).
 * @param wpm_efectivo Velocidad media con los silencios alargados. 0 o mayor que
 *                     wpm_caracter → sin Farnsworth (espaciado estándar 3/7).
 *
 * @return Los tiempos calculados.
 */
MORSE_Tiempos_t MORSE_CalcularTiempos(uint8_t wpm_caracter, uint8_t wpm_efectivo);

/* ------------------------------ Reproductor ----------------------------- */

/**
 * @brief Empieza a reproducir un texto (descarta lo que estuviera sonando).
 *        Los espacios separan palabras; los caracteres no soportados se saltan.
 *
 * @param r     Reproductor.
 * @param texto Debe seguir siendo válido hasta que termine la reproducción.
 * @param t     Tiempos de la reproducción.
 *
 * @return MORSE_OK si se cargó el texto (vacío = parado).
 * @return MORSE_INVALID_PARAM si r o texto son NULL.
 */
MORSE_Status_t MORSE_Reproducir(MORSE_Reproductor_t *r, const char *texto, MORSE_Tiempos_t t);

/**
 * @brief Avanza el reproductor 1 ms. Llamar desde la ISR del tick.
 *
 * @param r Reproductor válido.
 *
 * @return 1 si el tono debe sonar en este milisegundo, 0 si no.
 */
uint8_t MORSE_ReprodTick(MORSE_Reproductor_t *r);

/**
 * @brief Indica si queda algo por reproducir.
 *
 * @param r Reproductor válido.
 *
 * @return 1 si está reproduciendo, 0 si está parado.
 */
uint8_t MORSE_Reproduciendo(const MORSE_Reproductor_t *r);

/* ----------------------------- Decodificador ---------------------------- */

/**
 * @brief Arranca el decodificador con la velocidad de reproducción.
 *
 * @param d Decodificador.
 * @param t Velocidad de reproducción: punto inicial y umbral de palabra.
 *
 * @return MORSE_OK si se inicializó.
 * @return MORSE_INVALID_PARAM si d es NULL.
 */
MORSE_Status_t MORSE_DecodInit(MORSE_Decodificador_t *d, MORSE_Tiempos_t t);

/**
 * @brief Descarta la letra en curso y conserva el punto estimado.
 *
 * @param d Decodificador.
 *
 * @return MORSE_OK si se vació.
 * @return MORSE_INVALID_PARAM si d es NULL.
 */
MORSE_Status_t MORSE_DecodVaciar(MORSE_Decodificador_t *d);

/**
 * @brief Avanza el decodificador 1 ms. Llamar desde la ISR del tick.
 *
 * @param d       Decodificador válido.
 * @param pulsado Estado del manipulador ya sin rebotes (1 = pulsado).
 *
 * @return El carácter al cerrar una letra ('?' si el patrón no existe),
 *         ' ' al cerrar una palabra, o 0 si no hay nada nuevo.
 */
char MORSE_DecodTick(MORSE_Decodificador_t *d, uint8_t pulsado);

/**
 * @brief Calidad de la manipulación de la última letra cerrada, comparada con el
 *        carácter esperado. La tolerancia se estrecha con el nivel Koch: relación
 *        raya/punto 3 ± 1 en el nivel 2 → 3 ± 0,5 en el 36; hueco máximo entre
 *        símbolos 1,8 → 1,4 puntos.
 *
 * @param d        Decodificador justo tras devolver el carácter (pulso_ms, hueco_ms, patron).
 * @param esperado Carácter que debía manipularse.
 * @param nivel    Nivel Koch (se recorta a [MORSE_KOCH_MINIMO, MORSE_KOCH_TOTAL]).
 *
 * @return Indicación breve para la pantalla, o NULL si el ritmo es bueno o si se
 *         manipuló otro carácter (confusión de oído, no de ritmo).
 */
const char *MORSE_EvaluarManip(const MORSE_Decodificador_t *d, char esperado, uint8_t nivel);

/* ---------------------------- Progresión Koch --------------------------- */

/**
 * @brief Fija el nivel y vacía la ventana de intentos.
 *
 * @param p     Progreso.
 * @param nivel Nivel deseado (se recorta a [MORSE_KOCH_MINIMO, MORSE_KOCH_TOTAL]).
 *
 * @return MORSE_OK si se inicializó.
 * @return MORSE_INVALID_PARAM si p es NULL.
 */
MORSE_Status_t MORSE_KochInit(MORSE_KochProgreso_t *p, uint8_t nivel);

/**
 * @brief Aciertos dentro de la ventana.
 *
 * @param p Progreso válido.
 *
 * @return Aciertos (0 .. MORSE_KOCH_VENTANA).
 */
uint8_t MORSE_KochAciertos(const MORSE_KochProgreso_t *p);

/**
 * @brief Anota un intento y desbloquea el siguiente carácter si toca.
 *
 * @param p       Progreso válido.
 * @param acierto Distinto de 0 si el intento fue correcto.
 *
 * @return 1 si el intento desbloqueó MORSE_KochOrden[nivel - 1] (la ventana vuelve
 *         a empezar), 0 si no.
 */
uint8_t MORSE_KochRegistrar(MORSE_KochProgreso_t *p, uint8_t acierto);

/**
 * @brief Velocidad efectiva (Farnsworth) para un nivel: 'inicial' en MORSE_KOCH_MINIMO
 *        y crece en línea recta hasta 'caracter' (sin Farnsworth) en MORSE_KOCH_TOTAL.
 *
 * @param nivel    Nivel Koch (se recorta al rango válido).
 * @param inicial  WPM efectivos en el primer nivel (se limita a 'caracter').
 * @param caracter WPM de carácter.
 *
 * @return WPM efectivos para ese nivel.
 */
uint8_t MORSE_KochWpmEfectivo(uint8_t nivel, uint8_t inicial, uint8_t caracter);

/* ------------------------- Estadísticas de sesión ----------------------- */

/**
 * @brief Pone a cero las estadísticas.
 *
 * @param s Sesión.
 *
 * @return MORSE_OK si se inicializó.
 * @return MORSE_INVALID_PARAM si s es NULL.
 */
MORSE_Status_t MORSE_SesionInit(MORSE_Sesion_t *s);

/**
 * @brief Anota una respuesta.
 *
 * @param s           Sesión.
 * @param oido        Carácter que sonó.
 * @param respondido  Carácter manipulado; '?' o 0 (sin respuesta) no cuentan como confusión.
 * @param reconocido  Distinto de 0 si fue correcto y a tiempo.
 * @param latencia_ms Fin de la reproducción → primera pulsación, o MORSE_SESION_SIN_LATENCIA.
 *
 * @return MORSE_OK si se anotó.
 * @return MORSE_INVALID_PARAM si s es NULL u 'oido' no está en MORSE_KochOrden (no se anota nada).
 */
MORSE_Status_t MORSE_SesionRegistrar(MORSE_Sesion_t *s, char oido, char respondido,
                                     uint8_t reconocido, uint16_t latencia_ms);

/**
 * @brief Carácter aleatorio entre MORSE_KochOrden[0 .. nivel - 1], con más peso para
 *        los de menor acierto y para el recién desbloqueado en sus primeros intentos.
 *
 * @param s     Sesión válida.
 * @param nivel Nivel Koch actual (≥ 1).
 * @param solo  Restringe la elección a estos caracteres (NULL, vacío o sin ninguno
 *              desbloqueado = todos).
 * @param azar  Cualquier número aleatorio (p. ej. rand()).
 *
 * @return El carácter elegido.
 */
char MORSE_SesionElegir(const MORSE_Sesion_t *s, uint8_t nivel, const char *solo, uint32_t azar);

/**
 * @brief Caracteres para el repaso dirigido: los 3 más fallados y los dos de cada uno
 *        de los 3 pares más confundidos, sin repetir y solo los desbloqueados.
 *
 * @param s           Sesión válida.
 * @param nivel       Nivel Koch actual.
 * @param[out] salida De MORSE_SESION_REPASO_MAX + 1; "" si no hay fallos.
 *
 * @return Caracteres escritos en 'salida'.
 */
uint8_t MORSE_SesionRepaso(const MORSE_Sesion_t *s, uint8_t nivel, char *salida);

/**
 * @brief Hasta 'max' caracteres con fallos, de más a menos.
 *
 * @param s           Sesión válida.
 * @param[out] salida De max + 1.
 * @param max         Caracteres como mucho.
 *
 * @return Caracteres escritos en 'salida'.
 */
uint8_t MORSE_SesionMasFallados(const MORSE_Sesion_t *s, char *salida, uint8_t max);

/**
 * @brief Hasta 'max' pares "oído respondido" más repetidos.
 *
 * @param s           Sesión válida.
 * @param[out] salida De 2·max + 1.
 * @param max         Pares como mucho.
 *
 * @return Pares escritos en 'salida'.
 */
uint8_t MORSE_SesionConfusiones(const MORSE_Sesion_t *s, char *salida, uint8_t max);

#ifdef __cplusplus
}
#endif

#endif /* MORSE_H */
