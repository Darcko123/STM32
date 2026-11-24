/**
 * @file MORSE.c
 *
 * @brief Lógica Morse pura (sin HAL): codificación, tiempos WPM + Farnsworth,
 *        reproductor y decodificador avanzados por ticks de 1 ms, progresión
 *        Koch y estadísticas de la sesión.
 *
 * @author Daniel Ruiz
 * @date Noviembre 24, 2026
 * @version 0.1.0
 */

#include "MORSE.h"
#include <stddef.h>
#include <string.h>

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

/** Patrones A–Z (tablas reutilizadas de Projects/UARTToMorseCode) */
static const char *const MORSE_Letras[26] = {
    ".-",   "-...", "-.-.", "-..",  ".",    "..-.", "--.",  "....", "..",   /* A–I */
    ".---", "-.-",  ".-..", "--",   "-.",   "---",  ".--.", "--.-", ".-.",  /* J–R */
    "...",  "-",    "..-",  "...-", ".--",  "-..-", "-.--", "--.."          /* S–Z */
};

/** Patrones 0–9 */
static const char *const MORSE_Numeros[10] = {
    "-----", ".----", "..---", "...--", "....-",                            /* 0–4 */
    ".....", "-....", "--...", "---..", "----."                             /* 5–9 */
};

// ============================================================================
// VARIABLES PÚBLICAS
// ============================================================================

const char MORSE_KochOrden[MORSE_KOCH_TOTAL + 1U] = "KMURESNAPTLWIJZFOYVG5Q92H38B47C1D60X";

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Patrón de un carácter, o NULL si no está soportado.
 */
static const char *morse_PatronDe(char c)
{
    if (c >= 'a' && c <= 'z') { c = (char)(c - 'a' + 'A'); }
    if (c >= 'A' && c <= 'Z') { return MORSE_Letras[c - 'A'];  }
    if (c >= '0' && c <= '9') { return MORSE_Numeros[c - '0']; }
    return NULL;
}

/**
 * @brief División entera redondeada al más cercano.
 */
static uint32_t morse_DivRedondeo(uint32_t a, uint32_t b)
{
    return (a + b / 2U) / b;
}

/**
 * @brief Recorta un nivel Koch a [MORSE_KOCH_MINIMO, MORSE_KOCH_TOTAL].
 */
static uint8_t morse_RecortarNivel(uint8_t nivel)
{
    return (nivel < MORSE_KOCH_MINIMO) ? MORSE_KOCH_MINIMO
         : (nivel > MORSE_KOCH_TOTAL)  ? MORSE_KOCH_TOTAL : nivel;
}

/* ------------------------------ Reproductor ----------------------------- */

/**
 * @brief Salta lo no soportado.
 *
 * @return 1 si se cruzó un espacio, 0 si no.
 */
static uint8_t morse_SaltarNoSoportados(const char **texto)
{
    uint8_t espacio = 0U;
    while (**texto != '\0' && morse_PatronDe(**texto) == NULL)
    {
        espacio |= (uint8_t)(**texto == ' ');
        (*texto)++;
    }
    return espacio;
}

/**
 * @brief Carga el siguiente tramo (símbolo o silencio).
 *
 * @return 0 al llegar al fin del texto, 1 si hay tramo.
 */
static uint8_t morse_SiguienteTramo(MORSE_Reproductor_t *r)
{
    if (r->tono)                                    /* Acabó un símbolo: toca silencio */
    {
        r->tono = 0U;
        if (*r->simbolo != '\0')
        {
            r->ms = r->t.punto_ms;                  /* Entre símbolos de la letra */
            return 1U;
        }
        uint8_t espacio = morse_SaltarNoSoportados(&r->texto);
        if (*r->texto == '\0')
        {
            r->texto = NULL;                        /* Sin silencio final */
            return 0U;
        }
        r->ms = espacio ? r->t.palabra_ms : r->t.letra_ms;
        return 1U;
    }

    if (*r->simbolo == '\0')                        /* Acabó un silencio entre caracteres */
    {
        r->simbolo = morse_PatronDe(*r->texto++);   /* texto apunta siempre a uno válido */
    }
    r->ms   = (uint16_t)((*r->simbolo++ == '.') ? r->t.punto_ms : 3U * r->t.punto_ms);
    r->tono = 1U;
    return 1U;
}

/* ----------------------------- Decodificador ---------------------------- */

/**
 * @brief Letra de un patrón, o '?' si no existe.
 */
static char morse_BuscarPatron(const char *patron)
{
    for (uint8_t i = 0U; i < 26U; i++)
    {
        if (strcmp(patron, MORSE_Letras[i]) == 0) { return (char)('A' + i); }
    }
    for (uint8_t i = 0U; i < 10U; i++)
    {
        if (strcmp(patron, MORSE_Numeros[i]) == 0) { return (char)('0' + i); }
    }
    return '?';
}

/**
 * @brief Media móvil (peso 1/4) del punto del usuario. Cada muestra se limita a
 *        [u/2, 2u]: una pulsación de varios segundos no descoloca la estimación de golpe.
 */
static void morse_Adaptar(MORSE_Decodificador_t *d, uint32_t muestra)
{
    uint32_t u = d->punto_ms;
    muestra = (muestra < u / 2U) ? u / 2U : (muestra > 2U * u) ? 2U * u : muestra;
    u = (3U * u + muestra + 2U) / 4U;
    d->punto_ms = (uint16_t)((u < MORSE_PUNTO_MIN_MS) ? MORSE_PUNTO_MIN_MS
                           : (u > MORSE_PUNTO_MAX_MS) ? MORSE_PUNTO_MAX_MS : u);
}

/**
 * @brief Clasifica las pulsaciones guardadas (ver MORSE.h) y, si el carácter existe,
 *        ajusta el punto estimado con cada símbolo (raya / 3 = un punto).
 *
 * @return El carácter, o '?' si el patrón no existe.
 */
static char morse_CerrarLetra(MORSE_Decodificador_t *d)
{
    uint8_t  largo = (d->n <= MORSE_MAX_SIMBOLOS) ? d->n : MORSE_MAX_SIMBOLOS;
    uint16_t corta = UINT16_MAX;
    uint16_t larga = 0U;
    for (uint8_t i = 0U; i < largo; i++)
    {
        corta = (d->pulso_ms[i] < corta) ? d->pulso_ms[i] : corta;
        larga = (d->pulso_ms[i] > larga) ? d->pulso_ms[i] : larga;
    }
    /* ponytail: una raya muy sobrada (O como 180/400/180 ms) pasa por mezcla → R;
       si molesta, exigir además que el umbral medio no se aleje de 2u */
    uint32_t umbral = (larga >= 2U * (uint32_t)corta) ? ((uint32_t)corta + larga) / 2U
                                                      : 2U * (uint32_t)d->punto_ms;
    for (uint8_t i = 0U; i < largo; i++)
    {
        d->patron[i] = (d->pulso_ms[i] < umbral) ? '.' : '-';
    }
    d->patron[largo] = '\0';

    char c = (d->n <= MORSE_MAX_SIMBOLOS) ? morse_BuscarPatron(d->patron) : '?';  /* > 5: no existe */
    if (c != '?')
    {
        for (uint8_t i = 0U; i < largo; i++)
        {
            morse_Adaptar(d, (d->patron[i] == '.') ? d->pulso_ms[i] : d->pulso_ms[i] / 3U);
        }
    }
    return c;
}

/* ------------------------- Estadísticas de sesión ----------------------- */

/**
 * @brief Índice en MORSE_KochOrden, o -1 si no está (incluido 0, que strchr encontraría).
 */
static int morse_KochIndice(char c)
{
    const char *p = (c != '\0') ? strchr(MORSE_KochOrden, c) : NULL;
    return (p != NULL) ? (int)(p - MORSE_KochOrden) : -1;
}

/**
 * @brief Peso de elección: 1 + 4 · tasa de fallo suavizada (3 sin intentos, → 1 si
 *        siempre acierta, → 5 si siempre falla). ×3 para el recién desbloqueado en sus
 *        primeros intentos.
 */
static uint32_t morse_Peso(const MORSE_Sesion_t *s, uint8_t nivel, uint8_t i)
{
    uint32_t n = s->intentos[i];
    uint32_t w = 1U + 4U * (n - s->aciertos[i] + 1U) / (n + 2U);
    return (i == nivel - 1U && n < MORSE_SESION_NUEVO_INTENTOS) ? 3U * w : w;
}

/**
 * @brief Peso de elección, 0 para los que no están en 'solo' (NULL = todos).
 */
static uint32_t morse_PesoDe(const MORSE_Sesion_t *s, uint8_t nivel, const char *solo, uint8_t i)
{
    return (solo == NULL || strchr(solo, MORSE_KochOrden[i]) != NULL) ? morse_Peso(s, nivel, i) : 0U;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/* ----------------------------- Codificación ----------------------------- */

/**
 * @brief Patrón Morse de un carácter.
 *
 * @return MORSE_Status_t Estado de la operación.
 */
MORSE_Status_t MORSE_Patron(char c, const char **patron)
{
    if (patron == NULL) { return MORSE_INVALID_PARAM; }

    const char *p = morse_PatronDe(c);
    if (p == NULL) { return MORSE_INVALID_PARAM; }

    *patron = p;
    return MORSE_OK;
}

/**
 * @brief Tiempos para una velocidad de carácter y una efectiva (Farnsworth).
 *
 * @return MORSE_Tiempos_t Tiempos calculados.
 */
MORSE_Tiempos_t MORSE_CalcularTiempos(uint8_t wpm_caracter, uint8_t wpm_efectivo)
{
    uint32_t c = wpm_caracter ? wpm_caracter : 1U;
    uint32_t s = (wpm_efectivo == 0U || wpm_efectivo > c) ? c : wpm_efectivo;

    /* "PARIS " = 50 unidades: 31 de símbolos a velocidad c y 19 de silencio
     * (4 espacios de letra × 3 + 1 de palabra × 7). Farnsworth reparte en esas
     * 19 unidades el tiempo sobrante para que la palabra dure 60/s segundos:
     *   ta = (60·c − 37,2·s) / (s·c)  [s]   →   letra = 3·ta/19, palabra = 7·ta/19
     * Con s == c se reduce a 3 y 7 unidades. */
    uint32_t ta_ms_por_c = (60000U * c - 37200U * s) / s;   /* ta en ms × c; el /c va abajo */
    MORSE_Tiempos_t t = {
        .punto_ms   = (uint16_t)morse_DivRedondeo(1200U, c),
        .letra_ms   = (uint16_t)morse_DivRedondeo(3U * ta_ms_por_c, 19U * c),
        .palabra_ms = (uint16_t)morse_DivRedondeo(7U * ta_ms_por_c, 19U * c),
    };
    return t;
}

/* ------------------------------ Reproductor ----------------------------- */

/**
 * @brief Empieza a reproducir un texto.
 *
 * @return MORSE_Status_t Estado de la operación.
 */
MORSE_Status_t MORSE_Reproducir(MORSE_Reproductor_t *r, const char *texto, MORSE_Tiempos_t t)
{
    if (r == NULL || texto == NULL) { return MORSE_INVALID_PARAM; }

    (void)morse_SaltarNoSoportados(&texto);         /* Espacios iniciales: sin silencio previo */
    r->texto   = (*texto != '\0') ? texto : NULL;
    r->simbolo = "";
    r->t       = t;
    r->ms      = 0U;
    r->tono    = 0U;
    return MORSE_OK;
}

/**
 * @brief Avanza el reproductor 1 ms.
 *
 * @return 1 si el tono suena, 0 si no.
 */
uint8_t MORSE_ReprodTick(MORSE_Reproductor_t *r)
{
    if (r->ms == 0U && (r->texto == NULL || !morse_SiguienteTramo(r)))
    {
        return 0U;
    }
    r->ms--;
    return r->tono;
}

/**
 * @brief Indica si queda algo por reproducir.
 *
 * @return 1 si está reproduciendo, 0 si no.
 */
uint8_t MORSE_Reproduciendo(const MORSE_Reproductor_t *r)
{
    return (uint8_t)(r->texto != NULL);
}

/* ----------------------------- Decodificador ---------------------------- */

/**
 * @brief Arranca el decodificador.
 *
 * @return MORSE_Status_t Estado de la operación.
 */
MORSE_Status_t MORSE_DecodInit(MORSE_Decodificador_t *d, MORSE_Tiempos_t t)
{
    if (d == NULL) { return MORSE_INVALID_PARAM; }

    d->punto_ms   = t.punto_ms;
    d->palabra_ms = (uint16_t)(((uint32_t)t.letra_ms + t.palabra_ms) / 2U);
    return MORSE_DecodVaciar(d);
}

/**
 * @brief Descarta la letra en curso y conserva el punto estimado.
 *
 * @return MORSE_Status_t Estado de la operación.
 */
MORSE_Status_t MORSE_DecodVaciar(MORSE_Decodificador_t *d)
{
    if (d == NULL) { return MORSE_INVALID_PARAM; }

    uint16_t punto   = d->punto_ms;
    uint16_t palabra = d->palabra_ms;
    memset(d, 0, sizeof *d);
    d->punto_ms   = punto;
    d->palabra_ms = palabra;
    return MORSE_OK;
}

/**
 * @brief Avanza el decodificador 1 ms.
 *
 * @return Carácter cerrado, ' ' al cerrar palabra o 0.
 */
char MORSE_DecodTick(MORSE_Decodificador_t *d, uint8_t pulsado)
{
    if (pulsado != d->pulsado)                      /* Flanco: reinicia la cuenta */
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
        d->ms      = 0U;
    }
    if (d->ms < UINT16_MAX)
    {
        d->ms++;
    }
    if (pulsado)
    {
        return 0;
    }

    if (d->n != 0U && d->ms > 2U * d->punto_ms)    /* Fin de letra */
    {
        char c = morse_CerrarLetra(d);
        d->n       = 0U;
        d->palabra = 1U;
        return c;
    }
    /* Nunca por debajo del fin de letra: si no, saldría un espacio dentro de una letra lenta */
    uint32_t palabra = 5U * (uint32_t)d->punto_ms;
    if (d->palabra && d->ms > ((d->palabra_ms > palabra) ? d->palabra_ms : palabra))
    {
        d->palabra = 0U;                            /* Fin de palabra, una sola vez */
        return ' ';
    }
    return 0;
}

/**
 * @brief Calidad de la manipulación de la última letra cerrada.
 *
 * @return Indicación para la pantalla, o NULL si el ritmo es bueno.
 */
const char *MORSE_EvaluarManip(const MORSE_Decodificador_t *d, char esperado, uint8_t nivel)
{
    const char *e = morse_PatronDe(esperado);
    size_t      n = strlen(d->patron);
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
    uint32_t punto = 0U, raya = 0U;
    uint32_t np = 0U, nr = 0U;
    uint16_t punto_max = 0U, raya_min = UINT16_MAX;
    for (size_t i = 0U; i < n; i++)
    {
        uint16_t p = d->pulso_ms[i];
        if (e[i] == '.') { punto += p; np++; punto_max = (p > punto_max) ? p : punto_max; }
        else             { raya  += p; nr++; raya_min  = (p < raya_min)  ? p : raya_min;  }
    }

    nivel = morse_RecortarNivel(nivel);
    uint32_t tol = 10U - 5U * (nivel - MORSE_KOCH_MINIMO) / (MORSE_KOCH_TOTAL - MORSE_KOCH_MINIMO);  /* Décimas */

    if (np != 0U && nr != 0U && punto != 0U)
    {
        if (raya_min <= punto_max)
        {
            return NULL;   /* Alguna "raya" no supera a algún "punto": manipuló otro carácter */
        }
        uint32_t r = 10U * raya * np / (punto * nr);  /* Raya / punto, en décimas */
        if (r < 30U - tol) { return "Rayas cortas"; }
        if (r > 30U + tol) { return "Rayas largas"; }
    }

    uint32_t unidad = (np != 0U) ? punto / np : raya / (3U * nr);
    uint32_t hueco  = 0U;
    for (size_t i = 0U; i + 1U < n; i++)
    {
        hueco = (d->hueco_ms[i] > hueco) ? d->hueco_ms[i] : hueco;
    }
    if (10U * hueco > (10U + tol * 4U / 5U) * (unidad ? unidad : 1U))
    {
        return "Pausas largas";
    }
    return NULL;
}

/* ---------------------------- Progresión Koch --------------------------- */

/**
 * @brief Fija el nivel y vacía la ventana.
 *
 * @return MORSE_Status_t Estado de la operación.
 */
MORSE_Status_t MORSE_KochInit(MORSE_KochProgreso_t *p, uint8_t nivel)
{
    if (p == NULL) { return MORSE_INVALID_PARAM; }

    p->nivel     = morse_RecortarNivel(nivel);
    p->intentos  = 0U;
    p->historial = 0U;
    return MORSE_OK;
}

/**
 * @brief Aciertos dentro de la ventana.
 *
 * @return Número de aciertos.
 */
uint8_t MORSE_KochAciertos(const MORSE_KochProgreso_t *p)
{
    return (uint8_t)__builtin_popcount(p->historial);
}

/**
 * @brief Anota un intento.
 *
 * @return 1 si desbloqueó un carácter, 0 si no.
 */
uint8_t MORSE_KochRegistrar(MORSE_KochProgreso_t *p, uint8_t acierto)
{
    p->historial = ((p->historial << 1) | (acierto != 0U)) & (uint32_t)((1ULL << MORSE_KOCH_VENTANA) - 1U);
    if (p->intentos < MORSE_KOCH_VENTANA)
    {
        p->intentos++;
    }
    if (p->intentos < MORSE_KOCH_VENTANA || MORSE_KochAciertos(p) < MORSE_KOCH_ACIERTOS
        || p->nivel >= MORSE_KOCH_TOTAL)
    {
        return 0U;
    }
    (void)MORSE_KochInit(p, (uint8_t)(p->nivel + 1U));   /* p no es NULL: no puede fallar */
    return 1U;
}

/**
 * @brief Velocidad efectiva (Farnsworth) para un nivel.
 *
 * @return WPM efectivos.
 */
uint8_t MORSE_KochWpmEfectivo(uint8_t nivel, uint8_t inicial, uint8_t caracter)
{
    nivel   = morse_RecortarNivel(nivel);
    inicial = (inicial > caracter) ? caracter : inicial;
    return (uint8_t)(inicial + (uint32_t)(caracter - inicial) * (nivel - MORSE_KOCH_MINIMO)
                               / (MORSE_KOCH_TOTAL - MORSE_KOCH_MINIMO));
}

/* ------------------------- Estadísticas de sesión ----------------------- */

/**
 * @brief Pone a cero las estadísticas.
 *
 * @return MORSE_Status_t Estado de la operación.
 */
MORSE_Status_t MORSE_SesionInit(MORSE_Sesion_t *s)
{
    if (s == NULL) { return MORSE_INVALID_PARAM; }

    memset(s, 0, sizeof *s);
    return MORSE_OK;
}

/**
 * @brief Anota una respuesta.
 *
 * @return MORSE_Status_t Estado de la operación.
 */
MORSE_Status_t MORSE_SesionRegistrar(MORSE_Sesion_t *s, char oido, char respondido,
                                     uint8_t reconocido, uint16_t latencia_ms)
{
    if (s == NULL) { return MORSE_INVALID_PARAM; }

    int i = morse_KochIndice(oido);
    if (i < 0) { return MORSE_INVALID_PARAM; }

    /* Sin saturación: 65 535 intentos son más de 50 h seguidas */
    s->intentos[i]++;
    s->aciertos[i] = (uint16_t)(s->aciertos[i] + (reconocido != 0U));

    int j = morse_KochIndice(respondido);
    if (j >= 0 && j != i && s->confusion[i][j] < UINT8_MAX)
    {
        s->confusion[i][j]++;
    }
    if (latencia_ms != MORSE_SESION_SIN_LATENCIA)
    {
        s->latencia_total_ms += latencia_ms;
        s->latencias++;
    }
    return MORSE_OK;
}

/**
 * @brief Carácter aleatorio ponderado entre los desbloqueados.
 *
 * @return El carácter elegido.
 */
char MORSE_SesionElegir(const MORSE_Sesion_t *s, uint8_t nivel, const char *solo, uint32_t azar)
{
    uint32_t total = 0U;
    for (uint8_t i = 0U; i < nivel; i++)
    {
        total += morse_PesoDe(s, nivel, solo, i);
    }
    if (total == 0U)
    {
        return MORSE_SesionElegir(s, nivel, NULL, azar);   /* 'solo' vacío o sin desbloqueados */
    }
    azar %= total;
    for (uint8_t i = 0U; i < nivel; i++)
    {
        uint32_t w = morse_PesoDe(s, nivel, solo, i);
        if (azar < w)
        {
            return MORSE_KochOrden[i];
        }
        azar -= w;
    }
    return MORSE_KochOrden[nivel - 1U];   /* Inalcanzable */
}

/**
 * @brief Caracteres más fallados, de más a menos.
 *
 * @return Caracteres escritos.
 */
uint8_t MORSE_SesionMasFallados(const MORSE_Sesion_t *s, char *salida, uint8_t max)
{
    /* Orden: fallos descendentes, empate por orden Koch. Cada vuelta busca el
       siguiente tras (antes_f, antes_i), así no hace falta marcar los ya elegidos. */
    uint32_t antes_f = UINT32_MAX;
    int      antes_i = -1;
    uint8_t  k = 0U;
    for (; k < max; k++)
    {
        int      mejor   = -1;
        uint32_t mejor_f = 0U;
        for (int i = 0; i < (int)MORSE_KOCH_TOTAL; i++)
        {
            uint32_t f       = (uint32_t)(s->intentos[i] - s->aciertos[i]);
            uint8_t  despues = (uint8_t)((f < antes_f) || (f == antes_f && i > antes_i));
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
        salida[k] = MORSE_KochOrden[mejor];
        antes_f   = mejor_f;
        antes_i   = mejor;
    }
    salida[k] = '\0';
    return k;
}

/**
 * @brief Pares "oído respondido" más repetidos.
 *
 * @return Pares escritos.
 */
uint8_t MORSE_SesionConfusiones(const MORSE_Sesion_t *s, char *salida, uint8_t max)
{
    /* Mismo recorrido que MORSE_SesionMasFallados sobre la matriz aplanada */
    uint32_t antes_c = UINT32_MAX;
    int      antes_i = -1;
    uint8_t  k = 0U;
    for (; k < max; k++)
    {
        int      mejor   = -1;
        uint32_t mejor_c = 0U;
        for (int i = 0; i < (int)(MORSE_KOCH_TOTAL * MORSE_KOCH_TOTAL); i++)
        {
            uint32_t c       = s->confusion[i / MORSE_KOCH_TOTAL][i % MORSE_KOCH_TOTAL];
            uint8_t  despues = (uint8_t)((c < antes_c) || (c == antes_c && i > antes_i));
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
        salida[2 * k]      = MORSE_KochOrden[mejor / (int)MORSE_KOCH_TOTAL];
        salida[2 * k + 1U] = MORSE_KochOrden[mejor % (int)MORSE_KOCH_TOTAL];
        antes_c = mejor_c;
        antes_i = mejor;
    }
    salida[2 * k] = '\0';
    return k;
}

/**
 * @brief Caracteres para el repaso dirigido.
 *
 * @return Caracteres escritos.
 */
uint8_t MORSE_SesionRepaso(const MORSE_Sesion_t *s, uint8_t nivel, char *salida)
{
    char cand[MORSE_SESION_REPASO_MAX + 1U];        /* 3 fallados + 3 pares */
    (void)MORSE_SesionMasFallados(s, cand, 3U);
    (void)MORSE_SesionConfusiones(s, cand + strlen(cand), 3U);
    uint8_t n = 0U;
    for (const char *c = cand; *c != '\0'; c++)
    {
        int i = morse_KochIndice(*c);               /* Siempre ≥ 0: salen de MORSE_KochOrden */
        if (i >= 0 && i < (int)nivel && memchr(salida, *c, n) == NULL)
        {
            salida[n++] = *c;
        }
    }
    salida[n] = '\0';
    return n;
}
