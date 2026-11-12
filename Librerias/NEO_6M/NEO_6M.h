/**
 * @file NEO_6M.h
 * @brief Librería para la gestión de un módulo GPS NEO6M
 *
 * @details Recibe sentencias NMEA ($GPGGA, $GPGLL, $GPRMC) a través de UART mediante
 * recepción controlada por interrupciones. Valida la suma de comprobación y convierte
 * las coordenadas del formato NMEA (DDmm.mmmm) a grados decimales.
 *
 * Protocolo: UART a 9600 baudios, 8N1.
 *
 * @author Daniel Ruiz
 * @date Junio 05, 2026
 * @version 1.1.0
 */

#ifndef NEO_6M_H
#define NEO_6M_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdint.h>

// ============================================================================
// MACROS Y CONSTANTES [NEO_6M]
// ============================================================================

#define NEO_6M_RX_BUFFER_SIZE    128U    /**< Tamaño máximo del búfer de sentencia NMEA */
#define NEO_6M_NMEA_MAX_LEN      82U     /**< Longitud máxima de una sentencia NMEA 0183 (incluye '$' y CRLF) */

#ifndef NEO_6M_DATA_TIMEOUT_MS
#define NEO_6M_DATA_TIMEOUT_MS   3000U   /**< Sin sentencias válidas durante este tiempo, Get devuelve NEO_6M_TIMEOUT (el módulo emite a 1 Hz) */
#endif

#ifndef NEO_6M_EARTH_RADIUS_M
#define NEO_6M_EARTH_RADIUS_M    6371008.8  /**< Radio medio terrestre (IUGG) usado por NEO_6M_Distance, en metros */
#endif

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del NEO-6M.
 */
typedef enum {
    NEO_6M_OK              = 0,  /**< Operación exitosa */
    NEO_6M_ERROR           = 1,  /**< Error en la operación */
    NEO_6M_TIMEOUT         = 2,  /**< Timeout en la operación */
    NEO_6M_NOT_INITIALIZED = 3,  /**< Módulo no inicializado */
    NEO_6M_INVALID_PARAM   = 4   /**< Parámetro inválido */
} NEO_6M_Status_t;

/**
 * @brief Estructura de datos que almacena el último fix GPS parseado.
 */
typedef struct {
    double latitude;    /**< Latitud en grados decimales          */
    double longitude;   /**< Longitud en grados decimales         */
    char  northSouth;   /**< Indicador de hemisferio: 'N' o 'S'   */
    char  eastWest;     /**< Indicador de hemisferio: 'E' o 'W'   */
    uint8_t  hours;     /**< Hora UTC del fix (GPGGA/GPRMC)       */
    uint8_t  minutes;   /**< Minutos UTC del fix (GPGGA/GPRMC)    */
    uint8_t  seconds;   /**< Segundos UTC del fix (GPGGA/GPRMC)   */
    uint16_t milliseconds; /**< Milisegundos UTC del fix (GPGGA/GPRMC) */
    uint8_t  localHour; /**< Hora local calculada a partir de UTC y longitud */
    char  posStatus;    /**< Estado de posición (GPRMC): 'A' = válido, 'V' = vacío */
    float    altitude;  /**< Altitud en metros (GPGGA)            */
    uint8_t fixQuality; /**< Calidad del fix (GPGGA)              */
    uint8_t numSatellites; /**< Número de satélites en vista (GPGGA) */
    float    hdop;      /**< Dilución horizontal de precisión (GPGGA); menor = mejor */
    float    speedKmh;  /**< Velocidad sobre el suelo en km/h (GPRMC, nudos × 1.852) */
    float    course;    /**< Rumbo sobre el suelo en grados, 0–360 (GPRMC); solo fiable en movimiento */
    uint8_t  day;       /**< Día UTC del fix (GPRMC)              */
    uint8_t  month;     /**< Mes UTC del fix (GPRMC)              */
    uint8_t  year;      /**< Año UTC del fix, dos dígitos: 26 = 2026 (GPRMC) */
} NEO_6M_Data_t;

/**
 * @brief Estructura que almacena una coordenada en formato DMS (Grados, Minutos, Segundos).
 */
typedef struct {
    uint8_t degrees;    /**< Grados enteros de la coordenada                */
    uint8_t minutes;    /**< Minutos enteros de la coordenada               */
    uint8_t seconds;    /**< Segundos enteros de la coordenada              */
    char    hemisphere; /**< Hemisferio de la coordenada: 'N', 'S', 'E' o 'W' */
} NEO_6M_DMS_t;

/**
 * @brief Estructura que almacena una coordenada en formato UTM
 *        (proyección Universal Transversa de Mercator).
 */
typedef struct {
    uint32_t easting;    /**< Coordenada Este en metros                     */
    uint32_t northing;   /**< Coordenada Norte en metros                    */
    char     zone;       /**< Letra de banda de latitud UTM (ej: 'T')       */
    uint16_t zoneNumber; /**< Número de zona UTM (ej: 30)                   */
} NEO_6M_UTM_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el driver NEO-6M e inicia la recepción por interrupciones UART.
 *
 * @param[in] huart Handle UART de HAL.
 * @return NEO_6M_Status_t
 *         - NEO_6M_OK            si la inicialización fue exitosa.
 *         - NEO_6M_INVALID_PARAM si @p huart es NULL.
 *         - NEO_6M_ERROR         si HAL_UART_Receive_IT falla (el driver queda sin inicializar).
 *
 * @note Llamarlo de nuevo sin DeInit es válido: aborta la recepción previa y reinicia.
 */
NEO_6M_Status_t NEO_6M_Init(UART_HandleTypeDef* huart);

/**
 * @brief Desinicializa el driver NEO-6M y libera los recursos.
 *
 * @return NEO_6M_Status_t Siempre retorna NEO_6M_OK.
 */
NEO_6M_Status_t NEO_6M_DeInit(void);

/**
 * @brief Copia los últimos datos GPS parseados en la estructura del llamador.
 *
 * @details Sin fix, la posición conserva el último valor conocido: comprobar
 *          @c fixQuality (GPGGA) o @c posStatus (GPRMC) para saber si es actual.
 *
 * @param[out] data Puntero a una estructura NEO_6M_Data_t a rellenar.
 * @return NEO_6M_Status_t
 *         - NEO_6M_OK              si los datos fueron copiados correctamente.
 *         - NEO_6M_TIMEOUT         si no llega ninguna sentencia válida desde hace más de
 *                                  NEO_6M_DATA_TIMEOUT_MS (o aún no ha llegado ninguna).
 *                                  @p data se rellena igualmente con los últimos datos.
 *         - NEO_6M_NOT_INITIALIZED si el driver no está inicializado.
 *         - NEO_6M_INVALID_PARAM   si @p data es NULL.
 */
NEO_6M_Status_t NEO_6M_Get(NEO_6M_Data_t* data);

/**
 * @brief Convierte una coordenada en grados decimales a formato DMS
 *        (Grados, Minutos, Segundos).
 *
 * @param[in]  decimalDegrees Coordenada en grados decimales.
 * @param[in]  hemisphere     Hemisferio de la coordenada: 'N', 'S', 'E' o 'W'.
 * @param[out] dms            Puntero a la estructura NEO_6M_DMS_t a rellenar.
 * @return NEO_6M_Status_t
 *         - NEO_6M_OK            si la conversión fue exitosa.
 *         - NEO_6M_INVALID_PARAM si @p dms es NULL o |@p decimalDegrees| > 180 (o NaN).
 */
NEO_6M_Status_t NEO_6M_DecimalToDMS(double decimalDegrees, char hemisphere, NEO_6M_DMS_t* dms);

/**
 * @brief Convierte latitud/longitud decimal a UTM
 *        (proyección Universal Transversa de Mercator).
 *
 * @warning Requiere matemática de punto flotante y trigonometría.
 *
 * @param[in]  latitude  Latitud en grados decimales (rango válido: -80.0 a 84.0).
 * @param[in]  longitude Longitud en grados decimales (rango válido: -180.0 a 180.0).
 * @param[out] utm       Puntero a la estructura NEO_6M_UTM_t a rellenar.
 * @return NEO_6M_Status_t
 *         - NEO_6M_OK            si la conversión fue exitosa.
 *         - NEO_6M_INVALID_PARAM si @p utm es NULL o las coordenadas están fuera de rango.
 */
NEO_6M_Status_t NEO_6M_DecimalToUTM(double latitude, double longitude, NEO_6M_UTM_t* utm);

/**
 * @brief Distancia sobre la superficie terrestre entre dos puntos (fórmula de haversine).
 *
 * @details Modelo de Tierra esférica de radio NEO_6M_EARTH_RADIUS_M: error < 0.5 % frente
 *          al elipsoide WGS84, muy por debajo del error de posición del propio GPS.
 *
 * @param[in]  lat1   Latitud del punto 1 en grados decimales (-90.0 a 90.0).
 * @param[in]  lon1   Longitud del punto 1 en grados decimales (-180.0 a 180.0).
 * @param[in]  lat2   Latitud del punto 2 en grados decimales (-90.0 a 90.0).
 * @param[in]  lon2   Longitud del punto 2 en grados decimales (-180.0 a 180.0).
 * @param[out] meters Distancia en metros.
 * @return NEO_6M_Status_t
 *         - NEO_6M_OK            si el cálculo fue exitoso.
 *         - NEO_6M_INVALID_PARAM si @p meters es NULL o alguna coordenada está fuera de rango (o NaN).
 */
NEO_6M_Status_t NEO_6M_Distance(double lat1, double lon1, double lat2, double lon2, double* meters);

/**
 * @brief Rumbo inicial (azimut) del punto 1 al punto 2 sobre el círculo máximo.
 *
 * @details 0° = Norte, 90° = Este, en sentido horario. Es el rumbo de salida: en
 *          trayectos largos el rumbo cambia a lo largo del camino. Si ambos puntos
 *          coinciden el rumbo no está definido y se devuelve 0.
 *
 * @param[in]  lat1    Latitud de origen en grados decimales (-90.0 a 90.0).
 * @param[in]  lon1    Longitud de origen en grados decimales (-180.0 a 180.0).
 * @param[in]  lat2    Latitud de destino en grados decimales (-90.0 a 90.0).
 * @param[in]  lon2    Longitud de destino en grados decimales (-180.0 a 180.0).
 * @param[out] degrees Rumbo en grados, rango [0, 360).
 * @return NEO_6M_Status_t
 *         - NEO_6M_OK            si el cálculo fue exitoso.
 *         - NEO_6M_INVALID_PARAM si @p degrees es NULL o alguna coordenada está fuera de rango (o NaN).
 */
NEO_6M_Status_t NEO_6M_Bearing(double lat1, double lon1, double lat2, double lon2, double* degrees);

/**
 * @brief Manejador de recepción UART completa — llamar desde HAL_UART_RxCpltCallback.
 *
 * @details Acumula bytes hasta encontrar un salto de línea, luego valida y parsea
 *          la sentencia NMEA. Rearma la interrupción automáticamente.
 *
 * @param[in] huart Handle UART de HAL pasado por el callback.
 */
void NEO_6M_UART_RxCpltCallback(UART_HandleTypeDef* huart);

/**
 * @brief Manejador de errores UART — llamar desde HAL_UART_ErrorCallback.
 *
 * @details Un overrun (ORE) hace que la HAL aborte la recepción por IT. Esta función
 *          descarta la sentencia en curso y rearma la recepción. Con errores no
 *          bloqueantes (ruido, framing) la HAL sigue recibiendo y solo se descarta la sentencia.
 *
 * @param[in] huart Handle UART de HAL pasado por el callback.
 */
void NEO_6M_UART_ErrorCallback(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif /* NEO_6M_H */
