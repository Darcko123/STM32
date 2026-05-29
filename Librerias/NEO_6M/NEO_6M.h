/**
 * @file NEO6M_GPS.h
 * @brief Librería para la gestión de un módulo GPS NEO6M
 *
 * @details Recibe sentencias NMEA ($GPGGA, $GPGLL, $GPRMC) a través de UART mediante
 * recepción controlada por interrupciones. Valida la suma de comprobación y convierte
 * las coordenadas del formato NMEA (DDmm.mmmm) a grados decimales.
 *
 * Protocolo: UART a 9600 baudios, 8N1.
 *
 * @author Daniel Ruiz
 * @date May 29, 2026
 * @version 0.1.0
 */

#ifndef NEO_6M_H
#define NEO_6M_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// MACROS Y CONSTANTES [NEO6M_GPS]
// ============================================================================

#define NEO6M_GPS_RX_BUFFER_SIZE    128U    /**< Tamaño máximo del búfer de sentencia NMEA */
#define NEO6M_GPS_NMEA_MAX_LEN      75U     /**< Longitud máxima válida de la sentencia NMEA */

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del NEO6M_GPS.
 */
typedef enum {
    NEO6M_GPS_OK              = 0,  /**< Operación exitosa */
    NEO6M_GPS_ERROR           = 1,  /**< Error en la operación */
    NEO6M_TIMEOUT             = 2,  /**< Timeoute en la operación */
    NEO6M_GPS_NOT_INITIALIZED = 3,  /**< Módulo no inicializado */
    NEO6M_GPS_INVALID_PARAM   = 4   /**< Parámetro inválido */
} NEO6M_GPS_Status_t;

/**
 * @brief Estructura de datos que almacena el último fix GPS parseado.
 */
typedef struct {
    float latitude;     /**< Latitud en grados decimales          */
    float longitude;    /**< Longitud en grados decimales         */
    char  northSouth;   /**< Indicador de hemisferio: 'N' o 'S'   */
    char  eastWest;     /**< Indicador de hemisferio: 'E' o 'W'   */
    uint8_t  hours;     /**< Hora UTC del fix (GPGGA/GPRMC)       */
    uint8_t  minutes;   /**< Minutos UTC del fix (GPGGA/GPRMC)    */
    uint8_t  seconds;   /**< Segundos UTC del fix (GPGGA/GPRMC)   */
    uint16_t milliseconds; /**< Milisegundos UTC del fix (GPGGA/GPRMC) */
    char  posStatus;    /**< Estado de posición (GPRMC): 'A' = válido, 'V' = vacío */
    float    altitude;  /**< Altitud en metros (GPGGA)            */
    uint8_t fixQuality; /**< Calidad del fix (GPGGA)              */
    uint8_t numSatellites; /**< Número de satélites en vista (GPGGA) */
} NEO6M_GPS_Data_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el driver NEO6M_GPS e inicia la recepción por interrupciones UART.
 *
 * @param[in] huart Handle UART de HAL.
 * @return NEO6M_GPS_Status_t
 *         - NEO6M_GPS_OK            si la inicialización fue exitosa.
 *         - NEO6M_GPS_INVALID_PARAM si @p huart es NULL.
 */
NEO6M_GPS_Status_t NEO6M_GPS_Init(UART_HandleTypeDef* huart);

/**
 * @brief Desinicializa el driver NEO6M_GPS y libera los recursos.
 *
 * @return NEO6M_GPS_Status_t Siempre retorna NEO6M_GPS_OK.
 */
NEO6M_GPS_Status_t NEO6M_GPS_DeInit(void);

/**
 * @brief Copia los últimos datos GPS parseados en la estructura del llamador.
 *
 * @param[out] data Puntero a una estructura NEO6M_GPS_Data_t a rellenar.
 * @return NEO6M_GPS_Status_t
 *         - NEO6M_GPS_OK              si los datos fueron copiados correctamente.
 *         - NEO6M_GPS_NOT_INITIALIZED si el driver no está inicializado.
 *         - NEO6M_GPS_INVALID_PARAM   si @p data es NULL.
 */
NEO6M_GPS_Status_t NEO6M_GPS_Get(NEO6M_GPS_Data_t* data);

/**
 * @brief Manejador de recepción UART completa — llamar desde HAL_UART_RxCpltCallback.
 *
 * @details Acumula bytes hasta encontrar un salto de línea, luego valida y parsea
 *          la sentencia NMEA. Rearma la interrupción automáticamente.
 *
 * @param[in] huart Handle UART de HAL pasado por el callback.
 */
void NEO6M_GPS_UART_RxCpltCallback(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif /* NEO6M_GPS_H */
