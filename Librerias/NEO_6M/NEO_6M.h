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
 * @date Jun 1, 2026
 * @version 1.0.0
 */

#ifndef NEO_6M_H
#define NEO_6M_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

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
} NEO6M_GPS_Data_t;

typedef struct {
    uint8_t degrees;
    uint8_t minutes;
    uint8_t seconds;
    char    hemisphere;  // 'N','S','E','W'
} NEO6M_GPS_DMS_t;

typedef struct {
    uint32_t easting;    // metros
    uint32_t northing;   // metros
    char     zone;       // zona UTM (ej: 'T')
    uint16_t zoneNumber; // número de zona (ej: 30)
} NEO6M_GPS_UTM_t;

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
 * @brief Convierte latitud/longitud decimal a DMS (Grados, Minutos, Segundos)
 */
NEO6M_GPS_Status_t NEO6M_GPS_DecimalToDMS(double decimalDegrees, NEO6M_GPS_DMS_t* dms);

/**
 * @brief Convierte latitud/longitud decimal a UTM (proyección Universal Transversa de Mercator)
 * @warning Requiere matemática de punto flotante y trigonometría.
 */
NEO6M_GPS_Status_t NEO6M_GPS_DecimalToUTM(double latitude, double longitude, NEO6M_GPS_UTM_t* utm);

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
