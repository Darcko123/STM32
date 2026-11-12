/**
 * @file NEO_6M.c
 * @brief Implementación de la librería NEO6M_GPS.
 *
 * @author Daniel Ruiz
 * @date Junio 05, 2026
 * @version 1.1.0
 */

#include "NEO_6M.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static UART_HandleTypeDef* NEO6M_GPS_huart       = NULL;  /**< Manejador de la interfaz UART utilizada para comunicarse con el módulo GPS */
static uint8_t             NEO6M_GPS_Initialized = 0U;    /**< Bandera para verificar si el módulo está inicializado */

static uint8_t             NEO6M_GPS_RxBuffer[NEO6M_GPS_RX_BUFFER_SIZE] = {0};  /**< Búfer de la sentencia NMEA en recepción */
static uint8_t             NEO6M_GPS_RxIndex     = 0U;    /**< Posición de escritura en el búfer de recepción */
static volatile uint8_t    NEO6M_GPS_RxData;              /**< Escrito por HAL desde el hardware UART */

static NEO6M_GPS_Data_t    NEO6M_GPS_Data        = {0};   /**< Último fix GPS parseado */

// ============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
// ============================================================================

static double neo6m_NmeaToDecimal(double coordinate);
static void   neo6m_ParseUtcTime(const char* timeStr);
static void   neo6m_UtcToLocalTime(NEO6M_GPS_Data_t* data);
static void   neo6m_ParseNmea(char* strParse);
static int    neo6m_ValidateChecksum(char* nmea);

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Convierte una coordenada NMEA (DDmm.mmmm) a grados decimales.
 *
 * @param[in] coordinate Coordenada en formato NMEA (DDmm.mmmm o DDDmm.mmmm).
 * @return double Coordenada en grados decimales (sin signo de hemisferio).
 */
static double neo6m_NmeaToDecimal(double coordinate)
{
    int    degree  = (int)(coordinate / 100.0);
    double minutes = coordinate - degree * 100.0;
    return degree + (minutes / 60.0);
}

/**
 * @brief Convierte un string de tiempo NMEA "hhmmss.ss" en campo de tiempo NEO6M_GPS_Data.
 *
 * @param[in] timeStr Campo de hora NMEA "hhmmss.ss"; si es NULL o vacío no se modifica nada.
 */
static void neo6m_ParseUtcTime(const char* timeStr)
{
    if (timeStr == NULL || timeStr[0] == '\0')
    {
        return;
    }

    char tmp[3] = {0};

    tmp[0] = timeStr[0]; tmp[1] = timeStr[1];
    NEO6M_GPS_Data.hours = (uint8_t)atoi(tmp);

    tmp[0] = timeStr[2]; tmp[1] = timeStr[3];
    NEO6M_GPS_Data.minutes = (uint8_t)atoi(tmp);

    tmp[0] = timeStr[4]; tmp[1] = timeStr[5];
    NEO6M_GPS_Data.seconds = (uint8_t)atoi(tmp);

    if (timeStr[6] == '.' && timeStr[7] != '\0' && timeStr[8] != '\0')
    {
        tmp[0] = timeStr[7]; tmp[1] = timeStr[8];
        NEO6M_GPS_Data.milliseconds = (uint16_t)(atoi(tmp) * 10);
    }
    else
    {
        NEO6M_GPS_Data.milliseconds = 0U;
    }
}

/**
 * @brief Calcula la hora local a partir de la hora UTC y la longitud GPS.
 *
 * @details Estima el huso horario dividiendo la longitud entre 15°. El resultado
 *          es una aproximación válida en ausencia de datos de zona horaria civil.
 *
 * @param[in,out] data Puntero a la estructura GPS. Se leen los campos @c longitude,
 *                     @c eastWest y @c hours, y se escribe el campo @c localHour.
 */
static void neo6m_UtcToLocalTime(NEO6M_GPS_Data_t* data)
{
    if (data == NULL)
    {
        return;
    }

    int husoHorario = (int)(fabs(data->longitude) / 15.0);
    if (data->longitude < 0.0) husoHorario = -husoHorario;

    int localHour = (data->hours + husoHorario) % 24;

    if (localHour < 0)
    {
        localHour += 24;
    }

    data->localHour = (uint8_t)localHour;
}

/**
 * @brief Parsea una sentencia NMEA validada y actualiza NEO6M_GPS_Data.
 *
 * @param[in] strParse Sentencia NMEA ($GPGGA, $GPGLL o $GPRMC) con checksum ya validado.
 */
static void neo6m_ParseNmea(char* strParse)
{
    char   timeStr[12] = {0};
    double nmeaLat     = 0.0;
    double nmeaLong    = 0.0;
    int    fixQ        = 0;
    int    numSat      = 0;

    if (!strncmp(strParse, "$GPGGA", 6))
    {
        sscanf(strParse, "$GPGGA,%11[^,],%lf,%c,%lf,%c,%d,%d,%*f,%f",
               timeStr, &nmeaLat, &NEO6M_GPS_Data.northSouth,
               &nmeaLong, &NEO6M_GPS_Data.eastWest,
               &fixQ, &numSat, &NEO6M_GPS_Data.altitude);

        NEO6M_GPS_Data.fixQuality    = (uint8_t)fixQ;
        NEO6M_GPS_Data.numSatellites = (uint8_t)numSat;
        neo6m_ParseUtcTime(timeStr);
    }
    else if (!strncmp(strParse, "$GPGLL", 6))
    {
        sscanf(strParse, "$GPGLL,%lf,%c,%lf,%c,%11[^,]",
               &nmeaLat, &NEO6M_GPS_Data.northSouth,
               &nmeaLong, &NEO6M_GPS_Data.eastWest, timeStr);

        neo6m_ParseUtcTime(timeStr);
    }
    else if (!strncmp(strParse, "$GPRMC", 6))
    {
        sscanf(strParse, "$GPRMC,%11[^,],%c,%lf,%c,%lf,%c",
               timeStr, &NEO6M_GPS_Data.posStatus,
               &nmeaLat, &NEO6M_GPS_Data.northSouth,
               &nmeaLong, &NEO6M_GPS_Data.eastWest);

        neo6m_ParseUtcTime(timeStr);
    }
    else
    {
        return;
    }

    NEO6M_GPS_Data.latitude  = neo6m_NmeaToDecimal(nmeaLat);
    NEO6M_GPS_Data.longitude = neo6m_NmeaToDecimal(nmeaLong);

    if (NEO6M_GPS_Data.northSouth == 'S') NEO6M_GPS_Data.latitude  = -NEO6M_GPS_Data.latitude;
    if (NEO6M_GPS_Data.eastWest   == 'W') NEO6M_GPS_Data.longitude = -NEO6M_GPS_Data.longitude;

    neo6m_UtcToLocalTime(&NEO6M_GPS_Data);
}

/**
 * @brief Valida el checksum NMEA (XOR de los bytes entre '$' y '*').
 *
 * @param[in] nmea Sentencia NMEA terminada en '\0' que comienza con '$'.
 * @return int 1 si el checksum coincide, 0 en caso contrario.
 */
static int neo6m_ValidateChecksum(char* nmea)
{
    char check[3];
    char calculatedString[3];
    int  index           = 0;
    int  calculatedCheck = 0;

    if (nmea[index] == '$')
        index++;
    else
        return 0;

    while ((nmea[index] != 0) && (nmea[index] != '*') && (index < (int)NEO6M_GPS_NMEA_MAX_LEN))
    {
        calculatedCheck ^= nmea[index];
        index++;
    }

    if (index >= (int)NEO6M_GPS_NMEA_MAX_LEN)
        return 0;

    if (nmea[index] == '*')
    {
        check[0] = nmea[index + 1];
        check[1] = nmea[index + 2];
        check[2] = '\0';
    }
    else
        return 0;

    snprintf(calculatedString, sizeof(calculatedString), "%02X", calculatedCheck);
    return ((calculatedString[0] == check[0]) && (calculatedString[1] == check[1])) ? 1 : 0;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el driver NEO6M_GPS e inicia la recepción por interrupciones UART.
 *
 * @param[in] huart Handle UART de HAL utilizado para comunicarse con el módulo GPS.
 * @return NEO6M_GPS_Status_t
 *         - NEO6M_GPS_OK            si la inicialización fue exitosa.
 *         - NEO6M_GPS_INVALID_PARAM si @p huart es NULL.
 */
NEO6M_GPS_Status_t NEO6M_GPS_Init(UART_HandleTypeDef* huart)
{
    if (huart == NULL)
    {
        return NEO6M_GPS_INVALID_PARAM;
    }

    NEO6M_GPS_huart = huart;

    NEO6M_GPS_RxIndex = 0U;
    memset(NEO6M_GPS_RxBuffer, 0, sizeof(NEO6M_GPS_RxBuffer));
    memset(&NEO6M_GPS_Data, 0, sizeof(NEO6M_GPS_Data));

    /* El GPS transmite sin parar: si llegaron bytes desde MX_USARTx_UART_Init, ORE
     * ya está activo y la HAL lo trata como error bloqueante (aborta la recepción
     * por IT nada más armarla). Se limpia antes de armar. */
    // ponytail: sin HAL_UART_ErrorCallback, un ORE en marcha (IRQs bloqueadas > 2 ms) detiene la RX; añadir un NEO6M_GPS_UART_ErrorCallback que rearme si hace falta
    __HAL_UART_CLEAR_OREFLAG(NEO6M_GPS_huart);
    HAL_UART_Receive_IT(NEO6M_GPS_huart, (uint8_t*)&NEO6M_GPS_RxData, 1U);

    NEO6M_GPS_Initialized = 1U;

    return NEO6M_GPS_OK;
}

/**
 * @brief Desinicializa el driver NEO6M_GPS, aborta la recepción UART y libera los recursos.
 *
 * @return NEO6M_GPS_Status_t Siempre retorna NEO6M_GPS_OK.
 */
NEO6M_GPS_Status_t NEO6M_GPS_DeInit(void)
{
    /* Sin Init previo el handle es NULL y AbortReceive_IT lo desreferencia */
    if (NEO6M_GPS_huart != NULL)
    {
        HAL_UART_AbortReceive_IT(NEO6M_GPS_huart);
    }

    NEO6M_GPS_huart       = NULL;
    NEO6M_GPS_Initialized = 0U;

    return NEO6M_GPS_OK;
}

/**
 * @brief Copia los últimos datos GPS parseados en la estructura del llamador.
 *
 * @details La copia se realiza con interrupciones deshabilitadas para garantizar
 *          coherencia frente a actualizaciones concurrentes desde el ISR de UART.
 *
 * @param[out] data Puntero a una estructura NEO6M_GPS_Data_t a rellenar.
 * @return NEO6M_GPS_Status_t
 *         - NEO6M_GPS_OK              si los datos fueron copiados correctamente.
 *         - NEO6M_GPS_NOT_INITIALIZED si el driver no está inicializado.
 *         - NEO6M_GPS_INVALID_PARAM   si @p data es NULL.
 */
NEO6M_GPS_Status_t NEO6M_GPS_Get(NEO6M_GPS_Data_t* data)
{
    if (data == NULL)
    {
        return NEO6M_GPS_INVALID_PARAM;
    }

    if (NEO6M_GPS_Initialized != 1U)
    {
        return NEO6M_GPS_NOT_INITIALIZED;
    }

    __disable_irq();
    *data = NEO6M_GPS_Data;
    __enable_irq();

    return NEO6M_GPS_OK;
}

/**
 * @brief Convierte una coordenada en grados decimales a formato DMS
 *        (Grados, Minutos, Segundos).
 *
 * @param[in]  decimalDegrees Coordenada en grados decimales.
 * @param[in]  hemisphere     Hemisferio de la coordenada: 'N', 'S', 'E' o 'W'.
 * @param[out] dms            Puntero a la estructura NEO6M_GPS_DMS_t a rellenar.
 * @return NEO6M_GPS_Status_t
 *         - NEO6M_GPS_OK            si la conversión fue exitosa.
 *         - NEO6M_GPS_INVALID_PARAM si @p dms es NULL.
 */
NEO6M_GPS_Status_t NEO6M_GPS_DecimalToDMS(double decimalDegrees, char hemisphere, NEO6M_GPS_DMS_t* dms)
{
    if (dms == NULL)
    {
        return NEO6M_GPS_INVALID_PARAM;
    }

    double absDegrees    = fabs(decimalDegrees);
    dms->degrees         = (uint8_t)absDegrees;

    double fractional    = absDegrees - dms->degrees;
    double minutesFloat  = fractional * 60.0;
    dms->minutes         = (uint8_t)minutesFloat;
    dms->seconds         = (uint8_t)((minutesFloat - dms->minutes) * 60.0);
    dms->hemisphere      = hemisphere;

    return NEO6M_GPS_OK;
}

/**
 * @brief Convierte latitud/longitud decimal a UTM
 *        (proyección Universal Transversa de Mercator).
 *
 * @warning Requiere matemática de punto flotante y trigonometría.
 *
 * @param[in]  latitude  Latitud en grados decimales (rango válido: -80.0 a 84.0).
 * @param[in]  longitude Longitud en grados decimales (rango válido: -180.0 a 180.0).
 * @param[out] utm       Puntero a la estructura NEO6M_GPS_UTM_t a rellenar.
 * @return NEO6M_GPS_Status_t
 *         - NEO6M_GPS_OK            si la conversión fue exitosa.
 *         - NEO6M_GPS_INVALID_PARAM si @p utm es NULL o las coordenadas están fuera de rango.
 */
NEO6M_GPS_Status_t NEO6M_GPS_DecimalToUTM(double latitude, double longitude, NEO6M_GPS_UTM_t* utm)
{
    if (utm == NULL || latitude < -80.0 || latitude > 84.0 ||
        longitude < -180.0 || longitude > 180.0)
    {
        return NEO6M_GPS_INVALID_PARAM;
    }

    /* WGS84 ellipsoid */
    const double a   = 6378137.0;
    const double e2  = 0.00669437999014;
    const double e4  = e2 * e2;
    const double e6  = e4 * e2;
    const double ep2 = e2 / (1.0 - e2);
    const double k0  = 0.9996;

    int zoneNum = (int)((longitude + 180.0) / 6.0) + 1;

    /* Excepciones de zona: Noruega y Svalbard (estándar UTM) */
    if (latitude >= 56.0 && latitude < 64.0 && longitude >= 3.0 && longitude < 12.0)
        zoneNum = 32;
    if (latitude >= 72.0 && latitude < 84.0)
    {
        if      (longitude >= 0.0  && longitude <  9.0) zoneNum = 31;
        else if (longitude >= 9.0  && longitude < 21.0) zoneNum = 33;
        else if (longitude >= 21.0 && longitude < 33.0) zoneNum = 35;
        else if (longitude >= 33.0 && longitude < 42.0) zoneNum = 37;
    }

    double lonOrigin = (double)((zoneNum - 1) * 6 - 180 + 3);
    double latRad    = latitude  * (M_PI / 180.0);
    double lonDiff   = (longitude - lonOrigin) * (M_PI / 180.0);

    double sinLat  = sin(latRad);
    double cosLat  = cos(latRad);
    double tanLat  = tan(latRad);
    double sin2Lat = sinLat * sinLat;

    double N  = a / sqrt(1.0 - e2 * sin2Lat);
    double T  = tanLat * tanLat;
    double C  = ep2 * cosLat * cosLat;
    double A  = cosLat * lonDiff;
    double A2 = A * A;
    double A3 = A2 * A;
    double A4 = A3 * A;
    double A5 = A4 * A;
    double A6 = A5 * A;

    double M = a * (
        (1.0 - e2/4.0   - 3.0*e4/64.0   - 5.0*e6/256.0)   * latRad
      - (3.0*e2/8.0 + 3.0*e4/32.0 + 45.0*e6/1024.0)        * sin(2.0 * latRad)
      + (15.0*e4/256.0 + 45.0*e6/1024.0)                    * sin(4.0 * latRad)
      - (35.0*e6/3072.0)                                     * sin(6.0 * latRad));

    double easting = k0 * N * (A
        + (1.0 - T + C)                           * A3 / 6.0
        + (5.0 - 18.0*T + T*T + 72.0*C - 58.0*ep2) * A5 / 120.0)
        + 500000.0;

    double northing = k0 * (M + N * tanLat * (A2 / 2.0
        + (5.0 - T + 9.0*C + 4.0*C*C)              * A4 / 24.0
        + (61.0 - 58.0*T + T*T + 600.0*C - 330.0*ep2) * A6 / 720.0));

    if (latitude < 0.0)
        northing += 10000000.0;   /* desplazamiento falso para hemisferio sur */

    /* Letra de banda de latitud (C…X, sin I ni O) */
    const char bands[] = "CDEFGHJKLMNPQRSTUVWX";
    int idx = (int)((latitude + 80.0) / 8.0);
    if (idx < 0)  idx = 0;
    if (idx > 19) idx = 19;

    utm->easting    = (uint32_t)(easting  + 0.5);
    utm->northing   = (uint32_t)(northing + 0.5);
    utm->zone       = bands[idx];
    utm->zoneNumber = (uint16_t)zoneNum;

    return NEO6M_GPS_OK;
}

/**
 * @brief Manejador de recepción UART completa — llamar desde HAL_UART_RxCpltCallback.
 *
 * @details Acumula bytes en @c NEO6M_GPS_RxBuffer hasta encontrar un salto de línea ('\\n').
 *          Al completar una sentencia, valida el checksum NMEA y, si es correcto,
 *          invoca @c neo6m_ParseNmea. Rearma automáticamente la interrupción de recepción.
 *
 * @param[in] huart Handle UART de HAL pasado por el callback de HAL.
 */
void NEO6M_GPS_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (NEO6M_GPS_huart == NULL || huart != NEO6M_GPS_huart)
        return;

    if (NEO6M_GPS_RxData != '\n' && NEO6M_GPS_RxIndex < (sizeof(NEO6M_GPS_RxBuffer) - 1U))
    {
        NEO6M_GPS_RxBuffer[NEO6M_GPS_RxIndex++] = NEO6M_GPS_RxData;
    }
    else
    {
        if (neo6m_ValidateChecksum((char*)NEO6M_GPS_RxBuffer))
            neo6m_ParseNmea((char*)NEO6M_GPS_RxBuffer);

        NEO6M_GPS_RxIndex = 0U;
        memset(NEO6M_GPS_RxBuffer, 0, sizeof(NEO6M_GPS_RxBuffer));
    }

    HAL_UART_Receive_IT(NEO6M_GPS_huart, (uint8_t*)&NEO6M_GPS_RxData, 1U);
}
