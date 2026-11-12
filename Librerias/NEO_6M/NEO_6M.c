/**
 * @file NEO_6M.c
 * @brief Implementación de la librería NEO-6M.
 *
 * @author Daniel Ruiz
 * @date Junio 05, 2026
 * @version 1.1.0
 */

#include "NEO_6M.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define NEO_6M_MAX_FIELDS        20U     /**< Campos NMEA máximos por sentencia (GPGGA usa 15) */

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static UART_HandleTypeDef* NEO_6M_huart       = NULL;  /**< Manejador de la interfaz UART utilizada para comunicarse con el módulo GPS */
static uint8_t             NEO_6M_Initialized = 0U;    /**< Bandera para verificar si el módulo está inicializado */

static uint8_t             NEO_6M_RxBuffer[NEO_6M_RX_BUFFER_SIZE] = {0};  /**< Búfer de la sentencia NMEA en recepción */
static uint8_t             NEO_6M_RxIndex     = 0U;    /**< Posición de escritura en el búfer de recepción */
static volatile uint8_t    NEO_6M_RxData;              /**< Escrito por HAL desde el hardware UART */

static NEO_6M_Data_t       NEO_6M_Data        = {0};   /**< Último fix GPS parseado */
static uint32_t            NEO_6M_LastTick    = 0U;    /**< HAL_GetTick() de la última sentencia con checksum válido */
static uint8_t             NEO_6M_HasData     = 0U;    /**< 1 tras la primera sentencia válida desde Init */

// ============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
// ============================================================================

static double  neo_6m_NmeaToDecimal(double coordinate);
static void    neo_6m_ParseUtcTime(const char* timeStr);
static void    neo_6m_UtcToLocalTime(NEO_6M_Data_t* data);
static uint8_t neo_6m_SplitFields(char* s, char* fields[], uint8_t maxFields);
static void    neo_6m_SetPosition(const char* lat, const char* ns, const char* lon, const char* ew);
static void    neo_6m_ParseNmea(char* strParse);
static int     neo_6m_ValidateChecksum(char* nmea);
static int     neo_6m_ValidLatLon(double lat, double lon);

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Convierte una coordenada NMEA (DDmm.mmmm) a grados decimales.
 *
 * @param[in] coordinate Coordenada en formato NMEA (DDmm.mmmm o DDDmm.mmmm).
 * @return double Coordenada en grados decimales (sin signo de hemisferio).
 */
static double neo_6m_NmeaToDecimal(double coordinate)
{
    int    degree  = (int)(coordinate / 100.0);
    double minutes = coordinate - degree * 100.0;
    return degree + (minutes / 60.0);
}

/**
 * @brief Convierte un string de tiempo NMEA "hhmmss.ss" en campo de tiempo NEO_6M_Data.
 *
 * @param[in] timeStr Campo de hora NMEA "hhmmss.ss"; si es NULL o vacío no se modifica nada.
 */
static void neo_6m_ParseUtcTime(const char* timeStr)
{
    /* Campo vacío o truncado: se conserva la hora anterior */
    if (timeStr == NULL || strlen(timeStr) < 6U)
    {
        return;
    }

    char tmp[3] = {0};

    tmp[0] = timeStr[0]; tmp[1] = timeStr[1];
    NEO_6M_Data.hours = (uint8_t)atoi(tmp);

    tmp[0] = timeStr[2]; tmp[1] = timeStr[3];
    NEO_6M_Data.minutes = (uint8_t)atoi(tmp);

    tmp[0] = timeStr[4]; tmp[1] = timeStr[5];
    NEO_6M_Data.seconds = (uint8_t)atoi(tmp);

    if (timeStr[6] == '.' && timeStr[7] != '\0' && timeStr[8] != '\0')
    {
        tmp[0] = timeStr[7]; tmp[1] = timeStr[8];
        NEO_6M_Data.milliseconds = (uint16_t)(atoi(tmp) * 10);
    }
    else
    {
        NEO_6M_Data.milliseconds = 0U;
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
static void neo_6m_UtcToLocalTime(NEO_6M_Data_t* data)
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
 * @brief Divide una sentencia NMEA en campos, in situ.
 *
 * @details Sustituye cada ',' y el '*' por '\0'. A diferencia de sscanf, conserva los
 *          campos vacíos (",,"), que NMEA usa cuando no hay fix.
 *
 * @param[in,out] s         Sentencia NMEA terminada en '\0' (se modifica).
 * @param[out]    fields    Punteros al inicio de cada campo; fields[0] es el identificador ("$GPGGA").
 * @param[in]     maxFields Tamaño de @p fields.
 * @return uint8_t Número de campos encontrados.
 */
static uint8_t neo_6m_SplitFields(char* s, char* fields[], uint8_t maxFields)
{
    uint8_t n = 0U;

    fields[n++] = s;
    for (; *s != '\0' && n < maxFields; s++)
    {
        if (*s == ',' || *s == '*')
        {
            *s = '\0';
            fields[n++] = s + 1;
        }
    }

    return n;
}

/**
 * @brief Actualiza latitud/longitud y hemisferios solo si los campos traen datos.
 *
 * @details Sin fix el módulo envía los campos vacíos; en ese caso se conserva la última
 *          posición conocida en lugar de sobrescribirla con (0, 0).
 */
static void neo_6m_SetPosition(const char* lat, const char* ns, const char* lon, const char* ew)
{
    if (lat[0] == '\0' || ns[0] == '\0' || lon[0] == '\0' || ew[0] == '\0')
    {
        return;
    }

    NEO_6M_Data.northSouth = ns[0];
    NEO_6M_Data.eastWest   = ew[0];
    NEO_6M_Data.latitude   = neo_6m_NmeaToDecimal(strtod(lat, NULL));
    NEO_6M_Data.longitude  = neo_6m_NmeaToDecimal(strtod(lon, NULL));

    if (NEO_6M_Data.northSouth == 'S') NEO_6M_Data.latitude  = -NEO_6M_Data.latitude;
    if (NEO_6M_Data.eastWest   == 'W') NEO_6M_Data.longitude = -NEO_6M_Data.longitude;
}

/**
 * @brief Parsea una sentencia NMEA validada y actualiza NEO_6M_Data.
 *
 * @details Solo se escriben los campos que vienen con datos; los vacíos conservan su
 *          valor anterior. Excepción: @c fixQuality y @c numSatellites se escriben siempre
 *          que llega una GPGGA (vacío = 0), para que una pérdida de fix se refleje.
 *
 * @param[in] strParse Sentencia NMEA ($GPGGA, $GPGLL o $GPRMC) con checksum ya validado.
 */
static void neo_6m_ParseNmea(char* strParse)
{
    char*   f[NEO_6M_MAX_FIELDS];
    uint8_t n = neo_6m_SplitFields(strParse, f, (uint8_t)NEO_6M_MAX_FIELDS);

    /* GPGGA: 1 hora, 2-5 posición, 6 calidad, 7 satélites, 8 HDOP, 9 altitud */
    if (strcmp(f[0], "$GPGGA") == 0 && n >= 10U)
    {
        neo_6m_ParseUtcTime(f[1]);
        neo_6m_SetPosition(f[2], f[3], f[4], f[5]);
        NEO_6M_Data.fixQuality    = (uint8_t)atoi(f[6]);
        NEO_6M_Data.numSatellites = (uint8_t)atoi(f[7]);
        if (f[8][0] != '\0')
        {
            NEO_6M_Data.hdop = (float)strtod(f[8], NULL);
        }
        if (f[9][0] != '\0')
        {
            NEO_6M_Data.altitude = (float)strtod(f[9], NULL);
        }
    }
    /* GPGLL: 1-4 posición, 5 hora */
    else if (strcmp(f[0], "$GPGLL") == 0 && n >= 6U)
    {
        neo_6m_SetPosition(f[1], f[2], f[3], f[4]);
        neo_6m_ParseUtcTime(f[5]);
    }
    /* GPRMC: 1 hora, 2 estado, 3-6 posición, 7 velocidad (nudos), 8 rumbo, 9 fecha DDMMYY */
    else if (strcmp(f[0], "$GPRMC") == 0 && n >= 10U)
    {
        neo_6m_ParseUtcTime(f[1]);
        if (f[2][0] != '\0')
        {
            NEO_6M_Data.posStatus = f[2][0];
        }
        neo_6m_SetPosition(f[3], f[4], f[5], f[6]);
        if (f[7][0] != '\0')
        {
            NEO_6M_Data.speedKmh = (float)(strtod(f[7], NULL) * 1.852);
        }
        if (f[8][0] != '\0')
        {
            NEO_6M_Data.course = (float)strtod(f[8], NULL);
        }
        if (strlen(f[9]) == 6U)
        {
            NEO_6M_Data.day   = (uint8_t)((f[9][0] - '0') * 10 + (f[9][1] - '0'));
            NEO_6M_Data.month = (uint8_t)((f[9][2] - '0') * 10 + (f[9][3] - '0'));
            NEO_6M_Data.year  = (uint8_t)((f[9][4] - '0') * 10 + (f[9][5] - '0'));
        }
    }
    else
    {
        return;
    }

    neo_6m_UtcToLocalTime(&NEO_6M_Data);
}

/**
 * @brief Valida el checksum NMEA (XOR de los bytes entre '$' y '*').
 *
 * @param[in] nmea Sentencia NMEA terminada en '\0' que comienza con '$'.
 * @return int 1 si el checksum coincide, 0 en caso contrario.
 */
static int neo_6m_ValidateChecksum(char* nmea)
{
    char check[3];
    char calculatedString[3];
    int  index           = 0;
    int  calculatedCheck = 0;

    if (nmea[index] == '$')
        index++;
    else
        return 0;

    while ((nmea[index] != 0) && (nmea[index] != '*') && (index < (int)NEO_6M_NMEA_MAX_LEN))
    {
        calculatedCheck ^= nmea[index];
        index++;
    }

    if (index >= (int)NEO_6M_NMEA_MAX_LEN)
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

/**
 * @brief Comprueba que una latitud/longitud está en rango.
 *
 * @return int 1 si es válida, 0 si está fuera de rango o es NaN.
 */
static int neo_6m_ValidLatLon(double lat, double lon)
{
    /* NaN falla todas las comparaciones, así que también se rechaza */
    return (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) ? 1 : 0;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el driver NEO-6M e inicia la recepción por interrupciones UART.
 *
 * @param[in] huart Handle UART de HAL utilizado para comunicarse con el módulo GPS.
 * @return NEO_6M_Status_t
 *         - NEO_6M_OK            si la inicialización fue exitosa.
 *         - NEO_6M_INVALID_PARAM si @p huart es NULL.
 *         - NEO_6M_ERROR         si HAL_UART_Receive_IT falla (el driver queda sin inicializar).
 *
 * @note Llamarlo de nuevo sin DeInit es válido: aborta la recepción previa y reinicia.
 */
NEO_6M_Status_t NEO_6M_Init(UART_HandleTypeDef* huart)
{
    if (huart == NULL)
    {
        return NEO_6M_INVALID_PARAM;
    }

    /* Init doble sin DeInit: la RX anterior sigue armada y Receive_IT devolvería
     * HAL_BUSY. Se aborta antes de tocar el búfer para que el ISR no lo pise. */
    NEO_6M_DeInit();

    NEO_6M_RxIndex = 0U;
    NEO_6M_HasData = 0U;
    memset(&NEO_6M_Data, 0, sizeof(NEO_6M_Data));

    /* El GPS transmite sin parar: si llegaron bytes desde MX_USARTx_UART_Init, ORE
     * ya está activo y la HAL lo trata como error bloqueante (aborta la recepción
     * por IT nada más armarla). Se limpia antes de armar; un ORE posterior lo
     * recupera NEO_6M_UART_ErrorCallback. */
    __HAL_UART_CLEAR_OREFLAG(huart);

    /* El handle se publica antes de armar: el primer byte puede llegar al ISR
     * antes de que Receive_IT retorne. */
    NEO_6M_huart = huart;
    if (HAL_UART_Receive_IT(huart, (uint8_t*)&NEO_6M_RxData, 1U) != HAL_OK)
    {
        NEO_6M_huart = NULL;
        return NEO_6M_ERROR;
    }

    NEO_6M_Initialized = 1U;

    return NEO_6M_OK;
}

/**
 * @brief Desinicializa el driver NEO-6M, aborta la recepción UART y libera los recursos.
 *
 * @return NEO_6M_Status_t Siempre retorna NEO_6M_OK.
 */
NEO_6M_Status_t NEO_6M_DeInit(void)
{
    /* Sin Init previo el handle es NULL y AbortReceive_IT lo desreferencia */
    if (NEO_6M_huart != NULL)
    {
        HAL_UART_AbortReceive_IT(NEO_6M_huart);
    }

    NEO_6M_huart       = NULL;
    NEO_6M_Initialized = 0U;

    return NEO_6M_OK;
}

/**
 * @brief Copia los últimos datos GPS parseados en la estructura del llamador.
 *
 * @details La copia se realiza con interrupciones deshabilitadas para garantizar
 *          coherencia frente a actualizaciones concurrentes desde el ISR de UART.
 *          Se restaura el PRIMASK previo, así que es seguro dentro de una sección crítica.
 *
 * @param[out] data Puntero a una estructura NEO_6M_Data_t a rellenar.
 * @return NEO_6M_Status_t
 *         - NEO_6M_OK              si los datos fueron copiados correctamente.
 *         - NEO_6M_TIMEOUT         si no llega ninguna sentencia válida desde hace más de
 *                                  NEO_6M_DATA_TIMEOUT_MS; @p data se rellena igualmente.
 *         - NEO_6M_NOT_INITIALIZED si el driver no está inicializado.
 *         - NEO_6M_INVALID_PARAM   si @p data es NULL.
 */
NEO_6M_Status_t NEO_6M_Get(NEO_6M_Data_t* data)
{
    if (data == NULL)
    {
        return NEO_6M_INVALID_PARAM;
    }

    if (NEO_6M_Initialized != 1U)
    {
        return NEO_6M_NOT_INITIALIZED;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *data = NEO_6M_Data;
    uint32_t lastTick = NEO_6M_LastTick;
    uint8_t  hasData  = NEO_6M_HasData;
    __set_PRIMASK(primask);

    /* Resta sin signo: correcta aunque HAL_GetTick() desborde (cada ~49 días) */
    if (hasData == 0U || (HAL_GetTick() - lastTick) > NEO_6M_DATA_TIMEOUT_MS)
    {
        return NEO_6M_TIMEOUT;
    }

    return NEO_6M_OK;
}

/**
 * @brief Convierte una coordenada en grados decimales a formato DMS
 *        (Grados, Minutos, Segundos).
 *
 * @param[in]  decimalDegrees Coordenada en grados decimales.
 * @param[in]  hemisphere     Hemisferio de la coordenada: 'N', 'S', 'E' o 'W'.
 * @param[out] dms            Puntero a la estructura NEO_6M_DMS_t a rellenar.
 * @return NEO_6M_Status_t
 *         - NEO_6M_OK            si la conversión fue exitosa.
 *         - NEO_6M_INVALID_PARAM si @p dms es NULL.
 */
NEO_6M_Status_t NEO_6M_DecimalToDMS(double decimalDegrees, char hemisphere, NEO_6M_DMS_t* dms)
{
    double absDegrees    = fabs(decimalDegrees);

    /* Negado para que NaN también se rechace; > 180 desbordaría dms->degrees */
    if (dms == NULL || !(absDegrees <= 180.0))
    {
        return NEO_6M_INVALID_PARAM;
    }

    dms->degrees         = (uint8_t)absDegrees;

    double fractional    = absDegrees - dms->degrees;
    double minutesFloat  = fractional * 60.0;
    dms->minutes         = (uint8_t)minutesFloat;
    dms->seconds         = (uint8_t)((minutesFloat - dms->minutes) * 60.0);
    dms->hemisphere      = hemisphere;

    return NEO_6M_OK;
}

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
NEO_6M_Status_t NEO_6M_DecimalToUTM(double latitude, double longitude, NEO_6M_UTM_t* utm)
{
    /* Negado para que NaN también se rechace */
    if (utm == NULL || !(latitude >= -80.0 && latitude <= 84.0) ||
        !(longitude >= -180.0 && longitude <= 180.0))
    {
        return NEO_6M_INVALID_PARAM;
    }

    /* WGS84 ellipsoid */
    const double a   = 6378137.0;
    const double e2  = 0.00669437999014;
    const double e4  = e2 * e2;
    const double e6  = e4 * e2;
    const double ep2 = e2 / (1.0 - e2);
    const double k0  = 0.9996;

    int zoneNum = (int)((longitude + 180.0) / 6.0) + 1;
    if (zoneNum > 60) zoneNum = 60;   /* longitude == 180.0 pertenece a la zona 60 */

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

    return NEO_6M_OK;
}

/**
 * @brief Distancia sobre la superficie terrestre entre dos puntos (fórmula de haversine).
 *
 * @param[in]  lat1   Latitud del punto 1 en grados decimales.
 * @param[in]  lon1   Longitud del punto 1 en grados decimales.
 * @param[in]  lat2   Latitud del punto 2 en grados decimales.
 * @param[in]  lon2   Longitud del punto 2 en grados decimales.
 * @param[out] meters Distancia en metros.
 * @return NEO_6M_Status_t
 *         - NEO_6M_OK            si el cálculo fue exitoso.
 *         - NEO_6M_INVALID_PARAM si @p meters es NULL o alguna coordenada está fuera de rango.
 */
NEO_6M_Status_t NEO_6M_Distance(double lat1, double lon1, double lat2, double lon2, double* meters)
{
    if (meters == NULL || !neo_6m_ValidLatLon(lat1, lon1) || !neo_6m_ValidLatLon(lat2, lon2))
    {
        return NEO_6M_INVALID_PARAM;
    }

    const double deg2rad = M_PI / 180.0;
    double sinDLat = sin((lat2 - lat1) * deg2rad / 2.0);
    double sinDLon = sin((lon2 - lon1) * deg2rad / 2.0);

    double h = sinDLat * sinDLat +
               cos(lat1 * deg2rad) * cos(lat2 * deg2rad) * sinDLon * sinDLon;

    /* El redondeo puede dejar h un pelo fuera de [0, 1] en puntos antipodales → sqrt(<0) = NaN */
    if (h > 1.0) h = 1.0;

    /* Forma atan2: precisa también a distancias cortas, donde asin pierde resolución */
    *meters = 2.0 * NEO_6M_EARTH_RADIUS_M * atan2(sqrt(h), sqrt(1.0 - h));

    return NEO_6M_OK;
}

/**
 * @brief Rumbo inicial (azimut) del punto 1 al punto 2 sobre el círculo máximo.
 *
 * @param[in]  lat1    Latitud de origen en grados decimales.
 * @param[in]  lon1    Longitud de origen en grados decimales.
 * @param[in]  lat2    Latitud de destino en grados decimales.
 * @param[in]  lon2    Longitud de destino en grados decimales.
 * @param[out] degrees Rumbo en grados, rango [0, 360). 0 si ambos puntos coinciden.
 * @return NEO_6M_Status_t
 *         - NEO_6M_OK            si el cálculo fue exitoso.
 *         - NEO_6M_INVALID_PARAM si @p degrees es NULL o alguna coordenada está fuera de rango.
 */
NEO_6M_Status_t NEO_6M_Bearing(double lat1, double lon1, double lat2, double lon2, double* degrees)
{
    if (degrees == NULL || !neo_6m_ValidLatLon(lat1, lon1) || !neo_6m_ValidLatLon(lat2, lon2))
    {
        return NEO_6M_INVALID_PARAM;
    }

    const double deg2rad = M_PI / 180.0;
    double phi1 = lat1 * deg2rad;
    double phi2 = lat2 * deg2rad;
    double dLon = (lon2 - lon1) * deg2rad;

    double y = sin(dLon) * cos(phi2);
    double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dLon);

    /* atan2 → (-180, 180]; se lleva a [0, 360). atan2(0, 0) = 0 para puntos iguales */
    double b = atan2(y, x) / deg2rad;
    if (b < 0.0) b += 360.0;
    if (b >= 360.0) b = 0.0;   /* -1e-15 + 360 redondea a 360.0 */

    *degrees = b;

    return NEO_6M_OK;
}

/**
 * @brief Manejador de recepción UART completa — llamar desde HAL_UART_RxCpltCallback.
 *
 * @details Acumula bytes en @c NEO_6M_RxBuffer hasta encontrar un salto de línea ('\\n').
 *          Al completar una sentencia, valida el checksum NMEA y, si es correcto,
 *          invoca @c neo_6m_ParseNmea. Rearma automáticamente la interrupción de recepción.
 *
 * @param[in] huart Handle UART de HAL pasado por el callback de HAL.
 */
void NEO_6M_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (NEO_6M_huart == NULL || huart != NEO_6M_huart)
        return;

    if (NEO_6M_RxData != '\n' && NEO_6M_RxIndex < (sizeof(NEO_6M_RxBuffer) - 1U))
    {
        NEO_6M_RxBuffer[NEO_6M_RxIndex++] = NEO_6M_RxData;
    }
    else
    {
        NEO_6M_RxBuffer[NEO_6M_RxIndex] = '\0';

        if (neo_6m_ValidateChecksum((char*)NEO_6M_RxBuffer))
        {
            NEO_6M_LastTick = HAL_GetTick();
            NEO_6M_HasData  = 1U;
            neo_6m_ParseNmea((char*)NEO_6M_RxBuffer);
        }

        NEO_6M_RxIndex = 0U;
    }

    HAL_UART_Receive_IT(NEO_6M_huart, (uint8_t*)&NEO_6M_RxData, 1U);
}

/**
 * @brief Manejador de errores UART — llamar desde HAL_UART_ErrorCallback.
 *
 * @details Descarta la sentencia en curso (ya está corrupta) y, si la HAL abortó la
 *          recepción (ORE), la rearma. Si la HAL sigue recibiendo (ruido/framing),
 *          RxState sigue en BUSY_RX y no se toca.
 *
 * @param[in] huart Handle UART de HAL pasado por el callback de HAL.
 */
void NEO_6M_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (NEO_6M_huart == NULL || huart != NEO_6M_huart)
        return;

    NEO_6M_RxIndex = 0U;

    if (huart->RxState == HAL_UART_STATE_READY)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(huart, (uint8_t*)&NEO_6M_RxData, 1U);
    }
}
