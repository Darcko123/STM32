/**
 * @file NEO6M_GPS.c
 * @brief Implementación de la librería NEO6M_GPS.
 *
 * @author Daniel Ruiz
 * @date May 29, 2026
 * @version 0.1.0
 */

#include "NEO_6M.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static UART_HandleTypeDef* NEO6M_GPS_huart = NULL;  /**< Manejador de la interfaz UART utilizada para comunicarse con el módulo GPS */
static uint8_t       NEO6M_GPS_Initialized = 0U;    /**< Bandera para verificar si el módulo está inicializado */

static uint8_t rxBuffer[NEO6M_GPS_RX_BUFFER_SIZE] = {0};
static uint8_t rxIndex = 0U;
static uint8_t rxData;

static NEO6M_GPS_Data_t gpsData = {0};

// ============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
// ============================================================================

static double nmeaToDecimal(double coordinate);
static void   parseUtcTime(const char* timeStr);
static void   gpsParse(char* strParse);
static int    gpsValidate(char* nmea);

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Convierte una coordenada NMEA (DDmm.mmmm) a grados decimales.
 */
static double nmeaToDecimal(double coordinate)
{
    int    degree  = (int)(coordinate / 100.0);
    double minutes = coordinate - degree * 100.0;
    return degree + (minutes / 60.0);
}

/**
 * @brief Convierte un string de tiempo NMEA "hhmmss.ss" en campo de tiempo gpsData.
 */
static void parseUtcTime(const char* timeStr)
{
    if (timeStr == NULL || timeStr[0] == '\0')
    {
        return;
    }

    char tmp[3] = {0};

    tmp[0] = timeStr[0]; tmp[1] = timeStr[1];
    gpsData.hours = (uint8_t)atoi(tmp);

    tmp[0] = timeStr[2]; tmp[1] = timeStr[3];
    gpsData.minutes = (uint8_t)atoi(tmp);

    tmp[0] = timeStr[4]; tmp[1] = timeStr[5];
    gpsData.seconds = (uint8_t)atoi(tmp);

    if (timeStr[6] == '.' && timeStr[7] != '\0' && timeStr[8] != '\0')
    {
        tmp[0] = timeStr[7]; tmp[1] = timeStr[8];
        gpsData.milliseconds = (uint16_t)(atoi(tmp) * 10);
    }
    else
    {
        gpsData.milliseconds = 0U;
    }
}

/**
 * @brief Parsea una sentencia NMEA validada y actualiza gpsData.
 */
static void gpsParse(char* strParse)
{
    char   timeStr[12] = {0};
    double nmeaLat     = 0.0;
    double nmeaLong    = 0.0;
    int    fixQ        = 0;
    int    numSat      = 0;

    if (!strncmp(strParse, "$GPGGA", 6))
    {
        sscanf(strParse, "$GPGGA,%11[^,],%lf,%c,%lf,%c,%d,%d,%*f,%f",
               timeStr, &nmeaLat, &gpsData.northSouth,
               &nmeaLong, &gpsData.eastWest,
               &fixQ, &numSat, &gpsData.altitude);

        gpsData.fixQuality    = (uint8_t)fixQ;
        gpsData.numSatellites = (uint8_t)numSat;
        parseUtcTime(timeStr);
    }
    else if (!strncmp(strParse, "$GPGLL", 6))
    {
        sscanf(strParse, "$GPGLL,%lf,%c,%lf,%c,%11[^,]",
               &nmeaLat, &gpsData.northSouth,
               &nmeaLong, &gpsData.eastWest, timeStr);

        parseUtcTime(timeStr);
    }
    else if (!strncmp(strParse, "$GPRMC", 6))
    {
        sscanf(strParse, "$GPRMC,%11[^,],%c,%lf,%c,%lf,%c",
               timeStr, &gpsData.posStatus,
               &nmeaLat, &gpsData.northSouth,
               &nmeaLong, &gpsData.eastWest);

        parseUtcTime(timeStr);
    }
    else
    {
        return;
    }

    gpsData.latitude  = nmeaToDecimal(nmeaLat);
    gpsData.longitude = nmeaToDecimal(nmeaLong);
}

/**
 * @brief Valida el checksum NMEA (XOR de los bytes entre '$' y '*').
 *
 * @return 1 si el checksum coincide, 0 en caso contrario.
 */
static int gpsValidate(char* nmea)
{
    char check[3];
    char calculatedString[3];
    int  index          = 0;
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

    sprintf(calculatedString, "%02X", calculatedCheck);
    return ((calculatedString[0] == check[0]) && (calculatedString[1] == check[1])) ? 1 : 0;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

NEO6M_GPS_Status_t NEO6M_GPS_Init(UART_HandleTypeDef* huart)
{
    if (huart == NULL)
    {
        return NEO6M_GPS_INVALID_PARAM;
    }

    NEO6M_GPS_huart = huart;

    rxIndex = 0U;
    memset(rxBuffer, 0, sizeof(rxBuffer));
    memset(&gpsData, 0, sizeof(gpsData));

    HAL_UART_Receive_IT(NEO6M_GPS_huart, &rxData, 1U);

    NEO6M_GPS_Initialized = 1U;

    return NEO6M_GPS_OK;
}

NEO6M_GPS_Status_t NEO6M_GPS_DeInit(void)
{
    NEO6M_GPS_huart       = NULL;
    NEO6M_GPS_Initialized = 0U;

    return NEO6M_GPS_OK;
}

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

    *data = gpsData;

    return NEO6M_GPS_OK;
}

void NEO6M_GPS_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (NEO6M_GPS_huart == NULL || huart != NEO6M_GPS_huart)
        return;

    if (rxData != '\n' && rxIndex < sizeof(rxBuffer))
    {
        rxBuffer[rxIndex++] = rxData;
    }
    else
    {
        if (gpsValidate((char*)rxBuffer))
            gpsParse((char*)rxBuffer);

        rxIndex = 0U;
        memset(rxBuffer, 0, sizeof(rxBuffer));
    }

    HAL_UART_Receive_IT(NEO6M_GPS_huart, &rxData, 1U);
}
