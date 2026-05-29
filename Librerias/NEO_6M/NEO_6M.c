/**
 * @file NEO6M_GPS.c
 * @brief Implementación de la librería NEO6M_GPS.
 *
 * @author Daniel Ruiz
 * @date May 29, 2026
 * @version 0.1.0
 */

#include "NEO6M_GPS.h"
#include <stdio.h>
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

static float nmeaToDecimal(float coordinate);
static void  gpsParse(char* strParse);
static int   gpsValidate(char* nmea);

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Convierte una coordenada NMEA (DDmm.mmmm) a grados decimales.
 */
static float nmeaToDecimal(float coordinate)
{
    int   degree  = (int)(coordinate / 100);
    float minutes = coordinate - degree * 100;
    return degree + (minutes / 60.0f);
}

/**
 * @brief Parsea una sentencia NMEA validada y actualiza gpsData.
 */
static void gpsParse(char* strParse)
{
    float nmeaLat  = 0.0f;
    float nmeaLong = 0.0f;

    if (!strncmp(strParse, "$GPGGA", 6))
    {
        sscanf(strParse, "$GPGGA,%f,%f,%c,%f,%c",
               &gpsData.utcTime, &nmeaLat, &gpsData.northSouth,
               &nmeaLong, &gpsData.eastWest);
    }
    else if (!strncmp(strParse, "$GPGLL", 6))
    {
        sscanf(strParse, "$GPGLL,%f,%c,%f,%c,%f",
               &nmeaLat, &gpsData.northSouth, &nmeaLong,
               &gpsData.eastWest, &gpsData.utcTime);
    }
    else if (!strncmp(strParse, "$GPRMC", 6))
    {
        sscanf(strParse, "$GPRMC,%f,%c,%f,%c,%f,%c",
               &gpsData.utcTime, &gpsData.posStatus, &nmeaLat,
               &gpsData.northSouth, &nmeaLong, &gpsData.eastWest);
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
    if (config == NULL || config->huart == NULL)
        return NEO6M_GPS_INVALID_PARAM;

    NEO6M_GPS_huart = config->huart;

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

NEO6M_GPS_Status_t NEO6M_GPS_GetData(NEO6M_GPS_Data_t* data)
{
    if (data == NULL)
        return NEO6M_GPS_INVALID_PARAM;

    if (NEO6M_GPS_Initialized != 1U)
        return NEO6M_GPS_NOT_INITIALIZED;

    *data = gpsData;

    return NEO6M_GPS_OK;
}

void NEO6M_GPS_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (NEO6M_GPS_huart == NULL || huart->Instance != NEO6M_GPS_huart->Instance)
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
