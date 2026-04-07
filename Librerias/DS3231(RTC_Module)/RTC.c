/**
 * @file RTC.c
 * @brief Implementación de la librería para el módulo RTC DS3231 utilizando I2C en STM32.
 *
 * @author Daniel Ruiz
 * @date April 5, 2026
 * @version 2.1.0
 */

#include "RTC.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static I2C_HandleTypeDef* RTC_hi2c  =   NULL;     /**< Manejador de la interfaz I2C utilizado para comunicarse con el RTC */
static uint8_t  DS3231_Initialized  =   0;        /**< Bandera para verificar si el módulo está inicializado */

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Convierte un número decimal a formato BCD.
 * 
 * @param val Valor en formato decimal.
 * @return Valor convertido a formato BCD.
 */
static uint8_t decToBcd(uint8_t val)
{
    return (uint8_t)((val / 10 * 16) + (val % 10));
}

/**
 * @brief Convierte un número en formato BCD a decimal.
 * 
 * @param val Valor en formato BCD.
 * @return Valor convertido a formato decimal.
 */
static uint8_t bcdToDec(uint8_t val)
{
    return (uint8_t)((val / 16 * 10) + (val % 16));
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el sensor DS3231 (RTC)
 * 
 * @param hi2c Puntero al handler de I2C utilizado para comunicarse con el módulo RTC.
 * 
 * @details
 * - Configura el handler de I2C para la comunicación con el módulo RTC.
 * - No realiza configuraciones adicionales ya que el DS3231 está listo para usarse tras la alimentación.
 * 
 * @return DS3231_Status_t 
 */
DS3231_Status_t RTC_Init(I2C_HandleTypeDef* hi2c)
{
    if(hi2c == NULL)
    {
        return DS3231_ERROR;
    }

    RTC_hi2c = hi2c;
    DS3231_Initialized = 1;

    return DS3231_OK;
}

/**
 * @brief Configura la hora y fecha en el módulo RTC.
 * 
 * @param hour Hora (0-23).
 * @param min Minutos (0-59).
 * @param sec Segundos (0-59).
 * @param dom Día del mes (1-31).
 * @param month Mes (1-12).
 * @param year Año (0-99).
 * @return DS3231_Status_t 
 */
DS3231_Status_t RTC_SetTime(uint8_t hour, uint8_t min, uint8_t sec, uint8_t dom, uint8_t month, uint8_t year)
{
    if(DS3231_Initialized != 1)
    {
        return DS3231_NOT_INITIALIZED;
    }

    uint8_t set_time[7];
    set_time[0] = decToBcd(sec);
    set_time[1] = decToBcd(min);
    set_time[2] = decToBcd(hour);
    set_time[3] = 0x01; // Day of week fijo en 1 (no se usa; el DS3231 lo requiere en el registro)
    set_time[4] = decToBcd(dom);
    set_time[5] = decToBcd(month);
    set_time[6] = decToBcd(year);

    if(HAL_I2C_Mem_Write(RTC_hi2c, DS3231_ADDRESS, DS3231_REG_SECONDS, 1, set_time, 7, DS3231_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return DS3231_ERROR;
    }

    return DS3231_OK;
}

 /**
  * @brief Obtiene la hora y fecha actual desde el módulo RTC.
  * 
  * @param time Puntero a una estructura TIME para almacenar la hora y fecha.
  * @return DS3231_Status_t 
  */
DS3231_Status_t RTC_GetTime(ds3231_time_t *time)
{
    if(DS3231_Initialized != 1)
    {
        return DS3231_NOT_INITIALIZED;
    }

    uint8_t get_time[7];
    if(HAL_I2C_Mem_Read(RTC_hi2c, DS3231_ADDRESS, DS3231_REG_SECONDS, 1, get_time, 7, DS3231_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return DS3231_ERROR;
    }

    time->seconds    = bcdToDec(get_time[0]);
    time->minutes    = bcdToDec(get_time[1]);
    time->hour       = bcdToDec(get_time[2]);
    time->dayofmonth = bcdToDec(get_time[4]);
    time->month      = bcdToDec(get_time[5]);
    time->year       = bcdToDec(get_time[6]);

    return DS3231_OK;
}

/**
 * @brief Configura la Alarma 1 en el módulo RTC.
 * 
 * @param hourAlarm Horas de la alarma.
 * @param minAlarm Minutos de la alarma.
 * @param secAlarm Segundos de la alarma.
 * @return DS3231_Status_t 
 */
DS3231_Status_t RTC_SetAlarm1(uint8_t hourAlarm, uint8_t minAlarm, uint8_t secAlarm)
{
    if(DS3231_Initialized != 1)
    {
        return DS3231_NOT_INITIALIZED;
    }

    uint8_t set_alarm[3];

    set_alarm[0] = decToBcd(secAlarm); 
    set_alarm[1] = decToBcd(minAlarm); 
    set_alarm[2] = decToBcd(hourAlarm); 

    if(HAL_I2C_Mem_Write(RTC_hi2c, DS3231_ADDRESS, DS3231_REG_ALARM1_SECONDS, 1, set_alarm, 3, DS3231_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return DS3231_ERROR;
    }

    return DS3231_OK;
}

/**
 * @brief Obtiene los valores de Alarma 1.
 * 
 * @param alarma1 Puntero a una estructura ALARM1 para almacenar los datos.
 * @return DS3231_Status_t 
 */
DS3231_Status_t RTC_GetAlarm1(ds3231_alarm1_t *alarma1)
{

    if(DS3231_Initialized != 1)
    {
        return DS3231_NOT_INITIALIZED;
    }

    uint8_t get_alarm[3];

    if(HAL_I2C_Mem_Read(RTC_hi2c, DS3231_ADDRESS, DS3231_REG_ALARM1_SECONDS, 1, get_alarm, 3, DS3231_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return DS3231_ERROR;
    }

    alarma1->seconds = bcdToDec(get_alarm[0]);
    alarma1->minutes = bcdToDec(get_alarm[1]);
    alarma1->hour    = bcdToDec(get_alarm[2]);

    return DS3231_OK;
}

/**
 * @brief Configura la Alarma 2 en el módulo RTC.
 * 
 * @param hourAlarm Horas de la alarma.
 * @param minAlarm Minutos de la alarma.
 * @return DS3231_Status_t 
 */
DS3231_Status_t RTC_SetAlarm2(uint8_t hourAlarm, uint8_t minAlarm)
{

    if(DS3231_Initialized != 1)
    {
        return DS3231_NOT_INITIALIZED;
    }

    uint8_t set_alarm[2];

    set_alarm[0] = decToBcd(minAlarm);
    set_alarm[1] = decToBcd(hourAlarm);

    if(HAL_I2C_Mem_Write(RTC_hi2c, DS3231_ADDRESS, DS3231_REG_ALARM2_MINUTES, 1, set_alarm, 2, DS3231_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return DS3231_ERROR;
    }

    return DS3231_OK;
}

/**
 * @brief Obtiene los valores de Alarma 2.
 * 
 * @param alarma2 Puntero a una estructura ALARM2 para almacenar los datos.
 * @return DS3231_Status_t 
 */
DS3231_Status_t RTC_GetAlarm2(ds3231_alarm2_t *alarma2)
{
    if(DS3231_Initialized != 1)
    {
        return DS3231_NOT_INITIALIZED;
    }

    uint8_t get_alarm[2];

    if(HAL_I2C_Mem_Read(RTC_hi2c, DS3231_ADDRESS, DS3231_REG_ALARM2_MINUTES, 1, get_alarm, 2, DS3231_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return DS3231_ERROR;
    }

    alarma2->minutes = bcdToDec(get_alarm[0]);
    alarma2->hour    = bcdToDec(get_alarm[1]);

    return DS3231_OK;
}

/**
 * @brief Lee la temperatura interna del sensor DS3231.
 *
 * @details El DS3231 integra un sensor de temperatura que compensa su
 *          oscilador TCXO. Los datos residen en dos registros consecutivos:
 *
 *            - 0x11 (MSB): parte entera en complemento a dos (int8_t).
 *            - 0x12 (LSB): los dos bits más significativos (bits 7 y 6)
 *                          representan la fracción en múltiplos de 0.25 °C.
 *
 *          Tabla de conversión de la fracción:
 *            Bits [7:6] | Fracción (°C)
 *            -----------+---------------
 *               00      |  0.00
 *               01      |  0.25
 *               10      |  0.50
 *               11      |  0.75
 *
 *          El campo `raw` almacena la temperatura escalada ×100 (sin punto
 *          flotante), lo que permite imprimir fácilmente con:
 *
 *            printf("%d.%02d °C", temp.integer, temp.fraction);
 *
 * @param[out] temp Puntero a la estructura ds3231_temp_t donde se guardarán
 *                  los valores leídos.
 * @return DS3231_Status_t
 */
DS3231_Status_t RTC_GetTemperature(ds3231_temp_t *temp)
{
    if(DS3231_Initialized != 1)
    {
        return DS3231_NOT_INITIALIZED;
    }

    if(temp == NULL)
    {
        return DS3231_ERROR;
    }

    uint8_t raw_bytes[2];

    /* Leer 2 bytes consecutivos a partir del registro 0x11 */
    if(HAL_I2C_Mem_Read(RTC_hi2c, DS3231_ADDRESS, DS3231_TEMP_REG, 1, raw_bytes, 2, DS3231_MAX_BUSY_TIMEOUT) != HAL_OK)
    {
        return DS3231_ERROR;
    }

    /* Parte entera: byte MSB interpretado como entero con signo */
    temp->integer = (int8_t)raw_bytes[0];

    /* Parte fraccionaria: bits [7:6] del byte LSB → 0, 25, 50 ó 75 */
    uint8_t frac_bits = (raw_bytes[1] >> 6) & 0x03U;
    const uint8_t frac_table[4] = {0U, 25U, 50U, 75U};
    temp->fraction = frac_table[frac_bits];

    /*
     * Campo raw: temperatura en centésimas de °C (entero escalado ×100).
     * Para valores negativos la fracción se resta (ej. -6.25 °C → raw = -625).
     */
    if(temp->integer < 0)
    {
        temp->raw = (int16_t)((int16_t)temp->integer * 100) - (int16_t)temp->fraction;
    }
    else
    {
        temp->raw = (int16_t)((int16_t)temp->integer * 100) + (int16_t)temp->fraction;
    }

    return DS3231_OK;
}
