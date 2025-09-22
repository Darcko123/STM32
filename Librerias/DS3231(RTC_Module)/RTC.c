/*
 * RTC.c
 *
 * @brief Implementación de la librería para el módulo RTC DS3231 utilizando I2C en STM32.
 *
 * @author Daniel Ruiz
 * @date Jan 4, 2025
 * @version 1.0
 */

#include "RTC.h"

static I2C_HandleTypeDef* RTC_hi2c;     /**< Manejador de la interfaz I2C utilizado para comunicarse con el RTC */

/**
 * @brief Inicializa el sensor DS3231 (RTC)
 * 
 * @details
 * - Configura el handler de I2C para la comunicación con el módulo RTC.
 * - No realiza configuraciones adicionales ya que el DS3231 está listo para usarse tras la alimentación.
 */
void RTC_Init(I2C_HandleTypeDef* hi2c)
{
    RTC_hi2c = hi2c;
}

/**
 * @brief Convierte un número decimal a formato BCD.
 * 
 * @param val Valor en formato decimal.
 * @return Valor convertido a formato BCD.
 */
static uint8_t decToBcd(int val)
{
    return (uint8_t)((val / 10 * 16) + (val % 10));
}

/**
 * @brief Convierte un número en formato BCD a decimal.
 * 
 * @param val Valor en formato BCD.
 * @return Valor convertido a formato decimal.
 */
static int bcdToDec(uint8_t val)
{
    return (int)((val / 16 * 10) + (val % 16));
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
 */
void RTC_SetTime(uint8_t hour, uint8_t min, uint8_t sec, uint8_t dom, uint8_t month, uint8_t year)
{
    uint8_t set_time[7];
    set_time[0] = decToBcd(sec);
    set_time[1] = decToBcd(min);
    set_time[2] = decToBcd(hour);
    // set_time[3] = decToBcd(dow); // Day of the week (not used because we don't care about it. Zeller's algorithm can be implemented to get it)
    set_time[4] = decToBcd(dom);
    set_time[5] = decToBcd(month);
    set_time[6] = decToBcd(year);

    HAL_I2C_Mem_Write(RTC_hi2c, DS3231_ADDRESS, 0x00, 1, set_time, 7, 1000);
}

/**
 * @brief Obtiene la hora y fecha actual desde el módulo RTC.
 * 
 * @param time Puntero a una estructura TIME para almacenar la hora y fecha.
 */
void RTC_GetTime(TIME *time)
{
    uint8_t get_time[7];
    HAL_I2C_Mem_Read(RTC_hi2c, DS3231_ADDRESS, 0x00, 1, get_time, 7, 1000);

    time->seconds    = bcdToDec(get_time[0]);
    time->minutes    = bcdToDec(get_time[1]);
    time->hour       = bcdToDec(get_time[2]);
    time->dayofmonth = bcdToDec(get_time[4]);
    time->month      = bcdToDec(get_time[5]);
    time->year       = bcdToDec(get_time[6]);
}

/**
 * @brief Configura la Alarma 1 en el módulo RTC.
 * 
 * @param hourAlarm Horas de la alarma.
 * @param minAlarm Minutos de la alarma.
 * @param secAlarm Segundos de la alarma.
 */
void RTC_SetAlarm1(uint8_t hourAlarm, uint8_t minAlarm, uint8_t secAlarm)
{
    uint8_t set_alarm[3];

    set_alarm[0] = decToBcd(secAlarm); 
    set_alarm[1] = decToBcd(minAlarm); 
    set_alarm[2] = decToBcd(hourAlarm); 

    HAL_I2C_Mem_Write(RTC_hi2c, DS3231_ADDRESS, 0x07, 1, set_alarm, 3, 1000);
}

/**
 * @brief Obtiene los valores de Alarma 1.
 * 
 * @param alarma1 Puntero a una estructura ALARM1 para almacenar los datos.
 */
void RTC_GetAlarm1(ALARM1 *alarma1)
{
    uint8_t get_alarm[3];

    HAL_I2C_Mem_Read(RTC_hi2c, DS3231_ADDRESS, 0x07, 1, get_alarm, 3, 1000);

    alarma1->seconds = bcdToDec(get_alarm[0]);
    alarma1->minutes = bcdToDec(get_alarm[1]);
    alarma1->hour    = bcdToDec(get_alarm[2]);
}

/**
 * @brief Configura la Alarma 2 en el módulo RTC.
 * 
 * @param hourAlarm Horas de la alarma.
 * @param minAlarm Minutos de la alarma.
 */
void RTC_SetAlarm2(uint8_t hourAlarm, uint8_t minAlarm)
{
    uint8_t set_alarm[2];

    set_alarm[0] = decToBcd(minAlarm);
    set_alarm[1] = decToBcd(hourAlarm);

    HAL_I2C_Mem_Write(RTC_hi2c, DS3231_ADDRESS, 0x0B, 1, set_alarm, 2, 1000);
}

/**
 * @brief Obtiene los valores de Alarma 2.
 * 
 * @param alarma2 Puntero a una estructura ALARM2 para almacenar los datos.
 */
void RTC_GetAlarm2(ALARM2 *alarma2)
{
    uint8_t get_alarm[2];

    HAL_I2C_Mem_Read(RTC_hi2c, DS3231_ADDRESS, 0x0B, 1, get_alarm, 2, 1000);

    alarma2->minutes = bcdToDec(get_alarm[0]);
    alarma2->hour    = bcdToDec(get_alarm[1]);
}
