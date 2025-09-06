/** 
 * RTC.h
 *
 * @brief Librería para la gestión del módulo RTC DS3231 mediante comunicación I2C.
 *
 * @author Daniel Ruiz
 * @date Jan 4, 2025
 * @version 1.0
 */

#ifndef RTC_H_
#define RTC_H_

//Cambiar según el microcontrolador que se esté utilizando
#include "stm32f1xx_hal.h"

/**
 * @brief Dirección I2C del módulo RTC DS3231.
 */
#define DS3231_ADDRESS 0xD0

/**
 * @brief Estructura que representa la fecha y hora actuales.
 *
 * @param seconds   Segundos (0-59)
 * @param minutes   Minutos (0-59)
 * @param hour      Hora (0-23 en formato 24h)
 * @param dayofmonth Día del mes (1-31)
 * @param month     Mes (1-12)
 * @param year      Año (0-99)
 */
typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
    uint8_t dayofmonth;
    uint8_t month;
    uint8_t year;
} TIME;

/**
 * @brief Estructura para la configuración de la Alarma 1.
 *
 * @param seconds Segundos para la alarma (0-59)
 * @param minutes Minutos para la alarma (0-59)
 * @param hour    Hora para la alarma (0-23 en formato 24h)
 */
typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
} ALARM1;

/**
 * @brief Estructura para la configuración de la Alarma 2.
 *
 * @param minutes Minutos para la alarma (0-59)
 * @param hour    Hora para la alarma (0-23 en formato 24h)
 */
typedef struct
{
    uint8_t minutes;
    uint8_t hour;
} ALARM2;

/**
 * @brief Configura la hora y fecha del módulo RTC DS3231.
 *
 * @param hour  Hora (0-23)
 * @param min   Minutos (0-59)
 * @param sec   Segundos (0-59)
 * @param dom   Día del mes (1-31)
 * @param month Mes (1-12)
 * @param year  Año (0-99)
 */
void RTC_SetTime(uint8_t hour, uint8_t min, uint8_t sec, uint8_t dom, uint8_t month, uint8_t year);

/**
 * @brief Obtiene la hora y fecha actuales del módulo RTC DS3231.
 *
 * @param[out] time Puntero a la estructura TIME donde se almacenarán los datos.
 */
void RTC_GetTime(TIME *time);

/**
 * @brief Configura la Alarma 1 del módulo RTC DS3231.
 *
 * @param hourAlarm Horas de la alarma (0-23)
 * @param minAlarm  Minutos de la alarma (0-59)
 * @param secAlarm  Segundos de la alarma (0-59) 
 */
void RTC_SetAlarm1(uint8_t hourAlarm, uint8_t minAlarm, uint8_t secAlarm);

/**
 * @brief Obtiene la configuración actual de la Alarma 1.
 *
 * @param[out] alarma1 Puntero a la estructura ALARM1 donde se almacenarán los datos.
 */
void RTC_GetAlarm1(ALARM1 *alarma1);

/**
 * @brief Configura la Alarma 2 del módulo RTC DS3231.
 *
 * @param hourAlarm Horas de la alarma (0-23)
 * @param minAlarm  Minutos de la alarma (0-59)
 */
void RTC_SetAlarm2(uint8_t hourAlarm, uint8_t minAlarm);

/**
 * @brief Obtiene la configuración actual de la Alarma 2.
 *
 * @param[out] alarma2 Puntero a la estructura ALARM2 donde se almacenarán los datos.
 */
void RTC_GetAlarm2(ALARM2 *alarma2);

#endif /* RTC_H_ */
