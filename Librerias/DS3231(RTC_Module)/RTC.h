/** 
 * RTC.h
 *
 * @brief Librería para la gestión del módulo RTC DS3231 mediante comunicación I2C.
 *
 * @author Daniel Ruiz
 * @date Sep 22, 2025
 * @version 2.0.0
 */

#ifndef RTC_H_
#define RTC_H_

/**
 * @brief Incluir el encabezado adecuado según la familia STM32 utilizada.
 * Por ejemplo:
 * - Para STM32F1xx: "stm32f1xx_hal.h"
 * - Para STM32F4xx: "stm32f4xx_hal.h"
 */
#include "stm32f4xx_hal.h"

// ============================================================================
// MACROS Y CONSTANTES DE COMANDOS DS3231
// ============================================================================ 

/**
 * @brief Dirección I2C del módulo RTC DS3231.
 */
#define DS3231_ADDRESS 0xD0

// ============================================================================
// CONFIGURACIÓN TIEMPO (ESTRUCTURAS)
// ============================================================================

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

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================
/**
 * @brief Enumeración para estados de retorno del SX1262.
 */
typedef enum {
	DS3231_OK = 0,				/**< Operación exitosa */
	DS3231_ERROR = 1,			/**< Error en la operación */
	DS3231_TIMEOUT = 2,			/**< Timeout en la operación */
	DS3231_NOT_INITIALIZED = 3,	/**< Módulo no inicializado */
}DS3231_Status_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configura la hora y fecha del módulo RTC DS3231.
 *
 * @param hour  Hora (0-23)
 * @param min   Minutos (0-59)
 * @param sec   Segundos (0-59)
 * @param dom   Día del mes (1-31)
 * @param month Mes (1-12)
 * @param year  Año (0-99)
 * 
 * @return DS3231_Status_t
 */
DS3231_Status_t RTC_SetTime(uint8_t hour, uint8_t min, uint8_t sec, uint8_t dom, uint8_t month, uint8_t year);

/**
 * @brief Obtiene la hora y fecha actuales del módulo RTC DS3231.
 *
 * @param[out] time Puntero a la estructura TIME donde se almacenarán los datos.
 */
DS3231_Status_t RTC_GetTime(TIME *time);

/**
 * @brief Configura la Alarma 1 del módulo RTC DS3231.
 *
 * @param hourAlarm Horas de la alarma (0-23)
 * @param minAlarm  Minutos de la alarma (0-59)
 * @param secAlarm  Segundos de la alarma (0-59) 
 * 
 * @return DS3231_Status_t
 */
DS3231_Status_t RTC_SetAlarm1(uint8_t hourAlarm, uint8_t minAlarm, uint8_t secAlarm);

/**
 * @brief Obtiene la configuración actual de la Alarma 1.
 *
 * @param[out] alarma1 Puntero a la estructura ALARM1 donde se almacenarán los datos.
 * 
 * @return DS3231_Status_t
 */
DS3231_Status_t RTC_GetAlarm1(ALARM1 *alarma1);

/**
 * @brief Configura la Alarma 2 del módulo RTC DS3231.
 *
 * @param hourAlarm Horas de la alarma (0-23)
 * @param minAlarm  Minutos de la alarma (0-59)
 * 
 * @return DS3231_Status_t
 */
DS3231_Status_t RTC_SetAlarm2(uint8_t hourAlarm, uint8_t minAlarm);

/**
 * @brief Obtiene la configuración actual de la Alarma 2.
 *
 * @param[out] alarma2 Puntero a la estructura ALARM2 donde se almacenarán los datos.
 * 
 * @return DS3231_Status_t
 */
DS3231_Status_t RTC_GetAlarm2(ALARM2 *alarma2);

#ifdef __cplusplus
}
#endif

#endif /* RTC_H_ */
