/**
 * @file RTC.h
 * @brief Librería para la gestión del módulo RTC DS3231 mediante comunicación I2C.
 *
 * @author Daniel Ruiz
 * @date April 5, 2026
 * @version 2.1.0
 */

#ifndef RTC_H
#define RTC_H

/**
 * @brief Incluir el encabezado adecuado según la familia STM32 utilizada.
 * Por ejemplo:
 * - Para STM32F1xx: "stm32f1xx_hal.h"
 * - Para STM32F4xx: "stm32f4xx_hal.h"
 */
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// MACROS Y CONSTANTES DE COMANDOS DS3231
// ============================================================================ 

/**
 * @brief Dirección I2C del módulo RTC DS3231.
 */
#define DS3231_ADDRESS 0xD0

#define DS3231_REG_SECONDS 0x00
#define DS3231_REG_ALARM1_SECONDS 0x07
#define DS3231_REG_ALARM2_MINUTES 0x0B
#define DS3231_TEMP_REG      0x11    /**< Registro MSB de temperatura (parte entera, con signo) */
#define DS3231_TEMP_REG_FRAC 0x12    /**< Registro LSB de temperatura (fracción, bits [7:6] → 0.25°C/bit) */

// Timeout
#define DS3231_MAX_BUSY_TIMEOUT 500     /**< milisegundos*/

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
} ds3231_time_t;

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
} ds3231_alarm1_t;

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
} ds3231_alarm2_t;

/**
 * @brief Estructura para la temperatura interna del DS3231.
 *
 * @param integer   Parte entera de la temperatura en grados Celsius (puede ser negativo).
 * @param fraction  Parte fraccionaria en cuartos de grado: 0, 25, 50 ó 75 (centésimas de °C).
 * @param raw       Temperatura en punto fijo Q10.2 (centésimas de °C, con signo).
 *                  Ejemplo: 2350 → 23.50 °C, -625 → -6.25 °C.
 */
typedef struct
{
    int8_t  integer;    /**< Parte entera (°C), rango: -128..+127 */
    uint8_t fraction;   /**< Parte fraccionaria en centésimas: 0, 25, 50 ó 75 */
    int16_t raw;        /**< Temperatura escalada ×100 (centésimas de °C) */
} ds3231_temp_t;

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================
/**
 * @brief Enumeración para estados de retorno del DS3231.
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
 * @brief Inicializa el módulo DS3231 con el handle I2C.
 *
 * @param hi2c Puntero al handle de I2C utilizado para comunicarse con el DS3231.
 * @return DS3231_Status_t Estado de la inicialización (OK, ERROR, etc.)
 */
DS3231_Status_t RTC_Init(I2C_HandleTypeDef* hi2c);

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
 * @param[out] time Puntero a la estructura ds3231_time_t donde se almacenarán los datos.
 * @return DS3231_Status_t
 */
DS3231_Status_t RTC_GetTime(ds3231_time_t *time);

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
 * @param[out] alarma1 Puntero a la estructura ds3231_alarm1_t donde se almacenarán los datos.
 *
 * @return DS3231_Status_t
 */
DS3231_Status_t RTC_GetAlarm1(ds3231_alarm1_t *alarma1);

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
 * @param[out] alarma2 Puntero a la estructura ds3231_alarm2_t donde se almacenarán los datos.
 *
 * @return DS3231_Status_t
 */
DS3231_Status_t RTC_GetAlarm2(ds3231_alarm2_t *alarma2);

/**
 * @brief Lee la temperatura interna del sensor DS3231.
 *
 * @details El DS3231 dispone de un sensor de temperatura integrado que alimenta
 *          su oscilador de compensación. La resolución es de 0.25 °C. La
 *          conversión se actualiza automáticamente cada 64 segundos (o de forma
 *          manual activando el bit CONV del registro de control).
 *
 *          Los datos se leen de los registros 0x11 (MSB, parte entera en
 *          complemento a dos) y 0x12 (LSB, bits [7:6] para la fracción).
 *
 * @param[out] temp Puntero a la estructura ds3231_temp_t donde se almacenarán
 *                  los valores de temperatura.
 * @return DS3231_Status_t
 *   - DS3231_OK             : Lectura exitosa.
 *   - DS3231_ERROR          : Fallo en la comunicación I2C.
 *   - DS3231_NOT_INITIALIZED: El módulo no fue inicializado previamente.
 */
DS3231_Status_t RTC_GetTemperature(ds3231_temp_t *temp);

#ifdef __cplusplus
}
#endif

#endif /* RTC_H */
