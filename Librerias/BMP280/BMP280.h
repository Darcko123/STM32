/**
 * @file BMP280.h
 * @brief Driver I2C para el sensor barométrico BMP280 en STM32.
 *
 * @details Proporciona inicialización, lectura de temperatura y presión con
 *          compensación por coeficientes de calibración de fábrica, y cálculo
 *          de altitud estimada. Compatible con BMP280 (Chip ID 0x58).
 *          Basado en la implementación de sheinz (MIT License, 2016).
 *
 * @author Daniel Ruiz
 * @date Mayo 28, 2026
 * @version 1.0.0
 */

#ifndef BMP280_H
#define BMP280_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"

#include <stdint.h>
#include <math.h>

// ============================================================================
// MACROS Y CONSTANTES [BMP280]
// ============================================================================

/**
 * @brief Dirección I2C del módulo BMP280.
 */
#define BMP280_ADDRESS_0        0x76U       /**< Dirección I2C, SDO = GND */
#define BMP280_ADDRESS_1        0x77U       /**< Dirección I2C, SDO = VCC */

#define BMP280_REG_ID           0xD0U       /**< Chip ID (lectura = 0x58) */
#define BMP280_REG_RESET        0xE0U       /**< Registro de reinicio */
#define BMP280_REG_STATUS       0xF3U       /**< bit[0]: im_update, bit[3]: measuring */
#define BMP280_REG_CTRL         0xF4U       /**< osrs_t[7:5], osrs_p[4:2], mode[1:0] */
#define BMP280_REG_CONFIG       0xF5U       /**< t_sb[7:5], filter[4:2] */
#define BMP280_REG_PRESSURE     0xF7U       /**< primer registro del bloque de datos */

#define BMP280_CHIP_ID          0x58U
#define BMP280_RESET_VALUE      0xB6U

#define BMP280_TIMEOUT_MS       100U        /**< Timeout HAL por defecto [ms] */

#define BMP280_SEA_LEVEL_HPA    1013.25f    /**< Presión de referencia al nivel del mar [hPa] */

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del BMP280.
 */
typedef enum {
    BMP280_OK               = 0,    /**< Operación exitosa */
    BMP280_ERROR            = 1,    /**< Error en la operación */
    BMP280_TIMEOUT          = 2,    /**< Timeout en la operación */
    BMP280_NOT_INITIALIZED  = 3,    /**< Módulo no inicializado */
    BMP280_INVALID_PARAM    = 4     /**< Parámetro inválido */
} BMP280_Status_t;

/**
 * @brief Coeficiente del filtro IIR para suavizado de mediciones.
 */
typedef enum {
    BMP280_FILTER_OFF   = 0,
    BMP280_FILTER_2     = 1,
    BMP280_FILTER_4     = 2,
    BMP280_FILTER_8     = 3,
    BMP280_FILTER_16    = 4
} BMP280_Filter_t;

/**
 * @brief Oversampling de presión y temperatura.
 */
typedef enum {
    BMP280_OS_SKIPPED   = 0,    /**< Sin medición */
    BMP280_OS_X1        = 1,    /**< Oversampling x1 */
    BMP280_OS_X2        = 2,    /**< Oversampling x2 */
    BMP280_OS_X4        = 3,    /**< Oversampling x4 */
    BMP280_OS_X8        = 4,    /**< Oversampling x8 */
    BMP280_OS_X16       = 5     /**< Oversampling x16 */
} BMP280_Oversampling_t;

/**
 * @brief Tiempo de espera entre mediciones en modo normal.
 */
typedef enum {
    BMP280_STANDBY_0_5MS    = 0,    /**< 0.5 ms */
    BMP280_STANDBY_62_5MS   = 1,    /**< 62.5 ms */
    BMP280_STANDBY_125MS    = 2,    /**< 125 ms */
    BMP280_STANDBY_250MS    = 3,    /**< 250 ms */
    BMP280_STANDBY_500MS    = 4,    /**< 500 ms */
    BMP280_STANDBY_1000MS   = 5,    /**< 1 s */
    BMP280_STANDBY_2000MS   = 6,    /**< 2 s */
    BMP280_STANDBY_4000MS   = 7     /**< 4 s */
} BMP280_StandbyTime_t;

/**
 * @brief Datos de medición del BMP280.
 */
typedef struct {
    float temperatura;  /**< Temperatura [°C] */
    float presion;      /**< Presión [hPa] */
    float altitud;      /**< Altitud estimada [m] */
} BMP280_Data_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el driver BMP280 con configuración por defecto.
 *
 * Intenta detectar el sensor en las direcciones 0x76 y 0x77.
 * Realiza un soft reset y aplica una configuración óptima para uso general.
 *
 * @param[in] hi2c   Puntero al handle de I2C.
 * @return BMP280_Status_t
 *         - BMP280_OK             si la inicialización fue exitosa.
 *         - BMP280_ERROR          si el Chip ID no coincide o hay error de comunicación.
 *         - BMP280_INVALID_PARAM  si @p hi2c es NULL.
 */
BMP280_Status_t BMP280_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief Lee temperatura, presión y altitud compensadas del BMP280.
 *
 * @param[out] data Puntero a la estructura donde se guardarán los datos.
 * @return BMP280_Status_t
 */
BMP280_Status_t BMP280_Get(BMP280_Data_t* data);

#ifdef __cplusplus
}
#endif

#endif /* BMP280_H */