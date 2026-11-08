#ifndef BME688_H
#define BME688_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"

#include <stdint.h>
#include <math.h>

// ============================================================================
// MACROS Y CONSTANTES [BME688]
// ============================================================================

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del BME688.
 */
typedef enum {
    BME688_OK               = 0,    /**< Operación exitosa */
    BME688_ERROR            = 1,    /**< Error en la operación */
    BME688_TIMEOUT          = 2,    /**< Timeout en la operación */
    BME688_NOT_INITIALIZED  = 3,    /**< Módulo no inicializado */
    BME688_INVALID_PARAM    = 4     /**< Parámetro inválido */
} BME688_Status_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el driver BME688 con configuración por defecto.
 *
 * @param[in] hi2c   Puntero al handle de I2C.
 * @return BME688_Status_t
 *         - BME688_OK             si la inicialización fue exitosa.
 *         - BME688_ERROR          si el Chip ID no coincide o hay error de comunicación.
 *         - BME688_INVALID_PARAM  si @p hi2c es NULL.
 */
BME688_Status_t BME688_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief Lee temperatura, presión y altitud compensadas del BMP280.
 *
 * @param[out] data Puntero a la estructura donde se guardarán los datos.
 * @return BME688_Status_t
 */
BME688_Status_t BME688_Get(BME688_Data_t* data);

#ifdef __cplusplus
}
#endif

#endif /* BME688_H */