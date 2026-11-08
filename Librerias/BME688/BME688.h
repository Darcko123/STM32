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

#define BME688_ADDRESS_0        0x76U       /**< Dirección I2C, SDO = GND */
#define BME688_ADDRESS_1        0x77U       /**< Dirección I2C, SDO = VCC */

#define BME688_REG_COEFF3       0x00U       /**< Bloque 3 de calibración (5 bytes) */
#define BME688_REG_COEFF1       0x8AU       /**< Bloque 1 de calibración (23 bytes) */
#define BME688_REG_ID           0xD0U       /**< Chip ID (lectura = 0x61) */
#define BME688_REG_RESET        0xE0U       /**< Registro de reinicio */
#define BME688_REG_COEFF2       0xE1U       /**< Bloque 2 de calibración (14 bytes) */
#define BME688_REG_VARIANT_ID   0xF0U       /**< Variante: 0x00 BME680, 0x01 BME688 */

#define BME688_LEN_COEFF1       23U
#define BME688_LEN_COEFF2       14U
#define BME688_LEN_COEFF3       5U
#define BME688_LEN_COEFF_ALL    (BME688_LEN_COEFF1 + BME688_LEN_COEFF2 + BME688_LEN_COEFF3)

#define BME688_CHIP_ID          0x61U
#define BME688_RESET_VALUE      0xB6U
#define BME688_RESET_DELAY_MS   10U         /**< Tiempo de arranque tras soft reset [ms] */

#define BME688_TIMEOUT_MS       100U        /**< Timeout HAL por defecto [ms] */

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

/**
 * @brief Datos de medición del BME688.
 */
typedef struct {
    float   temperatura;    /**< Temperatura [°C] */
    float   presion;        /**< Presión [hPa] */
    float   humedad;        /**< Humedad relativa [%RH] */
    float   gas;            /**< Resistencia del sensor de gas [Ω] */
    uint8_t gas_valido;     /**< 1 si la medición de gas es válida y el calentador se estabilizó */
} BME688_Data_t;

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
 * @brief Realiza una medición en forced mode y retorna temperatura, presión, humedad y gas compensados.
 *
 * @note Bloquea durante la medición y el calentamiento (~145 ms con la configuración por defecto).
 *
 * @param[out] data Puntero a la estructura donde se guardarán los datos.
 * @return BME688_Status_t
 *         - BME688_OK              si la lectura fue exitosa.
 *         - BME688_TIMEOUT         si no hubo datos nuevos o el sensor no volvió a sleep.
 *         - BME688_NOT_INITIALIZED si no se llamó a BME688_Init().
 *         - BME688_INVALID_PARAM   si @p data es NULL.
 */
BME688_Status_t BME688_Get(BME688_Data_t* data);

#ifdef __cplusplus
}
#endif

#endif /* BME688_H */