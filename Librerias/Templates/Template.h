/**
 * @file [[PREFIX]].h
 * @brief [[Descripción breve de una línea de la librería]].
 *
 * @details [[Descripción detallada: qué hace la librería, qué protocolo usa
 *          (I2C/SPI/UART/GPIO), y cualquier detalle relevante del chip/módulo
 *          (ej. Chip ID, rangos soportados, modos de operación)]].
 *
 * @author [[Autor]]
 * @date [[Mes DD, AAAA]]
 * @version [[X.X.X]]
 */

#ifndef [[PREFIX]]_H
#define [[PREFIX]]_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"

#include <stdint.h>

// ============================================================================
// MACROS Y CONSTANTES [[PREFIX]]
// ============================================================================

/**
 * @brief Dirección [[PROTOCOLO]] del módulo [[NOMBRE_MODULO]].
 */
#define [[PREFIX]]_ADDRESS         [[0xXX]]U       /**< [[Descripción de la dirección]] */

#define [[PREFIX]]_REG_[[NOMBRE]]  [[0xXX]]U       /**< [[Descripción del registro]] */

#define [[PREFIX]]_TIMEOUT_MS      100U            /**< Timeout HAL por defecto [ms] */

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del [[PREFIX]].
 */
typedef enum {
    [[PREFIX]]_OK               = 0,    /**< Operación exitosa */
    [[PREFIX]]_ERROR            = 1,    /**< Error en la operación */
    [[PREFIX]]_TIMEOUT          = 2,    /**< Timeout en la operación */
    [[PREFIX]]_NOT_INITIALIZED  = 3,    /**< Módulo no inicializado */
    [[PREFIX]]_INVALID_PARAM    = 4     /**< Parámetro inválido */
} [[PREFIX]]_Status_t;

/**
 * @brief [[Descripción de la estructura de configuración, si aplica]].
 */
typedef struct {
    [[tipo]] [[campo1]];  /**< [[Descripción del campo1]] */
    [[tipo]] [[campo2]];  /**< [[Descripción del campo2]] */
} [[PREFIX]]_Config_t;

/**
 * @brief Datos de medición/lectura del [[NOMBRE_MODULO]].
 */
typedef struct {
    [[tipo]] [[campo1]];  /**< [[Descripción del campo1]] [[unidad]] */
    [[tipo]] [[campo2]];  /**< [[Descripción del campo2]] [[unidad]] */
} [[PREFIX]]_Data_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el driver [[PREFIX]] con configuración por defecto.
 *
 * @param[in] [[h_protocolo]]  Puntero al handle de [[PROTOCOLO]] (ej. I2C_HandleTypeDef*).
 * @return [[PREFIX]]_Status_t
 *         - [[PREFIX]]_OK             si la inicialización fue exitosa.
 *         - [[PREFIX]]_ERROR          si el chip no responde o hay error de comunicación.
 *         - [[PREFIX]]_INVALID_PARAM  si el handle es NULL.
 */
[[PREFIX]]_Status_t [[PREFIX]]_Init([[Tipo_Handle]]* [[h_protocolo]]);

/**
 * @brief Desinicializa el módulo [[PREFIX]], liberando recursos y marcando el módulo como no inicializado.
 *
 * @return [[PREFIX]]_Status_t Siempre retorna [[PREFIX]]_OK.
 */
[[PREFIX]]_Status_t [[PREFIX]]_DeInit(void);

/**
 * @brief [[Descripción de la función principal de lectura/escritura]].
 *
 * @param[out] data Puntero a la estructura donde se guardarán los datos.
 * @return [[PREFIX]]_Status_t
 *         - [[PREFIX]]_OK               si la operación fue exitosa.
 *         - [[PREFIX]]_INVALID_PARAM    si @p data es NULL.
 *         - [[PREFIX]]_NOT_INITIALIZED  si el módulo no fue inicializado.
 */
[[PREFIX]]_Status_t [[PREFIX]]_Get([[PREFIX]]_Data_t* data);

#ifdef __cplusplus
}
#endif

#endif /* [[PREFIX]]_H */
