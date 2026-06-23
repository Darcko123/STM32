/**
 * @file NV3007.h
 * @brief 
 *
 * @details 
 *
 * @author Daniel Ruiz
 * @date Junio 23, 2026
 * @version 0.1.0
 */

#ifndef NV3007_H
#define NV3007_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"

#include <stdint.h>

// ============================================================================
// MACROS Y CONSTANTES NV3007
// ============================================================================


// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del NV3007.
 */
typedef enum {
    NV3007_OK               = 0,    /**< Operación exitosa */
    NV3007_ERROR            = 1,    /**< Error en la operación */
    NV3007_TIMEOUT          = 2,    /**< Timeout en la operación */
    NV3007_NOT_INITIALIZED  = 3,    /**< Módulo no inicializado */
    NV3007_INVALID_PARAM    = 4     /**< Parámetro inválido */
} NV3007_Status_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el módulo NV3007
 * 
 * @param hspi Puntero al manejador de la interfaz SPI utilizada para comunicarse con el módulo.
 * @param GPIOx Puerto GPIO del pin CS del NV3007.
 * @param GPIO_PIN Pin GPIO del pin CS del NV3007.
 * 
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN);

/**
 * @brief Desinicializa el módulo NV3007, apagando la pantalla y liberando la configuración almacenada.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* NV3007_H */