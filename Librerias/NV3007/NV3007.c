/**
 * @file NV3007.c
 * @brief 
 *
 * @details 
 *
 * @author Daniel Ruiz
 * @date Junio 23, 2026
 * @version 0.1.0
 */

#include "NV3007.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static SPI_HandleTypeDef*   NV3007_hspi         = NULL; /**< Handle de SPI utilizado para comunicarse con el NV3007 */
static GPIO_TypeDef*        NV3007_CS_GPIO_Port = NULL; /**< Puerto GPIO del pin de datos */
static uint16_t             CS_Pin              = 0;    /**< Pin GPIO del pin de datos del NV3007 */   
static uint8_t              NV3007_Initialized  = 0U;   /**< Bandera para verificar si el módulo está inicializado */

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el módulo NV3007
 * 
 * @param hspi Puntero al manejador de la interfaz SPI utilizada para comunicarse con el módulo.
 * @param GPIOx Puerto GPIO del pin CS del NV3007.
 * @param GPIO_PIN Pin GPIO del pin CS del NV3007.
 * 
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN)
{
    NV3007_Status_t status;

    // Validar parámetros de entrada
    if(hspi == NULL || GPIOx == NULL || GPIO_PIN == 0U)
    {
        return NV3007_INVALID_PARAM;
    }

    // Almacenar configuración para uso en funciones posteriores
    NV3007_hspi         = hspi;
    NV3007_CS_GPIO_Port = GPIOx;
    CS_Pin              = GPIO_PIN; 
}

/**
 * @brief Desinicializa el módulo NV3007, apagando la pantalla y liberando la configuración almacenada.
 *
 * @return NV3007_Status_t Estado de la operación.
 */
NV3007_Status_t NV3007_DeInit(void)
{
    NV3007_Status_t status;

    if(!NV3007_Initialized)
    {
        return NV3007_NOT_INITIALIZED;
    }

    // Apagar la pantalla (modo shutdown)

    // Liberar configuración
    NV3007_hspi         = NULL;
    NV3007_CS_GPIO_Port = NULL;
    CS_Pin              = 0U;
    NV3007_Initialized  = 0U;

    return NV3007_OK;
}