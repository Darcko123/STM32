#include "BME688.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static I2C_HandleTypeDef*   BME688_hi2c        = NULL;
static uint8_t              BME688_addr        = BME688_ADDRESS_0;
static uint8_t              BME688_Initialized = 0U;


// ============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
// ============================================================================

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

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
BME688_Status_t BME688_Init(I2C_HandleTypeDef* hi2c)
{

}

/**
 * @brief Lee temperatura, presión y altitud compensadas del BMP280.
 *
 * @param[out] data Puntero a la estructura donde se guardarán los datos.
 * @return BME688_Status_t
 */
BME688_Status_t BME688_Get(BME688_Data_t* data)
{
    
}