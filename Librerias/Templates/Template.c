/**
 * @file [[PREFIX]].c
 * @brief [[Descripción breve de una línea de la librería]].
 *
 * @details [[Descripción detallada, igual a la del .h]].
 *
 * @author [[Autor]]
 * @date [[Mes DD, AAAA]]
 * @version [[X.X.X]]
 */

#include "[[PREFIX]].h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static [[Tipo_Handle]]*     [[PREFIX]]_h[[protocolo]]    = NULL; /**< Handle de [[PROTOCOLO]] utilizado para comunicarse con el [[NOMBRE_MODULO]] */
static uint8_t              [[PREFIX]]_Initialized       = 0U;   /**< Bandera para verificar si el módulo está inicializado */

// ============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
// ============================================================================

static [[PREFIX]]_Status_t [[prefix]]_WriteReg(uint8_t reg, uint8_t value);
static [[PREFIX]]_Status_t [[prefix]]_ReadRegs(uint8_t reg, uint8_t* buf, uint16_t len);

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Escribe un valor de 8 bits en un registro del [[NOMBRE_MODULO]].
 */
static [[PREFIX]]_Status_t [[prefix]]_WriteReg(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Write(
        [[PREFIX]]_h[[protocolo]],
        (uint16_t)([[PREFIX]]_ADDRESS << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1U,
        [[PREFIX]]_TIMEOUT_MS
    );

    if (hal == HAL_TIMEOUT) { return [[PREFIX]]_TIMEOUT; }
    if (hal != HAL_OK)      { return [[PREFIX]]_ERROR;   }

    return [[PREFIX]]_OK;
}

/**
 * @brief Lee @p len bytes a partir de un registro del [[NOMBRE_MODULO]].
 */
static [[PREFIX]]_Status_t [[prefix]]_ReadRegs(uint8_t reg, uint8_t* buf, uint16_t len)
{
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Read(
        [[PREFIX]]_h[[protocolo]],
        (uint16_t)([[PREFIX]]_ADDRESS << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        len,
        [[PREFIX]]_TIMEOUT_MS
    );

    if (hal == HAL_TIMEOUT) { return [[PREFIX]]_TIMEOUT; }
    if (hal != HAL_OK)      { return [[PREFIX]]_ERROR;   }

    return [[PREFIX]]_OK;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el driver [[PREFIX]] con configuración por defecto.
 *
 * @param[in] [[h_protocolo]] Puntero al handle de [[PROTOCOLO]].
 * @return [[PREFIX]]_Status_t
 */
[[PREFIX]]_Status_t [[PREFIX]]_Init([[Tipo_Handle]]* [[h_protocolo]])
{
    if ([[h_protocolo]] == NULL) { return [[PREFIX]]_INVALID_PARAM; }

    [[PREFIX]]_h[[protocolo]] = [[h_protocolo]];

    /* [[Verificar identidad del chip, ej. leer WHO_AM_I / Chip ID]] */
    uint8_t id = 0U;
    [[PREFIX]]_Status_t status = [[prefix]]_ReadRegs([[PREFIX]]_REG_[[NOMBRE]], &id, 1U);
    if (status != [[PREFIX]]_OK || id != [[CHIP_ID_ESPERADO]]) {
        return [[PREFIX]]_ERROR;
    }

    /* [[Configuración inicial del módulo: reset, rangos, oversampling, etc.]] */

    [[PREFIX]]_Initialized = 1U;

    return [[PREFIX]]_OK;
}

/**
 * @brief Desinicializa el módulo [[PREFIX]], liberando recursos y marcando el módulo como no inicializado.
 *
 * @return [[PREFIX]]_Status_t Siempre retorna [[PREFIX]]_OK.
 */
[[PREFIX]]_Status_t [[PREFIX]]_DeInit(void)
{
    [[PREFIX]]_h[[protocolo]]  = NULL;
    [[PREFIX]]_Initialized     = 0U;

    return [[PREFIX]]_OK;
}

/**
 * @brief [[Descripción de la función principal de lectura/escritura]].
 *
 * @param[out] data Puntero a la estructura donde se guardarán los datos.
 * @return [[PREFIX]]_Status_t
 */
[[PREFIX]]_Status_t [[PREFIX]]_Get([[PREFIX]]_Data_t* data)
{
    if (data == NULL)                  { return [[PREFIX]]_INVALID_PARAM;   }
    if ([[PREFIX]]_Initialized != 1U)  { return [[PREFIX]]_NOT_INITIALIZED; }

    /* [[Leer registros crudos del módulo]] */
    uint8_t raw[[[N]]];
    [[PREFIX]]_Status_t status = [[prefix]]_ReadRegs([[PREFIX]]_REG_[[NOMBRE]], raw, [[N]]U);
    if (status != [[PREFIX]]_OK) { return status; }

    /* [[Convertir/compensar los datos crudos y asignarlos a 'data']] */
    data->[[campo1]] = [[conversión campo1]];
    data->[[campo2]] = [[conversión campo2]];

    return [[PREFIX]]_OK;
}
