/**
 * @file LIS3DH.c
 * @brief Implementación de funciones para el manejo del acelerómetro LIS3DH
 * 
 * @details Este archivo contiene las definiciones de las funciones para inicializar
 *          y leer datos del acelerómetro LIS3DH mediante la interfaz 
 *          I2C en un microcontrolador STM32.
 * 
 * @author   Daniel Ruiz
 * @date     Octubre 15, 2026
 * @version  1.0.0
 */

#include "LIS3DH.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static I2C_HandleTypeDef*   LIS3DH_hi2c        = NULL; /**< Manejador de la interfaz I2C utilizado para comunicarse con el LIS3DH */
static uint8_t              LIS3DH_Initialized = 0U;   /**< Bandera para verificar si el sensor está inicializado */

/** @brief Almacenan valores RAW del acelerómetro. */
static int16_t Accel_X_RAW = 0, Accel_Y_RAW = 0, Accel_Z_RAW = 0;

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief  Inicializa el sensor LIS3DH.
 * 
 * @param[in] hi2c Puntero al manejador I2C.
 * 
 * @details  
 * - Espera el tiempo de arranque del sensor (5 ms según datasheet).
 * - Verifica la conexión con el dispositivo leyendo el registro WHO_AM_I.
 * - Habilita los ejes X, Y, Z a 400 Hz en modo normal (CTRL_REG1).
 * - Activa BDU y modo alta resolución, escala ±2g (CTRL_REG4).
 * - Enruta la interrupción DRDY1 al pin INT1 (CTRL_REG3).
 * - Habilita el ADC auxiliar (TEMP_CFG_REG).
 * 
 * @return LIS3DH_Status_t Estado de la operación
 */
LIS3DH_Status_t LIS3DH_Init(I2C_HandleTypeDef* hi2c)
{
    HAL_StatusTypeDef hal_status;
    uint8_t check, data;

    // Validar parámetros de entrada
    if(hi2c == NULL)
    {
        return LIS3DH_INVALID_PARAM;
    }

    // Almacenar configuración para uso en funciones posteriores
    LIS3DH_hi2c = hi2c;

    // Tiempo de arranque tras alimentación (datasheet: 5 ms)
    HAL_Delay(5);

    // Comprobar conexión con el LIS3DH mediante el registro WHO_AM_I
    hal_status = HAL_I2C_Mem_Read(LIS3DH_hi2c, LIS3DH_ADDRESS, WHO_AM_I_REG, 1, &check, 1, 1000);
    if(hal_status != HAL_OK)
    {
        return LIS3DH_ERROR;
    }

    // Verificar que el dispositivo es un LIS3DH
    if(check != LIS3DH_WHO_AM_I_CONTENT)
    {
        return LIS3DH_ERROR;
    }

    // Habilitar ejes X, Y, Z, modo normal (LPen = 0), ODR = 400 Hz
    data = 0x77; // ODR[3:0] = 0111 | LPen = 0 | Zen = Yen = Xen = 1
    hal_status = HAL_I2C_Mem_Write(LIS3DH_hi2c, LIS3DH_ADDRESS, CTRL_REG1, 1, &data, 1, 1000);
    if(hal_status != HAL_OK)
    {
        return LIS3DH_ERROR;
    }

    // Block Data Update y alta resolución (12 bits), escala ±2g
    data = 0x88; // BDU = 1 | FS = 00 | HR = 1
    hal_status = HAL_I2C_Mem_Write(LIS3DH_hi2c, LIS3DH_ADDRESS, CTRL_REG4, 1, &data, 1, 1000);
    if(hal_status != HAL_OK)
    {
        return LIS3DH_ERROR;
    }

    // Interrupción de dato listo (DRDY1) en el pin INT1
    data = 0x10; // I1_ZYXDA = 1
    hal_status = HAL_I2C_Mem_Write(LIS3DH_hi2c, LIS3DH_ADDRESS, CTRL_REG3, 1, &data, 1, 1000);
    if(hal_status != HAL_OK)
    {
        return LIS3DH_ERROR;
    }

    // Habilitar el ADC auxiliar
    data = 0x80; // ADC_EN = 1
    hal_status = HAL_I2C_Mem_Write(LIS3DH_hi2c, LIS3DH_ADDRESS, TEMP_CFG_REG, 1, &data, 1, 1000);
    if(hal_status != HAL_OK)
    {
        return LIS3DH_ERROR;
    }

    // Marcar como inicializado
    LIS3DH_Initialized = 1U;

    return LIS3DH_OK;
}

/**
 * @brief Desinicializa el sensor LIS3DH, liberando recursos y marcando el módulo como no inicializado.
 * 
 * @return LIS3DH_Status_t Siempre retorna LIS3DH_OK
 */
LIS3DH_Status_t LIS3DH_DeInit(void)
{
    LIS3DH_hi2c         = NULL;    // Limpia el handler de I2C
    LIS3DH_Initialized  = 0U;      // Marca el módulo como no inicializado
    
    return LIS3DH_OK;
}