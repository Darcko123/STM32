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

/**
 * @brief Lee los valores del acelerómetro y los convierte a unidades 'g'.
 *
 * @param[out] accel Puntero a la estructura donde se almacenarán los valores de aceleración en 'g'.
 *
 * @return LIS3DH_Status_t Estado de la operación
 *
 * @details
 * - Lee CTRL_REG1..CTRL_REG4 para conocer el modo (LPen, HR) y la escala (FS).
 * - Lee 6 bytes desde OUT_X_L con auto-incremento (bit 7 de la subdirección).
 * - Los datos son little-endian y justificados a la izquierda; se escalan según
 *   la sensibilidad del datasheet (tabla 4) para el modo y la escala actuales.
 */
LIS3DH_Status_t LIS3DH_GetAccel(LIS3DH_Accel_t *accel)
{
    // Sensibilidad en mg/dígito [modo][FS] (datasheet LIS3DH, tabla 4)
    static const uint8_t sens_mg[3][4] = {
        {  1,  2,  4,  12 },    // Alta resolución (12 bits)
        {  4,  8, 16,  48 },    // Normal (10 bits)
        { 16, 32, 64, 192 }     // Bajo consumo (8 bits)
    };
    static const uint8_t shift[3] = { 4, 6, 8 }; // Bits sin usar a la derecha

    HAL_StatusTypeDef hal_status;
    uint8_t ctrl[4], Rec_Data[6];
    uint8_t mode, fs;
    float scale;

    // Verificación de inicialización
    if(LIS3DH_Initialized == 0) { return LIS3DH_NOT_INITIALIZED; }

    // Validar puntero de salida
    if(accel == NULL) { return LIS3DH_INVALID_PARAM; }

    // Leer CTRL_REG1..CTRL_REG4 (auto-incremento)
    hal_status = HAL_I2C_Mem_Read(LIS3DH_hi2c, LIS3DH_ADDRESS, CTRL_REG1 | LIS3DH_AUTO_INC, 1, ctrl, 4, 1000);
    if(hal_status == HAL_OK)
    {
        // Leer 6 bytes de datos del acelerómetro (auto-incremento)
        hal_status = HAL_I2C_Mem_Read(LIS3DH_hi2c, LIS3DH_ADDRESS, OUT_X_L_REG | LIS3DH_AUTO_INC, 1, Rec_Data, 6, 1000);
    }

    if(hal_status == HAL_TIMEOUT)
    {
        return LIS3DH_TIMEOUT;
    }
    else if (hal_status != HAL_OK)
    {
        return LIS3DH_ERROR;
    }

    // Modo: LPen (CTRL_REG1 bit 3) / HR (CTRL_REG4 bit 3); escala: FS (CTRL_REG4 bits 5:4)
    if(ctrl[0] & 0x08)      { mode = 2; }
    else if(ctrl[3] & 0x08) { mode = 0; }
    else                    { mode = 1; }
    fs = (ctrl[3] >> 4) & 0x03;

    // g por cuenta del valor de 16 bits justificado a la izquierda
    scale = sens_mg[mode][fs] / (1000.0f * (1U << shift[mode]));

    // Convertir datos RAW (little-endian) a valores enteros de 16 bits
    Accel_X_RAW = (int16_t)(Rec_Data[1] << 8 | Rec_Data[0]);
    Accel_Y_RAW = (int16_t)(Rec_Data[3] << 8 | Rec_Data[2]);
    Accel_Z_RAW = (int16_t)(Rec_Data[5] << 8 | Rec_Data[4]);

    // Convertir valores RAW a aceleración en 'g'
    accel->x = Accel_X_RAW * scale;
    accel->y = Accel_Y_RAW * scale;
    accel->z = Accel_Z_RAW * scale;

    return LIS3DH_OK;
}

/**
 * @brief Configura la escala completa del acelerómetro.
 *
 * @param[in] range Escala deseada (±2g, ±4g, ±8g o ±16g).
 *
 * @return LIS3DH_Status_t Estado de la operación
 *
 * @details
 * - Lectura-modificación-escritura de CTRL_REG4: solo cambia FS[1:0] (bits 5:4),
 *   conservando BDU, HR y el resto de bits.
 * - LIS3DH_GetAccel lee FS del registro, por lo que la conversión a 'g' se
 *   ajusta automáticamente.
 */
LIS3DH_Status_t LIS3DH_SetRange(LIS3DH_Range_t range)
{
    HAL_StatusTypeDef hal_status;
    uint8_t ctrl4;

    // Verificación de inicialización
    if(LIS3DH_Initialized == 0) { return LIS3DH_NOT_INITIALIZED; }

    // Validar escala
    if(range > LIS3DH_RANGE_16G) { return LIS3DH_INVALID_PARAM; }

    // Leer CTRL_REG4, sustituir FS[1:0] y escribir de vuelta
    hal_status = HAL_I2C_Mem_Read(LIS3DH_hi2c, LIS3DH_ADDRESS, CTRL_REG4, 1, &ctrl4, 1, 1000);
    if(hal_status == HAL_OK)
    {
        ctrl4 = (ctrl4 & ~0x30) | ((uint8_t)range << 4);
        hal_status = HAL_I2C_Mem_Write(LIS3DH_hi2c, LIS3DH_ADDRESS, CTRL_REG4, 1, &ctrl4, 1, 1000);
    }

    if(hal_status == HAL_TIMEOUT)
    {
        return LIS3DH_TIMEOUT;
    }
    else if (hal_status != HAL_OK)
    {
        return LIS3DH_ERROR;
    }

    // Tiempo para que la nueva escala se estabilice
    HAL_Delay(15);

    return LIS3DH_OK;
}