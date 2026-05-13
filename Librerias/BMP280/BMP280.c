/**
 * @file BMP280.c
 * @brief Driver I2C para el sensor barométrico BMP280 en STM32.
 *
 * @details Proporciona inicialización, lectura de temperatura y presión con
 *          compensación por coeficientes de calibración de fábrica, y cálculo
 *          de altitud estimada. Compatible con BMP280 (Chip ID 0x58).
 *          Basado en la implementación de sheinz (MIT License, 2016).
 *
 * @author Daniel Ruiz
 * @date Mayo 12, 2026
 * @version 0.1.0
 */

#include "BMP280.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static I2C_HandleTypeDef*  BMP280_hi2c        = NULL;
static uint8_t              BMP280_addr        = BMP280_ADDRESS_0;
static uint8_t              BMP280_Initialized = 0U;

/* Coeficientes de calibración de fábrica (registros 0x88–0x9F) */
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
// ============================================================================

static BMP280_Status_t bmp280_WriteReg      (uint8_t reg, uint8_t value);
static BMP280_Status_t bmp280_ReadRegs      (uint8_t reg, uint8_t* buf, uint16_t len);
static BMP280_Status_t bmp280_ReadCalibration(void);
static int32_t         bmp280_CompensateTemp (int32_t adc_T, int32_t* fine_temp);
static uint32_t        bmp280_CompensatePress(int32_t adc_P, int32_t fine_temp);

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

static BMP280_Status_t bmp280_WriteReg(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Write(
        BMP280_hi2c,
        (uint16_t)(BMP280_addr << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1U,
        BMP280_TIMEOUT_MS
    );

    if (hal == HAL_TIMEOUT) { return BMP280_TIMEOUT; }
    if (hal != HAL_OK)      { return BMP280_ERROR;   }

    return BMP280_OK;
}

static BMP280_Status_t bmp280_ReadRegs(uint8_t reg, uint8_t* buf, uint16_t len)
{
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Read(
        BMP280_hi2c,
        (uint16_t)(BMP280_addr << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        len,
        BMP280_TIMEOUT_MS
    );

    if (hal == HAL_TIMEOUT) { return BMP280_TIMEOUT; }
    if (hal != HAL_OK)      { return BMP280_ERROR;   }

    return BMP280_OK;
}

static BMP280_Status_t bmp280_ReadCalibration(void)
{
    uint8_t buf[24];

    BMP280_Status_t status = bmp280_ReadRegs(0x88U, buf, 24U);
    if (status != BMP280_OK) { return status; }

    /* Los coeficientes están en formato little-endian */
    dig_T1 = (uint16_t)((uint16_t)buf[1]  << 8 | buf[0]);
    dig_T2 = (int16_t) ((uint16_t)buf[3]  << 8 | buf[2]);
    dig_T3 = (int16_t) ((uint16_t)buf[5]  << 8 | buf[4]);
    dig_P1 = (uint16_t)((uint16_t)buf[7]  << 8 | buf[6]);
    dig_P2 = (int16_t) ((uint16_t)buf[9]  << 8 | buf[8]);
    dig_P3 = (int16_t) ((uint16_t)buf[11] << 8 | buf[10]);
    dig_P4 = (int16_t) ((uint16_t)buf[13] << 8 | buf[12]);
    dig_P5 = (int16_t) ((uint16_t)buf[15] << 8 | buf[14]);
    dig_P6 = (int16_t) ((uint16_t)buf[17] << 8 | buf[16]);
    dig_P7 = (int16_t) ((uint16_t)buf[19] << 8 | buf[18]);
    dig_P8 = (int16_t) ((uint16_t)buf[21] << 8 | buf[20]);
    dig_P9 = (int16_t) ((uint16_t)buf[23] << 8 | buf[22]);

    return BMP280_OK;
}

/* Algoritmo de compensación de temperatura del datasheet BMP280.
 * Retorna temperatura en centésimas de °C (e.g., 2350 = 23.50 °C).
 * Calcula fine_temp, requerido por bmp280_CompensatePress. */
static int32_t bmp280_CompensateTemp(int32_t adc_T, int32_t* fine_temp)
{
    int32_t var1, var2;

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * (int32_t)dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)dig_T1) *
              ((adc_T >> 4) - (int32_t)dig_T1)) >> 12) * (int32_t)dig_T3) >> 14;

    *fine_temp = var1 + var2;
    return (*fine_temp * 5 + 128) >> 8;
}

/* Algoritmo de compensación de presión del datasheet BMP280.
 * Retorna presión en formato Q24.8 Pa (dividir por 256 para obtener Pa). */
static uint32_t bmp280_CompensatePress(int32_t adc_P, int32_t fine_temp)
{
    int64_t var1, var2, p;

    var1 = (int64_t)fine_temp - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) { return 0U; }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)dig_P7 << 4);
    return (uint32_t)p;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el driver BMP280 con configuración por defecto.
 *
 * @param[in] hi2c   Puntero al handle de I2C.
 * @return BMP280_Status_t
 */
BMP280_Status_t BMP280_Init(I2C_HandleTypeDef* hi2c)
{
    if (hi2c == NULL) { return BMP280_INVALID_PARAM; }

    BMP280_hi2c = hi2c;

    /* Intentar detectar el sensor en las dos direcciones posibles */
    uint8_t id = 0U;
    BMP280_addr = BMP280_ADDRESS_0;
    BMP280_Status_t status = bmp280_ReadRegs(BMP280_REG_ID, &id, 1U);

    if (status != BMP280_OK || id != BMP280_CHIP_ID) {
        BMP280_addr = BMP280_ADDRESS_1;
        status = bmp280_ReadRegs(BMP280_REG_ID, &id, 1U);
        if (status != BMP280_OK || id != BMP280_CHIP_ID) {
            return BMP280_ERROR;
        }
    }

    /* Soft reset */
    status = bmp280_WriteReg(BMP280_REG_RESET, BMP280_RESET_VALUE);
    if (status != BMP280_OK) { return status; }

    /* Esperar a que termine la copia de coeficientes desde NVM */
    uint8_t stat = 1U;
    do {
        HAL_Delay(1U);
        status = bmp280_ReadRegs(BMP280_REG_STATUS, &stat, 1U);
        if (status != BMP280_OK) { return status; }
    } while (stat & 0x01U);

    /* Leer coeficientes de calibración de fábrica */
    status = bmp280_ReadCalibration();
    if (status != BMP280_OK) { return status; }

    /* Configuración por defecto: Filter x16, Standby 0.5ms */
    uint8_t cfg = (uint8_t)(((uint8_t)BMP280_STANDBY_0_5MS << 5) | ((uint8_t)BMP280_FILTER_16 << 2));
    status = bmp280_WriteReg(BMP280_REG_CONFIG, cfg);
    if (status != BMP280_OK) { return status; }

    /* Configuración por defecto: Temp x2, Press x16, Mode Normal */
    uint8_t ctrl = (uint8_t)(((uint8_t)BMP280_OS_X2 << 5) | ((uint8_t)BMP280_OS_X16 << 2) | 0x03U);
    status = bmp280_WriteReg(BMP280_REG_CTRL, ctrl);
    if (status != BMP280_OK) { return status; }

    BMP280_Initialized = 1U;

    return BMP280_OK;
}

/**
 * @brief Lee temperatura, presión y altitud compensadas del BMP280.
 *
 * @param[out] data Puntero a la estructura donde se guardarán los datos.
 * @return BMP280_Status_t
 */
BMP280_Status_t BMP280_ReadData(BMP280_Data_t* data)
{
    if (data == NULL)             { return BMP280_INVALID_PARAM;   }
    if (BMP280_Initialized != 1U) { return BMP280_NOT_INITIALIZED; }

    /* Leer bloque de 6 bytes: presión (0xF7–0xF9) y temperatura (0xFA–0xFC) */
    uint8_t raw[6];
    BMP280_Status_t status = bmp280_ReadRegs(BMP280_REG_PRESSURE, raw, 6U);
    if (status != BMP280_OK) { return status; }

    int32_t adc_P = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | (raw[5] >> 4));

    int32_t  fine_temp;
    int32_t  temp_c100  = bmp280_CompensateTemp(adc_T, &fine_temp);
    uint32_t press_q248 = bmp280_CompensatePress(adc_P, fine_temp);

    data->temperatura = (float)temp_c100  / 100.0f;
    data->presion     = (float)press_q248 / 25600.0f; // Convertir Pa a hPa
    data->altitud     = 44330.0f * (1.0f - powf(data->presion / BMP280_SEA_LEVEL_HPA, 0.1903f));

    return BMP280_OK;
}
