#include "BME688.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static I2C_HandleTypeDef*   BME688_hi2c        = NULL;
static uint8_t              BME688_addr        = BME688_ADDRESS_0;
static uint8_t              BME688_Initialized = 0U;
static uint8_t              BME688_variant     = 0U;

/* Coeficientes de calibración de fábrica (registros 0x8A, 0xE1 y 0x00) */
static struct {
    uint16_t par_t1;
    int16_t  par_t2;
    int8_t   par_t3;

    uint16_t par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int16_t  par_p4;
    int16_t  par_p5;
    int8_t   par_p6;
    int8_t   par_p7;
    int16_t  par_p8;
    int16_t  par_p9;
    uint8_t  par_p10;

    uint16_t par_h1;
    uint16_t par_h2;
    int8_t   par_h3;
    int8_t   par_h4;
    int8_t   par_h5;
    uint8_t  par_h6;
    int8_t   par_h7;

    int8_t   par_gh1;
    int16_t  par_gh2;
    int8_t   par_gh3;

    uint8_t  res_heat_range;
    int8_t   res_heat_val;
    int8_t   range_sw_err;
} BME688_calib;


// ============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
// ============================================================================

static BME688_Status_t bme688_WriteReg       (uint8_t reg, uint8_t value);
static BME688_Status_t bme688_ReadRegs       (uint8_t reg, uint8_t* buf, uint16_t len);
static BME688_Status_t bme688_ReadCalibration(void);

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

static BME688_Status_t bme688_WriteReg(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Write(
        BME688_hi2c,
        (uint16_t)(BME688_addr << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1U,
        BME688_TIMEOUT_MS
    );

    if (hal == HAL_TIMEOUT) { return BME688_TIMEOUT; }
    if (hal != HAL_OK)      { return BME688_ERROR;   }

    return BME688_OK;
}

static BME688_Status_t bme688_ReadRegs(uint8_t reg, uint8_t* buf, uint16_t len)
{
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Read(
        BME688_hi2c,
        (uint16_t)(BME688_addr << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        len,
        BME688_TIMEOUT_MS
    );

    if (hal == HAL_TIMEOUT) { return BME688_TIMEOUT; }
    if (hal != HAL_OK)      { return BME688_ERROR;   }

    return BME688_OK;
}

static BME688_Status_t bme688_ReadCalibration(void)
{
    uint8_t buf[BME688_LEN_COEFF_ALL];

    /* Los coeficientes están repartidos en tres bloques no contiguos */
    BME688_Status_t status = bme688_ReadRegs(BME688_REG_COEFF1, &buf[0], BME688_LEN_COEFF1);
    if (status != BME688_OK) { return status; }

    status = bme688_ReadRegs(BME688_REG_COEFF2, &buf[BME688_LEN_COEFF1], BME688_LEN_COEFF2);
    if (status != BME688_OK) { return status; }

    status = bme688_ReadRegs(BME688_REG_COEFF3, &buf[BME688_LEN_COEFF1 + BME688_LEN_COEFF2], BME688_LEN_COEFF3);
    if (status != BME688_OK) { return status; }

    /* Temperatura (little-endian) */
    BME688_calib.par_t1  = (uint16_t)((uint16_t)buf[32] << 8 | buf[31]);
    BME688_calib.par_t2  = (int16_t) ((uint16_t)buf[1]  << 8 | buf[0]);
    BME688_calib.par_t3  = (int8_t)  buf[2];

    /* Presión */
    BME688_calib.par_p1  = (uint16_t)((uint16_t)buf[5]  << 8 | buf[4]);
    BME688_calib.par_p2  = (int16_t) ((uint16_t)buf[7]  << 8 | buf[6]);
    BME688_calib.par_p3  = (int8_t)  buf[8];
    BME688_calib.par_p4  = (int16_t) ((uint16_t)buf[11] << 8 | buf[10]);
    BME688_calib.par_p5  = (int16_t) ((uint16_t)buf[13] << 8 | buf[12]);
    BME688_calib.par_p6  = (int8_t)  buf[15];
    BME688_calib.par_p7  = (int8_t)  buf[14];
    BME688_calib.par_p8  = (int16_t) ((uint16_t)buf[19] << 8 | buf[18]);
    BME688_calib.par_p9  = (int16_t) ((uint16_t)buf[21] << 8 | buf[20]);
    BME688_calib.par_p10 = (uint8_t) buf[22];

    /* Humedad: H1 y H2 son de 12 bits y comparten el byte 24 */
    BME688_calib.par_h1  = (uint16_t)(((uint16_t)buf[25] << 4) | (buf[24] & 0x0FU));
    BME688_calib.par_h2  = (uint16_t)(((uint16_t)buf[23] << 4) | (buf[24] >> 4));
    BME688_calib.par_h3  = (int8_t)  buf[26];
    BME688_calib.par_h4  = (int8_t)  buf[27];
    BME688_calib.par_h5  = (int8_t)  buf[28];
    BME688_calib.par_h6  = (uint8_t) buf[29];
    BME688_calib.par_h7  = (int8_t)  buf[30];

    /* Calentador de gas */
    BME688_calib.par_gh1 = (int8_t)  buf[35];
    BME688_calib.par_gh2 = (int16_t) ((uint16_t)buf[34] << 8 | buf[33]);
    BME688_calib.par_gh3 = (int8_t)  buf[36];

    /* Otros */
    BME688_calib.res_heat_range = (uint8_t)((buf[39] & 0x30U) / 16U);
    BME688_calib.res_heat_val   = (int8_t)buf[37];
    BME688_calib.range_sw_err   = (int8_t)((int8_t)(buf[41] & 0xF0U) / 16);

    return BME688_OK;
}

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
    if (hi2c == NULL) { return BME688_INVALID_PARAM; }

    BME688_Initialized = 0U;
    BME688_hi2c        = hi2c;

    /* Intentar detectar el sensor en las dos direcciones posibles */
    uint8_t id = 0U;
    BME688_addr = BME688_ADDRESS_0;
    BME688_Status_t status = bme688_ReadRegs(BME688_REG_ID, &id, 1U);

    if (status != BME688_OK || id != BME688_CHIP_ID) {
        BME688_addr = BME688_ADDRESS_1;
        status = bme688_ReadRegs(BME688_REG_ID, &id, 1U);
        if (status != BME688_OK || id != BME688_CHIP_ID) {
            return BME688_ERROR;
        }
    }

    /* Soft reset */
    status = bme688_WriteReg(BME688_REG_RESET, BME688_RESET_VALUE);
    if (status != BME688_OK) { return status; }

    /* El BME688 necesita hasta 10 ms para reiniciarse antes de responder por I2C */
    HAL_Delay(BME688_RESET_DELAY_MS);

    /* Confirmar que el sensor volvió a responder tras el reset */
    status = bme688_ReadRegs(BME688_REG_ID, &id, 1U);
    if (status != BME688_OK)  { return status;       }
    if (id != BME688_CHIP_ID) { return BME688_ERROR; }

    /* Variante del sensor (BME680 = gas low, BME688 = gas high) */
    status = bme688_ReadRegs(BME688_REG_VARIANT_ID, &BME688_variant, 1U);
    if (status != BME688_OK) { return status; }

    /* Leer coeficientes de calibración de fábrica */
    status = bme688_ReadCalibration();
    if (status != BME688_OK) { return status; }

    BME688_Initialized = 1U;

    return BME688_OK;
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