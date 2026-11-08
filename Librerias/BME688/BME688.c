#include "BME688.h"

// ============================================================================
// MACROS PRIVADAS
// ============================================================================

/* Registros de medición y control */
#define BME688_REG_FIELD0       0x1DU       /**< Inicio del bloque de datos del campo 0 (17 bytes) */
#define BME688_REG_RES_HEAT0    0x5AU       /**< Resistencia objetivo del calentador, perfil 0 */
#define BME688_REG_GAS_WAIT0    0x64U       /**< Tiempo de calentamiento, perfil 0 */
#define BME688_REG_CTRL_GAS_0   0x70U       /**< bit[3]: heat_off */
#define BME688_REG_CTRL_GAS_1   0x71U       /**< odr3[7], run_gas[5:4], nb_conv[3:0] */
#define BME688_REG_CTRL_HUM     0x72U       /**< osrs_h[2:0] (efectivo tras escribir ctrl_meas) */
#define BME688_REG_CTRL_MEAS    0x74U       /**< osrs_t[7:5], osrs_p[4:2], mode[1:0] */
#define BME688_REG_CONFIG       0x75U       /**< odr20[7:5], filter[4:2] */

#define BME688_LEN_FIELD        17U

#define BME688_MODE_MSK         0x03U
#define BME688_MODE_SLEEP       0x00U
#define BME688_MODE_FORCED      0x01U

#define BME688_NEW_DATA_MSK     0x80U
#define BME688_GAS_RANGE_MSK    0x0FU
#define BME688_GASM_VALID_MSK   0x20U
#define BME688_HEAT_STAB_MSK    0x10U

#define BME688_VARIANT_GAS_HIGH 0x01U       /**< BME688 (0x00 = BME680) */
#define BME688_RUN_GAS_LOW      0x01U       /**< run_gas para BME680 */
#define BME688_RUN_GAS_HIGH     0x02U       /**< run_gas para BME688 */

#define BME688_POLL_PERIOD_MS   10U         /**< Espera entre sondeos de new_data / modo [ms] */
#define BME688_POLL_RETRIES     5U

/* Configuración por defecto (ejemplo forced mode de Bosch): T x2, P x1, H x16, filtro off, sin ODR */
#define BME688_OS_T             2U          /**< osrs_t: x2  */
#define BME688_OS_P             1U          /**< osrs_p: x1  */
#define BME688_OS_H             5U          /**< osrs_h: x16 */
#define BME688_FILTER           0U          /**< Filtro IIR desactivado */

/* Perfil del calentador de gas: ajustar según la aplicación */
#define BME688_HEATER_TEMP_C    300U        /**< Temperatura objetivo [°C] (máx. 400) */
#define BME688_HEATER_DUR_MS    100U        /**< Duración del calentamiento [ms] (máx. 4032) */
#define BME688_AMB_TEMP_C       25          /**< Temperatura ambiente supuesta para calcular res_heat [°C] */

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

static BME688_Status_t bme688_WriteReg         (uint8_t reg, uint8_t value);
static BME688_Status_t bme688_ReadRegs         (uint8_t reg, uint8_t* buf, uint16_t len);
static BME688_Status_t bme688_UpdateReg        (uint8_t reg, uint8_t mask, uint8_t value);
static BME688_Status_t bme688_ReadCalibration  (void);
static BME688_Status_t bme688_SetMode          (uint8_t mode);
static BME688_Status_t bme688_Configure        (void);
static BME688_Status_t bme688_ReadField        (BME688_Data_t* data);
static uint32_t        bme688_GetMeasDuration  (void);
static uint8_t         bme688_CalcResHeat      (uint16_t temp);
static uint8_t         bme688_CalcGasWait      (uint16_t dur);
static float           bme688_CompensateTemp   (uint32_t adc_T, float* t_fine);
static float           bme688_CompensatePress  (uint32_t adc_P, float t_fine);
static float           bme688_CompensateHum    (uint16_t adc_H, float temp);
static float           bme688_CompensateGasLow (uint16_t adc_G, uint8_t range);
static float           bme688_CompensateGasHigh(uint16_t adc_G, uint8_t range);

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

/* Lectura-modificación-escritura: solo cambia los bits de @p mask */
static BME688_Status_t bme688_UpdateReg(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current;

    BME688_Status_t status = bme688_ReadRegs(reg, &current, 1U);
    if (status != BME688_OK) { return status; }

    return bme688_WriteReg(reg, (uint8_t)((current & (uint8_t)~mask) | (value & mask)));
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

/* El modo solo puede cambiarse desde sleep: forzar sleep y luego escribir el modo pedido */
static BME688_Status_t bme688_SetMode(uint8_t mode)
{
    uint8_t ctrl    = 0U;
    uint8_t retries = BME688_POLL_RETRIES;
    BME688_Status_t status;

    while (1) {
        status = bme688_ReadRegs(BME688_REG_CTRL_MEAS, &ctrl, 1U);
        if (status != BME688_OK) { return status; }

        if ((ctrl & BME688_MODE_MSK) == BME688_MODE_SLEEP) { break; }
        if (retries-- == 0U)                                { return BME688_TIMEOUT; }

        status = bme688_WriteReg(BME688_REG_CTRL_MEAS, (uint8_t)(ctrl & (uint8_t)~BME688_MODE_MSK));
        if (status != BME688_OK) { return status; }

        HAL_Delay(BME688_POLL_PERIOD_MS);
    }

    if (mode == BME688_MODE_SLEEP) { return BME688_OK; }

    return bme688_WriteReg(BME688_REG_CTRL_MEAS, (uint8_t)((ctrl & (uint8_t)~BME688_MODE_MSK) | mode));
}

/* Oversampling, filtro y perfil 0 del calentador para forced mode (requiere modo sleep) */
static BME688_Status_t bme688_Configure(void)
{
    const uint8_t run_gas = (BME688_variant == BME688_VARIANT_GAS_HIGH) ? BME688_RUN_GAS_HIGH : BME688_RUN_GAS_LOW;

    const struct { uint8_t reg; uint8_t mask; uint8_t value; } cfg[] = {
        /* osrs_h se aplica al escribir ctrl_meas, por eso va antes */
        { BME688_REG_CTRL_HUM,   0x07U, BME688_OS_H },
        { BME688_REG_CTRL_MEAS,  0xFCU, (uint8_t)((BME688_OS_T << 5) | (BME688_OS_P << 2)) },
        /* odr20 = 0 y filtro */
        { BME688_REG_CONFIG,     0xFCU, (uint8_t)(BME688_FILTER << 2) },
        /* Perfil 0 del calentador */
        { BME688_REG_RES_HEAT0,  0xFFU, bme688_CalcResHeat(BME688_HEATER_TEMP_C) },
        { BME688_REG_GAS_WAIT0,  0xFFU, bme688_CalcGasWait(BME688_HEATER_DUR_MS) },
        /* heat_off = 0 (calentador habilitado) */
        { BME688_REG_CTRL_GAS_0, 0x08U, 0x00U },
        /* odr3 = 1 (sin ODR), run_gas según variante, nb_conv = 0 (perfil 0) */
        { BME688_REG_CTRL_GAS_1, 0xBFU, (uint8_t)(0x80U | (run_gas << 4)) },
    };

    for (uint8_t i = 0U; i < (uint8_t)(sizeof(cfg) / sizeof(cfg[0])); i++) {
        BME688_Status_t status = bme688_UpdateReg(cfg[i].reg, cfg[i].mask, cfg[i].value);
        if (status != BME688_OK) { return status; }
    }

    return BME688_OK;
}

/* Lee y compensa el campo 0; sondea new_data si la medición aún no termina */
static BME688_Status_t bme688_ReadField(BME688_Data_t* data)
{
    uint8_t buf[BME688_LEN_FIELD];

    for (uint8_t tries = BME688_POLL_RETRIES; tries > 0U; tries--) {
        BME688_Status_t status = bme688_ReadRegs(BME688_REG_FIELD0, buf, BME688_LEN_FIELD);
        if (status != BME688_OK) { return status; }

        if ((buf[0] & BME688_NEW_DATA_MSK) == 0U) {
            HAL_Delay(BME688_POLL_PERIOD_MS);
            continue;
        }

        uint32_t adc_P = ((uint32_t)buf[2] << 12) | ((uint32_t)buf[3] << 4) | (buf[4] >> 4);
        uint32_t adc_T = ((uint32_t)buf[5] << 12) | ((uint32_t)buf[6] << 4) | (buf[7] >> 4);
        uint16_t adc_H = (uint16_t)(((uint16_t)buf[8] << 8) | buf[9]);

        /* El BME688 entrega el gas en los bytes 15–16; el BME680 en los 13–14 */
        uint8_t  gas_msb = (BME688_variant == BME688_VARIANT_GAS_HIGH) ? buf[15] : buf[13];
        uint8_t  gas_lsb = (BME688_variant == BME688_VARIANT_GAS_HIGH) ? buf[16] : buf[14];
        uint16_t adc_G   = (uint16_t)(((uint16_t)gas_msb << 2) | (gas_lsb >> 6));
        uint8_t  range   = gas_lsb & BME688_GAS_RANGE_MSK;

        float t_fine;
        data->temperatura = bme688_CompensateTemp(adc_T, &t_fine);
        data->presion     = bme688_CompensatePress(adc_P, t_fine) / 100.0f; // Convertir Pa a hPa
        data->humedad     = bme688_CompensateHum(adc_H, data->temperatura);
        data->gas         = (BME688_variant == BME688_VARIANT_GAS_HIGH)
                          ? bme688_CompensateGasHigh(adc_G, range)
                          : bme688_CompensateGasLow(adc_G, range);

        /* La resistencia de gas solo es fiable si la medición es válida y el calentador se estabilizó */
        data->gas_valido  = ((gas_lsb & (BME688_GASM_VALID_MSK | BME688_HEAT_STAB_MSK)) ==
                             (BME688_GASM_VALID_MSK | BME688_HEAT_STAB_MSK)) ? 1U : 0U;

        return BME688_OK;
    }

    return BME688_TIMEOUT;
}

/* Duración de una medición forced (TPH + gas + despertar) más el calentamiento, en ms */
static uint32_t bme688_GetMeasDuration(void)
{
    static const uint8_t os_to_cycles[6] = { 0U, 1U, 2U, 4U, 8U, 16U };

    uint32_t dur_us = (uint32_t)(os_to_cycles[BME688_OS_T] + os_to_cycles[BME688_OS_P] + os_to_cycles[BME688_OS_H]) * 1963U;
    dur_us += 477U * 4U;    /* Conmutación TPH */
    dur_us += 477U * 5U;    /* Medición de gas */
    dur_us += 1000U;        /* Despertar en forced mode */

    return ((dur_us + 999U) / 1000U) + BME688_HEATER_DUR_MS;
}

/* Valor de res_heat_x para alcanzar @p temp [°C] (algoritmo float de Bosch) */
static uint8_t bme688_CalcResHeat(uint16_t temp)
{
    if (temp > 400U) { temp = 400U; }

    float var1 = ((float)BME688_calib.par_gh1 / 16.0f) + 49.0f;
    float var2 = (((float)BME688_calib.par_gh2 / 32768.0f) * 0.0005f) + 0.00235f;
    float var3 = (float)BME688_calib.par_gh3 / 1024.0f;
    float var4 = var1 * (1.0f + (var2 * (float)temp));
    float var5 = var4 + (var3 * (float)BME688_AMB_TEMP_C);

    return (uint8_t)(3.4f * ((var5 * (4.0f / (4.0f + (float)BME688_calib.res_heat_range)) *
                              (1.0f / (1.0f + ((float)BME688_calib.res_heat_val * 0.002f)))) - 25.0f));
}

/* Codifica @p dur [ms] en gas_wait_x: 6 bits de valor y 2 bits de factor (x1, x4, x16, x64) */
static uint8_t bme688_CalcGasWait(uint16_t dur)
{
    if (dur >= 0xFC0U) { return 0xFFU; }

    uint8_t factor = 0U;
    while (dur > 0x3FU) {
        dur /= 4U;
        factor++;
    }

    return (uint8_t)(dur + (factor * 64U));
}

/* Retorna temperatura en °C y calcula t_fine, requerido por la compensación de presión */
static float bme688_CompensateTemp(uint32_t adc_T, float* t_fine)
{
    float var1 = (((float)adc_T / 16384.0f) - ((float)BME688_calib.par_t1 / 1024.0f)) * (float)BME688_calib.par_t2;
    float var2 = ((float)adc_T / 131072.0f) - ((float)BME688_calib.par_t1 / 8192.0f);
    var2 = (var2 * var2) * ((float)BME688_calib.par_t3 * 16.0f);

    *t_fine = var1 + var2;
    return *t_fine / 5120.0f;
}

/* Retorna presión en Pa */
static float bme688_CompensatePress(uint32_t adc_P, float t_fine)
{
    float var1 = (t_fine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * ((float)BME688_calib.par_p6 / 131072.0f);
    var2 = var2 + (var1 * (float)BME688_calib.par_p5 * 2.0f);
    var2 = (var2 / 4.0f) + ((float)BME688_calib.par_p4 * 65536.0f);
    var1 = ((((float)BME688_calib.par_p3 * var1 * var1) / 16384.0f) + ((float)BME688_calib.par_p2 * var1)) / 524288.0f;
    var1 = (1.0f + (var1 / 32768.0f)) * (float)BME688_calib.par_p1;

    /* Evitar división por cero */
    if ((int32_t)var1 == 0) { return 0.0f; }

    float p    = 1048576.0f - (float)adc_P;
    p          = ((p - (var2 / 4096.0f)) * 6250.0f) / var1;
    var1       = ((float)BME688_calib.par_p9 * p * p) / 2147483648.0f;
    var2       = p * ((float)BME688_calib.par_p8 / 32768.0f);
    float var3 = (p / 256.0f) * (p / 256.0f) * (p / 256.0f) * ((float)BME688_calib.par_p10 / 131072.0f);

    return p + ((var1 + var2 + var3 + ((float)BME688_calib.par_p7 * 128.0f)) / 16.0f);
}

/* Retorna humedad relativa en %RH, acotada a 0–100 */
static float bme688_CompensateHum(uint16_t adc_H, float temp)
{
    float var1 = (float)adc_H - (((float)BME688_calib.par_h1 * 16.0f) + (((float)BME688_calib.par_h3 / 2.0f) * temp));
    float var2 = var1 * (((float)BME688_calib.par_h2 / 262144.0f) *
                         (1.0f + (((float)BME688_calib.par_h4 / 16384.0f) * temp) +
                          (((float)BME688_calib.par_h5 / 1048576.0f) * temp * temp)));
    float var3 = (float)BME688_calib.par_h6 / 16384.0f;
    float var4 = (float)BME688_calib.par_h7 / 2097152.0f;
    float hum  = var2 + ((var3 + (var4 * temp)) * var2 * var2);

    if (hum > 100.0f) { return 100.0f; }
    if (hum < 0.0f)   { return 0.0f;   }

    return hum;
}

/* Resistencia de gas del BME680 (variante gas low) en Ω */
static float bme688_CompensateGasLow(uint16_t adc_G, uint8_t range)
{
    static const float k1[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, -0.8f, 0.0f, 0.0f, -0.2f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f
    };
    static const float k2[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.7f, 0.0f, -0.8f, -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };

    float var1 = 1340.0f + (5.0f * (float)BME688_calib.range_sw_err);
    float var2 = var1 * (1.0f + (k1[range] / 100.0f));
    float var3 = 1.0f + (k2[range] / 100.0f);

    return 1.0f / (var3 * 0.000000125f * (float)(1UL << range) * ((((float)adc_G - 512.0f) / var2) + 1.0f));
}

/* Resistencia de gas del BME688 (variante gas high) en Ω */
static float bme688_CompensateGasHigh(uint16_t adc_G, uint8_t range)
{
    uint32_t var1 = 262144UL >> range;
    int32_t  var2 = 4096 + (((int32_t)adc_G - 512) * 3);

    return 1000000.0f * (float)var1 / (float)var2;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

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

    /* La configuración solo se acepta en modo sleep */
    status = bme688_SetMode(BME688_MODE_SLEEP);
    if (status != BME688_OK) { return status; }

    status = bme688_Configure();
    if (status != BME688_OK) { return status; }

    BME688_Initialized = 1U;

    return BME688_OK;
}

/**
 * @brief Realiza una medición en forced mode y retorna temperatura, presión, humedad y gas compensados.
 *
 * @param[out] data Puntero a la estructura donde se guardarán los datos.
 * @return BME688_Status_t
 */
BME688_Status_t BME688_Get(BME688_Data_t* data)
{
    if (data == NULL)             { return BME688_INVALID_PARAM;   }
    if (BME688_Initialized != 1U) { return BME688_NOT_INITIALIZED; }

    /* Disparar una medición; al terminar el sensor vuelve solo a sleep */
    BME688_Status_t status = bme688_SetMode(BME688_MODE_FORCED);
    if (status != BME688_OK) { return status; }

    HAL_Delay(bme688_GetMeasDuration());

    return bme688_ReadField(data);
}
