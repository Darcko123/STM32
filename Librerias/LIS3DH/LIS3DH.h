/**
 * @file LIS3DH.h
 * @brief    Librería para la comunicación y lectura de datos de un acelerómetro LIS3DH
 * 
 * @details Esta librería proporciona funciones para inicializar y leer datos del 
 *  acelerómetro LIS3DH mediante la interfaz I2C en un microcontrolador STM32.
 *  Permite obtener valores de aceleración y giroscopio en los tres ejes (X, Y, Z).
 *  
 * @author   Daniel Ruiz
 * @date     Octubre 15, 2026
 * @version  1.0.0
 *  
 */

#ifndef LIS3DH_H_
#define LIS3DH_H_

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdint.h>

// ============================================================================
// DEFINICIONES DE CONSTANTES
// ============================================================================

#define LIS3DH_ADDRESS          (0x18 << 1) /**< Dirección I2C (SA0 = GND). Usar (0x19 << 1) si SA0 = VDD. */
#define LIS3DH_WHO_AM_I_CONTENT 0x33        /**< Valor esperado del registro WHO_AM_I. */
#define LIS3DH_GRAVITY_STANDARD 9.80665f    /**< Gravedad estándar (m/s²) para convertir 'g' a m/s². */

#define TEMP_CFG_REG        0x1F    /**< Configuración del ADC auxiliar y sensor de temperatura. */
#define WHO_AM_I_REG        0x0F    /**< Registro de identificación del dispositivo. */
#define CTRL_REG1           0x20    /**< ODR, modo bajo consumo y habilitación de ejes. */
#define CTRL_REG3           0x22    /**< Enrutamiento de interrupciones al pin INT1. */
#define CTRL_REG4           0x23    /**< BDU, escala completa y modo alta resolución. */
#define OUT_X_L_REG         0x28    /**< Primer registro de datos de aceleración (X_L, X_H, Y_L, Y_H, Z_L, Z_H). */

#define LIS3DH_AUTO_INC     0x80    /**< Bit 7 de la subdirección I2C: auto-incremento en lecturas multibyte. */

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del LIS3DH.
 */
typedef enum {
    LIS3DH_OK              = 0,    /**< Operación exitosa */
    LIS3DH_ERROR           = 1,    /**< Error en la operación */
    LIS3DH_TIMEOUT         = 2,    /**< Timeout en la operación */
    LIS3DH_NOT_INITIALIZED = 3,	/**< Módulo no inicializado */
    LIS3DH_INVALID_PARAM   = 4     /**< Parámetro inválido */
}LIS3DH_Status_t;

/**
 * @brief Datos de medición del acelerómetro del LIS3DH.
 */
typedef struct {
    float x;    /**< Valor de aceleración en el eje X. */
    float y;    /**< Valor de aceleración en el eje Y. */
    float z;    /**< Valor de aceleración en el eje Z. */
} LIS3DH_Accel_t;

/**
 * @brief Escala completa del acelerómetro (bits FS[1:0] de CTRL_REG4).
 */
typedef enum {
    LIS3DH_RANGE_2G  = 0,   /**< ±2g */
    LIS3DH_RANGE_4G  = 1,   /**< ±4g */
    LIS3DH_RANGE_8G  = 2,   /**< ±8g */
    LIS3DH_RANGE_16G = 3    /**< ±16g */
} LIS3DH_Range_t;

/**
 * @brief Tasa de muestreo del acelerómetro (bits ODR[3:0] de CTRL_REG1).
 */
typedef enum {
    LIS3DH_DATARATE_POWERDOWN       = 0,    /**< Modo apagado */
    LIS3DH_DATARATE_1HZ             = 1,    /**< 1 Hz */
    LIS3DH_DATARATE_10HZ            = 2,    /**< 10 Hz */
    LIS3DH_DATARATE_25HZ            = 3,    /**< 25 Hz */
    LIS3DH_DATARATE_50HZ            = 4,    /**< 50 Hz */
    LIS3DH_DATARATE_100HZ           = 5,    /**< 100 Hz */
    LIS3DH_DATARATE_200HZ           = 6,    /**< 200 Hz */
    LIS3DH_DATARATE_400HZ           = 7,    /**< 400 Hz */
    LIS3DH_DATARATE_LOWPOWER_1K6HZ  = 8,    /**< 1.6 kHz (solo modo bajo consumo) */
    LIS3DH_DATARATE_1K344HZ_LP5K376 = 9     /**< 1.344 kHz (normal/HR) o 5.376 kHz (bajo consumo) */
} LIS3DH_DataRate_t;

/**
 * @brief Descripción del sensor según su configuración actual.
 */
typedef struct {
    const char *name;       /**< Nombre del sensor. */
    int32_t     version;    /**< Versión del driver. */
    float       max_value;  /**< Valor máximo medible (m/s²). */
    float       min_value;  /**< Valor mínimo medible (m/s²). */
    float       resolution; /**< Resolución (m/s² por dígito). */
    uint32_t    min_delay;  /**< Periodo de muestreo en µs (0 si está apagado). */
} LIS3DH_Sensor_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el LIS3DH.
 * @details Configura los registros básicos del LIS3DH para activar el dispositivo
 *          y establecer parámetros predeterminados de muestreo.
 * 
 * @param[in] hi2c Puntero al manejador I2C.
 *
 * @return LIS3DH_Status_t Estado de la operación
 */
LIS3DH_Status_t LIS3DH_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief Desinicializa el sensor LIS3DH, liberando recursos y marcando el módulo como no inicializado.
 * 
 * @return LIS3DH_Status_t Siempre retorna LIS3DH_OK
 */
LIS3DH_Status_t LIS3DH_DeInit(void);

/**
 * @brief Lee los valores del acelerómetro y los convierte a unidades 'g'.
 *
 * @param[out] accel Puntero a la estructura donde se almacenarán los valores de aceleración en 'g'.
 *
 * @return LIS3DH_Status_t Estado de la operación
 */
LIS3DH_Status_t LIS3DH_GetAccel(LIS3DH_Accel_t *accel);

/**
 * @brief Configura la escala completa del acelerómetro.
 *
 * @param[in] range Escala deseada (±2g, ±4g, ±8g o ±16g).
 *
 * @return LIS3DH_Status_t Estado de la operación
 */
LIS3DH_Status_t LIS3DH_SetRange(LIS3DH_Range_t range);

/**
 * @brief Obtiene la escala completa del acelerómetro.
 *
 * @param[out] range Puntero donde se almacenará la escala actual.
 *
 * @return LIS3DH_Status_t Estado de la operación
 */
LIS3DH_Status_t LIS3DH_GetRange(LIS3DH_Range_t *range);

/**
 * @brief Configura la tasa de muestreo del acelerómetro.
 *
 * @param[in] dataRate Tasa de muestreo deseada.
 *
 * @return LIS3DH_Status_t Estado de la operación
 */
LIS3DH_Status_t LIS3DH_SetDataRate(LIS3DH_DataRate_t dataRate);

/**
 * @brief Obtiene la tasa de muestreo del acelerómetro.
 *
 * @param[out] dataRate Puntero donde se almacenará la tasa de muestreo actual.
 *
 * @return LIS3DH_Status_t Estado de la operación
 */
LIS3DH_Status_t LIS3DH_GetDataRate(LIS3DH_DataRate_t *dataRate);

/**
 * @brief Lee los valores del acelerómetro y los convierte a m/s².
 *
 * @param[out] accel Puntero a la estructura donde se almacenarán los valores de aceleración en m/s².
 *
 * @return LIS3DH_Status_t Estado de la operación
 */
LIS3DH_Status_t LIS3DH_GetEvent(LIS3DH_Accel_t *accel);

/**
 * @brief Obtiene la descripción del sensor (rango, resolución y periodo de muestreo).
 *
 * @param[out] sensor Puntero a la estructura donde se almacenará la descripción.
 *
 * @return LIS3DH_Status_t Estado de la operación
 */
LIS3DH_Status_t LIS3DH_GetSensor(LIS3DH_Sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* LIS3DH_H_ */
