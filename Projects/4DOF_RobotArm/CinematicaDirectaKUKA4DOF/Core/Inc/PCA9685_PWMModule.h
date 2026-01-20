/**
 * @file PCA9685_PWMModule.h
 * @brief Definiciones y prototipos para el control del módulo PCA9685 mediante I2C en STM32.
 * @author Daniel Ruiz
 * @date Jul 3, 2025
 * @version 1.2.0
 */

#ifndef PCA9685_PWMModule_H
#define PCA9685_PWMModule_H

/**
 * @brief Incluir el encabezado adecuado según la familia STM32 utilizada.
 * Por ejemplo:
 * - Para STM32F1xx: "stm32f1xx_hal.h"
 * - Para STM32F4xx: "stm32f4xx_hal.h"
 */
#include "stm32f4xx_hal.h"
#include "stdbool.h"

/**
 * @brief Dirección I2C por defecto del PCA9685 (ajustar si se modifican pines A0-A5).
 */
#define PCA9685_ADDRESS 0x80

/**
 * @brief Definiciones de registros del PCA9685.
 */
#define PCA9685_MODE1              0x00  /**< Registro MODE1 */
#define PCA9685_PRE_SCALE          0xFE  /**< Registro de preescalado de frecuencia */
#define PCA9685_LED0_ON_L          0x06  /**< Registro LED0 ON LSB */

/**
 * @brief Bits específicos dentro del registro MODE1.
 */
#define PCA9685_MODE1_SLEEP_BIT     4    /**< Bit para entrar/salir del modo de bajo consumo */
#define PCA9685_MODE1_AI_BIT        5    /**< Bit para habilitar auto-incremento de direcciones */
#define PCA9685_MODE1_RESTART_BIT   7    /**< Bit para reinicio del dispositivo */

// Enumeración para estados de retorno
typedef enum {
    PCA9685_OK = 0,         /** Operación exitosa */
    PCA9685_ERROR = 1,      /** Error en la operación */
    PCA9685_TIMEOUT = 2     /** Timeout en la operación */
}PCA9685_Status_t;

/**
 * @brief Estructura para manejar el movimiento suave de un servomotor
 */
typedef struct {
    uint8_t channel;           /**< Canal del servomotor (0-15) */
    float currentAngle;        /**< Ángulo actual del servomotor */
    float targetAngle;         /**< Ángulo objetivo */
    float stepSize;            /**< Tamaño del paso por intervalo */
    uint32_t updateInterval;   /**< Intervalo entre actualizaciones en ms */
    uint32_t lastUpdateTime;   /**< Último tiempo de actualización */
    bool isMoving;             /**< Flag que indica si el servomotor está en movimiento */
} Servo_Smooth_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el módulo PCA9685 con una frecuencia determinada.
 * @param hi2c Puntero al manejador I2C.
 * @param frequency Frecuencia de PWM deseada en Hz (ej. 50 Hz para servos).
 */
PCA9685_Status_t PCA9685_Init(I2C_HandleTypeDef* hi2c, uint16_t frequency);

/**
 * @brief Establece el tiempo de encendido y apagado para un canal PWM.
 * @param Channel Canal del 0 al 15.
 * @param OnTime Valor (0-4095) que indica el momento de activación.
 * @param OffTime Valor (0-4095) que indica el momento de desactivación.
 */
PCA9685_Status_t PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);

/**
 * @brief Controla el ángulo de un servomotor conectado a un canal específico.
 * @param Channel Canal del 0 al 15.
 * @param Angle Ángulo deseado (0° a 180°).
 */
PCA9685_Status_t PCA9685_SetServoAngle(uint8_t Channel, float Angle);

/**
 * @brief Inicializa una estructura de movimiento suave para un servomotor
 * @param servo Puntero a la estructura Servo_Smooth_t
 * @param channel Canal del servomotor (0-15)
 * @param initialAngle Ángulo inicial del servomotor
 * @param updateInterval Intervalo entre actualizaciones en ms (ej. 20ms)
 */
PCA9685_Status_t PCA9685_InitSmoothServo(Servo_Smooth_t* servo, uint8_t channel, float initialAngle, uint32_t updateInterval);

/**
 * @brief Configura un movimiento suave hacia un ángulo objetivo
 * @param servo Puntero a la estructura Servo_Smooth_t
 * @param targetAngle Ángulo objetivo (0° a 180°)
 * @param durationMs Duración total del movimiento en milisegundos
 */
PCA9685_Status_t PCA9685_SetSmoothAngle(Servo_Smooth_t* servo, float targetAngle, uint32_t durationMs);

/**
 * @brief Actualiza el movimiento suave del servomotor (debe llamarse periódicamente)
 * @param servo Puntero a la estructura Servo_Smooth_t
 * @return true si el servomotor alcanzó el ángulo objetivo, false si aún está en movimiento
 */
bool PCA9685_UpdateSmoothServo(Servo_Smooth_t* servo);

/**
 * @brief Mueve un servomotor suavemente de un ángulo a otro
 * @param channel Canal del servomotor (0-15)
 * @param startAngle Ángulo inicial
 * @param endAngle Ángulo final
 * @param durationMs Duración del movimiento en milisegundos
 * @param updateIntervalMs Intervalo entre actualizaciones en milisegundos
 */
PCA9685_Status_t PCA9685_SmoothMove(uint8_t channel, float startAngle, float endAngle, uint32_t durationMs, uint32_t updateIntervalMs);

#ifdef __cplusplus
}
#endif

#endif // PCA9685_PWMModule_H
