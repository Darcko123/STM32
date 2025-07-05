/**
 * @file PCA9685_PWMModule.h
 * @brief Definiciones y prototipos para el control del módulo PCA9685 mediante I2C en STM32.
 * @author Daniel Ruiz
 * @date Jul 3, 2025
 * @version 1.0.0
 */

#ifndef PCA9685_PWMModule_H
#define PCA9685_PWMModule_H

/**
 * @brief Incluir el encabezado adecuado según la familia STM32 utilizada.
 * Por ejemplo:
 * - Para STM32F1xx: "stm32f1xx_hal.h"
 * - Para STM32F4xx: "stm32f4xx_hal.h"
 */
#include "stm32f1xx_hal.h"

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

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el módulo PCA9685 con una frecuencia determinada.
 * @param hi2c Puntero al manejador I2C.
 * @param frequency Frecuencia de PWM deseada en Hz (ej. 50 Hz para servos).
 */
void PCA9685_Init(I2C_HandleTypeDef* hi2c, uint16_t frequency);

/**
 * @brief Modifica un bit específico de un registro del PCA9685.
 * @param Register Dirección del registro a modificar.
 * @param Bit Posición del bit a modificar (0-7).
 * @param Value Valor a establecer (0 o 1).
 */
void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value);

/**
 * @brief Establece el tiempo de encendido y apagado para un canal PWM.
 * @param Channel Canal del 0 al 15.
 * @param OnTime Valor (0-4095) que indica el momento de activación.
 * @param OffTime Valor (0-4095) que indica el momento de desactivación.
 */
void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);

/**
 * @brief Controla el ángulo de un servomotor conectado a un canal específico.
 * @param Channel Canal del 0 al 15.
 * @param Angle Ángulo deseado (0° a 180°).
 */
void PCA9685_SetServoAngle(uint8_t Channel, float Angle);

#ifdef __cplusplus
}
#endif

#endif // PCA9685_PWMModule_H
