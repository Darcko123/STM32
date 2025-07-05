/**
 * @file PCA9685_PWMModule.c
 * @brief Implementación de la librería para el control del módulo PCA9685 mediante I2C en STM32.
 * @author Daniel Ruiz
 * @date Jul 3, 2025
 * @version 1.0.0
 */

#include "PCA9685_PWMModule.h"

static I2C_HandleTypeDef* PCA9685_hi2c;  /**< Manejador de la interfaz I2C utilizado para comunicarse con el PCA9685 */

/**
 * @brief Modifica un solo bit de un registro del PCA9685.
 * @param Register Dirección del registro a modificar.
 * @param Bit Posición del bit a modificar (0-7).
 * @param Value Valor a establecer en el bit (0 o 1).
 */
void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
    uint8_t readValue;
    HAL_I2C_Mem_Read(PCA9685_hi2c, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
    if (Value == 0)
        readValue &= ~(1 << Bit);
    else
        readValue |= (1 << Bit);
    HAL_I2C_Mem_Write(PCA9685_hi2c, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
    HAL_Delay(1);
}

/**
 * @brief Configura la frecuencia de salida PWM del módulo PCA9685.
 * @param frequency Frecuencia deseada en Hz (24 - 1526 Hz).
 */
void PCA9685_SetPWMFrequency(uint16_t frequency)
{
    uint8_t prescale;
    if (frequency >= 1526) prescale = 0x03;
    else if (frequency <= 24) prescale = 0xFF;
    else prescale = 25000000 / (4096 * frequency); // Oscilador interno de 25 MHz

    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);  // Entrar a modo sleep
    HAL_I2C_Mem_Write(PCA9685_hi2c, PCA9685_ADDRESS, PCA9685_PRE_SCALE, 1, &prescale, 1, 10);
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);  // Salir de modo sleep
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1); // Reiniciar
}

/**
 * @brief Inicializa el módulo PCA9685 con una frecuencia específica.
 * @param hi2c Puntero al manejador I2C.
 * @param frequency Frecuencia de PWM deseada (ej. 50 Hz para servos).
 */
void PCA9685_Init(I2C_HandleTypeDef* hi2c, uint16_t frequency)
{
    PCA9685_hi2c = hi2c;
    PCA9685_SetPWMFrequency(frequency);
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1); // Auto-Increment
}

/**
 * @brief Establece el tiempo de encendido y apagado para un canal específico.
 * @param Channel Canal del 0 al 15.
 * @param OnTime Tiempo en el cual la señal se activa (0-4095).
 * @param OffTime Tiempo en el cual la señal se desactiva (0-4095).
 */
void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
    uint8_t registerAddress = PCA9685_LED0_ON_L + (4 * Channel);
    uint8_t pwm[4];

    pwm[0] = OnTime & 0xFF;
    pwm[1] = OnTime >> 8;
    pwm[2] = OffTime & 0xFF;
    pwm[3] = OffTime >> 8;

    HAL_I2C_Mem_Write(PCA9685_hi2c, PCA9685_ADDRESS, registerAddress, 1, pwm, 4, 10);
}

/**
 * @brief Configura el ángulo de un servomotor conectado a un canal.
 * @param Channel Canal del 0 al 15.
 * @param Angle Ángulo deseado entre 0° y 180°.
 */
void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
    float Value;
    // Conversión del ángulo al valor PWM correspondiente entre 0.5ms (102.4) y 2.5ms (511.9)
    Value = (Angle * (511.9f - 102.4f) / 180.0f) + 102.4f;
    PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}
