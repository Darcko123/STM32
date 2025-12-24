/**
 * @file PCA9685_PWMModule.c
 * @brief Implementación de la librería para el control del módulo PCA9685 mediante I2C en STM32.
 * @author Daniel Ruiz
 * @date Jul 3, 2025
 * @version 1.1.0
 */

#include "PCA9685_PWMModule.h"
#include <math.h>

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

    // Asegurar que el ángulo esté dentro del rango
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 180.0f) Angle = 180.0f;

    // Conversión del ángulo al valor PWM correspondiente entre 0.5ms (102.4) y 2.5ms (511.9)
    Value = (Angle * (511.9f - 102.4f) / 180.0f) + 102.4f;
    PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}

/**
 * @brief Inicializa una estructura de movimiento suave para un servomotor
 * @param servo Puntero a la estructura Servo_Smooth_t
 * @param channel Canal del servomotor (0-15)
 * @param initialAngle Ángulo inicial del servomotor
 * @param updateInterval Intervalo entre actualizaciones en ms (ej. 20ms)
 */
void PCA9685_InitSmoothServo(Servo_Smooth_t* servo, uint8_t channel, float initialAngle, uint32_t updateInterval)
{
    servo->channel = channel;
    servo->currentAngle = initialAngle;
    servo->targetAngle = initialAngle;
    servo->stepSize = 0.0f;
    servo->updateInterval = updateInterval;
    servo->lastUpdateTime = HAL_GetTick();
    servo->isMoving = false;

    // Establecer el ángulo inicial
    PCA9685_SetServoAngle(channel, initialAngle);
}

/**
 * @brief Configura un movimiento suave hacia un ángulo objetivo
 * @param servo Puntero a la estructura Servo_Smooth_t
 * @param targetAngle Ángulo objetivo (0° a 180°)
 * @param durationMs Duración total del movimiento en milisegundos
 */
void PCA9685_SetSmoothAngle(Servo_Smooth_t* servo, float targetAngle, uint32_t durationMs)
{
    // Limitar el ángulo objetivo al rango permitido
    if (targetAngle < 0.0f) targetAngle = 0.0f;
    if (targetAngle > 180.0f) targetAngle = 180.0f;

    servo->targetAngle = targetAngle;

    // Calcular el número de pasos basado en la duración y el intervalo de actualización
    uint32_t numSteps = durationMs / servo->updateInterval;

    if (numSteps > 0) {
        // Calcular el tamaño del paso
        servo->stepSize = (targetAngle - servo->currentAngle) / (float)numSteps;
        servo->isMoving = true;
        servo->lastUpdateTime = HAL_GetTick();
    } else {
        // Si la duración es 0, mover inmediatamente
        servo->currentAngle = targetAngle;
        PCA9685_SetServoAngle(servo->channel, targetAngle);
        servo->isMoving = false;
    }
}

/**
 * @brief Actualiza el movimiento suave del servomotor (debe llamarse periódicamente)
 * @param servo Puntero a la estructura Servo_Smooth_t
 * @return true si el servomotor alcanzó el ángulo objetivo, false si aún está en movimiento
 */
bool PCA9685_UpdateSmoothServo(Servo_Smooth_t* servo)
{
    if (!servo->isMoving) {
        return true; // El movimiento ya ha terminado
    }

    uint32_t currentTime = HAL_GetTick();

    // Verificar si ha pasado el intervalo de actualización
    if ((currentTime - servo->lastUpdateTime) >= servo->updateInterval) {

        // Calcular el nuevo ángulo
        float newAngle = servo->currentAngle + servo->stepSize;

        // Verificar si hemos alcanzado o superado el objetivo
        if ((servo->stepSize > 0 && newAngle >= servo->targetAngle) ||
            (servo->stepSize < 0 && newAngle <= servo->targetAngle)) {
            newAngle = servo->targetAngle;
            servo->isMoving = false;
        }

        // Actualizar el ángulo actual
        servo->currentAngle = newAngle;

        // Enviar el nuevo ángulo al servomotor
        PCA9685_SetServoAngle(servo->channel, newAngle);

        // Actualizar el tiempo de la última actualización
        servo->lastUpdateTime = currentTime;

        return !servo->isMoving; // Devuelve true si el movimiento ha terminado
    }

    return false; // Aún no es tiempo de actualizar
}

/**
 * @brief Mueve un servomotor suavemente de un ángulo a otro
 * @param channel Canal del servomotor (0-15)
 * @param startAngle Ángulo inicial
 * @param endAngle Ángulo final
 * @param durationMs Duración del movimiento en milisegundos
 * @param updateIntervalMs Intervalo entre actualizaciones en milisegundos
 */
void PCA9685_SmoothMove(uint8_t channel, float startAngle, float endAngle, uint32_t durationMs, uint32_t updateIntervalMs)
{
    // Validar parámetros
    if (startAngle < 0.0f) startAngle = 0.0f;
    if (startAngle > 180.0f) startAngle = 180.0f;
    if (endAngle < 0.0f) endAngle = 0.0f;
    if (endAngle > 180.0f) endAngle = 180.0f;

    // Establecer el ángulo inicial
    PCA9685_SetServoAngle(channel, startAngle);

    // Calcular el número de pasos
    uint32_t numSteps = durationMs / updateIntervalMs;

    if (numSteps == 0) {
        // Movimiento inmediato
        PCA9685_SetServoAngle(channel, endAngle);
        return;
    }

    // Calcular el incremento por paso
    float stepIncrement = (endAngle - startAngle) / (float)numSteps;

    // Realizar el movimiento suave
    for (uint32_t i = 0; i < numSteps; i++) {
        float currentAngle = startAngle + (stepIncrement * (i + 1));
        PCA9685_SetServoAngle(channel, currentAngle);
        HAL_Delay(updateIntervalMs);
    }

    // Asegurar el ángulo final exacto
    PCA9685_SetServoAngle(channel, endAngle);
}
