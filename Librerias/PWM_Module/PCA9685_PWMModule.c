/**
 * @file PCA9685_PWMModule.c
 * @brief Implementación de la librería para el control del módulo PCA9685 mediante I2C en STM32.
 * @author Daniel Ruiz
 * @date Jul 3, 2025
 * @version 1.2.0
 */

#include "PCA9685_PWMModule.h"
#include <math.h>

static I2C_HandleTypeDef* PCA9685_hi2c = NULL;  /**< Manejador de la interfaz I2C utilizado para comunicarse con el PCA9685 */

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Modifica un solo bit de un registro del PCA9685.
 * @param Register Dirección del registro a modificar.
 * @param Bit Posición del bit a modificar (0-7).
 * @param Value Valor a establecer en el bit (0 o 1).
 * @return PCA9685_Status_t Estado de la operación.
 */
static PCA9685_Status_t PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
    HAL_StatusTypeDef hal_status;
    uint8_t readValue;

    // Validar que el handler I2C esté inicializado
    if (PCA9685_hi2c == NULL)
    {
        return PCA9685_ERROR;
    }

    // Validar parámetros
    if (Bit > 7)
    {
        return PCA9685_ERROR;
    }

    // Leer el valor actual del registro
    hal_status = HAL_I2C_Mem_Read(PCA9685_hi2c, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
    if (hal_status == HAL_TIMEOUT)
    {
        return PCA9685_TIMEOUT;
    } else if (hal_status != HAL_OK)
    {
        return PCA9685_ERROR;
    }

    // Modificar el bit
    if (Value == 0)
        readValue &= ~(1 << Bit);
    else
        readValue |= (1 << Bit);

    // Escribir el nuevo valor
    hal_status = HAL_I2C_Mem_Write(PCA9685_hi2c, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
    if (hal_status == HAL_TIMEOUT)
    {
        return PCA9685_TIMEOUT;
    } else if (hal_status != HAL_OK)
    {
        return PCA9685_ERROR;
    }

    HAL_Delay(1);
    return PCA9685_OK;
}

/**
 * @brief Configura la frecuencia de salida PWM del módulo PCA9685.
 * @param frequency Frecuencia deseada en Hz (24 - 1526 Hz).
 * @return PCA9685_Status_t Estado de la operación.
 */
static PCA9685_Status_t PCA9685_SetPWMFrequency(uint16_t frequency)
{
    HAL_StatusTypeDef hal_status;
    PCA9685_Status_t status;
    uint8_t prescale;

    // Validar que el handler I2C esté inicializado
    if (PCA9685_hi2c == NULL)
    {
        return PCA9685_ERROR;
    }

    // Calcular el prescaler
    if (frequency >= 1526) {
        prescale = 0x03;
    } else if (frequency <= 24) {
        prescale = 0xFF;
    } else {
        prescale = (uint8_t)(25000000 / (4096 * frequency)); // Oscilador interno de 25 MHz
    }

    // Entrar a modo sleep
    status = PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
    if (status != PCA9685_OK)
    {
        return status;
    }

    // Escribir el prescaler
    hal_status = HAL_I2C_Mem_Write(PCA9685_hi2c, PCA9685_ADDRESS, PCA9685_PRE_SCALE, 1, &prescale, 1, 10);
    if (hal_status == HAL_TIMEOUT) {
        return PCA9685_TIMEOUT;
    } else if (hal_status != HAL_OK) {
        return PCA9685_ERROR;
    }

    // Salir de modo sleep
    status = PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
    if (status != PCA9685_OK)
    {
        return status;
    }

    // Reiniciar
    status = PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
    if (status != PCA9685_OK)
    {
        return status;
    }

    return PCA9685_OK;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el módulo PCA9685 con una frecuencia específica.
 * @param hi2c Puntero al manejador I2C.
 * @param frequency Frecuencia de PWM deseada (ej. 50 Hz para servos).
 * @return PCA9685_Status_t Estado de la operación.
 */
PCA9685_Status_t PCA9685_Init(I2C_HandleTypeDef* hi2c, uint16_t frequency)
{
    PCA9685_Status_t status;

    if(hi2c == NULL)
    {
        return PCA9685_ERROR;
    }

    // Validar rango de frecuencia
    if (frequency < 24 || frequency > 1526)
    {
        return PCA9685_ERROR;
    }

    // Asignar el handler I2C
    PCA9685_hi2c = hi2c;

    // Configurar la frecuencia PWM
    status = PCA9685_SetPWMFrequency(frequency);
    if (status != PCA9685_OK)
    {
        return status;
    }

    // Habilitar auto-incremento
    status = PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
    if (status != PCA9685_OK)
    {
        return status;
    }

    return PCA9685_OK;
}

/**
 * @brief Establece el tiempo de encendido y apagado para un canal específico.
 * @param Channel Canal del 0 al 15.
 * @param OnTime Tiempo en el cual la señal se activa (0-4095).
 * @param OffTime Tiempo en el cual la señal se desactiva (0-4095).
 * @return PCA9685_Status_t Estado de la operación.
 */
PCA9685_Status_t PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
    HAL_StatusTypeDef hal_status;
    uint8_t registerAddress;
    uint8_t pwm[4];

    // Validar que el handler I2C esté inicializado
    if (PCA9685_hi2c == NULL)
    {
        return PCA9685_ERROR;
    }

    // Validar parámetros
    if (Channel > 15)
    {
        return PCA9685_ERROR;
    }

    if (OnTime > 4095 || OffTime > 4095)
    {
        return PCA9685_ERROR;
    }

    // Calcular dirección del registro
    registerAddress = PCA9685_LED0_ON_L + (4 * Channel);

    // Preparar datos PWM
    pwm[0] = OnTime & 0xFF;
    pwm[1] = OnTime >> 8;
    pwm[2] = OffTime & 0xFF;
    pwm[3] = OffTime >> 8;

    // Escribir datos
    hal_status = HAL_I2C_Mem_Write(PCA9685_hi2c, PCA9685_ADDRESS, registerAddress, 1, pwm, 4, 10);
    if (hal_status == HAL_TIMEOUT)
    {
        return PCA9685_TIMEOUT;
    } else if (hal_status != HAL_OK)
    {
        return PCA9685_ERROR;
    }

    return PCA9685_OK;
}

/**
 * @brief Configura el ángulo de un servomotor conectado a un canal.
 * @param Channel Canal del 0 al 15.
 * @param Angle Ángulo deseado entre 0° y 180°.
 * @return PCA9685_Status_t Estado de la operación.
 */
PCA9685_Status_t PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
    float Value;

    // Validar que el handler I2C esté inicializado
    if (PCA9685_hi2c == NULL)
    {
        return PCA9685_ERROR;
    }

    // Validar parámetros
    if (Channel > 15)
    {
        return PCA9685_ERROR;
    }

    // Asegurar que el ángulo esté dentro del rango
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 180.0f) Angle = 180.0f;

    // Conversión del ángulo al valor PWM correspondiente entre 0.5ms (102.4) y 2.5ms (511.9)
    Value = (Angle * (511.9f - 102.4f) / 180.0f) + 102.4f;

    return PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}

/**
 * @brief Inicializa una estructura de movimiento suave para un servomotor
 * @param servo Puntero a la estructura Servo_Smooth_t
 * @param channel Canal del servomotor (0-15)
 * @param initialAngle Ángulo inicial del servomotor
 * @param updateInterval Intervalo entre actualizaciones en ms (ej. 20ms)
 * @return PCA9685_Status_t Estado de la operación.
 */
PCA9685_Status_t PCA9685_InitSmoothServo(Servo_Smooth_t* servo, uint8_t channel, float initialAngle, uint32_t updateInterval)
{
    PCA9685_Status_t status;

    // Validar parámetros
    if (servo == NULL)
    {
        return PCA9685_ERROR;
    }

    if (channel > 15)
    {
        return PCA9685_ERROR;
    }

    if (initialAngle < 0.0f || initialAngle > 180.0f)
    {
        return PCA9685_ERROR;
    }

    if (updateInterval == 0)
    {
        return PCA9685_ERROR;
    }

    // Inicializar estructura
    servo->channel = channel;
    servo->currentAngle = initialAngle;
    servo->targetAngle = initialAngle;
    servo->stepSize = 0.0f;
    servo->updateInterval = updateInterval;
    servo->lastUpdateTime = HAL_GetTick();
    servo->isMoving = false;

    // Establecer el ángulo inicial
    status = PCA9685_SetServoAngle(channel, initialAngle);
    if (status != PCA9685_OK)
    {
        return status;
    }

    return PCA9685_OK;
}

/**
 * @brief Configura un movimiento suave hacia un ángulo objetivo
 * @param servo Puntero a la estructura Servo_Smooth_t
 * @param targetAngle Ángulo objetivo (0° a 180°)
 * @param durationMs Duración total del movimiento en milisegundos
 * @return PCA9685_Status_t Estado de la operación.
 */
PCA9685_Status_t PCA9685_SetSmoothAngle(Servo_Smooth_t* servo, float targetAngle, uint32_t durationMs)
{

    PCA9685_Status_t status;
    uint32_t numSteps;

    // Validar parámetros
    if (servo == NULL)
    {
        return PCA9685_ERROR;
    }

    // Limitar el ángulo objetivo al rango permitido
    if (targetAngle < 0.0f) targetAngle = 0.0f;
    if (targetAngle > 180.0f) targetAngle = 180.0f;

    servo->targetAngle = targetAngle;

    // Calcular el número de pasos basado en la duración y el intervalo de actualización
    numSteps = durationMs / servo->updateInterval;

    if (numSteps > 0)
    {
        // Calcular el tamaño del paso
        servo->stepSize = (targetAngle - servo->currentAngle) / (float)numSteps;
        servo->isMoving = true;
        servo->lastUpdateTime = HAL_GetTick();
    } else
    {
        // Si la duración es 0, mover inmediatamente
        servo->currentAngle = targetAngle;
        status = PCA9685_SetServoAngle(servo->channel, targetAngle);
        if(status != PCA9685_OK)
        {
            return status;
        }
        servo->isMoving = false;
    }
    return PCA9685_OK;
}

/**
 * @brief Actualiza el movimiento suave del servomotor (debe llamarse periódicamente)
 * @param servo Puntero a la estructura Servo_Smooth_t
 * @return true si el servomotor alcanzó el ángulo objetivo, false si aún está en movimiento
 */
bool PCA9685_UpdateSmoothServo(Servo_Smooth_t* servo)
{
    uint32_t currentTime;
    float newAngle;
    PCA9685_Status_t status;

    // Validar parámetros
    if (servo == NULL)
    {
        return true; // Retornar true para evitar bucles infinitos
    }

    if (!servo->isMoving)
    {
        return true; // El movimiento ya ha terminado
    }

    currentTime = HAL_GetTick();

    // Verificar si ha pasado el intervalo de actualización
    if ((currentTime - servo->lastUpdateTime) >= servo->updateInterval)
    {
        // Calcular el nuevo ángulo
        newAngle = servo->currentAngle + servo->stepSize;

        // Verificar si hemos alcanzado o superado el objetivo
        if ((servo->stepSize > 0 && newAngle >= servo->targetAngle) ||
            (servo->stepSize < 0 && newAngle <= servo->targetAngle))
        {
            newAngle = servo->targetAngle;
            servo->isMoving = false;
        }

        // Actualizar el ángulo actual
        servo->currentAngle = newAngle;

        // Enviar el nuevo ángulo al servomotor
        status = PCA9685_SetServoAngle(servo->channel, newAngle);
        if(status != PCA9685_OK)
        {
            // En caso de error, detener el movimiento
            servo->isMoving = false;
            return true;
        }

        // Actualizar el tiempo de la última actualización
        servo->lastUpdateTime = currentTime;

        return !servo->isMoving; // Devuelve true si el movimiento ha terminado
    }

    return false; // Aún no es tiempo de actualizar
}

/**
 * @brief Mueve un servomotor suavemente de un ángulo a otro (función bloqueante)
 * @param channel Canal del servomotor (0-15)
 * @param startAngle Ángulo inicial
 * @param endAngle Ángulo final
 * @param durationMs Duración del movimiento en milisegundos
 * @param updateIntervalMs Intervalo entre actualizaciones en milisegundos
 * @return PCA9685_Status_t Estado de la operación.
 */
PCA9685_Status_t PCA9685_SmoothMove(uint8_t channel, float startAngle, float endAngle, uint32_t durationMs, uint32_t updateIntervalMs)
{
    PCA9685_Status_t status;
    uint32_t numSteps;
    float stepIncrement;
    uint32_t i;
    float currentAngle;

    // Validar parámetros
    if (channel > 15)
    {
        return PCA9685_ERROR;
    }

    if (updateIntervalMs == 0)
    {
        return PCA9685_ERROR;
    }

    // Validar y limitar ángulos
    if (startAngle < 0.0f) startAngle = 0.0f;
    if (startAngle > 180.0f) startAngle = 180.0f;
    if (endAngle < 0.0f) endAngle = 0.0f;
    if (endAngle > 180.0f) endAngle = 180.0f;

    // Establecer el ángulo inicial
    status = PCA9685_SetServoAngle(channel, startAngle);
    if (status != PCA9685_OK)
    {
        return status;
    }

    // Calcular el número de pasos
    numSteps = durationMs / updateIntervalMs;

    if (numSteps == 0) {
        // Movimiento inmediato
        status = PCA9685_SetServoAngle(channel, endAngle);
        return status;
    }

    // Calcular el incremento por paso
    stepIncrement = (endAngle - startAngle) / (float)numSteps;

    // Realizar el movimiento suave
    for (i = 0; i < numSteps; i++)
    {
        currentAngle = startAngle + (stepIncrement * (i + 1));
        status = PCA9685_SetServoAngle(channel, currentAngle);
        if(status != PCA9685_OK)
        {
            return status;
        }
        HAL_Delay(updateIntervalMs);
    }

    // Asegurar el ángulo final exacto
    status = PCA9685_SetServoAngle(channel, endAngle);
    return status;
}
