# PCA9685 STM32 PWM Control Library

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)
[![Version](https://img.shields.io/badge/Version-1.1.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/PWM_Module)

---

## 📦 Descripción
Librería desarrollada en C para el control del módulo **PCA9685** mediante la interfaz **I2C**, utilizando microcontroladores **STM32** con funciones HAL. Este controlador permite generar hasta **16 canales PWM independientes** con resolución de 12 bits, ideal para aplicaciones como servomotores, control de brillo LED, robótica, entre otros.

---

## 📦 Características
- Comunicación mediante I2C con STM32 HAL.
- Control de hasta **16 salidas PWM** de 0–100% duty cycle.
- Frecuencia configurable desde 24 Hz hasta 1526 Hz.
- Funciones dedicadas para control de **ángulo de servomotores** (0° - 180°).
- **Movimiento suave con interpolación** para transiciones naturales.
- Control no bloqueante para múltiples servomotores simultáneos.
- Función bloqueante para movimientos simples.
- Código limpio y documentado con estilo **Doxygen**.

---

## 🔧 Requisitos

- STM32CubeIDE o STM32CubeMX.
- Biblioteca HAL correspondiente a tu microcontrolador STM32.
- Módulo PCA9685 conectado mediante I2C.

## 🚀 Ejemplo de uso

### Ejemplo básico - Control directo de ángulo

```c
#include "PCA9685_PWMModule.h"

int main()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    // Inicializar PCA9685 a 50 Hz (frecuencia estándar para servos)
    PCA9685_Init(&hi2c1, 50);

    while(1)
    {
        // Ángulo 0° en el canal 0
        PCA9685_SetServoAngle(0, 0);
        HAL_Delay(1000);

        // Ángulo 90° en el canal 0
        PCA9685_SetServoAngle(0, 90);
        HAL_Delay(1000);

        // Ángulo 180° en el canal 0
        PCA9685_SetServoAngle(0, 180);
        HAL_Delay(1000);
    }
}
```

### Ejemplo avanzado - Movimiento suave bloqueante

```c
#include "PCA9685_PWMModule.h"

int main()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    PCA9685_Init(&hi2c1, 50);

    while(1)
    {
        // Mover suavemente de 0° a 180° en 2 segundos
        // Con actualizaciones cada 20ms
        PCA9685_SmoothMove(0, 0, 180, 2000, 20);
        HAL_Delay(500);

        // Regresar suavemente a 0°
        PCA9685_SmoothMove(0, 180, 0, 2000, 20);
        HAL_Delay(500);
    }
}
```

### Ejemplo avanzado - Movimiento suave no bloqueante

```c
#include "PCA9685_PWMModule.h"

Servo_Smooth_t servo1;
Servo_Smooth_t servo2;

int main()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    PCA9685_Init(&hi2c1, 50);

    // Inicializar servos con movimiento suave
    // Canal 0: ángulo inicial 90°, actualización cada 20ms
    PCA9685_InitSmoothServo(&servo1, 0, 90, 20);
    
    // Canal 1: ángulo inicial 45°, actualización cada 20ms
    PCA9685_InitSmoothServo(&servo2, 1, 45, 20);

    // Configurar movimientos objetivo
    PCA9685_SetSmoothAngle(&servo1, 180, 2000); // 2 segundos hasta 180°
    PCA9685_SetSmoothAngle(&servo2, 135, 1500); // 1.5 segundos hasta 135°

    while(1)
    {
        // Actualizar ambos servos (no bloqueante)
        bool servo1_finished = PCA9685_UpdateSmoothServo(&servo1);
        bool servo2_finished = PCA9685_UpdateSmoothServo(&servo2);

        // Verificar si ambos servos terminaron
        if (servo1_finished && servo2_finished)
        {
            HAL_Delay(1000);
            
            // Configurar nuevos movimientos
            PCA9685_SetSmoothAngle(&servo1, 0, 2000);
            PCA9685_SetSmoothAngle(&servo2, 45, 1500);
        }

        // El loop continúa ejecutándose, permitiendo otras tareas
        HAL_Delay(1);
    }
}
```

## API Reference

### Funciones básicas

#### `PCA9685_Init()`
Inicializa el módulo PCA9685 con la frecuencia PWM especificada.
```c
void PCA9685_Init(I2C_HandleTypeDef* hi2c, uint16_t frequency);
```

#### `PCA9685_SetPWM()`
Establece los valores de encendido/apagado para un canal PWM específico.
```c
void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
```

#### `PCA9685_SetServoAngle()`
Controla directamente el ángulo de un servomotor (0° - 180°).
```c
void PCA9685_SetServoAngle(uint8_t Channel, float Angle);
```

### Funciones de movimiento suave

#### `PCA9685_SmoothMove()` - Bloqueante

Mueve un servomotor suavemente de un ángulo a otro. Esta función es bloqueante.
```c
void PCA9685_SmoothMove(uint8_t channel, float startAngle, float endAngle, 
                        uint32_t durationMs, uint32_t updateIntervalMs);
```
**Parámetros:**
- `channel`: Canal del servomotor (0-15)
- `startAngle`: Ángulo inicial (0° - 180°)
- `endAngle`: Ángulo final (0° - 180°)
- `durationMs`: Duración total del movimiento en milisegundos
- `updateIntervalMs`: Intervalo entre actualizaciones (recomendado: 20ms)

#### `PCA9685_InitSmoothServo()` - No bloqueante
Inicializa una estructura para control no bloqueante de servomotor.
```c
void PCA9685_InitSmoothServo(Servo_Smooth_t* servo, uint8_t channel, 
                             float initialAngle, uint32_t updateInterval);
```

#### `PCA9685_SetSmoothAngle()` - No bloqueante
Configura un movimiento suave hacia un ángulo objetivo.
```c
void PCA9685_SetSmoothAngle(Servo_Smooth_t* servo, float targetAngle, 
                            uint32_t durationMs);
```

#### `PCA9685_UpdateSmoothServo()` - No bloqueante
Actualiza el movimiento suave. Debe llamarse periódicamente en el loop principal.
```c
bool PCA9685_UpdateSmoothServo(Servo_Smooth_t* servo);
```
**Retorna:** `true` si el movimiento terminó, `false` si aún está en progreso.

## Conexión de hardware

| PCA9685 | STM32   |
|---------|---------|
| VCC     | 5V/3.3V |
| GND     | GND     |
| SDA     | I2C_SDA |
| SCL     | I2C_SCL |

>[!NOTE]
>Asegúrate de que la dirección I2C del PCA9685 coincida con la definida en el código (`0x80` por defecto). Esto se puede modificar mediante los pines A0-A5 del módulo.

---

---

## Pruebas
Probado en las siguientes plataformas:
- ✅ STM32F103C8T6 (Blue Pill)
- ✅ STM32F429ZI (Discovery)

---

## 📄 Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog

### Version 1.1.0
- Añadidas funciones de movimiento suave con interpolación
- Implementación de control no bloqueante para múltiples servos
- Función bloqueante `PCA9685_SmoothMove()` para movimientos simples
- Documentación mejorada con ejemplos avanzados

### Version 1.0.0
- Versión inicial con control básico de PWM y servomotores