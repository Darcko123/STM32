# PCA9685 STM32 PWM Control Library

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)

---

## 📦 Descripción
Librería desarrollada en C para el control del módulo **PCA9685** mediante la interfaz **I2C**, utilizando microcontroladores **STM32** con funciones HAL. Este controlador permite generar hasta **16 canales PWM independientes** con resolución de 12 bits, ideal para aplicaciones como servomotores, control de brillo LED, robótica, entre otros.

---

## 📦 Características
- Comunicación mediante I2C con STM32 HAL.
- Control de hasta **16 salidas PWM** de 0–100% duty cycle.
- Frecuencia configurable desde 24 Hz hasta 1526 Hz.
- Funciones dedicadas para control de **ángulo de servomotores**.
- Código limpio y documentado con estilo **Doxygen**.

---

## 🔧 Requisitos

- STM32CubeIDE o STM32CubeMX.
- Biblioteca HAL correspondiente a tu microcontrolador STM32.
- Módulo PCA9685 conectado mediante I2C.

## 🚀 Ejemplo de uso

``` C
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

## 🧪 Pruebas
Probado en las siguientes plataformas:
- STM32F103C8T6 (Blue Pill)

- STM32F429ZI (Discovery)