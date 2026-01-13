# PCA9685 STM32 PWM Control Library

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)
[![Version](https://img.shields.io/badge/Version-1.2.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/PWM_Module)

---

## Tabla de Contenidos
- [PCA9685 STM32 PWM Control Library](#pca9685-stm32-pwm-control-library)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Requisitos](#requisitos)
  - [Ejemplo de uso](#ejemplo-de-uso)
    - [Ejemplo básico - Control directo de ángulo con manejo de errores](#ejemplo-básico---control-directo-de-ángulo-con-manejo-de-errores)
    - [Ejemplo avanzado - Movimiento suave bloqueante con verificación de errores](#ejemplo-avanzado---movimiento-suave-bloqueante-con-verificación-de-errores)
    - [Ejemplo avanzado - Movimiento suave no bloqueante con gestión de errores](#ejemplo-avanzado---movimiento-suave-no-bloqueante-con-gestión-de-errores)
  - [API Reference](#api-reference)
    - [Tipo de dato para gestión de errores](#tipo-de-dato-para-gestión-de-errores)
    - [Funciones básicas](#funciones-básicas)
      - [`PCA9685_Init()`](#pca9685_init)
      - [`PCA9685_SetPWM()`](#pca9685_setpwm)
      - [`PCA9685_SetServoAngle()`](#pca9685_setservoangle)
    - [Funciones de movimiento suave](#funciones-de-movimiento-suave)
      - [`PCA9685_SmoothMove()` - Bloqueante](#pca9685_smoothmove---bloqueante)
      - [`PCA9685_InitSmoothServo()` - No bloqueante](#pca9685_initsmoothservo---no-bloqueante)
      - [`PCA9685_SetSmoothAngle()` - No bloqueante](#pca9685_setsmoothangle---no-bloqueante)
      - [`PCA9685_UpdateSmoothServo()` - No bloqueante](#pca9685_updatesmoothservo---no-bloqueante)
  - [Conexión de hardware](#conexión-de-hardware)
  - [Pruebas](#pruebas)
  - [📄 Licencia](#-licencia)
  - [Changelog](#changelog)
    - [Version 1.2.0](#version-120)
    - [Version 1.1.0](#version-110)
    - [Version 1.0.0](#version-100)

## Descripción
Librería desarrollada en C para el control del módulo **PCA9685** mediante la interfaz **I2C**, utilizando microcontroladores **STM32** con funciones HAL. Este controlador permite generar hasta **16 canales PWM independientes** con resolución de 12 bits, ideal para aplicaciones como servomotores, control de brillo LED, robótica, entre otros.

---

## Características
- Comunicación mediante I2C con STM32 HAL.
- Control de hasta **16 salidas PWM** de 0–100% duty cycle.
- Frecuencia configurable desde 24 Hz hasta 1526 Hz.
- Funciones dedicadas para control de **ángulo de servomotores** (0° - 180°).
- **Movimiento suave con interpolación** para transiciones naturales.
- Control no bloqueante para múltiples servomotores simultáneos.
- Función bloqueante para movimientos simples.
- **Sistema completo de gestión de errores** con códigos de retorno específicos.
- **Validación exhaustiva de parámetros** para mayor robustez.
- Código limpio y documentado con estilo **Doxygen**.

---

## Requisitos

- STM32CubeIDE o STM32CubeMX.
- Biblioteca HAL correspondiente a tu microcontrolador STM32.
- Módulo PCA9685 conectado mediante I2C.

## Ejemplo de uso

### Ejemplo básico - Control directo de ángulo con manejo de errores

```c
#include "PCA9685_PWMModule.h"

int main()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    // Inicializar PCA9685 a 50 Hz (frecuencia estándar para servos)
    PCA9685_Status_t status = PCA9685_Init(&hi2c1, 50);
    
    if (status != PCA9685_OK)
    {
        // Manejar error de inicialización
        Error_Handler();
    }

    while(1)
    {
        // Ángulo 0° en el canal 0
        status = PCA9685_SetServoAngle(0, 0);
        if (status != PCA9685_OK)
        {
            // Manejar error
        }
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

### Ejemplo avanzado - Movimiento suave bloqueante con verificación de errores

```c
#include "PCA9685_PWMModule.h"

int main()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    PCA9685_Status_t status = PCA9685_Init(&hi2c1, 50);
    if(status != PCA9685_OK)
    {
        // Manejar error de inicialización
        Error_Handler();
    }

    while(1)
    {
        // Mover suavemente de 0° a 180° en 2 segundos
        // Con actualizaciones cada 20ms
        status = PCA9685_SmoothMove(0, 0, 180, 2000, 20);
        if (status != PCA9685_OK)
        {
            // Manejar error en movimiento
        }
        HAL_Delay(500);

        // Regresar suavemente a 0°
        status = PCA9685_SmoothMove(0, 180, 0, 2000, 20);
        if (status != PCA9685_OK)
        {
            // Manejar error en movimiento
        }
        HAL_Delay(500);
    }
}
```

### Ejemplo avanzado - Movimiento suave no bloqueante con gestión de errores

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

    PCA9685_Status_t status = PCA9685_Init(&hi2c1, 50);
    if (status != PCA9685_OK) {
        Error_Handler();
    }

    // Inicializar servos con movimiento suave
    // Canal 0: ángulo inicial 90°, actualización cada 20ms
    status = PCA9685_InitSmoothServo(&servo1, 0, 90, 20);
    if (status != PCA9685_OK) {
        // Manejar error de inicialización
    }
    
    // Canal 1: ángulo inicial 45°, actualización cada 20ms
    status = PCA9685_InitSmoothServo(&servo2, 1, 45, 20);
    if (status != PCA9685_OK)
    {
        // Manejar error de inicialización
    }

    // Configurar movimientos objetivo
    status = PCA9685_SetSmoothAngle(&servo1, 180, 2000); // 2 segundos hasta 180°
    if (status != PCA9685_OK)
    {
        // Manejar error
    }

    status = PCA9685_SetSmoothAngle(&servo2, 135, 1500); // 1.5 segundos hasta 135°
    if (status != PCA9685_OK) {
        // Manejar error
    }

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

### Tipo de dato para gestión de errores
```c
typedef enum {
    PCA9685_OK = 0,         /**< Operación exitosa */
    PCA9685_ERROR = 1,      /**< Error en la operación */
    PCA9685_TIMEOUT = 2     /**< Timeout en la operación */
} PCA9685_Status_t;
```

### Funciones básicas

---

#### `PCA9685_Init()`
Inicializa el módulo PCA9685 con la frecuencia PWM especificada.
```c
PCA9685_Status_t PCA9685_Init(I2C_HandleTypeDef* hi2c, uint16_t frequency);
```
**Parámetros:**
- `hi2c`: Puntero a la estructura del manejador I2C.
- `frequency`: Frecuencia PWM deseada en Hz.

**Retorna:** 
- `PCA9685_Status_t` indicando el estado de la operación.

---

#### `PCA9685_SetPWM()`
Establece los valores de encendido/apagado para un canal PWM específico.
```c
PCA9685_Status_t PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
```
**Validaciones:**
- `Channel`: Debe estar entre 0 y 15.
- `OnTime` y `OffTime`: Deben estar entre 0 y 409
- Módulo inicializado previamente

---

#### `PCA9685_SetServoAngle()`
Controla directamente el ángulo de un servomotor (0° - 180°).
```c
PCA9685_Status_t PCA9685_SetServoAngle(uint8_t Channel, float Angle);
```
**Validaciones:**
- `Channel`: Debe estar entre 0 y 15.
- Ángulo automáticamente limitado a 0°-180°

---

### Funciones de movimiento suave

---

#### `PCA9685_SmoothMove()` - Bloqueante

Mueve un servomotor suavemente de un ángulo a otro. Esta función es bloqueante.
```c
PCA9685_Status_t PCA9685_SmoothMove(uint8_t channel, float startAngle, float endAngle, 
                        uint32_t durationMs, uint32_t updateIntervalMs);
```
**Parámetros:**
- `channel`: Canal del servomotor (0-15)
- `startAngle`: Ángulo inicial (0° - 180°)
- `endAngle`: Ángulo final (0° - 180°)
- `durationMs`: Duración total del movimiento en milisegundos
- `updateIntervalMs`: Intervalo entre actualizaciones (recomendado: 20ms)

**Validaciones:**
- `channel`: Debe estar entre 0 y 15.
- `startAngle` y `endAngle`: Deben estar entre 0° y 180°.
- `durationMs` y `updateIntervalMs`: Deben ser mayores a 0.
- Módulo inicializado previamente

---

#### `PCA9685_InitSmoothServo()` - No bloqueante
Inicializa una estructura para control no bloqueante de servomotor.
```c
PCA9685_Status_t PCA9685_InitSmoothServo(Servo_Smooth_t* servo, uint8_t channel, 
                             float initialAngle, uint32_t updateInterval);
```
**Validaciones:**
- Puntero `servo` no nulo.
- `channel`: Debe estar entre 0 y 15.
- `initialAngle`: Debe estar entre 0° y 180°.
- `updateInterval`: Debe ser mayor a 0.

---

#### `PCA9685_SetSmoothAngle()` - No bloqueante
Configura un movimiento suave hacia un ángulo objetivo.
```c
PCA9685_Status_t PCA9685_SetSmoothAngle(Servo_Smooth_t* servo, float targetAngle, 
                            uint32_t durationMs);
```
**Validaciones:**
- Puntero `servo` no nulo.
- `targetAngle`: Debe estar entre 0° y 180°.
- `durationMs`: Debe ser mayor a 0.

---

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
- ✅ Validación exhaustiva de casos de error
- ✅ Pruebas de comunicación I2C con fallos simulados

---

## 📄 Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog

### Version 1.2.0
- Sistema completo de gestión de errores con tipo PCA9685_Status_t
- Validación exhaustiva de parámetros en todas las funciones
- Funciones `PCA9685_SetBit` y `PCA9685_SetPWMFrequency` convertidas a privadas.
- Diferenciación entre errores generales y timeouts I2C
- Verificación de inicialización previa del módulo
- Control de punteros NULL en todas las funciones públicas
- Corrección de bugs menores en funciones de movimiento suave

### Version 1.1.0
- Añadidas funciones de movimiento suave con interpolación
- Implementación de control no bloqueante para múltiples servos
- Función bloqueante `PCA9685_SmoothMove()` para movimientos simples
- Documentación mejorada con ejemplos avanzados

### Version 1.0.0
- Versión inicial con control básico de PWM y servomotores