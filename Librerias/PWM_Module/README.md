# Librería para el módulo PWM PCA9685 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-2.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/PWM_Module)
[![Protocol](https://img.shields.io/badge/Protocol-I2C-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/PWM_Module)

---

## Tabla de Contenidos
- [Librería para el módulo PWM PCA9685 en STM32](#librería-para-el-módulo-pwm-pca9685-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
  - [Configuración I2C](#configuración-i2c)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Control directo de ángulo](#2-control-directo-de-ángulo)
    - [3. Movimiento suave bloqueante](#3-movimiento-suave-bloqueante)
    - [4. Movimiento suave no bloqueante](#4-movimiento-suave-no-bloqueante)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`PCA9685_Status_t` - Estados de Retorno](#pca9685_status_t---estados-de-retorno)
      - [`Servo_Smooth_t` - Estructura de Movimiento Suave](#servo_smooth_t---estructura-de-movimiento-suave)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`PCA9685_Init()` - Inicialización del Driver](#pca9685_init---inicialización-del-driver)
      - [`PCA9685_SetPWM()` - Control Directo de PWM](#pca9685_setpwm---control-directo-de-pwm)
      - [`PCA9685_SetServoAngle()` - Control de Ángulo](#pca9685_setservoangle---control-de-ángulo)
      - [`PCA9685_SmoothMove()` - Movimiento Suave Bloqueante](#pca9685_smoothmove---movimiento-suave-bloqueante)
      - [`PCA9685_InitSmoothServo()` - Inicialización No Bloqueante](#pca9685_initsmoothservo---inicialización-no-bloqueante)
      - [`PCA9685_SetSmoothAngle()` - Configurar Movimiento No Bloqueante](#pca9685_setsmoothangle---configurar-movimiento-no-bloqueante)
      - [`PCA9685_UpdateSmoothServo()` - Actualizar Movimiento No Bloqueante](#pca9685_updatesmoothservo---actualizar-movimiento-no-bloqueante)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[2.0.0\] - 16-04-2026](#200---16-04-2026)
      - [Added](#added)
      - [Changed](#changed)
    - [\[1.2.0\] - 13-01-2026](#120---13-01-2026)
      - [Added](#added-1)
      - [Changed](#changed-1)
      - [Fixed](#fixed)
    - [\[1.1.0\] - 21-12-2025](#110---21-12-2025)
      - [Added](#added-2)
    - [\[1.0.0\] - 07-01-2025](#100---07-01-2025)
      - [Added](#added-3)

---

## Descripción
Librería desarrollada en C para el control del módulo **NXP PCA9685** mediante la interfaz **I2C**, utilizando microcontroladores **STM32** con funciones HAL. Este controlador permite generar hasta **16 canales PWM independientes** con resolución de 12 bits, ideal para aplicaciones como servomotores, control de brillo LED y robótica. La librería incluye soporte para movimiento suave con interpolación lineal, tanto en modalidad bloqueante como no bloqueante.

---

## Características
- **Comunicación I2C**: Abstracción completa de las comunicaciones con el PCA9685.
- **Control de hasta 16 salidas PWM** de 0–100% duty cycle con resolución de 12 bits.
- **Frecuencia configurable** desde 24 Hz hasta 1526 Hz.
- **Control de servomotores**: Funciones dedicadas para control de ángulo (0° - 180°).
- **Movimiento suave con interpolación lineal** para transiciones naturales.
- **Control no bloqueante** para múltiples servomotores simultáneos mediante `Servo_Smooth_t`.
- **Función bloqueante** `PCA9685_SmoothMove()` para movimientos simples.
- **Manejo robusto de errores**: Códigos de retorno específicos para error de I2C, timeout o módulo no inicializado (`PCA9685_Status_t`).
- **Validación exhaustiva de parámetros** en todas las funciones públicas.
- **Portabilidad**: Compatible con múltiples familias STM32 mediante la capa HAL. Solo requiere cambiar el `#include` del encabezado HAL en `PCA9685.h`.

---

## Pinout y Conexiones
### Pines requeridos

| Pin PCA9685 | Dirección | Descripción | Tipo GPIO | Observaciones |
|-------------|-----------|-------------|-----------|---------------|
| **VCC**     | Alimentación | 2.3V a 5.5V | N/A | Soporta 3.3V o 5V |
| **GND**     | Tierra    | — | N/A | — |
| **SCL**     | Input (open-drain) | Línea de reloj I2C | `I2C_SCL` | Requiere resistencia pull-up (típicamente 4.7kΩ) |
| **SDA**     | Input/Output (open-drain) | Línea de datos I2C | `I2C_SDA` | Requiere resistencia pull-up (típicamente 4.7kΩ) |
| **A0–A5**   | Input (opcional) | Configuración de dirección I2C | `GPIO_INPUT` | Por defecto todos a GND → dirección 0x40 (7 bits) |
| **OE**      | Input (opcional) | Habilitación de salidas (activo en bajo) | `GPIO_OUTPUT` | Si no se usa, conectar a GND |

> [!NOTE]
> Las resistencias pull-up en las líneas SDA y SCL son **obligatorias** para el correcto funcionamiento de I2C. Muchos módulos comerciales ya las incluyen.

---

## Configuración I2C
Configura tu periférico I2C en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | I2C | Selecciona el modo I2C |
| **Speed** | Fast Mode (400 kHz) | El PCA9685 soporta hasta 400 kHz |
| **Clock Speed** | 100 kHz o 400 kHz | Recomendado 100 kHz para debug, luego 400 kHz |
| **Primary Address Length** | 7-bit | Dirección estándar |
| **Own Address 1** | 0x00 | No relevante para modo maestro |

> [!NOTE]
> La dirección I2C del PCA9685 es configurable mediante los pines A0–A5: **0x40** (7 bits) / **0x80** (8 bits con bit R/W). La librería ya la tiene definida como `PCA9685_ADDRESS 0x80`.

---

## Instalación
1. Copia `PCA9685.c` y `PCA9685.h` a tu proyecto (ej: `Librerias/PWM_Module/`).
2. Ajusta el `#include` del encabezado HAL en `PCA9685.h` según tu familia STM32:
  ```c
  // STM32F1xx:
  #include "stm32f1xx_hal.h"
  // STM32F4xx:
  #include "stm32f4xx_hal.h"
  ```
3. Incluye la librería en tu main.c o archivo principal:
  ```c
  #include "PCA9685.h"
  ```
4. Configura I2C y GPIOs en CubeMX (ver sección anterior).
5. Genera código y compila.

---

## Uso Básico

### 1. Inicialización

```c
// En main() después de HAL_Init() y MX_I2Cx_Init()
PCA9685_Status_t status = PCA9685_Init(&hi2c1, 50); // 50 Hz para servos

if (status != PCA9685_OK) {
    Error_Handler();  // Módulo no responde o handle NULL
}
```

### 2. Control directo de ángulo

```c
// Mover servo del canal 0 a 0°, 90° y 180°
PCA9685_SetServoAngle(0, 0.0f);
HAL_Delay(1000);

PCA9685_SetServoAngle(0, 90.0f);
HAL_Delay(1000);

PCA9685_SetServoAngle(0, 180.0f);
HAL_Delay(1000);
```

### 3. Movimiento suave bloqueante

```c
// Mover suavemente de 0° a 180° en 2 segundos con actualizaciones cada 20ms
PCA9685_Status_t status = PCA9685_SmoothMove(0, 0.0f, 180.0f, 2000, 20);
if (status != PCA9685_OK) {
    // Manejar error en movimiento
}
HAL_Delay(500);

// Regresar suavemente a 0°
PCA9685_SmoothMove(0, 180.0f, 0.0f, 2000, 20);
```

### 4. Movimiento suave no bloqueante

```c
Servo_Smooth_t servo1;
Servo_Smooth_t servo2;

// Inicializar servo 1 en canal 0 a 90°, actualización cada 20ms
PCA9685_InitSmoothServo(&servo1, 0, 90.0f, 20);

// Inicializar servo 2 en canal 1 a 45°, actualización cada 20ms
PCA9685_InitSmoothServo(&servo2, 1, 45.0f, 20);

// Configurar movimientos objetivo
PCA9685_SetSmoothAngle(&servo1, 180.0f, 2000); // 2 segundos hasta 180°
PCA9685_SetSmoothAngle(&servo2, 135.0f, 1500); // 1.5 segundos hasta 135°

while (1) {
    // Actualizar ambos servos (no bloqueante)
    bool servo1_done = PCA9685_UpdateSmoothServo(&servo1);
    bool servo2_done = PCA9685_UpdateSmoothServo(&servo2);

    if (servo1_done && servo2_done) {
        HAL_Delay(1000);
        PCA9685_SetSmoothAngle(&servo1, 0.0f, 2000);
        PCA9685_SetSmoothAngle(&servo2, 45.0f, 1500);
    }

    HAL_Delay(1);
}
```

---

## API Reference

### 1. Tipos de Datos

#### `PCA9685_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería, permitiendo una gestión robusta de errores y estados del chip.

```c
typedef enum {
    PCA9685_OK = 0,             /**< Operación exitosa */
    PCA9685_ERROR = 1,          /**< Error en la operación */
    PCA9685_TIMEOUT = 2,        /**< Timeout en la operación */
    PCA9685_NOT_INITIALIZED = 3 /**< Módulo no inicializado */
} PCA9685_Status_t;
```

| Valor | Código | Significado |
|-------|--------|-------------|
| `PCA9685_OK` | 0 | Operación completada sin errores |
| `PCA9685_ERROR` | 1 | Error de I2C, parámetro inválido o condición inesperada |
| `PCA9685_TIMEOUT` | 2 | Timeout en la comunicación I2C |
| `PCA9685_NOT_INITIALIZED` | 3 | `PCA9685_Init()` no se llamó o falló |

---

#### `Servo_Smooth_t` - Estructura de Movimiento Suave

Estructura que encapsula el estado de un servomotor para control de movimiento suave no bloqueante. Debe inicializarse con `PCA9685_InitSmoothServo()` antes de usarse.

```c
typedef struct {
    uint8_t  channel;          /**< Canal del servomotor (0-15) */
    float    currentAngle;     /**< Ángulo actual del servomotor */
    float    targetAngle;      /**< Ángulo objetivo */
    float    stepSize;         /**< Tamaño del paso por intervalo */
    uint32_t updateInterval;   /**< Intervalo entre actualizaciones en ms */
    uint32_t lastUpdateTime;   /**< Último tiempo de actualización (HAL_GetTick) */
    bool     isMoving;         /**< true si el servomotor está en movimiento */
} Servo_Smooth_t;
```

| Campo | Tipo | Rango | Descripción |
|-------|------|-------|-------------|
| `channel` | `uint8_t` | 0–15 | Canal del PCA9685 al que está conectado el servo |
| `currentAngle` | `float` | 0.0–180.0 | Ángulo actual en grados |
| `targetAngle` | `float` | 0.0–180.0 | Ángulo objetivo en grados |
| `stepSize` | `float` | — | Incremento de ángulo por cada intervalo de actualización |
| `updateInterval` | `uint32_t` | >0 | Tiempo en ms entre actualizaciones (recomendado: 20ms) |
| `lastUpdateTime` | `uint32_t` | — | Marca de tiempo de la última actualización (`HAL_GetTick()`) |
| `isMoving` | `bool` | — | Indica si el servo está en movimiento activo |

---

### 2. Funciones Públicas

#### `PCA9685_Init()` - Inicialización del Driver

Inicializa el driver, almacenando el handle I2C y configurando la frecuencia PWM del módulo. Todas las demás funciones retornan `PCA9685_NOT_INITIALIZED` si esta función no se ha llamado exitosamente.

```c
PCA9685_Status_t PCA9685_Init(I2C_HandleTypeDef *hi2c, uint16_t frequency);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `hi2c` | `I2C_HandleTypeDef*` | Handle del periférico I2C configurado | — |
| `frequency` | `uint16_t` | Frecuencia PWM en Hz (50 Hz para servos estándar) | 24–1526 |

**Retorna**: `PCA9685_OK` si la inicialización fue exitosa, `PCA9685_ERROR` si el handle es NULL o la frecuencia está fuera de rango, `PCA9685_TIMEOUT` si falla la comunicación I2C.

---

#### `PCA9685_SetPWM()` - Control Directo de PWM

Establece los valores de encendido y apagado para un canal PWM con resolución de 12 bits (0–4095).

```c
PCA9685_Status_t PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `Channel` | `uint8_t` | Canal PWM a configurar | 0–15 |
| `OnTime` | `uint16_t` | Tick en el que la salida se activa | 0–4095 |
| `OffTime` | `uint16_t` | Tick en el que la salida se desactiva | 0–4095 |

**Retorna**: `PCA9685_OK` si la operación fue exitosa, `PCA9685_ERROR` si el canal o los valores están fuera de rango, `PCA9685_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `PCA9685_SetServoAngle()` - Control de Ángulo

Controla el ángulo de un servomotor estándar (pulso entre 0.5 ms y 2.5 ms). El ángulo se limita automáticamente al rango 0°–180°.

```c
PCA9685_Status_t PCA9685_SetServoAngle(uint8_t Channel, float Angle);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `Channel` | `uint8_t` | Canal del servomotor | 0–15 |
| `Angle` | `float` | Ángulo deseado en grados | 0.0–180.0 |

**Retorna**: `PCA9685_OK` si la operación fue exitosa, `PCA9685_ERROR` si el canal es inválido o el handle es NULL, `PCA9685_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `PCA9685_SmoothMove()` - Movimiento Suave Bloqueante

Mueve un servomotor suavemente de un ángulo a otro mediante interpolación lineal. Esta función es **bloqueante**: no retorna hasta que el movimiento se completa.

```c
PCA9685_Status_t PCA9685_SmoothMove(uint8_t channel, float startAngle, float endAngle,
                                    uint32_t durationMs, uint32_t updateIntervalMs);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `channel` | `uint8_t` | Canal del servomotor | 0–15 |
| `startAngle` | `float` | Ángulo inicial en grados | 0.0–180.0 |
| `endAngle` | `float` | Ángulo final en grados | 0.0–180.0 |
| `durationMs` | `uint32_t` | Duración total del movimiento en ms | >0 |
| `updateIntervalMs` | `uint32_t` | Intervalo entre pasos en ms (recomendado: 20) | >0 |

**Retorna**: `PCA9685_OK` si el movimiento se completó, `PCA9685_ERROR` si algún parámetro es inválido, `PCA9685_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `PCA9685_InitSmoothServo()` - Inicialización No Bloqueante

Inicializa una estructura `Servo_Smooth_t` para control no bloqueante y mueve el servo al ángulo inicial.

```c
PCA9685_Status_t PCA9685_InitSmoothServo(Servo_Smooth_t *servo, uint8_t channel,
                                         float initialAngle, uint32_t updateInterval);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `servo` | `Servo_Smooth_t*` | Puntero a la estructura a inicializar | — |
| `channel` | `uint8_t` | Canal del servomotor | 0–15 |
| `initialAngle` | `float` | Ángulo inicial en grados | 0.0–180.0 |
| `updateInterval` | `uint32_t` | Intervalo de actualización en ms (recomendado: 20) | >0 |

**Retorna**: `PCA9685_OK` si la inicialización fue exitosa, `PCA9685_ERROR` si algún parámetro es inválido o `servo` es NULL.

---

#### `PCA9685_SetSmoothAngle()` - Configurar Movimiento No Bloqueante

Configura un movimiento suave hacia un ángulo objetivo. El movimiento se ejecuta llamando periódicamente a `PCA9685_UpdateSmoothServo()`.

```c
PCA9685_Status_t PCA9685_SetSmoothAngle(Servo_Smooth_t *servo, float targetAngle,
                                        uint32_t durationMs);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `servo` | `Servo_Smooth_t*` | Puntero a la estructura del servo | — |
| `targetAngle` | `float` | Ángulo objetivo en grados | 0.0–180.0 |
| `durationMs` | `uint32_t` | Duración total del movimiento en ms | ≥0 |

**Retorna**: `PCA9685_OK` si la configuración fue exitosa, `PCA9685_ERROR` si `servo` es NULL.

> [!NOTE]
> Si `durationMs` es 0, el servo se mueve inmediatamente al ángulo objetivo.

---

#### `PCA9685_UpdateSmoothServo()` - Actualizar Movimiento No Bloqueante

Actualiza el movimiento suave de un servo. Debe llamarse periódicamente en el loop principal. Internamente verifica si ha transcurrido el intervalo configurado antes de actualizar el ángulo.

```c
bool PCA9685_UpdateSmoothServo(Servo_Smooth_t *servo);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `servo` | `Servo_Smooth_t*` | Puntero a la estructura del servo |

**Retorna**: `true` si el servo alcanzó el ángulo objetivo o si `servo` es NULL, `false` si el movimiento sigue en progreso.

---

## Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [2.0.0] - 16-04-2026

#### Added
- Sección de Pinout y Conexiones en README.
- Sección de Configuración I2C en README.
- Sección de Instalación en README.
- Documentación completa de tipos de datos en API Reference.

#### Changed
- Restructuración completa del README para seguir el estándar de la librería.
- Referencias al encabezado corregidas de `PCA9685_PWMModule.h` a `PCA9685.h`.

---

### [1.2.0] - 13-01-2026

#### Added
- Sistema completo de gestión de errores con tipo `PCA9685_Status_t`.
- Validación exhaustiva de parámetros en todas las funciones públicas.
- Control de punteros NULL en todas las funciones públicas.
- Verificación de inicialización previa del módulo.
- Diferenciación entre errores generales y timeouts I2C.

#### Changed
- Funciones `PCA9685_SetBit` y `PCA9685_SetPWMFrequency` convertidas a privadas.

#### Fixed
- Versión en README ahora coincide con el archivo VERSION (2.0.0).
- Referencias al encabezado corregidas de `PCA9685_PWMModule.h` a `PCA9685.h`.

---

### [1.1.0] - 21-12-2025

#### Added
- Funciones de movimiento suave con interpolación lineal.
- Control no bloqueante para múltiples servos simultáneos mediante `Servo_Smooth_t`.
- Función bloqueante `PCA9685_SmoothMove()` para movimientos simples.
- Documentación mejorada con ejemplos avanzados de uso.

---

### [1.0.0] - 07-01-2025

#### Added
- Versión inicial con control básico de PWM y servomotores.
- Comunicación I2C con STM32 HAL.
- Control de hasta 16 canales PWM independientes con resolución de 12 bits.
- Frecuencia PWM configurable de 24 Hz a 1526 Hz.
- Función `PCA9685_SetServoAngle()` para control de ángulo (0° - 180°).
