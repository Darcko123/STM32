# Librería para el módulo MPU6050 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)
[![Version](https://img.shields.io/badge/Version-2.1.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/MPU6050)
[![Protocol](https://img.shields.io/badge/Protocol-I2C-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/MPU6050)

---

## Tabla de Contenidos
- [Librería para el módulo MPU6050 en STM32](#librería-para-el-módulo-mpu6050-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
  - [Configuración I2C](#configuración-i2c)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Lectura de aceleración y giroscopio](#2-lectura-de-aceleración-y-giroscopio)
    - [3. Lectura de temperatura y modo sleep](#3-lectura-de-temperatura-y-modo-sleep)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`MPU6050_Status_t` - Estados de Retorno](#mpu6050_status_t---estados-de-retorno)
      - [`MPU6050_Accel_t` - Datos de aceleración](#mpu6050_accel_t---datos-de-aceleración)
      - [`MPU6050_Gyro_t` - Datos de giroscopio](#mpu6050_gyro_t---datos-de-giroscopio)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`MPU6050_Init()` - Inicialización del Driver](#mpu6050_init---inicialización-del-driver)
      - [`MPU6050_DeInit()` - Desinicialización del Driver](#mpu6050_deinit---desinicialización-del-driver)
      - [`MPU6050_Read_Accel()` - Lectura del acelerómetro](#mpu6050_read_accel---lectura-del-acelerómetro)
      - [`MPU6050_Read_Gyro()` - Lectura del giroscopio](#mpu6050_read_gyro---lectura-del-giroscopio)
      - [`MPU6050_Read_Temp()` - Lectura de temperatura](#mpu6050_read_temp---lectura-de-temperatura)
      - [`MPU6050_Set_Sleep_Mode()` - Modo de bajo consumo](#mpu6050_set_sleep_mode---modo-de-bajo-consumo)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[2.1.0\] - 17-06-2026](#210---17-06-2026)
      - [Added](#added)
      - [Changed](#changed)
    - [\[2.0.1\] - 23-01-2026](#201---23-01-2026)
      - [Fixed](#fixed)
    - [\[2.0.0\] - 21-01-2026](#200---21-01-2026)
      - [Added](#added-1)
      - [Changed](#changed-1)
    - [\[1.2.0\] - 22-09-2025](#120---22-09-2025)
      - [Added](#added-2)
      - [Fixed](#fixed-1)
    - [\[1.1.0\] - 10-01-2025](#110---10-01-2025)
      - [Changed](#changed-2)
    - [\[1.0.0\] - 7-01-2025](#100---7-01-2025)
      - [Added](#added-3)

---

## Descripción
Librería desarrollada en C para la interfaz con el módulo **InvenSense MPU6050** (acelerómetro y giroscopio de 6 ejes) utilizando microcontroladores STM32. Proporciona funciones para inicializar el sensor, leer datos de aceleración y giroscopio en los tres ejes (X, Y, Z), leer la temperatura interna y controlar el modo de bajo consumo. La librería está diseñada para ser fácil de usar, eficiente y compatible con la mayoría de las series STM32 (F1, F4, etc.) utilizando HAL.

El MPU6050 es ampliamente utilizado en aplicaciones de estabilización (drones, gimbals), medición de orientación, robótica y sistemas de navegación inercial.

---

## Características
- **Comunicación I2C**: Abstracción completa de las operaciones de lectura/escritura de registros mediante HAL_I2C.
- **Lectura de aceleración**: Obtención de los tres ejes (X, Y, Z) convertidos a unidades de 'g'.
- **Lectura de giroscopio**: Obtención de los tres ejes (X, Y, Z) convertidos a grados por segundo ('dps').
- **Lectura de temperatura**: Conversión directa a grados Celsius.
- **Manejo robusto de errores**: Códigos de retorno específicos para error de I2C, timeout, parámetro inválido o módulo no inicializado (`MPU6050_Status_t`).
- **Modo de bajo consumo**: Activación/desactivación del modo sleep sin perder el resto de la configuración del sensor.
- **Portabilidad**: Compatible con múltiples familias STM32 mediante la capa HAL. Solo requiere cambiar el `#include` del encabezado HAL en `MPU6050.h` (vía `main.h`).

---

## Pinout y Conexiones
### Pines requeridos

| Pin MPU6050 | Dirección | Descripción | Tipo GPIO | Observaciones |
|-------------|-----------|-------------|-----------|---------------|
| **VCC**     | Alimentación | Alimentación del sensor (3.3V o 5V según el módulo) | N/A | Usar el voltaje indicado por el fabricante del módulo |
| **GND**     | Tierra    | — | N/A | — |
| **SDA**     | Input/Output (open-drain) | Línea de datos I2C | `I2C_SDA` | Requiere resistencia de pull-up (suele incluirse en el módulo) |
| **SCL**     | Output (open-drain) | Línea de reloj I2C | `I2C_SCL` | Requiere resistencia de pull-up (suele incluirse en el módulo) |
| **AD0**     | Input (opcional) | Selección de dirección I2C | `GPIO_Input` | A GND → `0xD0`; a VCC → `0xD1` |

> [!NOTE]
> La mayoría de los módulos comerciales del MPU6050 ya incluyen las resistencias de pull-up para SDA/SCL.

> [!WARNING]
> Verifica el voltaje de alimentación soportado por tu módulo específico antes de conectarlo, ya que el chip MPU6050 internamente opera a 3.3V.

---

## Configuración I2C
Configura tu periférico I2C en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | I2C | Modo estándar maestro |
| **I2C Speed Mode** | Standard/Fast Mode | 100 kHz o 400 kHz según tu aplicación |
| **Clock Speed** | 100000 - 400000 Hz | Ajustar según el bus y otros dispositivos conectados |
| **Addressing Mode** | 7-bit | El MPU6050 utiliza direccionamiento de 7 bits |

> [!NOTE]
> La dirección I2C del MPU6050 es: **0xD0** (formato de 8 bits, pin AD0 a GND). La librería ya la tiene definida como `MPU6050_ADDRESS 0xD0`.

---

## Instalación
1. Copia `MPU6050.c` y `MPU6050.h` a tu proyecto (ej: `Librerias/MPU6050/`).
2. El encabezado HAL se incluye automáticamente a través de `main.h` en `MPU6050.h`:
   ```c
   #include "main.h"
   ```
   Asegúrate de que `main.h` incluya el HAL correspondiente a tu familia STM32 (`stm32f1xx_hal.h`, `stm32f4xx_hal.h`, etc.).
3. Incluye la librería en tu `main.c` o archivo principal:
```c
#include "MPU6050.h"
```
3. Configura I2C y GPIOs en CubeMX (ver sección anterior).
4. Genera código y compila.

---

## Uso Básico

### 1. Inicialización
```c
// En main() después de HAL_Init() y MX_I2C1_Init()
MPU6050_Status_t status = MPU6050_Init(&hi2c1);

if (status != MPU6050_OK) {
    Error_Handler();  // Módulo no responde o handle NULL
}
```

### 2. Lectura de aceleración y giroscopio
```c
MPU6050_Accel_t accel;
MPU6050_Gyro_t gyro;

MPU6050_Read_Accel(&accel);
printf("Accel X: %.2f, Y: %.2f, Z: %.2f\n", accel.x, accel.y, accel.z);

MPU6050_Read_Gyro(&gyro);
printf("Gyro X: %.2f, Y: %.2f, Z: %.2f\n", gyro.x, gyro.y, gyro.z);
```

### 3. Lectura de temperatura y modo sleep
```c
float temp;
MPU6050_Read_Temp(&temp);
printf("Temp: %.2f C\n", temp);

// Poner el sensor en modo de bajo consumo
MPU6050_Set_Sleep_Mode(1);

// Despertar el sensor
MPU6050_Set_Sleep_Mode(0);
```
> [!NOTE]
> Se recomienda esperar al menos 1 ms después de `MPU6050_Init()` antes de realizar la primera lectura, para permitir que el sensor estabilice sus registros internos.

---

## API Reference

### 1. Tipos de Datos

#### `MPU6050_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería, permitiendo una gestión robusta de errores y estados del chip.

```c
typedef enum {
    MPU6050_OK              = 0,    /**< Operación exitosa */
    MPU6050_ERROR           = 1,    /**< Error en la operación */
    MPU6050_TIMEOUT         = 2,    /**< Timeout en la operación */
    MPU6050_NOT_INITIALIZED = 3,    /**< Módulo no inicializado */
    MPU6050_INVALID_PARAM   = 4     /**< Parámetro inválido */
} MPU6050_Status_t;
```

|Valor|Código|Significado|
|---|---|---|
|`MPU6050_OK`|0|Operación completada sin errores|
|`MPU6050_ERROR`|1|Error de I2C o condición inesperada|
|`MPU6050_TIMEOUT`|2|Timeout de comunicación I2C|
|`MPU6050_NOT_INITIALIZED`|3|`MPU6050_Init()` no se llamó o falló|
|`MPU6050_INVALID_PARAM`|4|Puntero de parámetro NULL o valor fuera de rango|

---

#### `MPU6050_Accel_t` - Datos de aceleración
Estructura que agrupa los valores de aceleración de los tres ejes, en unidades de 'g'.

```c
typedef struct {
    float x;    /**< Valor de aceleración en el eje X. */
    float y;    /**< Valor de aceleración en el eje Y. */
    float z;    /**< Valor de aceleración en el eje Z. */
} MPU6050_Accel_t;
```

|Campo|Tipo|Unidad|Descripción|
|---|---|---|---|
|`x`|`float`|g|Aceleración en el eje X|
|`y`|`float`|g|Aceleración en el eje Y|
|`z`|`float`|g|Aceleración en el eje Z|

---

#### `MPU6050_Gyro_t` - Datos de giroscopio

Estructura que agrupa los valores de velocidad angular de los tres ejes, en grados por segundo.

```c
typedef struct {
    float x;    /**< Valor de velocidad angular en el eje X. */
    float y;    /**< Valor de velocidad angular en el eje Y. */
    float z;    /**< Valor de velocidad angular en el eje Z. */
} MPU6050_Gyro_t;
```

|Campo|Tipo|Unidad|Descripción|
|---|---|---|---|
|`x`|`float`|dps|Velocidad angular en el eje X|
|`y`|`float`|dps|Velocidad angular en el eje Y|
|`z`|`float`|dps|Velocidad angular en el eje Z|

---

### 2. Funciones Públicas

#### `MPU6050_Init()` - Inicialización del Driver

Inicializa el módulo configurando los registros necesarios mediante I2C.

```c
MPU6050_Status_t MPU6050_Init(I2C_HandleTypeDef* hi2c);
```
|Parámetro|Tipo|Descripción|
|---|---|---|
|`hi2c`|`I2C_HandleTypeDef*`|Puntero a la estructura de manejo del I2C|

**Retorna**: `MPU6050_OK` si la inicialización fue exitosa, `MPU6050_ERROR`/`MPU6050_TIMEOUT` si el sensor no responde, `MPU6050_INVALID_PARAM` si `hi2c` es NULL.

**Secuencia interna:**

1. Verifica que el puntero `hi2c` no sea NULL.
2. Configura el registro `PWR_MGMT_1` para despertar el sensor (desactivar el modo sleep).
3. Configura los registros `SMPLRT_DIV`, `GYRO_CONFIG` y `ACCEL_CONFIG` con los valores predeterminados de muestreo y rango.
4. Marca el módulo como inicializado y almacena el manejador I2C.

---

#### `MPU6050_DeInit()` - Desinicialización del Driver

Desinicializa el sensor, liberando el manejador I2C y marcando el módulo como no inicializado.

```c
MPU6050_Status_t MPU6050_DeInit(void);
```

**Retorna**: Siempre retorna `MPU6050_OK`.

---

#### `MPU6050_Read_Accel()` - Lectura del acelerómetro

Lee los valores de aceleración en los tres ejes y los convierte a unidades de 'g'.

```c
MPU6050_Status_t MPU6050_Read_Accel(MPU6050_Accel_t *accel);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`accel`|`MPU6050_Accel_t*`|Puntero a la estructura donde se almacenarán los valores de aceleración|

**Retorna**: `MPU6050_OK` si la lectura fue exitosa, `MPU6050_ERROR` si falla la comunicación I2C, `MPU6050_INVALID_PARAM` si `accel` es NULL, `MPU6050_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `MPU6050_Read_Gyro()` - Lectura del giroscopio

Lee los valores de velocidad angular en los tres ejes y los convierte a grados por segundo ('dps').

```c
MPU6050_Status_t MPU6050_Read_Gyro(MPU6050_Gyro_t *gyro);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`gyro`|`MPU6050_Gyro_t*`|Puntero a la estructura donde se almacenarán los valores de velocidad angular|

**Retorna**: `MPU6050_OK` si la lectura fue exitosa, `MPU6050_ERROR` si falla la comunicación I2C, `MPU6050_INVALID_PARAM` si `gyro` es NULL, `MPU6050_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `MPU6050_Read_Temp()` - Lectura de temperatura

Lee la temperatura del sensor y la convierte a grados Celsius.

```c
MPU6050_Status_t MPU6050_Read_Temp(float *temp);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`temp`|`float*`|Puntero para almacenar la temperatura en grados Celsius|

**Retorna**: `MPU6050_OK` si la lectura fue exitosa, `MPU6050_ERROR` si falla la comunicación I2C, `MPU6050_INVALID_PARAM` si `temp` es NULL, `MPU6050_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `MPU6050_Set_Sleep_Mode()` - Modo de bajo consumo

Activa o desactiva el modo de bajo consumo (sleep) del MPU6050 sin reinicializar el sensor, preservando el resto de la configuración del registro `PWR_MGMT_1`.

```c
MPU6050_Status_t MPU6050_Set_Sleep_Mode(uint8_t enable);
```

|Parámetro|Tipo|Descripción|Rango|
|---|---|---|---|
|`enable`|`uint8_t`|`1` para activar el modo sleep, `0` para despertar el sensor|0 - 1|

**Retorna**: `MPU6050_OK` si la operación fue exitosa, `MPU6050_ERROR` si falla la comunicación I2C, `MPU6050_NOT_INITIALIZED` si el módulo no fue inicializado.

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [2.1.0] - 17-06-2026

#### Added
- Función `MPU6050_DeInit()` para desinicializar el sensor y liberar recursos.
- Función `MPU6050_Read_Temp()` para leer la temperatura del sensor.
- Función `MPU6050_Set_Sleep_Mode()` para activar/desactivar el modo de bajo consumo sin reinicializar el sensor.
- Estructuras `MPU6050_Accel_t` y `MPU6050_Gyro_t` para agrupar los valores de los tres ejes.
- Nuevo valor `MPU6050_INVALID_PARAM` en `MPU6050_Status_t` para indicar parámetros inválidos.

#### Changed
- `MPU6050_Read_Accel()` y `MPU6050_Read_Gyro()` ahora reciben un único puntero a estructura (`MPU6050_Accel_t*` / `MPU6050_Gyro_t*`) en lugar de tres punteros `float*` separados.

---

### [2.0.1] - 23-01-2026

#### Fixed
- Eliminación de warnings en la compilación por variables no usadas.

---

### [2.0.0] - 21-01-2026

#### Added
- Sistema de gestión de errores con tipo `MPU6050_Status_t`.
- Comentarios Doxygen detallados para cada función y parámetro.

#### Changed
- Las funciones principales ahora retornan `MPU6050_Status_t` en lugar de `void`.

---

### [1.2.0] - 22-09-2025

#### Added
- Manejador de la interfaz I2C como parámetro en `MPU6050_Init()`.

#### Fixed
- Corrección de errores menores en la documentación.

---

### [1.1.0] - 10-01-2025

#### Changed
- Renombrado del archivo de `accelerometer.h` a `MPU6050.h`.
- Renombradas las funciones: `accelerometer_read()` → `MPU6050_Read_Accel()` y `gyroscope_read()` → `MPU6050_Read_Gyro()`.
- Renombrados los parámetros de entrada: `*x`, `*y`, `*z` → `*Ax`, `*Ay`, `*Az` (acelerómetro) y `*Gx`, `*Gy`, `*Gz` (giroscopio).

---

### [1.0.0] - 7-01-2025

#### Added
- Versión inicial con funciones básicas de inicialización y lectura de datos.
- Soporte para lectura de aceleración y giroscopio en los tres ejes (X, Y, Z).
