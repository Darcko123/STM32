# Librería para el módulo RTC DS3231 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-2.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/DS3231(RTC_Module))
[![Protocol](https://img.shields.io/badge/Protocol-I2C-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/DS3231(RTC_Module))

---

## Tabla de Contenidos
- [Librería para el módulo RTC DS3231 en STM32](#librería-para-el-módulo-rtc-ds3231-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
  - [Configuración I2C](#configuración-i2c)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2.  Configurar y leer la hora](#2--configurar-y-leer-la-hora)
    - [3. Configurar y leer la Alarma 1](#3-configurar-y-leer-la-alarma-1)
    - [4. Configurar y leer la Alarma 2](#4-configurar-y-leer-la-alarma-2)
    - [5. Leer la temperatura interna](#5-leer-la-temperatura-interna)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`DS3231_Status_t` - Estados de Retorno](#ds3231_status_t---estados-de-retorno)
      - [`ds3231_time_t` - Configuración de Parámetros de Tiempo](#ds3231_time_t---configuración-de-parámetros-de-tiempo)
      - [`ds3231_alarm1_t` - Configuración de Alarma 1](#ds3231_alarm1_t---configuración-de-alarma-1)
      - [`ds3231_alarm2_t` - Configuración de Alarma 2](#ds3231_alarm2_t---configuración-de-alarma-2)
      - [`ds3231_temp_t` - Temperatura Interna](#ds3231_temp_t---temperatura-interna)
    - [Funciones Públicas](#funciones-públicas)
      - [`DS3231_Init()` - Inicialización del Driver](#ds3231_init---inicialización-del-driver)
      - [`DS3231_SetTime()` - Configurar Fecha y Hora](#ds3231_settime---configurar-fecha-y-hora)
      - [`DS3231_GetTime()` - Obtener Fecha y Hora](#ds3231_gettime---obtener-fecha-y-hora)
      - [`DS3231_SetAlarm1()` / `DS3231_GetAlarm1()` - Alarma 1](#ds3231_setalarm1--ds3231_getalarm1---alarma-1)
      - [`DS3231_SetAlarm2()` / `DS3231_GetAlarm2()` - Alarma 2](#ds3231_setalarm2--ds3231_getalarm2---alarma-2)
      - [`DS3231_GetTemperature()` - Leer Temperatura](#ds3231_gettemperature---leer-temperatura)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[2.0.0\] - 09-04-2025](#200---09-04-2025)
      - [Added](#added)
      - [Changed](#changed)
    - [\[1.1.0\] - 22-09-2025](#110---22-09-2025)
      - [Added](#added-1)
      - [Changed](#changed-1)
    - [\[1.0.0\] - 07-01-2025](#100---07-01-2025)
      - [Added](#added-2)

---

## Descripción
Librería desarrollada en C para la interfaz con el módulo RTC (Real-Time Clock) de alta precisión **Maxim Integrated DS3231** utilizando microcontroladores STM32. Proporciona funciones para configurar y leer la hora/fecha, gestionar las dos alarmas disponibles y leer la temperatura interna del sensor, todo a través de la interfaz I2C. La librería está diseñada para ser fácil de usar, eficiente y compatible con la mayoría de las series STM32 (F1, F4, etc.) utilizando HAL.

---

## Características
- **Comunicación I2C**: Abstracción completa de las comunicaciones con el DS3231.
- **Gestión completa de tiempo**: Configuración y lectura de segundos, minutos, hora, día del mes, mes y año.
- **Dos Alarmas independientes**:
  - Alarma 1: Resolución de segundos.
  - Alarma 2: Resolución de minutos.
- **Sensor de temperatura interno**: Permite leer la temperatura del chip con una resolución de 0.25°C.
- **Manejo robusto de errores**: Códigos de retorno específicos para error de I2C, timeout o módulo no inicializado (`DS3231_Status_t`).
- **Conversión automática BCD/Decimal**: La librería convierte internamente entre los formatos, liberando al usuario de esta tarea.
- **Portabilidad**: Compatible con múltiples familias STM32 mediante la capa HAL. Solo requiere cambiar el `#include` del encabezado HAL en `DS3231.h`.

---

## Pinout y Conexiones
### Pines requeridos

| Pin DS3231 | Dirección | Descripción | Tipo GPIO | Observaciones |
|------------|-----------|-------------|-----------|---------------|
| **VCC**    | Alimentación | 2.3V a 5.5V | N/A | Puede alimentarse a 3.3V o 5V |
| **GND**    | Tierra    | — | N/A | — |
| **SCL**    | Input (open-drain) | Línea de reloj I2C | `I2C_SCL` | Requiere resistencia pull-up (típicamente 4.7kΩ) |
| **SDA**    | Input/Output (open-drain) | Línea de datos I2C | `I2C_SDA` | Requiere resistencia pull-up (típicamente 4.7kΩ) |
| **SQW/INT**| Output (opcional) | Salida de onda cuadrada o interrupción de alarma | `GPIO_INPUT` o `GPIO_EXTI` | Se puede conectar a un pin GPIO para detectar alarmas |

> [!NOTE]
> Las resistencias pull-up en las líneas SDA y SCL son **obligatorias** para el correcto funcionamiento de I2C. Muchos módulos comerciales ya las incluyen.

---

## Configuración I2C
Configura tu periférico I2C en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | I2C | Selecciona el modo I2C |
| **Speed** | Fast Mode (400 kHz) | El DS3231 soporta hasta 400 kHz |
| **Clock Speed** | 100 kHz o 400 kHz | Recomendado 100 kHz para debug, luego 400 kHz |
| **Primary Address Length** | 7-bit | Dirección estándar |
| **Own Address 1** | 0x00 | No relevante para modo maestro |

> [!NOTE]
> La dirección I2C del DS3231 es fija: **0x68** (escrita) / **0xD0** (para escritura con bit R/W). La librería ya la tiene definida como `DS3231_ADDRESS 0xD0`.

---

## Instalación
1. Copia `DS3231.c` y `DS3231.h` a tu proyecto (ej: `Librerias/DS3231/`).
2. Ajusta el `#include` del encabezado HAL en `DS3231.h` según tu familia STM32:
  ```c
  // STM32F1xx:
  #include "stm32f1xx_hal.h"
  // STM32F4xx:
  #include "stm32f4xx_hal.h"
  ```
3. Incluye la librería en tu main.c o archivo principal:
  ```c
  #include "DS3231.h"
  ```
4. Configura I2C y GPIOs en CubeMX (ver sección anterior).
5. Genera código y compila.

---

## Uso Básico
### 1. Inicialización

```c
// En main() después de HAL_Init() y MX_I2Cx_Init()
DS3231_Status_t status = DS3231_Init(&hi2c1); // &hi2c1, &hi2c2, etc.

if (status != DS3231_OK) {
    Error_Handler();  // Módulo no responde o handle NULL
}
```

### 2.  Configurar y leer la hora

```c
ds3231_time_t current_time;

// Establecer la hora: 14:30:45, 15 de agosto de 2025
if (DS3231_SetTime(14, 30, 45, 15, 8, 25) != DS3231_OK) {
    // Error al configurar la hora
}

// Leer la hora actual
if (DS3231_GetTime(&current_time) == DS3231_OK) {
    // current_time.hour, current_time.minutes, current_time.seconds...
    printf("Hora: %02d:%02d:%02d\r\n", current_time.hour, current_time.minutes, current_time.seconds);
}
```

### 3. Configurar y leer la Alarma 1

```c
ds3231_alarm1_t alarm1_cfg;

// Configurar Alarma 1 para las 08:30:15
if (DS3231_SetAlarm1(8, 30, 15) != DS3231_OK) {
    // Error al configurar la alarma
}

// Leer la configuración actual de la Alarma 1
if (DS3231_GetAlarm1(&alarm1_cfg) == DS3231_OK) {
    // alarm1_cfg.hour, alarm1_cfg.minutes, alarm1_cfg.seconds...
}
```

### 4. Configurar y leer la Alarma 2

```c
ds3231_alarm2_t alarm2_cfg;

// Configurar Alarma 2 para las 09:45 (sin segundos)
if (DS3231_SetAlarm2(9, 45) != DS3231_OK) {
    // Error al configurar la alarma
}

// Leer la configuración actual de la Alarma 2
if (DS3231_GetAlarm2(&alarm2_cfg) == DS3231_OK) {
    // alarm2_cfg.hour, alarm2_cfg.minutes...
}
```

### 5. Leer la temperatura interna

```c
ds3231_temp_t temperature;

if (DS3231_GetTemperature(&temperature) == DS3231_OK) {
    // Usar los campos integer y fraction para evitar floats
    printf("Temperatura: %d.%02d °C\r\n", temperature.integer, temperature.fraction);
    
    // O usar el campo raw (centésimas de grado)
    int16_t temp_raw = temperature.raw; // Ej: 2350 = 23.50°C
}
```

---

## API Reference

### 1. Tipos de Datos

#### `DS3231_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería, permitiendo una gestión robusta de errores y estados del chip.

```c
typedef enum {
    DS3231_OK = 0,              /**< Operación exitosa */
    DS3231_ERROR = 1,           /**< Error en la operación */
    DS3231_TIMEOUT = 2,         /**< Timeout en la operación */
    DS3231_NOT_INITIALIZED = 3, /**< Módulo no inicializado */
} DS3231_Status_t;
```

| Valor | Código | Significado |
|-------|--------|-------------|
| `DS3231_OK` | 0 | Operación completada sin errores |
| `DS3231_ERROR` | 1 | Error de I2C, parámetro inválido o condición inesperada |
| `DS3231_TIMEOUT` | 2 | Timeout de software o timeout interno del chip |
| `DS3231_NOT_INITIALIZED` | 3 | `DS3231_Init()` no se llamó o falló |

---

#### `ds3231_time_t` - Configuración de Parámetros de Tiempo

Estructura que encapsula todos los parámetros configurables de la fecha y hora.

```c
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
    uint8_t dayofmonth;
    uint8_t month;
    uint8_t year;
} ds3231_time_t;
```

| Campo | Rango | Descripción |
|-------|-------|-------------|
| `seconds` | 0–59 | Segundos |
| `minutes` | 0–59 | Minutos |
| `hour` | 0–23 | Hora en formato 24h |
| `dayofmonth` | 1–31 | Día del mes |
| `month` | 1–12 | Mes |
| `year` | 0–99 | Año (últimos dos dígitos, p. ej. 25 → 2025) |

---

#### `ds3231_alarm1_t` - Configuración de Alarma 1

```c
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
} ds3231_alarm1_t;
```

| Campo | Rango | Descripción |
|-------|-------|-------------|
| `seconds` | 0–59 | Segundos de la alarma |
| `minutes` | 0–59 | Minutos de la alarma |
| `hour` | 0–23 | Hora de la alarma en formato 24h |

---

#### `ds3231_alarm2_t` - Configuración de Alarma 2

```c
typedef struct {
    uint8_t minutes;
    uint8_t hour;
} ds3231_alarm2_t;
```

| Campo | Rango | Descripción |
|-------|-------|-------------|
| `minutes` | 0–59 | Minutos de la alarma |
| `hour` | 0–23 | Hora de la alarma en formato 24h |

> [!NOTE]
> La Alarma 2 no soporta segundos; su resolución mínima es de un minuto.

---

#### `ds3231_temp_t` - Temperatura Interna

Estructura para almacenar la temperatura del sensor interno del DS3231

```c
typedef struct {
    int8_t  integer;   /**< Parte entera (°C), rango: -128..+127 */
    uint8_t fraction;  /**< Parte fraccionaria en centésimas: 0, 25, 50 ó 75 */
    int16_t raw;       /**< Temperatura escalada ×100 (centésimas de °C) */
} ds3231_temp_t;
```

| Campo | Tipo | Descripción |
|-------|------|-------------|
| `integer` | `int8_t` | Parte entera de la temperatura en °C (puede ser negativo) |
| `fraction` | `uint8_t` | Parte fraccionaria en centésimas: `0`, `25`, `50` ó `75` |
| `raw` | `int16_t` | Temperatura completa escalada ×100. Ejemplo: `2350` → 23.50 °C, `-625` → −6.25 °C |

```c
// Ejemplo de impresión sin punto flotante
ds3231_temp_t temp;
DS3231_GetTemperature(&temp);
printf("Temperatura: %d.%02d °C\n", temp.integer, temp.fraction);
```

---

### Funciones Públicas

#### `DS3231_Init()` - Inicialización del Driver

Inicializa el driver, almacenando el handle I2C para futuras operaciones. Todas las demás funciones retornan DS3231_NOT_INITIALIZED si esta función no se ha llamado exitosamente.

```c
DS3231_Status_t DS3231_Init(I2C_HandleTypeDef* hi2c);
```

| Parametro | Tipo | Descripción |
|-----------|------|-------------|
| hi2c | I2C_HandleTypeDef*	| Handle del periférico I2C configurado |

**Retorna**: `DS3231_OK` si el handle es válido, `DS3231_ERROR` si es NULL

---

#### `DS3231_SetTime()` - Configurar Fecha y Hora

Configura la fecha y hora en el RTC. Los valores se convierten automáticamente a BCD antes de ser escritos en el chip.

```c
DS3231_Status_t DS3231_SetTime(uint8_t hour, uint8_t min, uint8_t sec, uint8_t dom, uint8_t month, uint8_t year);
```


| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| hour | uint8_t | Hora | 0-23 |
| min | uint8_t | Minutos | 0-59 |
| sec | uint8_t | Segundos | 0-59 |
| dom | uint8_t | Día del mes | 1-31 |
| month | uint8_t | Mes | 1-12 |
| year | uint8_t | Año (últimos dos dígitos) | 0-99 |

**Retorna**: `DS3231_OK` si la operación fue exitosa, `DS3231_ERROR` si la comunicación I2C falla, `DS3231_NOT_INITIALIZED` si el módulo no fue inicializado.

> [!NOTE]
> El día de la semana es gestionado automáticamente por el DS3231. La librería escribe un valor fijo (1) en este registro, ya que la mayoría de las aplicaciones no lo utilizan.

#### `DS3231_GetTime()` - Obtener Fecha y Hora
Lee la fecha y hora actual desde el RTC.

```c
DS3231_Status_t DS3231_GetTime(ds3231_time_t *time);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| time | ds3231_time_t* | Puntero a la estructura donde se almacenarán los valores leídos |

**Retorna**: `DS3231_OK` si la lectura fue exitosa, `DS3231_ERROR` si falla la comunicación I2C o `time` es NULL, `DS3231_NOT_INITIALIZED` si el módulo no fue inicializado.

#### `DS3231_SetAlarm1()` / `DS3231_GetAlarm1()` - Alarma 1
Configura o lee la Alarma 1 del DS3231. La Alarma 1 permite configurar segundos, minutos y hora.

```c
DS3231_Status_t DS3231_SetAlarm1(uint8_t hourAlarm, uint8_t minAlarm, uint8_t secAlarm);
DS3231_Status_t DS3231_GetAlarm1(ds3231_alarm1_t *alarma1);
```

**Retorna**: Los mismos códigos que las funciones anteriores.

#### `DS3231_SetAlarm2()` / `DS3231_GetAlarm2()` - Alarma 2
Configura o lee la Alarma 2 del DS3231. La Alarma 2 no soporta segundos, solo minutos y hora.

```c
DS3231_Status_t DS3231_SetAlarm2(uint8_t hourAlarm, uint8_t minAlarm);
DS3231_Status_t DS3231_GetAlarm2(ds3231_alarm2_t *alarma2);
```

**Retorna**: Los mismos códigos que las funciones anteriores.

#### `DS3231_GetTemperature()` - Leer Temperatura
Lee la temperatura interna del sensor del DS3231. El valor se actualiza automáticamente cada 64 segundos.

```c
DS3231_Status_t DS3231_GetTemperature(ds3231_temp_t *temp);
```

| Parámetro	| Tipo | Descripción |
|-----------|------|-------------|
| temp | ds3231_temp_t* | Puntero a la estructura donde se almacenará la temperatura |

**Retorna**: `DS3231_OK` si la lectura fue exitosa, `DS3231_ERROR` si falla la comunicación I2C o `temp` es NULL, `DS3231_NOT_INITIALIZED` si el módulo no fue inicializado.

---

## Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [2.0.0] - 09-04-2025

#### Added
- Sistema de gestión de errores con códigos de retorno específicos (`DS3231_Status_t`).
- Función `DS3231_GetTemperature()` para leer el sensor de temperatura interno.
- Estructura `ds3231_temp_t` para manejar la temperatura sin punto flotante.
- Documentación completa en formato Doxygen.

#### Changed
- Cambio de nombre de todas las funciones públicas de `RTC_*` a `DS3231_*` para mayor claridad y consistencia.
- La función `DS3231_SetTime()` ahora escribe un valor fijo en el registro del día de la semana (no se usa en esta API).

---

### [1.1.0] - 22-09-2025

#### Added
- Manejador de la interfaz I2C como parámetro en las funciones de inicialización y lectura.
- Soporte para múltiples instancias del RTC mediante el paso del handle I2C.

#### Changed
- Modificación de las funciones públicas para recibir el handle I2C como argumento.

---

### [1.0.0] - 07-01-2025

#### Added
- Versión inicial con funciones básicas de inicialización, lectura y escritura de datos.
- Soporte para configuración de hora y fecha mediante `RTC_SetTime()`.
- Soporte para lectura de hora y fecha mediante `RTC_GetTime()`.
- Configuración y lectura de Alarma 1 (`RTC_SetAlarm1()`, `RTC_GetAlarm1()`).
- Configuración y lectura de Alarma 2 (`RTC_SetAlarm2()`, `RTC_GetAlarm2()`).
- Estructuras `TIME`, `ALARM1` y `ALARM2` para almacenamiento de datos.
- Conversión automática entre formato BCD y decimal.