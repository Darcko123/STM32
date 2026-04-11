# Librería para el módulo SI7021 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-2.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/SI7021)
[![Protocol](https://img.shields.io/badge/Protocol-I2C-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/Si7021)

---

## Tabla de Contenidos
- [Librería para el módulo SI7021 en STM32](#librería-para-el-módulo-si7021-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
  - [Configuración I2C](#configuración-i2c)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Adquisición de datos](#2-adquisición-de-datos)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`SI7021_Status_t` - Estados de Retorno](#si7021_status_t---estados-de-retorno)
      - [`si7021_data_t` - Estructura de Datos](#si7021_data_t---estructura-de-datos)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`SI7021_Init()` - Inicialización del Driver](#si7021_init---inicialización-del-driver)
      - [`SI7021_Get()` - Obtención de Datos del Entorno](#si7021_get---obtención-de-datos-del-entorno)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[2.0.0\] - 10-04-2026](#200---10-04-2026)
      - [Added](#added)
    - [\[1.2.0\] - 22-09-2025](#120---22-09-2025)
      - [Added](#added-1)
      - [Fixed](#fixed)
    - [\[1.1.0\] - 13-01-2025](#110---13-01-2025)
      - [Changed](#changed)
    - [\[1.0.0\] - 28-12-2024](#100---28-12-2024)
      - [Added](#added-2)

---

## Descripción
Librería desarrollada en C para la interfaz con el sensor de temperatura y humedad **Silicon Labs SI7021** utilizando microcontroladores STM32. Proporciona funciones para inicializar el sensor y leer los valores de temperatura y humedad relativa a través del bus I2C. La librería está diseñada para ser fácil de usar, eficiente y compatible con la mayoría de las series STM32 (F1, F4, etc.) utilizando HAL.

El sensor SI7021 es ampliamente utilizado en aplicaciones de monitoreo ambiental, HVAC y sistemas embebidos debido a su precisión (±0.4°C y ±3% RH) y facilidad de integración.

---

## Características
- **Comunicación I2C**: Abstracción completa de las comunicaciones con el SI7021.
- **Medición de temperatura**: Rango de -40°C a +125°C, resolución de 14 bits (hasta 0.01°C).
- **Medición de humedad relativa**: Rango de 0% a 100%, resolución de 12 bits (hasta 0.04% RH).
- **Reset por hardware**: Al inicializar, se envía el comando de reset para asegurar un estado conocido.
- **Manejo robusto de errores**: Códigos de retorno específicos para error de I2C, timeout o módulo no inicializado (`SI7021_Status_t`).
- **Cálculo automático**: Conversión de datos crudos a valores físicos utilizando las fórmulas del datasheet.
- **Portabilidad**: Compatible con múltiples familias STM32 mediante la capa HAL. Solo requiere cambiar el `#include` del encabezado HAL en `SI7021.h`.

---

## Pinout y Conexiones
### Pines requeridos

| Pin SI7021 | Dirección | Descripción | Tipo GPIO | Observaciones |
|------------|-----------|-------------|-----------|---------------|
| **VCC**    | Alimentación | 1.9V a 3.6V | N/A | **Importante:** usar solo 3.3V, **no 5V** |
| **GND**    | Tierra    | — | N/A | — |
| **SCL**    | Input (open-drain) | Línea de reloj I2C | `I2C_SCL` | Requiere resistencia pull-up (típicamente 4.7kΩ) |
| **SDA**    | Input/Output (open-drain) | Línea de datos I2C | `I2C_SDA` | Requiere resistencia pull-up (típicamente 4.7kΩ) |

> [!WARNING]
> El SI7021 **NO es tolerante a 5V** en sus pines de alimentación ni en las líneas I2C. Alimentarlo a 5V puede dañarlo permanentemente. Si tu microcontrolador trabaja a 5V, debes usar un nivel lógico (level shifter) en las líneas SDA y SCL.

> [!NOTE]
> Las resistencias pull-up en las líneas SDA y SCL son **obligatorias** para el correcto funcionamiento de I2C. Muchos módulos comerciales ya las incluyen.

---

## Configuración I2C
Configura tu periférico I2C en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | I2C | Selecciona el modo I2C |
| **Speed** | Fast Mode (400 kHz) | El SI7021 soporta hasta 400 kHz |
| **Clock Speed** | 100 kHz o 400 kHz | Recomendado 100 kHz para debug, luego 400 kHz |
| **Primary Address Length** | 7-bit | Dirección estándar |
| **Own Address 1** | 0x00 | No relevante para modo maestro |

> [!NOTE]
> La dirección I2C del SI7021 es fija: **0x40** (7 bits) / **0x80** (8 bits con bit R/W). La librería ya la tiene definida como `SI7021_ADDRESS 0x80`.

---

## Instalación
1. Copia `SI7021.c` y `SI7021.h` a tu proyecto (ej: `Librerias/SI7021/`).
2. Ajusta el `#include` del encabezado HAL en `SI7021.h` según tu familia STM32:
  ```c
  // STM32F1xx:
  #include "stm32f1xx_hal.h"
  // STM32F4xx:
  #include "stm32f4xx_hal.h"
  ```
3. Incluye la librería en tu main.c o archivo principal:
  ```c
  #include "SI7021.h"
  ```
4. Configura I2C y GPIOs en CubeMX (ver sección anterior).
5. Genera código y compila.

---

## Uso Básico
### 1. Inicialización

```c
// En main() después de HAL_Init() y MX_I2Cx_Init()
SI7021_Status_t status = SI7021_Init(&hi2c1); // &hi2c1, &hi2c2, etc.

if (status != SI7021_OK) {
    Error_Handler();  // Sensor no responde o handle NULL
}
```

### 2. Adquisición de datos

```c
si7021_data_t entorno;

// Leer temperatura y humedad
if (SI7021_Get(&entorno) == SI7021_OK) {
    printf("Humedad: %.2f%%\tTemperatura: %.2f°C\r\n", entorno.humedad, entorno.temperatura);
}
```

> [!NOTE]
> La función `SI7021_Get()` realiza dos conversiones independientes (primero temperatura, luego humedad). Cada conversión toma aproximadamente 25 ms, por lo que el tiempo total de lectura es de ~50 ms.

---

## API Reference

### 1. Tipos de Datos

#### `SI7021_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería, permitiendo una gestión robusta de errores y estados del chip.

```c
typedef enum {
    SI7021_OK = 0,              /**< Operación exitosa */
    SI7021_ERROR = 1,           /**< Error en la operación */
    SI7021_TIMEOUT = 2,         /**< Timeout en la operación */
    SI7021_NOT_INITIALIZED = 3, /**< Módulo no inicializado */
} SI7021_Status_t;
```

| Valor | Código | Significado |
|-------|--------|-------------|
| `SI7021_OK` | 0 | Operación completada sin errores |
| `SI7021_ERROR` | 1 | Error de I2C, parámetro inválido o condición inesperada |
| `SI7021_TIMEOUT` | 2 | Timeout de software o timeout interno del chip |
| `SI7021_NOT_INITIALIZED` | 3 | `SI7021_Init()` no se llamó o falló |

---

#### `si7021_data_t` - Estructura de Datos

Estructura que encapsula los valores de temperatura y humedad relativa del entorno.

```c
/**
 * @brief Estructura para almacenar los datos de temperatura y humedad.
 */
typedef struct {
    float temperatura;  /**< Temperatura en grados Celsius (°C), rango: -40 a +125 */
    float humedad;      /**< Humedad relativa (%RH), rango: 0 a 100 */
} si7021_data_t;
```

| Campo | Tipo | Rango | Unidad | Descripción |
|-------|------|-------|--------|-------------|
| `temperatura` | `float` | -40 - +125 | °C | Temperatura |
| `humedad` | `float` | 0 - 100 | %RH | Humedad Relativa del entorno |

---

### 2. Funciones Públicas

#### `SI7021_Init()` - Inicialización del Driver

Inicializa el driver, almacenando el handle I2C para futuras operaciones y enviando un comando de reset al sensor para asegurar un estado conocido. Todas las demás funciones retornan `SI7021_NOT_INITIALIZED` si esta función no se ha llamado exitosamente.

```c
SI7021_Status_t SI7021_Init(I2C_HandleTypeDef* hi2c);
```

| Parametro | Tipo | Descripción |
|-----------|------|-------------|
| `hi2c` | `I2C_HandleTypeDef*`	| Handle del periférico I2C configurado |

**Retorna**: `SI7021_OK` si el handle es válido y el reset fue exitoso, `SI7021_ERROR` si el handle es NULL o la comunicación I2C falla.

**Secuencia interna:**
1. Valida que el handle I2C no sea NULL.
2. Almacena el handle en una variable estática.
3. Envía el comando de reset (0xFE) al sensor.
4. Espera 20 ms para que el reset se complete.
5. Marca el módulo como inicializado.

---

#### `SI7021_Get()` - Obtención de Datos del Entorno

Lee los valores de temperatura y humedad del sensor SI7021. Realiza dos conversiones independientes: primero temperatura, luego humedad. Cada conversión requiere un tiempo de espera de aproximadamente 25 ms.

```c
SI7021_Status_t SI7021_Get(si7021_data_t *environment);
```

| Parámetro	| Tipo | Descripción |
|-----------|------|-------------|
| `environment` | `si7021_data_t*` | Puntero a la estructura donde se almacenará la humedad y la temperatura |

**Retorna**: `SI7021_OK` si la lectura fue exitosa, `SI7021_ERROR` si falla la comunicación I2C o `environment` es NULL, `SI7021_NOT_INITIALIZED` si el módulo no fue inicializado.

---

## Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [2.0.0] - 10-04-2026

#### Added
- Sistema de gestión de errores con códigos de retorno específicos (`SI7021_Status_t`).
- Reestructuración completa del código para mejorar la legibilidad y mantenimiento.
- Estructura `si7021_data_t` para manejar la humedad y la temperatura
- Documentación completa en formato Doxygen.

---

### [1.2.0] - 22-09-2025

#### Added
- Manejador de la interfaz I2C como parámetro en `SI7021_Init()`.

#### Fixed
- Corrección de errores menores en la documentación.

---

### [1.1.0] - 13-01-2025

#### Changed
- Cambio a uso de mayúsculas para funciones públicas (`si7021_init` a `SI7021_Init`, `get_si7021` a `SI7021_Get`).

---

### [1.0.0] - 28-12-2024

#### Added
- Librería inicial para comunicación con sensor SI7021 mediante I2C.
- Función `si7021_init()` con reset por hardware.
- Función `get_si7021()` para lectura de temperatura (°C) y humedad relativa (%HR).
- Conversión de datos crudos según fórmulas del datasheet.
- Soporte para STM32 HAL Library.
- Archivos `si7021.h` y `si7021.c`.
- Documentación completa y ejemplo de uso.