# Librería para el módulo AHT10 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/AHT10)
[![Protocol](https://img.shields.io/badge/Protocol-I2C-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/AHT10)

---

## Tabla de Contenidos
- [Librería para el módulo AHT10 en STM32](#librería-para-el-módulo-aht10-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
  - [Configuración I2C](#configuración-i2c)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Obtener Datos](#2-obtener-datos)
    - [3. Desinicialización](#3-desinicialización)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`AHT10_Status_t` - Estados de Retorno](#aht10_status_t---estados-de-retorno)
      - [`AHT10_Data_t` - Estructura de Datos](#aht10_data_t---estructura-de-datos)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`AHT10_Init()` - Inicialización del Driver](#aht10_init---inicialización-del-driver)
      - [`AHT10_Get()` - Lectura de Datos](#aht10_get---lectura-de-datos)
      - [`AHT10_DeInit()` - Desinicialización del Driver](#aht10_deinit---desinicialización-del-driver)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.0.0\] - 20-06-2026](#100---20-06-2026)
      - [Added](#added)

---

## Descripción
Librería desarrollada en C para la interfaz con el módulo sensor de temperatura y humedad **ASAIR AHT10** utilizando microcontroladores STM32. Proporciona funciones para leer la temperatura y la humedad relativa con los coeficientes de calibración de fábrica cargados automáticamente durante la inicialización. La librería está diseñada para ser fácil de usar, eficiente y compatible con la mayoría de las series STM32 (F1, F4, etc.) utilizando HAL.

Ideal para aplicaciones de monitoreo ambiental, estaciones meteorológicas, control de climatización (HVAC) y proyectos de IoT.

---

## Características
- **Comunicación I2C**: Interfaz I2C estándar para envío de comandos y lectura de medición.
- **Calibración de fábrica**: Carga automática de los coeficientes de calibración internos durante la inicialización (`AHT10_INIT_CAL_ENABLE`).
- **Conversión integrada**: Cálculo automático de temperatura (°C) y humedad relativa (%RH) a partir de los datos crudos de 20 bits según el datasheet del AHT10.
- **Saturación de humedad**: La humedad se limita automáticamente al rango válido 0% – 100% RH.
- **Manejo robusto de errores**: Códigos de retorno específicos para error de I2C, módulo no inicializado o parámetro inválido (`AHT10_Status_t`).
- **Portabilidad**: Compatible con múltiples familias STM32 mediante la capa HAL. Solo requiere cambiar el `#include` del encabezado HAL en `AHT10.h`.
- **Instancia única**: El driver gestiona un único sensor AHT10 por proyecto.

---

## Pinout y Conexiones
### Pines requeridos

| Pin AHT10 | Dirección | Descripción | Tipo GPIO | Observaciones |
|-----------|-----------|-------------|-----------|---------------|
| **VCC**   | Alimentación | 1.8V a 3.6V | N/A | **Importante:** usar 3.3V, **no 5V** |
| **GND**   | Tierra    | — | N/A | — |
| **SCL**   | Input (open-drain) | Línea de reloj I2C | `I2C_SCL` | Requiere resistencia pull-up (típicamente 4.7kΩ) |
| **SDA**   | Input/Output (open-drain) | Línea de datos I2C | `I2C_SDA` | Requiere resistencia pull-up (típicamente 4.7kΩ) |

> [!NOTE]
> La mayoría de los módulos comerciales ya integran las resistencias pull-up en las líneas SDA y SCL. El AHT10 opera estrictamente a niveles lógicos de 3.3V.

> [!WARNING]
> No alimentar a 5V. El AHT10 está especificado para un máximo de 3.6V y puede dañarse de forma permanente.

---

## Configuración I2C
Configura tu periférico I2C en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | I2C | — |
| **I2C Speed Mode** | Standard Mode / Fast Mode | Soporta 100 kHz y 400 kHz |
| **I2C Clock Speed** | 100000 o 400000 Hz | Depende del requerimiento |

> [!NOTE]
> La dirección I2C del AHT10 es: **0x38** (7 bits). La librería ya la tiene definida como `AHT10_ADDRESS 0x70`, que corresponde al valor desplazado 1 bit a la izquierda (`0x38 << 1`) requerido por la capa HAL de STM32.

---

## Instalación
1. Copia `AHT10.c` y `AHT10.h` a tu proyecto (ej: `Inc/AHT10.h`, `Src/AHT10.c`).
2. Incluye la librería en tu código fuente:
   ```c
   #include "AHT10.h"
   ```
3. Configura I2C y GPIOs en CubeMX (ver sección anterior).
4. Genera código y compila.

---

## Uso Básico

### 1. Inicialización
```c
// En main() después de HAL_Init() y MX_I2C1_Init()
AHT10_Status_t status = AHT10_Init(&hi2c1);

if (status != AHT10_OK) {
    Error_Handler();  // Módulo no responde, handle NULL o fallo de calibración
}
```

### 2. Obtener Datos

```c
// Declarar estructura de datos
AHT10_Data_t aht_data;

// Leer los datos del sensor
if (AHT10_Get(&aht_data) == AHT10_OK)
{
    // Uso de los datos: aht_data.temperatura, aht_data.humedad
    printf("Temp: %.2f C, Humedad: %.2f %%RH\n",
           aht_data.temperatura, aht_data.humedad);
}
```

### 3. Desinicialización

```c
// Libera el handle de I2C y marca el módulo como no inicializado
AHT10_DeInit();
```

> [!NOTE]
> Tras llamar a `AHT10_Init()`, el sensor requiere un breve tiempo de estabilización interno. La librería gestiona automáticamente los retardos necesarios (`AHT10_POWER_ON_DELAY`, `AHT10_CMD_DELAY` y `AHT10_MEASURMENT_DELAY`) entre el envío de comandos y la lectura de la medición.

---

## API Reference

### 1. Tipos de Datos

#### `AHT10_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería, permitiendo una gestión robusta de errores y estados del sensor.

```c
typedef enum {
    AHT10_OK              = 0,  /**< Operación exitosa */
    AHT10_ERROR           = 1,  /**< Error en la operación */
    AHT10_TIMEOUT         = 2,  /**< Timeout en la operación */
    AHT10_NOT_INITIALIZED = 3,  /**< Módulo no inicializado */
    AHT10_INVALID_PARAM   = 4   /**< Parámetro inválido */
} AHT10_Status_t;
```

|Valor|Código|Significado|
|---|---|---|
|`AHT10_OK`|0|Operación completada sin errores|
|`AHT10_ERROR`|1|Error de I2C, fallo de calibración o comunicación fallida|
|`AHT10_TIMEOUT`|2|Timeout de software o timeout interno del sensor|
|`AHT10_NOT_INITIALIZED`|3|`AHT10_Init()` no se llamó o falló|
|`AHT10_INVALID_PARAM`|4|Parámetro nulo (handle I2C o puntero de datos NULL)|

---

#### `AHT10_Data_t` - Estructura de Datos

Estructura para almacenar los datos de temperatura y humedad leídos del sensor.

```c
typedef struct {
    float humedad;      /**< Humedad relativa (%RH) */
    float temperatura;  /**< Temperatura en grados Celsius (°C) */
} AHT10_Data_t;
```

| Campo | Tipo | Rango | Unidad | Descripción |
|-------|------|-------|--------|-------------|
| `humedad` | `float` | 0 - 100 | %RH | Humedad relativa |
| `temperatura` | `float` | -40 - +85 | °C | Temperatura |

---

### 2. Funciones Públicas

#### `AHT10_Init()` - Inicialización del Driver

Inicializa el sensor AHT10. Configura el sensor en *Normal Mode*, carga los coeficientes de calibración de fábrica y verifica el bit de calibración a través del byte de estado.

```c
AHT10_Status_t AHT10_Init(I2C_HandleTypeDef* hi2c);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`hi2c`|`I2C_HandleTypeDef*`|Puntero al handle de I2C|

**Retorna**: `AHT10_OK` si la inicialización fue exitosa, `AHT10_ERROR` si hay fallo de comunicación o el bit de calibración no se activa, `AHT10_INVALID_PARAM` si el handle es nulo.

**Secuencia interna:**

1. Retardo de encendido para la estabilización interna del sensor.
2. Envío del comando *Normal Mode* (`0xA8`).
3. Carga de coeficientes de calibración de fábrica (`0xE1` + `0x08`).
4. Lectura del byte de estado y verificación del bit BUSY (bit 7) y del bit de calibración (bit 3).

---

#### `AHT10_Get()` - Lectura de Datos

Inicia una medición y lee la temperatura y humedad compensadas del AHT10.

```c
AHT10_Status_t AHT10_Get(AHT10_Data_t* environment);
```

|Parámetro|Tipo|Descripción|Rango|
|---|---|---|---|
|`environment`|`AHT10_Data_t*`|Puntero a la estructura donde se guardarán los datos|N/A|

**Retorna**: `AHT10_OK` si la lectura fue exitosa, `AHT10_ERROR` si hay fallo de comunicación I2C o se pierde la calibración, `AHT10_NOT_INITIALIZED` si el módulo no fue inicializado, `AHT10_INVALID_PARAM` si `environment` es NULL.

**Secuencia interna:**

1. Envío del comando de inicio de medición (`0xAC` + `0x33` + `0x00`).
2. Retardo de conversión.
3. Lectura de 6 bytes de datos crudos.
4. Verificación del bit de calibración del byte de estado.
5. Construcción de los datos crudos de 20 bits y conversión a valores reales (°C y %RH).

---

#### `AHT10_DeInit()` - Desinicialización del Driver

Desinicializa el sensor AHT10, libera el handle de I2C y marca el módulo como no inicializado. Tras llamarla, es necesario invocar nuevamente `AHT10_Init()` antes de realizar lecturas.

```c
AHT10_Status_t AHT10_DeInit(void);
```

**Retorna**: Siempre retorna `AHT10_OK`.

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.  
El formato está basado en [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

### [1.0.0] - 20-06-2026

#### Added
- Versión inicial del driver AHT10 para STM32 con interfaz I2C.
- Inicialización con carga automática de los coeficientes de calibración de fábrica y verificación del byte de estado.
- Lectura de temperatura (°C) y humedad relativa (%RH) con conversión según el datasheet del AHT10.
- Saturación automática de la humedad al rango válido 0% – 100% RH.
- Funciones públicas `AHT10_Init()`, `AHT10_Get()` y `AHT10_DeInit()`.
- Definición de tipos: `AHT10_Status_t`, `AHT10_Data_t`.
- Manejo robusto de errores (I2C, parámetros inválidos, no inicializado).
