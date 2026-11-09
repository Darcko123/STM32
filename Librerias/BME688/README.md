# Librería para el módulo BME688 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/BME688)
[![Protocol](https://img.shields.io/badge/Protocol-I2C-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/BME688)

---

## Tabla de Contenidos
- [Librería para el módulo BME688 en STM32](#librería-para-el-módulo-bme688-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
  - [Configuración I2C](#configuración-i2c)
  - [Configuración del Driver](#configuración-del-driver)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Obtener Datos](#2-obtener-datos)
    - [3. Modo Sleep](#3-modo-sleep)
    - [4. Desinicialización](#4-desinicialización)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`BME688_Status_t` - Estados de Retorno](#bme688_status_t---estados-de-retorno)
      - [`BME688_Data_t` - Estructura de Datos](#bme688_data_t---estructura-de-datos)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`BME688_Init()` - Inicialización del Driver](#bme688_init---inicialización-del-driver)
      - [`BME688_Get()` - Lectura de Datos](#bme688_get---lectura-de-datos)
      - [`BME688_Sleep()` - Modo Sleep](#bme688_sleep---modo-sleep)
      - [`BME688_DeInit()` - Desinicialización del Driver](#bme688_deinit---desinicialización-del-driver)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.0.0\] - 08-11-2026](#100---08-11-2026)
      - [Added](#added)

---

## Descripción
Librería desarrollada en C para la interfaz con el sensor ambiental **Bosch BME688** utilizando microcontroladores STM32. Mide temperatura, presión barométrica, humedad relativa y la resistencia del sensor de gas (MOX) en *forced mode*, aplicando los coeficientes de calibración de fábrica que se leen automáticamente durante la inicialización. La compensación usa los algoritmos en punto flotante de Bosch. La librería está diseñada para ser fácil de usar y compatible con la mayoría de las series STM32 (F1, F4, etc.) utilizando HAL.

El driver detecta la variante del chip (registro `variant_id`) y también funciona con el **BME680**, usando en ese caso su posición de datos de gas y su fórmula de compensación.

Ideal para monitoreo de calidad del aire interior, estaciones meteorológicas, control de climatización (HVAC) y proyectos de IoT.

---

## Características
- **Comunicación I2C**: Interfaz I2C estándar con lecturas y escrituras de registros mediante `HAL_I2C_Mem_Read/Write`.
- **Detección automática de dirección**: `BME688_Init()` prueba `0x76` (SDO = GND) y, si no responde, `0x77` (SDO = VCC).
- **Verificación del chip**: Comprueba el Chip ID (`0x61`) antes y después del *soft reset*.
- **Calibración de fábrica**: Lectura de los tres bloques de coeficientes (`0x8A`, `0xE1`, `0x00`) durante la inicialización.
- **Medición en forced mode**: Cada `BME688_Get()` dispara una conversión, espera su duración calculada y lee temperatura (°C), presión (hPa), humedad (%RH) y resistencia de gas (Ω).
- **Validez del gas**: El campo `gas_valido` indica si la medición de gas es válida y el calentador alcanzó su temperatura objetivo.
- **Compatible con BME680**: Selección automática de `run_gas` y de la compensación de gas según la variante.
- **Saturación de humedad**: La humedad se limita automáticamente al rango 0% – 100% RH.
- **Bajo consumo**: El sensor vuelve solo a sleep tras cada medición; `BME688_Sleep()` y `BME688_DeInit()` lo garantizan o abortan una conversión en curso.
- **Esperas acotadas**: Los sondeos de `new_data` y de cambio de modo tienen un número máximo de intentos y devuelven `BME688_TIMEOUT` en lugar de bloquear indefinidamente.
- **Manejo robusto de errores**: Códigos de retorno específicos para error de I2C, timeout, módulo no inicializado o parámetro inválido (`BME688_Status_t`).
- **Instancia única**: El driver gestiona un único sensor BME688 por proyecto.

---

## Pinout y Conexiones
### Pines requeridos

| Pin BME688 | Dirección | Descripción | Tipo GPIO | Observaciones |
|------------|-----------|-------------|-----------|---------------|
| **VCC**    | Alimentación | Alimentación del sensor | N/A | Ver nota sobre el voltaje |
| **GND**    | Tierra    | — | N/A | — |
| **SCL**    | Input (open-drain) | Línea de reloj I2C | `I2C_SCL` | Requiere resistencia pull-up |
| **SDA**    | Input/Output (open-drain) | Línea de datos I2C | `I2C_SDA` | Requiere resistencia pull-up |
| **SDO**    | Input | Selección de dirección I2C | N/A | GND → `0x76`, VCC → `0x77` (el driver detecta ambas) |
| **CS**     | Input | Selección de interfaz | N/A | Debe estar en alto para usar I2C |

> [!NOTE]
> El chip BME688 se alimenta con un máximo de 3.6 V; muchos módulos comerciales incluyen regulador, desplazador de niveles y las resistencias pull-up en SDA/SCL. Revisa el esquema de tu módulo antes de conectarlo a 5 V.

---

## Configuración I2C
Configura tu periférico I2C en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | I2C | — |
| **I2C Speed Mode** | Standard Mode / Fast Mode | Soporta 100 kHz y 400 kHz |
| **I2C Clock Speed** | 100000 o 400000 Hz | Depende del requerimiento |

> [!NOTE]
> Las direcciones I2C del BME688 son **0x76** y **0x77** (7 bits), definidas como `BME688_ADDRESS_0` y `BME688_ADDRESS_1`. El driver las desplaza 1 bit a la izquierda internamente, como requiere la capa HAL de STM32.

---

## Configuración del Driver
Los parámetros de medición son macros privadas al inicio de `BME688.c`. Los valores por defecto siguen el ejemplo de *forced mode* de Bosch:

| Macro | Valor por defecto | Descripción |
|-------|-------------------|-------------|
| `BME688_OS_T` | `2` (x2) | Oversampling de temperatura |
| `BME688_OS_P` | `1` (x1) | Oversampling de presión |
| `BME688_OS_H` | `5` (x16) | Oversampling de humedad |
| `BME688_FILTER` | `0` | Filtro IIR desactivado |
| `BME688_HEATER_TEMP_C` | `300` | Temperatura objetivo del calentador [°C] (máx. 400) |
| `BME688_HEATER_DUR_MS` | `100` | Duración del calentamiento [ms] (máx. 4032) |
| `BME688_AMB_TEMP_C` | `25` | Temperatura ambiente supuesta para calcular la resistencia del calentador [°C] |

> [!TIP]
> El perfil del calentador depende de la aplicación (gas a detectar, consumo). La duración total de `BME688_Get()` se recalcula sola a partir de estas macros.

---

## Instalación
1. Copia `BME688.c` y `BME688.h` a tu proyecto (ej: `Inc/BME688.h`, `Src/BME688.c`).
2. Incluye la librería en tu código fuente:
   ```c
   #include "BME688.h"
   ```
3. Configura I2C y GPIOs en CubeMX (ver sección anterior).
4. Genera código y compila.

---

## Uso Básico

### 1. Inicialización
```c
// En main() después de HAL_Init() y MX_I2C1_Init()
BME688_Status_t status = BME688_Init(&hi2c1);

if (status != BME688_OK) {
    Error_Handler();  // Sensor no detectado en 0x76/0x77, Chip ID incorrecto o handle NULL
}
```

### 2. Obtener Datos

```c
// Declarar estructura de datos
BME688_Data_t bme_data;

// Medición en forced mode (bloquea ~145 ms con la configuración por defecto)
if (BME688_Get(&bme_data) == BME688_OK)
{
    printf("Temp: %.2f C, Presion: %.2f hPa, Humedad: %.2f %%RH\n",
           bme_data.temperatura, bme_data.presion, bme_data.humedad);

    if (bme_data.gas_valido) {
        printf("Gas: %.0f Ohm\n", bme_data.gas);
    }
}
```

> [!NOTE]
> Para usar `%f` en `printf` el proyecto debe enlazar con `-u _printf_float` (STM32CubeIDE: *Use float with printf from newlib-nano*).

### 3. Modo Sleep

```c
// Garantiza mínimo consumo (calentador apagado) o aborta una medición en curso.
// La configuración se conserva: la siguiente BME688_Get() funciona sin volver a llamar a Init.
BME688_Sleep();
```

### 4. Desinicialización

```c
// Deja el sensor en sleep, libera el handle de I2C y marca el módulo como no inicializado
BME688_DeInit();
```

---

## API Reference

### 1. Tipos de Datos

#### `BME688_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería.

```c
typedef enum {
    BME688_OK               = 0,    /**< Operación exitosa */
    BME688_ERROR            = 1,    /**< Error en la operación */
    BME688_TIMEOUT          = 2,    /**< Timeout en la operación */
    BME688_NOT_INITIALIZED  = 3,    /**< Módulo no inicializado */
    BME688_INVALID_PARAM    = 4     /**< Parámetro inválido */
} BME688_Status_t;
```

|Valor|Código|Significado|
|---|---|---|
|`BME688_OK`|0|Operación completada sin errores|
|`BME688_ERROR`|1|Error de I2C o Chip ID incorrecto|
|`BME688_TIMEOUT`|2|Timeout de HAL, sin datos nuevos o el sensor no volvió a sleep|
|`BME688_NOT_INITIALIZED`|3|`BME688_Init()` no se llamó o falló|
|`BME688_INVALID_PARAM`|4|Parámetro nulo (handle I2C o puntero de datos NULL)|

---

#### `BME688_Data_t` - Estructura de Datos

Estructura para almacenar los datos compensados de una medición.

```c
typedef struct {
    float   temperatura;    /**< Temperatura [°C] */
    float   presion;        /**< Presión [hPa] */
    float   humedad;        /**< Humedad relativa [%RH] */
    float   gas;            /**< Resistencia del sensor de gas [Ω] */
    uint8_t gas_valido;     /**< 1 si la medición de gas es válida y el calentador se estabilizó */
} BME688_Data_t;
```

| Campo | Tipo | Unidad | Descripción |
|-------|------|--------|-------------|
| `temperatura` | `float` | °C | Temperatura |
| `presion` | `float` | hPa | Presión barométrica |
| `humedad` | `float` | %RH | Humedad relativa, acotada a 0 – 100 |
| `gas` | `float` | Ω | Resistencia del sensor de gas |
| `gas_valido` | `uint8_t` | — | `1` si `gas` es fiable (bits `gas_valid` y `heat_stab`), `0` si no |

---

### 2. Funciones Públicas

#### `BME688_Init()` - Inicialización del Driver

Detecta el sensor, lo reinicia, lee la calibración de fábrica y lo configura para *forced mode*.

```c
BME688_Status_t BME688_Init(I2C_HandleTypeDef* hi2c);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`hi2c`|`I2C_HandleTypeDef*`|Puntero al handle de I2C|

**Retorna**: `BME688_OK` si la inicialización fue exitosa, `BME688_ERROR` si el Chip ID no coincide o hay error de comunicación, `BME688_TIMEOUT` si el bus o el sensor no responden a tiempo, `BME688_INVALID_PARAM` si `hi2c` es NULL.

**Secuencia interna:**

1. Lectura del Chip ID en `0x76` y, si falla, en `0x77`.
2. *Soft reset* (`0xB6` en `0xE0`), espera de 10 ms y nueva verificación del Chip ID.
3. Lectura del `variant_id` (BME680 o BME688).
4. Lectura de los coeficientes de calibración.
5. Paso a sleep y escritura de oversampling, filtro y perfil 0 del calentador.

---

#### `BME688_Get()` - Lectura de Datos

Dispara una medición en *forced mode* y retorna temperatura, presión, humedad y gas compensados. Al terminar, el sensor vuelve solo a sleep.

```c
BME688_Status_t BME688_Get(BME688_Data_t* data);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`data`|`BME688_Data_t*`|Puntero a la estructura donde se guardarán los datos|

**Retorna**: `BME688_OK` si la lectura fue exitosa, `BME688_ERROR` si hay fallo de comunicación I2C, `BME688_TIMEOUT` si no hubo datos nuevos o el sensor no volvió a sleep, `BME688_NOT_INITIALIZED` si el módulo no fue inicializado, `BME688_INVALID_PARAM` si `data` es NULL.

> [!NOTE]
> La función es bloqueante: espera la duración de la conversión TPH más el calentamiento (~145 ms con la configuración por defecto) y, si los datos aún no están listos, sondea `new_data` hasta 5 veces cada 10 ms.

---

#### `BME688_Sleep()` - Modo Sleep

Lleva el sensor a modo sleep (mínimo consumo, calentador apagado). Si hay una medición en curso, espera como máximo la duración de una medición completa.

```c
BME688_Status_t BME688_Sleep(void);
```

**Retorna**: `BME688_OK` si el sensor quedó en sleep (o ya lo estaba), `BME688_ERROR` si hay error de comunicación, `BME688_TIMEOUT` si el sensor no entró en sleep a tiempo, `BME688_NOT_INITIALIZED` si no se llamó a `BME688_Init()`.

---

#### `BME688_DeInit()` - Desinicialización del Driver

Deja el sensor en sleep, libera el handle de I2C y marca el módulo como no inicializado. El estado se libera siempre, aunque falle el paso a sleep. Para volver a usar el sensor se debe llamar a `BME688_Init()`.

```c
BME688_Status_t BME688_DeInit(void);
```

**Retorna**: `BME688_OK` si el sensor quedó en sleep y el driver se liberó, `BME688_ERROR` si hay error de comunicación al pasar a sleep, `BME688_TIMEOUT` si el sensor no entró en sleep a tiempo, `BME688_NOT_INITIALIZED` si no se llamó a `BME688_Init()`.

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.  
El formato está basado en [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

### [1.0.0] - 08-11-2026

Versión inicial del driver BME688 para STM32 con interfaz I2C.

#### Added
- **Inicialización**: `BME688_Init()` detecta el sensor en `0x76`/`0x77`, verifica el Chip ID, aplica *soft reset*, identifica la variante (BME680/BME688), lee la calibración de fábrica y configura *forced mode*.
- **Medición**: `BME688_Get()` dispara una conversión en *forced mode* y retorna temperatura (°C), presión (hPa), humedad (%RH, acotada a 0 – 100) y resistencia de gas (Ω) compensadas con los algoritmos float de Bosch.
- **Validez del gas**: campo `gas_valido` en `BME688_Data_t`, activo solo si la medición de gas es válida y el calentador se estabilizó.
- **Modo sleep**: `BME688_Sleep()` deja el sensor en mínimo consumo conservando la configuración.
- **Desinicialización**: `BME688_DeInit()` pasa el sensor a sleep y libera el handle de I2C.
- **Configuración por macros**: oversampling, filtro IIR y perfil del calentador ajustables en `BME688.c`; la duración de la medición se calcula a partir de ellos.
- Definición de tipos: `BME688_Status_t`, `BME688_Data_t`.
- Manejo de errores con esperas acotadas (I2C, timeout, parámetros inválidos, no inicializado).
