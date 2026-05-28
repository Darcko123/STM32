# Librería para el módulo BMP280 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/BMP280)
[![Protocol](https://img.shields.io/badge/Protocol-I2C-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/BMP280)

---

## Tabla de Contenidos
- [Librería para el módulo BMP280 en STM32](#librería-para-el-módulo-bmp280-en-stm32)
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
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`BMP280_Status_t` - Estados de Retorno](#bmp280_status_t---estados-de-retorno)
      - [`BMP280_Data_t` - Estructura de Datos](#bmp280_data_t---estructura-de-datos)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`BMP280_Init()` - Inicialización del Driver](#bmp280_init---inicialización-del-driver)
      - [`BMP280_Get()` - Lectura de Datos](#bmp280_get---lectura-de-datos)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.0.0\] - 28-05-2026](#100---28-05-2026)
      - [Added](#added)

---

## Descripción
Librería desarrollada en C para la interfaz con el módulo **Bosch BMP280** utilizando microcontroladores STM32. Proporciona funciones para leer temperatura, presión y estimar la altitud con compensación por coeficientes de calibración de fábrica. La librería está diseñada para ser fácil de usar, eficiente y compatible con la mayoría de las series STM32 (F1, F4, etc.) utilizando HAL.

Ideal para aplicaciones de monitoreo meteorológico, altimetría básica y drones.

---

## Características
- **Comunicación I2C**: Interfaz I2C estándar para lectura de registros.
- **Compensación Integrada**: Uso de coeficientes de calibración internos para obtener valores reales.
- **Estimación de Altitud**: Cálculo automático de altitud basado en presión al nivel del mar.
- **Configuración por Defecto Optimizada**: Inicialización con filtro IIR x16, oversampling de temperatura x2, oversampling de presión x16 y standby de 0.5ms — ajustada para uso general sin configuración adicional.
- **Manejo robusto de errores**: Códigos de retorno específicos para error de I2C, timeout o módulo no inicializado (`BMP280_Status_t`).
- **Portabilidad**: Compatible con múltiples familias STM32 mediante la capa HAL. Solo requiere cambiar el `#include` del encabezado HAL en `BMP280.h`.
- **Instancia única**: El driver gestiona un único sensor BMP280 por proyecto.

---

## Pinout y Conexiones
### Pines requeridos

| Pin BMP280 | Dirección | Descripción | Tipo GPIO | Observaciones |
|------------|-----------|-------------|-----------|---------------|
| **VCC**    | Alimentación | 1.9V a 3.6V | N/A | **Importante:** usar solo 3.3V, **no 5V** |
| **GND**    | Tierra    | — | N/A | — |
| **SCL**    | Input (open-drain) | Línea de reloj I2C | `I2C_SCL` | Requiere resistencia pull-up (típicamente 4.7kΩ) |
| **SDA**    | Input/Output (open-drain) | Línea de datos I2C | `I2C_SDA` | Requiere resistencia pull-up (típicamente 4.7kΩ) |

> [!NOTE]
> La mayoría de los módulos comerciales ya integran regulador de voltaje LDO y conversores lógicos (level shifters), pero los módulos bare-bone (solo PCB morada/verde sencilla) operan estrictamente a 3.3V.

> [!WARNING]
> No conectar a 5V a menos que el módulo tenga regulador integrado. Puede dañar el sensor.

---

## Configuración I2C
Configura tu periférico I2C en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | I2C | — |
| **I2C Speed Mode** | Standard Mode / Fast Mode | Soporta 100 kHz y 400 kHz |
| **I2C Clock Speed** | 100000 o 400000 Hz | Depende del requerimiento |

> [!NOTE]
> La dirección I2C del BMP280 es: **0x76** (si SDO está a GND) o **0x77** (si SDO está a VCC). La librería intenta detectar automáticamente ambas.

---

## Instalación
1. Copia `BMP280.c` y `BMP280.h` a tu proyecto (ej: `Librerias/BMP280/`).
2. Incluye la librería en tu `main.c` o archivo principal:
   ```c
   #include "BMP280.h"
   ```
3. Configura I2C y GPIOs en CubeMX (ver sección anterior).
4. Genera código y compila.

---

## Uso Básico

### 1. Inicialización
```c
// En main() después de HAL_Init() y MX_I2C1_Init()
BMP280_Status_t status = BMP280_Init(&hi2c1);

if (status != BMP280_OK) {
    Error_Handler();  // Módulo no responde o error de inicialización
}
```

### 2. Obtener Datos

```c
// Declarar estructura de datos
BMP280_Data_t bmp_data;

// Leer los datos del sensor
if (BMP280_Get(&bmp_data) == BMP280_OK) {
    // Uso de los datos: bmp_data.temperatura, bmp_data.presion, bmp_data.altitud
    printf("Temp: %.2f C, Pres: %.2f hPa, Alt: %.2f m\n", 
           bmp_data.temperatura, bmp_data.presion, bmp_data.altitud);
}
```

---

## API Reference

### 1. Tipos de Datos

#### `BMP280_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería, permitiendo una gestión robusta de errores y estados del chip.

```c
typedef enum {
    BMP280_OK               = 0,    /**< Operación exitosa */
    BMP280_ERROR            = 1,    /**< Error en la operación */
    BMP280_TIMEOUT          = 2,    /**< Timeout en la operación */
    BMP280_NOT_INITIALIZED  = 3,    /**< Módulo no inicializado */
    BMP280_INVALID_PARAM    = 4     /**< Parámetro inválido */
} BMP280_Status_t;
```

|Valor|Código|Significado|
|---|---|---|
|`BMP280_OK`|0|Operación completada sin errores|
|`BMP280_ERROR`|1|Error de I2C, error de ID del chip o comunicación fallida|
|`BMP280_TIMEOUT`|2|Timeout de software o timeout interno del chip|
|`BMP280_NOT_INITIALIZED`|3|`BMP280_Init()` no se llamó o falló|
|`BMP280_INVALID_PARAM`|4|Parámetro nulo o configuración inválida|

---

#### `BMP280_Data_t` - Estructura de Datos

Estructura para almacenar los datos de temperatura, presión y altitud.

```c
typedef struct {
    float temperatura;  /**< Temperatura [°C] */
    float presion;      /**< Presión [hPa] */
    float altitud;      /**< Altitud estimada [m] */
} BMP280_Data_t;
```

| Campo | Tipo | Rango | Unidad | Descripción |
|-------|------|-------|--------|-------------|
| `temperatura` | `float` | -40 - +85 | °C | Temperatura |
| `presion` | `float` | 300 - 1100 | hPa | Presión atmosférica |
| `altitud` | `float` | -500 - +9000 | m | Altitud calculada en base a la presión |

---

### 2. Funciones Públicas

#### `BMP280_Init()` - Inicialización del Driver

Inicializa el driver BMP280 con configuración por defecto. Intenta detectar el sensor en las direcciones 0x76 y 0x77. Realiza un soft reset y aplica una configuración óptima para uso general.

```c
BMP280_Status_t BMP280_Init(I2C_HandleTypeDef* hi2c);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`hi2c`|`I2C_HandleTypeDef*`|Puntero al handle de I2C|

**Retorna**: `BMP280_OK` si la inicialización fue exitosa, `BMP280_ERROR` si el Chip ID no coincide o hay error de comunicación, `BMP280_INVALID_PARAM` si el handle es nulo.

**Secuencia interna:**

1. Intento de detección de dirección (0x76 o 0x77).
2. Verificación del Chip ID.
3. Soft reset y lectura de coeficientes de calibración.
4. Escritura de registros de configuración (oversampling, filtro, standby).

---

#### `BMP280_Get()` - Lectura de Datos

Lee temperatura, presión y altitud compensadas del BMP280.

```c
BMP280_Status_t BMP280_Get(BMP280_Data_t* data);
```

|Parámetro|Tipo|Descripción|Rango|
|---|---|---|---|
|`data`|`BMP280_Data_t*`|Puntero a la estructura donde se guardarán los datos|N/A|

**Retorna**: `BMP280_OK` si la operación fue exitosa, `BMP280_ERROR` si hay fallo de comunicación, `BMP280_INVALID_PARAM` si el puntero de datos es nulo.

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](https://opensource.org/licenses/MIT) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.  
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [1.0.0] - 28-05-2026

#### Added
- Implementación completa del driver BMP280 para STM32 con interfaz I2C.
- Soporte para detección automática de dirección I2C (`0x76` o `0x77`).
- Lectura y compensación de coeficientes de calibración de fábrica.
- Algoritmos de compensación de temperatura y presión según el datasheet de Bosch.
- Cálculo automático de altitud estimada usando presión al nivel del mar.
- Configuración por defecto: oversampling x2 (temp), x16 (presión), filtro IIR x16, standby 0.5ms.
- Funciones públicas `BMP280_Init()` e `BMP280_Get()`.
- Definición de tipos: `BMP280_Status_t`, `BMP280_Data_t`.
- Enumeraciones para filtro, oversampling y tiempo de standby.
- Manejo robusto de errores (timeout, I2C, parámetros inválidos, no inicializado).

---