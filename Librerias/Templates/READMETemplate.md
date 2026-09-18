# Librería para el módulo [[NOMBRE_MODULO]] en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-X.X.X-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/[[CARPETA_PROYECTO]])
[![Protocol](https://img.shields.io/badge/Protocol-[[PROTOCOLO]]-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/[[CARPETA_PROYECTO]])

---

## Tabla de Contenidos
- [Librería para el módulo \[\[NOMBRE\_MODULO\]\] en STM32](#librería-para-el-módulo-nombre_modulo-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
  - [Configuración \[\[PROTOCOLO\]\]](#configuración-protocolo)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. \[\[PRIMERA\_FUNCION\_PRINCIPAL\]\]](#2-primera_funcion_principal)
    - [3. \[\[SEGUNDA\_FUNCION\_PRINCIPAL\]\]](#3-segunda_funcion_principal)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`[[PREFIX]]_Status_t` - Estados de Retorno](#prefix_status_t--estados-de-retorno)
      - [`[[estructura_config_t]]` - \[\[DESCRIPCION\_ESTRUCTURA\_CONFIG\]\]](#estructura_config_t--descripcion_estructura_config)
      - [`[[estructura_datos_t]]` - \[\[DESCRIPCION\_ESTRUCTURA\_DATOS\]\]](#estructura_datos_t--descripcion_estructura_datos)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`[[PREFIX]]_Init()` - Inicialización del Driver](#prefix_init--inicialización-del-driver)
      - [`[[PREFIX]]_[[Funcion1]]()` - \[\[DescripcionFuncion1\]\]](#prefix_funcion1--descripcionfuncion1)
      - [`[[PREFIX]]_[[Funcion2]]()` - \[\[DescripcionFuncion2\]\]](#prefix_funcion2--descripcionfuncion2)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[X.X.X\] - DD-MM-AAAA](#xxx---dd-mm-aaaa)
      - [Added](#added)
      - [Changed](#changed)
      - [Fixed](#fixed)

---

## Descripción
Librería desarrollada en C para la interfaz con el módulo **[[FABRICANTE]] [[NOMBRE_MODULO]]** utilizando microcontroladores STM32. Proporciona funciones para [[descripción general de lo que hace la librería]]. La librería está diseñada para ser fácil de usar, eficiente y compatible con la mayoría de las series STM32 (F1, F4, etc.) utilizando HAL.

[[Información adicional sobre aplicaciones típicas del módulo]].

---

## Características
- **Comunicación [[PROTOCOLO]]**: [[Descripción de la abstracción del protocolo]].
- **[[Característica 1]]**: [[Descripción detallada]].
- **[[Característica 2]]**: [[Descripción detallada]].
- **[[Característica 3]]**: [[Descripción detallada]].
- **Manejo robusto de errores**: Códigos de retorno específicos para error de [[PROTOCOLO]], timeout o módulo no inicializado (`[[PREFIX]]_Status_t`).
- **[[Otra característica importante, ej: conversión automática, LDRO, etc.]]**: [[Descripción]].
- **Portabilidad**: Compatible con múltiples familias STM32 mediante la capa HAL. Solo requiere cambiar el `#include` del encabezado HAL en `[[PREFIX]].h`.

---

## Pinout y Conexiones
### Pines requeridos

| Pin [[NOMBRE_MODULO]] | Dirección | Descripción | Tipo GPIO | Observaciones |
|-----------------------|-----------|-------------|-----------|---------------|
| **VCC**               | Alimentación | [[Voltaje de alimentación]] | N/A | [[Notas sobre voltaje, ej: "usar solo 3.3V"]] |
| **GND**               | Tierra    | — | N/A | — |
| **[[PIN1]]**          | [[Input/Output/Open-drain]] | [[Descripción del pin]] | `[[TIPO_GPIO]]` | [[Observaciones, ej: "requiere pull-up"]] |
| **[[PIN2]]**          | [[Input/Output/Open-drain]] | [[Descripción del pin]] | `[[TIPO_GPIO]]` | [[Observaciones]] |
| **[[PIN_OPCIONAL]]**  | [[Input/Output]] (opcional) | [[Descripción del pin opcional]] | `[[TIPO_GPIO]]` | [[Observaciones]] |

> [!NOTE]
> [[Notas generales sobre pull-ups, módulos comerciales, etc.]]

> [!WARNING]
> [[Advertencias importantes, como niveles de voltaje, pines obligatorios, etc.]]

---

## Configuración [[PROTOCOLO]]
Configura tu periférico [[PROTOCOLO]] en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | [[Modo, ej: I2C/SPI]] | [[Descripción]] |
| **[[Parámetro1]]** | [[Valor]] | [[Nota]] |
| **[[Parámetro2]]** | [[Valor]] | [[Nota]] |
| **[[Parámetro3]]** | [[Valor]] | [[Nota]] |

> [!NOTE]
> La dirección [[PROTOCOLO]] del [[NOMBRE_MODULO]] es: **[[DIRECCION]]** ([[formato, ej: 7 bits]]). La librería ya la tiene definida como `[[PREFIX]]_ADDRESS [[VALOR]]`.

---

## Instalación
1. Copia `[[PREFIX]].c` y `[[PREFIX]].h` a tu proyecto (ej: `Inc/[[PREFIX]].h` y `Src/[[PREFIX.c]]`).

2. Incluye la librería en tu `main.c` o archivo principal:
```c
#include "[[PREFIX]].h"
```
1. Configura [[PROTOCOLO]] y GPIOs en CubeMX (ver sección anterior).
2. Genera código y compila.

---

## Uso Básico

### 1. Inicialización
```c
// En main() después de HAL_Init() y MX_[[PROTOCOLO]]x_Init()
[[PREFIX]]_Status_t status = [[PREFIX]]_Init(&h[[PROTOCOLO]]1 /* , otros parámetros si los hay */);

if (status != [[PREFIX]]_OK) {
    Error_Handler();  // Módulo no responde o handle NULL
}
```

### 2. [[PRIMERA_FUNCION_PRINCIPAL]]

```c
// [[Descripción de lo que hace el ejemplo]]
[[código de ejemplo]]
```

### 3. [[SEGUNDA_FUNCION_PRINCIPAL]]

```c
// [[Descripción de lo que hace el ejemplo]]
[[código de ejemplo]]
```
> [!NOTE]  
> [[Notas importantes sobre el uso, tiempos de espera, configuración previa, etc.]]

---

## API Reference

### 1. Tipos de Datos

#### `[[PREFIX]]_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería, permitiendo una gestión robusta de errores y estados del chip.

```c
typedef enum {
    [[PREFIX]]_OK = 0,              /**< Operación exitosa */
    [[PREFIX]]_ERROR = 1,           /**< Error en la operación */
    [[PREFIX]]_TIMEOUT = 2,         /**< Timeout en la operación */
    [[PREFIX]]_NOT_INITIALIZED = 3, /**< Módulo no inicializado */
    [[PREFIX]]_BUSY = 4             /**< Recurso ocupado (opcional) */
} [[PREFIX]]_Status_t;
```

|Valor|Código|Significado|
|---|---|---|
|`[[PREFIX]]_OK`|0|Operación completada sin errores|
|`[[PREFIX]]_ERROR`|1|Error de [[PROTOCOLO]], parámetro inválido o condición inesperada|
|`[[PREFIX]]_TIMEOUT`|2|Timeout de software o timeout interno del chip|
|`[[PREFIX]]_NOT_INITIALIZED`|3|`[[PREFIX]]_Init()` no se llamó o falló|

---

#### `[[estructura_config_t]]` - [[DESCRIPCION_ESTRUCTURA_CONFIG]]
[[Descripción de la estructura de configuración]].

```c
typedef struct {
    [[tipo]] [[campo1]];  /**< [[Descripción del campo1]] */
    [[tipo]] [[campo2]];  /**< [[Descripción del campo2]] */
    [[tipo]] [[campo3]];  /**< [[Descripción del campo3]] */
} [[estructura_config_t]];
```

|Campo|Tipo|Rango|Descripción|
|---|---|---|---|
|`[[campo1]]`|`[[tipo]]`|[[rango]]|[[Descripción]]|
|`[[campo2]]`|`[[tipo]]`|[[rango]]|[[Descripción]]|

---

#### `[[estructura_datos_t]]` - [[DESCRIPCION_ESTRUCTURA_DATOS]]

[[Descripción de la estructura de datos]].

```c
typedef struct {
    [[tipo]] [[campo1]];  /**< [[Descripción del campo1]] */
    [[tipo]] [[campo2]];  /**< [[Descripción del campo2]] */
} [[estructura_datos_t]];
```
|Campo|Tipo|Rango|Unidad|Descripción|
|---|---|---|---|---|
|`[[campo1]]`|`[[tipo]]`|[[rango]]|[[unidad]]|[[Descripción]]|
|`[[campo2]]`|`[[tipo]]`|[[rango]]|[[unidad]]|[[Descripción]]|

---

### 2. Funciones Públicas

#### `[[PREFIX]]_Init()` - Inicialización del Driver

[[Descripción de la función de inicialización]].

```c
[[PREFIX]]_Status_t [[PREFIX]]_Init([[parametros]]);
```
|Parámetro|Tipo|Descripción|
|---|---|---|
|`[[param1]]`|`[[tipo]]`|[[Descripción]]|
|`[[param2]]`|`[[tipo]]`|[[Descripción]]|

**Retorna**: `[[PREFIX]]_OK` si la inicialización fue exitosa, `[[PREFIX]]_ERROR` si [[condiciones de error]].

**Secuencia interna:**

1. [[Paso 1]]
    
2. [[Paso 2]]
    
3. [[Paso 3]]
    

---

#### `[[PREFIX]]_[[Funcion1]]()` - [[DescripcionFuncion1]]

[[Descripción de la función]].
```c
[[PREFIX]]_Status_t [[PREFIX]]_[[Funcion1]]([[parametros]]);
```
 
 |Parámetro|Tipo|Descripción|Rango|
|---|---|---|---|
|`[[param1]]`|`[[tipo]]`|[[Descripción]]|[[rango]]|
|`[[param2]]`|`[[tipo]]`|[[Descripción]]|[[rango]]|

**Retorna**: `[[PREFIX]]_OK` si la operación fue exitosa, `[[PREFIX]]_ERROR` si [[condiciones de error]], `[[PREFIX]]_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `[[PREFIX]]_[[Funcion2]]()` - [[DescripcionFuncion2]]

[[Descripción de la función]].

```c
[[PREFIX]]_Status_t [[PREFIX]]_[[Funcion2]]([[parametros]]);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`[[param1]]`|`[[tipo]]*`|[[Descripción]]|

**Retorna**: `[[PREFIX]]_OK` si la lectura fue exitosa, `[[PREFIX]]_ERROR` si falla la comunicación [[PROTOCOLO]] o `[[param1]]` es NULL.

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](https://license.md/) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.  
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [X.X.X] - DD-MM-AAAA

#### Added

- [[Nueva característica 1]].
    
- [[Nueva característica 2]].
    

#### Changed

- [[Cambio 1]].
    
- [[Cambio 2]].
    

#### Fixed

- [[Corrección 1]].
    
- [[Corrección 2]].