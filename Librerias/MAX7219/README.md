# Librería para el módulo MAX7219 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)
[![Version](https://img.shields.io/badge/Version-2.1.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/MAX7219)
[![Protocol](https://img.shields.io/badge/Protocol-SPI-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/MAX7219)

---

## Tabla de Contenidos
- [Librería para el módulo MAX7219 en STM32](#librería-para-el-módulo-max7219-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
  - [Configuración SPI](#configuración-spi)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Mostrar texto estático](#2-mostrar-texto-estático)
    - [3. Desplazar texto (scroll)](#3-desplazar-texto-scroll)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`MAX7219_Status_t` - Estados de Retorno](#max7219_status_t---estados-de-retorno)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`MAX7219_Init()` - Inicialización del Driver](#max7219_init---inicialización-del-driver)
      - [`MAX7219_DeInit()` - Desinicialización del Driver](#max7219_deinit---desinicialización-del-driver)
      - [`MAX7219_Set_Sleep()` - Modo de Bajo Consumo](#max7219_set_sleep---modo-de-bajo-consumo)
      - [`MAX7219_ClearDisplay()` - Limpiar Pantalla](#max7219_cleardisplay---limpiar-pantalla)
      - [`MAX7219_PrintString()` - Mostrar Texto Estático](#max7219_printstring---mostrar-texto-estático)
      - [`MAX7219_ScrollString()` - Desplazar Texto](#max7219_scrollstring---desplazar-texto)
      - [`MAX7219_SetPixel()` - Control de Píxeles Individuales](#max7219_setpixel---control-de-píxeles-individuales)
  - [Notas](#notas)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[2.1.0\] - 18-06-2026](#210---18-06-2026)
      - [Added](#added)
      - [Changed](#changed)
    - [\[2.0.0\] - 03-02-2026](#200---03-02-2026)
      - [Added](#added-1)
      - [Changed](#changed-1)
      - [Removed](#removed)
    - [\[1.1.0\] - 10-10-2025](#110---10-10-2025)
      - [Changed](#changed-2)
    - [\[1.0.0\] - 06-02-2025](#100---06-02-2025)
      - [Added](#added-2)

---

## Descripción
Librería desarrollada en C para el control de una matriz de LEDs basada en el integrado **MAX7219** mediante comunicación **SPI** en microcontroladores STM32. Permite mostrar caracteres estáticos, desplazarlos (scroll) y manipular píxeles individuales de la matriz de manera eficiente, gestionando un buffer interno de columnas.

Esta librería está basada en el código de **Controllerstech**: [Cómo desplazar cadenas en una pantalla de matriz de puntos](https://controllerstech.com/how-to-scroll-string-on-dot-matrix-display/#info_box).

---

## Características
- **Comunicación SPI**: Envío de comandos y datos al MAX7219 mediante el periférico SPI en modo Half-Duplex Master.
- **Múltiples dispositivos en cascada**: Soporta hasta `MAX7219_MAX_DEV` módulos MAX7219 encadenados, configurables en tiempo de ejecución.
- **Texto estático y desplazamiento**: Funciones para mostrar texto fijo (`MAX7219_PrintString`) o desplazarlo por la matriz (`MAX7219_ScrollString`).
- **Control de píxeles individuales**: `MAX7219_SetPixel()` permite dibujar gráficos/iconos arbitrarios además de texto.
- **Manejo robusto de errores**: Códigos de retorno específicos para error de SPI, timeout, módulo no inicializado o parámetro inválido (`MAX7219_Status_t`).
- **Modo de bajo consumo**: `MAX7219_Set_Sleep()` permite activar el modo shutdown sin perder la configuración almacenada.
- **Fuente de caracteres 8x8**: Conjunto completo de fuentes para los 256 valores de un byte.
- **Portabilidad**: Compatible con múltiples familias STM32 mediante la capa HAL, ya que el header incluye `main.h` en lugar de un header de familia específica.

---

## Pinout y Conexiones
### Pines requeridos

| Pin MAX7219 | Dirección | Descripción | Tipo GPIO | Observaciones |
|-------------|-----------|-------------|-----------|---------------|
| **VCC**     | Alimentación | Alimentación del módulo | N/A | 5V o 3.3V según el módulo |
| **GND**     | Tierra    | — | N/A | — |
| **DIN**     | Input     | Datos seriales hacia el MAX7219 | SPI MOSI | — |
| **CS**      | Input     | Selección de chip (Chip Select) | GPIO Output Push-Pull | Controlado por software en cada transacción |
| **CLK**     | Input     | Reloj serial SPI | SPI SCK | — |

> [!NOTE]
> En módulos con varios MAX7219 en cascada, solo se conecta un único pin CS a todos los dispositivos; la selección de a cuál se le envía el dato real se gestiona internamente en la librería.

---

## Configuración SPI
Configura tu periférico SPI en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | Half-Duplex Master | — |
| **Frame Format** | Motorola | — |
| **Data Size** | 16 Bits | Cada transacción envía dirección/fila + dato en una sola palabra |
| **First Bit** | MSB First | — |
| **Prescaler** | 32 | Ajustable según la velocidad del reloj del sistema |
| **Clock Polarity** | High | — |
| **Clock Phase** | 1 Edge | — |

El pin CS se configura como GPIO de salida independiente:

| Parámetro | Valor |
|-----------|-------|
| **GPIO Output Level** | HIGH |
| **GPIO Mode** | Output Push-Pull |
| **Maximum Output Speed** | Low |

---

## Instalación
1. Copia `MAX7219.c` y `MAX7219.h` a tu proyecto (ej: `Inc/MAX7219.h`, `Src/MAX7219.c`).
2. Incluye la librería en tu `main.c` o archivo principal:
```c
#include "MAX7219.h"
```
3. Configura el periférico SPI y el GPIO de CS en CubeMX (ver secciones anteriores).
4. Genera código y compila.

---

## Uso Básico

### 1. Inicialización
```c
// En main() después de HAL_Init() y MX_SPI1_Init()
if(MAX7219_Init(&hspi1, GPIOA, GPIO_PIN_6, 4) != MAX7219_OK)
{
    sprintf(buffer, "%sError inicializando MAX7219%s\r\n", COLOR_RED, COLOR_RESET);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
    while(1);
}
```

### 2. Mostrar texto estático
```c
if(MAX7219_ClearDisplay() != MAX7219_OK)
{
    sprintf(buffer, "%sError limpiando MAX7219%s\r\n", COLOR_RED, COLOR_RESET);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

if(MAX7219_PrintString("HOLA") != MAX7219_OK)
{
    sprintf(buffer, "%sError mandando cadena%s\r\n", COLOR_RED, COLOR_RESET);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}
```

### 3. Desplazar texto (scroll)
```c
while (1)
{
    if(MAX7219_ScrollString("HOLA MUNDO", 100) != MAX7219_OK)
    {
        sprintf(buffer, "%sError mandando cadena%s\r\n", COLOR_RED, COLOR_RESET);
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
    }
    HAL_Delay(200);

    MAX7219_ClearDisplay();
}
```

> [!NOTE]
> El número de dispositivos en cascada se fija en `MAX7219_Init()` mediante `numDevices` y determina el rango válido de columnas (`x`) en `MAX7219_SetPixel()`.

---

## API Reference

### 1. Tipos de Datos

#### `MAX7219_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería, permitiendo una gestión robusta de errores y estados del chip.

```c
typedef enum {
    MAX7219_OK = 0,                /**< Operación exitosa */
    MAX7219_ERROR = 1,             /**< Error en la operación */
    MAX7219_TIMEOUT = 2,           /**< Timeout en la operación */
    MAX7219_NOT_INITIALIZED = 3,   /**< Módulo no inicializado */
    MAX7219_INVALID_PARAM = 4      /**< Parámetro inválido */
} MAX7219_Status_t;
```

|Valor|Código|Significado|
|---|---|---|
|`MAX7219_OK`|0|Operación completada sin errores|
|`MAX7219_ERROR`|1|Error de SPI u otra condición inesperada|
|`MAX7219_TIMEOUT`|2|Timeout de la transmisión SPI|
|`MAX7219_NOT_INITIALIZED`|3|`MAX7219_Init()` no se llamó o falló|
|`MAX7219_INVALID_PARAM`|4|Puntero `NULL` o parámetro fuera de rango|

---

### 2. Funciones Públicas

#### `MAX7219_Init()` - Inicialización del Driver

Inicializa el módulo MAX7219 con los parámetros de SPI y GPIO para CS, y configura el número de dispositivos en cascada.

```c
MAX7219_Status_t MAX7219_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN, uint8_t numDevices);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`hspi`|`SPI_HandleTypeDef*`|Puntero al manejador del periférico SPI|
|`GPIOx`|`GPIO_TypeDef*`|Puerto GPIO donde está conectado el pin CS|
|`GPIO_PIN`|`uint16_t`|Pin GPIO donde está conectado el pin CS|
|`numDevices`|`uint8_t`|Número de dispositivos MAX7219 en cascada (1 a `MAX7219_MAX_DEV`)|

**Retorna**: `MAX7219_OK` si la inicialización fue exitosa, `MAX7219_INVALID_PARAM` si algún puntero es `NULL` o `numDevices` está fuera de rango, `MAX7219_ERROR`/`MAX7219_TIMEOUT` si falla la comunicación SPI durante la configuración.

**Secuencia interna:**

1. Validar parámetros y almacenar configuración (handle SPI, puerto/pin CS, número de dispositivos).
2. Configurar el modo de decodificación, intensidad de brillo, escaneo de columnas y modo de operación normal.
3. Deshabilitar el test de display y limpiar la pantalla (`MAX7219_ClearDisplay()`).

---

#### `MAX7219_DeInit()` - Desinicialización del Driver

Apaga la pantalla (modo shutdown) y libera la configuración almacenada (handle SPI, puerto/pin CS y número de dispositivos).

```c
MAX7219_Status_t MAX7219_DeInit(void);
```

**Retorna**: `MAX7219_OK` si la operación fue exitosa, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `MAX7219_Set_Sleep()` - Modo de Bajo Consumo

Activa o desactiva el modo de bajo consumo (shutdown) sin perder la configuración almacenada. A diferencia de `MAX7219_DeInit()`, el módulo puede reactivarse sin volver a llamar a `MAX7219_Init()`.

```c
MAX7219_Status_t MAX7219_Set_Sleep(uint8_t enable);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`enable`|`uint8_t`|Distinto de 0 para entrar en modo de bajo consumo, 0 para volver al modo normal|

**Retorna**: `MAX7219_OK` si la operación fue exitosa, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `MAX7219_ClearDisplay()` - Limpiar Pantalla

Apaga todos los LEDs de la matriz, limpiando el buffer de columnas y actualizando la pantalla.

```c
MAX7219_Status_t MAX7219_ClearDisplay(void);
```

**Retorna**: `MAX7219_OK` si la operación fue exitosa, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `MAX7219_PrintString()` - Mostrar Texto Estático

Convierte una cadena de texto en su representación de bytes y la muestra de forma estática en la matriz de LEDs.

```c
MAX7219_Status_t MAX7219_PrintString(const char *str);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`str`|`const char*`|Cadena de texto a mostrar|

**Retorna**: `MAX7219_OK` si la operación fue exitosa, `MAX7219_INVALID_PARAM` si `str` es `NULL`, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `MAX7219_ScrollString()` - Desplazar Texto

Desplaza una cadena de texto a través de la matriz de LEDs, carácter por carácter, con un retardo configurable entre cada paso.

```c
MAX7219_Status_t MAX7219_ScrollString(char *str, int delay);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`str`|`char*`|Cadena de texto a desplazar|
|`delay`|`int`|Retardo entre cada paso de desplazamiento, en milisegundos|

**Retorna**: `MAX7219_OK` si la operación fue exitosa, `MAX7219_INVALID_PARAM` si `str` es `NULL`, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `MAX7219_SetPixel()` - Control de Píxeles Individuales

Enciende o apaga un píxel individual de la matriz de LEDs, manipulando directamente el buffer de columnas. Permite dibujar gráficos/iconos arbitrarios en lugar de limitarse a texto. La actualización de la pantalla es inmediata.

```c
MAX7219_Status_t MAX7219_SetPixel(int x, int y, uint8_t state);
```

|Parámetro|Tipo|Descripción|Rango|
|---|---|---|---|
|`x`|`int`|Columna del píxel|0 a `numDevices * 8 - 1`|
|`y`|`int`|Fila del píxel dentro de la columna|0 a 7|
|`state`|`uint8_t`|Distinto de 0 para encender el píxel, 0 para apagarlo|—|

**Retorna**: `MAX7219_OK` si la operación fue exitosa, `MAX7219_INVALID_PARAM` si `x` o `y` están fuera de rango, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado.

---

## Notas
- El número de dispositivos MAX7219 en cascada se indica mediante el parámetro `numDevices` de `MAX7219_Init()`. El máximo soportado se define con `MAX7219_MAX_DEV` en **MAX7219.h**, que determina el tamaño del buffer estático.

---

## Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

### [2.1.0] - 18-06-2026

#### Added
- Función `MAX7219_DeInit()` para desinicializar el módulo y liberar la configuración almacenada.
- Función `MAX7219_Set_Sleep()` para activar/desactivar el modo de bajo consumo (shutdown) sin perder la configuración.
- Función `MAX7219_SetPixel()` para encender/apagar píxeles individuales de la matriz de LEDs.
- Nuevo valor `MAX7219_INVALID_PARAM` en `MAX7219_Status_t` para distinguir errores de parámetros inválidos de errores generales.

#### Changed
- `MAX7219_Init()` ahora recibe el parámetro `numDevices` para configurar el número de dispositivos en cascada en tiempo de ejecución, en lugar de la macro fija `NUM_DEV`.
- Renombrada la macro `NUM_DEV` a `MAX7219_MAX_DEV`, que ahora representa el límite máximo de dispositivos soportado (tamaño del buffer estático) en lugar del número fijo configurado.
- Las funciones `MAX7219_ScrollString()` y `MAX7219_PrintString()` ahora retornan `MAX7219_INVALID_PARAM` (en lugar de `MAX7219_ERROR`) cuando se les pasa un puntero `NULL`.
- El header ahora incluye `main.h` en lugar de `stm32f4xx_hal.h`, para no atar la librería a una familia STM32 específica.

---

### [2.0.0] - 03-02-2026

#### Added
- Sistema de gestión de errores con tipo `MAX7219_Status_t` (OK, ERROR, TIMEOUT, NOT_INITIALIZED).
- Validación de punteros NULL en todas las funciones públicas.
- Bandera interna de verificación de inicialización del módulo.
- Documentación completa con comentarios Doxygen en todas las funciones.

#### Changed
- `MAX7219_Init()` ahora recibe parámetros de GPIO para el pin CS.
- Nombres de funciones actualizados para seguir convención de nomenclatura consistente.

#### Removed
- Funciones `max7219_cmd`, `max7219_write`, `flushBuffer`, `ShiftLeft`, `ShiftRight` y
  `shiftchar` movidas a privadas (ya no forman parte de la API pública).

---

### [1.1.0] - 10-10-2025

#### Changed
- `MAX7219_Init()` parametrizado para recibir el puerto SPI como argumento.

---

### [1.0.0] - 06-02-2025

#### Added
- Control básico de matriz de LEDs mediante MAX7219 vía SPI.
- Soporte para múltiples dispositivos en cascada (`NUM_DEV`).
- Funciones `MAX7219_ScrollString()` y `MAX7219_PrintString()`.
- Conjunto completo de fuentes de caracteres 8×8.
- Gestión de buffer de columnas para actualizaciones eficientes.
