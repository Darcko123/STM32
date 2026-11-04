# Librería para el módulo MAX7219 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
[![Version](https://img.shields.io/badge/Version-3.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/MAX7219)
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
    - [4. Dibujar en el buffer](#4-dibujar-en-el-buffer)
    - [5. Texto con cursor (centrado)](#5-texto-con-cursor-centrado)
  - [Sistema de coordenadas](#sistema-de-coordenadas)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos y Constantes](#1-tipos-de-datos-y-constantes)
      - [`MAX7219_Status_t` - Estados de Retorno](#max7219_status_t---estados-de-retorno)
      - [`MAX7219_COLOR_t` - Estado de un LED](#max7219_color_t---estado-de-un-led)
      - [Constantes configurables](#constantes-configurables)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`MAX7219_Init()` - Inicialización del Driver](#max7219_init---inicialización-del-driver)
      - [`MAX7219_DeInit()` - Desinicialización del Driver](#max7219_deinit---desinicialización-del-driver)
      - [`MAX7219_SetSleep()` - Modo de Bajo Consumo](#max7219_setsleep---modo-de-bajo-consumo)
      - [`MAX7219_SetIntensity()` - Brillo](#max7219_setintensity---brillo)
      - [`MAX7219_Clear()` - Limpiar Pantalla](#max7219_clear---limpiar-pantalla)
      - [`MAX7219_PrintString()` - Mostrar Texto Estático](#max7219_printstring---mostrar-texto-estático)
      - [`MAX7219_ScrollString()` - Desplazar Texto](#max7219_scrollstring---desplazar-texto)
      - [`MAX7219_SetPixel()` - Píxel con Actualización Inmediata](#max7219_setpixel---píxel-con-actualización-inmediata)
      - [`MAX7219_GetWidth()` / `MAX7219_GetHeight()` - Dimensiones](#max7219_getwidth--max7219_getheight---dimensiones)
      - [`MAX7219_UpdateScreen()` - Volcar el Buffer](#max7219_updatescreen---volcar-el-buffer)
      - [`MAX7219_Fill()` - Rellenar el Buffer](#max7219_fill---rellenar-el-buffer)
      - [`MAX7219_ToggleInvert()` - Inversión por Software](#max7219_toggleinvert---inversión-por-software)
      - [Primitivas de dibujo](#primitivas-de-dibujo)
      - [Texto con cursor](#texto-con-cursor)
  - [Notas](#notas)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[3.0.0\] - 03-11-2026](#300---03-11-2026)
      - [Added](#added)
      - [Changed](#changed)
      - [Fixed](#fixed)
      - [Removed](#removed)
    - [\[2.1.0\] - 18-06-2026](#210---18-06-2026)
      - [Added](#added-1)
      - [Changed](#changed-1)
    - [\[2.0.0\] - 03-02-2026](#200---03-02-2026)
      - [Added](#added-2)
      - [Changed](#changed-2)
      - [Removed](#removed-1)
    - [\[1.1.0\] - 10-10-2025](#110---10-10-2025)
      - [Changed](#changed-3)
    - [\[1.0.0\] - 06-02-2025](#100---06-02-2025)
      - [Added](#added-3)

---

## Descripción
Librería desarrollada en C para el control de una matriz de LEDs basada en el integrado **MAX7219** mediante comunicación **SPI** en microcontroladores STM32. Permite mostrar texto estático o con desplazamiento (scroll) y dibujar gráficos (píxeles, líneas, rectángulos, bitmaps y texto posicionado) sobre un buffer interno que se vuelca a la pantalla con `MAX7219_UpdateScreen()`, con la misma filosofía que las librerías de pantallas SSD1306/SSD1681 del repositorio.

Esta librería está basada en el código de **Controllerstech**: [Cómo desplazar cadenas en una pantalla de matriz de puntos](https://controllerstech.com/how-to-scroll-string-on-dot-matrix-display/#info_box).

---

## Características
- **Comunicación SPI**: Envío de comandos y datos al MAX7219 mediante el periférico SPI en modo Half-Duplex Master.
- **Múltiples dispositivos en cascada**: Soporta hasta `MAX7219_MAX_DEV` módulos MAX7219 encadenados, configurables en tiempo de ejecución.
- **Texto estático y desplazamiento**: `MAX7219_PrintString()` muestra texto fijo y `MAX7219_ScrollString()` lo desplaza por la matriz.
- **Dibujo en buffer**: `MAX7219_DrawPixel()`, `MAX7219_Fill()` y primitivas 2D (líneas horizontales/verticales, Bresenham, rectángulos, bitmaps compatibles con image2cpp) que solo escriben en el buffer; `MAX7219_UpdateScreen()` las muestra de una vez.
- **Texto con cursor**: `MAX7219_GotoXY()`, `MAX7219_Putc()` y `MAX7219_Puts()` escriben texto en cualquier posición sin borrar el resto del dibujo; `MAX7219_GetStringWidth()` permite centrarlo.
- **Coordenadas estándar**: origen (0,0) en la esquina superior izquierda, igual que el resto de pantallas del repositorio.
- **Brillo ajustable**: `MAX7219_SetIntensity()` con 16 niveles (0 a `MAX7219_INTENSITY_MAX`).
- **Inversión por software**: `MAX7219_ToggleInvert()` invierte el buffer y hace que las primitivas pinten con el color contrario.
- **Manejo robusto de errores**: Códigos de retorno específicos para error de SPI, timeout, módulo no inicializado o parámetro inválido (`MAX7219_Status_t`).
- **Modo de bajo consumo**: `MAX7219_SetSleep()` activa el modo shutdown sin perder la configuración almacenada.
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

MAX7219_SetIntensity(4);   // Opcional: Init arranca en el nivel 1
```

### 2. Mostrar texto estático
```c
if(MAX7219_PrintString("HOLA") != MAX7219_OK)
{
    sprintf(buffer, "%sError mandando cadena%s\r\n", COLOR_RED, COLOR_RESET);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}
```

`MAX7219_PrintString()` apaga las columnas que el texto no ocupa, así que no hace falta llamar antes a `MAX7219_Clear()`.

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

    MAX7219_Clear();
}
```

### 4. Dibujar en el buffer
```c
// Las funciones Draw* y Fill solo escriben en el buffer; UpdateScreen lo muestra de una vez
MAX7219_Fill(MAX7219_COLOR_OFF);
MAX7219_DrawRectangle(0, 0, MAX7219_GetWidth() - 1, 7, MAX7219_COLOR_ON);  // Marco
MAX7219_DrawLine(0, 0, 7, 7, MAX7219_COLOR_ON);                            // Diagonal
MAX7219_DrawFilledRectangle(12, 2, 4, 4, MAX7219_COLOR_ON);                // Cuadro relleno
MAX7219_UpdateScreen();
```

### 5. Texto con cursor (centrado)
```c
const char* msg = "OK";
uint16_t w = MAX7219_GetStringWidth(msg);

MAX7219_Fill(MAX7219_COLOR_OFF);
if (w <= MAX7219_GetWidth())                       // La resta es sin signo
{
    MAX7219_GotoXY((MAX7219_GetWidth() - w) / 2, 0);
}
MAX7219_Puts(msg, MAX7219_COLOR_ON);
MAX7219_UpdateScreen();
```

---

## Sistema de coordenadas

Desde la versión **3.0.0** el origen (0,0) es la **esquina superior izquierda**: `x` crece hacia la derecha (hasta `MAX7219_GetWidth() - 1`) e `y` hacia abajo (hasta 7).

> [!WARNING]
> En la versión 2.x, `MAX7219_SetPixel()` usaba como origen la esquina inferior derecha. Para conservar un dibujo antiguo sin rehacer las coordenadas:
> ```c
> MAX7219_SetPixel(MAX7219_GetWidth() - 1 - x, 7 - y, state);
> ```

---

## API Reference

### 1. Tipos de Datos y Constantes

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

#### `MAX7219_COLOR_t` - Estado de un LED

```c
typedef enum {
    MAX7219_COLOR_OFF = 0x00,   /**< LED apagado */
    MAX7219_COLOR_ON  = 0x01    /**< LED encendido */
} MAX7219_COLOR_t;
```

Lo reciben `MAX7219_Fill()`, `MAX7219_DrawPixel()`, las primitivas de dibujo y las funciones de texto con cursor. Con la inversión activa (`MAX7219_ToggleInvert()`) los colores se intercambian.

---

#### Constantes configurables

|Constante|Valor por defecto|Descripción|
|---|---|---|
|`MAX7219_MAX_DEV`|`4U`|Número máximo de MAX7219 en cascada; dimensiona el buffer estático. Redefinible antes de incluir el header (o con `-D`)|
|`MAX7219_SPI_TIMEOUT_MS`|`100U`|Timeout de cada transmisión SPI en ms. Redefinible igual que la anterior|
|`MAX7219_INTENSITY_MAX`|`15U`|Nivel máximo aceptado por `MAX7219_SetIntensity()` (fijo, viene del registro 0x0A)|

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

**Retorna**: `MAX7219_OK` si la inicialización fue exitosa, `MAX7219_INVALID_PARAM` si algún puntero es `NULL`, `GPIO_PIN` es 0 o `numDevices` está fuera de rango, `MAX7219_ERROR`/`MAX7219_TIMEOUT` si falla la comunicación SPI durante la configuración.

**Secuencia interna:**

1. Validar parámetros y almacenar configuración (handle SPI, puerto/pin CS, número de dispositivos); desactivar la inversión por software.
2. Configurar el modo sin decodificación, intensidad de brillo (nivel 1), escaneo de 8 columnas y modo de operación normal.
3. Deshabilitar el test de display y limpiar la pantalla (`MAX7219_Clear()`). Si la limpieza falla, el módulo queda marcado como no inicializado.

---

#### `MAX7219_DeInit()` - Desinicialización del Driver

Apaga la pantalla (modo shutdown) y libera la configuración almacenada (handle SPI, puerto/pin CS y número de dispositivos). La configuración se libera aunque falle el envío del comando de apagado.

```c
MAX7219_Status_t MAX7219_DeInit(void);
```

**Retorna**: el estado del envío del comando de apagado (`MAX7219_OK`, `MAX7219_ERROR` o `MAX7219_TIMEOUT`), o `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `MAX7219_SetSleep()` - Modo de Bajo Consumo

Activa o desactiva el modo de bajo consumo (shutdown) sin perder la configuración almacenada. A diferencia de `MAX7219_DeInit()`, el módulo puede reactivarse sin volver a llamar a `MAX7219_Init()`.

```c
MAX7219_Status_t MAX7219_SetSleep(uint8_t enable);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`enable`|`uint8_t`|Distinto de 0 para entrar en modo de bajo consumo, 0 para volver al modo normal|

**Retorna**: `MAX7219_OK` si la operación fue exitosa, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado, `MAX7219_ERROR`/`MAX7219_TIMEOUT` si falla el SPI.

---

#### `MAX7219_SetIntensity()` - Brillo

Escribe directamente en el registro de intensidad (0x0A) de todos los MAX7219 de la cascada; no usa el buffer ni requiere `MAX7219_UpdateScreen()`. El nivel 0 equivale a 1/32 de la corriente máxima y el 15 a 31/32. `MAX7219_Init()` arranca en el nivel 1.

```c
MAX7219_Status_t MAX7219_SetIntensity(uint8_t level);
```

|Parámetro|Tipo|Descripción|Rango|
|---|---|---|---|
|`level`|`uint8_t`|Nivel de brillo|0 a `MAX7219_INTENSITY_MAX` (15)|

**Retorna**: `MAX7219_OK` si el brillo se actualizó, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado, `MAX7219_INVALID_PARAM` si `level` es mayor que 15, `MAX7219_ERROR`/`MAX7219_TIMEOUT` si falla el SPI.

---

#### `MAX7219_Clear()` - Limpiar Pantalla

Apaga todos los LEDs de la matriz. Equivale a `MAX7219_Fill(MAX7219_COLOR_OFF)` seguido de `MAX7219_UpdateScreen()`: vacía el buffer y actualiza la pantalla de inmediato. Con la inversión activa deja todos los LEDs encendidos.

```c
MAX7219_Status_t MAX7219_Clear(void);
```

**Retorna**: `MAX7219_OK` si la operación fue exitosa, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado, `MAX7219_ERROR`/`MAX7219_TIMEOUT` si falla el SPI.

---

#### `MAX7219_PrintString()` - Mostrar Texto Estático

Convierte una cadena de texto en su representación de bytes y la muestra de forma estática en la matriz de LEDs, desde la izquierda. Las columnas no ocupadas por el texto se apagan y el texto que no cabe se recorta. Actualiza la pantalla de inmediato y no respeta la inversión por software.

```c
MAX7219_Status_t MAX7219_PrintString(const char *str);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`str`|`const char*`|Cadena de texto a mostrar|

**Retorna**: `MAX7219_OK` si la operación fue exitosa, `MAX7219_INVALID_PARAM` si `str` es `NULL`, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado, `MAX7219_ERROR`/`MAX7219_TIMEOUT` si falla el SPI.

---

#### `MAX7219_ScrollString()` - Desplazar Texto

Desplaza una cadena de texto a través de la matriz de LEDs, columna a columna, con un retardo configurable entre cada paso. Es bloqueante (usa `HAL_Delay()`) y no respeta la inversión por software.

```c
MAX7219_Status_t MAX7219_ScrollString(const char *str, uint32_t delay);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`str`|`const char*`|Cadena de texto a desplazar|
|`delay`|`uint32_t`|Retardo entre cada paso de desplazamiento, en milisegundos|

**Retorna**: `MAX7219_OK` si la operación fue exitosa, `MAX7219_INVALID_PARAM` si `str` es `NULL`, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado, `MAX7219_ERROR`/`MAX7219_TIMEOUT` si falla el SPI.

---

#### `MAX7219_SetPixel()` - Píxel con Actualización Inmediata

Enciende o apaga un píxel y actualiza la pantalla en el acto. Equivale a `MAX7219_DrawPixel()` seguido de `MAX7219_UpdateScreen()`; para dibujar varios píxeles es más eficiente usar `MAX7219_DrawPixel()` y actualizar una sola vez.

```c
MAX7219_Status_t MAX7219_SetPixel(uint16_t x, uint16_t y, uint8_t state);
```

|Parámetro|Tipo|Descripción|Rango|
|---|---|---|---|
|`x`|`uint16_t`|Columna del píxel (0 = izquierda)|0 a `MAX7219_GetWidth() - 1`|
|`y`|`uint16_t`|Fila del píxel (0 = arriba)|0 a 7|
|`state`|`uint8_t`|Distinto de 0 para encender el píxel, 0 para apagarlo|—|

**Retorna**: `MAX7219_OK` si el píxel se escribió y la pantalla se actualizó, `MAX7219_INVALID_PARAM` si `x` o `y` están fuera de rango, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado, `MAX7219_ERROR`/`MAX7219_TIMEOUT` si falla el SPI.

> [!WARNING]
> El origen cambió en la v3.0.0; ver [Sistema de coordenadas](#sistema-de-coordenadas).

---

#### `MAX7219_GetWidth()` / `MAX7219_GetHeight()` - Dimensiones

```c
uint16_t MAX7219_GetWidth(void);
uint16_t MAX7219_GetHeight(void);
```

**Retorna**: el ancho (`numDevices × 8`) y el alto (8) en píxeles, o 0 si el módulo no fue inicializado.

---

#### `MAX7219_UpdateScreen()` - Volcar el Buffer

Envía el contenido del buffer interno a la matriz. Las funciones de dibujo solo escriben en el buffer; los cambios se ven al llamar a esta función.

```c
MAX7219_Status_t MAX7219_UpdateScreen(void);
```

**Retorna**: `MAX7219_OK` si la pantalla se actualizó, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado, `MAX7219_ERROR`/`MAX7219_TIMEOUT` si falla el SPI.

---

#### `MAX7219_Fill()` - Rellenar el Buffer

Rellena todo el buffer con un color. Solo escribe en el buffer.

```c
MAX7219_Status_t MAX7219_Fill(MAX7219_COLOR_t color);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`color`|`MAX7219_COLOR_t`|`MAX7219_COLOR_ON` enciende todos los LEDs, `MAX7219_COLOR_OFF` los apaga|

**Retorna**: `MAX7219_OK` si el buffer se rellenó, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `MAX7219_ToggleInvert()` - Inversión por Software

El MAX7219 no tiene inversión por hardware, así que se hace sobre el buffer. Cada llamada invierte el contenido actual y alterna una bandera: mientras esté activa, `MAX7219_Fill()`, `MAX7219_Clear()`, `MAX7219_SetPixel()`, `MAX7219_DrawPixel()` y el resto de primitivas pintan con el color contrario, de modo que el dibujo conserva su aspecto invertido. `MAX7219_PrintString()` y `MAX7219_ScrollString()` no la respetan. La bandera se desactiva en `MAX7219_Init()`. Solo escribe en el buffer.

```c
MAX7219_Status_t MAX7219_ToggleInvert(void);
```

**Retorna**: `MAX7219_OK` si el buffer se invirtió, `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### Primitivas de dibujo

Todas solo escriben en el buffer (llamar a `MAX7219_UpdateScreen()` para mostrarlas) y retornan `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado.

```c
MAX7219_Status_t MAX7219_DrawPixel(uint16_t x, uint16_t y, MAX7219_COLOR_t color);
MAX7219_Status_t MAX7219_DrawFastHLine(int16_t x, int16_t y, int16_t w, MAX7219_COLOR_t color);
MAX7219_Status_t MAX7219_DrawFastVLine(int16_t x, int16_t y, int16_t h, MAX7219_COLOR_t color);
MAX7219_Status_t MAX7219_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, MAX7219_COLOR_t c);
MAX7219_Status_t MAX7219_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, MAX7219_COLOR_t c);
MAX7219_Status_t MAX7219_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, MAX7219_COLOR_t c);
MAX7219_Status_t MAX7219_DrawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, MAX7219_COLOR_t color);
```

|Función|Qué dibuja|Parámetros|Fuera de pantalla|
|---|---|---|---|
|`MAX7219_DrawPixel`|Un píxel|`x`, `y`, `color`|`MAX7219_INVALID_PARAM`|
|`MAX7219_DrawFastHLine`|Línea horizontal|Inicio (`x`, `y`) y ancho `w`; `w` negativo dibuja hacia la izquierda, `w = 0` no dibuja|Se recorta sin error|
|`MAX7219_DrawFastVLine`|Línea vertical|Inicio (`x`, `y`) y alto `h`; `h` negativo dibuja hacia arriba, `h = 0` no dibuja|Se recorta sin error|
|`MAX7219_DrawLine`|Línea entre dos puntos (Bresenham)|Extremos (`x0`, `y0`) y (`x1`, `y1`)|`MAX7219_INVALID_PARAM` si algún extremo está fuera (no dibuja nada)|
|`MAX7219_DrawRectangle`|Borde de un rectángulo|Dos esquinas opuestas en cualquier orden|`MAX7219_INVALID_PARAM` si alguna esquina está fuera (no dibuja nada)|
|`MAX7219_DrawFilledRectangle`|Rectángulo relleno|Esquina superior izquierda (`x`, `y`), ancho `w` y alto `h`|`MAX7219_INVALID_PARAM` si la esquina está fuera; lo que sobresale se recorta|
|`MAX7219_DrawBitmap`|Bitmap monocromo transparente (solo pinta los bits a 1)|Esquina (`x`, `y`, pueden ser negativas), datos, `w`, `h`, `color`|Se recorta sin error; `MAX7219_INVALID_PARAM` si `bitmap` es `NULL`|

> [!NOTE]
> El formato de `MAX7219_DrawBitmap()` es el mismo que `SSD1681_DrawBitmap()`: por filas, bit 7 = píxel de la izquierda y `(w + 7) / 8` bytes por fila (image2cpp, "Horizontal - 1 bit per pixel").

---

#### Texto con cursor

Escriben texto en el buffer a partir de un cursor (esquina superior izquierda del siguiente carácter). Cada carácter ocupa y avanza 7 columnas, igual que `MAX7219_PrintString()`, pero sin borrar el resto de la pantalla. Solo escriben en el buffer.

```c
MAX7219_Status_t MAX7219_GotoXY(uint16_t x, uint16_t y);
MAX7219_Status_t MAX7219_Putc(char ch, MAX7219_COLOR_t color);
MAX7219_Status_t MAX7219_Puts(const char* str, MAX7219_COLOR_t color);
uint16_t         MAX7219_GetStringWidth(const char* str);
```

|Función|Descripción|Retorno|
|---|---|---|
|`MAX7219_GotoXY`|Coloca el cursor en (`x`, `y`). No dibuja.|`MAX7219_OK`; `MAX7219_INVALID_PARAM` si el punto está fuera de la pantalla|
|`MAX7219_Putc`|Escribe un carácter (opaco: el fondo del glifo se pinta con el color contrario). Si no cabe entero se recorta y el cursor avanza igualmente.|`MAX7219_OK` aunque quede recortado; `MAX7219_INVALID_PARAM` si el cursor ya está fuera de la pantalla|
|`MAX7219_Puts`|Escribe una cadena terminada en `'\0'` llamando a `MAX7219_Putc`. El texto que pasa del borde derecho se recorta.|`MAX7219_OK` si se escribió entera; `MAX7219_INVALID_PARAM` si `str` es `NULL` o el texto llegó al borde antes de terminar|
|`MAX7219_GetStringWidth`|Ancho en columnas que ocupará la cadena con `MAX7219_Puts` (7 por carácter, sin recortar). No toca el buffer ni requiere `MAX7219_Init()`.|Ancho en columnas; 0 si `str` es `NULL` o está vacía|

Las tres primeras retornan `MAX7219_NOT_INITIALIZED` si el módulo no fue inicializado.

---

## Notas
- El número de dispositivos MAX7219 en cascada se indica mediante el parámetro `numDevices` de `MAX7219_Init()`. El máximo soportado se define con `MAX7219_MAX_DEV` en **MAX7219.h**, que determina el tamaño del buffer estático.
- El buffer interno ya no es accesible desde fuera de la librería (en la v2.x era `bufferCol`); para dibujar, usa las funciones `Draw*` y `MAX7219_UpdateScreen()`.

---

## Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

### [3.0.0] - 03-11-2026

API de dibujo sobre buffer con origen arriba a la izquierda, primitivas 2D, texto con cursor y brillo ajustable, alineada con el resto de pantallas del repositorio. Incluye **cambios incompatibles**: funciones renombradas, nuevo sistema de coordenadas en `MAX7219_SetPixel()` y buffer interno privado.

#### Added
- **Dibujo en buffer**: `MAX7219_DrawPixel()` y `MAX7219_Fill()` escriben en el buffer sin tocar la pantalla; `MAX7219_UpdateScreen()` lo vuelca de una vez. Nuevo tipo `MAX7219_COLOR_t` (`MAX7219_COLOR_OFF`/`MAX7219_COLOR_ON`).
- **Dimensiones**: `MAX7219_GetWidth()` y `MAX7219_GetHeight()` devuelven el tamaño configurado en `MAX7219_Init()` (0 si no está inicializado).
- **Brillo**: `MAX7219_SetIntensity()` ajusta el registro de intensidad (0 a `MAX7219_INTENSITY_MAX` = 15) en toda la cascada.
- **Primitivas 2D**: `MAX7219_DrawFastHLine()`, `MAX7219_DrawFastVLine()`, `MAX7219_DrawLine()` (Bresenham), `MAX7219_DrawRectangle()`, `MAX7219_DrawFilledRectangle()` y `MAX7219_DrawBitmap()` (formato image2cpp, transparente), con las mismas firmas y reglas de recorte que SSD1681.
- **Texto con cursor**: `MAX7219_GotoXY()`, `MAX7219_Putc()` y `MAX7219_Puts()` escriben texto en cualquier posición del buffer sin borrar el resto del dibujo.
- **Centrado de texto**: `MAX7219_GetStringWidth()` calcula el ancho en columnas de una cadena sin dibujarla.
- **Inversión por software**: `MAX7219_ToggleInvert()` invierte el buffer y hace que las primitivas pinten con el color contrario hasta la siguiente llamada.
- **Constante `MAX7219_SPI_TIMEOUT_MS`** (100 ms por defecto) para el timeout de cada transmisión SPI. Ésta y `MAX7219_MAX_DEV` se pueden redefinir antes de incluir el header.

#### Changed
- **[BREAKING]** `MAX7219_Set_Sleep()` se renombra a `MAX7219_SetSleep()`:
  `MAX7219_Set_Sleep(1)` → `MAX7219_SetSleep(1)`.
- **[BREAKING]** `MAX7219_ClearDisplay()` se renombra a `MAX7219_Clear()`:
  `MAX7219_ClearDisplay()` → `MAX7219_Clear()`.
- **[BREAKING]** `MAX7219_SetPixel()` cambia los tipos de las coordenadas y su origen pasa de la esquina inferior derecha a la **superior izquierda**:
  `MAX7219_SetPixel(int x, int y, uint8_t state)` → `MAX7219_SetPixel(uint16_t x, uint16_t y, uint8_t state)`.
  Para conservar un dibujo de la v2.x: `MAX7219_SetPixel(MAX7219_GetWidth() - 1 - x, 7 - y, state)`.
- **[BREAKING]** `MAX7219_ScrollString()` recibe una cadena constante y el retardo sin signo:
  `MAX7219_ScrollString(char *str, int delay)` → `MAX7219_ScrollString(const char *str, uint32_t delay)`. Las llamadas existentes compilan igual; solo cambia quien guarde un puntero a la función.
- `MAX7219_PrintString()` apaga las columnas que el texto no ocupa (antes quedaban restos del contenido anterior).

#### Fixed
- `MAX7219_PrintString()` ya no escribe fuera del buffer con cadenas más largas que la pantalla: el texto sobrante se recorta.
- `MAX7219_Init()` devolvía siempre `MAX7219_NOT_INITIALIZED`: limpiaba la pantalla antes de marcar el módulo como inicializado. Ahora lo marca antes, lo revierte si la limpieza falla y ya no ignora el resultado del comando de test de display.

#### Removed
- **[BREAKING]** Se eliminan del header el buffer global `bufferCol` y la macro `FONT_WIDTH`; el buffer pasa a ser privado. Quien escribiera en `bufferCol` debe usar `MAX7219_DrawPixel()`/`MAX7219_DrawBitmap()` y `MAX7219_UpdateScreen()`.

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
