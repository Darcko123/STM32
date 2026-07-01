# Librería para el módulo SSD1306 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-2.1.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/SSD1306)
[![Protocol](https://img.shields.io/badge/Protocol-I2C-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/SSD1306)

> **Basado en el trabajo de:** Tilen Majerle y Alexander Lutsai (modificación para STM32F10x).
> Esta librería retoma el driver original y añade manejo robusto de errores mediante códigos de retorno (`SSD1306_Status_t`) en todas las funciones públicas, propagación de errores en las primitivas de dibujo y un algoritmo de relleno de triángulos optimizado.

---

## Tabla de Contenidos
- [Librería para el módulo SSD1306 en STM32](#librería-para-el-módulo-ssd1306-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
  - [Configuración I2C](#configuración-i2c)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Dibujar primitivas](#2-dibujar-primitivas)
    - [3. Escribir texto](#3-escribir-texto)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`SSD1306_Status_t` - Estados de Retorno](#ssd1306_status_t---estados-de-retorno)
      - [`SSD1306_COLOR_t` - Color de Píxel](#ssd1306_color_t---color-de-píxel)
      - [`FontDef_t` - Definición de Fuente](#fontdef_t---definición-de-fuente)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`SSD1306_Init()` - Inicialización del Driver](#ssd1306_init---inicialización-del-driver)
      - [`SSD1306_GetWidth()` / `SSD1306_GetHeight()` - Consultar Dimensiones](#ssd1306_getwidth--ssd1306_getheight---consultar-dimensiones)
      - [`SSD1306_UpdateScreen()` - Actualizar Pantalla](#ssd1306_updatescreen---actualizar-pantalla)
      - [`SSD1306_Fill()` - Rellenar Buffer](#ssd1306_fill---rellenar-buffer)
      - [`SSD1306_Clear()` - Limpiar Pantalla](#ssd1306_clear---limpiar-pantalla)
      - [`SSD1306_DrawPixel()` - Dibujar Píxel](#ssd1306_drawpixel---dibujar-píxel)
      - [`SSD1306_DrawLine()` - Dibujar Línea](#ssd1306_drawline---dibujar-línea)
      - [`SSD1306_DrawFastVLine()` / `SSD1306_DrawFastHLine()` - Líneas Rápidas](#ssd1306_drawfastvline--ssd1306_drawfasthline---líneas-rápidas)
      - [`SSD1306_DrawRectangle()` / `SSD1306_DrawFilledRectangle()` - Rectángulos](#ssd1306_drawrectangle--ssd1306_drawfilledrectangle---rectángulos)
      - [`SSD1306_DrawTriangle()` / `SSD1306_DrawFilledTriangle()` - Triángulos](#ssd1306_drawtriangle--ssd1306_drawfilledtriangle---triángulos)
      - [`SSD1306_DrawCircle()` / `SSD1306_DrawFilledCircle()` - Círculos](#ssd1306_drawcircle--ssd1306_drawfilledcircle---círculos)
      - [`SSD1306_DrawEllipse()` / `SSD1306_DrawFilledEllipse()` - Elipses](#ssd1306_drawellipse--ssd1306_drawfilledellipse---elipses)
      - [`SSD1306_DrawArc()` / `SSD1306_DrawFilledArc()` - Arcos](#ssd1306_drawarc--ssd1306_drawfilledarc---arcos)
      - [`SSD1306_DrawBitmap()` - Dibujar Bitmap](#ssd1306_drawbitmap---dibujar-bitmap)
      - [`SSD1306_GotoXY()` / `SSD1306_Putc()` / `SSD1306_Puts()` - Texto](#ssd1306_gotoxy--ssd1306_putc--ssd1306_puts---texto)
      - [`SSD1306_ToggleInvert()` / `SSD1306_InvertDisplay()` - Inversión de Color](#ssd1306_toggleinvert--ssd1306_invertdisplay---inversión-de-color)
      - [`SSD1306_ON()` / `SSD1306_OFF()` - Encendido y Apagado](#ssd1306_on--ssd1306_off---encendido-y-apagado)
      - [Funciones de Scroll por Hardware](#funciones-de-scroll-por-hardware)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[2.1.0\] - 30-06-2026](#210---30-06-2026)
      - [Added](#added)
      - [Changed](#changed)
      - [Fixed](#fixed)
    - [\[2.0.0\] - 29-06-2026](#200---29-06-2026)
      - [Added](#added-1)
    - [\[1.x.x\] - Versión Original](#1xx---versión-original)

---

## Descripción
Librería desarrollada en C para la interfaz con el controlador de pantalla OLED monocromática **SSD1306** utilizando microcontroladores STM32. Proporciona funciones para inicialización, primitivas de dibujo (píxeles, líneas, rectángulos, triángulos, círculos, bitmaps), texto con fuentes personalizadas y efectos de scroll por hardware. La librería está diseñada para ser fácil de usar, eficiente y compatible con la mayoría de las series STM32 (F1, F4, etc.) utilizando HAL.

Ideal para mostrar texto, gráficos simples, menús e indicadores de estado en proyectos embebidos con pantallas OLED de 128x64.

---

## Características
- **Comunicación I2C**: Escritura de comandos y datos al SSD1306 mediante `HAL_I2C_Master_Transmit`, con detección de dispositivo en `SSD1306_Init()`.
- **Dimensiones configurables en tiempo de ejecución**: El ancho y alto real de la pantalla se pasan como parámetros a `SSD1306_Init()`, con soporte para pantallas de hasta `SSD1306_MAX_WIDTH` × `SSD1306_MAX_HEIGHT` píxeles (128×64 por defecto, redefinibles antes de incluir `SSD1306.h`). Los comandos de multiplexado y COM pins se calculan automáticamente según el alto indicado.
- **Buffer en RAM**: Toda la pantalla se compone en un buffer local (1 bit por píxel, dimensionado al tamaño máximo soportado) antes de enviarse al hardware con `SSD1306_UpdateScreen()`.
- **Primitivas de dibujo completas**: Píxeles, líneas (algoritmo de Bresenham), líneas rápidas horizontales/verticales, rectángulos (borde y relleno), triángulos (borde y relleno mediante scanline), círculos (borde y relleno), elipses (borde y relleno), arcos/sectores de anillo (borde y relleno) y bitmaps monocromáticos.
- **Relleno de triángulos optimizado**: Algoritmo de scanline que ordena los vértices por Y y rellena por líneas horizontales (`ssd1306_FillHLine`), evitando el cómputo repetido de Bresenham.
- **Líneas rápidas**: `SSD1306_DrawFastVLine()` y `SSD1306_DrawFastHLine()` dibujan líneas horizontales/verticales directamente en el buffer, sin el cómputo de pendiente de Bresenham.
- **Elipses y arcos**: `SSD1306_DrawEllipse()`/`SSD1306_DrawFilledEllipse()` mediante el algoritmo de punto medio, y `SSD1306_DrawArc()`/`SSD1306_DrawFilledArc()` para sectores de anillo entre dos ángulos, con relleno por barrido fila a fila (`ssd1306_FillArcHelper`).
- **Texto con fuentes personalizadas**: Soporte para múltiples tamaños de fuente mediante `FontDef_t` (ver librería `FONTS.h`: 7x10, 11x18 y 16x26 píxeles).
- **Manejo robusto de errores**: Códigos de retorno específicos para error de I2C, timeout, módulo no inicializado o parámetro inválido (`SSD1306_Status_t`), propagados en cascada por todas las funciones de dibujo.
- **Inversión de color**: A nivel de buffer (`SSD1306_ToggleInvert`) o a nivel de controlador (`SSD1306_InvertDisplay`).
- **Efectos de scroll por hardware**: Scroll horizontal (izquierda/derecha) y diagonal, gestionados directamente por el controlador SSD1306.
- **Portabilidad**: Compatible con múltiples familias STM32 mediante la capa HAL. Solo requiere ajustar el `#include` de `main.h` según el proyecto.

---

## Pinout y Conexiones
### Pines requeridos

| Pin SSD1306 | Dirección | Descripción | Tipo GPIO | Observaciones |
|-------------|-----------|-------------|-----------|---------------|
| **VCC**     | Alimentación | 3.3V o 5V (según módulo) | N/A | Verificar el regulador integrado del módulo comercial |
| **GND**     | Tierra    | — | N/A | — |
| **SCL**     | Input (open-drain) | Línea de reloj I2C | `GPIO_AF_OD` | Requiere resistencia pull-up (típicamente 4.7kΩ) |
| **SDA**     | Input/Output (open-drain) | Línea de datos I2C | `GPIO_AF_OD` | Requiere resistencia pull-up (típicamente 4.7kΩ) |

> [!NOTE]
> La mayoría de los módulos comerciales OLED SSD1306 (128x64) ya integran las resistencias pull-up necesarias para I2C.

> [!WARNING]
> Verifica el voltaje de alimentación soportado por tu módulo específico antes de conectarlo. Algunos módulos solo soportan 3.3V.

---

## Configuración I2C
Configura tu periférico I2C en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | I2C | — |
| **I2C Speed Mode** | Standard Mode / Fast Mode | Soporta 100 kHz y 400 kHz |
| **I2C Clock Speed** | 100000 o 400000 Hz | Depende del requerimiento |

> [!NOTE]
> La dirección I2C del SSD1306 es: **0x78** (formato de 8 bits, ya desplazada). La librería ya la tiene definida como `SSD1306_I2C_ADDR 0x78U`. Si tu módulo responde en `0x3C`/`0x3D` (7 bits), ajusta la macro en `SSD1306.h` según corresponda.

---

## Instalación
1. Copia `SSD1306.c`, `SSD1306.h`, `FONTS.c` y `FONTS.h` a tu proyecto (ej: `Inc/SSD1306.h`, `Src/SSD1306.c`).
2. Incluye la librería en tu `main.c` o archivo principal:
   ```c
   #include "SSD1306.h"
   ```
3. Configura I2C en CubeMX (ver sección anterior).
4. Genera código y compila.

---

## Uso Básico

### 1. Inicialización
```c
// En main() después de HAL_Init() y MX_I2C1_Init()
SSD1306_Status_t status = SSD1306_Init(&hi2c1, 128, 64);

if (status != SSD1306_OK) {
    Error_Handler();  // Módulo no responde, handle NULL o dimensiones inválidas
}
```

### 2. Dibujar primitivas
```c
// Limpiar el buffer y dibujar algunas figuras
SSD1306_Fill(SSD1306_COLOR_BLACK);

SSD1306_DrawRectangle(10, 10, 50, 30, SSD1306_COLOR_WHITE);
SSD1306_DrawFilledCircle(90, 25, 12, SSD1306_COLOR_WHITE);
SSD1306_DrawLine(0, 50, 127, 50, SSD1306_COLOR_WHITE);

// Enviar el buffer a la pantalla física
SSD1306_UpdateScreen();
```

### 3. Escribir texto
```c
#include "FONTS.h"

SSD1306_GotoXY(0, 0);
SSD1306_Puts("Hola STM32!", &Font_7x10, SSD1306_COLOR_WHITE);

SSD1306_UpdateScreen();
```

> [!NOTE]
> Las funciones de dibujo y texto solo modifican el buffer en RAM. Es necesario llamar a `SSD1306_UpdateScreen()` para reflejar los cambios en la pantalla física.

---

## API Reference

### 1. Tipos de Datos

#### `SSD1306_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería, permitiendo una gestión robusta de errores y estados del chip.

```c
typedef enum {
    SSD1306_OK              = 0,    /**< Operación exitosa */
    SSD1306_ERROR           = 1,    /**< Error en la operación */
    SSD1306_TIMEOUT         = 2,    /**< Timeout en la operación */
    SSD1306_NOT_INITIALIZED = 3,    /**< Módulo no inicializado */
    SSD1306_INVALID_PARAM   = 4,    /**< Parámetro inválido */
} SSD1306_Status_t;
```

|Valor|Código|Significado|
|---|---|---|
|`SSD1306_OK`|0|Operación completada sin errores|
|`SSD1306_ERROR`|1|Error de I2C, parámetro inválido o condición inesperada|
|`SSD1306_TIMEOUT`|2|Timeout de software o timeout interno del chip|
|`SSD1306_NOT_INITIALIZED`|3|`SSD1306_Init()` no se llamó o falló|
|`SSD1306_INVALID_PARAM`|4|Coordenadas fuera de rango, puntero NULL o dimensión inválida|

---

#### `SSD1306_COLOR_t` - Color de Píxel

Enumeración para el color de un píxel individual en el buffer de pantalla.

```c
typedef enum {
	SSD1306_COLOR_BLACK = 0x00, /**< Píxel apagado (negro) */
	SSD1306_COLOR_WHITE = 0x01  /**< Píxel encendido (blanco) */
} SSD1306_COLOR_t;
```

|Valor|Código|Significado|
|---|---|---|
|`SSD1306_COLOR_BLACK`|0x00|Píxel apagado|
|`SSD1306_COLOR_WHITE`|0x01|Píxel encendido|

---

#### `FontDef_t` - Definición de Fuente

Estructura definida en `FONTS.h` que describe una fuente bitmap utilizada por `SSD1306_Putc()` y `SSD1306_Puts()`.

```c
typedef struct {
	uint8_t FontWidth;    /*!< Ancho de la fuente en píxeles */
	uint8_t FontHeight;   /*!< Alto de la fuente en píxeles */
	const uint16_t *data; /*!< Puntero a los datos de la fuente */
} FontDef_t;
```

|Campo|Tipo|Descripción|
|---|---|---|
|`FontWidth`|`uint8_t`|Ancho de cada carácter en píxeles|
|`FontHeight`|`uint8_t`|Alto de cada carácter en píxeles|
|`data`|`const uint16_t*`|Datos del bitmap de la fuente|

> [!NOTE]
> `FONTS.h` exporta tres fuentes predefinidas listas para usar: `Font_7x10`, `Font_11x18` y `Font_16x26`.

---

### 2. Funciones Públicas

#### `SSD1306_Init()` - Inicialización del Driver

Inicializa el controlador SSD1306 mediante I2C, verifica que el dispositivo responda, aplica la secuencia de configuración del datasheet y limpia la pantalla.

```c
SSD1306_Status_t SSD1306_Init(I2C_HandleTypeDef* hi2c, uint8_t width, uint8_t height);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`hi2c`|`I2C_HandleTypeDef*`|Puntero al handle de I2C|
|`width`|`uint8_t`|Ancho de la pantalla en píxeles|
|`height`|`uint8_t`|Alto de la pantalla en píxeles|

**Retorna**: `SSD1306_OK` si la inicialización fue exitosa, `SSD1306_ERROR` si ocurrió un error de comunicación, `SSD1306_INVALID_PARAM` si `hi2c` es NULL, `width` o `height` son 0, o exceden `SSD1306_MAX_WIDTH` / `SSD1306_MAX_HEIGHT`.

**Secuencia interna:**

1. Valida parámetros y verifica que el dispositivo responda en `SSD1306_I2C_ADDR`.
2. Almacena `width` y `height` en el estado interno para que todas las funciones operen con las dimensiones reales.
3. Envía la secuencia de comandos de inicialización: el ratio de multiplexado se calcula como `height - 1` y la configuración de COM pins se selecciona automáticamente (`0x12` para 64 píxeles de alto, `0x02` para otros valores).
4. Inicializa el estado interno del cursor y de inversión.
5. Limpia el buffer y actualiza la pantalla física.

---

#### `SSD1306_GetWidth()` / `SSD1306_GetHeight()` - Consultar Dimensiones

Retornan el ancho y alto de la pantalla tal como fueron configurados en `SSD1306_Init()`. Útiles cuando el código de aplicación necesita conocer las dimensiones en tiempo de ejecución sin acceso directo a las macros de compilación.

```c
uint16_t SSD1306_GetWidth(void);
uint16_t SSD1306_GetHeight(void);
```

**Retornan**: El ancho o alto en píxeles si el módulo está inicializado, o `0` si `SSD1306_Init()` no fue llamada o falló.

---

#### `SSD1306_UpdateScreen()` - Actualizar Pantalla

Envía el contenido completo del buffer interno a la pantalla física, página por página (una página por cada 8 píxeles de alto; el número de páginas se calcula en función del alto configurado en `SSD1306_Init()`).

```c
SSD1306_Status_t SSD1306_UpdateScreen(void);
```

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado, `SSD1306_ERROR` / `SSD1306_TIMEOUT` si falla la comunicación I2C.

---

#### `SSD1306_Fill()` - Rellenar Buffer

Rellena todo el buffer interno con un color sólido, sin actualizar la pantalla física.

```c
SSD1306_Status_t SSD1306_Fill(SSD1306_COLOR_t color);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`color`|`SSD1306_COLOR_t`|Color de relleno (`SSD1306_COLOR_BLACK` o `SSD1306_COLOR_WHITE`)|

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `SSD1306_Clear()` - Limpiar Pantalla

Rellena el buffer con negro y actualiza la pantalla física en una sola llamada.

```c
SSD1306_Status_t SSD1306_Clear(void);
```

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado, `SSD1306_ERROR` / `SSD1306_TIMEOUT` si falla la comunicación I2C.

---

#### `SSD1306_DrawPixel()` - Dibujar Píxel

Dibuja un único píxel en el buffer interno, aplicando la inversión de color si está activa.

```c
SSD1306_Status_t SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color);
```

|Parámetro|Tipo|Descripción|Rango|
|---|---|---|---|
|`x`|`uint16_t`|Coordenada X|0 – `SSD1306_GetWidth()-1`|
|`y`|`uint16_t`|Coordenada Y|0 – `SSD1306_GetHeight()-1`|
|`color`|`SSD1306_COLOR_t`|Color del píxel|—|

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado, `SSD1306_INVALID_PARAM` si `x` o `y` exceden las dimensiones de la pantalla.

---

#### `SSD1306_DrawLine()` - Dibujar Línea

Dibuja una línea recta entre dos puntos usando el algoritmo de Bresenham. Las líneas horizontales y verticales usan una ruta optimizada sin cómputo de pendiente.

```c
SSD1306_Status_t SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR_t c);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`x0`, `y0`|`uint16_t`|Coordenadas del punto de inicio|
|`x1`, `y1`|`uint16_t`|Coordenadas del punto final|
|`c`|`SSD1306_COLOR_t`|Color de la línea|

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado, propaga el error de `SSD1306_DrawPixel()` en caso de fallo.

---

#### `SSD1306_DrawFastVLine()` / `SSD1306_DrawFastHLine()` - Líneas Rápidas

Dibujan líneas verticales u horizontales escribiendo directamente en el buffer, sin el cómputo de pendiente del algoritmo de Bresenham. El alto/ancho puede ser negativo: la función normaliza el origen y la longitud antes de dibujar.

```c
SSD1306_Status_t SSD1306_DrawFastVLine(int16_t x, int16_t y, int16_t h, SSD1306_COLOR_t color);
SSD1306_Status_t SSD1306_DrawFastHLine(int16_t x, int16_t y, int16_t w, SSD1306_COLOR_t color);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`x`, `y`|`int16_t`|Coordenadas de inicio de la línea|
|`h`|`int16_t`|Alto de la línea vertical (puede ser negativo)|
|`w`|`int16_t`|Ancho de la línea horizontal (puede ser negativo)|
|`color`|`SSD1306_COLOR_t`|Color de la línea|

**Retorna**: `SSD1306_OK` si la operación fue exitosa (incluyendo el caso de longitud 0, donde no dibuja nada), `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `SSD1306_DrawRectangle()` / `SSD1306_DrawFilledRectangle()` - Rectángulos

Dibujan un rectángulo a partir de su esquina superior izquierda, ancho y alto. La versión `Filled` rellena el área completa trazando líneas horizontales sucesivas.

```c
SSD1306_Status_t SSD1306_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR_t c);
SSD1306_Status_t SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`x`, `y`|`uint16_t`|Coordenadas de la esquina superior izquierda|
|`w`, `h`|`uint16_t`|Ancho y alto del rectángulo (se recortan si exceden la pantalla)|
|`c`|`SSD1306_COLOR_t`|Color de la línea o del relleno|

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado, `SSD1306_INVALID_PARAM` si las coordenadas de origen exceden la pantalla.

---

#### `SSD1306_DrawTriangle()` / `SSD1306_DrawFilledTriangle()` - Triángulos

Dibujan un triángulo a partir de sus 3 vértices. `SSD1306_DrawTriangle()` traza solo el borde uniendo los vértices con `SSD1306_DrawLine()`. `SSD1306_DrawFilledTriangle()` ordena los vértices por coordenada Y y rellena mediante un algoritmo de scanline (`ssd1306_FillHLine`), incluyendo el caso degenerado donde los 3 vértices comparten la misma fila.

```c
SSD1306_Status_t SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color);
SSD1306_Status_t SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`x1`, `y1`|`uint16_t`|Coordenadas del primer vértice|
|`x2`, `y2`|`uint16_t`|Coordenadas del segundo vértice|
|`x3`, `y3`|`uint16_t`|Coordenadas del tercer vértice|
|`color`|`SSD1306_COLOR_t`|Color del triángulo|

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `SSD1306_DrawCircle()` / `SSD1306_DrawFilledCircle()` - Círculos

Dibujan un círculo a partir de su centro y radio, usando el algoritmo de punto medio (midpoint circle algorithm). La versión `Filled` rellena cada octante mediante líneas horizontales (`ssd1306_FillHLine`).

```c
SSD1306_Status_t SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c);
SSD1306_Status_t SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`x0`, `y0`|`int16_t`|Coordenadas del centro|
|`r`|`int16_t`|Radio del círculo|
|`c`|`SSD1306_COLOR_t`|Color del círculo|

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `SSD1306_DrawEllipse()` / `SSD1306_DrawFilledEllipse()` - Elipses

Dibujan una elipse a partir de su centro y sus radios horizontal/vertical, usando el algoritmo de punto medio para elipses (dos regiones según la pendiente). Si alguno de los radios es 0, la elipse degenera en una línea recta (vertical u horizontal). La versión `Filled` rellena el área mediante líneas horizontales (`ssd1306_FillHLine`).

```c
SSD1306_Status_t SSD1306_DrawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, SSD1306_COLOR_t color);
SSD1306_Status_t SSD1306_DrawFilledEllipse(int16_t x, int16_t y, int16_t rx, int16_t ry, uint16_t color);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`x0`/`x`, `y0`/`y`|`int16_t`|Coordenadas del centro|
|`rx`|`int16_t`|Radio horizontal|
|`ry`|`int16_t`|Radio vertical|
|`color`|`SSD1306_COLOR_t` / `uint16_t`|Color del borde o del relleno|

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado, `SSD1306_INVALID_PARAM` si `rx` o `ry` son negativos.

---

#### `SSD1306_DrawArc()` / `SSD1306_DrawFilledArc()` - Arcos

Dibujan un arco (sector de anillo) entre dos radios y dos ángulos, expresados en grados (0° = derecha, sentido horario). `SSD1306_DrawArc()` traza únicamente el contorno (bordes rectos en `start`/`end` y bordes curvos en los radios interior/exterior); `SSD1306_DrawFilledArc()` rellena todo el sector. Ambas funciones normalizan internamente los ángulos al rango [0°, 360°) y ordenan los radios para que `r1` sea siempre el exterior.

```c
SSD1306_Status_t SSD1306_DrawArc(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, SSD1306_COLOR_t color);
SSD1306_Status_t SSD1306_DrawFilledArc(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, uint16_t color);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`x`, `y`|`int16_t`|Coordenadas del centro|
|`r1`|`int16_t`|Radio exterior del arco|
|`r2`|`int16_t`|Radio interior del arco|
|`start`|`float`|Ángulo inicial en grados|
|`end`|`float`|Ángulo final en grados|
|`color`|`SSD1306_COLOR_t` / `uint16_t`|Color del contorno o del relleno|

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado, `SSD1306_INVALID_PARAM` si `r1` o `r2` son negativos.

---

#### `SSD1306_DrawBitmap()` - Dibujar Bitmap

Dibuja un bitmap monocromático (1 bit por píxel, empaquetado por byte) en la posición indicada.

```c
SSD1306_Status_t SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`x`, `y`|`int16_t`|Coordenadas de inicio|
|`bitmap`|`const unsigned char*`|Datos del bitmap|
|`w`, `h`|`int16_t`|Ancho y alto del bitmap en píxeles|
|`color`|`uint16_t`|Color de los píxeles activos|

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado, `SSD1306_INVALID_PARAM` si `bitmap` es NULL.

---

#### `SSD1306_GotoXY()` / `SSD1306_Putc()` / `SSD1306_Puts()` - Texto

`SSD1306_GotoXY()` posiciona el cursor de escritura. `SSD1306_Putc()` dibuja un carácter en la posición actual usando la fuente indicada y avanza el cursor. `SSD1306_Puts()` dibuja una cadena completa llamando a `SSD1306_Putc()` por cada carácter.

```c
SSD1306_Status_t SSD1306_GotoXY(uint16_t x, uint16_t y);
SSD1306_Status_t SSD1306_Putc(char ch, FontDef_t* Font, SSD1306_COLOR_t color);
SSD1306_Status_t SSD1306_Puts(char* str, FontDef_t* Font, SSD1306_COLOR_t color);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`x`, `y`|`uint16_t`|Coordenadas de inicio del cursor|
|`ch`|`char`|Carácter a dibujar|
|`str`|`char*`|Cadena a dibujar|
|`Font`|`FontDef_t*`|Fuente a utilizar (ver `FONTS.h`)|
|`color`|`SSD1306_COLOR_t`|Color del texto|

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado, `SSD1306_INVALID_PARAM` si `Font`/`str` es NULL o no hay espacio suficiente en pantalla para el carácter.

---

#### `SSD1306_ToggleInvert()` / `SSD1306_InvertDisplay()` - Inversión de Color

`SSD1306_ToggleInvert()` invierte el color de todos los píxeles ya almacenados en el buffer (negro↔blanco), afectando también a los píxeles que se dibujen después. `SSD1306_InvertDisplay()` invierte los colores a nivel de controlador, sin modificar el buffer.

```c
SSD1306_Status_t SSD1306_ToggleInvert(void);
SSD1306_Status_t SSD1306_InvertDisplay(int i);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`i`|`int`|Distinto de 0 para invertir, 0 para modo normal|

**Retorna**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado.

---

#### `SSD1306_ON()` / `SSD1306_OFF()` - Encendido y Apagado

Encienden o apagan la pantalla (modo de bajo consumo), gestionando el DC-DC interno del controlador.

```c
SSD1306_Status_t SSD1306_ON(void);
SSD1306_Status_t SSD1306_OFF(void);
```

**Retornan**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado, `SSD1306_ERROR` / `SSD1306_TIMEOUT` si falla la comunicación I2C.

---

#### Funciones de Scroll por Hardware

Inician o detienen efectos de scroll gestionados directamente por el controlador SSD1306, sin intervención del CPU una vez activados.

| Función | Descripción |
|---------|-------------|
| `SSD1306_ScrollRight(uint8_t start_row, uint8_t end_row)` | Scroll horizontal hacia la derecha |
| `SSD1306_ScrollLeft(uint8_t start_row, uint8_t end_row)` | Scroll horizontal hacia la izquierda |
| `SSD1306_Scrolldiagright(uint8_t start_row, uint8_t end_row)` | Scroll diagonal hacia la derecha (vertical + horizontal) |
| `SSD1306_Scrolldiagleft(uint8_t start_row, uint8_t end_row)` | Scroll diagonal hacia la izquierda (vertical + horizontal) |
| `SSD1306_Stopscroll(void)` | Detiene cualquier efecto de scroll activo |

```c
SSD1306_Status_t SSD1306_ScrollRight(uint8_t start_row, uint8_t end_row);
SSD1306_Status_t SSD1306_ScrollLeft(uint8_t start_row, uint8_t end_row);
SSD1306_Status_t SSD1306_Scrolldiagright(uint8_t start_row, uint8_t end_row);
SSD1306_Status_t SSD1306_Scrolldiagleft(uint8_t start_row, uint8_t end_row);
SSD1306_Status_t SSD1306_Stopscroll(void);
```

|Parámetro|Tipo|Descripción|Rango|
|---|---|---|---|
|`start_row`|`uint8_t`|Página de inicio|0 – 7|
|`end_row`|`uint8_t`|Página de fin|0 – 7|

**Retornan**: `SSD1306_OK` si la operación fue exitosa, `SSD1306_NOT_INITIALIZED` si el módulo no fue inicializado, `SSD1306_ERROR` / `SSD1306_TIMEOUT` si falla la comunicación I2C.

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [2.1.0] - 30-06-2026

#### Added
- Líneas rápidas horizontales y verticales sin cómputo de pendiente: `SSD1306_DrawFastVLine()` y `SSD1306_DrawFastHLine()`.
- Dibujo de elipses (borde y relleno) mediante el algoritmo de punto medio: `SSD1306_DrawEllipse()` y `SSD1306_DrawFilledEllipse()`.
- Dibujo de arcos/sectores de anillo entre dos ángulos (borde y relleno), con relleno por barrido fila a fila: `SSD1306_DrawArc()` y `SSD1306_DrawFilledArc()`.
- Soporte para pantallas de dimensiones configurables en tiempo de ejecución: el buffer interno se dimensiona al máximo soportado mediante las macros `SSD1306_MAX_WIDTH` / `SSD1306_MAX_HEIGHT` (128×64 por defecto, redefinibles con `#define` antes de incluir `SSD1306.h`), y el ancho/alto real se pasan a `SSD1306_Init()`.
- Nuevas funciones públicas `SSD1306_GetWidth()` y `SSD1306_GetHeight()` para consultar las dimensiones configuradas en tiempo de ejecución.

#### Changed
- `SSD1306_UpdateScreen()` transmite únicamente las páginas activas (`height / 8`) en lugar de las 8 páginas fijas.
- Todas las funciones internas (`DrawPixel`, `DrawLine`, `Fill`, `GotoXY`, `Putc`, rectángulos, scroll diagonal, etc.) usan las dimensiones reales del estado interno en lugar de las macros de compilación.
- Incluidas cabeceras adicionales necesarias (`math.h`, `float.h`, `stdbool.h`) para soportar las nuevas primitivas gráficas.
- Optimizado el algoritmo de relleno de arcos mediante `ssd1306_FillArcHelper()`.

#### Fixed
- Corregido el dimensionamiento del buffer interno para que soporte correctamente pantallas de diferentes tamaños.
- Ajustada la validación de parámetros en `SSD1306_Init()` para verificar que `width` y `height` no excedan `SSD1306_MAX_WIDTH` y `SSD1306_MAX_HEIGHT`.

---

### [2.0.0] - 29-06-2026

#### Added
- Implementación completa del driver SSD1306 para STM32 con interfaz I2C.
- Primitivas de dibujo: píxeles, líneas (Bresenham), rectángulos (borde y relleno), triángulos (borde y relleno por scanline), círculos (borde y relleno) y bitmaps monocromáticos.
- Algoritmo de relleno de triángulos optimizado mediante scanline, con manejo de casos degenerados.
- Soporte de texto con fuentes personalizadas a través de `FONTS.h` (`Font_7x10`, `Font_11x18`, `Font_16x26`).
- Funciones de inversión de color a nivel de buffer (`SSD1306_ToggleInvert`) y de controlador (`SSD1306_InvertDisplay`).
- Funciones de scroll por hardware: horizontal (izquierda/derecha) y diagonal.
- Manejo robusto de errores y propagación de estados (`SSD1306_Status_t`) en todas las funciones de dibujo.
- Funciones públicas `SSD1306_Init()`, `SSD1306_UpdateScreen()`, `SSD1306_Fill()`, `SSD1306_Clear()`, `SSD1306_ON()`, `SSD1306_OFF()`, entre otras.

---

### [1.x.x] - Versión Original

Desarrollada por **Tilen Majerle y Alexander Lutsai** Repositorio original: [stm32libs](https://github.com/SL-RU/stm32libs/tree/master/stm32f10x/ssd1306).

Esta versión implementó la interfaz básica del SSD1306 sobre I2C para STM32 HAL.