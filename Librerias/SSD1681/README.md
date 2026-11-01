# Librería para la pantalla e-paper SSD1681 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32%20(HAL)-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
[![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/SSD1681)
[![Protocol](https://img.shields.io/badge/Protocol-SPI-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/SSD1681)

---

## Tabla de Contenidos
- [Librería para la pantalla e-paper SSD1681 en STM32](#librería-para-la-pantalla-e-paper-ssd1681-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Alimentación](#alimentación)
    - [Bus SPI](#bus-spi)
    - [GPIO de Control](#gpio-de-control)
  - [Configuración SPI](#configuración-spi)
    - [Parámetros en STM32CubeIDE / CubeMX](#parámetros-en-stm32cubeide--cubemx)
    - [Macros configurables](#macros-configurables)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Dibujo en pantalla](#2-dibujo-en-pantalla)
    - [3. Texto en pantalla](#3-texto-en-pantalla)
    - [4. Refresco parcial](#4-refresco-parcial)
    - [5. Deep sleep](#5-deep-sleep)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`SSD1681_Status_t` - Estados de Retorno](#ssd1681_status_t---estados-de-retorno)
      - [`SSD1681_COLOR_t` - Color de Píxel](#ssd1681_color_t---color-de-píxel)
      - [`FontDef_t` - Fuente Bitmap](#fontdef_t---fuente-bitmap)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`SSD1681_Init()` - Inicialización del Driver](#ssd1681_init---inicialización-del-driver)
      - [`SSD1681_GetWidth()` / `SSD1681_GetHeight()` - Consultar Dimensiones](#ssd1681_getwidth--ssd1681_getheight---consultar-dimensiones)
      - [`SSD1681_UpdateScreen()` - Refresco Completo](#ssd1681_updatescreen---refresco-completo)
      - [`SSD1681_UpdateScreenPartial()` - Refresco Parcial](#ssd1681_updatescreenpartial---refresco-parcial)
      - [`SSD1681_Sleep()` - Deep Sleep](#ssd1681_sleep---deep-sleep)
      - [`SSD1681_ToggleInvert()` - Invertir Colores](#ssd1681_toggleinvert---invertir-colores)
      - [`SSD1681_Fill()` - Rellenar Buffer](#ssd1681_fill---rellenar-buffer)
      - [`SSD1681_Clear()` - Limpiar Pantalla](#ssd1681_clear---limpiar-pantalla)
      - [`SSD1681_DrawPixel()` - Dibujar Píxel](#ssd1681_drawpixel---dibujar-píxel)
      - [`SSD1681_GotoXY()` / `SSD1681_Putc()` / `SSD1681_Puts()` - Texto](#ssd1681_gotoxy--ssd1681_putc--ssd1681_puts---texto)
      - [`SSD1681_DrawLine()` - Dibujar Línea](#ssd1681_drawline---dibujar-línea)
      - [`SSD1681_DrawFastVLine()` / `SSD1681_DrawFastHLine()` - Líneas Rápidas](#ssd1681_drawfastvline--ssd1681_drawfasthline---líneas-rápidas)
      - [`SSD1681_DrawRectangle()` / `SSD1681_DrawFilledRectangle()` - Rectángulos](#ssd1681_drawrectangle--ssd1681_drawfilledrectangle---rectángulos)
      - [`SSD1681_DrawTriangle()` / `SSD1681_DrawFilledTriangle()` - Triángulos](#ssd1681_drawtriangle--ssd1681_drawfilledtriangle---triángulos)
      - [`SSD1681_DrawCircle()` / `SSD1681_DrawFilledCircle()` - Círculos](#ssd1681_drawcircle--ssd1681_drawfilledcircle---círculos)
      - [`SSD1681_DrawEllipse()` / `SSD1681_DrawFilledEllipse()` - Elipses](#ssd1681_drawellipse--ssd1681_drawfilledellipse---elipses)
      - [`SSD1681_DrawArc()` / `SSD1681_DrawFilledArc()` - Arcos](#ssd1681_drawarc--ssd1681_drawfilledarc---arcos)
      - [`SSD1681_DrawBitmap()` - Dibujar Bitmap](#ssd1681_drawbitmap---dibujar-bitmap)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.0.0\] - 31-10-2026](#100---31-10-2026)
      - [Added](#added)

---

## Descripción

Driver en C para pantallas **e-paper monocromáticas (blanco/negro)** con controlador Solomon Systech **SSD1681** (y los compatibles **SSD1680** y **SSD1683**) sobre **SPI de 4 hilos** (SCK, MOSI, CS y un pin D/C para distinguir comando de dato), más los pines **RST** y **BUSY** propios de los paneles e-paper.

El dibujo se hace sobre un **buffer interno de 1 bit por píxel** en la RAM del STM32; el panel solo cambia cuando se llama a `SSD1681_UpdateScreen()` (refresco completo) o `SSD1681_UpdateScreenPartial()` (refresco parcial, sin parpadeo). La resolución se fija en tiempo de ejecución en `SSD1681_Init()`, hasta un máximo de `SSD1681_MAX_WIDTH` × `SSD1681_MAX_HEIGHT` (400×300 por defecto), así que la misma librería sirve para paneles de distintos tamaños.

> [!IMPORTANT]
> Un panel e-paper mantiene la imagen sin alimentación y tarda en refrescarse (el driver espera a que el panel libere BUSY en cada actualización). Agrupa todo el dibujo de un fotograma y llama a `SSD1681_UpdateScreen()` / `SSD1681_UpdateScreenPartial()` **una sola vez** al final.

---

## Características

- **Resolución configurable en tiempo de ejecución**: `SSD1681_Init()` recibe `width` y `height`; el buffer interno se dimensiona con `SSD1681_MAX_WIDTH` / `SSD1681_MAX_HEIGHT`, que pueden redefinirse antes de incluir el `.h` para paneles mayores (o menores, para ahorrar RAM: el buffer ocupa `((MAX_WIDTH + 7) / 8) × MAX_HEIGHT` bytes, 15 000 bytes con 400×300).
- **Refresco completo**: `SSD1681_UpdateScreen()` envía el buffer a la RAM blanco/negro del controlador (`0x24`) y lanza la secuencia de actualización completa (`0x22 = 0xF7`).
- **Refresco parcial sin parpadeo**: `SSD1681_UpdateScreenPartial()` usa la forma de onda del modo 2 (`0x22 = 0xFF`), que compara la RAM nueva (`0x24`) con la anterior (`0x26`) y solo mueve los píxeles que cambian. La primera llamada tras `SSD1681_Init()` o `SSD1681_UpdateScreen()` hace un refresco completo para fijar la imagen base.
- **Deep sleep con despertar automático**: `SSD1681_Sleep()` pone el panel en deep sleep modo 1 (la imagen sigue visible). La siguiente llamada a `SSD1681_UpdateScreen()` / `SSD1681_UpdateScreenPartial()` lo despierta con un reset por hardware y lo reconfigura, sin volver a llamar a `SSD1681_Init()`.
- **Espera de BUSY con timeout**: todas las esperas al panel terminan en `SSD1681_TIMEOUT` si BUSY no se libera en `SSD1681_BUSY_TIMEOUT_MS`, en lugar de colgar el programa (p. ej. con el panel desconectado).
- **Inversión de colores**: `SSD1681_ToggleInvert()` invierte el buffer y deja activa la inversión para el dibujo posterior.
- **Primitivas de dibujo completas**: píxeles, líneas (Bresenham) con variantes rápidas horizontal/vertical, rectángulos (contorno y relleno), triángulos (contorno y relleno), círculos (contorno y relleno), elipses (contorno y relleno), arcos/sectores de anillo entre dos ángulos (contorno y relleno) y bitmaps monocromos de 1 bit por píxel.
- **Texto**: `SSD1681_GotoXY()`, `SSD1681_Putc()` y `SSD1681_Puts()` con las fuentes de `FONTS.h` (`Font_7x10`, `Font_11x18`, `Font_16x26`).
- **Manejo robusto de errores**: retornos `SSD1681_Status_t` en todas las funciones públicas, con códigos específicos para error de comunicación, timeout, módulo no inicializado y parámetro inválido.
- **Portabilidad HAL**: solo requiere `main.h` (que incluye el HAL de STM32 correspondiente) y `FONTS.h`. Los pines CS, D/C, RST y BUSY se pasan como parámetros a `SSD1681_Init()`.

---

## Pinout y Conexiones

Los pines de control **no están definidos como macros**: se pasan como argumentos a `SSD1681_Init()`, por lo que puedes usar cualquier puerto/pin GPIO libre de tu placa.

```c
SSD1681_Status_t SSD1681_Init(SPI_HandleTypeDef* hspi,
                              GPIO_TypeDef* csPort, uint16_t csPin,
                              GPIO_TypeDef* dcPort, uint16_t dcPin,
                              GPIO_TypeDef* rstPort, uint16_t rstPin,
                              GPIO_TypeDef* busyPort, uint16_t busyPin,
                              uint16_t width, uint16_t height);
```

### Alimentación

| Pin del módulo | Descripción | Observaciones |
|----------------|-------------|---------------|
| **VCC** | Alimentación del módulo | Ver nota |
| **GND** | Tierra | Común con el STM32 |

> [!NOTE]
> Falta documentar la tensión de alimentación y si el módulo usado es tolerante a 5 V (depende de la placa del panel: modelo y regulador). Complétalo con el dato del módulo probado.

### Bus SPI

| Pin del módulo | Dirección | Descripción | Configuración CubeMX |
|----------------|-----------|-------------|----------------------|
| **SCK / CLK** | Input (al panel) | Reloj SPI | Configurado por el periférico SPI |
| **SDA / DIN / MOSI** | Input (al panel) | Datos del STM32 hacia el panel | Configurado por el periférico SPI |

> [!NOTE]
> El driver solo escribe (`HAL_SPI_Transmit`), nunca lee del panel. Puedes configurar el periférico como **Transmit Only Master**; **Full-Duplex Master** también funciona sin cambios en el código.

### GPIO de Control

| Pin del módulo | Dirección | Descripción | Tipo GPIO | Output Level inicial | Etiqueta CubeMX sugerida |
|----------------|-----------|-------------|-----------|----------------------|--------------------------|
| **CS** | Output | Chip Select por software (activo en bajo) | `GPIO_OUTPUT` | **High** | `EPD_CS` |
| **DC** | Output | Selección Comando (0) / Dato (1) | `GPIO_OUTPUT` | Indiferente | `EPD_DC` |
| **RST** | Output | Reset por hardware (activo en bajo) | `GPIO_OUTPUT` | **High** | `EPD_RST` |
| **BUSY** | Input | Panel ocupado (alto mientras trabaja) | `GPIO_INPUT` | — | `EPD_BUSY` |

> [!NOTE]
> El pulso de reset es LOW → `SSD1681_RESET_PULSE_MS` (10 ms) → HIGH → 10 ms. El nivel activo de BUSY es alto en los SSD168x; si tu panel lo invierte, redefine `SSD1681_BUSY_ACTIVE_LEVEL` a `GPIO_PIN_RESET` antes de incluir `SSD1681.h`.

---

## Configuración SPI

### Parámetros en STM32CubeIDE / CubeMX

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | Transmit Only Master *(o Full-Duplex Master)* | La librería solo transmite |
| **Hardware NSS Signal** | Disable | El CS se gestiona por software desde la librería |
| **Frame Format** | Motorola | — |
| **Data Size** | 8 Bits | — |
| **First Bit** | MSB First | Obligatorio |
| **Clock Polarity (CPOL)** | Low | **Modo SPI 0** |
| **Clock Phase (CPHA)** | 1 Edge | **Modo SPI 0** |
| **CRC Calculation** | Disabled | No utilizado |
| **Prescaler (Baud Rate)** | Ver nota | — |

> [!NOTE]
> Falta indicar el baud rate máximo probado con este panel (consúltalo en el datasheet del SSD1681 o en el módulo usado). Ten en cuenta que el buffer completo (hasta 15 000 bytes con 400×300) se envía en una sola llamada a `HAL_SPI_Transmit()` con timeout `SSD1681_SPI_TIMEOUT_MS` (1000 ms); con un reloj SPI muy bajo y un panel grande puede ser necesario subir ese timeout.

### Macros configurables

Todas se pueden redefinir antes de incluir `SSD1681.h`:

| Macro | Valor por defecto | Descripción |
|-------|-------------------|-------------|
| `SSD1681_MAX_WIDTH` | `400U` | Ancho máximo soportado (dimensiona el buffer) |
| `SSD1681_MAX_HEIGHT` | `300U` | Alto máximo soportado (dimensiona el buffer) |
| `SSD1681_SPI_TIMEOUT_MS` | `1000U` | Timeout de cada transmisión SPI |
| `SSD1681_BUSY_TIMEOUT_MS` | `5000U` | Espera máxima a que se libere BUSY |
| `SSD1681_RESET_PULSE_MS` | `10U` | Duración del pulso y la espera de reset |
| `SSD1681_BUSY_ACTIVE_LEVEL` | `GPIO_PIN_SET` | Nivel de BUSY mientras el panel está ocupado |

---

## Instalación

1. Copia `SSD1681.c`, `SSD1681.h`, `FONTS.c` y `FONTS.h` a tu proyecto (ej: `Librerias/SSD1681/`).
2. Incluye la librería en tu `main.c` o archivo principal:
   ```c
   #include "SSD1681.h"
   ```
3. Configura el SPI y los GPIO como se describe en [Configuración SPI](#configuración-spi).
4. Genera código y compila.

> [!NOTE]
> El driver usa `sinf()`, `cosf()` y `sqrtf()` (arcos y elipses), por lo que el proyecto debe enlazar con la librería matemática (`-lm`, activada por defecto en STM32CubeIDE).

---

## Uso Básico

### 1. Inicialización

```c
/* Después de HAL_Init(), MX_GPIO_Init() y MX_SPIx_Init() */
if (SSD1681_Init(&hspi1,
                 EPD_CS_GPIO_Port,   EPD_CS_Pin,
                 EPD_DC_GPIO_Port,   EPD_DC_Pin,
                 EPD_RST_GPIO_Port,  EPD_RST_Pin,
                 EPD_BUSY_GPIO_Port, EPD_BUSY_Pin,
                 200, 200) != SSD1681_OK)   /* Resolución del panel */
{
    Error_Handler();
}

SSD1681_Clear();  /* Buffer en blanco + refresco completo */
```

> [!IMPORTANT]
> `MX_GPIO_Init()` debe ejecutarse **antes** que `SSD1681_Init()`: el reset por hardware y la espera de BUSY se hacen dentro de la inicialización. Si `SSD1681_Init()` devuelve `SSD1681_TIMEOUT`, BUSY nunca se liberó: revisa el cableado de BUSY/RST y la alimentación del panel.

---

### 2. Dibujo en pantalla

```c
SSD1681_Fill(SSD1681_COLOR_WHITE);

SSD1681_DrawPixel(10, 10, SSD1681_COLOR_BLACK);
SSD1681_DrawLine(0, 0, 199, 199, SSD1681_COLOR_BLACK);
SSD1681_DrawFastHLine(0, 100, 200, SSD1681_COLOR_BLACK);
SSD1681_DrawFastVLine(100, 0, 200, SSD1681_COLOR_BLACK);

SSD1681_DrawRectangle(10, 10, 90, 60, SSD1681_COLOR_BLACK);         /* Esquinas opuestas */
SSD1681_DrawFilledRectangle(110, 10, 80, 50, SSD1681_COLOR_BLACK);  /* x, y, ancho, alto */

SSD1681_DrawTriangle(10, 190, 50, 120, 90, 190, SSD1681_COLOR_BLACK);
SSD1681_DrawFilledCircle(150, 150, 30, SSD1681_COLOR_BLACK);
SSD1681_DrawEllipse(100, 100, 60, 30, SSD1681_COLOR_BLACK);
SSD1681_DrawFilledArc(100, 100, 80, 70, 0.0f, 90.0f, SSD1681_COLOR_BLACK);

SSD1681_DrawBitmap(0, 0, icon_16x16, 16, 16, SSD1681_COLOR_BLACK);

SSD1681_UpdateScreen();  /* Nada se ve hasta aquí */
```

> [!NOTE]
> `SSD1681_DrawPixel()`, `SSD1681_DrawLine()`, `SSD1681_DrawRectangle()`, `SSD1681_DrawFilledRectangle()` y los triángulos devuelven `SSD1681_INVALID_PARAM` si un punto cae fuera de pantalla (`SSD1681_DrawFilledRectangle()` sí recorta el ancho/alto que sobresale). Las líneas rápidas, círculos, elipses, arcos y bitmaps recortan en silencio lo que queda fuera.

---

### 3. Texto en pantalla

```c
SSD1681_GotoXY(10, 20);
SSD1681_Puts("Hola STM32", &Font_11x18, SSD1681_COLOR_BLACK);

SSD1681_GotoXY(10, 45);
SSD1681_Putc('A', &Font_16x26, SSD1681_COLOR_BLACK);

SSD1681_UpdateScreen();
```

> [!NOTE]
> El texto se dibuja de **celda opaca**: los píxeles del glifo con el color indicado y el resto de la celda con el color contrario. No hay salto de línea automático: si un carácter no cabe en la pantalla desde el cursor, `SSD1681_Putc()` devuelve `SSD1681_INVALID_PARAM` y `SSD1681_Puts()` se detiene ahí. Solo se aceptan caracteres ASCII imprimibles (32..126).

---

### 4. Refresco parcial

Para relojes, contadores o lecturas de sensores, donde el refresco completo (con parpadeo) es demasiado lento:

```c
uint32_t n = 0;

while (1)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", n);

    SSD1681_DrawFilledRectangle(10, 80, 180, 26, SSD1681_COLOR_WHITE);  /* Borra el valor anterior */
    SSD1681_GotoXY(10, 80);
    SSD1681_Puts(buf, &Font_16x26, SSD1681_COLOR_BLACK);

    if (++n % 50 == 0) { SSD1681_UpdateScreen(); }         /* Limpia el ghosting */
    else               { SSD1681_UpdateScreenPartial(); }

    HAL_Delay(1000);
}
```

> [!WARNING]
> Los refrescos parciales acumulan *ghosting* (restos de imágenes anteriores). Intercala un `SSD1681_UpdateScreen()` cada cierto número de refrescos parciales. El refresco parcial solo es válido en paneles **blanco/negro**: en paneles BWR (rojo) la RAM `0x26` guarda el color rojo.

---

### 5. Deep sleep

```c
SSD1681_UpdateScreen();
SSD1681_Sleep();          /* La imagen sigue visible, el panel apenas consume */

/* ... minutos u horas después ... */
SSD1681_Puts("Nuevo", &Font_11x18, SSD1681_COLOR_BLACK);
SSD1681_UpdateScreen();   /* Despierta el panel (reset + reconfiguración) y refresca */
```

> [!TIP]
> El buffer interno no se modifica al dormir, así que se puede seguir dibujando mientras el panel está en deep sleep. No hace falta volver a llamar a `SSD1681_Init()`.

---

## API Reference

### 1. Tipos de Datos

#### `SSD1681_Status_t` - Estados de Retorno

```c
typedef enum {
    SSD1681_OK              = 0,    /**< Operación exitosa */
    SSD1681_ERROR           = 1,    /**< Error en la operación */
    SSD1681_TIMEOUT         = 2,    /**< Timeout en la operación */
    SSD1681_NOT_INITIALIZED = 3,    /**< Módulo no inicializado */
    SSD1681_INVALID_PARAM   = 4,    /**< Parámetro inválido */
} SSD1681_Status_t;
```

| Valor | Código | Significado |
|-------|--------|-------------|
| `SSD1681_OK` | 0 | Operación completada sin errores |
| `SSD1681_ERROR` | 1 | Fallo de transmisión SPI |
| `SSD1681_TIMEOUT` | 2 | BUSY no se liberó en `SSD1681_BUSY_TIMEOUT_MS` |
| `SSD1681_NOT_INITIALIZED` | 3 | `SSD1681_Init()` no se llamó o falló |
| `SSD1681_INVALID_PARAM` | 4 | Puntero NULL, coordenada fuera de pantalla o parámetro fuera de rango |

---

#### `SSD1681_COLOR_t` - Color de Píxel

```c
typedef enum {
    SSD1681_COLOR_BLACK = 0x00, /**< Píxel negro */
    SSD1681_COLOR_WHITE = 0x01  /**< Píxel blanco */
} SSD1681_COLOR_t;
```

---

#### `FontDef_t` - Fuente Bitmap

Definida en `FONTS.h` y usada por `SSD1681_Putc()` / `SSD1681_Puts()`:

```c
typedef struct {
    uint8_t FontWidth;    /*!< Ancho de la fuente en píxeles */
    uint8_t FontHeight;   /*!< Alto de la fuente en píxeles  */
    const uint16_t *data; /*!< Datos de la fuente            */
} FontDef_t;
```

Fuentes disponibles: `Font_7x10`, `Font_11x18` y `Font_16x26`.

---

### 2. Funciones Públicas

#### `SSD1681_Init()` - Inicialización del Driver

Valida los parámetros, registra los pines, aplica el reset por hardware y configura el controlador para el panel completo.

```c
SSD1681_Status_t SSD1681_Init(SPI_HandleTypeDef* hspi,
                              GPIO_TypeDef* csPort, uint16_t csPin,
                              GPIO_TypeDef* dcPort, uint16_t dcPin,
                              GPIO_TypeDef* rstPort, uint16_t rstPin,
                              GPIO_TypeDef* busyPort, uint16_t busyPin,
                              uint16_t width, uint16_t height);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `hspi` | `SPI_HandleTypeDef*` | Handle del periférico SPI |
| `csPort`, `csPin` | `GPIO_TypeDef*`, `uint16_t` | Pin CS |
| `dcPort`, `dcPin` | `GPIO_TypeDef*`, `uint16_t` | Pin D/C |
| `rstPort`, `rstPin` | `GPIO_TypeDef*`, `uint16_t` | Pin RST |
| `busyPort`, `busyPin` | `GPIO_TypeDef*`, `uint16_t` | Pin BUSY (entrada) |
| `width`, `height` | `uint16_t` | Resolución del panel (máx. `SSD1681_MAX_WIDTH` × `SSD1681_MAX_HEIGHT`) |

**Retorna**: `SSD1681_OK` si la inicialización fue exitosa; `SSD1681_INVALID_PARAM` si algún handle/puerto es NULL o la resolución es 0 o excede el máximo; `SSD1681_TIMEOUT` si BUSY no se liberó; `SSD1681_ERROR` si falla una transmisión SPI.

**Secuencia interna:**
1. Reset por hardware y espera de BUSY.
2. SW reset (`0x12`) y espera de BUSY.
3. Número de compuertas (`0x01`) según el alto del panel.
4. Modo de avance de RAM X+/Y+ por filas (`0x11 = 0x03`).
5. Ventana de RAM del panel completo (`0x44`/`0x45`) y contadores al origen (`0x4E`/`0x4F`).
6. Borde blanco (`0x3C = 0x05`) y sensor de temperatura interno (`0x18 = 0x80`).

---

#### `SSD1681_GetWidth()` / `SSD1681_GetHeight()` - Consultar Dimensiones

```c
uint16_t SSD1681_GetWidth(void);
uint16_t SSD1681_GetHeight(void);
```

**Retorna**: la resolución pasada a `SSD1681_Init()`, o `0` si el módulo no está inicializado.

---

#### `SSD1681_UpdateScreen()` - Refresco Completo

Envía el buffer a la RAM `0x24` y ejecuta el refresco completo. Despierta el panel si está en deep sleep. Invalida la imagen base del refresco parcial, así que el siguiente `SSD1681_UpdateScreenPartial()` será completo.

```c
SSD1681_Status_t SSD1681_UpdateScreen(void);
```

**Retorna**: `SSD1681_OK`, `SSD1681_ERROR`, `SSD1681_TIMEOUT` o `SSD1681_NOT_INITIALIZED`.

---

#### `SSD1681_UpdateScreenPartial()` - Refresco Parcial

Envía el buffer a la RAM `0x24`, ejecuta el refresco en modo 2 y copia el buffer a la RAM `0x26` como referencia para la siguiente llamada. Si la referencia no es válida (primera llamada tras `SSD1681_Init()`, tras `SSD1681_UpdateScreen()` o tras un error), hace un refresco completo. Despierta el panel si está en deep sleep. Solo para paneles blanco/negro.

```c
SSD1681_Status_t SSD1681_UpdateScreenPartial(void);
```

**Retorna**: `SSD1681_OK`, `SSD1681_ERROR`, `SSD1681_TIMEOUT` o `SSD1681_NOT_INITIALIZED`.

---

#### `SSD1681_Sleep()` - Deep Sleep

Espera a que BUSY se libere y pone el panel en deep sleep modo 1 (`0x10 = 0x01`, conserva la RAM). La imagen permanece visible. Si ya estaba dormido, no hace nada.

```c
SSD1681_Status_t SSD1681_Sleep(void);
```

**Retorna**: `SSD1681_OK`, `SSD1681_ERROR`, `SSD1681_TIMEOUT` o `SSD1681_NOT_INITIALIZED`.

---

#### `SSD1681_ToggleInvert()` - Invertir Colores

Invierte todos los píxeles del buffer y alterna el estado de inversión: mientras esté activo, `SSD1681_Fill()` y `SSD1681_DrawPixel()` (y por tanto todas las primitivas) dibujan con el color contrario al indicado.

```c
SSD1681_Status_t SSD1681_ToggleInvert(void);
```

**Retorna**: `SSD1681_OK` o `SSD1681_NOT_INITIALIZED`.

---

#### `SSD1681_Fill()` - Rellenar Buffer

```c
SSD1681_Status_t SSD1681_Fill(SSD1681_COLOR_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `color` | `SSD1681_COLOR_t` | Color de relleno |

**Retorna**: `SSD1681_OK` o `SSD1681_NOT_INITIALIZED`.

---

#### `SSD1681_Clear()` - Limpiar Pantalla

Rellena el buffer de blanco y hace un refresco completo (`SSD1681_UpdateScreen()`).

```c
SSD1681_Status_t SSD1681_Clear(void);
```

**Retorna**: lo mismo que `SSD1681_UpdateScreen()`.

---

#### `SSD1681_DrawPixel()` - Dibujar Píxel

```c
SSD1681_Status_t SSD1681_DrawPixel(uint16_t x, uint16_t y, SSD1681_COLOR_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x`, `y` | `uint16_t` | Coordenadas (0..ancho-1, 0..alto-1) |
| `color` | `SSD1681_COLOR_t` | Color del píxel |

**Retorna**: `SSD1681_OK`, `SSD1681_NOT_INITIALIZED`, o `SSD1681_INVALID_PARAM` si el píxel cae fuera de pantalla.

---

#### `SSD1681_GotoXY()` / `SSD1681_Putc()` / `SSD1681_Puts()` - Texto

```c
SSD1681_Status_t SSD1681_GotoXY(uint16_t x, uint16_t y);
SSD1681_Status_t SSD1681_Putc(char ch, FontDef_t* Font, SSD1681_COLOR_t color);
SSD1681_Status_t SSD1681_Puts(const char* str, FontDef_t* Font, SSD1681_COLOR_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x`, `y` | `uint16_t` | Posición del cursor (esquina superior izquierda de la celda) |
| `ch` / `str` | `char` / `const char*` | Carácter o cadena (ASCII 32..126) |
| `Font` | `FontDef_t*` | Fuente a utilizar (ver `FONTS.h`) |
| `color` | `SSD1681_COLOR_t` | Color del glifo; el fondo de la celda usa el contrario |

`SSD1681_Putc()` avanza el cursor `FontWidth` píxeles tras cada carácter.

**Retorna**: `SSD1681_OK`, `SSD1681_NOT_INITIALIZED`, o `SSD1681_INVALID_PARAM` si el cursor está fuera de pantalla, `Font`/`str` es NULL, el carácter no es imprimible o no cabe en la pantalla.

---

#### `SSD1681_DrawLine()` - Dibujar Línea

Línea entre dos puntos con el algoritmo de Bresenham.

```c
SSD1681_Status_t SSD1681_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1681_COLOR_t c);
```

**Retorna**: `SSD1681_OK`, `SSD1681_NOT_INITIALIZED`, o `SSD1681_INVALID_PARAM` si algún extremo cae fuera de pantalla.

---

#### `SSD1681_DrawFastVLine()` / `SSD1681_DrawFastHLine()` - Líneas Rápidas

Líneas verticales/horizontales sin Bresenham. La longitud puede ser negativa (se normaliza) y lo que sale de pantalla se recorta.

```c
SSD1681_Status_t SSD1681_DrawFastVLine(int16_t x, int16_t y, int16_t h, SSD1681_COLOR_t color);
SSD1681_Status_t SSD1681_DrawFastHLine(int16_t x, int16_t y, int16_t w, SSD1681_COLOR_t color);
```

**Retorna**: `SSD1681_OK` o `SSD1681_NOT_INITIALIZED`.

---

#### `SSD1681_DrawRectangle()` / `SSD1681_DrawFilledRectangle()` - Rectángulos

```c
SSD1681_Status_t SSD1681_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1681_COLOR_t c);
SSD1681_Status_t SSD1681_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1681_COLOR_t c);
```

| Función | Parámetros de geometría |
|---------|-------------------------|
| `SSD1681_DrawRectangle()` | Dos esquinas opuestas `(x0, y0)` y `(x1, y1)`, en cualquier orden |
| `SSD1681_DrawFilledRectangle()` | Esquina superior izquierda `(x, y)`, ancho `w` y alto `h`; lo que sobresale de la pantalla se recorta |

**Retorna**: `SSD1681_OK`, `SSD1681_NOT_INITIALIZED`, o `SSD1681_INVALID_PARAM` si alguna esquina (o el origen, en el relleno) cae fuera de pantalla.

---

#### `SSD1681_DrawTriangle()` / `SSD1681_DrawFilledTriangle()` - Triángulos

```c
SSD1681_Status_t SSD1681_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1681_COLOR_t color);
SSD1681_Status_t SSD1681_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1681_COLOR_t color);
```

**Retorna**: `SSD1681_OK`, `SSD1681_NOT_INITIALIZED`, o `SSD1681_INVALID_PARAM` si algún vértice cae fuera de pantalla.

---

#### `SSD1681_DrawCircle()` / `SSD1681_DrawFilledCircle()` - Círculos

```c
SSD1681_Status_t SSD1681_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1681_COLOR_t c);
SSD1681_Status_t SSD1681_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1681_COLOR_t c);
```

Lo que sale de pantalla se recorta sin error.

**Retorna**: `SSD1681_OK`, `SSD1681_NOT_INITIALIZED`, o `SSD1681_INVALID_PARAM` si `r < 0`.

---

#### `SSD1681_DrawEllipse()` / `SSD1681_DrawFilledEllipse()` - Elipses

```c
SSD1681_Status_t SSD1681_DrawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, SSD1681_COLOR_t color);
SSD1681_Status_t SSD1681_DrawFilledEllipse(int16_t x, int16_t y, int16_t rx, int16_t ry, SSD1681_COLOR_t color);
```

`rx` y `ry` son los radios horizontal y vertical; un radio a 0 degenera en una línea. Lo que sale de pantalla se recorta sin error.

**Retorna**: `SSD1681_OK`, `SSD1681_NOT_INITIALIZED`, o `SSD1681_INVALID_PARAM` si algún radio es negativo.

---

#### `SSD1681_DrawArc()` / `SSD1681_DrawFilledArc()` - Arcos

Contorno o relleno de un sector de anillo entre los radios `r1` y `r2` (se ordenan solos) y los ángulos `start` → `end`.

```c
SSD1681_Status_t SSD1681_DrawArc(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, SSD1681_COLOR_t color);
SSD1681_Status_t SSD1681_DrawFilledArc(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, SSD1681_COLOR_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x`, `y` | `int16_t` | Centro |
| `r1`, `r2` | `int16_t` | Radios exterior e interior |
| `start`, `end` | `float` | Ángulos en grados: 0° a la derecha, sentido horario. `0 → 360` dibuja el anillo completo |
| `color` | `SSD1681_COLOR_t` | Color |

Lo que sale de pantalla se recorta sin error.

**Retorna**: `SSD1681_OK`, `SSD1681_NOT_INITIALIZED`, o `SSD1681_INVALID_PARAM` si algún radio es negativo.

---

#### `SSD1681_DrawBitmap()` - Dibujar Bitmap

```c
SSD1681_Status_t SSD1681_DrawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, SSD1681_COLOR_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x`, `y` | `int16_t` | Esquina superior izquierda |
| `bitmap` | `const uint8_t*` | 1 bit por píxel, bit 7 = píxel de la izquierda, cada fila rellenada a byte completo |
| `w`, `h` | `int16_t` | Ancho y alto en píxeles |
| `color` | `SSD1681_COLOR_t` | Color de los bits a 1 (los bits a 0 no se dibujan) |

Lo que sale de pantalla se recorta sin error.

**Retorna**: `SSD1681_OK`, `SSD1681_NOT_INITIALIZED`, o `SSD1681_INVALID_PARAM` si `bitmap` es NULL.

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](../../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [1.0.0] - 31-10-2026

Primera versión publicada del driver para pantallas e-paper SSD1680/SSD1681/SSD1683 por SPI.

#### Added
- **Inicialización**: `SSD1681_Init()` con pines CS, D/C, RST y BUSY y resolución configurables en tiempo de ejecución (hasta `SSD1681_MAX_WIDTH` × `SSD1681_MAX_HEIGHT`), con espera de BUSY limitada por `SSD1681_BUSY_TIMEOUT_MS`.
- **Refresco completo**: `SSD1681_UpdateScreen()` envía el buffer de 1 bpp y refresca el panel; `SSD1681_Clear()` lo deja en blanco.
- **Refresco parcial**: `SSD1681_UpdateScreenPartial()` refresca sin parpadeo comparando con la imagen anterior (modo 2), para relojes, contadores y lecturas de sensores en paneles blanco/negro.
- **Deep sleep**: `SSD1681_Sleep()` duerme el panel conservando la imagen; el siguiente refresco lo despierta y reconfigura solo.
- **Buffer**: `SSD1681_Fill()`, `SSD1681_ToggleInvert()`, `SSD1681_GetWidth()` y `SSD1681_GetHeight()`.
- **Primitivas de dibujo**: píxel, líneas (y líneas rápidas horizontal/vertical), rectángulos, triángulos, círculos, elipses y arcos, en contorno y relleno, y bitmaps monocromos.
- **Texto**: `SSD1681_GotoXY()`, `SSD1681_Putc()` y `SSD1681_Puts()` con las fuentes de `FONTS.h` (`Font_7x10`, `Font_11x18`, `Font_16x26`).

---
