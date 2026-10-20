# Librería para la pantalla TFT LCD NV3007 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32%20(HAL)-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
[![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/NV3007)
[![Protocol](https://img.shields.io/badge/Protocol-SPI-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/NV3007)

---

## Tabla de Contenidos
- [Librería para la pantalla TFT LCD NV3007 en STM32](#librería-para-la-pantalla-tft-lcd-nv3007-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Alimentación](#alimentación)
    - [Bus SPI](#bus-spi)
    - [GPIO de Control](#gpio-de-control)
  - [Configuración SPI](#configuración-spi)
    - [Parámetros en STM32CubeIDE / CubeMX](#parámetros-en-stm32cubeide--cubemx)
    - [Baud Rate recomendado](#baud-rate-recomendado)
    - [Tabla de preescaladores](#tabla-de-preescaladores)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Dibujo en pantalla](#2-dibujo-en-pantalla)
    - [3. Texto en pantalla](#3-texto-en-pantalla)
    - [4. Orientación y offsets de GRAM](#4-orientación-y-offsets-de-gram)
  - [Calibración de offsets](#calibración-de-offsets)
    - [Inversión de color](#inversión-de-color)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`NV3007_Status_t` - Estados de Retorno](#nv3007_status_t---estados-de-retorno)
      - [`NV3007_Orientation_t` - Orientaciones de Pantalla](#nv3007_orientation_t---orientaciones-de-pantalla)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`NV3007_Init()` - Inicialización del Driver](#nv3007_init---inicialización-del-driver)
      - [`NV3007_DeInit()` - Desinicialización del Driver](#nv3007_deinit---desinicialización-del-driver)
      - [`NV3007_GetWidth()` / `NV3007_GetHeight()` - Geometría Activa](#nv3007_getwidth--nv3007_getheight---geometría-activa)
      - [`NV3007_Rotate()` - Rotar Pantalla](#nv3007_rotate---rotar-pantalla)
      - [`NV3007_SetOffset()` - Ajustar Offset de GRAM](#nv3007_setoffset---ajustar-offset-de-gram)
      - [`NV3007_InvertDisplay()` - Inversión de Color](#nv3007_invertdisplay---inversión-de-color)
      - [`NV3007_DisplayOn()` / `NV3007_DisplayOff()` - Encendido y Apagado](#nv3007_displayon--nv3007_displayoff---encendido-y-apagado)
      - [`NV3007_WriteAddrWindow()` - Ventana de Direccionamiento](#nv3007_writeaddrwindow---ventana-de-direccionamiento)
      - [`NV3007_WritePixel()` / `NV3007_DrawPixel()` - Escribir Píxel](#nv3007_writepixel--nv3007_drawpixel---escribir-píxel)
      - [`NV3007_FillScreen()` - Rellenar Pantalla](#nv3007_fillscreen---rellenar-pantalla)
      - [`NV3007_DrawLine()` - Dibujar Línea](#nv3007_drawline---dibujar-línea)
      - [`NV3007_DrawFastVLine()` / `NV3007_DrawFastHLine()` - Líneas Rápidas](#nv3007_drawfastvline--nv3007_drawfasthline---líneas-rápidas)
      - [`NV3007_DrawRectangle()` - Dibujar Rectángulo](#nv3007_drawrectangle---dibujar-rectángulo)
      - [`NV3007_WriteFilledRectangle()` / `NV3007_DrawFilledRectangle()` - Rectángulo Relleno](#nv3007_writefilledrectangle--nv3007_drawfilledrectangle---rectángulo-relleno)
      - [`NV3007_DrawCircle()` / `NV3007_DrawFilledCircle()` - Círculos](#nv3007_drawcircle--nv3007_drawfilledcircle---círculos)
      - [`NV3007_DrawTriangle()` / `NV3007_DrawFilledTriangle()` - Triángulos](#nv3007_drawtriangle--nv3007_drawfilledtriangle---triángulos)
      - [`NV3007_DrawEllipse()` / `NV3007_DrawFilledEllipse()` - Elipses](#nv3007_drawellipse--nv3007_drawfilledellipse---elipses)
      - [`NV3007_DrawArc()` / `NV3007_DrawFilledArc()` - Arcos](#nv3007_drawarc--nv3007_drawfilledarc---arcos)
      - [`NV3007_DrawRoundRect()` / `NV3007_DrawFilledRoundRect()` - Rectángulos Redondeados](#nv3007_drawroundrect--nv3007_drawfilledroundrect---rectángulos-redondeados)
      - [`NV3007_DrawBitmap()` - Dibujar Bitmap](#nv3007_drawbitmap---dibujar-bitmap)
      - [`NV3007_Putc()` - Renderizar Carácter](#nv3007_putc---renderizar-carácter)
      - [`NV3007_Puts()` - Renderizar Cadena](#nv3007_puts---renderizar-cadena)
      - [`NV3007_GetStringSize` - Ancho de Cadena](#nv3007_getstringsize---ancho-de-cadena)
  - [Colores Predefinidos](#colores-predefinidos)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.0.0\] - 19-10-2026](#100---19-10-2026)
      - [Added](#added)
      - [Changed](#changed)
      - [Fixed](#fixed)

---

## Descripción

Driver en C para el controlador LCD **NV3007** sobre interfaz **SPI de 4 hilos** (SCK, MOSI, CS y un pin DC dedicado para distinguir comando de dato), ajustado al panel **Estardyn 2.79" de 142×428 píxeles**. Es la traducción a **STM32 HAL** de la librería `Arduino_NV3007` del proyecto [Arduino_GFX](https://github.com/moononournation/Arduino_GFX), conservando sus primitivas de dibujo.

Proporciona una API completa para inicialización, control de orientación, ajuste de los offsets de direccionamiento de la GRAM, primitivas de dibujo (píxeles, líneas, rectángulos, círculos, triángulos, elipses, arcos y bitmaps) y renderizado de texto con las fuentes de `lcd_fonts.h`. Los pines CS, DC y RST son configurables en tiempo de ejecución, por lo que la librería no está atada a ninguna placa de desarrollo en particular.

La secuencia de inicialización es la variante de 2.79", que coincide byte a byte con el fichero de inicialización del fabricante (*NV3006A1N/NV3007 + IVO2.66*). La secuencia estándar de 168 columnas de Arduino_GFX **no** genera imagen en este panel aunque el backlight encienda.

> [!IMPORTANT]
> La GRAM del NV3007 es de **168×428**, pero el panel solo expone **142 columnas**. El área visible no empieza en la columna 0, así que `CASET`/`RASET` aplican un desplazamiento (`NV3007_OFFSET_*` en `NV3007.h`). Sin él la imagen sale corrida y recortada. Ver [Calibración de offsets](#calibración-de-offsets).

El formato de color es **RGB565 (16 bits por píxel)**, fijado por el comando `0x3A = 0x05` dentro de la secuencia de inicialización.

---

## Características

- **Driver solo-escritura sobre SPI de 4 hilos**: usa un pin DC dedicado para distinguir comando de dato. El panel nunca se lee, por lo que el periférico SPI puede configurarse como *Transmit Only Master* y liberar el pin MISO para otro uso.
- **Secuencia de inicialización proprietary del panel 2.79"**: 117 registros (365 bytes) almacenados como tabla (`NV3007_InitTable`) y reproducidos en una única transacción SPI con CS bajo, exactamente como el fichero del fabricante. La tabla se expone de forma pública (`extern const uint8_t NV3007_InitTable[]`) para poder replicarla por un transporte alternativo (p. ej. SPI de 3 hilos / 9 bits) sin duplicarla.
- **Validación de la configuración SPI en `NV3007_Init()`**: el panel no da ninguna señal de error cuando el bus está mal configurado (se queda negro con el backlight encendido, indistinguible de un fallo de cableado). El driver comprueba el tamaño de trama (8 bits), el orden de bits (MSB first) y el modo SPI (0 o 3) y devuelve `NV3007_INVALID_PARAM` si no se cumplen.
- **Rellenos eficientes por ráfagas**: la ventana de direccionamiento se abre una vez (`CASET`/`RASET`/`RAMWR`) y los píxeles viajan en una única transacción SPI con CS bajo, en bloques de 64 píxeles, sin conmutar CS por píxel.
- **Primitivas de dibujo completas**: píxeles, líneas (Bresenham) con variantes rápidas horizontal/vertical, rectángulos (contorno y relleno), rectángulos de esquinas redondeadas, círculos (contorno y relleno), triángulos (contorno y relleno por scanline), elipses (algoritmo de punto medio de Zingl), arcos/sectores de anillo entre dos ángulos (contorno y relleno) y bitmaps monocromos de 1 bit por píxel.
- **Paleta de 147 colores predefinidos** en formato RGB565 basada en el estándar de nombres de color X11/CSS, generada con la macro pública `RGB565(r, g, b)`. Incluye también `RGB16TO24(c)` para expandir un color RGB565 de vuelta a RGB888.
- **Renderizado de texto**: `NV3007_Putc()` / `NV3007_Puts()` con las fuentes de `lcd_fonts.h` (`LCD_Font_7x10`, `LCD_Font_11x18`, `LCD_Font_16x26`), con saltos de línea, retorno de carro, fondo opaco y salto de línea automático al llegar al borde.
- **Offsets de GRAM por orientación**: el driver aplica automáticamente el desplazamiento de `CASET`/`RASET` correspondiente a cada orientación, con los valores confirmados en hardware sobre este mismo panel por tres implementaciones independientes.
- **Cuatro orientaciones de pantalla**: Portrait 0°/180° y Landscape 90°/270° configurables en tiempo de ejecución con `NV3007_Rotate()`, que además actualiza la geometría lógica y recarga los offsets.
- **Calibración en tiempo de ejecución**: `NV3007_SetOffset()` permite ajustar el desplazamiento de la GRAM sin recompilar, para luego trasladar el valor probado a las constantes `NV3007_OFFSET_*`.
- **Manejo robusto de errores**: retornos `NV3007_Status_t` en todas las funciones públicas con valores específicos para parámetro inválido, error de transmisión, timeout, módulo no inicializado y parámetro inválido.
- **Portabilidad HAL**: solo requiere `main.h` (que incluye el HAL de STM32 correspondiente) y `lcd_fonts.h` para compilar. Los pines CS, DC y RST se pasan como parámetros a `NV3007_Init()`, por lo que no hay nada que editar en la librería para adaptarla a otra placa o distribución de pines.

---

## Pinout y Conexiones

Los pines de control **no están definidos como macros**: se pasan como argumentos a `NV3007_Init()`, por lo que puedes usar cualquier puerto/pin GPIO libre de tu placa.

```c
NV3007_Status_t NV3007_Init(SPI_HandleTypeDef* hspi,
                            GPIO_TypeDef* CS_GPIOx,  uint16_t CS_Pin,
                            GPIO_TypeDef* DC_GPIOx,  uint16_t DC_Pin,
                            GPIO_TypeDef* RST_GPIOx, uint16_t RST_Pin);
```

### Alimentación

| Pin del módulo | Descripción | Observaciones |
|----------------|-------------|---------------|
| **VCC**  | Alimentación lógica 3.3 V | El NV3007 **no es tolerante a 5 V**. Si tu módulo no trae regulador, aliméntalo desde los 3.3 V del STM32 |
| **GND**  | Tierra | Común con el STM32 |
| **BL / LEDA** *(si existe)* | Retroiluminación | **No gestionada por la librería**. Conéctala a 3.3 V para brillo fijo, o a un pin `GPIO_OUTPUT` / canal PWM (TIM) si quieres controlarla desde el código |

### Bus SPI

| Pin del módulo | Dirección | Descripción | Configuración CubeMX |
|----------------|-----------|-------------|----------------------|
| **SCK / SCL**  | Input (al panel) | Reloj SPI | Configurado por el periférico SPI |
| **SDA / MOSI** | Input (al panel) | Datos del STM32 hacia el panel | Configurado por el periférico SPI |
| **MISO / SDO** | — | **No se usa** | La librería nunca lee del panel; el pin puede quedar libre |

> [!NOTE]
> El driver solo escribe (`HAL_SPI_Transmit`). Por eso puedes configurar el periférico como **Transmit Only Master** y liberar el pin MISO para otro uso. **Full-Duplex Master** también funciona sin cambios en el código.

### GPIO de Control

| Pin del módulo | Dirección | Descripción | Tipo GPIO | Output Level inicial | Output Type | Pull-up/Pull-down | Maximum Output Speed | Etiqueta CubeMX sugerida |
|----------------|-----------|-------------|-----------|----------------------|-------------|-------------------|----------------------|--------------------------|
| **CS**  | Output | Chip Select por software | `GPIO_OUTPUT` | **High** | Push-Pull | No pull-up and no pull-down | **Very High** | `LCD_CS` |
| **DC / RS** | Output | Selección Comando (0) / Dato (1) | `GPIO_OUTPUT` | Indiferente (Low) | Push-Pull | No pull-up and no pull-down | **Very High** | `LCD_DC` |
| **RST** | Output | Reset por hardware del panel | `GPIO_OUTPUT` | **High** | Push-Pull | No pull-up and no pull-down | Low | `LCD_RST` |

> [!IMPORTANT]
> **CS** y **DC** deben configurarse con *Maximum Output Speed* = **Very High**. Ambos conmutan alrededor de cada byte enviado (`NV3007_Select()` / `NV3007_WriteCommandRaw()` / `NV3007_WriteDataRaw()`), y con velocidad de salida baja el flanco puede llegar tarde respecto al reloj SPI y hacer que el panel interprete un dato como comando (síntoma típico: pantalla en blanco o con basura tras el `Init`).

> [!NOTE]
> **RST es obligatorio.** `NV3007_Init()` retorna `NV3007_INVALID_PARAM` si se le pasa `NULL` o pin `0`; no existe camino de reset por software. La secuencia aplicada es: HIGH → 100 ms → LOW → 120 ms (`NV3007_RST_DELAY`) → HIGH → 120 ms.

---

## Configuración SPI

### Parámetros en STM32CubeIDE / CubeMX

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | Transmit Only Master *(o Full-Duplex Master)* | La librería solo transmite |
| **Hardware NSS Signal** | Disable | El CS se gestiona por software desde la librería |
| **Frame Format** | Motorola | — |
| **Data Size** | 8 Bits | El driver envía byte a byte (comandos y datos RGB565 en dos bytes, MSB primero) |
| **First Bit** | MSB First | Obligatorio |
| **Prescaler (Baud Rate)** | Ver [tabla](#tabla-de-preescaladores) | Objetivo ≈ **20 Mbit/s** |
| **Clock Polarity (CPOL)** | Low | **Modo SPI 0** |
| **Clock Phase (CPHA)** | 1 Edge | **Modo SPI 0** |
| **CRC Calculation** | Disabled | No utilizado |
| **NSSP Mode** *(si aparece)* | Disabled | Solo aplica con NSS por hardware |

> [!IMPORTANT]
> El **Modo SPI 0** (CPOL = Low, CPHA = 1 Edge) no es opcional: la librería original fuerza `SPI_MODE0` para este panel (`_override_datamode = SPI_MODE0` en `Arduino_NV3007.cpp`). Con cualquier otro modo el panel no responde correctamente a la secuencia de inicialización. `NV3007_Init()` valida este punto y devuelve `NV3007_INVALID_PARAM` si el modo no es 0 ni 3.

### Baud Rate recomendado

| Etapa | Velocidad | Motivo |
|-------|-----------|--------|
| **Puesta en marcha / depuración** | **5 – 10 Mbit/s** | Tolera cables dupont largos y protoboard. Úsala para validar el cableado la primera vez |
| **Operación normal (recomendado)** | **≈ 20 Mbit/s** | Mejor relación velocidad/robustez con PCB o cables cortos (< 10 cm) |
| **Máximo práctico** | **≈ 40 Mbit/s** | Solo con PCB, pistas cortas y GND sólido. Por encima de este valor la mejora es marginal (ver nota) |

> [!NOTE]
> **Por qué no vale la pena subir más allá de ~20–40 Mbit/s con esta librería:** cada byte se envía con una llamada independiente a `HAL_SPI_Transmit()` y, en los rellenos (`NV3007_FillColor()`), cada píxel es una transmisión de 2 bytes. El coste dominante es el *overhead* de CPU por llamada, no el reloj del bus, así que duplicar el baud rate a partir de cierto punto casi no reduce el tiempo de un `NV3007_FillScreen()`.

> [!WARNING]
> Si al aumentar el baud rate aparecen píxeles con color incorrecto, franjas o la pantalla queda en blanco, **baja un escalón el preescalador** (mitad de frecuencia) antes de sospechar del código. Es el síntoma clásico de integridad de señal en el cableado.

### Tabla de preescaladores

La frecuencia del bus es `f_SPI = f_PCLK / Prescaler`, donde `f_PCLK` es el reloj del bus APB al que está conectado el periférico (APB2 para SPI1/SPI4/SPI5/SPI6, APB1 para SPI2/SPI3 en la mayoría de familias STM32). CubeMX muestra el resultado en tiempo real en el campo *Baud Rate*.

| Prescaler | f_PCLK = 42 MHz | f_PCLK = 45 MHz | f_PCLK = 84 MHz | f_PCLK = 90 MHz | f_PCLK = 100 MHz |
|-----------|-----------------|-----------------|-----------------|-----------------|------------------|
| **/2**  | 21.0 Mbit/s ✅ | 22.5 Mbit/s ✅ | 42.0 Mbit/s ⚠️ | 45.0 Mbit/s ⚠️ | 50.0 Mbit/s ⚠️ |
| **/4**  | 10.5 Mbit/s ✅ | 11.25 Mbit/s ✅ | 21.0 Mbit/s ✅ | 22.5 Mbit/s ✅ | 25.0 Mbit/s ✅ |
| **/8**  | 5.25 Mbit/s 🔹 | 5.625 Mbit/s 🔹 | 10.5 Mbit/s ✅ | 11.25 Mbit/s ✅ | 12.5 Mbit/s ✅ |
| **/16** | 2.625 Mbit/s 🔹 | 2.8125 Mbit/s 🔹 | 5.25 Mbit/s 🔹 | 5.625 Mbit/s 🔹 | 6.25 Mbit/s 🔹 |
| **/32** | 1.31 Mbit/s | 1.41 Mbit/s | 2.625 Mbit/s 🔹 | 2.8125 Mbit/s 🔹 | 3.125 Mbit/s 🔹 |

**Leyenda:** ✅ recomendado · 🔹 lento pero seguro (depuración) · ⚠️ solo con PCB y cableado corto

**Ejemplos concretos:**

| MCU / periférico | f_PCLK | Prescaler sugerido | Baud Rate resultante |
|------------------|--------|--------------------|----------------------|
| STM32F411 @ 100 MHz — SPI1 (APB2) | 100 MHz | **/4** | 25 Mbit/s |
| STM32F407 @ 168 MHz — SPI1 (APB2) | 84 MHz | **/4** | 21 Mbit/s |
| STM32F407 @ 168 MHz — SPI2/SPI3 (APB1) | 42 MHz | **/2** | 21 Mbit/s |
| STM32F429 @ 180 MHz — SPI5 (APB2) | 90 MHz | **/4** | 22.5 Mbit/s |
| STM32F103 @ 72 MHz — SPI1 (APB2) | 72 MHz | **/4** | 18 Mbit/s |

---

## Instalación

1. Copia `NV3007.c`, `NV3007.h`, `lcd_fonts.h` y `lcd_fonts.c` a tu proyecto (ej: `Librerias/NV3007/`).
2. Incluye la librería en tu `main.c` o archivo principal:
   ```c
   #include "NV3007.h"
   ```
3. Configura el SPI y los GPIO como se describe en [Configuración SPI](#configuración-spi).
4. Genera código y compila.

> [!IMPORTANT]
> **`lcd_fonts.h` y `lcd_fonts.c` son obligatorios**: `NV3007.h` incluye `lcd_fonts.h` para definir el tipo `LCD_FontDef_t` que usan `NV3007_Putc()` y `NV3007_Puts()`, y `lcd_fonts.c` contiene los datos de las fuentes. Ambos archivos están en `Librerias/ILI9341_Disc1/` y son los mismos que usa el driver del ILI9341, así que las fuentes (`LCD_Font_7x10`, `LCD_Font_11x18`, `LCD_Font_16x26`) son compartidas entre los dos drivers.

---

## Uso Básico

### 1. Inicialización

```c
/* Pines de control del panel: se pasan como parámetros, no hay macros que editar */
#define LCD_CS_GPIO_Port   GPIOC
#define LCD_CS_Pin         GPIO_PIN_2
#define LCD_DC_GPIO_Port   GPIOD
#define LCD_DC_Pin         GPIO_PIN_12
#define LCD_RST_GPIO_Port  GPIOD
#define LCD_RST_Pin        GPIO_PIN_13

/* Después de HAL_Init(), MX_GPIO_Init() y MX_SPIx_Init() */
if (NV3007_Init(&hspi1,
                 LCD_CS_GPIO_Port,  LCD_CS_Pin,
                 LCD_DC_GPIO_Port,  LCD_DC_Pin,
                 LCD_RST_GPIO_Port, LCD_RST_Pin) != NV3007_OK)
{
    Error_Handler();
}

NV3007_FillScreen(NV3007_COLOR_BLACK);  /* Pantalla en negro (RGB565) */
```

> [!IMPORTANT]
> `MX_GPIO_Init()` debe ejecutarse **antes** que `NV3007_Init()`, ya que el reset por hardware y el primer `CS = HIGH` se aplican dentro de la inicialización de la librería.

> [!NOTE]
> Tras `NV3007_Init()` la pantalla queda en `NV3007_Orientation_Landscape_1` (428×142). Para trabajar en la orientación nativa del cristal (142×428), llama a `NV3007_Rotate(NV3007_Orientation_Portrait_1)` después de inicializar.

> [!TIP]
> `NV3007_Init()` valida la configuración del periférico SPI y devuelve `NV3007_INVALID_PARAM` si el tamaño de trama no es de 8 bits, si no es MSB First, o si el modo SPI no es 0 ni 3. Un bus mal configurado deja el panel en negro con el backlight encendido, síntoma indistinguible de un fallo de cableado; esta comprobación lo descarta de entrada.

---

### 2. Dibujo en pantalla

```c
/* Limpiar la pantalla con color negro */
NV3007_FillScreen(NV3007_COLOR_BLACK);

/* Dibujar un píxel rojo en (120, 160) */
NV3007_DrawPixel(120, 160, NV3007_COLOR_RED);

/* Dibujar una línea azul */
NV3007_DrawLine(0, 0, 239, 319, NV3007_COLOR_BLUE);

/* Dibujar una línea vertical/horizontal rápida (sin Bresenham) */
NV3007_DrawFastVLine(120, 0, 141, NV3007_COLOR_GRAY);
NV3007_DrawFastHLine(0, 70, 427, NV3007_COLOR_GRAY);

/* Dibujar el contorno de un rectángulo */
NV3007_DrawRectangle(20, 20, 220, 100, NV3007_COLOR_GREEN);

/* Dibujar un rectángulo relleno */
NV3007_DrawFilledRectangle(20, 20, 220, 100, NV3007_COLOR_YELLOW);

/* Rectángulo con esquinas redondeadas (radio = 12) */
NV3007_DrawRoundRect(20, 20, 220, 100, 12, NV3007_COLOR_ORANGE);
NV3007_DrawFilledRoundRect(20, 20, 220, 100, 12, NV3007_COLOR_ORANGE);

/* Contorno y relleno de un círculo */
NV3007_DrawCircle(120, 160, 50, NV3007_COLOR_CYAN);
NV3007_DrawFilledCircle(120, 160, 30, NV3007_COLOR_MAGENTA);

/* Contorno y relleno de un triángulo */
NV3007_DrawTriangle(10, 10, 230, 10, 120, 150, NV3007_COLOR_RED);
NV3007_DrawFilledTriangle(10, 10, 230, 10, 120, 150, NV3007_COLOR_BLUE);

/* Contorno y relleno de una elipse */
NV3007_DrawEllipse(120, 160, 80, 40, NV3007_COLOR_DODGERBLUE);
NV3007_DrawFilledEllipse(120, 160, 40, 80, NV3007_COLOR_GOLD);

/* Contorno y relleno de un arco (sector de anillo) de 0° a 90° */
NV3007_DrawArc(120, 160, 100, 60, 0.0f, 90.0f, NV3007_COLOR_TOMATO);
NV3007_DrawFilledArc(120, 160, 100, 60, 180.0f, 270.0f, NV3007_COLOR_LIMEGREEN);

/* Bitmap monocromo de 1 bit por píxel */
NV3007_DrawBitmap(10, 10, icon_16x16, 16, 16, NV3007_COLOR_WHITE);
```

> [!NOTE]
> Todas las primitivas recortan automáticamente contra los límites de la pantalla: lo que cae fuera se descarta silenciosamente (igual que en Arduino_GFX), sin devolver error. Usa `NV3007_GetWidth()` / `NV3007_GetHeight()` para conocer la geometría activa tras una rotación.

---

### 3. Texto en pantalla

```c
/* Un carácter */
NV3007_Putc(10, 20, 'A', &LCD_Font_11x18, NV3007_COLOR_WHITE, NV3007_COLOR_BLACK);

/* Una cadena con salto de línea.
 * '\n' avanza a la siguiente línea volviendo a x=10 (la posición x inicial).
 * La secuencia '\n'+'\r' avanza a la siguiente línea y reinicia a columna 0. */
NV3007_Puts(10, 20, "Hola STM32\nNV3007", &LCD_Font_11x18,
                 NV3007_COLOR_YELLOW, NV3007_COLOR_BLACK);

/* Calcular el ancho y alto de una cadena antes de dibujarla */
uint16_t w, h;
ILI9341_GetStringSize("Texto", &Font_11x18, &w, &h);
/* Centrar horizontalmente */
ILI9341_Puts((240 - w) / 2, 150, "Texto",
                 &Font_11x18,
                 ILI9341_COLOR_YELLOW, ILI9341_COLOR_BLACK);
```

| Función | Descripción |
|---------|-------------|
| `NV3007_Putc(x, y, c, font, fg, bg)` | Dibuja `c` en la celda de la esquina superior izquierda `(x, y)`. Los caracteres fuera de 32..126 se ignoran y devuelven `NV3007_OK` |
| `NV3007_Puts(x, y, str, font, fg, bg)` | Dibuja la cadena `str`. `'\n'` baja una línea (`FontHeight + 1` px) y vuelve a la columna `x`; `"\n\r"` vuelve a la columna 0; un `'\r'` suelto se ignora |
| `ILI9341_GetStringSize` | Calcula el ancho y alto de la cadena `str` con la fuente `font`. Devuelve 0 si `str` es `NULL` o vacía |

El tipo de fuente es `LCD_FontDef_t` (`lcd_fonts.h`) y las fuentes disponibles son `LCD_Font_7x10`, `LCD_Font_11x18` y `LCD_Font_16x26`. Como el ancho máximo de una fuente de este tipo es de 16 px, el driver rechaza con `NV3007_INVALID_PARAM` cualquier fuente más ancha.

> [!NOTE]
> El texto se dibuja de **celda completa opaca**: cada píxel de la celda se escribe, con el color de primer plano si el bit correspondiente de la fuente está a 1 y con el de fondo si está a 0. El fondo es por tanto obligatorio: la celda anterior se sobreescribe por completo.

> [!TIP]
> Son el equivalente de `ILI9341_Putc()` / `ILI9341_Puts()` de la librería `ILI9341_Disc1`, con la misma convención de cursor: si la celda no cabe en el ancho restante, `NV3007_Putc()` salta al inicio de la línea siguiente (`x = 0`, `y += FontHeight`).

---

### 4. Orientación y offsets de GRAM

```c
/* Rotar a la orientación nativa del cristal (142x428) */
NV3007_Rotate(NV3007_Orientation_Portrait_1);

/* Geometría lógica activa */
uint16_t w = NV3007_GetWidth();   /* 142 */
uint16_t h = NV3007_GetHeight();  /* 428 */
```

| Orientación | Rotación | Resolución activa | MADCTL | Offset del eje de 142 px |
|-------------|----------|-------------------|--------|---------------------------|
| `Portrait_1`  | 0°   | 142 × 428 | `0x00` | 12 (eje X) |
| `Portrait_2`  | 180° | 142 × 428 | `0xC0` | 14 (eje X) |
| `Landscape_1` | 90°  | 428 × 142 | `0x60` | 12 (eje Y) |
| `Landscape_2` | 270° | 428 × 142 | `0xA0` | 14 (eje Y) |

> [!NOTE]
> Usa siempre `NV3007_GetWidth()` / `NV3007_GetHeight()` y no `NV3007_WIDTH` / `NV3007_HEIGHT`: las constantes son la geometría en `Portrait` y quedan intercambiadas tras un `NV3007_Rotate()` a `Landscape`.

> [!NOTE]
> `NV3007_Rotate()` recarga los offsets desde la tabla, así que descarta cualquier valor puesto con `NV3007_SetOffset()`. Llama siempre a `NV3007_SetOffset()` **después** de `NV3007_Rotate()`.

---

## Calibración de offsets

La GRAM del NV3007 es de 168×428 y el cristal del panel solo expone 142 columnas: el área visible está recortada 26 columnas dentro de la GRAM (12 por un lado y 14 por el otro). El driver suma un offset a `CASET`/`RASET` en cada `NV3007_WriteAddrWindow()`.

> [!IMPORTANT]
> **Sin ese offset el panel no se rellena entero.** El driver escribe 142 columnas de la GRAM a partir de la columna 0, pero el cristal solo muestra las columnas 12…153: se pintan 12 columnas que quedan fuera del cristal y **se quedan sin pintar 12 columnas que sí son visibles**. De ahí la franja de color/"basura" en un borde de la pantalla.

El offset se aplica siempre sobre el **eje de 142 px**, que es el eje X lógico en `Portrait` (MV = 0) y el eje Y lógico en `Landscape` (MV = 1), porque con MV el controlador intercambia los ejes: el eje rápido (X lógico, 428 px) se direcciona con `ROWSET` y el lento (Y lógico, 142 px) con `COLSET`.

Los valores viven en `NV3007.h`, uno por orientación:

| Constante | Valor | Origen |
|-----------|-------|--------|
| `NV3007_OFFSET_P1_X` / `_P1_Y` | 12 / 0 | `Portrait_1` (MADCTL 0x00): margen en el lado natural del eje de 142 px |
| `NV3007_OFFSET_P2_X` / `_P2_Y` | 14 / 0 | `Portrait_2` (MADCTL 0xC0): eje de 142 px espejado → margen en el lado opuesto (168 − 142 − 12 = 14) |
| `NV3007_OFFSET_L1_X` / `_L1_Y` | 0 / 12 | `Landscape_1` (MADCTL 0x60): eje de 142 px sin espejar → 12 |
| `NV3007_OFFSET_L2_X` / `_L2_Y` | 0 / 14 | `Landscape_2` (MADCTL 0xA0): eje de 142 px espejado → 14 |

Valores confirmados en hardware sobre este mismo panel por tres implementaciones independientes: la demo del fabricante para el 2.79" (x_offset = 12), la librería `STM32_Lib_TFT_NV3007` (12/14 según la orientación) y el port de ESP-IDF `esp_lcd_nv3007` (12 en `Portrait`, 14 en `Landscape`).

Para ajustar sin recompilar cada intento, usa `NV3007_SetOffset()`:

```c
NV3007_Rotate(NV3007_Orientation_Landscape_1);
NV3007_SetOffset(0, 12);                 /* prueba y ajusta de 1 en 1 */

NV3007_FillScreen(NV3007_COLOR_BLACK);
NV3007_DrawRectangle(0, 0, NV3007_GetWidth(), NV3007_GetHeight(), NV3007_COLOR_WHITE);
```

El marco debe verse **completo y pegado al borde físico** del panel. Si falta un lado o queda un hueco, corrige el offset y repite. Cuando encaje, traslada los números a las constantes `NV3007_OFFSET_*` y vuelve a probar sin llamar a `NV3007_SetOffset()`.

> [!NOTE]
> Usa siempre `NV3007_GetWidth()` / `NV3007_GetHeight()` y no `NV3007_WIDTH` / `NV3007_HEIGHT`: las constantes son la geometría en `Portrait` y quedan intercambiadas tras un `NV3007_Rotate()` a `Landscape`.

> [!NOTE]
> `NV3007_Rotate()` recarga los offsets desde la tabla, así que descarta cualquier valor puesto con `NV3007_SetOffset()`. Llama siempre a `NV3007_SetOffset()` **después** de `NV3007_Rotate()`.

### Inversión de color

Este módulo monta un panel **IPS**, cuya polaridad de inversión va al revés que la de un TN. `NV3007.h` define `NV3007_IPS` a `0`, lo que hace que `NV3007_Init()` deje el panel con `INVOFF`: es la configuración validada en hardware sobre este módulo con la secuencia de inicialización de la cabecera (los colores salen correctos, no en negativo).

Si los colores salen **en negativo** (un fondo negro se ve blanco), define `NV3007_IPS` a `1` antes de incluir la cabecera, o cámbialo en `NV3007.h`.

---

## API Reference

### 1. Tipos de Datos

#### `NV3007_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería.

```c
typedef enum {
    NV3007_OK              = 0,  /**< Operación exitosa            */
    NV3007_ERROR           = 1,  /**< Fallo de transmisión SPI     */
    NV3007_TIMEOUT         = 2,  /**< Tiempo de espera HAL agotado */
    NV3007_NOT_INITIALIZED = 3,  /**< Driver no inicializado       */
    NV3007_INVALID_PARAM   = 4   /**< Parámetro inválido           */
} NV3007_Status_t;
```

| Valor | Código | Significado |
|-------|--------|-------------|
| `NV3007_OK` | 0 | Operación completada sin errores |
| `NV3007_ERROR` | 1 | Fallo de transmisión SPI |
| `NV3007_TIMEOUT` | 2 | Reservado: si `HAL_SPI_Transmit()` expira su timeout de 5000 ms, el driver devuelve `NV3007_ERROR` |
| `NV3007_NOT_INITIALIZED` | 3 | `NV3007_Init()` no se llamó o falló |
| `NV3007_INVALID_PARAM` | 4 | Puntero NULL, parámetro fuera de rango o configuración SPI inválida |

---

#### `NV3007_Orientation_t` - Orientaciones de Pantalla

```c
typedef enum {
    NV3007_Orientation_Portrait_1,    /**< Sin rotación          */
    NV3007_Orientation_Portrait_2,    /**< Rotación 180°         */
    NV3007_Orientation_Landscape_1,   /**< Rotación 90°          */
    NV3007_Orientation_Landscape_2    /**< Rotación 270° (-90°)  */
} NV3007_Orientation_t;
```

| Valor | Rotación | Resolución activa | MADCTL |
|-------|----------|-------------------|--------|
| `Portrait_1`  | 0°   | 142 × 428 | `0x00` |
| `Portrait_2`  | 180° | 142 × 428 | `0xC0` |
| `Landscape_1` | 90°  | 428 × 142 | `0x60` |
| `Landscape_2` | 270° | 428 × 142 | `0xA0` |

> [!NOTE]
> El orden de los valores debe mantenerse sincronizado con la tabla interna `NV3007_OffsetTable`, que se indexa con este enum. `NV3007_Rotate()` valida el valor recibido y devuelve `NV3007_INVALID_PARAM` si queda fuera de rango.

---

### 2. Funciones Públicas

#### `NV3007_Init()` - Inicialización del Driver

Inicializa el panel NV3007: valida los parámetros y la configuración del SPI, aplica el reset por hardware, reproduce la secuencia de inicialización proprietary del panel 2.79", configura `MADCTL` e inversión de color, y deja la pantalla en `NV3007_Orientation_Landscape_1` (428×142).

```c
NV3007_Status_t NV3007_Init(SPI_HandleTypeDef* hspi,
                             GPIO_TypeDef* CS_GPIOx,  uint16_t CS_Pin,
                             GPIO_TypeDef* DC_GPIOx,  uint16_t DC_Pin,
                             GPIO_TypeDef* RST_GPIOx, uint16_t RST_Pin);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `hspi` | `SPI_HandleTypeDef*` | Handle del periférico SPI (obligatorio) |
| `CS_GPIOx`, `CS_Pin` | `GPIO_TypeDef*`, `uint16_t` | Puerto/pin del pin CS del panel (obligatorio) |
| `DC_GPIOx`, `DC_Pin` | `GPIO_TypeDef*`, `uint16_t` | Puerto/pin del pin DC (comando/dato) del panel (obligatorio) |
| `RST_GPIOx`, `RST_Pin` | `GPIO_TypeDef*`, `uint16_t` | Puerto/pin del pin RESET del panel (obligatorio) |

**Retorna**: `NV3007_OK` si la inicialización fue exitosa; `NV3007_INVALID_PARAM` si algún puntero es NULL, algún pin es `0`, o la configuración del SPI no es válida (trama de 8 bits, MSB first y modo SPI 0 o 3); `NV3007_ERROR` si falla una transmisión SPI durante la secuencia de inicialización.

**Secuencia interna:**
1. Validación de parámetros y de la configuración del SPI (tamaño de trama, orden de bits y modo de reloj).
2. Registro de los pines CS/DC/RST indicados y `CS = HIGH`.
3. Reset por hardware: HIGH → 100 ms → LOW → 120 ms → HIGH → 120 ms.
4. Envío de `NV3007_InitTable` (117 registros) en una única transacción SPI, seguido de `SLPOUT` (220 ms) y `DISPON` (200 ms).
5. `MADCTL` explícito vía `NV3007_Rotate(NV3007_Orientation_Landscape_1)` (la secuencia del fabricante no incluye el comando `0x36`).
6. `NV3007_InvertDisplay(false)`, que envía `INVOFF` con el valor por defecto `NV3007_IPS = 0`.

---

#### `NV3007_DeInit()` - Desinicialización del Driver

Apaga la pantalla (`DISPOFF` + `SLPIN`) y libera los handles internos (SPI y pines GPIO). No libera ni reserva memoria: la memoria sigue siendo responsabilidad del usuario.

```c
NV3007_Status_t NV3007_DeInit(void);
```

**Retorna**: `NV3007_OK` si la desinicialización fue exitosa, `NV3007_NOT_INITIALIZED` si el driver no estaba inicializado.

---

#### `NV3007_GetWidth()` / `NV3007_GetHeight()` - Geometría Activa

Devuelven el ancho y el alto lógicos, ya intercambiados según la rotación en curso.

```c
uint16_t NV3007_GetWidth(void);
uint16_t NV3007_GetHeight(void);
```

> [!NOTE]
> Usa estas funciones y no las constantes `NV3007_WIDTH` / `NV3007_HEIGHT`: las constantes son la geometría en `Portrait` (142×428) y quedan intercambiadas tras un `NV3007_Rotate()` a `Landscape` (428×142).

---

#### `NV3007_Rotate()` - Rotar Pantalla

Envía el comando `MADCTL` al NV3007 y actualiza el ancho/alto lógicos, el flag de intercambio de ejes y los offsets de GRAM de la orientación. La geometría interna solo se actualiza si el comando SPI es exitoso.

```c
NV3007_Status_t NV3007_Rotate(NV3007_Orientation_t orientation);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `orientation` | `NV3007_Orientation_t` | Orientación deseada (en pasos de 90°) |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED` si el driver no está inicializado, `NV3007_INVALID_PARAM` si la orientación queda fuera de rango, `NV3007_ERROR` si falla la transmisión SPI.

> [!NOTE]
> Esta función **recarga los offsets de GRAM** desde la tabla interna, por lo que descarta cualquier valor previamente ajustado con `NV3007_SetOffset()`.

---

#### `NV3007_SetOffset()` - Ajustar Offset de GRAM

Sobrescribe en tiempo de ejecución el offset de GRAM aplicado a `CASET`/`RASET`. Pensado para calibrar el panel sin recompilar: se ajustan los valores hasta que la imagen encaja con el borde físico y luego se trasladan a las constantes `NV3007_OFFSET_*` de `NV3007.h`.

```c
NV3007_Status_t NV3007_SetOffset(uint16_t x_offset, uint16_t y_offset);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x_offset` | `uint16_t` | Desplazamiento aplicado al eje X lógico (sumado en `CASET`) |
| `y_offset` | `uint16_t` | Desplazamiento aplicado al eje Y lógico (sumado en `RASET`) |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED` si el driver no está inicializado.

> [!NOTE]
> El offset que hay que mover es el del **eje de 142 px**, porque es el único recortado dentro de la GRAM de 168 columnas. En `Portrait` ese eje es X; en `Landscape`, Y.

---

#### `NV3007_InvertDisplay()` - Inversión de Color

Activa o desactiva la inversión de color de la pantalla mediante `INVON` / `INVOFF`. La polaridad se invierte automáticamente según el valor de `NV3007_IPS` definido en `NV3007.h`.

```c
NV3007_Status_t NV3007_InvertDisplay(bool invert);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `invert` | `bool` | `true` para invertir los colores, `false` para restaurarlos |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED` si el driver no está inicializado, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_DisplayOn()` / `NV3007_DisplayOff()` - Encendido y Apagado

`NV3007_DisplayOn()` sale del modo Sleep (`SLPOUT`, con 220 ms de espera) y habilita la salida (`DISPON`). `NV3007_DisplayOff()` deshabilita la salida (`DISPOFF`) y entra en modo Sleep (`SLPIN`, con 120 ms de espera): el orden inverso evita el destello que produce cortar la alimentación con la salida de display todavía activa.

```c
NV3007_Status_t NV3007_DisplayOn(void);
NV3007_Status_t NV3007_DisplayOff(void);
```

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED` si el driver no está inicializado, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_WriteAddrWindow()` - Ventana de Direccionamiento

Establece la ventana de direccionamiento activa (`CASET`/`RASET`) para la siguiente escritura en RAM y abre la transacción (`RAMWR`). El offset de GRAM se suma a las coordenadas en el momento de emitir los comandos, de modo que el llamante sigue razonando en píxeles de pantalla.

```c
NV3007_Status_t NV3007_WriteAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x` | `uint16_t` | Columna inicial de la ventana |
| `y` | `uint16_t` | Fila inicial de la ventana |
| `w` | `uint16_t` | Ancho de la ventana en píxeles |
| `h` | `uint16_t` | Alto de la ventana en píxeles |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED`, `NV3007_INVALID_PARAM` si la ventana, con el offset sumado, sale de la GRAM, `NV3007_ERROR` si falla la transmisión SPI.

> [!IMPORTANT]
> En el camino exitoso **no se libera CS**: `RAMWR` y los datos de píxel que siguen deben viajar en una única transacción SPI con CS bajo; si CS sube entre medias, el NV3007 cierra el ciclo de escritura en RAM y descarta los píxeles. `NV3007_SendColor()` / `NV3007_FillColor()` (internas) son quienes liberan CS tras enviar los datos.

---

#### `NV3007_WritePixel()` / `NV3007_DrawPixel()` - Escribir Píxel

Escribe un píxel de color RGB565, recortándolo contra los límites de la pantalla. `NV3007_DrawPixel()` es un alias de `NV3007_WritePixel()`.

```c
NV3007_Status_t NV3007_WritePixel(int16_t x, int16_t y, uint16_t color);
NV3007_Status_t NV3007_DrawPixel(int16_t x, int16_t y, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x` | `int16_t` | Columna del píxel |
| `y` | `int16_t` | Fila del píxel |
| `color` | `uint16_t` | Color en formato RGB565 |

**Retorna**: `NV3007_OK` (también si el píxel cae fuera de pantalla, en cuyo caso se descarta silenciosamente), `NV3007_NOT_INITIALIZED`, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_FillScreen()` - Rellenar Pantalla

Rellena toda la pantalla con un color sólido. Equivale a `NV3007_DrawFilledRectangle()` sobre el área completa.

```c
NV3007_Status_t NV3007_FillScreen(uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `color` | `uint16_t` | Color de relleno en formato RGB565 |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED`, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_DrawLine()` - Dibujar Línea

Dibuja una línea entre dos puntos. Las líneas horizontales y verticales se delegan en las variantes rápidas (`NV3007_DrawFastHLine()` / `NV3007_DrawFastVLine()`); el resto usa el algoritmo de Bresenham. Las coordenadas fuera de rango se recortan automáticamente.

```c
NV3007_Status_t NV3007_DrawLine(int16_t x0, int16_t y0,
                                int16_t x1, int16_t y1,
                                uint16_t color);
```

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED`, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_DrawFastVLine()` / `NV3007_DrawFastHLine()` - Líneas Rápidas

Dibujan una línea vertical u horizontal sin pasar por el algoritmo de Bresenham: emiten una única ventana de direccionamiento y una ráfaga de datos. El alto/ancho puede pasarse como negativo; en ese caso se normaliza invirtiendo el punto de partida. Las coordenadas y longitudes se recortan automáticamente.

```c
NV3007_Status_t NV3007_DrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
NV3007_Status_t NV3007_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x`, `y` | `int16_t` | Punto de inicio de la línea |
| `h` | `int16_t` | Alto de la línea, en `DrawFastVLine` (negativo se normaliza) |
| `w` | `int16_t` | Ancho de la línea, en `DrawFastHLine` (negativo se normaliza) |
| `color` | `uint16_t` | Color RGB565 |

**Retorna**: `NV3007_OK` (incluido el caso `h`/`w == 0`, que no dibuja nada), `NV3007_NOT_INITIALIZED`, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_DrawRectangle()` - Dibujar Rectángulo

Dibuja el contorno de un rectángulo definido por su esquina superior izquierda `(x, y)` y su tamaño `(w, h)`.

```c
NV3007_Status_t NV3007_DrawRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
```

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED`, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_WriteFilledRectangle()` / `NV3007_DrawFilledRectangle()` - Rectángulo Relleno

Rellena un rectángulo con un color sólido. Acepta `w`/`h` negativos (se normalizan moviendo la esquina) y descarta los rectángulos completamente fuera de pantalla. `NV3007_DrawFilledRectangle()` es un alias de `NV3007_WriteFilledRectangle()`.

```c
NV3007_Status_t NV3007_WriteFilledRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
NV3007_Status_t NV3007_DrawFilledRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x`, `y` | `int16_t` | Esquina superior izquierda del rectángulo |
| `w` | `int16_t` | Ancho del rectángulo (negativo = hacia la izquierda de la esquina) |
| `h` | `int16_t` | Alto del rectángulo (negativo = hacia arriba de la esquina) |
| `color` | `uint16_t` | Color de relleno en formato RGB565 |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED`, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_DrawCircle()` / `NV3007_DrawFilledCircle()` - Círculos

`NV3007_DrawCircle()` dibuja el contorno de un círculo con el algoritmo de punto medio de Bresenham. `NV3007_DrawFilledCircle()` lo rellena con franjas verticales.

```c
NV3007_Status_t NV3007_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
NV3007_Status_t NV3007_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `int16_t` | Centro del círculo |
| `r` | `int16_t` | Radio en píxeles |
| `color` | `uint16_t` | Color RGB565 |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED`, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_DrawTriangle()` / `NV3007_DrawFilledTriangle()` - Triángulos

`NV3007_DrawTriangle()` traza las tres aristas del triángulo delegando en `NV3007_DrawLine()`. `NV3007_DrawFilledTriangle()` lo rellena con un barrido de líneas horizontales: los vértices se ordenan por coordenada Y y los bordes se interpolan con aritmética entera. Los triángulos degenerados (los tres vértices en la misma fila) se reducen a un tramo horizontal.

```c
NV3007_Status_t NV3007_DrawTriangle(int16_t x0, int16_t y0,
                                    int16_t x1, int16_t y1,
                                    int16_t x2, int16_t y2,
                                    uint16_t color);
NV3007_Status_t NV3007_DrawFilledTriangle(int16_t x0, int16_t y0,
                                           int16_t x1, int16_t y1,
                                           int16_t x2, int16_t y2,
                                           uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `int16_t` | Primer vértice |
| `x1`, `y1` | `int16_t` | Segundo vértice |
| `x2`, `y2` | `int16_t` | Tercer vértice |
| `color` | `uint16_t` | Color RGB565 |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED`, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_DrawEllipse()` / `NV3007_DrawFilledEllipse()` - Elipses

`NV3007_DrawEllipse()` traza el contorno de una elipse con el algoritmo de punto medio de Zingl (variante entera), recortando cada píxel a los límites de la pantalla. `NV3007_DrawFilledEllipse()` rellena cada fila visitada con un tramo horizontal completo, lo que reduce el coste a ~2 × `ry` transacciones SPI.

Los casos degenerados (`rx == 0` o `ry == 0`) se delegan en `NV3007_DrawFastVLine()` / `NV3007_DrawFastHLine()`.

```c
NV3007_Status_t NV3007_DrawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color);
NV3007_Status_t NV3007_DrawFilledEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `int16_t` | Centro de la elipse |
| `rx` | `int16_t` | Semieje horizontal en píxeles (no negativo) |
| `ry` | `int16_t` | Semieje vertical en píxeles (no negativo) |
| `color` | `uint16_t` | Color RGB565 |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED`, `NV3007_INVALID_PARAM` si `rx` o `ry` son negativos, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_DrawArc()` / `NV3007_DrawFilledArc()` - Arcos

`NV3007_DrawArc()` traza las cuatro aristas que delimitan un sector de anillo: la curva exterior de radio `r1`, la interior de radio `r2` y las dos rectas radiales que unen ambas en los ángulos `start` y `end`. `NV3007_DrawFilledArc()` rellena ese mismo sector.

```c
NV3007_Status_t NV3007_DrawArc(int16_t x0, int16_t y0, int16_t r1, int16_t r2, float start, float end, uint16_t color);
NV3007_Status_t NV3007_DrawFilledArc(int16_t x0, int16_t y0, int16_t r1, int16_t r2, float start, float end, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `int16_t` | Centro del arco |
| `r1` | `int16_t` | Radio exterior (si se pasa menor que `r2`, se intercambian) |
| `r2` | `int16_t` | Radio interior (un radio de `0` se eleva a `1`) |
| `start` | `float` | Ángulo inicial en grados (0° = derecha, sentido horario) |
| `end` | `float` | Ángulo final en grados |
| `color` | `uint16_t` | Color RGB565 |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED`, `NV3007_INVALID_PARAM` si `r1` o `r2` son negativos, `NV3007_ERROR` si falla la transmisión SPI.

> [!NOTE]
> Los ángulos van en grados, con **0° en la derecha** y creciendo en sentido **horario** (hacia abajo). Si `end < start` el barrido pasa por 0° (por ejemplo, `350° → 10°` son 20°). Si `start` y `end` coinciden tras normalizar pero los valores originales no (caso `0° → 360°`), se dibuja el anillo completo. Si `start == end` el sector degenera y solo se traza la arista radial.

> [!TIP]
> Son el equivalente de `ILI9341_DrawArc()` / `ILI9341_DrawFilledArc()` de la librería `ILI9341_Disc1`, con la misma convención de ángulos, pero con la geometría corregida: el algoritmo heredado no dibuja los barridos de más de 180° ni el arco completo.

---

#### `NV3007_DrawRoundRect()` / `NV3007_DrawFilledRoundRect()` - Rectángulos Redondeados

`NV3007_DrawRoundRect()` dibuja el contorno de un rectángulo combinando los lados rectos con cuatro cuadrantes de círculo en las esquinas. `NV3007_DrawFilledRoundRect()` lo rellena combinando un rectángulo central con los cuadrantes de círculo relleno. El radio se recorta automáticamente a la mitad del lado menor.

```c
NV3007_Status_t NV3007_DrawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
NV3007_Status_t NV3007_DrawFilledRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `int16_t` | Esquina superior izquierda |
| `w`, `h` | `int16_t` | Ancho y alto del rectángulo |
| `radius` | `int16_t` | Radio de las esquinas en píxeles |
| `color` | `uint16_t` | Color RGB565 |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED`, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_DrawBitmap()` - Dibujar Bitmap

Dibuja un bitmap monocromo de 1 bit por píxel residente en memoria, usando el color indicado para los bits activos y descartando los bits inactivos (transparentes). Cada fila del bitmap se rellena a byte completo y los bits se leen de MSB a LSB.

```c
NV3007_Status_t NV3007_DrawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x`, `y` | `int16_t` | Esquina superior izquierda del bitmap |
| `bitmap` | `const uint8_t[]` | Bitmap monocromo (MSB primero, filas rellenadas a byte completo) |
| `w`, `h` | `int16_t` | Ancho y alto del bitmap en píxeles |
| `color` | `uint16_t` | Color RGB565 de los bits activos |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED`, `NV3007_ERROR` si falla la transmisión SPI.

---

#### `NV3007_Putc()` - Renderizar Carácter

Renderiza un carácter ASCII imprimible (32..126) a partir de una definición de fuente `LCD_FontDef_t`. Abre una ventana del tamaño de la celda y vuelca el glifo fila a fila dentro de una única transacción `RAMWR` (CS bajo), en lugar de un píxel por píxel.

```c
NV3007_Status_t NV3007_Putc(uint16_t x, uint16_t y, char c,
                            LCD_FontDef_t* font,
                            uint16_t foreground, uint16_t background);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x`, `y` | `uint16_t` | Esquina superior izquierda de la celda |
| `c` | `char` | Carácter ASCII a renderizar (rango `' '` – `'~'`) |
| `font` | `LCD_FontDef_t*` | Puntero a la definición de la fuente (`lcd_fonts.h`) |
| `foreground` | `uint16_t` | Color de los píxeles activos (RGB565) |
| `background` | `uint16_t` | Color del fondo de la celda (RGB565) |

**Retorna**: `NV3007_OK` (los caracteres fuera de 32..126 se ignoran y devuelven `NV3007_OK`), `NV3007_NOT_INITIALIZED`, `NV3007_INVALID_PARAM` si `font` es NULL, tiene un ancho mayor de 16 px o la celda no cabe en la pantalla, `NV3007_ERROR` si falla la transmisión SPI.

> [!NOTE]
> Si la celda no cabe en el ancho restante, la función salta al inicio de la línea siguiente (`x = 0`, `y += FontHeight`). Tras el dibujo, el cursor de texto interno avanza `FontWidth` columnas.

---

#### `NV3007_Puts()` - Renderizar Cadena

Renderiza una cadena terminada en nulo. Interpreta `'\n'` como salto de línea retornando a la posición `x` inicial con la que se llamó a `Puts()`. La secuencia `'\n'`+`'\r'` avanza a la siguiente línea y reinicia la columna a 0. Un `'\r'` aislado es ignorado.

```c
NV3007_Status_t NV3007_Puts(uint16_t x, uint16_t y, const char* str,
                            LCD_FontDef_t* font,
                            uint16_t foreground, uint16_t background);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x`, `y` | `uint16_t` | Esquina superior izquierda del primer carácter |
| `str` | `const char*` | Cadena terminada en nulo |
| `font` | `LCD_FontDef_t*` | Puntero a la definición de la fuente (`lcd_fonts.h`) |
| `foreground` | `uint16_t` | Color del texto (RGB565) |
| `background` | `uint16_t` | Color de fondo de las celdas (RGB565) |

**Retorna**: `NV3007_OK`, `NV3007_NOT_INITIALIZED`, `NV3007_INVALID_PARAM` si `str` o `font` son NULL, `NV3007_ERROR` si falla la transmisión SPI (se detiene en el primer carácter que falle).

---

#### `NV3007_GetStringSize` - Ancho de Cadena

Calcula el ancho en píxeles que ocupará una cadena al renderizarla con la fuente indicada. La altura de la cadena es siempre `FontHeight` de la fuente.

```c
void NV3007_GetStringSize(char* str, LCD_FontDef_t* font, uint16_t* width, uint16_t* height);
``` 

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `str` | `char*` | Cadena terminada en nulo |
| `font` | `LCD_FontDef_t*` | Definición de la fuente |
| `width` | `uint16_t*` | Ancho total en píxeles (salida) |
| `height` | `uint16_t*` | Alto total en píxeles (salida) = `font->FontHeight` |

---

## Colores Predefinidos

La librería incluye **147 macros de colores** en formato **RGB565**, generadas a partir de la paleta estándar de nombres de color **X11/CSS** (p. ej. `NV3007_COLOR_CORNFLOWERBLUE`, `NV3007_COLOR_MEDIUMSEAGREEN`, `NV3007_COLOR_DARKSLATEGRAY`). Todas se definen internamente con la macro `RGB565()`.

```c
/* Convierte componentes R, G, B de 8 bits (0-255) a un color RGB565 de 16 bits */
#define RGB565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

/* Expande un color RGB565 de 16 bits de vuelta a RGB888 de 24 bits (con pérdida de precisión) */
#define RGB16TO24(c) ((((uint32_t)(c) & 0xF800) << 8) | (((c) & 0x07E0) << 5) | (((c) & 0x1F) << 3))
```

<details>
<summary>Ver los 147 colores predefinidos</summary>

| Macro | Valor RGB565 |
|-------|--------------|
| `NV3007_COLOR_ALICEBLUE` | `0xF7DF` |
| `NV3007_COLOR_ANTIQUEWHITE` | `0xFF7B` |
| `NV3007_COLOR_AQUA` | `0x07FF` |
| `NV3007_COLOR_AQUAMARINE` | `0x87FB` |
| `NV3007_COLOR_AZURE` | `0xF7FF` |
| `NV3007_COLOR_BEIGE` | `0xFFBC` |
| `NV3007_COLOR_BISQUE` | `0xFF39` |
| `NV3007_COLOR_BLACK` | `0x0000` |
| `NV3007_COLOR_BLANCHEDALMOND` | `0xFF7A` |
| `NV3007_COLOR_BLUE` | `0x001F` |
| `NV3007_COLOR_BLUEVIOLET` | `0x897C` |
| `NV3007_COLOR_BROWN` | `0xA965` |
| `NV3007_COLOR_BURLYWOOD` | `0xE5D1` |
| `NV3007_COLOR_CADETBLUE` | `0x6514` |
| `NV3007_COLOR_CHARTREUSE` | `0x87E0` |
| `NV3007_COLOR_CHOCOLATE` | `0xD344` |
| `NV3007_COLOR_CORAL` | `0xFC0A` |
| `NV3007_COLOR_CORNFLOWERBLUE` | `0x6CBE` |
| `NV3007_COLOR_CORNSILK` | `0xFFDC` |
| `NV3007_COLOR_CRIMSON` | `0xE0A8` |
| `NV3007_COLOR_CYAN` | `0x07FF` |
| `NV3007_COLOR_DARKBLUE` | `0x0011` |
| `NV3007_COLOR_DARKCYAN` | `0x0471` |
| `NV3007_COLOR_DARKGOLDENROD` | `0xBC41` |
| `NV3007_COLOR_DARKGRAY` | `0xAD55` |
| `NV3007_COLOR_DARKGREEN` | `0x0320` |
| `NV3007_COLOR_DARKGREY` | `0xAD55` |
| `NV3007_COLOR_DARKKHAKI` | `0xC5CD` |
| `NV3007_COLOR_DARKMAGENTA` | `0x8811` |
| `NV3007_COLOR_DARKOLIVEGREEN` | `0x5B66` |
| `NV3007_COLOR_DARKORANGE` | `0xFC60` |
| `NV3007_COLOR_DARKORCHID` | `0x99BA` |
| `NV3007_COLOR_DARKRED` | `0x8800` |
| `NV3007_COLOR_DARKSALMON` | `0xECCF` |
| `NV3007_COLOR_DARKSEAGREEN` | `0x95F2` |
| `NV3007_COLOR_DARKSLATEBLUE` | `0x49F1` |
| `NV3007_COLOR_DARKSLATEGRAY` | `0x328A` |
| `NV3007_COLOR_DARKSLATEGREY` | `0x328A` |
| `NV3007_COLOR_DARKTURQUOISE` | `0x069A` |
| `NV3007_COLOR_DARKVIOLET` | `0x981A` |
| `NV3007_COLOR_DEEPPINK` | `0xF8B2` |
| `NV3007_COLOR_DEEPSKYBLUE` | `0x061F` |
| `NV3007_COLOR_DIMGRAY` | `0x6B4D` |
| `NV3007_COLOR_DIMGREY` | `0x6B4D` |
| `NV3007_COLOR_DODGERBLUE` | `0x249F` |
| `NV3007_COLOR_FIREBRICK` | `0xB124` |
| `NV3007_COLOR_FLORALWHITE` | `0xFFFE` |
| `NV3007_COLOR_FORESTGREEN` | `0x2464` |
| `NV3007_COLOR_FUCHSIA` | `0xF81F` |
| `NV3007_COLOR_GAINSBORO` | `0xE6FC` |
| `NV3007_COLOR_GHOSTWHITE` | `0xFFDF` |
| `NV3007_COLOR_GOLD` | `0xFEC0` |
| `NV3007_COLOR_GOLDENROD` | `0xDD24` |
| `NV3007_COLOR_GRAY` | `0x8410` |
| `NV3007_COLOR_GREEN` | `0x0400` |
| `NV3007_COLOR_GREENYELLOW` | `0xB7E6` |
| `NV3007_COLOR_GREY` | `0x8410` |
| `NV3007_COLOR_HONEYDEW` | `0xF7FE` |
| `NV3007_COLOR_HOTPINK` | `0xFB57` |
| `NV3007_COLOR_INDIANRED` | `0xD2EC` |
| `NV3007_COLOR_INDIGO` | `0x4810` |
| `NV3007_COLOR_IVORY` | `0xFFFE` |
| `NV3007_COLOR_KHAKI` | `0xF752` |
| `NV3007_COLOR_LAVENDER` | `0xEF5F` |
| `NV3007_COLOR_LAVENDERBLUSH` | `0xFF9F` |
| `NV3007_COLOR_LAWNGREEN` | `0x87E0` |
| `NV3007_COLOR_LEMONCHIFFON` | `0xFFFA` |
| `NV3007_COLOR_LIGHTBLUE` | `0xB6DD` |
| `NV3007_COLOR_LIGHTCORAL` | `0xF410` |
| `NV3007_COLOR_LIGHTCYAN` | `0xE7FF` |
| `NV3007_COLOR_LIGHTGOLDENRODYELLOW` | `0xFFFA` |
| `NV3007_COLOR_LIGHTGRAY` | `0xD6BA` |
| `NV3007_COLOR_LIGHTGREEN` | `0x9792` |
| `NV3007_COLOR_LIGHTGREY` | `0xD6BA` |
| `NV3007_COLOR_LIGHTPINK` | `0xFDD8` |
| `NV3007_COLOR_LIGHTSALMON` | `0xFD0F` |
| `NV3007_COLOR_LIGHTSEAGREEN` | `0x25B5` |
| `NV3007_COLOR_LIGHTSKYBLUE` | `0x8E9F` |
| `NV3007_COLOR_LIGHTSLATEGRAY` | `0x7C53` |
| `NV3007_COLOR_LIGHTSLATEGREY` | `0x7C53` |
| `NV3007_COLOR_LIGHTSTEELBLUE` | `0xB63C` |
| `NV3007_COLOR_LIGHTYELLOW` | `0xFFFC` |
| `NV3007_COLOR_LIME` | `0x07E0` |
| `NV3007_COLOR_LIMEGREEN` | `0x3666` |
| `NV3007_COLOR_LINEN` | `0xFF9D` |
| `NV3007_COLOR_MAGENTA` | `0xF81F` |
| `NV3007_COLOR_MAROON` | `0x8000` |
| `NV3007_COLOR_MEDIUMAQUAMARINE` | `0x6E75` |
| `NV3007_COLOR_MEDIUMBLUE` | `0x001A` |
| `NV3007_COLOR_MEDIUMORCHID` | `0xBABA` |
| `NV3007_COLOR_MEDIUMPURPLE` | `0x939B` |
| `NV3007_COLOR_MEDIUMSEAGREEN` | `0x45AE` |
| `NV3007_COLOR_MEDIUMSLATEBLUE` | `0x7B5E` |
| `NV3007_COLOR_MEDIUMSPRINGGREEN` | `0x07F3` |
| `NV3007_COLOR_MEDIUMTURQUOISE` | `0x4E9A` |
| `NV3007_COLOR_MEDIUMVIOLETRED` | `0xC8B1` |
| `NV3007_COLOR_MIDNIGHTBLUE` | `0x18CE` |
| `NV3007_COLOR_MINTCREAM` | `0xFFFF` |
| `NV3007_COLOR_MISTYROSE` | `0xFF3C` |
| `NV3007_COLOR_MOCCASIN` | `0xFF37` |
| `NV3007_COLOR_NAVAJOWHITE` | `0xFF16` |
| `NV3007_COLOR_NAVY` | `0x0010` |
| `NV3007_COLOR_OLDLACE` | `0xFFBD` |
| `NV3007_COLOR_OLIVE` | `0x8400` |
| `NV3007_COLOR_OLIVEDRAB` | `0x6C84` |
| `NV3007_COLOR_ORANGE` | `0xFD20` |
| `NV3007_COLOR_ORANGERED` | `0xFA20` |
| `NV3007_COLOR_ORCHID` | `0xDB9B` |
| `NV3007_COLOR_PALEGOLDENROD` | `0xF755` |
| `NV3007_COLOR_PALEGREEN` | `0x9FF3` |
| `NV3007_COLOR_PALETURQUOISE` | `0xB79E` |
| `NV3007_COLOR_PALEVIOLETRED` | `0xDB92` |
| `NV3007_COLOR_PAPAYAWHIP` | `0xFF9B` |
| `NV3007_COLOR_PEACHPUFF` | `0xFEF7` |
| `NV3007_COLOR_PERU` | `0xD428` |
| `NV3007_COLOR_PINK` | `0xFE19` |
| `NV3007_COLOR_PLUM` | `0xE51C` |
| `NV3007_COLOR_POWDERBLUE` | `0xB71D` |
| `NV3007_COLOR_PURPLE` | `0x8010` |
| `NV3007_COLOR_RED` | `0xF800` |
| `NV3007_COLOR_ROSYBROWN` | `0xC492` |
| `NV3007_COLOR_ROYALBLUE` | `0x435C` |
| `NV3007_COLOR_SADDLEBROWN` | `0x8A22` |
| `NV3007_COLOR_SALMON` | `0xFC0E` |
| `NV3007_COLOR_SANDYBROWN` | `0xFD2C` |
| `NV3007_COLOR_SEAGREEN` | `0x346B` |
| `NV3007_COLOR_SEASHELL` | `0xFFBE` |
| `NV3007_COLOR_SIENNA` | `0xA2A6` |
| `NV3007_COLOR_SILVER` | `0xC618` |
| `NV3007_COLOR_SKYBLUE` | `0x8E9D` |
| `NV3007_COLOR_SLATEBLUE` | `0x6AFA` |
| `NV3007_COLOR_SLATEGRAY` | `0x7412` |
| `NV3007_COLOR_SLATEGREY` | `0x7412` |
| `NV3007_COLOR_SNOW` | `0xFFFF` |
| `NV3007_COLOR_SPRINGGREEN` | `0x07F0` |
| `NV3007_COLOR_STEELBLUE` | `0x4C37` |
| `NV3007_COLOR_TAN` | `0xD5B2` |
| `NV3007_COLOR_TEAL` | `0x0410` |
| `NV3007_COLOR_THISTLE` | `0xDE1B` |
| `NV3007_COLOR_TOMATO` | `0xFB29` |
| `NV3007_COLOR_TURQUOISE` | `0x471A` |
| `NV3007_COLOR_VIOLET` | `0xF43E` |
| `NV3007_COLOR_WHEAT` | `0xFF16` |
| `NV3007_COLOR_WHITE` | `0xFFFF` |
| `NV3007_COLOR_WHITESMOKE` | `0xFFBF` |
| `NV3007_COLOR_YELLOW` | `0xFFE0` |
| `NV3007_COLOR_YELLOWGREEN` | `0x9E66` |

</details>

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](../../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [1.0.0] - 19-10-2026

Primera versión estable: añade elipses, arcos y texto, y corrige el direccionamiento de la GRAM en `Landscape`. Respecto a `0.1.0` incluye **cambios incompatibles** en los valores por defecto (orientación inicial, inversión de color y offsets) y una nueva dependencia de `lcd_fonts.h`.

#### Added
- **Elipses**: `NV3007_DrawEllipse()` traza el contorno con el algoritmo de punto medio de Zingl y `NV3007_DrawFilledEllipse()` la rellena con un tramo horizontal por fila. Un semieje a 0 degenera en una línea.
- **Arcos**: `NV3007_DrawArc()` traza el contorno de un sector de anillo (curvas de radio `r1` y `r2` y las dos aristas radiales) y `NV3007_DrawFilledArc()` lo rellena. Ángulos en grados, 0° a la derecha y sentido horario; `0° → 360°` dibuja el anillo completo.
- **Texto**: `NV3007_Putc()` y `NV3007_Puts()` renderizan caracteres ASCII 32..126 con las fuentes de `lcd_fonts.h` (`LCD_Font_7x10`, `LCD_Font_11x18`, `LCD_Font_16x26`), con fondo opaco y salto de línea automático al llegar al borde.

#### Changed
- **[BREAKING]** `NV3007_Init()` deja la pantalla en `NV3007_Orientation_Landscape_1` (428×142) en lugar de `Portrait_1` (142×428). Para conservar el comportamiento anterior, llama a `NV3007_Rotate(NV3007_Orientation_Portrait_1)` tras la inicialización.
- **[BREAKING]** `NV3007_IPS` pasa de `1` a `0` por defecto: `NV3007_Init()` envía `INVOFF`, validado en hardware sobre este módulo. Si los colores salen en negativo, define `NV3007_IPS` a `1`.
- **[BREAKING]** `NV3007.h` incluye `lcd_fonts.h`: hay que copiar `lcd_fonts.h` y `lcd_fonts.c` (de `Librerias/ILI9341_Disc1/`) al proyecto o no compila.
- Offsets de `Landscape`: `NV3007_OFFSET_L1_Y` pasa de 14 a 12 y `NV3007_OFFSET_L2_Y` de 12 a 14. Los cuatro pares de offsets quedan confirmados en hardware.
- `NV3007_WriteAddrWindow()` devuelve `NV3007_INVALID_PARAM` si la ventana, con el offset sumado, sale de la GRAM, en lugar de dejar que los contadores de dirección den la vuelta.

#### Fixed
- Direccionamiento en orientaciones `Landscape` (MV = 1): los límites de `CASET`/`RASET` se intercambian con los ejes, así que la imagen ya no sale corrida ni troceada.
- Primitivas grandes o parcialmente fuera de pantalla: el recorte se hace con coordenadas de 32 bits y ya no desborda.
