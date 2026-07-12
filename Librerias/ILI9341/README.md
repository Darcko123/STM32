# Librería genérica para la pantalla TFT LCD ILI9341 (STM32)

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32%20(HAL)-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
[![Version](https://img.shields.io/badge/Version-2.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/ILI9341)
[![Protocol](https://img.shields.io/badge/Protocol-SPI%20%2B%20I2C%20%2B%20DMA2D%20%2B%20SPI--DMA-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/ILI9341)

---

## Tabla de Contenidos
- [Librería genérica para la pantalla TFT LCD ILI9341 (STM32)](#librería-genérica-para-la-pantalla-tft-lcd-ili9341-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Bus SPI (LCD ILI9341)](#bus-spi-lcd-ili9341)
    - [GPIO de Control (LCD ILI9341)](#gpio-de-control-lcd-ili9341)
    - [Bus I2C (Panel táctil STMPE811, opcional)](#bus-i2c-panel-táctil-stmpe811-opcional)
  - [Configuración SPI](#configuración-spi)
  - [Configuración I2C](#configuración-i2c)
  - [Configuración DMA2D (opcional)](#configuración-dma2d-opcional)
  - [Configuración DMA SPI TX (para DisplayImage y Flush)](#configuración-dma-spi-tx-para-displayimage-y-flush)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Dibujo en pantalla](#2-dibujo-en-pantalla)
    - [3. Texto en pantalla](#3-texto-en-pantalla)
    - [4. Frame buffer fuera de pantalla (RAM)](#4-frame-buffer-fuera-de-pantalla-ram)
    - [5. Doble buffer con pipelining DMA](#5-doble-buffer-con-pipelining-dma)
    - [6. Panel táctil](#6-panel-táctil)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`ILI9341_Status_t` - Estados de Retorno](#ili9341_status_t---estados-de-retorno)
      - [`ILI9341_Orientation_t` - Orientaciones de Pantalla](#ili9341_orientation_t---orientaciones-de-pantalla)
      - [`ILI9341_TextAlign_t` - Alineación de Texto](#ili9341_textalign_t---alineación-de-texto)
      - [`TP_STATE` - Estado del Panel Táctil](#tp_state---estado-del-panel-táctil)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`ILI9341_Init()` - Inicialización del Driver](#ili9341_init---inicialización-del-driver)
      - [`ILI9341_DeInit()` - Desinicialización del Driver](#ili9341_deinit---desinicialización-del-driver)
      - [`ILI9341_Color565()` - Convertir RGB888 a RGB565](#ili9341_color565---convertir-rgb888-a-rgb565)
      - [`ILI9341_Fill()` - Rellenar Pantalla](#ili9341_fill---rellenar-pantalla)
      - [`ILI9341_Rotate()` - Rotar Pantalla](#ili9341_rotate---rotar-pantalla)
      - [`ILI9341_DrawPixel()` - Dibujar Píxel](#ili9341_drawpixel---dibujar-píxel)
      - [`ILI9341_DrawLine()` - Dibujar Línea](#ili9341_drawline---dibujar-línea)
      - [`ILI9341_DrawThickLine()` - Dibujar Línea con Grosor](#ili9341_drawthickline---dibujar-línea-con-grosor)
      - [`ILI9341_DrawFastVLine()` / `ILI9341_DrawFastHLine()` - Líneas Rápidas](#ili9341_drawfastvline--ili9341_drawfasthline---líneas-rápidas)
      - [`ILI9341_DrawRectangle()` - Dibujar Rectángulo](#ili9341_drawrectangle---dibujar-rectángulo)
      - [`ILI9341_DrawFilledRectangle()` - Dibujar Rectángulo Relleno](#ili9341_drawfilledrectangle---dibujar-rectángulo-relleno)
      - [`ILI9341_DrawRoundRect()` - Dibujar Rectángulo con Esquinas Redondeadas](#ili9341_drawroundrect---dibujar-rectángulo-con-esquinas-redondeadas)
      - [`ILI9341_DrawFilledRoundRect()` - Dibujar Rectángulo Redondeado Relleno](#ili9341_drawfilledroundrect---dibujar-rectángulo-redondeado-relleno)
      - [`ILI9341_DrawCircle()` - Dibujar Círculo](#ili9341_drawcircle---dibujar-círculo)
      - [`ILI9341_DrawFilledCircle()` - Dibujar Círculo Relleno](#ili9341_drawfilledcircle---dibujar-círculo-relleno)
      - [`ILI9341_DrawTriangle()` - Dibujar Triángulo](#ili9341_drawtriangle---dibujar-triángulo)
      - [`ILI9341_DrawFilledTriangle()` - Dibujar Triángulo Relleno](#ili9341_drawfilledtriangle---dibujar-triángulo-relleno)
      - [`ILI9341_DrawEllipse()` - Dibujar Elipse](#ili9341_drawellipse---dibujar-elipse)
      - [`ILI9341_DrawFilledEllipse()` - Dibujar Elipse Rellena](#ili9341_drawfilledellipse---dibujar-elipse-rellena)
      - [`ILI9341_DrawArc()` - Dibujar Arco](#ili9341_drawarc---dibujar-arco)
      - [`ILI9341_DrawFilledArc()` - Dibujar Arco Relleno](#ili9341_drawfilledarc---dibujar-arco-relleno)
      - [`ILI9341_Putc()` - Renderizar Carácter](#ili9341_putc---renderizar-carácter)
      - [`ILI9341_Puts()` - Renderizar Cadena](#ili9341_puts---renderizar-cadena)
      - [`ILI9341_Printf()` - Renderizar Texto Formateado](#ili9341_printf---renderizar-texto-formateado)
      - [`ILI9341_GetStringSize()` - Calcular Tamaño de Cadena](#ili9341_getstringsize---calcular-tamaño-de-cadena)
      - [`ILI9341_PutsAligned()` / `ILI9341_PrintfAligned()` - Texto Alineado Horizontalmente](#ili9341_putsaligned--ili9341_printfaligned---texto-alineado-horizontalmente)
      - [`ILI9341_DisplayImage()` - Transferir Frame Buffer](#ili9341_displayimage---transferir-frame-buffer)
      - [Funciones de Frame Buffer fuera de Pantalla](#funciones-de-frame-buffer-fuera-de-pantalla)
      - [`ILI9341_Printf_ImageBuffer()` - Texto Formateado en Buffer](#ili9341_printf_imagebuffer---texto-formateado-en-buffer)
      - [`ILI9341_PutsAligned_ImageBuffer()` / `ILI9341_PrintfAligned_ImageBuffer()` - Texto Alineado en Buffer](#ili9341_putsaligned_imagebuffer--ili9341_printfaligned_imagebuffer---texto-alineado-en-buffer)
      - [`ILI9341_DrawThickLine_ImageBuffer()` - Línea con Grosor en Buffer](#ili9341_drawthickline_imagebuffer---línea-con-grosor-en-buffer)
      - [`ILI9341_DrawRoundRect_ImageBuffer()` - Rectángulo Redondeado en Buffer](#ili9341_drawroundrect_imagebuffer---rectángulo-redondeado-en-buffer)
      - [`ILI9341_DrawFilledRoundRect_ImageBuffer()` - Rectángulo Redondeado Relleno en Buffer](#ili9341_drawfilledroundrect_imagebuffer---rectángulo-redondeado-relleno-en-buffer)
      - [`ILI9341_DrawCircle_ImageBuffer()` - Círculo en Buffer](#ili9341_drawcircle_imagebuffer---círculo-en-buffer)
      - [`ILI9341_DrawFilledCircle_ImageBuffer()` - Círculo Relleno en Buffer](#ili9341_drawfilledcircle_imagebuffer---círculo-relleno-en-buffer)
      - [`ILI9341_DrawTriangle_ImageBuffer()` - Triángulo en Buffer](#ili9341_drawtriangle_imagebuffer---triángulo-en-buffer)
      - [`ILI9341_DrawFilledTriangle_ImageBuffer()` - Triángulo Relleno en Buffer](#ili9341_drawfilledtriangle_imagebuffer---triángulo-relleno-en-buffer)
      - [`ILI9341_DrawEllipse_ImageBuffer()` - Elipse en Buffer](#ili9341_drawellipse_imagebuffer---elipse-en-buffer)
      - [`ILI9341_DrawFilledEllipse_ImageBuffer()` - Elipse Rellena en Buffer](#ili9341_drawfilledellipse_imagebuffer---elipse-rellena-en-buffer)
      - [`ILI9341_DrawArc_ImageBuffer()` - Arco en Buffer](#ili9341_drawarc_imagebuffer---arco-en-buffer)
      - [`ILI9341_DrawFilledArc_ImageBuffer()` - Arco Relleno en Buffer](#ili9341_drawfilledarc_imagebuffer---arco-relleno-en-buffer)
      - [`ILI9341_BlitImage()` - Copiar Imagen con DMA2D *(solo DMA2D)*](#ili9341_blitimage---copiar-imagen-con-dma2d-solo-dma2d)
      - [`ILI9341_SetFrameBuffers()` - Registrar Buffers de Doble Buffer](#ili9341_setframebuffers---registrar-buffers-de-doble-buffer)
      - [`ILI9341_Flush()` - Presentar Frame (Doble Buffer)](#ili9341_flush---presentar-frame-doble-buffer)
      - [`ILI9341_Sync()` - Sincronizar DMA con el bus SPI](#ili9341_sync---sincronizar-dma-con-el-bus-spi)
      - [`ILI9341_GetFrameBuffer()` - Obtener Puntero al Back Buffer](#ili9341_getframebuffer---obtener-puntero-al-back-buffer)
      - [`ILI9341_TP_Config()` - Configurar Panel Táctil](#ili9341_tp_config---configurar-panel-táctil)
      - [`ILI9341_TP_GetState()` - Obtener Estado del Toque](#ili9341_tp_getstate---obtener-estado-del-toque)
  - [Colores Predefinidos](#colores-predefinidos)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[2.0.0\] - 08-07-2026](#200---08-07-2026)
      - [Added](#added)
      - [Changed](#changed)
      - [Removed](#removed)
      - [Migration notes](#migration-notes)
    - [\[1.4.0\] - 07-07-2026](#140---07-07-2026)
      - [Added](#added-1)
    - [\[1.3.0\] - 03-07-2026](#130---03-07-2026)
      - [Added](#added-2)
      - [Changed](#changed-1)
      - [Removed](#removed-1)
      - [Migration notes](#migration-notes-1)
    - [\[1.2.0\] - 15-06-2026](#120---15-06-2026)
      - [Added](#added-3)
      - [Changed](#changed-2)
      - [Fixed](#fixed)
      - [Migration notes](#migration-notes-2)
    - [\[1.1.0\] - 14-06-2026](#110---14-06-2026)
      - [Added](#added-4)
      - [Changed](#changed-3)
      - [Fixed](#fixed-1)
    - [\[1.0.1\]](#101)
      - [Fixed](#fixed-2)
    - [\[1.0.0\] - 08-06-2026](#100---08-06-2026)
      - [Added](#added-5)

---

## Descripción

Librería desarrollada en C para el control de la pantalla TFT LCD **Ilitek ILI9341** de 240×320 píxeles sobre cualquier microcontrolador **STM32** con HAL, mediante SPI. Proporciona una API completa para inicialización, primitivas de dibujo (píxeles, líneas, rectángulos, círculos), renderizado de texto con fuentes personalizables, transferencia eficiente de imágenes completas y soporte de frame buffer fuera de pantalla en cualquier memoria RAM que el usuario reserve (interna o externa), incluyendo un modo de doble buffer con pipelining DMA. Incluye aceleración por hardware **DMA2D** (opcional, si el MCU lo tiene) para relleno de rectángulos y copia de imágenes al frame buffer, así como soporte opcional para el panel táctil resistivo basado en el controlador **STMPE811** por I2C. Los pines CS, RESET y D/C del LCD son configurables en tiempo de ejecución, por lo que la librería no está atada a ninguna placa de desarrollo en particular.

Diseñada para ser portable y robusta: toda función pública (incluidas las variantes `*_ImageBuffer()`) retorna un código de estado `ILI9341_Status_t` que permite detectar errores de comunicación SPI/I2C, timeouts y condiciones de driver no inicializado.

---

## Características

- **Comunicación SPI optimizada**: Inicialización a 2 Mbit/s para la secuencia de configuración del chip; tras `ILI9341_Init()` el preescalador se eleva automáticamente a 45 Mbit/s para máxima velocidad de refresco.
- **Escrituras SPI optimizadas mediante acceso directo al registro `DR`**: `ILI9341_Fill()`, `ILI9341_DrawFilledRectangle()` e `ILI9341_Putc()` acceden directamente al registro `DR` del SPI. El sondeo de TXE usa un contador de iteraciones (`SPI_ILI9341_WaitTXE`) en lugar de `HAL_GetTick()`, eliminando una llamada a función y una lectura de tick por byte en los bucles críticos de volcado.
- **Volcado de frame buffer por DMA SPI**: `ILI9341_DisplayImage()` e `ILI9341_Flush()` transfieren los 76 800 píxeles del frame buffer a la pantalla usando **DMA2\_Stream6** vinculado a SPI5\_TX en modo 16 bits. El SPI en modo 16 bits serializa cada `uint16_t` MSB-first, produciendo automáticamente el orden big-endian esperado por el ILI9341 sin swap manual de bytes. La transferencia se divide en dos tramos de 38 400 píxeles para respetar el límite de 65 535 items del registro NDTR del DMA.
- **Aceleración DMA2D** *(requiere `HAL_DMA2D_MODULE_ENABLED`)*: `ILI9341_Init()` acepta un `DMA2D_HandleTypeDef*` opcional; si no es NULL, configura el periférico DMA2D una sola vez y lo reutiliza en modo R2M (relleno) para `ILI9341_DrawFilledRectangle_ImageBuffer()` y en modo M2M (copia) para `ILI9341_BlitImage()`. Si se pasa NULL, ambas operaciones usan el camino CPU.
- **Primitivas de dibujo completas**: Píxeles, líneas (algoritmo de Bresenham) con variantes rápidas horizontal/vertical (`ILI9341_DrawFastHLine`/`ILI9341_DrawFastVLine`) y con grosor configurable (`ILI9341_DrawThickLine`), rectángulos (contorno y relleno), círculos (contorno y relleno), triángulos (contorno y relleno por scanline), elipses (contorno y relleno, algoritmo de punto medio de Zingl) y arcos/sectores de anillo entre dos ángulos (contorno y relleno) directamente sobre la pantalla.
- **Paleta de 147 colores predefinidos** en formato RGB565 basada en el estándar de nombres de color X11/CSS, generada con la macro pública `RGB565(r, g, b)`. Incluye también `RGB16TO24(c)` para expandir un color RGB565 de vuelta a RGB888, y la función `ILI9341_Color565(r, g, b)` para convertir componentes RGB888 calculadas en tiempo de ejecución.
- **Renderizado de texto**: `ILI9341_Putc()` / `ILI9341_Puts()` con soporte de saltos de línea, retorno de carro y fuentes de ancho variable mediante `LCD_FontDef_t`. `ILI9341_Printf()` añade formateo estilo printf (vía `vsnprintf()` sobre un buffer interno configurable con `ILI9341_PRINTF_BUF_SIZE`), y `ILI9341_PutsAligned()` / `ILI9341_PrintfAligned()` permiten alinear el texto (izquierda, centro, derecha) dentro de una región horizontal.
- **Frame buffer fuera de pantalla (RAM)**: Juego completo de funciones `*_ImageBuffer()` que operan sobre un array `uint32_t[IMG_TOTAL_BUF32]` reservado por el usuario en la memoria que elija (RAM interna, RAM externa, etc.), empaquetando dos píxeles RGB565 por palabra de 32 bits. Todas estas funciones retornan `ILI9341_Status_t` para detectar errores (puntero NULL, fallo DMA2D). Ideal para composición de imagen sin parpadeo.
- **Doble buffer con pipelining DMA**: `ILI9341_SetFrameBuffers()` registra dos buffers provistos por el usuario; `ILI9341_Flush()` intercambia front/back e inicia el envío por DMA sin bloquear, permitiendo dibujar el siguiente frame mientras se transmite el anterior.
- **Panel táctil resistivo STMPE811** *(opcional, requiere `HAL_I2C_MODULE_ENABLED`)*: Configuración, lectura de coordenadas X/Y calibradas ([0, 239] × [0, 319]) y presión Z por I2C, con filtro de histeresis de 5 puntos.
- **Cuatro orientaciones de pantalla**: Portrait 0°/180° y Landscape 90°/270° configurables en tiempo de ejecución con `ILI9341_Rotate()`.
- **Manejo robusto de errores**: Retornos `ILI9341_Status_t` en todas las funciones públicas (incluidas las variantes `*_ImageBuffer()`) con valores específicos para parámetro inválido, timeout de SPI, fallo DMA2D y driver no inicializado.
- **Portabilidad HAL**: Solo requiere `main.h` (que incluye el HAL de STM32 correspondiente) para compilar. Los pines CS, RESET y D/C se pasan como parámetros a `ILI9341_Init()`, por lo que no hay nada que editar en la librería para adaptarla a otra placa o distribución de pines.

---

## Pinout y Conexiones

### Bus SPI (LCD ILI9341)

| Pin ILI9341 | Descripción                    | Configuración                                   |
|-------------|---------------------------------|--------------------------------------------------|
| **VCC**     | Alimentación                   | 3.3V                                              |
| **GND**     | Tierra                         | GND                                               |
| **SDO/MISO**| Salida LCD (opcional, lectura) | Pin MISO del SPI que elijas (CubeMX)              |
| **SCK**     | Reloj SPI                      | Pin SCK del SPI que elijas (CubeMX)               |
| **SDI/MOSI**| Entrada datos al LCD           | Pin MOSI del SPI que elijas (CubeMX)              |

Estos pines (más VCC/GND) se configuran junto con el periférico SPI en CubeMX y no dependen de esta librería; puede usarse cualquier instancia SPI del MCU.

### GPIO de Control (LCD ILI9341)

| Señal         | Descripción                | Tipo GPIO                | Nivel inicial | Parámetro de `ILI9341_Init()` |
|---------------|-----------------------------|---------------------------|----------------|---------------------------------|
| **D/C (WRX)** | Selección Dato / Comando   | `GPIO_OUTPUT` push-pull  | —              | `dcPort`, `dcPin`               |
| **RESET**     | Reset por hardware del LCD | `GPIO_OUTPUT` push-pull  | HIGH           | `rstPort`, `rstPin`             |
| **CS**        | Selección de chip SPI      | `GPIO_OUTPUT` push-pull  | HIGH           | `csPort`, `csPin`               |

> [!NOTE]
> Los pines D/C, RESET y CS son de propósito general: configúralos como salida push-pull en CubeMX (en cualquier puerto/pin libre) y pasa el puerto/pin elegido como parámetro a `ILI9341_Init()`. No hay macros ni nada que editar en el código fuente para adaptar la librería a otra placa.

### Bus I2C (Panel táctil STMPE811, opcional)

Solo aplica si tu pantalla incluye un panel táctil basado en el controlador STMPE811 y compilas con `HAL_I2C_MODULE_ENABLED` definido.

| Señal         | Descripción | Tipo GPIO                | Observaciones                                              |
|---------------|--------------|---------------------------|--------------------------------------------------------------|
| **Touch SCL** | Reloj I2C    | `I2C_SCL` (open-drain)   | Requiere pull-up externo (4.7 kΩ) si tu placa no los trae   |
| **Touch SDA** | Datos I2C    | `I2C_SDA` (open-drain)   | Requiere pull-up externo (4.7 kΩ) si tu placa no los trae   |

> [!NOTE]
> Usa cualquier instancia I2C de tu MCU; la librería solo necesita el `I2C_HandleTypeDef*` correspondiente.

---

## Configuración SPI

Configura el periférico **SPI** que vayas a usar en CubeMX/STM32CubeIDE:

| Parámetro               | Valor                | Notas                                                                       |
|-------------------------|----------------------|-----------------------------------------------------------------------------|
| **Mode**                | Full-Duplex Master   | STM32 controla el bus                                                       |
| **Hardware NSS**        | Disable              | CS gestionado por software (pin GPIO indicado en `ILI9341_Init()`)          |
| **Data Size**           | 8 Bits               | —                                                                           |
| **First Bit**           | MSB First            | —                                                                           |
| **Prescaler**           | El que dé ~2 a ~3 Mbit/s | Valor inicial para la secuencia de Init; la librería lo eleva a `/2` al final |
| **CPOL**                | Low (0)              | —                                                                           |
| **CPHA**                | 1 Edge               | —                                                                           |
| **CRC**                 | Disabled             | No utilizado                                                                |

> [!NOTE]
> Tras una llamada exitosa a `ILI9341_Init()`, la librería reinicializa el SPI con el preescalador `/2` para maximizar la velocidad de transferencia de imágenes (la velocidad resultante depende del reloj de tu MCU). No es necesario hacer nada desde el código de usuario.

---

## Configuración I2C

Solo necesaria si usas el panel táctil STMPE811. Configura el periférico **I2C** que vayas a usar en CubeMX/STM32CubeIDE:

| Parámetro          | Valor            | Notas                                     |
|--------------------|------------------|-------------------------------------------|
| **Mode**           | I2C              | —                                         |
| **I2C Speed Mode** | Standard Mode    | Soporta 100 kHz y 400 kHz                 |
| **I2C Clock Speed**| 100 000          | Depende del requisito de latencia táctil  |

> [!NOTE]
> La dirección I2C del STMPE811 es **0x41** (7 bits) / **0x82** (8 bits, escritura). La librería gestiona internamente la composición de las direcciones de lectura y escritura; no es necesario configurar nada adicional.

---

## Configuración DMA2D (opcional)

El soporte de aceleración gráfica por hardware se activa automáticamente cuando `HAL_DMA2D_MODULE_ENABLED` está definido (lo incluye el HAL generado por CubeMX al habilitar el periférico DMA2D).

Habilita el periférico **DMA2D** en CubeMX: basta con marcarlo como *Activated* en la categoría *Multimedia*. La librería configura internamente el modo, el formato de color y la capa de entrada dentro de `ILI9341_Init()`, por lo que no es necesario ajustar ningún parámetro adicional en CubeMX.

| Función acelerada por DMA2D | Modo DMA2D | Descripción |
|-----------------------------|------------|-------------|
| `ILI9341_DrawFilledRectangle_ImageBuffer()` | R2M (Register to Memory) | Rellena un rectángulo en el frame buffer sin intervención de la CPU |
| `ILI9341_BlitImage()` | M2M (Memory to Memory) | Copia una imagen RGB565 fuente al frame buffer |

> [!NOTE]
> Si se pasa `NULL` como `hdma2d` en `ILI9341_Init()`, ambas operaciones usan el camino CPU como respaldo. El resto de funciones de dibujo no se ven afectadas.

---

## Configuración DMA SPI TX (para DisplayImage y Flush)

El volcado del frame buffer por DMA se activa al configurar un canal/stream DMA en modo TX para el periférico SPI elegido, en CubeMX. No requiere ninguna macro de compilación adicional; la librería usa `HAL_SPI_Transmit_DMA()` sobre el handle SPI que le pasaste en `ILI9341_Init()`.

En la pestaña **`<TuSPI>` → DMA Settings** de CubeMX (ejemplo con SPI5 en la familia STM32F4; el Stream/Channel exacto depende de tu MCU y del periférico SPI elegido):

| Parámetro | Valor |
|-----------|-------|
| **Request** | `<TuSPI>`_TX |
| **Stream/Channel DMA** | El que asigne CubeMX automáticamente |
| **Direction** | Memory To Peripheral |
| **Mode** | Normal |
| **Priority** | High |
| **Peripheral Data Width** | **Half Word** |
| **Memory Data Width** | **Half Word** |
| **Memory Increment** | Enabled ✓ |
| **Peripheral Increment** | Disabled |

> [!IMPORTANT]
> Tanto **Peripheral Data Width** como **Memory Data Width** deben configurarse en **Half Word** (16 bits). Esto es fundamental: el SPI se reconfigura a 16 bits justo antes del DMA para que el periférico envíe cada píxel RGB565 MSB-first (big-endian), de modo que no se necesita reordenar bytes desde el frame buffer. Con ancho de 8 bits el byte order sería incorrecto.

> [!NOTE]
> CubeMX genera el enlace `hspiX.hdmatx = &hdma_spiX_tx` dentro de `HAL_SPI_MspInit()` y el handler de interrupción del stream/canal DMA correspondiente. Verifica que la llamada a `MX_DMA_Init()` aparezca **antes** de la inicialización del SPI en `main.c`.

> [!WARNING]
> La función `HAL_SPI_TxCpltCallback` está definida dentro de la librería. Si tu proyecto ya define este callback para otro propósito, obtendrás un error de símbolo duplicado en el enlazado. En ese caso, gestiona los dos handles SPI con una condición `if (hspi == &tu_handle)` dentro de un único callback.

---

## Instalación

1. Copia `ILI9341.c`, `ILI9341.h` y `lcd_fonts.h` a tu proyecto (ej: `Librerias/ILI9341/`).
2. Incluye la librería en tu `main.c` o archivo principal:
   ```c
   #include "ILI9341.h"
   ```
3. Configura el SPI (y, si usas panel táctil, el I2C) en CubeMX (ver secciones anteriores).
4. Configura como salida push-pull los tres pines GPIO que usarás para CS, RESET y D/C.
5. Si quieres aceleración DMA2D, habilítala en CubeMX — `HAL_DMA2D_MODULE_ENABLED` se definirá automáticamente.
6. Genera código y compila.

---

## Uso Básico

### 1. Inicialización

```c
/* Pines de control del LCD: se pasan como parámetros, no hay macros que editar */
#define ILI9341_CS_PORT   GPIOC
#define ILI9341_CS_PIN    GPIO_PIN_2
#define ILI9341_RST_PORT  GPIOD
#define ILI9341_RST_PIN   GPIO_PIN_12
#define ILI9341_DC_PORT   GPIOD
#define ILI9341_DC_PIN    GPIO_PIN_13

/* Solo SPI (sin panel táctil, sin DMA2D) */
ILI9341_Status_t status = ILI9341_Init(&hspi5,
                                        ILI9341_CS_PORT, ILI9341_CS_PIN,
                                        ILI9341_RST_PORT, ILI9341_RST_PIN,
                                        ILI9341_DC_PORT, ILI9341_DC_PIN);

/* Con panel táctil I2C, sin DMA2D */
#ifdef HAL_I2C_MODULE_ENABLED
ILI9341_Status_t status = ILI9341_Init(&hspi5,
                                        ILI9341_CS_PORT, ILI9341_CS_PIN,
                                        ILI9341_RST_PORT, ILI9341_RST_PIN,
                                        ILI9341_DC_PORT, ILI9341_DC_PIN,
                                        &hi2c3);
#endif

/* Con DMA2D, sin panel táctil */
#ifdef HAL_DMA2D_MODULE_ENABLED
ILI9341_Status_t status = ILI9341_Init(&hspi5,
                                        ILI9341_CS_PORT, ILI9341_CS_PIN,
                                        ILI9341_RST_PORT, ILI9341_RST_PIN,
                                        ILI9341_DC_PORT, ILI9341_DC_PIN,
                                        &hdma2d);
#endif

/* Con panel táctil I2C y DMA2D */
#if defined(HAL_I2C_MODULE_ENABLED) && defined(HAL_DMA2D_MODULE_ENABLED)
ILI9341_Status_t status = ILI9341_Init(&hspi5,
                                        ILI9341_CS_PORT, ILI9341_CS_PIN,
                                        ILI9341_RST_PORT, ILI9341_RST_PIN,
                                        ILI9341_DC_PORT, ILI9341_DC_PIN,
                                        &hi2c3, &hdma2d);
#endif

if (status != ILI9341_OK) {
    Error_Handler();
}

/* Inicializar el panel táctil (solo si se pasó hi2c en Init) */
if (ILI9341_TP_Config() != ILI9341_OK) {
    Error_Handler();  // STMPE811 no detectado (ID incorrecto)
}
```

> [!NOTE]
> El SPI debe estar previamente inicializado a ~2-3 Mbit/s. La librería elevará el preescalador automáticamente al final de `ILI9341_Init()`. Los pines CS/RST/D-C se configuran como salida push-pull en CubeMX y se pasan como parámetros; el handle I2C (`hi2c`) y el handle DMA2D (`hdma2d`) son opcionales: usa la sobrecarga que corresponda a los módulos HAL habilitados en tu proyecto. Pasar `NULL` como `hdma2d` deshabilita la aceleración DMA2D sin error.

---

### 2. Dibujo en pantalla

```c
/* Limpiar pantalla con color negro */
ILI9341_Fill(ILI9341_COLOR_BLACK);

/* Dibujar un píxel rojo en (120, 160) */
ILI9341_DrawPixel(120, 160, ILI9341_COLOR_RED);

/* Dibujar una línea azul */
ILI9341_DrawLine(0, 0, 239, 319, ILI9341_COLOR_BLUE);

/* Dibujar una línea con grosor de 5 píxeles */
ILI9341_DrawThickLine(20, 20, 200, 80, 5, ILI9341_COLOR_RED);

/* Dibujar el contorno de un rectángulo */
ILI9341_DrawRectangle(20, 20, 220, 100, ILI9341_COLOR_GREEN);

/* Dibujar un rectángulo relleno */
ILI9341_DrawFilledRectangle(20, 120, 220, 200, ILI9341_COLOR_YELLOW);

/* Dibujar el contorno de un rectángulo con esquinas redondeadas (radio = 12) */
ILI9341_DrawRoundRect(20, 220, 220, 300, 12, ILI9341_COLOR_ORANGE);

/* Dibujar un rectángulo redondeado relleno */
ILI9341_DrawFilledRoundRect(20, 220, 220, 300, 12, ILI9341_COLOR_ORANGE);

/* Dibujar el contorno de un círculo */
ILI9341_DrawCircle(120, 160, 50, ILI9341_COLOR_CYAN);

/* Dibujar un círculo relleno */
ILI9341_DrawFilledCircle(120, 160, 30, ILI9341_COLOR_MAGENTA);

/* Dibujar el contorno de un triángulo */
ILI9341_DrawTriangle(10, 10, 230, 10, 120, 150, ILI9341_COLOR_RED);

/* Dibujar un triángulo relleno */
ILI9341_DrawFilledTriangle(10, 170, 230, 170, 120, 310, ILI9341_COLOR_BLUE);

/* Dibujar una línea vertical/horizontal rápida (sin Bresenham) */
ILI9341_DrawFastVLine(120, 0, 319, ILI9341_COLOR_GRAY);
ILI9341_DrawFastHLine(0, 160, 239, ILI9341_COLOR_GRAY);

/* Dibujar el contorno de una elipse */
ILI9341_DrawEllipse(120, 160, 80, 40, ILI9341_COLOR_DODGERBLUE);

/* Dibujar una elipse rellena */
ILI9341_DrawFilledEllipse(120, 160, 40, 80, ILI9341_COLOR_GOLD);

/* Dibujar el contorno de un arco (sector de anillo) de 0° a 90° */
ILI9341_DrawArc(120, 160, 100, 60, 0.0f, 90.0f, ILI9341_COLOR_TOMATO);

/* Dibujar un arco relleno de 180° a 270° */
ILI9341_DrawFilledArc(120, 160, 100, 60, 180.0f, 270.0f, ILI9341_COLOR_LIMEGREEN);

/* Rotar la pantalla a modo apaisado */
ILI9341_Rotate(ILI9341_Orientation_Landscape_1);
```

---

### 3. Texto en pantalla

```c
extern LCD_FontDef_t Font_11x18;  /* Fuente definida en lcd_fonts.h */

/* Renderizar un carácter */
ILI9341_Putc(10, 10, 'A', &Font_11x18,
                 ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

/* Renderizar una cadena con salto de línea.
 * '\n' avanza a la siguiente línea volviendo a x=10 (la posición x inicial).
 * La secuencia '\n'+'\r' avanza a la siguiente línea y reinicia a columna 0. */
ILI9341_Puts(10, 30, "Hola, mundo!\nSTM32",
                 &Font_11x18,
                 ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

/* Calcular el ancho y alto de una cadena antes de dibujarla */
uint16_t w, h;
ILI9341_GetStringSize("Texto", &Font_11x18, &w, &h);
/* Centrar horizontalmente */
ILI9341_Puts((240 - w) / 2, 150, "Texto",
                 &Font_11x18,
                 ILI9341_COLOR_YELLOW, ILI9341_COLOR_BLACK);

/* Renderizar texto con formato estilo printf */
uint16_t temp = 235;
ILI9341_Printf(10, 60, &Font_11x18,
                   ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK,
                   "Temp: %u.%u C", temp / 10, temp % 10);

/* Alinear una cadena (izquierda, centro o derecha) dentro de una región horizontal,
 * sin necesidad de calcular manualmente el ancho con ILI9341_GetStringSize() */
ILI9341_PutsAligned(0, 239, 80, ILI9341_ALIGN_CENTER, "Centrado",
                        &Font_11x18,
                        ILI9341_COLOR_YELLOW, ILI9341_COLOR_BLACK);

/* Combinar formato printf + alineación (por ejemplo, un valor pegado al borde derecho) */
ILI9341_PrintfAligned(0, 239, 100, ILI9341_ALIGN_RIGHT,
                          &Font_11x18,
                          ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK,
                          "%d%%", 87);
```

> [!NOTE]
> `ILI9341_Printf()` y `ILI9341_PrintfAligned()` formatean los argumentos variádicos con `vsnprintf()` en un buffer interno en pila de `ILI9341_PRINTF_BUF_SIZE` bytes (128 por defecto). Redefine esta macro antes de incluir `ILI9341.h` si necesitas cadenas más largas. Si el resultado formateado excede el buffer, la cadena se trunca de forma segura (sin desbordamiento).

---

### 4. Frame buffer fuera de pantalla (RAM)

Útil para eliminar el parpadeo al componer escenas complejas: se dibuja todo en el buffer de RAM y se envía a la pantalla de una sola vez.

```c
/* Buffer en RAM estática: 38 400 palabras × 4 bytes = 153 600 bytes */
static uint32_t fb[IMG_TOTAL_BUF32];

/* Limpiar el buffer (color negro) */
memset(fb, 0, sizeof(fb));

/* Dibujar sobre el buffer */
ILI9341_DrawLine_ImageBuffer(0, 0, 239, 319,
                                 ILI9341_COLOR_GREEN, fb);
ILI9341_DrawFilledRectangle_ImageBuffer(50, 50, 190, 270,
                                            ILI9341_COLOR_BLUE, fb);
ILI9341_Puts_ImageBuffer(10, 10, "Frame buffer",
                              &Font_11x18,
                              ILI9341_COLOR_WHITE, fb);

/* Volcar el buffer a la pantalla */
ILI9341_DisplayImage(fb);
```

> [!WARNING]
> El buffer ocupa **153 600 bytes** (`IMG_TOTAL_BUF32 * 4`). Verifica que tu MCU tenga RAM suficiente y declara el buffer como `static` o global para evitar desbordamiento de pila.

---

### 5. Doble buffer con pipelining DMA

Registra dos buffers propios (front/back) con `ILI9341_SetFrameBuffers()`; a partir de ahí, `ILI9341_GetFrameBuffer()` siempre devuelve el back buffer activo para dibujar, y `ILI9341_Flush()` intercambia los buffers e inicia el envío del frame recién dibujado por DMA sin bloquear, permitiendo componer el siguiente frame mientras el anterior todavía se está transmitiendo.

```c
/* Dos buffers de IMG_TOTAL_BUF32 palabras cada uno, en la memoria que prefieras
 * (RAM interna, RAM externa si tu MCU la tiene, etc.) */
static uint32_t fbA[IMG_TOTAL_BUF32];
static uint32_t fbB[IMG_TOTAL_BUF32];

ILI9341_SetFrameBuffers(fbA, fbB);  /* los limpia a cero */

for (;;) {
    uint32_t* fb = ILI9341_GetFrameBuffer();  /* back buffer activo; puntero fresco cada frame */

    ILI9341_DrawFilledRectangle_ImageBuffer(0, 0, 239, 319,
                                                ILI9341_COLOR_BLACK, fb);
    ILI9341_Puts_ImageBuffer(60, 150, "Doble buffer",
                                  &Font_11x18,
                                  ILI9341_COLOR_GREEN, fb);

    ILI9341_Flush();  /* intercambia buffers y envía por DMA sin bloquear */
}

/* Al salir del modo doble buffer, sincronizar antes de usar funciones de dibujo directo */
ILI9341_Sync();
ILI9341_Fill(ILI9341_COLOR_BLACK);
```

> [!NOTE]
> `ILI9341_SetFrameBuffers(NULL, NULL)` deshabilita el modo doble buffer; a partir de ahí `ILI9341_Flush()`/`ILI9341_Sync()`/`ILI9341_GetFrameBuffer()` vuelven a retornar `ILI9341_INVALID_PARAM`/`NULL`.

---

### 6. Panel táctil

```c
while (1) {
    TP_STATE* touch = ILI9341_TP_GetState();

    if (touch != NULL && touch->TouchDetected) {
        /* touch->X : coordenada X calibrada [0, 239] */
        /* touch->Y : coordenada Y calibrada [0, 319] */
        /* touch->Z : presión (valor ADC crudo)        */
        ILI9341_DrawPixel(touch->X, touch->Y, ILI9341_COLOR_RED);
    }

    HAL_Delay(20);  /* ~50 Hz de muestreo */
}
```

---

## API Reference

### 1. Tipos de Datos

#### `ILI9341_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería.

```c
typedef enum {
    ILI9341_OK              = 0,  /**< Operación exitosa            */
    ILI9341_ERROR           = 1,  /**< Operación fallida            */
    ILI9341_TIMEOUT         = 2,  /**< Tiempo de espera HAL agotado */
    ILI9341_NOT_INITIALIZED = 3,  /**< Driver no inicializado       */
    ILI9341_INVALID_PARAM   = 4   /**< Parámetro inválido           */
} ILI9341_Status_t;
```

| Valor | Código | Significado |
|-------|--------|-------------|
| `ILI9341_OK` | 0 | Operación completada sin errores |
| `ILI9341_ERROR` | 1 | Fallo de transmisión SPI o I2C |
| `ILI9341_TIMEOUT` | 2 | El bus SPI se bloqueó más allá del timeout |
| `ILI9341_NOT_INITIALIZED` | 3 | `ILI9341_Init()` no se llamó o falló |
| `ILI9341_INVALID_PARAM` | 4 | Puntero NULL u otro parámetro inválido |

---

#### `ILI9341_Orientation_t` - Orientaciones de Pantalla

```c
typedef enum {
    ILI9341_Orientation_Portrait_1,   /**< Sin rotación  (240×320) */
    ILI9341_Orientation_Portrait_2,   /**< Rotación 180° (240×320) */
    ILI9341_Orientation_Landscape_1,  /**< Rotación 90°  (320×240) */
    ILI9341_Orientation_Landscape_2   /**< Rotación 270° (320×240) */
} ILI9341_Orientation_t;
```

| Valor | Rotación | Resolución activa |
|-------|----------|-------------------|
| `Portrait_1`  | 0°   | 240 × 320 |
| `Portrait_2`  | 180° | 240 × 320 |
| `Landscape_1` | 90°  | 320 × 240 |
| `Landscape_2` | 270° | 320 × 240 |

---

#### `ILI9341_TextAlign_t` - Alineación de Texto

Usada por `ILI9341_PutsAligned()` / `ILI9341_PrintfAligned()` y sus variantes de frame buffer para posicionar una cadena dentro de una región horizontal `[x0, x1]`.

```c
typedef enum {
    ILI9341_ALIGN_LEFT,     /**< Cadena pegada al borde izquierdo de la región */
    ILI9341_ALIGN_CENTER,   /**< Cadena centrada dentro de la región           */
    ILI9341_ALIGN_RIGHT     /**< Cadena pegada al borde derecho de la región   */
} ILI9341_TextAlign_t;
```

| Valor | Descripción |
|-------|-------------|
| `ILI9341_ALIGN_LEFT`   | Equivalente a dibujar en `x0` |
| `ILI9341_ALIGN_CENTER` | Centra la cadena en `(x1 - x0 + 1)` |
| `ILI9341_ALIGN_RIGHT`  | Alinea el extremo derecho de la cadena con `x1` |

> [!NOTE]
> Si la cadena es más ancha que la región `[x0, x1]`, se alinea contra `x0` independientemente del valor de `align`.

---

#### `TP_STATE` - Estado del Panel Táctil

```c
typedef struct {
    uint16_t TouchDetected;  /**< Distinto de cero cuando hay toque activo */
    uint16_t X;              /**< Coordenada X calibrada [0, 239]          */
    uint16_t Y;              /**< Coordenada Y calibrada [0, 319]          */
    uint16_t Z;              /**< Índice de presión (valor ADC crudo)      */
} TP_STATE;
```

| Campo | Tipo | Rango | Descripción |
|-------|------|-------|-------------|
| `TouchDetected` | `uint16_t` | 0 / ≠0 | `0` si no hay toque; distinto de 0 si hay contacto activo |
| `X` | `uint16_t` | 0 – 239 | Coordenada horizontal calibrada |
| `Y` | `uint16_t` | 0 – 319 | Coordenada vertical calibrada |
| `Z` | `uint16_t` | variable | Valor ADC de presión (mayor = más presión) |

---

### 2. Funciones Públicas

#### `ILI9341_Init()` - Inicialización del Driver

Inicializa la pantalla ILI9341 y, opcionalmente, el acelerador DMA2D. Aplica la secuencia de configuración del chip, enciende la pantalla y eleva el preescalador SPI al finalizar. El estado inicial es Portrait 1 (240×320).

La firma varía según los módulos HAL habilitados:

| `HAL_I2C_MODULE_ENABLED` | `HAL_DMA2D_MODULE_ENABLED` | Firma resultante |
|:-------------------------:|:---------------------------:|------------------|
| No | No | `ILI9341_Init(hspi, csPort, csPin, rstPort, rstPin, dcPort, dcPin)` |
| No | Sí | `ILI9341_Init(hspi, csPort, csPin, rstPort, rstPin, dcPort, dcPin, hdma2d)` |
| Sí | No | `ILI9341_Init(hspi, csPort, csPin, rstPort, rstPin, dcPort, dcPin, hi2c)` |
| Sí | Sí | `ILI9341_Init(hspi, csPort, csPin, rstPort, rstPin, dcPort, dcPin, hi2c, hdma2d)` |

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `hspi` | `SPI_HandleTypeDef*` | Handle del periférico SPI (obligatorio) |
| `csPort`, `csPin` | `GPIO_TypeDef*`, `uint16_t` | Puerto/pin del pin CS del LCD (obligatorio) |
| `rstPort`, `rstPin` | `GPIO_TypeDef*`, `uint16_t` | Puerto/pin del pin RESET del LCD (obligatorio) |
| `dcPort`, `dcPin` | `GPIO_TypeDef*`, `uint16_t` | Puerto/pin del pin D/C (WRX) del LCD (obligatorio) |
| `hi2c` | `I2C_HandleTypeDef*` | Handle del periférico I2C (panel táctil); obligatorio si `HAL_I2C_MODULE_ENABLED` |
| `hdma2d` | `DMA2D_HandleTypeDef*` | Handle DMA2D. `NULL` deshabilita la aceleración DMA2D (rellenos y BlitImage usan CPU) |

**Retorna**: `ILI9341_OK` si la inicialización fue exitosa, `ILI9341_INVALID_PARAM` si `hspi`, `csPort`, `rstPort`, `dcPort` o `hi2c` (cuando aplica) son NULL, `ILI9341_ERROR` si una transmisión SPI o la configuración DMA2D falló durante la inicialización.

**Secuencia interna:**
1. Validación de parámetros; registro de los pines CS/RESET/D-C indicados.
2. Reset por hardware y envío de la secuencia de comandos de configuración del ILI9341 (power, gamma, MAC, pixel format, etc.).
3. Salida del modo sleep y encendido de la pantalla.
4. Reinicialización del SPI con preescalador `/2` (máxima velocidad soportada por el bus).
5. *(Si `hdma2d != NULL`)* Configuración única del DMA2D: formato de salida RGB565, capa de entrada RGB565. Las operaciones de dibujo posteriores solo actualizan modo y offset mediante `ILI9341_DMA2D_SetMode()`.

---

#### `ILI9341_DeInit()` - Desinicialización del Driver

Marca el driver como no inicializado, libera los handles internos (SPI, I2C, DMA2D) y limpia los punteros de frame buffer registrados con `ILI9341_SetFrameBuffers()`. La memoria de los buffers sigue siendo responsabilidad del usuario (la librería nunca la reserva ni la libera).

```c
ILI9341_Status_t ILI9341_DeInit(void);
```

**Retorna**: `ILI9341_OK` si la desinicialización fue exitosa, `ILI9341_NOT_INITIALIZED` si el driver no estaba inicializado.

---

#### `ILI9341_Color565()` - Convertir RGB888 a RGB565

Convierte una componente de color RGB888 (8 bits por canal) a RGB565. Equivale a la macro `RGB565(r, g, b)`, pero como función evita que el usuario tenga que calcular el empaquetado de bits manualmente y permite pasar valores calculados en tiempo de ejecución (por ejemplo, resultado de una interpolación de color).

```c
uint16_t ILI9341_Color565(uint8_t r, uint8_t g, uint8_t b);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `r` | `uint8_t` | Componente roja (0-255) |
| `g` | `uint8_t` | Componente verde (0-255) |
| `b` | `uint8_t` | Componente azul (0-255) |

**Retorna**: `uint16_t` — color empaquetado en formato RGB565.

---

#### `ILI9341_Fill()` - Rellenar Pantalla

Rellena toda la pantalla con un color sólido en formato RGB565.

```c
ILI9341_Status_t ILI9341_Fill(uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `color` | `uint16_t` | Color de relleno en formato RGB565 |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_ERROR` si el SPI estaba ocupado o `ILI9341_TIMEOUT` si el bus se bloqueó.

---

#### `ILI9341_Rotate()` - Rotar Pantalla

Envía el comando de rotación al ILI9341 y actualiza las dimensiones activas internas (ancho/alto). La geometría solo se actualiza si la comunicación SPI es exitosa.

```c
ILI9341_Status_t ILI9341_Rotate(ILI9341_Orientation_t orientation);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `orientation` | `ILI9341_Orientation_t` | Orientación deseada |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED` o `ILI9341_ERROR`.

---

#### `ILI9341_DrawPixel()` - Dibujar Píxel

```c
ILI9341_Status_t ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `x` | `uint16_t` | Coordenada X | 0 – (ancho - 1) |
| `y` | `uint16_t` | Coordenada Y | 0 – (alto - 1) |
| `color` | `uint16_t` | Color RGB565 | — |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED` o `ILI9341_ERROR`.

---

#### `ILI9341_DrawLine()` - Dibujar Línea

Dibuja una línea entre dos puntos usando el algoritmo de Bresenham. Las coordenadas fuera de rango se recortan automáticamente al borde de la pantalla. Las líneas horizontales y verticales se aceleran delegando en `ILI9341_DrawFilledRectangle()` (acceso directo al `DR` del SPI).

```c
ILI9341_Status_t ILI9341_DrawLine(uint16_t x0, uint16_t y0,
                                       uint16_t x1, uint16_t y1,
                                       uint16_t color);
```

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_ERROR` o `ILI9341_TIMEOUT`.

---

#### `ILI9341_DrawThickLine()` - Dibujar Línea con Grosor

Dibuja una línea con grosor (ancho de trazo) configurable. Las líneas horizontales y verticales se rellenan con un único rectángulo (recortado a los límites de pantalla, vía `ILI9341_DrawFilledRectangle()`). Las líneas diagonales se aproximan trazando `thickness` líneas de Bresenham paralelas, desplazadas sobre la normal del segmento y centradas en la línea original; en ángulos muy pronunciados puede quedar un ligero aliasing entre trazos adyacentes.

```c
ILI9341_Status_t ILI9341_DrawThickLine(uint16_t x0, uint16_t y0,
                                        uint16_t x1, uint16_t y1,
                                        uint16_t thickness, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `uint16_t` | Coordenada de inicio |
| `x1`, `y1` | `uint16_t` | Coordenada de fin |
| `thickness` | `uint16_t` | Grosor de la línea en píxeles (`0` y `1` equivalen a `ILI9341_DrawLine()`) |
| `color` | `uint16_t` | Color de la línea en formato RGB565 |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED` o `ILI9341_ERROR` si falla la transmisión SPI.

---

#### `ILI9341_DrawFastVLine()` / `ILI9341_DrawFastHLine()` - Líneas Rápidas

Dibujan una línea vertical u horizontal sin pasar por el algoritmo de Bresenham: delegan directamente en `ILI9341_DrawFilledRectangle()` (acceso al `DR` del SPI). El alto/ancho puede pasarse como negativo; en ese caso se normaliza invirtiendo el punto de partida. Las coordenadas y longitudes se recortan automáticamente a los límites de la pantalla.

```c
ILI9341_Status_t ILI9341_DrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
ILI9341_Status_t ILI9341_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x`, `y` | `int16_t` | Punto de inicio de la línea |
| `h` | `int16_t` | Alto de la línea, en `DrawFastVLine` (negativo se normaliza) |
| `w` | `int16_t` | Ancho de la línea, en `DrawFastHLine` (negativo se normaliza) |
| `color` | `uint16_t` | Color RGB565 |

**Retorna**: `ILI9341_OK` (incluyendo el caso `h`/`w == 0`, que no dibuja nada), `ILI9341_NOT_INITIALIZED`, `ILI9341_ERROR` o `ILI9341_TIMEOUT`.

---

#### `ILI9341_DrawRectangle()` - Dibujar Rectángulo

Dibuja el contorno de un rectángulo definido por sus esquinas superior-izquierda `(x0, y0)` e inferior-derecha `(x1, y1)`.

```c
ILI9341_Status_t ILI9341_DrawRectangle(uint16_t x0, uint16_t y0,
                                            uint16_t x1, uint16_t y1,
                                            uint16_t color);
```

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_ERROR` o `ILI9341_TIMEOUT`.

---

#### `ILI9341_DrawFilledRectangle()` - Dibujar Rectángulo Relleno

Dibuja un rectángulo sólido usando acceso directo al registro `DR` del SPI para máximo rendimiento.

```c
ILI9341_Status_t ILI9341_DrawFilledRectangle(uint16_t x0, uint16_t y0,
                                                  uint16_t x1, uint16_t y1,
                                                  uint16_t color);
```

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_ERROR` si el SPI estaba ocupado o `ILI9341_TIMEOUT` si el bus se bloqueó.

---

#### `ILI9341_DrawRoundRect()` - Dibujar Rectángulo con Esquinas Redondeadas

Dibuja el contorno de un rectángulo con esquinas redondeadas. Las esquinas se forman con arcos de cuarto de círculo de radio `r` usando el algoritmo de Bresenham. Si `r` supera la mitad del lado más corto se recorta automáticamente; con `r = 0` es equivalente a `ILI9341_DrawRectangle()`.

```c
ILI9341_Status_t ILI9341_DrawRoundRect(uint16_t x0, uint16_t y0,
                                        uint16_t x1, uint16_t y1,
                                        uint16_t r, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `uint16_t` | Esquina superior izquierda |
| `x1`, `y1` | `uint16_t` | Esquina inferior derecha |
| `r` | `uint16_t` | Radio de las esquinas en píxeles |
| `color` | `uint16_t` | Color del contorno en formato RGB565 |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED` o `ILI9341_ERROR`.

---

#### `ILI9341_DrawFilledRoundRect()` - Dibujar Rectángulo Redondeado Relleno

Dibuja un rectángulo relleno con esquinas redondeadas. Internamente combina una franja central con `ILI9341_DrawFilledRectangle()` y rellena los arcos superior e inferior con tramos horizontales generados por Bresenham, sin dejar huecos ni solapar píxeles. Con `r = 0` es equivalente a `ILI9341_DrawFilledRectangle()`.

```c
ILI9341_Status_t ILI9341_DrawFilledRoundRect(uint16_t x0, uint16_t y0,
                                              uint16_t x1, uint16_t y1,
                                              uint16_t r, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `uint16_t` | Esquina superior izquierda |
| `x1`, `y1` | `uint16_t` | Esquina inferior derecha |
| `r` | `uint16_t` | Radio de las esquinas en píxeles |
| `color` | `uint16_t` | Color de relleno en formato RGB565 |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_ERROR` o `ILI9341_TIMEOUT`.

---

#### `ILI9341_DrawCircle()` - Dibujar Círculo

Dibuja el contorno de un círculo usando el algoritmo de punto medio (Bresenham).

```c
ILI9341_Status_t ILI9341_DrawCircle(int16_t x0, int16_t y0,
                                         int16_t r, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `int16_t` | Centro del círculo |
| `r` | `int16_t` | Radio en píxeles |
| `color` | `uint16_t` | Color RGB565 |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED` o `ILI9341_ERROR`.

---

#### `ILI9341_DrawFilledCircle()` - Dibujar Círculo Relleno

```c
ILI9341_Status_t ILI9341_DrawFilledCircle(int16_t x0, int16_t y0,
                                               int16_t r, uint16_t color);
```

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_ERROR` o `ILI9341_TIMEOUT`.

---

#### `ILI9341_DrawTriangle()` - Dibujar Triángulo

Dibuja el contorno de un triángulo definido por tres vértices trazando las tres aristas con el algoritmo de Bresenham (delega en `ILI9341_DrawLine()`).

```c
ILI9341_Status_t ILI9341_DrawTriangle(uint16_t x0, uint16_t y0,
                                       uint16_t x1, uint16_t y1,
                                       uint16_t x2, uint16_t y2,
                                       uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `uint16_t` | Primer vértice |
| `x1`, `y1` | `uint16_t` | Segundo vértice |
| `x2`, `y2` | `uint16_t` | Tercer vértice |
| `color` | `uint16_t` | Color RGB565 del contorno |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_ERROR` o `ILI9341_TIMEOUT`.

---

#### `ILI9341_DrawFilledTriangle()` - Dibujar Triángulo Relleno

Dibuja un triángulo relleno usando un algoritmo de scanline. Los tres vértices se ordenan por coordenada Y y se interpolan los bordes con aritmética `int32_t` pura (sin punto flotante), garantizando cobertura exacta de todos los píxeles interiores. Los triángulos degenerados (los tres vértices en la misma fila) se reducen a un tramo horizontal.

```c
ILI9341_Status_t ILI9341_DrawFilledTriangle(uint16_t x0, uint16_t y0,
                                             uint16_t x1, uint16_t y1,
                                             uint16_t x2, uint16_t y2,
                                             uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `uint16_t` | Primer vértice |
| `x1`, `y1` | `uint16_t` | Segundo vértice |
| `x2`, `y2` | `uint16_t` | Tercer vértice |
| `color` | `uint16_t` | Color RGB565 del relleno |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_ERROR` o `ILI9341_TIMEOUT`.

---

#### `ILI9341_DrawEllipse()` - Dibujar Elipse

Dibuja el contorno de una elipse usando una implementación entera del algoritmo de punto medio (variante de Zingl), recortando cada píxel a los límites de la pantalla. Los casos degenerados (`rx == 0` o `ry == 0`) se delegan en `ILI9341_DrawFastVLine()` / `ILI9341_DrawFastHLine()`.

```c
ILI9341_Status_t ILI9341_DrawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `int16_t` | Centro de la elipse |
| `rx` | `int16_t` | Radio horizontal en píxeles |
| `ry` | `int16_t` | Radio vertical en píxeles |
| `color` | `uint16_t` | Color RGB565 del contorno |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` si `rx` o `ry` son negativos, `ILI9341_ERROR` si falla la transmisión SPI.

---

#### `ILI9341_DrawFilledEllipse()` - Dibujar Elipse Rellena

Misma lógica de trazado que `ILI9341_DrawEllipse()`, pero rellena cada fila visitada con un tramo horizontal (`DrawHSpanClipped()`) en lugar de graficar 4 píxeles por iteración.

```c
ILI9341_Status_t ILI9341_DrawFilledEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `int16_t` | Centro de la elipse |
| `rx` | `int16_t` | Radio horizontal en píxeles |
| `ry` | `int16_t` | Radio vertical en píxeles |
| `color` | `uint16_t` | Color RGB565 de relleno |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` si `rx` o `ry` son negativos, `ILI9341_ERROR` si falla la transmisión SPI.

---

#### `ILI9341_DrawArc()` - Dibujar Arco

Dibuja el contorno de un arco (sector de anillo delimitado por un radio exterior `r1` y uno interior `r2`) entre dos ángulos. Los ángulos se expresan en grados (0° = derecha, sentido horario) y se normalizan internamente con `fmodf()` al rango `[0, 360)`. Internamente traza los dos bordes rectos del sector y los dos bordes curvos (radio exterior e interior) mediante la función helper privada `ILI9341_FillArcHelper()`.

```c
ILI9341_Status_t ILI9341_DrawArc(int16_t x0, int16_t y0, int16_t r1, int16_t r2, float start, float end, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `int16_t` | Centro del arco |
| `r1` | `int16_t` | Radio exterior |
| `r2` | `int16_t` | Radio interior |
| `start` | `float` | Ángulo inicial en grados |
| `end` | `float` | Ángulo final en grados |
| `color` | `uint16_t` | Color RGB565 del contorno |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` si `r1` o `r2` son negativos, `ILI9341_ERROR` si falla la transmisión SPI.

---

#### `ILI9341_DrawFilledArc()` - Dibujar Arco Relleno

Rellena por completo el sector de anillo entre `start` y `end` usando la misma función helper privada `ILI9341_FillArcHelper()`, que recorre el cuadro delimitador fila por fila dibujando los tramos horizontales que caen dentro de la corona circular y del sector angular.

```c
ILI9341_Status_t ILI9341_DrawFilledArc(int16_t x0, int16_t y0, int16_t r1, int16_t r2, float start, float end, uint16_t color);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `int16_t` | Centro del arco |
| `r1` | `int16_t` | Radio exterior |
| `r2` | `int16_t` | Radio interior |
| `start` | `float` | Ángulo inicial en grados |
| `end` | `float` | Ángulo final en grados |
| `color` | `uint16_t` | Color RGB565 de relleno |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` si `r1` o `r2` son negativos, `ILI9341_ERROR` si falla la transmisión SPI.

---

#### `ILI9341_Putc()` - Renderizar Carácter

Renderiza un único carácter a partir de una definición de fuente `LCD_FontDef_t` usando acceso directo al registro `DR` del SPI. Si el carácter no cabe en la fila actual, pasa automáticamente a la siguiente.

```c
ILI9341_Status_t ILI9341_Putc(uint16_t x, uint16_t y, char c,
                                   LCD_FontDef_t* font,
                                   uint16_t foreground, uint16_t background);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x`, `y` | `uint16_t` | Esquina superior-izquierda de la celda |
| `c` | `char` | Carácter ASCII a renderizar (rango `' '` – `'~'`) |
| `font` | `LCD_FontDef_t*` | Puntero a la definición de la fuente |
| `foreground` | `uint16_t` | Color de los píxeles activos (RGB565) |
| `background` | `uint16_t` | Color del fondo de la celda (RGB565) |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` si `font` es NULL, `ILI9341_ERROR` si el SPI estaba ocupado o `ILI9341_TIMEOUT` si el bus se bloqueó.

---

#### `ILI9341_Puts()` - Renderizar Cadena

Renderiza una cadena terminada en nulo. Interpreta `'\n'` como salto de línea retornando a la posición `x` inicial con la que se llamó a `Puts()`. La secuencia `'\n'`+`'\r'` avanza a la siguiente línea y reinicia la columna a 0. Un `'\r'` aislado es ignorado.

```c
ILI9341_Status_t ILI9341_Puts(uint16_t x, uint16_t y, char* str,
                                   LCD_FontDef_t* font,
                                   uint16_t foreground, uint16_t background);
```

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` si `str` o `font` son NULL, `ILI9341_ERROR` o `ILI9341_TIMEOUT`.

---

#### `ILI9341_Printf()` - Renderizar Texto Formateado

Renderiza una cadena con formato estilo `printf`. Formatea los argumentos variádicos con `vsnprintf()` en un buffer interno en pila de `ILI9341_PRINTF_BUF_SIZE` bytes y delega el dibujo en `ILI9341_Puts()`. El uso de `vsnprintf()` con el tamaño del buffer garantiza que la cadena siempre quede terminada en nulo y sin desbordamiento, aunque el resultado se trunque.

```c
#ifndef ILI9341_PRINTF_BUF_SIZE
#define ILI9341_PRINTF_BUF_SIZE 128U   /* Redefinible antes de incluir el header */
#endif

ILI9341_Status_t ILI9341_Printf(uint16_t x, uint16_t y,
                                 LCD_FontDef_t* font,
                                 uint16_t foreground, uint16_t background,
                                 const char* fmt, ...);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x`, `y` | `uint16_t` | Esquina superior izquierda del primer carácter |
| `font` | `LCD_FontDef_t*` | Puntero a la definición de la fuente |
| `foreground` | `uint16_t` | Color de primer plano (RGB565) |
| `background` | `uint16_t` | Color de fondo (RGB565) |
| `fmt`, `...` | `const char*`, variádicos | Cadena de formato estilo printf y sus argumentos |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` si `fmt` o `font` son NULL, o si `vsnprintf()` falla, `ILI9341_ERROR` si falla la transmisión SPI.

---

#### `ILI9341_GetStringSize()` - Calcular Tamaño de Cadena

Calcula el bounding-box en píxeles de una cadena para una fuente dada, sin dibujar nada. Útil para centrar texto.

```c
void ILI9341_GetStringSize(char* str, LCD_FontDef_t* font,
                                uint16_t* width, uint16_t* height);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `str` | `char*` | Cadena terminada en nulo |
| `font` | `LCD_FontDef_t*` | Definición de la fuente |
| `width` | `uint16_t*` | Ancho total en píxeles (salida) |
| `height` | `uint16_t*` | Alto total en píxeles (salida) = `font->FontHeight` |

---

#### `ILI9341_PutsAligned()` / `ILI9341_PrintfAligned()` - Texto Alineado Horizontalmente

Renderizan una cadena alineada (izquierda, centro o derecha) dentro de una región horizontal `[x0, x1]`, sin que el usuario tenga que calcular manualmente el ancho con `ILI9341_GetStringSize()`. Pensadas para cadenas de una sola línea (títulos, etiquetas, valores numéricos); si la cadena es más ancha que la región, se alinea contra `x0`. `ILI9341_PrintfAligned()` añade formateo estilo `printf` (mismo mecanismo interno que `ILI9341_Printf()`) y delega en `ILI9341_PutsAligned()`.

```c
ILI9341_Status_t ILI9341_PutsAligned(uint16_t x0, uint16_t x1, uint16_t y,
                                      ILI9341_TextAlign_t align, char* str,
                                      LCD_FontDef_t* font,
                                      uint16_t foreground, uint16_t background);

ILI9341_Status_t ILI9341_PrintfAligned(uint16_t x0, uint16_t x1, uint16_t y,
                                        ILI9341_TextAlign_t align,
                                        LCD_FontDef_t* font,
                                        uint16_t foreground, uint16_t background,
                                        const char* fmt, ...);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0` | `uint16_t` | Borde izquierdo de la región de alineación |
| `x1` | `uint16_t` | Borde derecho de la región de alineación (`x1 >= x0`) |
| `y` | `uint16_t` | Coordenada Y superior izquierda del texto |
| `align` | `ILI9341_TextAlign_t` | Alineación deseada |
| `str` | `char*` | Cadena terminada en nulo (solo `PutsAligned`) |
| `font` | `LCD_FontDef_t*` | Puntero a la definición de la fuente |
| `foreground` | `uint16_t` | Color de primer plano (RGB565) |
| `background` | `uint16_t` | Color de fondo (RGB565) |
| `fmt`, `...` | `const char*`, variádicos | Cadena de formato estilo printf y sus argumentos (solo `PrintfAligned`) |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` si `str`/`fmt` o `font` son NULL, si `x1 < x0`, o si `vsnprintf()` falla (`PrintfAligned`), `ILI9341_ERROR` si falla la transmisión SPI.

---

#### `ILI9341_DisplayImage()` - Transferir Frame Buffer

Transfiere un frame buffer RGB565 de pantalla completa a la LCD mediante **DMA SPI en modo 16 bits** (dos tramos de 38 400 píxeles). El SPI se reconfigura temporalmente a 16 bits antes del DMA y se restaura a 8 bits al terminar; el resto de las funciones del driver no se ve afectado. El periférico serializa cada `uint16_t` MSB-first, produciendo el byte order big-endian esperado por el ILI9341 directamente desde el frame buffer sin reordenar bytes.

Cada palabra `uint32_t` del buffer contiene **dos píxeles RGB565**: el píxel de índice par en los bits `[15:0]` (word bajo) y el de índice impar en los bits `[31:16]` (word alto).

> [!NOTE]
> Requiere que **DMA2\_Stream6** esté configurado para SPI5\_TX con ancho Half Word en CubeMX (ver [Configuración DMA SPI TX](#configuración-dma-spi-tx-para-displayimage-y-flush)).

```c
ILI9341_Status_t ILI9341_DisplayImage(uint32_t image[IMG_TOTAL_BUF32]);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `image` | `uint32_t[38400]` | Frame buffer con `IMG_TOTAL_BUF32` = 38 400 palabras (153 600 bytes) |

**Retorna**: `ILI9341_OK` si todos los píxeles se enviaron, `ILI9341_NOT_INITIALIZED`, `ILI9341_ERROR` si el DMA no pudo iniciarse, `ILI9341_TIMEOUT` si el bus se bloqueó durante la transferencia.

---

#### Funciones de Frame Buffer fuera de Pantalla

Estas funciones realizan la misma operación que sus equivalentes directas, pero escriben en un `uint32_t image[IMG_TOTAL_BUF32]` en RAM en lugar de enviar datos por SPI. El formato de empaquetado es idéntico al requerido por `ILI9341_DisplayImage()`.

> [!NOTE]
> Las versiones `_ImageBuffer` de Putc y Puts **no reciben color de fondo**: solo dibujan los píxeles del trazo en primer plano, dejando el resto del buffer sin modificar (renderizado transparente). Usa `memset` o `DrawFilledRectangle_ImageBuffer` para limpiar el fondo antes de renderizar texto.

Todas retornan `ILI9341_Status_t` (`ILI9341_OK`, `ILI9341_INVALID_PARAM` o `ILI9341_ERROR`).

| Función | Descripción |
|---------|-------------|
| `ILI9341_Fill_ImageBuffer(color, image)` | Rellena el buffer completo con un color sólido (delega en `DrawFilledRectangle_ImageBuffer` sobre toda el área; DMA2D R2M si disponible) |
| `ILI9341_DrawPixel_ImageBuffer(x, y, color, image)` | Escribe un píxel en el buffer |
| `ILI9341_DrawLine_ImageBuffer(x0, y0, x1, y1, color, image)` | Dibuja una línea (Bresenham) |
| `ILI9341_DrawThickLine_ImageBuffer(x0, y0, x1, y1, thickness, color, image)` | Línea con grosor configurable |
| `ILI9341_DrawRectangle_ImageBuffer(x0, y0, x1, y1, color, image)` | Contorno de rectángulo |
| `ILI9341_DrawFilledRectangle_ImageBuffer(x0, y0, x1, y1, color, image)` | Rectángulo relleno (DMA2D R2M si disponible) |
| `ILI9341_DrawRoundRect_ImageBuffer(x0, y0, x1, y1, r, color, image)` | Contorno de rectángulo redondeado (Bresenham) |
| `ILI9341_DrawFilledRoundRect_ImageBuffer(x0, y0, x1, y1, r, color, image)` | Rectángulo redondeado relleno (franja central con DMA2D si disponible + arcos por CPU) |
| `ILI9341_DrawCircle_ImageBuffer(x0, y0, r, color, image)` | Contorno de círculo (Bresenham de punto medio) |
| `ILI9341_DrawFilledCircle_ImageBuffer(x0, y0, r, color, image)` | Círculo relleno |
| `ILI9341_DrawTriangle_ImageBuffer(x0, y0, x1, y1, x2, y2, color, image)` | Contorno de triángulo (tres llamadas a `DrawLine_ImageBuffer`) |
| `ILI9341_DrawFilledTriangle_ImageBuffer(x0, y0, x1, y1, x2, y2, color, image)` | Triángulo relleno (scanline, aritmética entera) |
| `ILI9341_Putc_ImageBuffer(x, y, c, font, fg, image)` | Carácter (sin fondo) |
| `ILI9341_Puts_ImageBuffer(x, y, str, font, fg, image)` | Cadena (sin fondo) |
| `ILI9341_Printf_ImageBuffer(x, y, font, fg, image, fmt, ...)` | Cadena con formato printf (sin fondo) |
| `ILI9341_PutsAligned_ImageBuffer(x0, x1, y, align, str, font, fg, image)` | Cadena alineada horizontalmente (sin fondo) |
| `ILI9341_PrintfAligned_ImageBuffer(x0, x1, y, align, font, fg, image, fmt, ...)` | Cadena con formato printf y alineada (sin fondo) |

---

#### `ILI9341_Printf_ImageBuffer()` - Texto Formateado en Buffer

Misma lógica que `ILI9341_Printf()` pero escribe en el frame buffer fuera de pantalla: formatea los argumentos variádicos con `vsnprintf()` en el buffer interno de `ILI9341_PRINTF_BUF_SIZE` bytes y delega en `ILI9341_Puts_ImageBuffer()` (sin color de fondo).

```c
ILI9341_Status_t ILI9341_Printf_ImageBuffer(uint16_t x, uint16_t y,
                                             LCD_FontDef_t* font, uint16_t foreground,
                                             uint32_t image[IMG_TOTAL_BUF32],
                                             const char* fmt, ...);
```

**Retorna**: `ILI9341_OK`, `ILI9341_INVALID_PARAM` si `fmt`, `font` o `image` son NULL, o si `vsnprintf()` falla.

---

#### `ILI9341_PutsAligned_ImageBuffer()` / `ILI9341_PrintfAligned_ImageBuffer()` - Texto Alineado en Buffer

Mismas variantes de alineación horizontal que `ILI9341_PutsAligned()` / `ILI9341_PrintfAligned()`, pero escriben en el frame buffer (sin color de fondo) delegando en `ILI9341_Puts_ImageBuffer()`.

```c
ILI9341_Status_t ILI9341_PutsAligned_ImageBuffer(uint16_t x0, uint16_t x1, uint16_t y,
                                                  ILI9341_TextAlign_t align, char* str,
                                                  LCD_FontDef_t* font, uint16_t foreground,
                                                  uint32_t image[IMG_TOTAL_BUF32]);

ILI9341_Status_t ILI9341_PrintfAligned_ImageBuffer(uint16_t x0, uint16_t x1, uint16_t y,
                                                    ILI9341_TextAlign_t align,
                                                    LCD_FontDef_t* font, uint16_t foreground,
                                                    uint32_t image[IMG_TOTAL_BUF32],
                                                    const char* fmt, ...);
```

**Retorna**: `ILI9341_OK`, `ILI9341_INVALID_PARAM` si `str`/`fmt`, `font` o `image` son NULL, si `x1 < x0`, o si `vsnprintf()` falla (`PrintfAligned_ImageBuffer`).

---

#### `ILI9341_DrawThickLine_ImageBuffer()` - Línea con Grosor en Buffer

Misma lógica que `ILI9341_DrawThickLine()` (rectángulo relleno para horizontales/verticales, trazos de Bresenham paralelos para diagonales) pero escribe directamente en el frame buffer, delegando en `ILI9341_DrawFilledRectangle_ImageBuffer()` y `ILI9341_DrawLine_ImageBuffer()` según el caso.

```c
ILI9341_Status_t ILI9341_DrawThickLine_ImageBuffer(uint16_t x0, uint16_t y0,
                                                    uint16_t x1, uint16_t y1,
                                                    uint16_t thickness, uint16_t color,
                                                    uint32_t image[IMG_TOTAL_BUF32]);
```

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` si `image` es NULL, `ILI9341_ERROR` si falla la transmisión SPI.

---

#### `ILI9341_DrawRoundRect_ImageBuffer()` - Rectángulo Redondeado en Buffer

Misma lógica que `ILI9341_DrawRoundRect()` pero escribe directamente en el frame buffer. `r` se recorta a `min(ancho, alto) / 2`; con `r = 0` delega en `ILI9341_DrawRectangle_ImageBuffer()`.

```c
ILI9341_Status_t ILI9341_DrawRoundRect_ImageBuffer(uint16_t x0, uint16_t y0,
                                                    uint16_t x1, uint16_t y1,
                                                    uint16_t r, uint16_t color,
                                                    uint32_t image[IMG_TOTAL_BUF32]);
```

**Retorna**: `ILI9341_OK` o `ILI9341_INVALID_PARAM` si `image` es NULL.

---

#### `ILI9341_DrawFilledRoundRect_ImageBuffer()` - Rectángulo Redondeado Relleno en Buffer

Misma lógica que `ILI9341_DrawFilledRoundRect()` pero sobre el frame buffer. La franja central se dibuja con `ILI9341_DrawFilledRectangle_ImageBuffer()` (que usa DMA2D R2M cuando está disponible); los arcos superior e inferior se rellenan con tramos horizontales de Bresenham vía CPU.

```c
ILI9341_Status_t ILI9341_DrawFilledRoundRect_ImageBuffer(uint16_t x0, uint16_t y0,
                                                          uint16_t x1, uint16_t y1,
                                                          uint16_t r, uint16_t color,
                                                          uint32_t image[IMG_TOTAL_BUF32]);
```

**Retorna**: `ILI9341_OK`, `ILI9341_INVALID_PARAM` si `image` es NULL, `ILI9341_ERROR` si falla la transferencia DMA2D de la franja central.

---

#### `ILI9341_DrawCircle_ImageBuffer()` - Círculo en Buffer

Misma lógica que `ILI9341_DrawCircle()` (algoritmo de punto medio de Bresenham con simetría de octantes) pero escribe directamente en el frame buffer mediante `DrawPixelClipped_ImageBuffer()`. Los píxeles se recortan a los límites fijos del panel.

```c
ILI9341_Status_t ILI9341_DrawCircle_ImageBuffer(int16_t x0, int16_t y0, int16_t r,
                                                 uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `int16_t` | Centro del círculo |
| `r` | `int16_t` | Radio en píxeles |
| `color` | `uint16_t` | Color RGB565 del contorno |
| `image` | `uint32_t[38400]` | Frame buffer destino |

**Retorna**: `ILI9341_OK`, `ILI9341_INVALID_PARAM` si `image` es NULL.

---

#### `ILI9341_DrawFilledCircle_ImageBuffer()` - Círculo Relleno en Buffer

Misma lógica que `ILI9341_DrawFilledCircle()` pero escribe directamente en el frame buffer: rellena cada fila visitada con un tramo horizontal (`DrawHSpanClipped_ImageBuffer()`) en lugar de graficar píxeles individuales.

```c
ILI9341_Status_t ILI9341_DrawFilledCircle_ImageBuffer(int16_t x0, int16_t y0, int16_t r,
                                                       uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `int16_t` | Centro del círculo |
| `r` | `int16_t` | Radio en píxeles |
| `color` | `uint16_t` | Color RGB565 de relleno |
| `image` | `uint32_t[38400]` | Frame buffer destino |

**Retorna**: `ILI9341_OK`, `ILI9341_INVALID_PARAM` si `image` es NULL.

---

#### `ILI9341_DrawTriangle_ImageBuffer()` - Triángulo en Buffer

Dibuja el contorno de un triángulo en el frame buffer trazando las tres aristas con Bresenham (delega en `ILI9341_DrawLine_ImageBuffer()`).

```c
ILI9341_Status_t ILI9341_DrawTriangle_ImageBuffer(uint16_t x0, uint16_t y0,
                                                   uint16_t x1, uint16_t y1,
                                                   uint16_t x2, uint16_t y2,
                                                   uint16_t color,
                                                   uint32_t image[IMG_TOTAL_BUF32]);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `uint16_t` | Primer vértice |
| `x1`, `y1` | `uint16_t` | Segundo vértice |
| `x2`, `y2` | `uint16_t` | Tercer vértice |
| `color` | `uint16_t` | Color RGB565 del contorno |
| `image` | `uint32_t[38400]` | Frame buffer destino |

**Retorna**: `ILI9341_OK`, `ILI9341_INVALID_PARAM` si `image` es NULL.

---

#### `ILI9341_DrawFilledTriangle_ImageBuffer()` - Triángulo Relleno en Buffer

Misma lógica que `ILI9341_DrawFilledTriangle()` pero escribe directamente en el frame buffer. El relleno usa aritmética `int32_t` pura (sin punto flotante) e internamente llama a la función helper privada `DrawHSpanClipped_ImageBuffer()`.

```c
ILI9341_Status_t ILI9341_DrawFilledTriangle_ImageBuffer(uint16_t x0, uint16_t y0,
                                                         uint16_t x1, uint16_t y1,
                                                         uint16_t x2, uint16_t y2,
                                                         uint16_t color,
                                                         uint32_t image[IMG_TOTAL_BUF32]);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `x0`, `y0` | `uint16_t` | Primer vértice |
| `x1`, `y1` | `uint16_t` | Segundo vértice |
| `x2`, `y2` | `uint16_t` | Tercer vértice |
| `color` | `uint16_t` | Color RGB565 del relleno |
| `image` | `uint32_t[38400]` | Frame buffer destino |

**Retorna**: `ILI9341_OK`, `ILI9341_INVALID_PARAM` si `image` es NULL.

---

#### `ILI9341_DrawEllipse_ImageBuffer()` - Elipse en Buffer

Misma lógica que `ILI9341_DrawEllipse()` (algoritmo de punto medio de Zingl) pero escribe directamente en el frame buffer mediante `DrawPixelClipped_ImageBuffer()`. Los casos degenerados (`rx == 0` o `ry == 0`) delegan en `ILI9341_DrawLine_ImageBuffer()`.

```c
ILI9341_Status_t ILI9341_DrawEllipse_ImageBuffer(int16_t x0, int16_t y0, int16_t rx, int16_t ry,
                                                  uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
```

**Retorna**: `ILI9341_OK`, `ILI9341_INVALID_PARAM` si `image` es NULL o si `rx`/`ry` son negativos.

---

#### `ILI9341_DrawFilledEllipse_ImageBuffer()` - Elipse Rellena en Buffer

Misma lógica que `ILI9341_DrawFilledEllipse()`, pero rellena cada fila con `DrawHSpanClipped_ImageBuffer()` en lugar de graficar píxeles individuales.

```c
ILI9341_Status_t ILI9341_DrawFilledEllipse_ImageBuffer(int16_t x0, int16_t y0, int16_t rx, int16_t ry,
                                                        uint16_t color, uint32_t image[IMG_TOTAL_BUF32]);
```

**Retorna**: `ILI9341_OK`, `ILI9341_INVALID_PARAM` si `image` es NULL o si `rx`/`ry` son negativos.

---

#### `ILI9341_DrawArc_ImageBuffer()` - Arco en Buffer

Misma lógica que `ILI9341_DrawArc()`, pero la función helper privada `ILI9341_FillArcHelper()` escribe en el frame buffer (`DrawHSpanClipped_ImageBuffer()`) en lugar de enviar los tramos por SPI.

```c
ILI9341_Status_t ILI9341_DrawArc_ImageBuffer(int16_t x0, int16_t y0, int16_t r1, int16_t r2,
                                              float start, float end, uint16_t color,
                                              uint32_t image[IMG_TOTAL_BUF32]);
```

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` si `image` es NULL o si `r1`/`r2` son negativos.

---

#### `ILI9341_DrawFilledArc_ImageBuffer()` - Arco Relleno en Buffer

Misma lógica que `ILI9341_DrawFilledArc()`, aplicada sobre el frame buffer mediante `ILI9341_FillArcHelper()`.

```c
ILI9341_Status_t ILI9341_DrawFilledArc_ImageBuffer(int16_t x0, int16_t y0, int16_t r1, int16_t r2,
                                                    float start, float end, uint16_t color,
                                                    uint32_t image[IMG_TOTAL_BUF32]);
```

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` si `image` es NULL o si `r1`/`r2` son negativos.

---

#### `ILI9341_BlitImage()` - Copiar Imagen con DMA2D *(solo DMA2D)*

Copia una imagen RGB565 al frame buffer usando DMA2D en modo memoria a memoria (M2M). La imagen se recorta automáticamente si sobresale del borde de la pantalla. Requiere que `ILI9341_Init()` haya sido invocado con un handle DMA2D válido.

```c
#ifdef HAL_DMA2D_MODULE_ENABLED
ILI9341_Status_t ILI9341_BlitImage(const uint16_t* src, uint16_t x0, uint16_t y0,
                                    uint16_t img_w, uint16_t img_h,
                                    uint32_t* framebuffer);
#endif
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `src` | `const uint16_t*` | Puntero a la imagen fuente en formato RGB565 |
| `x0`, `y0` | `uint16_t` | Esquina superior izquierda de destino en el frame buffer |
| `img_w` | `uint16_t` | Ancho de la imagen fuente en píxeles |
| `img_h` | `uint16_t` | Alto de la imagen fuente en píxeles |
| `framebuffer` | `uint32_t*` | Frame buffer destino (`IMG_TOTAL_BUF32` palabras) |

**Retorna**: `ILI9341_OK` si la copia fue exitosa (incluyendo el caso en que `x0`/`y0` están fuera de pantalla, que se ignora sin error), `ILI9341_INVALID_PARAM` si `src` o `framebuffer` son NULL o el handle DMA2D no fue inyectado, `ILI9341_ERROR` si la transferencia DMA2D falla.

---

#### `ILI9341_SetFrameBuffers()` - Registrar Buffers de Doble Buffer

Registra los dos buffers (front/back, cada uno `IMG_TOTAL_BUF32` palabras `uint32_t`) usados por el modo de doble buffer con pipelining DMA (`ILI9341_Flush()`, `ILI9341_Sync()`, `ILI9341_GetFrameBuffer()`). El usuario reserva la memoria donde quiera (RAM interna, RAM externa, etc.); se limpian a cero al registrarse.

```c
ILI9341_Status_t ILI9341_SetFrameBuffers(uint32_t* front, uint32_t* back);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `front` | `uint32_t*` | Buffer que se transmite a la LCD |
| `back` | `uint32_t*` | Buffer sobre el que dibuja la CPU |

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` si solo uno de los dos punteros es NULL. Pasar `NULL` en ambos deshabilita el modo doble buffer.

---

#### `ILI9341_Flush()` - Presentar Frame (Doble Buffer)

Espera el DMA anterior, intercambia front/back e inicia el envío por DMA del frame recién dibujado sin bloquear. Requiere haber llamado antes a `ILI9341_SetFrameBuffers()`.

```c
ILI9341_Status_t ILI9341_Flush(void);
```

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` (si no se registraron buffers), `ILI9341_TIMEOUT` o `ILI9341_ERROR`.

---

#### `ILI9341_Sync()` - Sincronizar DMA con el bus SPI

Espera a que concluya el DMA en curso y restaura el bus SPI al modo 8 bits. Si no hay DMA activo retorna inmediatamente sin efecto.

Debe llamarse al salir del modo de doble buffer antes de usar funciones de dibujo directo en pantalla (`ILI9341_Fill`, `ILI9341_DrawPixel`, `ILI9341_DrawFilledRectangle`, etc.). Sin esta llamada, el SPI permanece en modo 16 bits y CS queda bajo, lo que corrompe los comandos enviados por las funciones directas.

```c
ILI9341_Status_t ILI9341_Sync(void);
```

**Uso típico al salir de una animación:**

```c
/* Bucle de animación */
for (int frame = 0; frame < N; frame++) {
    uint32_t* fb = ILI9341_GetFrameBuffer();   /* puntero fresco en cada frame */
    draw_scene(fb);
    ILI9341_Flush();
}

/* Al salir del modo doble buffer, sincronizar antes de usar SPI directo */
ILI9341_Sync();
ILI9341_Fill(ILI9341_COLOR_BLACK);   /* ahora es seguro */
```

**Retorna**: `ILI9341_OK` si el bus quedó libre correctamente, `ILI9341_NOT_INITIALIZED` si el driver no está inicializado, `ILI9341_TIMEOUT` si el DMA no terminó en 5 000 ms.

---

#### `ILI9341_GetFrameBuffer()` - Obtener Puntero al Back Buffer

Retorna el puntero al back buffer activo registrado con `ILI9341_SetFrameBuffers()`, compatible con todas las funciones `*_ImageBuffer()`. El puntero cambia tras cada llamada a `ILI9341_Flush()`. Retorna `NULL` si no se registraron buffers o el driver no está inicializado.

```c
uint32_t* ILI9341_GetFrameBuffer(void);
```

---

#### `ILI9341_TP_Config()` - Configurar Panel Táctil

Verifica el ID del chip STMPE811, realiza un reset por software y configura el ADC y el controlador táctil con los parámetros óptimos para el panel resistivo.

```c
ILI9341_Status_t ILI9341_TP_Config(void);
```

**Retorna**: `ILI9341_OK` si el dispositivo fue detectado y configurado, `ILI9341_NOT_INITIALIZED` si el driver LCD no está inicializado, `ILI9341_ERROR` si el ID del chip no coincide con `0x0811` o si cualquier operación I2C de configuración falla.

---

#### `ILI9341_TP_GetState()` - Obtener Estado del Toque

Lee el estado completo del panel táctil con filtro de histeresis de 5 puntos para evitar jitter. Restablece el FIFO del STMPE811 al finalizar.

```c
TP_STATE* ILI9341_TP_GetState(void);
```

**Retorna**: Puntero a la estructura `TP_STATE` interna con valores actualizados, o `NULL` si el driver no está inicializado.

---

## Colores Predefinidos

La librería incluye **147 macros de colores** en formato **RGB565**, generadas a partir de la paleta estándar de nombres de color **X11/CSS** (p. ej. `ILI9341_COLOR_CORNFLOWERBLUE`, `ILI9341_COLOR_MEDIUMSEAGREEN`, `ILI9341_COLOR_DARKSLATEGRAY`). Todas se definen internamente con la macro `RGB565()`.

```c
/* Convierte componentes R, G, B de 8 bits (0-255) a un color RGB565 de 16 bits */
#define RGB565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

/* Expande un color RGB565 de 16 bits de vuelta a RGB888 de 24 bits (con pérdida de precisión) */
#define RGB16TO24(c) ((((uint32_t)(c) & 0xF800) << 8) | (((c) & 0x07E0) << 5) | (((c) & 0x1F) << 3))
```

> [!WARNING]
> **Cambio incompatible respecto a v1.2.0**: `ILI9341_COLOR_GREEN`, `ILI9341_COLOR_ORANGE`, `ILI9341_COLOR_MAGENTA`, `ILI9341_COLOR_GRAY`/`GREY` y `ILI9341_COLOR_BROWN` ahora usan los tonos estándar X11/CSS (valores distintos a los de v1.2.0), y las macros `ILI9341_COLOR_GREEN2` / `ILI9341_COLOR_BLUE2` fueron eliminadas. Ver [Changelog — Migration notes](#migration-notes) antes de actualizar.

<details>
<summary>Ver los 147 colores predefinidos</summary>

| Macro | Valor RGB565 |
|-------|--------------|
| `ILI9341_COLOR_ALICEBLUE` | `0xF7DF` |
| `ILI9341_COLOR_ANTIQUEWHITE` | `0xFF7B` |
| `ILI9341_COLOR_AQUA` | `0x07FF` |
| `ILI9341_COLOR_AQUAMARINE` | `0x87FB` |
| `ILI9341_COLOR_AZURE` | `0xF7FF` |
| `ILI9341_COLOR_BEIGE` | `0xFFBC` |
| `ILI9341_COLOR_BISQUE` | `0xFF39` |
| `ILI9341_COLOR_BLACK` | `0x0000` |
| `ILI9341_COLOR_BLANCHEDALMOND` | `0xFF7A` |
| `ILI9341_COLOR_BLUE` | `0x001F` |
| `ILI9341_COLOR_BLUEVIOLET` | `0x897C` |
| `ILI9341_COLOR_BROWN` | `0xA965` |
| `ILI9341_COLOR_BURLYWOOD` | `0xE5D1` |
| `ILI9341_COLOR_CADETBLUE` | `0x6514` |
| `ILI9341_COLOR_CHARTREUSE` | `0x87E0` |
| `ILI9341_COLOR_CHOCOLATE` | `0xD344` |
| `ILI9341_COLOR_CORAL` | `0xFC0A` |
| `ILI9341_COLOR_CORNFLOWERBLUE` | `0x6CBE` |
| `ILI9341_COLOR_CORNSILK` | `0xFFDC` |
| `ILI9341_COLOR_CRIMSON` | `0xE0A8` |
| `ILI9341_COLOR_CYAN` | `0x07FF` |
| `ILI9341_COLOR_DARKBLUE` | `0x0011` |
| `ILI9341_COLOR_DARKCYAN` | `0x0471` |
| `ILI9341_COLOR_DARKGOLDENROD` | `0xBC41` |
| `ILI9341_COLOR_DARKGRAY` | `0xAD55` |
| `ILI9341_COLOR_DARKGREEN` | `0x0320` |
| `ILI9341_COLOR_DARKGREY` | `0xAD55` |
| `ILI9341_COLOR_DARKKHAKI` | `0xC5CD` |
| `ILI9341_COLOR_DARKMAGENTA` | `0x8811` |
| `ILI9341_COLOR_DARKOLIVEGREEN` | `0x5B66` |
| `ILI9341_COLOR_DARKORANGE` | `0xFC60` |
| `ILI9341_COLOR_DARKORCHID` | `0x99BA` |
| `ILI9341_COLOR_DARKRED` | `0x8800` |
| `ILI9341_COLOR_DARKSALMON` | `0xECCF` |
| `ILI9341_COLOR_DARKSEAGREEN` | `0x95F2` |
| `ILI9341_COLOR_DARKSLATEBLUE` | `0x49F1` |
| `ILI9341_COLOR_DARKSLATEGRAY` | `0x328A` |
| `ILI9341_COLOR_DARKSLATEGREY` | `0x328A` |
| `ILI9341_COLOR_DARKTURQUOISE` | `0x069A` |
| `ILI9341_COLOR_DARKVIOLET` | `0x981A` |
| `ILI9341_COLOR_DEEPPINK` | `0xF8B2` |
| `ILI9341_COLOR_DEEPSKYBLUE` | `0x061F` |
| `ILI9341_COLOR_DIMGRAY` | `0x6B4D` |
| `ILI9341_COLOR_DIMGREY` | `0x6B4D` |
| `ILI9341_COLOR_DODGERBLUE` | `0x249F` |
| `ILI9341_COLOR_FIREBRICK` | `0xB124` |
| `ILI9341_COLOR_FLORALWHITE` | `0xFFFE` |
| `ILI9341_COLOR_FORESTGREEN` | `0x2464` |
| `ILI9341_COLOR_FUCHSIA` | `0xF81F` |
| `ILI9341_COLOR_GAINSBORO` | `0xE6FC` |
| `ILI9341_COLOR_GHOSTWHITE` | `0xFFDF` |
| `ILI9341_COLOR_GOLD` | `0xFEC0` |
| `ILI9341_COLOR_GOLDENROD` | `0xDD24` |
| `ILI9341_COLOR_GRAY` | `0x8410` |
| `ILI9341_COLOR_GREEN` | `0x0400` |
| `ILI9341_COLOR_GREENYELLOW` | `0xB7E6` |
| `ILI9341_COLOR_GREY` | `0x8410` |
| `ILI9341_COLOR_HONEYDEW` | `0xF7FE` |
| `ILI9341_COLOR_HOTPINK` | `0xFB57` |
| `ILI9341_COLOR_INDIANRED` | `0xD2EC` |
| `ILI9341_COLOR_INDIGO` | `0x4810` |
| `ILI9341_COLOR_IVORY` | `0xFFFE` |
| `ILI9341_COLOR_KHAKI` | `0xF752` |
| `ILI9341_COLOR_LAVENDER` | `0xEF5F` |
| `ILI9341_COLOR_LAVENDERBLUSH` | `0xFF9F` |
| `ILI9341_COLOR_LAWNGREEN` | `0x87E0` |
| `ILI9341_COLOR_LEMONCHIFFON` | `0xFFFA` |
| `ILI9341_COLOR_LIGHTBLUE` | `0xB6DD` |
| `ILI9341_COLOR_LIGHTCORAL` | `0xF410` |
| `ILI9341_COLOR_LIGHTCYAN` | `0xE7FF` |
| `ILI9341_COLOR_LIGHTGOLDENRODYELLOW` | `0xFFFA` |
| `ILI9341_COLOR_LIGHTGRAY` | `0xD6BA` |
| `ILI9341_COLOR_LIGHTGREEN` | `0x9792` |
| `ILI9341_COLOR_LIGHTGREY` | `0xD6BA` |
| `ILI9341_COLOR_LIGHTPINK` | `0xFDD8` |
| `ILI9341_COLOR_LIGHTSALMON` | `0xFD0F` |
| `ILI9341_COLOR_LIGHTSEAGREEN` | `0x25B5` |
| `ILI9341_COLOR_LIGHTSKYBLUE` | `0x8E9F` |
| `ILI9341_COLOR_LIGHTSLATEGRAY` | `0x7C53` |
| `ILI9341_COLOR_LIGHTSLATEGREY` | `0x7C53` |
| `ILI9341_COLOR_LIGHTSTEELBLUE` | `0xB63C` |
| `ILI9341_COLOR_LIGHTYELLOW` | `0xFFFC` |
| `ILI9341_COLOR_LIME` | `0x07E0` |
| `ILI9341_COLOR_LIMEGREEN` | `0x3666` |
| `ILI9341_COLOR_LINEN` | `0xFF9D` |
| `ILI9341_COLOR_MAGENTA` | `0xF81F` |
| `ILI9341_COLOR_MAROON` | `0x8000` |
| `ILI9341_COLOR_MEDIUMAQUAMARINE` | `0x6E75` |
| `ILI9341_COLOR_MEDIUMBLUE` | `0x001A` |
| `ILI9341_COLOR_MEDIUMORCHID` | `0xBABA` |
| `ILI9341_COLOR_MEDIUMPURPLE` | `0x939B` |
| `ILI9341_COLOR_MEDIUMSEAGREEN` | `0x45AE` |
| `ILI9341_COLOR_MEDIUMSLATEBLUE` | `0x7B5E` |
| `ILI9341_COLOR_MEDIUMSPRINGGREEN` | `0x07F3` |
| `ILI9341_COLOR_MEDIUMTURQUOISE` | `0x4E9A` |
| `ILI9341_COLOR_MEDIUMVIOLETRED` | `0xC8B1` |
| `ILI9341_COLOR_MIDNIGHTBLUE` | `0x18CE` |
| `ILI9341_COLOR_MINTCREAM` | `0xFFFF` |
| `ILI9341_COLOR_MISTYROSE` | `0xFF3C` |
| `ILI9341_COLOR_MOCCASIN` | `0xFF37` |
| `ILI9341_COLOR_NAVAJOWHITE` | `0xFF16` |
| `ILI9341_COLOR_NAVY` | `0x0010` |
| `ILI9341_COLOR_OLDLACE` | `0xFFBD` |
| `ILI9341_COLOR_OLIVE` | `0x8400` |
| `ILI9341_COLOR_OLIVEDRAB` | `0x6C84` |
| `ILI9341_COLOR_ORANGE` | `0xFD20` |
| `ILI9341_COLOR_ORANGERED` | `0xFA20` |
| `ILI9341_COLOR_ORCHID` | `0xDB9B` |
| `ILI9341_COLOR_PALEGOLDENROD` | `0xF755` |
| `ILI9341_COLOR_PALEGREEN` | `0x9FF3` |
| `ILI9341_COLOR_PALETURQUOISE` | `0xB79E` |
| `ILI9341_COLOR_PALEVIOLETRED` | `0xDB92` |
| `ILI9341_COLOR_PAPAYAWHIP` | `0xFF9B` |
| `ILI9341_COLOR_PEACHPUFF` | `0xFEF7` |
| `ILI9341_COLOR_PERU` | `0xD428` |
| `ILI9341_COLOR_PINK` | `0xFE19` |
| `ILI9341_COLOR_PLUM` | `0xE51C` |
| `ILI9341_COLOR_POWDERBLUE` | `0xB71D` |
| `ILI9341_COLOR_PURPLE` | `0x8010` |
| `ILI9341_COLOR_RED` | `0xF800` |
| `ILI9341_COLOR_ROSYBROWN` | `0xC492` |
| `ILI9341_COLOR_ROYALBLUE` | `0x435C` |
| `ILI9341_COLOR_SADDLEBROWN` | `0x8A22` |
| `ILI9341_COLOR_SALMON` | `0xFC0E` |
| `ILI9341_COLOR_SANDYBROWN` | `0xFD2C` |
| `ILI9341_COLOR_SEAGREEN` | `0x346B` |
| `ILI9341_COLOR_SEASHELL` | `0xFFBE` |
| `ILI9341_COLOR_SIENNA` | `0xA2A6` |
| `ILI9341_COLOR_SILVER` | `0xC618` |
| `ILI9341_COLOR_SKYBLUE` | `0x8E9D` |
| `ILI9341_COLOR_SLATEBLUE` | `0x6AFA` |
| `ILI9341_COLOR_SLATEGRAY` | `0x7412` |
| `ILI9341_COLOR_SLATEGREY` | `0x7412` |
| `ILI9341_COLOR_SNOW` | `0xFFFF` |
| `ILI9341_COLOR_SPRINGGREEN` | `0x07F0` |
| `ILI9341_COLOR_STEELBLUE` | `0x4C37` |
| `ILI9341_COLOR_TAN` | `0xD5B2` |
| `ILI9341_COLOR_TEAL` | `0x0410` |
| `ILI9341_COLOR_THISTLE` | `0xDE1B` |
| `ILI9341_COLOR_TOMATO` | `0xFB29` |
| `ILI9341_COLOR_TURQUOISE` | `0x471A` |
| `ILI9341_COLOR_VIOLET` | `0xF43E` |
| `ILI9341_COLOR_WHEAT` | `0xFF16` |
| `ILI9341_COLOR_WHITE` | `0xFFFF` |
| `ILI9341_COLOR_WHITESMOKE` | `0xFFBF` |
| `ILI9341_COLOR_YELLOW` | `0xFFE0` |
| `ILI9341_COLOR_YELLOWGREEN` | `0x9E66` |

</details>

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](../../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [2.0.0] - 08-07-2026

Versión que generaliza el driver, antes específico de la tarjeta STM32F429-Discovery, para que sea reutilizable en cualquier placa/MCU STM32 con HAL. Incluye cambios incompatibles con versiones anteriores.

#### Added

- **Pines CS, RESET y D/C parametrizables**: `ILI9341_Init()` ahora recibe `csPort`/`csPin`, `rstPort`/`rstPin` y `dcPort`/`dcPin` como parámetros. Internamente se guardan en variables estáticas referenciadas por las macros privadas `ILI9341_CS_SET/RESET`, `ILI9341_RST_SET/RESET` y `ILI9341_WRX_SET/RESET` (ya no son macros públicas con pines fijos en el header).
- **`ILI9341_SetFrameBuffers()`**: nueva función que registra un par de buffers (front/back, `IMG_TOTAL_BUF32` palabras cada uno) reservados por el usuario en cualquier memoria, para el modo de doble buffer con pipelining DMA. Reemplaza la reserva automática que antes hacía `ILI9341_Init()` sobre la SDRAM.
- Declaraciones que faltaban en el header (compilaban solo por definirse antes de su primer uso, pero no estaban expuestas como API pública): todas las funciones `*_ImageBuffer()`, `ILI9341_BlitImage()`, `ILI9341_Flush()`, `ILI9341_Sync()` y `ILI9341_GetFrameBuffer()`.

#### Changed

- **Todas las funciones `*_ImageBuffer()` dejan de depender de `HAL_SDRAM_MODULE_ENABLED`**: ya recibían el buffer destino como parámetro (`uint32_t image[IMG_TOTAL_BUF32]`), así que ahora están siempre disponibles y pueden apuntar a cualquier memoria (RAM interna, RAM externa, etc.), no solo SDRAM.
- **`ILI9341_Flush()` / `ILI9341_Sync()` / `ILI9341_GetFrameBuffer()`** ya no requieren SDRAM: operan sobre los buffers registrados con `ILI9341_SetFrameBuffers()`, en la memoria que el usuario elija.
- Archivos renombrados: `ILI9341_Disc1.c`/`ILI9341_Disc1.h` → `ILI9341.c`/`ILI9341.h`; include guard `ILI9341_DISC1_H` → `ILI9341_H`.

#### Removed

- **Soporte de SDRAM** (`HAL_SDRAM_MODULE_ENABLED`): se elimina por completo la gestión del periférico FMC-SDRAM — `SDRAM_Initialization_Sequence()`, el handle `hsdram` en `ILI9341_Init()`, las direcciones fijas `ILI9341_SDRAM_BASE`/`ILI9341_SDRAM_FB_SIZE` y la verificación de tamaño contra el chip IS42S16400J. El frame buffer de doble buffer ahora se registra con `ILI9341_SetFrameBuffers()` sobre memoria provista por el usuario.
- Macros públicas de pines fijos (`ILI9341_RST_SET/RESET`, `ILI9341_CS_SET/RESET`, `ILI9341_WRX_SET/RESET` en el header) — sustituidas por los parámetros de pin de `ILI9341_Init()`.

#### Migration notes

- Actualiza las llamadas a `ILI9341_Init()` para incluir los seis parámetros de pines (`csPort, csPin, rstPort, rstPin, dcPort, dcPin`) justo después de `hspi`, y elimina el argumento `hsdram` si lo usabas.
- Si usabas el frame buffer en SDRAM (`hsdram` en `Init()` + `ILI9341_Flush()`/`ILI9341_GetFrameBuffer()`), reserva tú mismo dos buffers `uint32_t[IMG_TOTAL_BUF32]` (en SDRAM si tu MCU la tiene y la configuras aparte, o en RAM interna) y llama a `ILI9341_SetFrameBuffers(front, back)` tras `ILI9341_Init()`. El resto del flujo (`ILI9341_GetFrameBuffer()`, `ILI9341_Flush()`, `ILI9341_Sync()`) no cambia.
- Reemplaza `#include "ILI9341_Disc1.h"` por `#include "ILI9341.h"` y renombra los archivos copiados a tu proyecto.

---

### [1.4.0] - 07-07-2026

#### Added

- **Renderizado de texto formateado** — `ILI9341_Printf()` y `ILI9341_Printf_ImageBuffer()` *(solo SDRAM)*: renderizan una cadena con formato estilo `printf`, formateando los argumentos variádicos con `vsnprintf()` en un buffer interno en pila de tamaño configurable `ILI9341_PRINTF_BUF_SIZE` (128 bytes por defecto, redefinible antes de incluir el header) y delegando el dibujo en `ILI9341_Puts()` / `ILI9341_Puts_ImageBuffer()`. El resultado se trunca de forma segura si excede el buffer.
- **Alineación horizontal de texto** — nuevo tipo `ILI9341_TextAlign_t` (`ILI9341_ALIGN_LEFT` / `ILI9341_ALIGN_CENTER` / `ILI9341_ALIGN_RIGHT`) y cuatro nuevas funciones que posicionan una cadena dentro de una región horizontal `[x0, x1]` sin que el usuario tenga que calcular el ancho manualmente con `ILI9341_GetStringSize()`:
  - `ILI9341_PutsAligned()` / `ILI9341_PrintfAligned()`: sobre pantalla.
  - `ILI9341_PutsAligned_ImageBuffer()` / `ILI9341_PrintfAligned_ImageBuffer()` *(solo SDRAM)*: sobre el frame buffer fuera de pantalla.
  - Función privada `ILI9341_AlignedX()`: deriva la coordenada X de arranque según la alineación solicitada; si la cadena es más ancha que la región, se alinea contra `x0`.
- **Líneas con grosor** — `ILI9341_DrawThickLine()` y `ILI9341_DrawThickLine_ImageBuffer()` *(solo SDRAM)*: dibujan una línea con grosor configurable. Las líneas horizontales y verticales se rellenan con un único rectángulo recortado a pantalla (`ILI9341_DrawFilledRectangle()` / `_ImageBuffer()`); las diagonales se aproximan trazando `thickness` líneas de Bresenham paralelas, desplazadas sobre la normal del segmento y centradas en la línea original. Con `thickness <= 1` delegan directamente en `ILI9341_DrawLine()` / `_ImageBuffer()`.
- **`ILI9341_Color565()`**: nueva función pública que convierte una componente de color RGB888 (8 bits por canal) a RGB565, equivalente a la macro `RGB565(r, g, b)` pero utilizable con valores calculados en tiempo de ejecución.
- Nuevos includes en `ILI9341_Disc1.h`: `<stdarg.h>` y `<stdio.h>` (soporte de `va_list`/`vsnprintf()` para las funciones `Printf*`).

---

### [1.3.0] - 03-07-2026

#### Added

- **Paleta de colores X11/CSS completa**: los 13 colores originales se reemplazan por **147 macros** `ILI9341_COLOR_*` correspondientes a la paleta estándar de nombres de color X11/CSS (p. ej. `ILI9341_COLOR_CORNFLOWERBLUE`, `ILI9341_COLOR_MEDIUMSEAGREEN`, `ILI9341_COLOR_DARKSLATEGRAY`), todas generadas con la nueva macro `RGB565()`.
  - `RGB565(r, g, b)`: antes solo aparecía como snippet de ejemplo en este README; ahora es una macro pública definida en `ILI9341_Disc1.h` y usada internamente para construir toda la paleta.
  - `RGB16TO24(c)`: nueva macro que expande un color RGB565 de 16 bits de vuelta a un valor RGB888 de 24 bits.
- **Líneas rápidas** — `ILI9341_DrawFastVLine()` y `ILI9341_DrawFastHLine()`: nuevas primitivas que dibujan líneas verticales/horizontales delegando directamente en `ILI9341_DrawFilledRectangle()`, sin pasar por el algoritmo de Bresenham. Aceptan alto/ancho negativo (se normaliza invirtiendo el punto de partida) y recortan automáticamente a los límites de la pantalla.
- **Elipses** — cuatro nuevas primitivas basadas en una implementación entera del algoritmo de punto medio (variante de Zingl):
  - `ILI9341_DrawEllipse()` / `ILI9341_DrawFilledEllipse()`: contorno y relleno sobre pantalla (el relleno usa `DrawHSpanClipped()` por fila).
  - `ILI9341_DrawEllipse_ImageBuffer()` / `ILI9341_DrawFilledEllipse_ImageBuffer()` *(solo SDRAM)*: mismas versiones sobre el frame buffer fuera de pantalla.
  - Los casos degenerados (`rx == 0` o `ry == 0`) delegan en `ILI9341_DrawFastVLine()`/`ILI9341_DrawFastHLine()` (o en `ILI9341_DrawLine_ImageBuffer()` en las variantes de buffer).
- **Arcos (sectores de anillo)** — cuatro nuevas primitivas para dibujar porciones de corona circular entre dos ángulos (0° = derecha, sentido horario, normalizados con `fmodf()`):
  - `ILI9341_DrawArc()` / `ILI9341_DrawFilledArc()`: contorno y relleno sobre pantalla.
  - `ILI9341_DrawArc_ImageBuffer()` / `ILI9341_DrawFilledArc_ImageBuffer()` *(solo SDRAM)*: mismas versiones sobre el frame buffer.
  - Función privada `ILI9341_FillArcHelper()`: recorre el cuadro delimitador del arco fila por fila y dibuja los tramos horizontales que caen dentro del sector angular `[start, end]` y la corona `[r2, r1]`, usando pendientes trigonométricas (`sinf`/`cosf`) para los bordes rectos. Es compartida por las cuatro funciones públicas de arco, tanto para el contorno (bordes rectos y curvos) como para el relleno completo.
- **`ILI9341_Fill_ImageBuffer()`** *(solo SDRAM)*: nueva función que rellena el frame buffer completo con un color sólido delegando en `ILI9341_DrawFilledRectangle_ImageBuffer()` sobre el área total del panel (hereda DMA2D R2M cuando está disponible).
- **`ILI9341_DrawCircle_ImageBuffer()`** *(solo SDRAM)*: nueva función que dibuja el contorno de un círculo directamente en el frame buffer, con la misma lógica de punto medio de Bresenham que `ILI9341_DrawCircle()`.
- Nuevos includes en `ILI9341_Disc1.h`: `<math.h>` (funciones trigonométricas para los arcos), `<float.h>` (`FLT_EPSILON` para comparar ángulos) y `<stdbool.h>` (tipo `bool` usado en las nuevas primitivas).

#### Changed

- **Valores de color modificados** para alinear la paleta con el estándar X11/CSS: `ILI9341_COLOR_GREEN` (`0x07E0` → `0x0400`), `ILI9341_COLOR_ORANGE` (`0xFBE4` → `0xFD20`), `ILI9341_COLOR_MAGENTA` (`0xA254` → `0xF81F`), `ILI9341_COLOR_GRAY`/`GREY` (`0x7BEF` → `0x8410`) y `ILI9341_COLOR_BROWN` (`0xBBCA` → `0xA965`). `ILI9341_COLOR_WHITE`, `BLACK`, `RED`, `BLUE`, `YELLOW` y `CYAN` conservan su valor anterior.

#### Removed

- `ILI9341_COLOR_GREEN2` (`0xB723`) y `ILI9341_COLOR_BLUE2` (`0x051D`) — reemplazadas por el conjunto completo de la paleta X11 (usa, por ejemplo, `ILI9341_COLOR_DARKGREEN`/`ILI9341_COLOR_FORESTGREEN` y `ILI9341_COLOR_DARKBLUE`/`ILI9341_COLOR_NAVY`/`ILI9341_COLOR_MIDNIGHTBLUE` en su lugar).

#### Migration notes

- Si tu proyecto usa `ILI9341_COLOR_GREEN`, `ILI9341_COLOR_ORANGE`, `ILI9341_COLOR_MAGENTA`, `ILI9341_COLOR_GRAY`/`GREY` o `ILI9341_COLOR_BROWN` esperando el valor RGB565 de v1.2.0, revisa la [tabla de colores](#colores-predefinidos): ahora corresponden a los tonos estándar X11/CSS (visualmente distintos en varios casos).
- `ILI9341_COLOR_GREEN2` y `ILI9341_COLOR_BLUE2` fueron eliminadas; sustitúyelas por el color X11 equivalente más cercano o calcula tu propio color con la macro `RGB565(r, g, b)`.
- Las nuevas funciones `ILI9341_DrawEllipse_ImageBuffer()`, `ILI9341_DrawFilledEllipse_ImageBuffer()`, `ILI9341_DrawArc_ImageBuffer()`, `ILI9341_DrawFilledArc_ImageBuffer()`, `ILI9341_Fill_ImageBuffer()` y `ILI9341_DrawCircle_ImageBuffer()` requieren `HAL_SDRAM_MODULE_ENABLED`, igual que el resto del grupo `*_ImageBuffer()` desde la v1.2.0.

---

### [1.2.0] - 15-06-2026

#### Added

- **Rectángulos con esquinas redondeadas** — cuatro nuevas primitivas de dibujo que combinan arcos de cuarto de círculo (algoritmo de Bresenham) con segmentos rectos para producir esquinas suaves:
  - `ILI9341_DrawRoundRect()`: contorno sobre pantalla. Traza cuatro segmentos rectos y cuatro arcos de cuarto de círculo, uno por esquina. El radio `r` se recorta automáticamente a `min(ancho, alto) / 2`; con `r = 0` delega en `ILI9341_DrawRectangle()`.
  - `ILI9341_DrawFilledRoundRect()`: relleno sobre pantalla. Dibuja la franja central con `ILI9341_DrawFilledRectangle()` (acceso directo al `DR` del SPI) y cubre los arcos superior e inferior con tramos horizontales generados por un único bucle Bresenham.
  - `ILI9341_DrawRoundRect_ImageBuffer()`: versión para frame buffer del contorno. Usa la función helper privada `DrawPixelClipped_ImageBuffer` para trazar los arcos con recorte a los límites del panel.
  - `ILI9341_DrawFilledRoundRect_ImageBuffer()`: versión para frame buffer del relleno. La franja central hereda la aceleración DMA2D R2M de `ILI9341_DrawFilledRectangle_ImageBuffer()` cuando el handle DMA2D fue inyectado en `ILI9341_Init()`; los arcos se rellenan siempre por CPU.
- Helper privado `DrawPixelClipped_ImageBuffer()`: análogo a `DrawPixelClipped()` pero para frame buffer; recorta coordenadas `int16_t` antes de delegar en `ILI9341_DrawPixel_ImageBuffer()`.
- **Triángulos** — cuatro nuevas primitivas que completan el conjunto de polígonos básicos:
  - `ILI9341_DrawTriangle()`: contorno sobre pantalla. Traza las tres aristas delegando en `ILI9341_DrawLine()` (Bresenham con acceso directo al `DR` del SPI).
  - `ILI9341_DrawFilledTriangle()`: relleno sobre pantalla. Ordena los tres vértices por coordenada Y y rellena fila a fila interpolando los bordes largo (a→c) y cortos (a→b / b→c) con división entera `int32_t` sin punto flotante. Los triángulos degenerados (collineales) se reducen a un único tramo horizontal. Protección explícita contra división por cero en triángulos de tope plano.
  - `ILI9341_DrawTriangle_ImageBuffer()`: contorno en frame buffer. Delega en `ILI9341_DrawLine_ImageBuffer()`.
  - `ILI9341_DrawFilledTriangle_ImageBuffer()`: relleno en frame buffer. Misma lógica de scanline que la versión directa pero usa la función helper privada `DrawHSpanClipped_ImageBuffer()`.

- **Volcado de frame buffer por DMA SPI** (`ILI9341_DisplayImage` / `ILI9341_Flush`): la transferencia de los 76 800 píxeles al ILI9341 ahora usa DMA2\_Stream6 vinculado a SPI5\_TX, eliminando el loop de sondeo byte a byte anterior.
  - Función privada `ILI9341_SPI_SetDataSize()`: cambia el bit `DFF` del registro `CR1` y actualiza `Init.DataSize` sin pasar por `HAL_SPI_DeInit/Init`, manteniendo el enlace DMA intacto.
  - Función privada `ILI9341_SPI_WaitDMAdone()`: poll bloqueante con timeout sobre la bandera `ILI9341_spi_dma_done`; llama a `HAL_SPI_DMAStop()` si se agota el tiempo.
  - Callback `HAL_SPI_TxCpltCallback()` (override del símbolo `__weak` del HAL): encadena automáticamente el segundo tramo DMA al terminar el primero (pipelining); señaliza `spi_dma_done` al completar el segundo.
- **`ILI9341_Sync()`** *(solo SDRAM)*: nueva función pública que espera a que concluya el DMA en curso y restaura el bus SPI a 8 bits. Necesaria al salir del modo de doble buffer (`ILI9341_Flush`) para volver a usar funciones de dibujo directo en pantalla sin corrupción de comandos SPI.
- Sección **Configuración DMA SPI TX** en el README con tabla de parámetros de CubeMX y notas sobre byte order y conflictos de callback.

#### Changed

- `ILI9341_DisplayImage()`: reemplaza el loop de polling `SPI_ILI9341_WaitTXE` por dos llamadas `HAL_SPI_Transmit_DMA` de 38 400 píxeles cada una en modo SPI 16 bits. El SPI se devuelve a 8 bits tras el volcado para que el resto del driver funcione sin cambios.
- La transferencia se divide en **dos tramos de 38 400 píxeles** para respetar el límite máximo de 65 535 items del registro NDTR del DMA de STM32F4.

#### Fixed

- **`HAL_SPI_TxCpltCallback`** — la segunda llamada a `HAL_SPI_Transmit_DMA` (encadenamiento del segundo tramo en modo pipelining) no verificaba su valor de retorno. Si fallaba, `ILI9341_dma_state` quedaba en `2` y `spi_dma_done` nunca se activaba, causando un stall de 5 s en el siguiente `ILI9341_Flush()`. Ahora en caso de fallo se señaliza `spi_dma_done = 1` y se resetea `dma_state = 0` para desbloquear el waiter limpiamente.
- **`ILI9341_SPI_WaitDMAdone`** — ante un timeout, `ILI9341_dma_state` no se reseteaba a `0` tras `HAL_SPI_DMAStop()`. Todas las llamadas subsiguientes a `ILI9341_FlushAsync()` retornaban `ILI9341_ERROR` por el guard `dma_state != 0`, dejando el driver permanentemente inutilizable tras un único timeout. Ahora se resetea `dma_state = 0` dentro del bloque de timeout.
- **`ILI9341_dma_px2` / `ILI9341_dma_half`** — declaradas `volatile`; son escritas en contexto normal y leídas desde el ISR `HAL_SPI_TxCpltCallback`, por lo que requieren `volatile` para garantizar visibilidad según el estándar C.
- **Guards `HAL_SDRAM_MODULE_ENABLED` en funciones `*_ImageBuffer()`** — todas las funciones del grupo `_ImageBuffer` (declaraciones en `.h`, implementaciones en `.c` y los helpers privados `DrawHSpanClipped_ImageBuffer` / `DrawPixelClipped_ImageBuffer`) carecían del guard `#ifdef HAL_SDRAM_MODULE_ENABLED`, por lo que se compilaban aunque el periférico FMC-SDRAM estuviese deshabilitado. Ahora el bloque completo queda protegido: sin `HAL_SDRAM_MODULE_ENABLED` las funciones no se declaran ni se compilan.

#### Migration notes

- Requiere añadir **DMA2\_Stream6 / Channel 7** a SPI5\_TX en CubeMX con ancho de dato **Half Word** en memoria y periférico.
- La llamada a `MX_DMA_Init()` debe aparecer antes de `MX_SPI5_Init()` en `main.c` (CubeMX la coloca así por defecto al regenerar código).
- Si el proyecto define `HAL_SPI_TxCpltCallback` para otro uso, hay conflicto de símbolo; ver nota en la sección de configuración.
- Al salir de un bucle de animación basado en `ILI9341_Flush()` para volver a funciones directas (p.ej. `ILI9341_Fill`), llamar a `ILI9341_Sync()` antes de la primera función directa.
- Las funciones `*_ImageBuffer()` ahora requieren `HAL_SDRAM_MODULE_ENABLED` para compilar. Proyectos que las usasen con un buffer estático en SRAM (sin FMC-SDRAM habilitado) deben habilitar el periférico en CubeMX o envolver sus llamadas en `#ifdef HAL_SDRAM_MODULE_ENABLED`.

---

### [1.1.0] - 14-06-2026

#### Added

- Soporte de aceleración gráfica por hardware mediante **DMA2D** (`HAL_DMA2D_MODULE_ENABLED`):
  - `ILI9341_BlitImage()`: copia una imagen RGB565 fuente al frame buffer usando DMA2D en modo M2M (memoria a memoria), con recorte automático al borde de pantalla.
  - `ILI9341_DrawFilledRectangle_ImageBuffer()` usa DMA2D en modo R2M (registro a memoria) cuando el handle DMA2D fue inyectado en `ILI9341_Init()`; cae al camino CPU si se pasa `NULL`.
  - Función privada `ILI9341_DMA2D_SetMode()` que reprograma modo y offset de salida del DMA2D directamente sobre los registros `CR`/`OOR`, evitando llamar a `HAL_DMA2D_Init()` en cada operación de dibujo.
  - La firma de `ILI9341_Init()` se amplía a 8 combinaciones para cubrir todas las combinaciones posibles de `HAL_I2C_MODULE_ENABLED`, `HAL_SDRAM_MODULE_ENABLED` y `HAL_DMA2D_MODULE_ENABLED`.

#### Changed

- Todas las funciones `*_ImageBuffer()` que retornaban `void` ahora retornan `ILI9341_Status_t` (`ILI9341_OK`, `ILI9341_INVALID_PARAM`, `ILI9341_ERROR`). Afecta a: `DrawPixel_ImageBuffer`, `Putc_ImageBuffer`, `Puts_ImageBuffer`, `DrawLine_ImageBuffer`, `DrawRectangle_ImageBuffer`, `DrawFilledRectangle_ImageBuffer`, `DrawFilledCircle_ImageBuffer`.
- `SPI_ILI9341_WaitTXE()` (privada): el sondeo de la bandera TXE ahora usa un contador de iteraciones (`ILI9341_SPI_TXE_SPIN_MAX = 1 000 000`) en lugar de `HAL_GetTick()`, eliminando una llamada a función y una lectura de tick por byte en los bucles críticos de `ILI9341_Fill()` e `ILI9341_DrawFilledRectangle()`.
- `DrawFilledRectangle_ImageBuffer`: los píxeles de cabecera y cola (alineación impar) se escriben como halfword (`uint16_t*`) en lugar de read-modify-write de 32 bits, eliminando artefactos de alineación.

#### Fixed

- Secuencia de inicialización SDRAM: el registro de modo ahora programa burst de **1 palabra** (en lugar de 2) tal como especifica el datasheet del IS42S16400J.

---

### [1.0.1]

#### Fixed
- Corrección en el Copyright del encabezado de los archivos fuente.

---

### [1.0.0] - 08-06-2026

#### Added

- Implementación completa del driver ILI9341 para la STM32F429-Discovery sobre SPI5 e I2C3.
- Secuencia de inicialización del chip con elevación automática del preescalador SPI a 45 Mbit/s.
- Primitivas de dibujo directo sobre pantalla: `DrawPixel`, `DrawLine` (Bresenham), `DrawRectangle`, `DrawFilledRectangle`, `DrawCircle`, `DrawFilledCircle`.
- Renderizado de texto: `Putc` y `Puts` con soporte de salto de línea, retorno de carro y fuentes `LCD_FontDef_t`.
- Función `GetStringSize()` para calcular el bounding-box de una cadena antes de dibujarla.
- Transferencia de imagen completa `DisplayImage()` mediante acceso directo al registro `DR` del SPI.
- Juego completo de funciones `*_ImageBuffer()` para composición off-screen en RAM interna (double buffering).
- Soporte de frame buffer en SDRAM IS42S16400J (`HAL_SDRAM_MODULE_ENABLED`): inicialización automática, `Flush()` y `GetFrameBuffer()`.
- Control de rotación en cuatro orientaciones mediante `ILI9341_Rotate()`.
- 13 colores predefinidos en formato RGB565.
- Driver del panel táctil resistivo STMPE811: `ILI9341_TP_Config()`, `ILI9341_TP_GetState()` con filtro de histeresis de 5 puntos.
- `ILI9341_DeInit()` para liberar periféricos y desinicializar la SDRAM.
- Manejo robusto de errores con `ILI9341_Status_t` en todas las funciones públicas.
