# Librería para la pantalla TFT LCD ILI9341 en STM32F429-Discovery

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F429--Discovery-black)](https://www.st.com/en/evaluation-tools/32f429idiscovery.html)
[![Version](https://img.shields.io/badge/Version-1.2.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/ILI9341_Disc1)
[![Protocol](https://img.shields.io/badge/Protocol-SPI%20%2B%20I2C%20%2B%20SDRAM%20%2B%20DMA2D%20%2B%20SPI--DMA-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/ILI9341_Disc1)

---

## Tabla de Contenidos
- [Librería para la pantalla TFT LCD ILI9341 en STM32F429-Discovery](#librería-para-la-pantalla-tft-lcd-ili9341-en-stm32f429-discovery)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Bus SPI (LCD ILI9341)](#bus-spi-lcd-ili9341)
    - [GPIO de Control (LCD ILI9341)](#gpio-de-control-lcd-ili9341)
    - [Bus I2C (Panel táctil STMPE811)](#bus-i2c-panel-táctil-stmpe811)
  - [Configuración SPI](#configuración-spi)
  - [Configuración I2C](#configuración-i2c)
  - [Configuración SDRAM (opcional)](#configuración-sdram-opcional)
    - [Configuración](#configuración)
  - [Configuración DMA2D (opcional)](#configuración-dma2d-opcional)
  - [Configuración DMA SPI TX (para DisplayImage y Flush)](#configuración-dma-spi-tx-para-displayimage-y-flush)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Dibujo en pantalla](#2-dibujo-en-pantalla)
    - [3. Texto en pantalla](#3-texto-en-pantalla)
    - [4. Frame buffer fuera de pantalla (RAM)](#4-frame-buffer-fuera-de-pantalla-ram)
    - [5. Frame buffer en SDRAM y Flush](#5-frame-buffer-en-sdram-y-flush)
    - [6. Panel táctil](#6-panel-táctil)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`ILI9341_Status_t` - Estados de Retorno](#ili9341_status_t---estados-de-retorno)
      - [`ILI9341_Orientation_t` - Orientaciones de Pantalla](#ili9341_orientation_t---orientaciones-de-pantalla)
      - [`TP_STATE` - Estado del Panel Táctil](#tp_state---estado-del-panel-táctil)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`ILI9341_Init()` - Inicialización del Driver](#ili9341_init---inicialización-del-driver)
      - [`ILI9341_DeInit()` - Desinicialización del Driver](#ili9341_deinit---desinicialización-del-driver)
      - [`ILI9341_Fill()` - Rellenar Pantalla](#ili9341_fill---rellenar-pantalla)
      - [`ILI9341_Rotate()` - Rotar Pantalla](#ili9341_rotate---rotar-pantalla)
      - [`ILI9341_DrawPixel()` - Dibujar Píxel](#ili9341_drawpixel---dibujar-píxel)
      - [`ILI9341_DrawLine()` - Dibujar Línea](#ili9341_drawline---dibujar-línea)
      - [`ILI9341_DrawRectangle()` - Dibujar Rectángulo](#ili9341_drawrectangle---dibujar-rectángulo)
      - [`ILI9341_DrawFilledRectangle()` - Dibujar Rectángulo Relleno](#ili9341_drawfilledrectangle---dibujar-rectángulo-relleno)
      - [`ILI9341_DrawRoundRect()` - Dibujar Rectángulo con Esquinas Redondeadas](#ili9341_drawroundrect---dibujar-rectángulo-con-esquinas-redondeadas)
      - [`ILI9341_DrawFilledRoundRect()` - Dibujar Rectángulo Redondeado Relleno](#ili9341_drawfilledroundrect---dibujar-rectángulo-redondeado-relleno)
      - [`ILI9341_DrawCircle()` - Dibujar Círculo](#ili9341_drawcircle---dibujar-círculo)
      - [`ILI9341_DrawFilledCircle()` - Dibujar Círculo Relleno](#ili9341_drawfilledcircle---dibujar-círculo-relleno)
      - [`ILI9341_DrawTriangle()` - Dibujar Triángulo](#ili9341_drawtriangle---dibujar-triángulo)
      - [`ILI9341_DrawFilledTriangle()` - Dibujar Triángulo Relleno](#ili9341_drawfilledtriangle---dibujar-triángulo-relleno)
      - [`ILI9341_Putc()` - Renderizar Carácter](#ili9341_putc---renderizar-carácter)
      - [`ILI9341_Puts()` - Renderizar Cadena](#ili9341_puts---renderizar-cadena)
      - [`ILI9341_GetStringSize()` - Calcular Tamaño de Cadena](#ili9341_getstringsize---calcular-tamaño-de-cadena)
      - [`ILI9341_DisplayImage()` - Transferir Frame Buffer](#ili9341_displayimage---transferir-frame-buffer)
      - [Funciones de Frame Buffer fuera de Pantalla](#funciones-de-frame-buffer-fuera-de-pantalla)
      - [`ILI9341_DrawRoundRect_ImageBuffer()` - Rectángulo Redondeado en Buffer](#ili9341_drawroundrect_imagebuffer---rectángulo-redondeado-en-buffer)
      - [`ILI9341_DrawFilledRoundRect_ImageBuffer()` - Rectángulo Redondeado Relleno en Buffer](#ili9341_drawfilledroundrect_imagebuffer---rectángulo-redondeado-relleno-en-buffer)
      - [`ILI9341_DrawTriangle_ImageBuffer()` - Triángulo en Buffer](#ili9341_drawtriangle_imagebuffer---triángulo-en-buffer)
      - [`ILI9341_DrawFilledTriangle_ImageBuffer()` - Triángulo Relleno en Buffer](#ili9341_drawfilledtriangle_imagebuffer---triángulo-relleno-en-buffer)
      - [`ILI9341_BlitImage()` - Copiar Imagen con DMA2D *(solo DMA2D)*](#ili9341_blitimage---copiar-imagen-con-dma2d-solo-dma2d)
      - [`ILI9341_Flush()` - Volcar Frame Buffer SDRAM *(solo SDRAM)*](#ili9341_flush---volcar-frame-buffer-sdram-solo-sdram)
      - [`ILI9341_Sync()` - Sincronizar DMA con el bus SPI *(solo SDRAM)*](#ili9341_sync---sincronizar-dma-con-el-bus-spi-solo-sdram)
      - [`ILI9341_GetFrameBuffer()` - Obtener Puntero al Frame Buffer SDRAM *(solo SDRAM)*](#ili9341_getframebuffer---obtener-puntero-al-frame-buffer-sdram-solo-sdram)
      - [`ILI9341_TP_Config()` - Configurar Panel Táctil](#ili9341_tp_config---configurar-panel-táctil)
      - [`ILI9341_TP_GetState()` - Obtener Estado del Toque](#ili9341_tp_getstate---obtener-estado-del-toque)
  - [Colores Predefinidos](#colores-predefinidos)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.2.0\] - 15-06-2026](#120---15-06-2026)
      - [Added](#added)
      - [Changed](#changed)
      - [Fixed](#fixed)
      - [Migration notes](#migration-notes)
    - [\[1.1.0\] - 14-06-2026](#110---14-06-2026)
      - [Added](#added-1)
      - [Changed](#changed-1)
      - [Fixed](#fixed-1)
    - [\[1.0.1\]](#101)
      - [Fixed](#fixed-2)
    - [\[1.0.0\] - 08-06-2026](#100---08-06-2026)
      - [Added](#added-2)

---

## Descripción

Librería desarrollada en C para el control de la pantalla TFT LCD **Ilitek ILI9341** de 240×320 píxeles integrada en la tarjeta **STM32F429-Discovery**. Proporciona una API completa para inicialización, primitivas de dibujo (píxeles, líneas, rectángulos, círculos), renderizado de texto con fuentes personalizables, transferencia eficiente de imágenes completas y soporte de frame buffer fuera de pantalla tanto en RAM interna como en la SDRAM externa IS42S16400J de 4 MB. Incluye aceleración por hardware **DMA2D** para relleno de rectángulos y copia de imágenes al frame buffer, así como la gestión completa del panel táctil resistivo basado en el controlador **STMPE811** por I2C.

Diseñada para ser portable y robusta: toda función pública (incluidas las variantes `*_ImageBuffer()`) retorna un código de estado `ILI9341_Status_t` que permite detectar errores de comunicación SPI/I2C, timeouts y condiciones de driver no inicializado.

---

## Características

- **Comunicación SPI optimizada**: Inicialización a 2 Mbit/s para la secuencia de configuración del chip; tras `ILI9341_Init()` el preescalador se eleva automáticamente a 45 Mbit/s para máxima velocidad de refresco.
- **Escrituras SPI optimizadas mediante acceso directo al registro `DR`**: `ILI9341_Fill()`, `ILI9341_DrawFilledRectangle()` e `ILI9341_Putc()` acceden directamente al registro `DR` del SPI. El sondeo de TXE usa un contador de iteraciones (`SPI_ILI9341_WaitTXE`) en lugar de `HAL_GetTick()`, eliminando una llamada a función y una lectura de tick por byte en los bucles críticos de volcado.
- **Volcado de frame buffer por DMA SPI**: `ILI9341_DisplayImage()` e `ILI9341_Flush()` transfieren los 76 800 píxeles del frame buffer a la pantalla usando **DMA2\_Stream6** vinculado a SPI5\_TX en modo 16 bits. El SPI en modo 16 bits serializa cada `uint16_t` MSB-first, produciendo automáticamente el orden big-endian esperado por el ILI9341 sin swap manual de bytes. La transferencia se divide en dos tramos de 38 400 píxeles para respetar el límite de 65 535 items del registro NDTR del DMA.
- **Aceleración DMA2D** *(requiere `HAL_DMA2D_MODULE_ENABLED`)*: `ILI9341_Init()` acepta un `DMA2D_HandleTypeDef*` opcional; si no es NULL, configura el periférico DMA2D una sola vez y lo reutiliza en modo R2M (relleno) para `ILI9341_DrawFilledRectangle_ImageBuffer()` y en modo M2M (copia) para `ILI9341_BlitImage()`. Si se pasa NULL, ambas operaciones usan el camino CPU.
- **Primitivas de dibujo completas**: Píxeles, líneas (algoritmo de Bresenham), rectángulos (contorno y relleno), círculos (contorno y relleno) y triángulos (contorno y relleno por scanline) directamente sobre la pantalla.
- **Renderizado de texto**: `ILI9341_Putc()` / `ILI9341_Puts()` con soporte de saltos de línea, retorno de carro y fuentes de ancho variable mediante `LCD_FontDef_t`.
- **Frame buffer fuera de pantalla (RAM)**: Juego completo de funciones `*_ImageBuffer()` que operan sobre un array `uint32_t[38 400]` en RAM interna, empaquetando dos píxeles RGB565 por palabra de 32 bits. Todas estas funciones retornan `ILI9341_Status_t` para detectar errores (puntero NULL, fallo DMA2D). Ideal para composición de imagen sin parpadeo.
- **Frame buffer en SDRAM** *(requiere `HAL_SDRAM_MODULE_ENABLED`)*: `ILI9341_Init()` acepta un `SDRAM_HandleTypeDef*` opcional; si no es NULL, inicializa la IS42S16400J y reserva los primeros 153 600 bytes de `0xD0000000` como frame buffer interno. `ILI9341_Flush()` vuelca el buffer a pantalla con una sola llamada.
- **Panel táctil resistivo STMPE811**: Configuración, lectura de coordenadas X/Y calibradas ([0, 239] × [0, 319]) y presión Z por I2C, con filtro de histeresis de 5 puntos.
- **Cuatro orientaciones de pantalla**: Portrait 0°/180° y Landscape 90°/270° configurables en tiempo de ejecución con `ILI9341_Rotate()`.
- **Manejo robusto de errores**: Retornos `ILI9341_Status_t` en todas las funciones públicas (incluidas las variantes `*_ImageBuffer()`) con valores específicos para parámetro inválido, timeout de SPI, fallo DMA2D y driver no inicializado.
- **Portabilidad HAL**: Solo requiere `main.h` (que incluye el HAL de STM32F4) para compilar. Los pines de control están definidos como macros modificables en el encabezado.

---

## Pinout y Conexiones

### Bus SPI (LCD ILI9341)

| Pin ILI9341 | Pin STM32F429 | Descripción                    | Configuración CubeMX       |
|-------------|---------------|--------------------------------|---------------------------|
| **VCC**     | 3.3V          | Alimentación                   | N/A                       |
| **GND**     | GND           | Tierra                         | N/A                       |
| **SDO/MISO**| PF8           | Salida LCD (opcional, lectura) | Configurado por SPI5      |
| **SCK**     | PF7           | Reloj SPI5                     | Configurado por SPI5      |
| **SDI/MOSI**| PF9           | Entrada datos al LCD           | Configurado por SPI5      |

### GPIO de Control (LCD ILI9341)

| Señal      | Pin STM32F429 | Descripción                  | Tipo GPIO       | Nivel inicial | Etiqueta CubeMX |
|------------|---------------|------------------------------|-----------------|---------------|-----------------|
| **D/C (WRX)** | PD13       | Selección Dato / Comando     | `GPIO_OUTPUT`   | —             | *(libre)*       |
| **RESET**  | PD12          | Reset por hardware del LCD   | `GPIO_OUTPUT`   | HIGH          | *(libre)*       |
| **CS**     | PC2           | Selección de chip SPI5       | `GPIO_OUTPUT`   | HIGH          | *(libre)*       |

> [!NOTE]
> Los pines D/C, RESET y CS se gestionan completamente por software dentro de la librería mediante las macros `ILI9341_WRX_SET/RESET`, `ILI9341_RST_SET/RESET` e `ILI9341_CS_SET/RESET` definidas en `ILI9341_Disc1.h`. Para adaptar a otra placa, modifica únicamente esas macros.

### Bus I2C (Panel táctil STMPE811)

| Señal       | Pin STM32F429 | Descripción           | Tipo GPIO                | Observaciones                      |
|-------------|---------------|-----------------------|--------------------------|------------------------------------|
| **Touch SCL** | PA8         | Reloj I2C3            | `I2C_SCL` (open-drain)  | Requiere pull-up externo (4.7 kΩ)  |
| **Touch SDA** | PC9         | Datos I2C3            | `I2C_SDA` (open-drain)  | Requiere pull-up externo (4.7 kΩ)  |

> [!WARNING]
> El STMPE811 en la STM32F429-Discovery ya incluye los pull-ups en la placa. No añadir pull-ups adicionales si se usa la placa original.

---

## Configuración SPI

Configura el periférico **SPI5** en CubeMX/STM32CubeIDE:

| Parámetro               | Valor                | Notas                                                                       |
|-------------------------|----------------------|-----------------------------------------------------------------------------|
| **Mode**                | Full-Duplex Master   | STM32 controla el bus                                                       |
| **Hardware NSS**        | Disable              | CS gestionado por software (GPIO PC2)                                       |
| **Data Size**           | 8 Bits               | —                                                                           |
| **First Bit**           | MSB First            | —                                                                           |
| **Prescaler**           | /32 ≈ 2.8 Mbit/s     | Valor inicial para la secuencia de Init; la librería lo eleva a /2 al final |
| **CPOL**                | Low (0)              | —                                                                           |
| **CPHA**                | 1 Edge               | —                                                                           |
| **CRC**                 | Disabled             | No utilizado                                                                |

> [!NOTE]
> Tras una llamada exitosa a `ILI9341_Init()`, la librería reinicializa SPI5 con el preescalador `/2` (90 MHz / 2 = **45 Mbit/s**) para maximizar la velocidad de transferencia de imágenes. No es necesario hacer nada desde el código de usuario.

---

## Configuración I2C

Configura el periférico **I2C3** en CubeMX/STM32CubeIDE:

| Parámetro          | Valor            | Notas                                     |
|--------------------|------------------|-------------------------------------------|
| **Mode**           | I2C              | —                                         |
| **I2C Speed Mode** | Standard Mode    | Soporta 100 kHz y 400 kHz                 |
| **I2C Clock Speed**| 100 000          | Depende del requisito de latencia táctil  |

> [!NOTE]
> La dirección I2C del STMPE811 es **0x41** (7 bits) / **0x82** (8 bits, escritura). La librería gestiona internamente la composición de las direcciones de lectura y escritura; no es necesario configurar nada adicional.

---

## Configuración SDRAM (opcional)

El soporte de frame buffer en SDRAM se activa automáticamente cuando `HAL_SDRAM_MODULE_ENABLED` está definido (lo incluye el HAL generado por CubeMX al habilitar el periférico FMC-SDRAM).

Configura el **FMC — SDRAM** en CubeMX con los siguientes parámetros para la IS42S16400J de la STM32F429-Discovery:

Seleccionar SDRAM 2:

| Parámetro                     | Valor         |
|-------------------------------|---------------|
| **Clock and chip enable**     | SDCKE1+SDNE1  |
| **Internal Bank Number**      | 4 banks       |
| **Address Bus Width**         | 12 bits       |
| **Data Bus Width**            | 16 bits       |
| **16-bit bytes enable**       | Enable        |

### Configuración

| Parámetro                     | Valor         |
|-------------------------------|---------------|
| **Bank**                      | SDRAM Bank 2  |
| **Number of Columns**         | 8 bits        |
| **Number of Rows**            | 12 bits       |
| **CAS Latency**               | 3 ciclos      |
| **Write Protection**          | Disable       |
| **SDRAM common clock**        | 2 HCLK clock cycles |
| **SDRAM common burst read**   | Disable       |
| **SDRAM common read pipe delay** | 1 HCLK clock cycle |

| SDRAM Timing Parameter        | Valor         |
|-------------------------------|---------------|
| **Load To Active Delay**       | 2 HCLK cycles |
| **Exit Self Refresh Delay**    | 7 HCLK cycles |
| **Self Refresh Time**          | 4 HCLK cycles |
| **SDRAM common row cycle delay** | 7 HCLK cycles |
| **Write Recovery Time**        | 3 HCLK cycles |
| **SDRAM common row precharge delay** | 2 HCLK cycles |
| **Row Cycle Delay**            | 2 HCLK cycles |

La secuencia de inicialización (habilitación de reloj, PALL, 4 ciclos de auto-refresh y programación del registro de modo) se ejecuta **automáticamente** dentro de `ILI9341_Init()` al pasar un `hsdram` válido. La librería reserva los primeros **153 600 bytes** de `0xD0000000` (inicio del banco 2 del FMC) como frame buffer.

> [!WARNING]
> Si tu aplicación también usa la SDRAM para otros datos, asegúrate de que las direcciones no se solapen con los primeros 153 600 bytes a partir de `0xD0000000`.

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

El volcado del frame buffer por DMA se activa al configurar **DMA2\_Stream6** como canal TX de SPI5 en CubeMX. No requiere ninguna macro de compilación adicional; la librería detecta en tiempo de ejecución si `hspi5.hdmatx` está enlazado.

En la pestaña **SPI5 → DMA Settings** de CubeMX:

| Parámetro | Valor |
|-----------|-------|
| **Request** | SPI5_TX |
| **Stream** | DMA2\_Stream6 (automático) |
| **Channel** | Channel 7 (automático) |
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
> CubeMX genera el enlace `hspi5.hdmatx = &hdma_spi5_tx` dentro de `HAL_SPI_MspInit()` (en `stm32f4xx_hal_msp.c`) y el handler `DMA2_Stream6_IRQHandler` en `stm32f4xx_it.c`. Verifica que la llamada a `MX_DMA_Init()` aparezca **antes** de `MX_SPI5_Init()` en `main.c`.

> [!WARNING]
> La función `HAL_SPI_TxCpltCallback` está definida dentro de la librería. Si tu proyecto ya define este callback para otro propósito, obtendrás un error de símbolo duplicado en el enlazado. En ese caso, mueve el cuerpo del callback al tuyo propio y llama a `ILI9341_SPI_FlushDoneNotify()` — o bien gestiona los dos handles SPI con una condición `if`.

---

## Instalación

1. Copia `ILI9341_Disc1.c`, `ILI9341_Disc1.h` y `lcd_fonts.h` a tu proyecto (ej: `Librerias/ILI9341_Disc1/`).
2. Incluye la librería en tu `main.c` o archivo principal:
   ```c
   #include "ILI9341_Disc1.h"
   ```
3. Configura SPI5, I2C3 y los GPIOs de control en CubeMX (ver secciones anteriores).
4. Si usas SDRAM, habilita el FMC-SDRAM en CubeMX — `HAL_SDRAM_MODULE_ENABLED` se definirá automáticamente.
5. Genera código y compila.

---

## Uso Básico

### 1. Inicialización

```c
/* Solo SPI (sin panel táctil, sin SDRAM, sin DMA2D) */
ILI9341_Status_t status = ILI9341_Init(&hspi5);

/* Con panel táctil I2C, sin SDRAM ni DMA2D */
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hi2c3);

/* Con DMA2D, sin panel táctil ni SDRAM */
#ifdef HAL_DMA2D_MODULE_ENABLED
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hdma2d);
#endif

/* Con panel táctil I2C y DMA2D, sin SDRAM */
#if defined(HAL_I2C_MODULE_ENABLED) && defined(HAL_DMA2D_MODULE_ENABLED)
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hi2c3, &hdma2d);
#endif

/* Con SDRAM, sin panel táctil ni DMA2D */
#ifdef HAL_SDRAM_MODULE_ENABLED
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hsdram);
#endif

/* Con SDRAM y DMA2D, sin panel táctil */
#if defined(HAL_SDRAM_MODULE_ENABLED) && defined(HAL_DMA2D_MODULE_ENABLED)
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hsdram, &hdma2d);
#endif

/* Con panel táctil I2C y SDRAM, sin DMA2D */
#if defined(HAL_SDRAM_MODULE_ENABLED) && defined(HAL_I2C_MODULE_ENABLED)
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hi2c3, &hsdram);
#endif

/* Con panel táctil I2C, SDRAM y DMA2D */
#if defined(HAL_SDRAM_MODULE_ENABLED) && defined(HAL_I2C_MODULE_ENABLED) && defined(HAL_DMA2D_MODULE_ENABLED)
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hi2c3, &hsdram, &hdma2d);
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
> El SPI debe estar previamente inicializado a ~5.6 Mbit/s mediante `MX_SPI5_Init()`. La librería elevará el preescalador automáticamente al final de `ILI9341_Init()`. Los handles I2C (`hi2c`), SDRAM (`hsdram`) y DMA2D (`hdma2d`) son opcionales: usa la sobrecarga que corresponda a los periféricos habilitados en tu proyecto. Pasar `NULL` como `hdma2d` deshabilita la aceleración DMA2D sin error.

---

### 2. Dibujo en pantalla

```c
/* Limpiar pantalla con color negro */
ILI9341_Fill(ILI9341_COLOR_BLACK);

/* Dibujar un píxel rojo en (120, 160) */
ILI9341_DrawPixel(120, 160, ILI9341_COLOR_RED);

/* Dibujar una línea azul */
ILI9341_DrawLine(0, 0, 239, 319, ILI9341_COLOR_BLUE);

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
ILI9341_Puts(10, 30, "Hola, mundo!\nSTM32F429",
                 &Font_11x18,
                 ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

/* Calcular el ancho y alto de una cadena antes de dibujarla */
uint16_t w, h;
ILI9341_GetStringSize("Texto", &Font_11x18, &w, &h);
/* Centrar horizontalmente */
ILI9341_Puts((240 - w) / 2, 150, "Texto",
                 &Font_11x18,
                 ILI9341_COLOR_YELLOW, ILI9341_COLOR_BLACK);
```

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
> El buffer ocupa **153 600 bytes** en RAM interna. La STM32F429 dispone de 256 KB de SRAM y 64 KB de CCM RAM. Declara el buffer como `static` o global para evitar desbordamiento de pila.

---

### 5. Frame buffer en SDRAM y Flush

Con SDRAM habilitada, el frame buffer interno reside en memoria externa y se accede de forma transparente a través del puntero devuelto por `ILI9341_GetFrameBuffer()`.

```c
#ifdef HAL_SDRAM_MODULE_ENABLED

uint32_t* fb = ILI9341_GetFrameBuffer();  /* NULL si SDRAM no habilitada */

if (fb != NULL) {
    /* Limpiar el frame buffer en SDRAM */
    memset(fb, 0, ILI9341_SDRAM_FB_SIZE);

    /* Dibujar usando las funciones _ImageBuffer con el puntero SDRAM */
    ILI9341_DrawFilledRectangle_ImageBuffer(0, 0, 239, 319,
                                                ILI9341_COLOR_BLACK, fb);
    ILI9341_Puts_ImageBuffer(60, 150, "SDRAM OK",
                                  &Font_11x18,
                                  ILI9341_COLOR_GREEN, fb);

    /* Volcar el frame buffer SDRAM a la pantalla */
    ILI9341_Flush();
}

#endif
```

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

Inicializa la pantalla ILI9341 y, opcionalmente, la SDRAM IS42S16400J y/o el acelerador DMA2D. Aplica la secuencia de configuración del chip, enciende la pantalla y eleva el preescalador SPI a 45 Mbit/s al finalizar. El estado inicial es Portrait 1 (240×320).

La firma varía según los módulos HAL habilitados:

| `HAL_SDRAM_MODULE_ENABLED` | `HAL_I2C_MODULE_ENABLED` | `HAL_DMA2D_MODULE_ENABLED` | Firma resultante |
|:--------------------------:|:------------------------:|:--------------------------:|------------------|
| No | No | No | `ILI9341_Init(hspi)` |
| No | No | Sí | `ILI9341_Init(hspi, hdma2d)` |
| No | Sí | No | `ILI9341_Init(hspi, hi2c)` |
| No | Sí | Sí | `ILI9341_Init(hspi, hi2c, hdma2d)` |
| Sí | No | No | `ILI9341_Init(hspi, hsdram)` |
| Sí | No | Sí | `ILI9341_Init(hspi, hsdram, hdma2d)` |
| Sí | Sí | No | `ILI9341_Init(hspi, hi2c, hsdram)` |
| Sí | Sí | Sí | `ILI9341_Init(hspi, hi2c, hsdram, hdma2d)` |

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `hspi` | `SPI_HandleTypeDef*` | Handle del periférico SPI5 (obligatorio) |
| `hi2c` | `I2C_HandleTypeDef*` | Handle del periférico I2C3 (panel táctil); obligatorio si `HAL_I2C_MODULE_ENABLED` |
| `hsdram` | `SDRAM_HandleTypeDef*` | Handle SDRAM del FMC. `NULL` deshabilita el frame buffer en SDRAM |
| `hdma2d` | `DMA2D_HandleTypeDef*` | Handle DMA2D. `NULL` deshabilita la aceleración DMA2D (rellenos y BlitImage usan CPU) |

**Retorna**: `ILI9341_OK` si la inicialización fue exitosa, `ILI9341_INVALID_PARAM` si `hspi` o `hi2c` son NULL, `ILI9341_ERROR` si una transmisión SPI o la configuración DMA2D falló durante la inicialización.

**Secuencia interna:**
1. Validación de parámetros y reset por hardware.
2. Envío de la secuencia de comandos de configuración del ILI9341 (power, gamma, MAC, pixel format, etc.).
3. Salida del modo sleep y encendido de la pantalla.
4. Reinicialización de SPI5 con preescalador `/2` (45 Mbit/s).
5. *(Si `hsdram != NULL`)* Ejecución de la secuencia de inicialización SDRAM y mapeo del frame buffer en `0xD0000000`.
6. *(Si `hdma2d != NULL`)* Configuración única del DMA2D: formato de salida RGB565, capa de entrada RGB565. Las operaciones de dibujo posteriores solo actualizan modo y offset mediante `ILI9341_DMA2D_SetMode()`.

---

#### `ILI9341_DeInit()` - Desinicialización del Driver

Marca el driver como no inicializado y libera los handles internos. Si la SDRAM estaba habilitada, llama a `HAL_SDRAM_DeInit()`.

```c
ILI9341_Status_t ILI9341_DeInit(void);
```

**Retorna**: `ILI9341_OK` si la desinicialización fue exitosa, `ILI9341_NOT_INITIALIZED` si el driver no estaba inicializado.

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
| `ILI9341_DrawPixel_ImageBuffer(x, y, color, image)` | Escribe un píxel en el buffer |
| `ILI9341_DrawLine_ImageBuffer(x0, y0, x1, y1, color, image)` | Dibuja una línea (Bresenham) |
| `ILI9341_DrawRectangle_ImageBuffer(x0, y0, x1, y1, color, image)` | Contorno de rectángulo |
| `ILI9341_DrawFilledRectangle_ImageBuffer(x0, y0, x1, y1, color, image)` | Rectángulo relleno (DMA2D R2M si disponible) |
| `ILI9341_DrawRoundRect_ImageBuffer(x0, y0, x1, y1, r, color, image)` | Contorno de rectángulo redondeado (Bresenham) |
| `ILI9341_DrawFilledRoundRect_ImageBuffer(x0, y0, x1, y1, r, color, image)` | Rectángulo redondeado relleno (franja central con DMA2D si disponible + arcos por CPU) |
| `ILI9341_DrawFilledCircle_ImageBuffer(x0, y0, r, color, image)` | Círculo relleno |
| `ILI9341_DrawTriangle_ImageBuffer(x0, y0, x1, y1, x2, y2, color, image)` | Contorno de triángulo (tres llamadas a `DrawLine_ImageBuffer`) |
| `ILI9341_DrawFilledTriangle_ImageBuffer(x0, y0, x1, y1, x2, y2, color, image)` | Triángulo relleno (scanline, aritmética entera) |
| `ILI9341_Putc_ImageBuffer(x, y, c, font, fg, image)` | Carácter (sin fondo) |
| `ILI9341_Puts_ImageBuffer(x, y, str, font, fg, image)` | Cadena (sin fondo) |

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

#### `ILI9341_Flush()` - Volcar Frame Buffer SDRAM *(solo SDRAM)*

Equivalente a llamar `ILI9341_DisplayImage(ILI9341_GetFrameBuffer())`. Requiere que `ILI9341_Init()` haya sido invocado con un handle SDRAM válido.

```c
ILI9341_Status_t ILI9341_Flush(void);
```

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` (si la SDRAM no fue habilitada), `ILI9341_TIMEOUT` o `ILI9341_ERROR`.

---

#### `ILI9341_Sync()` - Sincronizar DMA con el bus SPI *(solo SDRAM)*

Espera a que concluya el DMA en curso y restaura el bus SPI al modo 8 bits. Si no hay DMA activo retorna inmediatamente sin efecto.

Debe llamarse al salir del modo de doble buffer antes de usar funciones de dibujo directo en pantalla (`ILI9341_Fill`, `ILI9341_DrawPixel`, `ILI9341_DrawFilledRectangle`, etc.). Sin esta llamada, el SPI permanece en modo 16 bits y CS queda bajo, lo que corrompe los comandos enviados por las funciones directas.

```c
ILI9341_Status_t ILI9341_Sync(void);
```

**Uso típico al salir de una animación SDRAM:**

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

#### `ILI9341_GetFrameBuffer()` - Obtener Puntero al Frame Buffer SDRAM *(solo SDRAM)*

Retorna el puntero al frame buffer interno en SDRAM (`0xD0000000`), compatible con todas las funciones `*_ImageBuffer()`. Retorna `NULL` si la SDRAM no fue habilitada o el driver no está inicializado.

```c
uint32_t* ILI9341_GetFrameBuffer(void);
```

---

#### `ILI9341_TP_Config()` - Configurar Panel Táctil

Verifica el ID del chip STMPE811, realiza un reset por software y configura el ADC y el controlador táctil con los parámetros óptimos para la pantalla de la Discovery.

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

La librería incluye macros de colores en formato **RGB565** listas para usar:

| Macro | Valor | Color |
|-------|-------|-------|
| `ILI9341_COLOR_WHITE` | `0xFFFF` | Blanco |
| `ILI9341_COLOR_BLACK` | `0x0000` | Negro |
| `ILI9341_COLOR_RED` | `0xF800` | Rojo |
| `ILI9341_COLOR_GREEN` | `0x07E0` | Verde |
| `ILI9341_COLOR_GREEN2` | `0xB723` | Verde oscuro |
| `ILI9341_COLOR_BLUE` | `0x001F` | Azul |
| `ILI9341_COLOR_BLUE2` | `0x051D` | Azul oscuro |
| `ILI9341_COLOR_YELLOW` | `0xFFE0` | Amarillo |
| `ILI9341_COLOR_ORANGE` | `0xFBE4` | Naranja |
| `ILI9341_COLOR_CYAN` | `0x07FF` | Cian |
| `ILI9341_COLOR_MAGENTA` | `0xA254` | Magenta |
| `ILI9341_COLOR_GRAY` | `0x7BEF` | Gris |
| `ILI9341_COLOR_BROWN` | `0xBBCA` | Café |

Para obtener cualquier color en RGB565 a partir de componentes R, G, B (0–255):

```c
#define RGB565(r, g, b) ((uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)))
```

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](../../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

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
