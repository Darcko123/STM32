# Librería para la pantalla TFT LCD ILI9341 en STM32F429-Discovery

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F429--Discovery-black)](https://www.st.com/en/evaluation-tools/32f429idiscovery.html)
[![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/ILI9341_Disc1)
[![Protocol](https://img.shields.io/badge/Protocol-SPI%20%2B%20I2C%20%2B%20SDRAM-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/ILI9341_Disc1)

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
      - [`ILI9341_DrawCircle()` - Dibujar Círculo](#ili9341_drawcircle---dibujar-círculo)
      - [`ILI9341_DrawFilledCircle()` - Dibujar Círculo Relleno](#ili9341_drawfilledcircle---dibujar-círculo-relleno)
      - [`ILI9341_Putc()` - Renderizar Carácter](#ili9341_putc---renderizar-carácter)
      - [`ILI9341_Puts()` - Renderizar Cadena](#ili9341_puts---renderizar-cadena)
      - [`ILI9341_GetStringSize()` - Calcular Tamaño de Cadena](#ili9341_getstringsize---calcular-tamaño-de-cadena)
      - [`ILI9341_DisplayImage()` - Transferir Frame Buffer](#ili9341_displayimage---transferir-frame-buffer)
      - [Funciones de Frame Buffer fuera de Pantalla](#funciones-de-frame-buffer-fuera-de-pantalla)
      - [`ILI9341_Flush()` - Volcar Frame Buffer SDRAM *(solo SDRAM)*](#ili9341_flush---volcar-frame-buffer-sdram-solo-sdram)
      - [`ILI9341_GetFrameBuffer()` - Obtener Puntero al Frame Buffer SDRAM *(solo SDRAM)*](#ili9341_getframebuffer---obtener-puntero-al-frame-buffer-sdram-solo-sdram)
      - [`ILI9341_TP_Config()` - Configurar Panel Táctil](#ili9341_tp_config---configurar-panel-táctil)
      - [`ILI9341_TP_GetState()` - Obtener Estado del Toque](#ili9341_tp_getstate---obtener-estado-del-toque)
  - [Colores Predefinidos](#colores-predefinidos)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.0.0\] - 08-06-2026](#100---08-06-2026)
      - [Added](#added)

---

## Descripción

Librería desarrollada en C para el control de la pantalla TFT LCD **Ilitek ILI9341** de 240×320 píxeles integrada en la tarjeta **STM32F429-Discovery**. Proporciona una API completa para inicialización, primitivas de dibujo (píxeles, líneas, rectángulos, círculos), renderizado de texto con fuentes personalizables, transferencia eficiente de imágenes completas y soporte de frame buffer fuera de pantalla tanto en RAM interna como en la SDRAM externa IS42S16400J de 4 MB. Incluye además la gestión completa del panel táctil resistivo basado en el controlador **STMPE811** por I2C.

Diseñada para ser portable y robusta: toda función pública retorna un código de estado `ILI9341_Status_t` que permite detectar errores de comunicación SPI/I2C, timeouts y condiciones de driver no inicializado.

---

## Características

- **Comunicación SPI optimizada**: Inicialización a 2 Mbit/s para la secuencia de configuración del chip; tras `ILI9341_Init()` el preescalador se eleva automáticamente a 45 Mbit/s para máxima velocidad de refresco.
- **Escrituras SPI optimizadas mediante acceso directo al registro `DR`**: `ILI9341_Fill()`, `ILI9341_DrawFilledRectangle()`, `ILI9341_Putc()` e `ILI9341_DisplayImage()` acceden directamente al registro `DR` del SPI, eliminando el overhead de `HAL_SPI_Transmit` y maximizando el rendimiento en todas las operaciones de relleno y renderizado de texto.
- **Primitivas de dibujo completas**: Píxeles, líneas (algoritmo de Bresenham), rectángulos (contorno y relleno) y círculos (contorno y relleno) directamente sobre la pantalla.
- **Renderizado de texto**: `ILI9341_Putc()` / `ILI9341_Puts()` con soporte de saltos de línea, retorno de carro y fuentes de ancho variable mediante `LCD_FontDef_t`.
- **Frame buffer fuera de pantalla (RAM)**: Juego completo de funciones `*_ImageBuffer()` que operan sobre un array `uint32_t[38 400]` en RAM interna, empaquetando dos píxeles RGB565 por palabra de 32 bits. Ideal para composición de imagen sin parpadeo.
- **Frame buffer en SDRAM** *(requiere `HAL_SDRAM_MODULE_ENABLED`)*: `ILI9341_Init()` acepta un `SDRAM_HandleTypeDef*` opcional; si no es NULL, inicializa la IS42S16400J y reserva los primeros 153 600 bytes de `0xD0000000` como frame buffer interno. `ILI9341_Flush()` vuelca el buffer a pantalla con una sola llamada.
- **Panel táctil resistivo STMPE811**: Configuración, lectura de coordenadas X/Y calibradas ([0, 239] × [0, 319]) y presión Z por I2C, con filtro de histeresis de 5 puntos.
- **Cuatro orientaciones de pantalla**: Portrait 0°/180° y Landscape 90°/270° configurables en tiempo de ejecución con `ILI9341_Rotate()`.
- **Manejo robusto de errores**: Retornos `ILI9341_Status_t` en todas las funciones públicas con valores específicos para parámetro inválido, timeout de SPI y driver no inicializado.
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
/* Solo SPI (sin panel táctil ni SDRAM) */
ILI9341_Status_t status = ILI9341_Init(&hspi5);

/* Con panel táctil I2C, sin SDRAM */
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hi2c3);

/* Con SDRAM, sin panel táctil */
#ifdef HAL_SDRAM_MODULE_ENABLED
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hsdram);
#endif

/* Con panel táctil I2C y SDRAM */
#ifdef HAL_SDRAM_MODULE_ENABLED
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hi2c3, &hsdram);
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
> El SPI debe estar previamente inicializado a ~5.6 Mbit/s mediante `MX_SPI5_Init()`. La librería elevará el preescalador automáticamente al final de `ILI9341_Init()`. Tanto el handle I2C (`hi2c`) como el SDRAM (`hsdram`) son opcionales: usa la sobrecarga que corresponda a los periféricos de tu proyecto.

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

/* Dibujar el contorno de un círculo */
ILI9341_DrawCircle(120, 160, 50, ILI9341_COLOR_CYAN);

/* Dibujar un círculo relleno */
ILI9341_DrawFilledCircle(120, 160, 30, ILI9341_COLOR_MAGENTA);

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

Inicializa la pantalla ILI9341 y, opcionalmente, la SDRAM IS42S16400J. Aplica la secuencia de configuración del chip, enciende la pantalla y eleva el preescalador SPI a 45 Mbit/s al finalizar. El estado inicial es Portrait 1 (240×320).

```c
/* Solo SPI (sin panel táctil ni SDRAM) */
ILI9341_Status_t status = ILI9341_Init(&hspi5);

/* Con panel táctil I2C, sin SDRAM */
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hi2c3);

/* Con SDRAM, sin panel táctil */
#ifdef HAL_SDRAM_MODULE_ENABLED
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hsdram);
#endif

/* Con panel táctil I2C y SDRAM */
#ifdef HAL_SDRAM_MODULE_ENABLED
ILI9341_Status_t status = ILI9341_Init(&hspi5, &hi2c3, &hsdram);
#endif

if (status != ILI9341_OK) {
    Error_Handler();
}
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `hspi` | `SPI_HandleTypeDef*` | Handle del periférico SPI5 |
| `hi2c` | `I2C_HandleTypeDef*` | Handle del periférico I2C3 (panel táctil) |
| `hsdram` | `SDRAM_HandleTypeDef*` | Handle SDRAM del FMC. `NULL` deshabilita el frame buffer en SDRAM |

**Retorna**: `ILI9341_OK` si la inicialización fue exitosa, `ILI9341_INVALID_PARAM` si `hspi` o `hi2c` son NULL, `ILI9341_ERROR` si una transmisión SPI falló durante la configuración.

**Secuencia interna:**
1. Validación de parámetros y reset por hardware.
2. Envío de la secuencia de comandos de configuración del ILI9341 (power, gamma, MAC, pixel format, etc.).
3. Salida del modo sleep y encendido de la pantalla.
4. Reinicialización de SPI5 con preescalador `/2` (45 Mbit/s).
5. *(Si `hsdram != NULL`)* Ejecución de la secuencia de inicialización SDRAM y mapeo del frame buffer en `0xD0000000`.

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

Transfiere un frame buffer RGB565 de pantalla completa a la LCD mediante acceso directo al registro `DR` del SPI para máxima velocidad. Cada palabra `uint32_t` contiene **dos píxeles RGB565**: el píxel de índice par en los bits `[15:0]` (word bajo) y el de índice impar en los bits `[31:16]` (word alto).

```c
ILI9341_Status_t ILI9341_DisplayImage(uint32_t image[IMG_TOTAL_BUF32]);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `image` | `uint32_t[38400]` | Frame buffer con `IMG_TOTAL_BUF32` = 38 400 palabras (153 600 bytes) |

**Retorna**: `ILI9341_OK` si todos los píxeles se enviaron, `ILI9341_NOT_INITIALIZED`, `ILI9341_ERROR` si el SPI estaba ocupado, `ILI9341_TIMEOUT` si el bus se bloqueó.

---

#### Funciones de Frame Buffer fuera de Pantalla

Estas funciones realizan la misma operación que sus equivalentes directas, pero escriben en un `uint32_t image[IMG_TOTAL_BUF32]` en RAM en lugar de enviar datos por SPI. El formato de empaquetado es idéntico al requerido por `ILI9341_DisplayImage()`.

> [!NOTE]
> Las versiones `_ImageBuffer` de Putc y Puts **no reciben color de fondo**: solo dibujan los píxeles del trazo en primer plano, dejando el resto del buffer sin modificar (renderizado transparente). Usa `memset` o `DrawFilledRectangle_ImageBuffer` para limpiar el fondo antes de renderizar texto.

| Función | Descripción |
|---------|-------------|
| `ILI9341_DrawPixel_ImageBuffer(x, y, color, image)` | Escribe un píxel en el buffer |
| `ILI9341_DrawLine_ImageBuffer(x0, y0, x1, y1, color, image)` | Dibuja una línea (Bresenham) |
| `ILI9341_DrawRectangle_ImageBuffer(x0, y0, x1, y1, color, image)` | Contorno de rectángulo |
| `ILI9341_DrawFilledRectangle_ImageBuffer(x0, y0, x1, y1, color, image)` | Rectángulo relleno |
| `ILI9341_DrawFilledCircle_ImageBuffer(x0, y0, r, color, image)` | Círculo relleno |
| `ILI9341_Putc_ImageBuffer(x, y, c, font, fg, image)` | Carácter (sin fondo) |
| `ILI9341_Puts_ImageBuffer(x, y, str, font, fg, image)` | Cadena (sin fondo) |

---

#### `ILI9341_Flush()` - Volcar Frame Buffer SDRAM *(solo SDRAM)*

Equivalente a llamar `ILI9341_DisplayImage(ILI9341_GetFrameBuffer())`. Requiere que `ILI9341_Init()` haya sido invocado con un handle SDRAM válido.

```c
ILI9341_Status_t ILI9341_Flush(void);
```

**Retorna**: `ILI9341_OK`, `ILI9341_NOT_INITIALIZED`, `ILI9341_INVALID_PARAM` (si la SDRAM no fue habilitada), `ILI9341_TIMEOUT` o `ILI9341_ERROR`.

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
