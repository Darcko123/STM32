# Librería para el módulo WS2812 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32%20(HAL)-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)
[![Protocol](https://img.shields.io/badge/Protocol-SPI-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/WS2812)

---

## Tabla de Contenidos
- [Librería para el módulo WS2812 en STM32](#librería-para-el-módulo-ws2812-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
  - [Configuración SPI](#configuración-spi)
    - [Parámetros en STM32CubeIDE / CubeMX](#parámetros-en-stm32cubeide--cubemx)
    - [Frecuencia del bus y temporización](#frecuencia-del-bus-y-temporización)
    - [Memoria dinámica (heap)](#memoria-dinámica-heap)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Colores y transmisión](#2-colores-y-transmisión)
    - [3. Brillo global](#3-brillo-global)
    - [4. Arcoíris con HSV](#4-arcoíris-con-hsv)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`WS2812_Status_t` - Estados de Retorno](#ws2812_status_t---estados-de-retorno)
      - [Constantes](#constantes)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`WS2812_Init()` - Inicialización](#ws2812_init---inicialización)
      - [`WS2812_DeInit()` - Desinicialización](#ws2812_deinit---desinicialización)
      - [`WS2812_SetPixelColor()` / `WS2812_SetPixelColorPacked()` - Color de un Píxel](#ws2812_setpixelcolor--ws2812_setpixelcolorpacked---color-de-un-píxel)
      - [`WS2812_Fill()` - Rellenar un Rango](#ws2812_fill---rellenar-un-rango)
      - [`WS2812_SetBrightness()` / `WS2812_GetBrightness()` - Brillo Global](#ws2812_setbrightness--ws2812_getbrightness---brillo-global)
      - [`WS2812_ColorHSV()` - Conversión HSV a RGB](#ws2812_colorhsv---conversión-hsv-a-rgb)
      - [`WS2812_Clear()` - Apagar Todos los Píxeles](#ws2812_clear---apagar-todos-los-píxeles)
      - [`WS2812_Show()` - Transmitir a la Tira](#ws2812_show---transmitir-a-la-tira)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.0.0\] - 15-11-2026](#100---15-11-2026)
      - [Added](#added)

---

## Descripción

Driver en C para tiras de LEDs direccionables **WS2812 / WS2812B** (NeoPixel) sobre **STM32 HAL**. El protocolo de un solo hilo del WS2812 se genera con el periférico **SPI** usando solo la línea **MOSI**: cada bit WS2812 se codifica como 3 bits SPI (`1 → 110`, `0 → 100`), de modo que con el bus a ~2.4 MHz cada bit dura ~1.25 µs (800 kHz).

La API está inspirada en `Adafruit_NeoPixel` (relleno de rangos, brillo global con el mismo escalado, conversión `ColorHSV` equivalente), adaptada al estilo del resto de librerías del repositorio: retornos `WS2812_Status_t`, `Init`/`DeInit` y validación de parámetros.

Los cambios de color se hacen sobre un buffer en RAM y **no se ven en la tira hasta llamar a `WS2812_Show()`**.

---

## Características

- **Generación del protocolo por SPI**: no usa timers, DMA ni bit-banging; basta un SPI Master con MOSI. La transmisión es bloqueante (`HAL_SPI_Transmit`).
- **Cantidad de LEDs en tiempo de ejecución**: `WS2812_Init(hspi, num_leds)` reserva los buffers con `calloc`/`malloc` para 1 a `WS2812_MAX_LEDS` (7267) LEDs, el máximo que cabe en una sola llamada a `HAL_SPI_Transmit` (65535 bytes). Re-llamar a `Init` libera y reserva de nuevo.
- **Validación de la configuración SPI**: `WS2812_Init()` rechaza un SPI sin inicializar, que no sea Master, solo-RX, de trama distinta a 8 bits o que no sea MSB first.
- **Píxeles individuales o por rango**: `WS2812_SetPixelColor()` (R, G, B), `WS2812_SetPixelColorPacked()` (color `0x00RRGGBB`) y `WS2812_Fill()` para un rango o hasta el final de la tira. El orden de envío G, R, B lo gestiona la librería.
- **Brillo global 0-255**: se aplica al guardar cada color con el escalado de Adafruit `(c * (brillo + 1)) >> 8`; al cambiarlo con `WS2812_SetBrightness()` se re-escalan los píxeles ya almacenados.
- **Conversión HSV → RGB**: `WS2812_ColorHSV()` con matiz de 16 bits, función pura (no requiere `Init`) equivalente a `Adafruit_NeoPixel::ColorHSV`.
- **Latch automático**: cada `WS2812_Show()` añade 125 bytes a 0 al final de la trama (línea en bajo) para que la tira fije los colores.
- **Timeout configurable**: `WS2812_SPI_TIMEOUT_MS` (100 ms por defecto, margen base) se puede redefinir antes de incluir `WS2812.h`. `WS2812_Show()` le suma 1 ms por cada 100 bytes de trama (válido para f_SPI ≥ 800 kHz), así las tiras largas no dan `WS2812_TIMEOUT`.

---

## Pinout y Conexiones

| Pin de la tira | Conexión STM32 | Observaciones |
|----------------|----------------|---------------|
| **DIN** | `SPIx_MOSI` (p. ej. PA7 en SPI1) | Único pin de datos usado por la librería. SCK queda asignado al SPI pero no se conecta |
| **5V / VDD** | Fuente externa de 5 V | No alimentar la tira desde el pin de 5 V del USB salvo con muy pocos LEDs |
| **GND** | GND | **Común** entre la fuente de la tira y el STM32 |

> [!WARNING]
> Cada LED puede consumir del orden de 60 mA en blanco a brillo máximo (16 LEDs ≈ 1 A). Dimensiona la fuente o limita el brillo con `WS2812_SetBrightness()`.

> [!NOTE]
> La tira se alimenta a 5 V y el STM32 entrega 3.3 V en MOSI. Según el datasheet del WS2812B, el nivel alto de entrada es 0.7·VDD (3.5 V a 5 V), así que 3.3 V queda en el límite. Si aparecen parpadeos o colores erróneos, usa un adaptador de nivel en DIN (p. ej. 74AHCT125). Pendiente de confirmar con el módulo usado en pruebas.

---

## Configuración SPI

### Parámetros en STM32CubeIDE / CubeMX

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | Transmit Only Master *(o Full-Duplex Master)* | La librería solo transmite. `Init` rechaza Receive Only |
| **Hardware NSS Signal** | Disable | No hay CS |
| **Frame Format** | Motorola | — |
| **Data Size** | 8 Bits | Obligatorio (validado en `Init`) |
| **First Bit** | MSB First | Obligatorio (validado en `Init`) |
| **Prescaler (Baud Rate)** | Ver [tabla](#frecuencia-del-bus-y-temporización) | Objetivo ≈ **2.4 MHz** |
| **Clock Polarity (CPOL)** | Low | No se valida; SCK no se usa |
| **Clock Phase (CPHA)** | 1 Edge | No se valida; SCK no se usa |
| **CRC Calculation** | Disabled | — |

### Frecuencia del bus y temporización

Con la codificación de 3 bits SPI por bit WS2812, todos los tiempos dependen de `f_SPI = f_PCLK / Prescaler`:

- Periodo de bit WS2812 = `3 / f_SPI`
- `T0H` = `1 / f_SPI` · `T1H` = `2 / f_SPI`
- Latch (125 bytes a 0) = `1000 / f_SPI`

| f_PCLK / Prescaler | f_SPI | Bit WS2812 | T0H | T1H | Latch |
|--------------------|-------|------------|-----|-----|-------|
| Ideal | 2.4 MHz | 1.25 µs | 417 ns | 833 ns | 417 µs |
| 72 MHz / 32 | 2.25 MHz | 1.33 µs | 444 ns | 889 ns | 444 µs |
| 84 MHz / 32 | 2.625 MHz | 1.14 µs | 381 ns | 762 ns | 381 µs |
| 90 MHz / 32 | 2.8125 MHz | 1.07 µs | 356 ns | 711 ns | 356 µs |
| 100 MHz / 32 (Black Pill F411, SPI1) | 3.125 MHz | 0.96 µs | 320 ns | 640 ns | 320 µs |

> [!WARNING]
> El latch garantiza ≥ 300 µs hasta 3.33 MHz. Por encima la pausa final se acorta; entre dos `WS2812_Show()` seguidos el tiempo de codificación del buffer alarga la pausa real, pero si la tira no fija los colores, baja la frecuencia del SPI.

> [!NOTE]
> Falta confirmar con el datasheet del modelo concreto qué rango de `f_SPI` respeta sus tolerancias de `T0H`/`T1H`. El ejemplo `ExamplesLibraries/WS2812_Example` usa 3.125 MHz.

### Memoria dinámica (heap)

`WS2812_Init()` reserva `3·N` bytes para los colores y `9·N + 125` bytes para la trama SPI, es decir **`12·N + 125` bytes de heap** (más la sobrecarga del asignador). El `_Min_Heap_Size` por defecto de CubeIDE (`0x200` = 512 bytes) solo alcanza para ~30 LEDs; para tiras más largas auméntalo en el *linker script* (`.ld`) o en *Project Manager → Linker Settings → Minimum Heap Size*. Si no hay memoria, `Init` devuelve `WS2812_ERROR`.

---

## Instalación

1. Copia `WS2812.c` y `WS2812.h` a tu proyecto (ej: `Librerias/WS2812/`).
2. Incluye la librería en tu `main.c`:
   ```c
   #include "WS2812.h"
   ```
3. Configura el SPI como se describe en [Configuración SPI](#configuración-spi) y ajusta el heap según el número de LEDs.
4. Genera código y compila.

---

## Uso Básico

### 1. Inicialización

```c
#define NUM_LEDS 16

/* Después de HAL_Init(), SystemClock_Config() y MX_SPI1_Init() */
if (WS2812_Init(&hspi1, NUM_LEDS) != WS2812_OK)
{
    Error_Handler();
}
```

`Init` deja todos los píxeles apagados en el buffer y el brillo a 255, pero **no transmite**: la tira mantiene lo que tuviera hasta el primer `WS2812_Show()`.

### 2. Colores y transmisión

```c
WS2812_Clear();                              /* Todo apagado (buffer) */
WS2812_SetPixelColor(0, 255, 0, 0);          /* LED 0 en rojo */
WS2812_SetPixelColorPacked(1, 0x0000FF);     /* LED 1 en azul */
WS2812_Fill(0, 255, 0, 2, 4);                /* LEDs 2..5 en verde */
WS2812_Fill(255, 255, 255, 6, 0);            /* LEDs 6..final en blanco */

if (WS2812_Show() != WS2812_OK)              /* Envía el buffer a la tira */
{
    Error_Handler();
}
```

### 3. Brillo global

```c
WS2812_SetBrightness(32);                    /* ~12 %: re-escala lo ya guardado */
WS2812_Show();
```

> [!TIP]
> Fija el brillo **antes** de cargar los colores. Al subirlo, los píxeles ya almacenados se re-escalan con pérdida de resolución (y con brillo 0 se pierden por completo), igual que en `Adafruit_NeoPixel`.

### 4. Arcoíris con HSV

```c
uint16_t hue0 = 0;

while (1)
{
    for (uint16_t i = 0; i < NUM_LEDS; i++)
    {
        uint16_t hue = hue0 + (uint16_t)((65536UL * i) / NUM_LEDS);
        WS2812_SetPixelColorPacked(i, WS2812_ColorHSV(hue, 255, 255));
    }
    WS2812_Show();
    hue0 += 256;
    HAL_Delay(10);
}
```

---

## API Reference

### 1. Tipos de Datos

#### `WS2812_Status_t` - Estados de Retorno

```c
typedef enum {
    WS2812_OK               = 0,
    WS2812_ERROR            = 1,
    WS2812_TIMEOUT          = 2,
    WS2812_NOT_INITIALIZED  = 3,
    WS2812_INVALID_PARAM    = 4
} WS2812_Status_t;
```

| Valor | Código | Significado |
|-------|--------|-------------|
| `WS2812_OK` | 0 | Operación completada sin errores |
| `WS2812_ERROR` | 1 | Sin memoria en `Init`, o fallo del HAL en `Show` |
| `WS2812_TIMEOUT` | 2 | `HAL_SPI_Transmit()` superó `WS2812_SPI_TIMEOUT_MS` |
| `WS2812_NOT_INITIALIZED` | 3 | `WS2812_Init()` no se llamó o falló |
| `WS2812_INVALID_PARAM` | 4 | Índice fuera de la tira, `num_leds` inválido o configuración SPI no válida |

#### Constantes

| Macro | Valor | Descripción |
|-------|-------|-------------|
| `WS2812_MAX_LEDS` | 7267 | Máximo de LEDs: `7267·9 + 125 ≤ 65535` bytes por transmisión |
| `WS2812_SPI_TIMEOUT_MS` | 100 | Margen base del timeout de `HAL_SPI_Transmit()` (`Show` suma 1 ms por cada 100 bytes de trama); redefinible antes de incluir `WS2812.h` |

---

### 2. Funciones Públicas

#### `WS2812_Init()` - Inicialización

```c
WS2812_Status_t WS2812_Init(SPI_HandleTypeDef* hspi, uint16_t num_leds);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `hspi` | `SPI_HandleTypeDef*` | Handle del SPI ya inicializado por CubeMX |
| `num_leds` | `uint16_t` | LEDs de la tira (1 a `WS2812_MAX_LEDS`) |

Valida el SPI, reserva los buffers (`12·num_leds + 125` bytes de heap), apaga todos los píxeles del buffer y fija el brillo a 255. No transmite. Si ya estaba inicializado, libera los buffers anteriores y reinicia.

**Retorna**: `WS2812_OK`; `WS2812_INVALID_PARAM` si `num_leds` es 0 o supera el máximo, `hspi` es `NULL` o el SPI no está inicializado, no es Master, es solo-RX, no es de 8 bits o no es MSB first; `WS2812_ERROR` si no hay memoria.

---

#### `WS2812_DeInit()` - Desinicialización

```c
WS2812_Status_t WS2812_DeInit(void);
```

Libera los buffers y olvida el handle del SPI. No apaga la tira: para eso, `WS2812_Clear()` + `WS2812_Show()` antes de desinicializar.

**Retorna**: `WS2812_OK`, o `WS2812_NOT_INITIALIZED` si no estaba inicializado.

---

#### `WS2812_SetPixelColor()` / `WS2812_SetPixelColorPacked()` - Color de un Píxel

```c
WS2812_Status_t WS2812_SetPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
WS2812_Status_t WS2812_SetPixelColorPacked(uint16_t n, uint32_t c);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `n` | `uint16_t` | Índice del píxel (0 a `num_leds - 1`) |
| `r`, `g`, `b` | `uint8_t` | Componentes de color (0-255) |
| `c` | `uint32_t` | Color empaquetado `0x00RRGGBB` (p. ej. el de `WS2812_ColorHSV()`) |

Guarda el color en el buffer, ya escalado por el brillo global.

**Retorna**: `WS2812_OK`, `WS2812_NOT_INITIALIZED` o `WS2812_INVALID_PARAM` si `n` está fuera de la tira.

---

#### `WS2812_Fill()` - Rellenar un Rango

```c
WS2812_Status_t WS2812_Fill(uint8_t r, uint8_t g, uint8_t b, uint16_t first, uint16_t count);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `r`, `g`, `b` | `uint8_t` | Color de relleno |
| `first` | `uint16_t` | Primer píxel del rango |
| `count` | `uint16_t` | Píxeles a rellenar; `0` o un valor que se pase del final = hasta el último píxel |

**Retorna**: `WS2812_OK`, `WS2812_NOT_INITIALIZED` o `WS2812_INVALID_PARAM` si `first` está fuera de la tira.

---

#### `WS2812_SetBrightness()` / `WS2812_GetBrightness()` - Brillo Global

```c
WS2812_Status_t WS2812_SetBrightness(uint8_t brightness);
uint8_t         WS2812_GetBrightness(void);
```

`SetBrightness` fija el brillo (0-255) y re-escala los píxeles ya guardados (con pérdida al subirlo). No transmite. `GetBrightness` devuelve el brillo actual: 255 antes del primer `Init`, tras cada `Init` y tras `DeInit`.

**Retorna** (`SetBrightness`): `WS2812_OK` o `WS2812_NOT_INITIALIZED`.

---

#### `WS2812_ColorHSV()` - Conversión HSV a RGB

```c
uint32_t WS2812_ColorHSV(uint16_t hue, uint8_t sat, uint8_t val);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `hue` | `uint16_t` | Matiz: 0-65535 es una vuelta completa (rojo → verde → azul → rojo) |
| `sat` | `uint8_t` | Saturación: 0 = blanco, 255 = color puro |
| `val` | `uint8_t` | Valor: 0 = negro, 255 = máximo |

Función pura: no requiere `Init` ni aplica el brillo global (se aplica al pasarlo a `SetPixelColorPacked`). No aplica corrección gamma.

**Retorna**: color empaquetado `0x00RRGGBB`.

---

#### `WS2812_Clear()` - Apagar Todos los Píxeles

```c
WS2812_Status_t WS2812_Clear(void);
```

Pone el buffer a 0. No transmite: hace falta `WS2812_Show()`.

**Retorna**: `WS2812_OK` o `WS2812_NOT_INITIALIZED`.

---

#### `WS2812_Show()` - Transmitir a la Tira

```c
WS2812_Status_t WS2812_Show(void);
```

Codifica el buffer (3 bits SPI por bit WS2812), añade 125 bytes a 0 para el latch y lo envía con `HAL_SPI_Transmit()` de forma bloqueante. Tarda `(9·N + 125)·8 / f_SPI` más la codificación (≈ 0.7 ms de bus para 16 LEDs a 3.125 MHz).

**Retorna**: `WS2812_OK`, `WS2812_NOT_INITIALIZED`, `WS2812_TIMEOUT` si se supera `WS2812_SPI_TIMEOUT_MS`, o `WS2812_ERROR` ante cualquier otro fallo del HAL.

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](../../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [1.0.0] - 15-11-2026

Primera versión estable del driver de tiras WS2812/WS2812B por SPI.

#### Added
- **Inicialización dinámica**: `WS2812_Init()` valida la configuración del SPI y reserva en el heap los buffers para 1 a `WS2812_MAX_LEDS` LEDs; `WS2812_DeInit()` los libera.
- **Color por píxel**: `WS2812_SetPixelColor()` (R, G, B) y `WS2812_SetPixelColorPacked()` (`0x00RRGGBB`), con el orden G, R, B del WS2812 gestionado internamente.
- **Relleno de rangos**: `WS2812_Fill()` rellena desde `first` un número de píxeles o hasta el final de la tira.
- **Brillo global**: `WS2812_SetBrightness()` / `WS2812_GetBrightness()` con el escalado de `Adafruit_NeoPixel`, re-escalando los píxeles ya guardados.
- **Conversión HSV**: `WS2812_ColorHSV()` convierte un matiz de 16 bits, saturación y valor a RGB empaquetado.
- **Transmisión por SPI**: `WS2812_Clear()` apaga el buffer y `WS2812_Show()` lo codifica a 3 bits SPI por bit y lo envía con pausa de latch, con timeout configurable `WS2812_SPI_TIMEOUT_MS`.
