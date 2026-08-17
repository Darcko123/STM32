# Librería para la pantalla TFT LCD NV3007 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
[![Version](https://img.shields.io/badge/Version-0.1.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/NV3007)
[![Protocol](https://img.shields.io/badge/Protocol-SPI-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/NV3007)

---

## Tabla de Contenidos
- [Librería para la pantalla TFT LCD NV3007 en STM32](#librería-para-la-pantalla-tft-lcd-nv3007-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Alimentación](#alimentación)
    - [Bus SPI](#bus-spi)
    - [GPIO de Control](#gpio-de-control)
  - [Configuración SPI](#configuración-spi)
    - [Parámetros en STM32CubeIDE / CubeMX](#parámetros-en-stm32cubeide--cubemx)
    - [Baud Rate recomendado](#baud-rate-recomendado)
    - [Tabla de preescaladores](#tabla-de-preescaladores)
  - [Resumen de la configuración mínima](#resumen-de-la-configuración-mínima)
  - [Calibración de offsets](#calibración-de-offsets)
    - [Inversión de color](#inversión-de-color)

---

## Descripción

Driver en C para el controlador LCD **NV3007** sobre interfaz **SPI de 4 hilos** (SCK, MOSI, CS y un pin DC dedicado para distinguir comando de dato), ajustado al panel **Estardyn 2.79" de 142×428 píxeles**. Es la traducción a **STM32 HAL** de la librería `Arduino_NV3007` del proyecto [Arduino_GFX](https://github.com/moononournation/Arduino_GFX), conservando sus primitivas de dibujo.

La secuencia de inicialización es la variante de 2.79", que coincide byte a byte con el fichero de inicialización del fabricante (*NV3006A1N/NV3007 + IVO2.66*). La secuencia estándar de 168 columnas de Arduino_GFX **no** genera imagen en este panel aunque el backlight encienda.

> [!IMPORTANT]
> La GRAM del NV3007 es de **168×428**, pero el panel solo expone **142 columnas**. El área visible no empieza en la columna 0, así que `CASET`/`RASET` aplican un desplazamiento (`NV3007_OFFSET_*` en `NV3007.h`). Sin él la imagen sale corrida y recortada. Ver [Calibración de offsets](#calibración-de-offsets).

El formato de color es **RGB565 (16 bits por píxel)**, fijado por el comando `0x3A = 0x05` dentro de la secuencia de inicialización.

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
> El **Modo SPI 0** (CPOL = Low, CPHA = 1 Edge) no es opcional: la librería original fuerza `SPI_MODE0` para este panel (`_override_datamode = SPI_MODE0` en `Arduino_NV3007.cpp`). Con cualquier otro modo el panel no responde correctamente a la secuencia de inicialización.

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

## Resumen de la configuración mínima

1. **SPI**: *Transmit Only Master*, 8 bits, MSB First, CPOL = Low, CPHA = 1 Edge, NSS por software, preescalador que dé ≈ 20 Mbit/s.
2. **Tres GPIO de salida** (`GPIO_OUTPUT`, push-pull, sin pull): CS y DC en *Very High speed*, RST en *Low speed*.
3. **CS y RST** con nivel inicial **High**.
4. Alimentación **3.3 V** y masa común; retroiluminación cableada aparte (la librería no la controla).
5. En el código:

```c
/* Después de HAL_Init(), MX_GPIO_Init() y MX_SPIx_Init() */
if (NV3007_Init(&hspi1,
                LCD_CS_GPIO_Port,  LCD_CS_Pin,
                LCD_DC_GPIO_Port,  LCD_DC_Pin,
                LCD_RST_GPIO_Port, LCD_RST_Pin) != NV3007_OK)
{
    Error_Handler();
}

NV3007_FillScreen(0x0000);  /* Pantalla en negro (RGB565) */
```

> [!IMPORTANT]
> `MX_GPIO_Init()` debe ejecutarse **antes** que `NV3007_Init()`, ya que el reset por hardware y el primer `CS = HIGH` se aplican dentro de la inicialización de la librería.

> [!TIP]
> `NV3007_Init()` valida la configuración del periférico SPI y devuelve `NV3007_INVALID_PARAM` si el tamaño de trama no es de 8 bits, si no es MSB First, o si el modo SPI no es 0 ni 3. Un bus mal configurado deja el panel en negro con el backlight encendido, síntoma indistinguible de un fallo de cableado; esta comprobación lo descarta de entrada.

---

## Calibración de offsets

La GRAM del NV3007 es de 168×428 y el panel expone 142 columnas, así que el área visible arranca desplazada. El driver suma un offset a `CASET`/`RASET` en cada `NV3007_WriteAddrWindow()`.

Los valores viven en `NV3007.h`, uno por orientación:

| Constante | Valor | Origen |
|-----------|-------|--------|
| `NV3007_OFFSET_P1_X` / `_P1_Y` | 12 / 0 | Confirmado en hardware por dos implementaciones independientes |
| `NV3007_OFFSET_P2_X` / `_P2_Y` | 14 / 0 | Derivado de la geometría (168 − 142 − 12 = 14) |
| `NV3007_OFFSET_L1_X` / `_L1_Y` | 0 / 14 | Derivado |
| `NV3007_OFFSET_L2_X` / `_L2_Y` | 0 / 12 | Derivado |

Solo el par de `Portrait_1` está verificado sobre este panel. Para ajustar los demás sin recompilar cada intento, usa `NV3007_SetOffset()`:

```c
NV3007_Rotate(NV3007_Orientation_Landscape_1);
NV3007_SetOffset(0, 14);                 /* prueba y ajusta de 1 en 1 */

NV3007_FillScreen(NV3007_COLOR_BLACK);
NV3007_DrawRectangle(0, 0, NV3007_WIDTH, NV3007_HEIGHT, NV3007_COLOR_WHITE);
```

El marco debe verse **completo y pegado al borde físico** del panel. Si falta un lado o queda un hueco, corrige el offset y repite. Cuando encaje, traslada los números a las constantes `NV3007_OFFSET_*` y vuelve a probar sin llamar a `NV3007_SetOffset()`.

> [!NOTE]
> `NV3007_Rotate()` recarga los offsets desde la tabla, así que descarta cualquier valor puesto con `NV3007_SetOffset()`. Llama siempre a `NV3007_SetOffset()` **después** de `NV3007_Rotate()`.

### Inversión de color

Este módulo monta un panel **IPS**, cuya polaridad de inversión va al revés que la de un TN. `NV3007.h` define `NV3007_IPS` a `1`, lo que hace que `NV3007_Init()` deje el panel con `INVON`, que es la configuración validada para el 2.79" 142×428.

Si los colores salen **en negativo** (un fondo negro se ve blanco), define `NV3007_IPS` a `0` antes de incluir la cabecera, o cámbialo en `NV3007.h`.
