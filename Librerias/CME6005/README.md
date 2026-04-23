# Librería para el módulo receptor WWVB/60 kHz (CME6005) en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
[![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/CME6005)
[![Protocol](https://img.shields.io/badge/Protocol-WWVB%2060kHz-orange.svg)](https://www.nist.gov/pml/time-and-frequency-division/radio-stations/wwvb)

---

## Tabla de Contenidos
- [Librería para el módulo receptor WWVB/60 kHz (CME6005) en STM32](#librería-para-el-módulo-receptor-wwvb60-khz-cme6005-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
  - [Configuración GPIO en CubeMX](#configuración-gpio-en-cubemx)
    - [Pin PON](#pin-pon)
    - [Pin NTCO](#pin-ntco)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Configurar interrupciones GPIO](#2-configurar-interrupciones-gpio)
    - [3. Leer la hora](#3-leer-la-hora)
    - [4. Ciclo de bajo consumo](#4-ciclo-de-bajo-consumo)
  - [API Reference](#api-reference)
    - [Tipos de Datos](#tipos-de-datos)
      - [`CME6005_Status_t` — Códigos de retorno](#cme6005_status_t--códigos-de-retorno)
      - [`WWVB_Tiempo_t` — Tiempo decodificado](#wwvb_tiempo_t--tiempo-decodificado)
      - [`CME6005_Handle_t` — Handle del driver](#cme6005_handle_t--handle-del-driver)
    - [Funciones Públicas](#funciones-públicas)
      - [`CME6005_Init()` — Inicialización](#cme6005_init--inicialización)
      - [`CME6005_PowerOn()` / `CME6005_PowerDown()` — Control de energía](#cme6005_poweron--cme6005_powerdown--control-de-energía)
      - [`CME6005_Reset()` — Reinicio de sincronización](#cme6005_reset--reinicio-de-sincronización)
      - [`CME6005_NTCO_RisingEdgeCallback()` / `CME6005_NTCO_FallingEdgeCallback()`](#cme6005_ntco_risingedgecallback--cme6005_ntco_fallingedgecallback)
      - [`CME6005_IsFrameReady()` — Consulta de trama](#cme6005_isframeready--consulta-de-trama)
      - [`CME6005_GetTime()` — Obtener tiempo decodificado](#cme6005_gettime--obtener-tiempo-decodificado)
  - [Protocolo WWVB](#protocolo-wwvb)
    - [Modulación](#modulación)
    - [Estructura de la trama](#estructura-de-la-trama)
    - [Sincronización](#sincronización)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.0.0\] - 20-04-2026](#100---20-04-2026)
      - [Added](#added)

---

## Descripción

Librería desarrollada en C para la decodificación de la señal de tiempo atómico **WWVB** (60 kHz, NIST, Fort Collins CO) mediante el IC receptor **C-MAX CME6005**, integrado en el módulo **WVB-0860N-03A**.

La librería implementa una máquina de estados que se ejecuta en el contexto de interrupciones GPIO del STM32. A partir de las duraciones de los pulsos en la línea `NTCO`, decodifica la trama de 60 bits transmitida cada minuto y extrae: hora UTC, día del año, año, información de horario de verano (DST), año bisiesto y corrección UTI.

---

## Características

- Decodificación completa del protocolo WWVB (trama de 60 bits/minuto).
- Sincronización automática por detección de dos marcadores de 800 ms consecutivos.
- Validación de marcadores de posición (P0–P5) en tiempo real para detectar errores de sincronía.
- Control de encendido/apagado del módulo mediante el pin `PON` para aplicaciones de bajo consumo.
- API no bloqueante: la decodificación ocurre en interrupciones; el lazo principal solo consulta si la trama está lista.
- Compatible con todas las familias STM32 con HAL (F0, F1, F4, L0, L4, etc.).
- Sin dependencias externas; usa únicamente `HAL_GetTick()` y `HAL_GPIO_WritePin()`.

---

## Pinout y Conexiones

El módulo WVB-0860N-03A expone cuatro pines:

| Pin módulo | Dirección | Descripción | Conexión STM32 |
|------------|-----------|-------------|----------------|
| **GND**    | —         | Tierra | GND |
| **VCC**    | Entrada   | Alimentación 1.1–3.3 V | 3.3 V |
| **PON**    | Entrada   | Power ON, activo en LOW | GPIO Output (Push-Pull) |
| **NTCO**   | Salida    | Señal demodulada invertida | GPIO Input + EXTI (ambos flancos) |

> [!NOTE]
> `NTCO` corresponde al pin `TCON` del CME6005 (salida complementaria). Está en **HIGH** mientras la portadora está caída y vuelve a **LOW** cuando la portadora se restaura.

> [!WARNING]
> Alimentar el módulo con **3.3 V máximo**. El CME6005 soporta hasta 5.5 V en VCC, pero el módulo WVB-0860N-03A especifica 3.3 V como máximo operativo.

---

## Configuración GPIO en CubeMX

### Pin PON
| Parámetro | Valor |
|-----------|-------|
| Mode | GPIO Output Push-Pull |
| Pull | No pull-up/pull-down |
| Initial Output Level | High (módulo apagado al inicio) |
| Speed | Low |

### Pin NTCO
| Parámetro | Valor |
|-----------|-------|
| Mode | GPIO EXTI — Rising/Falling edge |
| Pull | Pull-down (si la salida NTCO es open-drain) |
| NVIC | Habilitar la interrupción EXTIx correspondiente |

---

## Instalación

1. Copia `CM6005.c` y `CM6005.h` a tu proyecto (ej: `Core/Src/` y `Core/Inc/`).
2. Ajusta el `#include` del encabezado HAL en `CM6005.h` según tu familia STM32:
   ```c
   // STM32F1xx:
   #include "stm32f1xx_hal.h"
   // STM32F4xx:
   #include "stm32f4xx_hal.h"
   // STM32L0xx:
   #include "stm32l0xx_hal.h"
   ```
3. Habilita la interrupción EXTI del pin NTCO en CubeMX y genera el código.
4. Incluye la librería:
   ```c
   #include "CM6005.h"
   ```

---

## Uso Básico

### 1. Inicialización

```c
#include "CM6005.h"

CME6005_Handle_t hwwvb;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    // Inicializar el driver (PON = GPIOB pin 0, NTCO = GPIOA pin 1)
    CME6005_Init(&hwwvb, GPIOB, GPIO_PIN_0, GPIOA, GPIO_PIN_1);

    // Encender el módulo y esperar estabilización (~2 s)
    CME6005_PowerOn(&hwwvb);

    while (1)
    {
        if (CME6005_IsFrameReady(&hwwvb))
        {
            WWVB_Tiempo_t t;
            CME6005_GetTime(&hwwvb, &t);

            // Usar t.horas, t.minutos, t.dia_del_anio, t.anio ...
        }
    }
}
```

### 2. Configurar interrupciones GPIO

Llama a los callbacks desde `HAL_GPIO_EXTI_Callback()` en `stm32xxxx_it.c` o `main.c`:

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_1)  // Pin NTCO
    {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
        {
            CME6005_NTCO_RisingEdgeCallback(&hwwvb);   // Portadora cae
        }
        else
        {
            CME6005_NTCO_FallingEdgeCallback(&hwwvb);  // Portadora se restaura
        }
    }
}
```

> [!NOTE]
> La interrupción EXTI del STM32 debe configurarse para **ambos flancos** (Rising + Falling).
> En el callback, leer el nivel actual del pin para discriminar el flanco.

### 3. Leer la hora

```c
WWVB_Tiempo_t tiempo;

if (CME6005_GetTime(&hwwvb, &tiempo) == CME6005_OK)
{
    printf("UTC: %02d:%02d  Dia: %03d  Anio: 20%02d\r\n",
           tiempo.horas, tiempo.minutos,
           tiempo.dia_del_anio, tiempo.anio);

    if (tiempo.dst & 0x02) {
        printf("Horario de verano vigente\r\n");
    }
}
```

### 4. Ciclo de bajo consumo

```c
// Encender solo para sincronizar una vez por hora
CME6005_PowerOn(&hwwvb);

while (!CME6005_IsFrameReady(&hwwvb))
{
    // Esperar; puede entrar en modo SLEEP entre interrupciones
    __WFI();
}

WWVB_Tiempo_t tiempo;
CME6005_GetTime(&hwwvb, &tiempo);

// Apagar el módulo hasta la próxima sincronización
CME6005_PowerDown(&hwwvb);
// Consumo en Power Down: < 0.05 µA (CME6005 datasheet)
```

---

## API Reference

### Tipos de Datos

#### `CME6005_Status_t` — Códigos de retorno

```c
typedef enum {
    CME6005_OK          = 0,  // Operación exitosa
    CME6005_ERROR       = 1,  // Error genérico
    CME6005_NOT_READY   = 2,  // Trama aún no disponible
    CME6005_FRAME_ERROR = 3,  // Error de sincronización
} CME6005_Status_t;
```

#### `WWVB_Tiempo_t` — Tiempo decodificado

```c
typedef struct {
    uint8_t  minutos;            // 0–59
    uint8_t  horas;              // 0–23 (UTC)
    uint16_t dia_del_anio;       // 1–366
    uint8_t  anio;               // 00–99 (últimos dos dígitos)
    uint8_t  dst;                // bits DST (ver tabla abajo)
    uint8_t  anio_bisiesto;      // 1 = año bisiesto
    uint8_t  segundo_intercalar; // 1 = leap second al final del mes
    int8_t   correccion_uti;     // corrección UTI en décimas de segundo
} WWVB_Tiempo_t;
```

| Campo `dst` | Significado |
|-------------|-------------|
| `0b00` (0) | Hora estándar |
| `0b10` (2) | Horario de verano vigente |
| `0b01` (1) | Horario de verano termina hoy a las 02:00 |
| `0b11` (3) | Horario de verano comienza hoy a las 02:00 |

#### `CME6005_Handle_t` — Handle del driver

Declara una instancia global por cada módulo físico. No modificar sus campos directamente.

```c
CME6005_Handle_t hwwvb;
```

---

### Funciones Públicas

#### `CME6005_Init()` — Inicialización

```c
void CME6005_Init(CME6005_Handle_t *hdev,
                  GPIO_TypeDef *pon_port, uint16_t pon_pin,
                  GPIO_TypeDef *ntco_port, uint16_t ntco_pin);
```

Inicializa el handle y pone el módulo en Power Down (PON = HIGH). Los pines deben estar habilitados previamente con `MX_GPIO_Init()`.

---

#### `CME6005_PowerOn()` / `CME6005_PowerDown()` — Control de energía

```c
void CME6005_PowerOn(CME6005_Handle_t *hdev);   // PON = LOW, espera 2 s
void CME6005_PowerDown(CME6005_Handle_t *hdev); // PON = HIGH
```

`CME6005_PowerOn()` bloquea `CME6005_POWER_ON_DELAY_MS` ms (2000 ms por defecto) para garantizar la estabilización interna del IC.

---

#### `CME6005_Reset()` — Reinicio de sincronización

```c
void CME6005_Reset(CME6005_Handle_t *hdev);
```

Fuerza una nueva sincronización sin apagar el módulo. Útil tras detectar errores consecutivos.

---

#### `CME6005_NTCO_RisingEdgeCallback()` / `CME6005_NTCO_FallingEdgeCallback()`

```c
void CME6005_NTCO_RisingEdgeCallback(CME6005_Handle_t *hdev);
void CME6005_NTCO_FallingEdgeCallback(CME6005_Handle_t *hdev);
```

Deben llamarse desde `HAL_GPIO_EXTI_Callback()`. El flanco de subida marca el inicio del segundo WWVB; el de bajada cierra el pulso y actualiza la máquina de estados.

---

#### `CME6005_IsFrameReady()` — Consulta de trama

```c
uint8_t CME6005_IsFrameReady(const CME6005_Handle_t *hdev);
// Retorna 1 si hay trama lista, 0 si no.
```

---

#### `CME6005_GetTime()` — Obtener tiempo decodificado

```c
CME6005_Status_t CME6005_GetTime(CME6005_Handle_t *hdev, WWVB_Tiempo_t *tiempo_out);
```

Copia el tiempo decodificado a `tiempo_out` y reinicia automáticamente la recepción de la siguiente trama. Retorna `CME6005_NOT_READY` si no hay trama disponible.

---

## Protocolo WWVB

### Modulación

La estación WWVB (Fort Collins, CO — 40°40'N 105°03'W, 60 kHz, 50 kW) transmite en formato OOK (On-Off Keying) con reducción de amplitud de portadora de 10 dB al inicio de cada segundo:

| Duración del pulso | Significado |
|--------------------|-------------|
| **200 ms**         | Bit "0"     |
| **500 ms**         | Bit "1"     |
| **800 ms**         | Marcador de posición (P) |

### Estructura de la trama

La trama dura exactamente **60 segundos** (bits 0–59). Los campos de tiempo se codifican en BCD:

| Bits    | Pesos BCD        | Campo                      |
|---------|------------------|----------------------------|
| 0       | —                | Marcador P0                |
| 1–3     | 40, 20, 10       | Minutos (decenas)          |
| 5–8     | 8, 4, 2, 1       | Minutos (unidades)         |
| 9       | —                | Marcador P1                |
| 12–13   | 20, 10           | Horas (decenas)            |
| 15–18   | 8, 4, 2, 1       | Horas (unidades)           |
| 19      | —                | Marcador P2                |
| 22–23   | 200, 100         | Día del año (centenas)     |
| 25–28   | 80, 40, 20, 10   | Día del año (decenas)      |
| 29      | —                | Marcador P3                |
| 30–33   | 8, 4, 2, 1       | Día del año (unidades)     |
| 36–38   | +, −, −          | Signo UTI                  |
| 39      | —                | Marcador P4                |
| 40–43   | 0.8, 0.4, 0.2, 0.1 | Corrección UTI           |
| 45–48   | 80, 40, 20, 10   | Año (decenas)              |
| 49      | —                | Marcador P5                |
| 50–53   | 8, 4, 2, 1       | Año (unidades)             |
| 54      | —                | Indicador año bisiesto     |
| 55      | —                | Advertencia seg. intercalar|
| 56–57   | —                | Bits DST                   |
| 59      | —                | Marcador de referencia (FRM)|

### Sincronización

La librería busca **dos pulsos de 800 ms consecutivos** para sincronizarse:
- **Segundo 59**: Marcador de referencia de trama (FRM) — no lleva datos.
- **Segundo 0**: Marcador P0 — inicio de la nueva trama.

Una vez sincronizada, valida en tiempo real que los bits 9, 19, 29, 39 y 49 sean marcadores. Si alguno falla, se fuerza una nueva sincronización automáticamente.

> [!NOTE]
> La sincronización inicial puede tomar entre 1 y 2 minutos dependiendo del momento en que se encienda el módulo. La recepción se ve afectada por ruido eléctrico e interferencias de RF; alejarse de fuentes de ruido (motores, pantallas, fuentes conmutadas) mejora notablemente la calidad de la señal.

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog

### [1.0.0] - 20-04-2026

#### Added
- Versión inicial del driver CME6005/WWVB para STM32 con HAL.
- Máquina de estados para sincronización y decodificación de tramas WWVB.
- Control de energía mediante el pin PON (Power Down / Power On).
- Decodificación completa: hora UTC, día del año, año, DST, año bisiesto, segundo intercalar y corrección UTI.
- Validación de marcadores de posición en tiempo real.
- API no bloqueante basada en callbacks de interrupción GPIO.
