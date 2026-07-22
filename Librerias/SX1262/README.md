# SX1262 LoRa Library for STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-2.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/SX1262)
[![Protocol](https://img.shields.io/badge/Protocol-LoRa%20%7C%20FSK-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/SX1262)

---

## Tabla de Contenidos
- [SX1262 LoRa Library for STM32](#sx1262-lora-library-for-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
    - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
      - [Bus SPI](#bus-spi)
      - [GPIO de Control](#gpio-de-control)
    - [Configuración EXTI para DIO1 (Modos No Bloqueantes)](#configuración-exti-para-dio1-modos-no-bloqueantes)
      - [Pasos en STM32CubeMX](#pasos-en-stm32cubemx)
  - [Configuración SPI](#configuración-spi)
  - [Instalación](#instalación)
  - [Uso básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Transmisión simple (bloqueante)](#2-transmisión-simple-bloqueante)
    - [3. Transmisión no bloqueante (basada en interrupciones)](#3-transmisión-no-bloqueante-basada-en-interrupciones)
    - [4. Recepción bloqueante](#4-recepción-bloqueante)
    - [5. Recepción no bloqueante (basada en interrupciones)](#5-recepción-no-bloqueante-basada-en-interrupciones)
    - [6. Configuración personalizada](#6-configuración-personalizada)
    - [7. Ejemplo de Transmisión y Recepción Completa](#7-ejemplo-de-transmisión-y-recepción-completa)
    - [8. Transmisión y recepción en FSK/GFSK](#8-transmisión-y-recepción-en-fskgfsk)
    - [9. Listen-before-talk con CAD](#9-listen-before-talk-con-cad)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`SX1262_Status_t` - Estados de Retorno](#sx1262_status_t---estados-de-retorno)
      - [`lora_config_t` - Configuración de Parámetros LoRa](#lora_config_t---configuración-de-parámetros-lora)
      - [`lora_network_mode_t` - Modos de Red LoRa](#lora_network_mode_t---modos-de-red-lora)
      - [`lora_signal_bandwidth_t` - Ancho de Banda LoRa](#lora_signal_bandwidth_t---ancho-de-banda-lora)
      - [`lora_coding_rate_t` - Coding Rate LoRa](#lora_coding_rate_t---coding-rate-lora)
      - [`fsk_config_t` - Configuración de Parámetros FSK/GFSK](#fsk_config_t---configuración-de-parámetros-fskgfsk)
      - [`fsk_shaping_t` - Shaping Gaussiano FSK](#fsk_shaping_t---shaping-gaussiano-fsk)
      - [`fsk_rx_bandwidth_t` - Ancho de Banda de Recepción FSK](#fsk_rx_bandwidth_t---ancho-de-banda-de-recepción-fsk)
      - [`fsk_crc_type_t` - Tipo de CRC FSK](#fsk_crc_type_t---tipo-de-crc-fsk)
      - [Banderas `volatile` de Evento](#banderas-volatile-de-evento)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`SX1262_Init()` - Inicialización del Driver](#sx1262_init---inicialización-del-driver)
      - [`SX1262_LoRa_Transmit()` - Transmisión Bloqueante](#sx1262_lora_transmit---transmisión-bloqueante)
      - [`SX1262_LoRa_StartTransmitIT()` - Iniciar Transmisión No Bloqueante](#sx1262_lora_starttransmitit---iniciar-transmisión-no-bloqueante)
      - [`SX1262_LoRa_GetTransmitStatus()` - Confirmar Resultado de TX](#sx1262_lora_gettransmitstatus---confirmar-resultado-de-tx)
      - [`SX1262_LoRa_AbortTransmit()` - Cancelar Transmisión](#sx1262_lora_aborttransmit---cancelar-transmisión)
      - [`SX1262_LoRa_Receive()` - Recepción Bloqueante](#sx1262_lora_receive---recepción-bloqueante)
      - [`SX1262_LoRa_StartReceiveIT()` - Iniciar Recepción No Bloqueante](#sx1262_lora_startreceiveit---iniciar-recepción-no-bloqueante)
      - [`SX1262_LoRa_GetReceivedPacket()` - Leer Paquete Recibido](#sx1262_lora_getreceivedpacket---leer-paquete-recibido)
      - [`SX1262_LoRa_AbortReceive()` - Cancelar Recepción](#sx1262_lora_abortreceive---cancelar-recepción)
      - [`SX1262_LoRa_ChannelActivityDetection()` - Detección de Actividad en el Canal (CAD)](#sx1262_lora_channelactivitydetection---detección-de-actividad-en-el-canal-cad)
      - [`SX1262_IRQ_Handler()` - Manejador de Interrupción](#sx1262_irq_handler---manejador-de-interrupción)
      - [`SX1262_LoRa_ApplyConfig()` - Aplicar Configuración LoRa](#sx1262_lora_applyconfig---aplicar-configuración-lora)
      - [`SX1262_LoRa_GetConfig()` - Obtener Configuración Actual](#sx1262_lora_getconfig---obtener-configuración-actual)
      - [`SX1262_LoRa_GetPacketStatus()` - Obtener RSSI y SNR del último paquete](#sx1262_lora_getpacketstatus---obtener-rssi-y-snr-del-último-paquete)
      - [`SX1262_LoRa_GetRSSI()` - Obtener RSSI del último paquete recibido](#sx1262_lora_getrssi---obtener-rssi-del-último-paquete-recibido)
      - [`SX1262_LoRa_GetSNR()` - Obtener SNR del último paquete recibido](#sx1262_lora_getsnr---obtener-snr-del-último-paquete-recibido)
      - [`SX1262_GetRSSIInst()` - Obtener RSSI instantáneo del canal](#sx1262_getrssiinst---obtener-rssi-instantáneo-del-canal)
    - [3. Funciones FSK/GFSK](#3-funciones-fskgfsk)
    - [4. Funciones de Diagnóstico](#4-funciones-de-diagnóstico)
      - [`SX1262_GetLastIrqStatus()` - Último registro IRQ de RX](#sx1262_getlastirqstatus---último-registro-irq-de-rx)
    - [5. Funciones de Gestión de Energía](#5-funciones-de-gestión-de-energía)
      - [`SX1262_SetSleep()` - Entrar en Modo Reposo](#sx1262_setsleep---entrar-en-modo-reposo)
      - [`SX1262_Wakeup()` - Despertar del Modo Reposo](#sx1262_wakeup---despertar-del-modo-reposo)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[2.0.0\] - 21-07-2026](#200---21-07-2026)
      - [Added](#added)
      - [Changed](#changed)
      - [Fixed](#fixed)
    - [\[1.6.0\] - 27-04-2026](#160---27-04-2026)
      - [Changed](#changed-1)
    - [\[1.5.0\] - 19-04-2026](#150---19-04-2026)
      - [Added](#added-1)
      - [Changed](#changed-2)
    - [\[1.4.0\] - 17-04-2026](#140---17-04-2026)
      - [Added](#added-2)
    - [\[1.3.0\] - 05-04-2026](#130---05-04-2026)
      - [Added](#added-3)
    - [\[1.2.0\] - 03-04-2026](#120---03-04-2026)
      - [Added](#added-4)
    - [\[1.1.0\] - 01-04-2026](#110---01-04-2026)
      - [Added](#added-5)
    - [\[1.0.1\] - 30-03-2026](#101---30-03-2026)
      - [Fixed](#fixed-1)
    - [\[1.0.0\] - 28-03-2026](#100---28-03-2026)
      - [Added](#added-6)

## Descripción
Librería desarrollada en C para la interfaz con el módulo transceptor **Semtech SX1262** utilizando microcontroladores STM32. Proporciona funciones para configurar parámetros de comunicación, transmitir y recibir datos en las modulaciones **LoRa** y **FSK/GFSK**, y manejar eventos de interrupción. La librería está diseñada para ser fácil de usar, eficiente y compatible con la mayoría de las series STM32 (F1, F4, etc.) utilizando HAL. Soporta configuraciones avanzadas de LoRa como Spreading Factor, Bandwidth, Coding Rate y potencia de transmisión, detección de actividad en el canal (CAD) para *listen-before-talk*, y telemetría de señal (RSSI/SNR). Ideal para proyectos de IoT, sensores remotos y redes de baja potencia.

---

### Características
- **Doble modulación LoRa y FSK/GFSK**: API simétrica con prefijos `SX1262_LoRa_*` y `SX1262_FSK_*`. Ambos modos son mutuamente excluyentes en el chip; se conmuta llamando a `SX1262_LoRa_ApplyConfig()` o `SX1262_FSK_ApplyConfig()`.
- **Comunicación SPI**: Abstracción de comandos (Write/Read registers, buffers) con manejo robusto del flag OVR.
- **Configuración completa de parámetros LoRa**: Frecuencia (433/868/915 MHz), Spreading Factor (SF5-12), Bandwidth (7.8-500 kHz), Coding Rate (4/5 a 4/8).
- **Configuración completa de parámetros FSK/GFSK**: Bit rate, desviación de frecuencia, RX bandwidth, shaping gaussiano (BT), sync word de hasta 8 bytes, CRC (1/2 bytes, con/sin inversión), whitening y paquetes de longitud fija o variable.
- **Transmisión bloqueante** (`SX1262_LoRa_Transmit` / `SX1262_FSK_Transmit`): Espera por IRQ vía polling en DIO1 (TxDone + Timeout), con timeout de software calculado a partir del Time on Air real.
- **Transmisión no bloqueante** (`..._StartTransmitIT` + `..._GetTransmitStatus`): El CPU no se bloquea. El chip notifica TX_DONE vía EXTI en DIO1; la bandera `SX1262_LoRa_TxDoneFlag` señaliza el evento. Incluye `..._AbortTransmit()` para timeouts de software.
- **Recepción bloqueante** (`SX1262_LoRa_Receive` / `SX1262_FSK_Receive`): Espera en polling hasta RxDone, con timeout configurable y protección de tamaño de buffer (`max_length`).
- **Recepción no bloqueante** (`..._StartReceiveIT` + `..._GetReceivedPacket`): El chip notifica vía EXTI en DIO1. El CPU no se bloquea; la bandera `SX1262_LoRa_RxDoneFlag` señaliza el evento. Incluye `..._AbortReceive()` para timeouts de software.
- **Protección contra desbordamiento en RX**: Todas las funciones de recepción reciben la capacidad del buffer (`max_length`) y retornan `SX1262_RX_BUFFER_TOO_SMALL` sin tocar el buffer si el paquete no cabe, devolviendo el tamaño real en `*length`.
- **Detección de actividad en el canal (CAD)** (`SX1262_LoRa_ChannelActivityDetection`): Primitiva *listen-before-talk* rápida y de bajo consumo; sensibilidad derivada del SF según la nota de aplicación Semtech AN1200.48.
- **Telemetría de señal**: `SX1262_LoRa_GetPacketStatus` (RSSI + SNR en una sola lectura), `SX1262_LoRa_GetRSSI`, `SX1262_LoRa_GetSNR` y `SX1262_GetRSSIInst` (RSSI instantáneo del canal, útil para escáner de espectro).
- **Dispatcher ISR unificado** (`SX1262_IRQ_Handler`): Un único callback EXTI en DIO1 maneja tanto TX como RX en ambas modulaciones. El semáforo interno `SX1262_TxActive` decide qué bandera activar, sin realizar ninguna comunicación SPI desde el ISR.
- **Soporte para redes públicas, privadas y Meshtastic**: SyncWord configurable por modo o valor personalizado. Presets Meshtastic listos para usar (`LongFast`, `LongSlow`, `MediumFast`, etc.).
- **LDRO automático**: Calcula y activa LowDataRateOptimize dinámicamente según SF y BW, siguiendo la sección 6.1.1.4 del datasheet.
- **Diagnóstico de recepción** (`SX1262_GetLastIrqStatus`): Expone el último registro IRQ leído para diagnosticar por qué una RX no devolvió `SX1262_OK`.
- **Manejo robusto de hardware**: Busy polling con timeout, reset hardware, wakeup desde Sleep.
- **Portabilidad sin edición manual**: `#include "main.h"` toma automáticamente el encabezado HAL de tu familia STM32 generado por CubeMX. Ya no es necesario editar el `#include` a mano.
- **Potencia TX**: Hasta +22 dBm, con rampa configurable.
- **CRC automático habilitado** en todos los paquetes transmitidos.
- **Gestión robusta de errores**: Códigos de retorno específicos para parámetro inválido, timeout, error de SPI, inicialización no completada, buffer insuficiente, RX/TX ocupado y flancos espurios sin paquete.

## Pinout y Conexiones
### Pines requeridos

#### Bus SPI

| Pin SX1262 | Dirección | Descripción | Configuración CubeMX |
|------------|-----------|-------------|----------------------|
| **VCC**    | Alimentación | 3.3V ±5% (hasta 120mA TX) | N/A |
| **GND**    | Tierra    | —                          | N/A |
| **SCK**    | Output    | SPI Clock                  | Configurado por periférico SPI |
| **MISO**   | Input     | SPI Data In                | Configurado por periférico SPI |
| **MOSI**   | Output    | SPI Data Out               | Configurado por periférico SPI |
| **ANT**    | RF        | Antena 50Ω (SAW filter recomendado) | N/A |

#### GPIO de Control

| Pin SX1262 | Dirección | Descripción | Tipo GPIO | Nivel por defecto | GPIO Pull-up/Pull-down | Velocidad Máxima de Salida | Etiqueta CubeMX |
|------------|-----------|-------------|-----------|-------------------|------------------------|----------------------------|-----------------|
| **NSS/CS** | Output    | Chip Select (SPI)          | `GPIO_OUTPUT` | HIGH | No pull-up no pull-down | Very High | `NSS` |
| **RST**    | Output    | Reset del módulo           | `GPIO_OUTPUT` | HIGH | No pull-up no pull-down | Low |`RST` |
| **BUSY**   | Input     | Estado del chip (polling)  | `GPIO_INPUT` | — | No pull-up no pull-down | n/a |`BUSY` |
| **DIO1**   | Input     | IRQ (Tx/Rx done) — **modo bloqueante:** `GPIO_INPUT`, **modo IT:** `GPIO_EXTI` flanco subida | `GPIO_INPUT` / `GPIO_MODE_IT_RISING` | — | No pull-up no pull-down - ver seccion [EXIT](#configuración-exti-para-dio1-modos-no-bloqueantes) | n/a |`DIO` |

> [!NOTE]
>  DIO2 es controlado internamente por la librería como RF Switch (`SET_DIO2_AS_RF_SWITCH_CTRL`). No necesita configurarse como GPIO externo en CubeMX salvo que el hardware del módulo lo requiera diferente.

---

### Configuración EXTI para DIO1 (Modos No Bloqueantes)

Para utilizar las funciones no bloqueantes (`SX1262_LoRa_StartTransmitIT` / `SX1262_LoRa_StartReceiveIT`), el pin DIO1 debe configurarse como entrada con interrupción externa (EXTI) en lugar de entrada GPIO simple. El chip SX1262 mantiene DIO1 en bajo en reposo y genera un **flanco de subida** al completar una operación (TxDone, RxDone, Timeout, etc.).

> [!NOTE]
> La misma configuración EXTI en DIO1 sirve tanto para transmisión como para recepción no bloqueante. El dispatcher `SX1262_IRQ_Handler()` determina automáticamente qué bandera activar (`SX1262_LoRa_TxDoneFlag` o `SX1262_LoRa_RxDoneFlag`) según el semáforo interno `SX1262_TxActive`.

#### Pasos en STM32CubeMX

> [!IMPORTANT]
> Los pasos siguientes asumen que DIO1 está conectado al pin **PXx** del STM32F429. Si usas otro pin, adapta el número de línea EXTI y el handler correspondiente.

**Paso 1 — Reconfigurar el modo del pin DIO1**

En la vista de pines (*Pinout & Configuration*), hacer clic sobre **PXx** y seleccionar:

```
GPIO_EXTIx
```

Esto configura automáticamente PXx en modo `GPIO_MODE_IT_RISING`.

**Paso 2 — Configurar los parámetros GPIO del pin**

En *GPIO* → seleccionar `PXx` → pestaña *Configuration*:

| Parámetro | Valor |
|-----------|-------|
| GPIO Mode | `External Interrupt Mode with Rising edge trigger detection` |
| GPIO Pull-up/Pull-down | `No pull-up and no pull-down` |
| User Label | `DIO` *(sin cambio)* |

**Paso 3 — Habilitar la interrupción en el NVIC**

En la pestaña *NVIC* (dentro de *System Core → NVIC*), localizar y habilitar:

| Interrupción | Handler generado | Estado |
|---|---|---|
| `EXTI line2 interrupt` | `EXTI2_IRQHandler` | ✅ **Enabled** |

Prioridad recomendada:

| Campo | Valor | Motivo |
|-------|-------|--------|
| Preemption Priority | `5` | Mayor que UART/SPI, menor que SysTick (0) |
| Sub Priority | `0` | — |

> [!WARNING]
> No asignes prioridad `0` a la línea EXTI. SysTick (que alimenta `HAL_GetTick`) corre en prioridad 0. Si EXTI2 también tiene prioridad 0, podría provocar una inanición del SysTick.

**Paso 4 — Regenerar el código**

Al regenerar con CubeMX, los siguientes cambios se aplican automáticamente:

- `MX_GPIO_Init()` configura PXx con `GPIO_MODE_IT_RISING`.
- `stm32f4xx_it.c` añade el handler `EXTI2_IRQHandler()` que llama a `HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2)`.

**Paso 5 — Añadir el callback en tu `main.c`**

Dentro del bloque `/* USER CODE BEGIN 4 */` en `main.c`, implementa:

```c
/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DIO_Pin)
    {
        SX1262_IRQ_Handler();  // Solo activa la bandera — sin SPI en el ISR
    }
}
/* USER CODE END 4 */
```

> [!NOTE]
> `SX1262_IRQ_Handler()` actúa como **dispatcher**: si `SX1262_TxActive == 1` activa `SX1262_LoRa_TxDoneFlag`; si es `0` activa `SX1262_LoRa_RxDoneFlag`. Toda comunicación SPI (leer IRQ status, leer/escribir buffer) se realiza únicamente en el main loop, **nunca dentro del ISR**.

## Configuración SPI
Configura tu periférico SPI en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | Full-Duplex Master | STM32 controla el bus |
| **Hardware NSS** | Disable | NSS por software (GPIO) |
| **Data Size** | 8 Bits | Bytes |
| **First Bit** | MSB First | Estándar |
| **Prescaler** | 16-32 (2-4 MHz) | Inicia lento para debug |
| **CPOL** | Low (0) | Modo SPI 0 |
| **CPHA** | 1 Edge | Modo SPI 0 |
| **CRC** | Disabled | No usado |

---

## Instalación
1. Copia `SX1262.c` y `SX1262.h` a tu proyecto (`Librerias/SX1262/`).
2. Incluye la librería en tu `main.c` o archivo principal:
   ```c
   #include "SX1262.h"
   ```
3. Configura SPI y GPIOs en CubeMX (ver secciones anteriores).
4. Genera código y compila.

> [!NOTE]
> Desde la versión 2.0.0 la librería incluye `"main.h"` internamente, por lo que toma automáticamente el encabezado HAL de tu familia STM32 (generado por CubeMX). **Ya no es necesario editar el `#include` del HAL a mano** como en versiones anteriores.

---

## Uso básico

### 1. Inicialización
```c
// En main() después de HAL_Init() y MX_SPIx_Init()
SX1262_Status_t status = SX1262_Init(
    &hspi1,                     // Handle SPI
    GPIOA, GPIO_PIN_4,          // NSS
    GPIOB, GPIO_PIN_0,          // BUSY
    GPIOB, GPIO_PIN_1,          // DIO1
    GPIOB, GPIO_PIN_12          // RST
);

if (status != SX1262_OK) {
    Error_Handler();  // Chip no responde
}
```

La inicialización aplica automáticamente la configuración por defecto:

| Parámetro | Valor por defecto |
|-----------|-------------------|
| Frecuencia | 915 MHz |
| Spreading Factor | SF7 |
| Bandwidth | 125 kHz |
| Coding Rate | CR 4/5 |
| Potencia TX | +22 dBm |
| Longitud de preámbulo | 12 |
| IQ Invertido | No |
| Red | Privada (SyncWord 0x12) |

### 2. Transmisión simple (bloqueante)
```c
uint8_t mensaje[] = "Hola LoRa!";
uint8_t len = sizeof(mensaje) - 1;

SX1262_Status_t status = SX1262_LoRa_Transmit(mensaje, len);
if (status == SX1262_OK) {
    // TX completado — TxDone confirmado por hardware
} else if (status == SX1262_TIMEOUT) {
    // No se recibió TxDone antes del timeout de software (ToA * 1.5 + 100 ms)
} else if (status == SX1262_ERROR) {
    // Error SPI o DIO1 subió sin TX_DONE válido
}
```

### 3. Transmisión no bloqueante (basada en interrupciones)

> [!IMPORTANT]
> Requiere configurar DIO1 como EXTI con flanco de subida en CubeMX. Ver sección [Configuración EXTI para DIO1](#configuración-exti-para-dio1-modos-no-bloqueantes).

```c
// ---------------------------------------------------------------
// En USER CODE BEGIN 4  (main.c)
// ---------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DIO_Pin)
    {
        SX1262_IRQ_Handler();  // Dispatcher TX/RX — sin SPI
    }
}

// ---------------------------------------------------------------
// Variables de estado (antes del while)
// ---------------------------------------------------------------
#define TX_INTERVAL_MS    5000UL  // Intervalo entre transmisiones
#define TX_SW_TIMEOUT_MS  3000UL  // Timeout de software

uint32_t last_tx_tick  = HAL_GetTick() - TX_INTERVAL_MS;
uint32_t tx_start_tick = 0;
bool     tx_busy       = false;
uint8_t  tx_buf[64];
uint8_t  tx_len;

// ---------------------------------------------------------------
// En while(1)
// ---------------------------------------------------------------

/* BLOQUE 1 — Disparar TX cuando el intervalo se cumple */
if (!tx_busy && (HAL_GetTick() - last_tx_tick) >= TX_INTERVAL_MS)
{
    tx_len = snprintf((char*)tx_buf, sizeof(tx_buf), "Hola LoRa!");
    if (SX1262_LoRa_StartTransmitIT(tx_buf, tx_len) == SX1262_OK)
    {
        tx_busy       = true;
        tx_start_tick = HAL_GetTick();
    }
}

/* BLOQUE 2 — Procesar TX_DONE activado por el ISR */
if (SX1262_LoRa_TxDoneFlag)
{
    SX1262_LoRa_TxDoneFlag = 0;  // Consumir bandera — SIEMPRE primero
    SX1262_Status_t result = SX1262_LoRa_GetTransmitStatus();

    if (result == SX1262_OK)          { /* TX_DONE confirmado        */ }
    else if (result == SX1262_TIMEOUT) { /* Timeout interno del chip  */ }
    else                               { /* Condición inesperada       */ }

    tx_busy      = false;
    last_tx_tick = HAL_GetTick();
}

/* BLOQUE 3 (opcional) — Timeout de software */
if (tx_busy && (HAL_GetTick() - tx_start_tick) >= TX_SW_TIMEOUT_MS)
{
    SX1262_LoRa_AbortTransmit();  // Standby RC + Clear IRQ + TxActive = 0
    tx_busy      = false;
    last_tx_tick = HAL_GetTick();
}

// [Otras tareas del sistema aquí — el CPU no se bloquea]
```

> [!NOTE]
> Ver `mainTransmitIT.c` para un ejemplo completo con máquina de estados, timeout de software configurable, LEDs de confirmación y patrón ping-pong (TX seguido de RX IT).

### 4. Recepción bloqueante
```c
uint8_t rx_buffer[256];
uint8_t rx_len = 0;

// Esperar hasta 5 segundos por un paquete
SX1262_Status_t status = SX1262_LoRa_Receive(rx_buffer, sizeof(rx_buffer), &rx_len, 5000);

if (status == SX1262_OK) {
    // rx_buffer[0..rx_len-1] contiene los datos recibidos
} else if (status == SX1262_TIMEOUT) {
    // No se recibió ningún paquete en el tiempo indicado
} else if (status == SX1262_RX_BUFFER_TOO_SMALL) {
    // El paquete no cabe en rx_buffer — rx_len trae el tamaño real requerido
} else if (status == SX1262_ERROR) {
    // Error CRC u otro error de recepción
}

// Recepción continua bloqueante indefinida (timeout_ms = 0)
status = SX1262_LoRa_Receive(rx_buffer, sizeof(rx_buffer), &rx_len, 0);
```

### 5. Recepción no bloqueante (basada en interrupciones)

> [!IMPORTANT]
> Requiere configurar DIO1 (PXx) como EXTI con flanco de subida en CubeMX. Ver sección [Configuración EXTI para DIO1](#configuración-exti-para-dio1-modos-no-bloqueantes).

```c
// ---------------------------------------------------------------
// En USER CODE BEGIN 4  (main.c)
// ---------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DIO_Pin)
    {
        SX1262_IRQ_Handler();  // Solo activa SX1262_LoRa_RxDoneFlag — sin SPI
    }
}

// ---------------------------------------------------------------
// En USER CODE BEGIN 2  (setup, antes del while)
// ---------------------------------------------------------------
SX1262_LoRa_StartReceiveIT();   // Pone el chip en RX continuo y retorna inmediatamente

// ---------------------------------------------------------------
// En while(1)
// ---------------------------------------------------------------
if (SX1262_LoRa_RxDoneFlag)
{
    SX1262_LoRa_RxDoneFlag = 0;                                                     // Consumir bandera
    status = SX1262_LoRa_GetReceivedPacket(rx_buffer, sizeof(rx_buffer), &rx_len);  // Leer payload

    if (status == SX1262_OK)
    {
        // Procesar rx_buffer[0..rx_len-1]
    }
    else if (status == SX1262_TIMEOUT) { /* timeout interno del chip */ }
    else if (status == SX1262_ERROR)   { /* CRC u otro error */         }

    SX1262_LoRa_StartReceiveIT();  // Re-enganche: volver a escuchar
}

// [Otras tareas del sistema aquí — el CPU no se bloquea]
```

### 6. Configuración personalizada
```c
lora_config_t mi_config = {
    .frequency        = 915000000,          // 915 MHz
    .spreading_factor = 9,                  // SF9
    .bandwidth        = BW_125_KHZ,
    .coding_rate      = CR_4_7,
    .tx_power         = 17,                 // 17 dBm
    .preamble_len     = 8,
    .iq_inverted      = false,
    .network_mode     = LORA_NETWORK_PUBLIC, // LoRaWAN SyncWord 0x34
    .lora_sync_word   = 0,                  // 0 = usar network_mode
    .config_pending   = false
};

SX1262_Status_t status = SX1262_LoRa_ApplyConfig(&mi_config);
```

> [!NOTE]
> `SX1262_LoRa_ApplyConfig()` puede llamarse en cualquier momento después de la inicialización para cambiar los parámetros sin reiniciar el chip. Todas las funciones de transmisión y recepción (`SX1262_LoRa_Transmit`, `SX1262_LoRa_StartTransmitIT`, `SX1262_LoRa_Receive`, `SX1262_LoRa_StartReceiveIT`) siempre trabajan con la última configuración aplicada. Si hay cambios pendientes sin aplicar (`config_pending == true`), retornan `SX1262_ERROR`.

### 7. Ejemplo de Transmisión y Recepción Completa

![Transmisión y Recepción](/Librerias/SX1262/Images/TransmisionRecepcion.png)

### 8. Transmisión y recepción en FSK/GFSK

La API FSK es simétrica a la de LoRa: mismos patrones bloqueante / no bloqueante, misma bandera compartida (`SX1262_LoRa_TxDoneFlag` / `SX1262_LoRa_RxDoneFlag`) y mismo `SX1262_IRQ_Handler()`. Solo cambia el prefijo (`SX1262_FSK_*`) y la estructura de configuración (`fsk_config_t`).

```c
// 1) Aplicar configuración FSK (conmuta el chip de LoRa a GFSK)
fsk_config_t fsk = {
    .frequency    = 915000000,
    .bitrate      = 50000,               // 50 kbps
    .freq_dev     = 25.0f,               // Desviación de frecuencia en kHz
    .shaping      = LORA_FSK_SHAPING_NONE,
    .rx_bandwidth = FSK_RXBW_156_2_KHZ,
    .tx_power     = 22,                  // +22 dBm
    .preamble_len = 16,
    .fsk_sync_word = { 0x12, 0xAD },
    .sync_word_len = 2,
    .fixed_length = false,               // Longitud variable
    .payload_len  = 0,                   // Ignorado en longitud variable
    .crc_type     = FSK_CRC_2_BYTE,
    .whitening    = true,
    .config_pending = false
};
SX1262_FSK_ApplyConfig(&fsk);

// 2) Transmisión bloqueante
uint8_t msg[] = "Hola FSK!";
SX1262_FSK_Transmit(msg, sizeof(msg) - 1);

// 3) Recepción bloqueante
uint8_t rx_buffer[256];
uint8_t rx_len = 0;
SX1262_Status_t status = SX1262_FSK_Receive(rx_buffer, sizeof(rx_buffer), &rx_len, 5000);
```

> [!IMPORTANT]
> LoRa y FSK son **modos mutuamente excluyentes** en el chip. Tras `SX1262_FSK_ApplyConfig()` el módulo queda en GFSK; para volver a LoRa hay que llamar de nuevo a `SX1262_LoRa_ApplyConfig()`. No mezcles funciones `SX1262_LoRa_*` y `SX1262_FSK_*` sin reconfigurar la modulación.

### 9. Listen-before-talk con CAD

`SX1262_LoRa_ChannelActivityDetection()` ejecuta un CAD bloqueante para comprobar si el canal está ocupado antes de transmitir, evitando colisiones y ahorrando energía frente a armar una RX completa.

```c
bool canal_ocupado;
if (SX1262_LoRa_ChannelActivityDetection(&canal_ocupado) == SX1262_OK && !canal_ocupado)
{
    SX1262_LoRa_Transmit(mensaje, len);   // Canal libre → transmitir
}
```

> [!NOTE]
> El CAD fuerza al chip a **Standby RC**, cancelando cualquier RX continuo armado con `SX1262_LoRa_StartReceiveIT()`. Si estabas escuchando, vuelve a llamar a `StartReceiveIT()` después del CAD. El CAD solo existe en modo LoRa (no en GFSK).

---

## API Reference

### 1. Tipos de Datos

#### `SX1262_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería, permitiendo una gestión robusta de errores y estados del chip.

```c
/**
 * @brief Enumeración para estados de retorno del SX1262.
 */
typedef enum {
    SX1262_OK                  = 0, /**< Operación exitosa */
    SX1262_ERROR               = 1, /**< Error en la operación */
    SX1262_TIMEOUT             = 2, /**< Timeout en la operación */
    SX1262_NOT_INITIALIZED     = 3, /**< Módulo no inicializado */
    SX1262_INVALID_PARAM       = 4, /**< Parámetro inválido (NULL, longitud 0, etc.) */
    SX1262_RX_BUSY             = 5, /**< Módulo en modo RX IT — ya hay recepción activa */
    SX1262_TX_BUSY             = 6, /**< Módulo en modo TX IT — ya hay transmisión activa */
    SX1262_RX_NO_PACKET        = 7, /**< DIO1 subió sin paquete completo (flanco espurio / falso sync). Evento benigno */
    SX1262_RX_BUFFER_TOO_SMALL = 8  /**< El paquete no cabe en el buffer del llamante (*length trae el tamaño real) */
} SX1262_Status_t;
```

| Valor | Código | Significado |
|-------|--------|-------------|
| `SX1262_OK` | 0 | Operación completada sin errores |
| `SX1262_ERROR` | 1 | Error de SPI o condición inesperada de IRQ |
| `SX1262_TIMEOUT` | 2 | Timeout de software o timeout interno del chip |
| `SX1262_NOT_INITIALIZED` | 3 | `SX1262_Init()` no se llamó o falló |
| `SX1262_INVALID_PARAM` | 4 | Puntero NULL, longitud 0 o `max_length` 0 |
| `SX1262_RX_BUSY` | 5 | Se intentó iniciar RX IT cuando ya había una en curso |
| `SX1262_TX_BUSY` | 6 | Se intentó iniciar TX IT cuando ya había una en curso |
| `SX1262_RX_NO_PACKET` | 7 | DIO1 subió sin paquete completo (flanco espurio o falso sync en el ruido). El chip sigue en RX continuo; no hay que rearmar |
| `SX1262_RX_BUFFER_TOO_SMALL` | 8 | El paquete recibido no cabe en el buffer del llamante. El buffer no se modifica; `*length` devuelve el tamaño real del paquete |

> [!WARNING]
> **Cambio incompatible en 2.0.0:** los valores numéricos de `SX1262_RX_BUSY` y `SX1262_TX_BUSY` cambiaron (4→5 y 5→6) al insertar `SX1262_INVALID_PARAM`. Si comparabas contra el valor entero en lugar del nombre del enum, revisa tu código.

#### `lora_config_t` - Configuración de Parámetros LoRa

Estructura que encapsula todos los parámetros configurables para la comunicación LoRa. Se pasa a `SX1262_LoRa_ApplyConfig()` para aplicar cambios al chip.

```c
typedef struct {
    uint32_t frequency;                 // Hz (default: 915000000)
    uint8_t spreading_factor;           // 5 to 12 (default: 7)
    lora_signal_bandwidth_t bandwidth;  // BW_125_KHZ, BW_250_KHZ, BW_500_KHZ...
    lora_coding_rate_t coding_rate;     // CR_4_5, CR_4_6, CR_4_7, CR_4_8 (default: CR_4_5)
    int8_t tx_power;                    // -9 to 22 dBm (default: 22)
    uint16_t preamble_len;              // Default: 12
    bool iq_inverted;                   // IQ inversion (default: false/normal)
    lora_network_mode_t network_mode;   // Sync word: LORA_NETWORK_PRIVATE / PUBLIC / MESHTASTIC
    uint8_t lora_sync_word;             // Custom sync word (distinto de 0 tiene prioridad sobre network_mode)
    bool config_pending;                // true if changes not yet applied
} lora_config_t;
```

> [!NOTE]
> Desde 2.0.0 el campo `coding_rate` es de tipo `lora_coding_rate_t` (antes `uint8_t`). El código que asigna las macros `CR_4_5`…`CR_4_8` no requiere cambios.

**Prioridad del Sync Word:** si `lora_sync_word != 0`, tiene prioridad absoluta sobre `network_mode`.

> [!IMPORTANT]
> Si `config_pending == true`, las funciones `SX1262_LoRa_Transmit()`, `SX1262_LoRa_StartTransmitIT()`, `SX1262_LoRa_Receive()` y `SX1262_LoRa_StartReceiveIT()` retornarán `SX1262_ERROR`. Debes llamar a `SX1262_LoRa_ApplyConfig()` antes de operar.

#### `lora_network_mode_t` - Modos de Red LoRa

Enumeración para seleccionar el modo de red LoRa, que determina el Sync Word utilizado para la comunicación, permitiendo compatibilidad con redes privadas, públicas (LoRaWAN) o Meshtastic.

```c
typedef enum {
    LORA_NETWORK_PRIVATE    = 0,  /**< Sync Word 0x12 — red privada (por defecto LoRa) */
    LORA_NETWORK_PUBLIC     = 1,  /**< Sync Word 0x34 — red pública (LoRaWAN)          */
    LORA_NETWORK_MESHTASTIC = 2,  /**< Sync Word 0x2B — compatible con Meshtastic       */
} lora_network_mode_t;
```

| Modo | Reg 0x0740 | Reg 0x0741 | Sync Word lógico |
|------|------------|------------|------------------|
| `LORA_NETWORK_PRIVATE`    | 0x14 | 0x24 | 0x12 |
| `LORA_NETWORK_PUBLIC`     | 0x34 | 0x44 | 0x34 |
| `LORA_NETWORK_MESHTASTIC` | 0x2B | 0xB4 | 0x2B |

#### `lora_signal_bandwidth_t` - Ancho de Banda LoRa

Enumeración para los valores de ancho de banda (Bandwidth) en la modulación LoRa, permitiendo seleccionar entre las opciones estándar de 7.8 kHz a 500 kHz.

```c
/**
 * @brief Enumeración para valores de ancho de banda (Bandwidth) en modulación LoRa.
 */
typedef enum {
	BW_7_8_KHZ    = 0x00,
	BW_10_4_KHZ   = 0x08,
	BW_15_6_KHZ   = 0x01,
	BW_20_8_KHZ   = 0x09,
	BW_31_25_KHZ  = 0x02,
	BW_41_7_KHZ   = 0x0A,
	BW_62_5_KHZ   = 0x03,
	BW_125_KHZ    = 0x04,
	BW_250_KHZ    = 0x05,
	BW_500_KHZ    = 0x06
} lora_signal_bandwidth_t;

```

#### `lora_coding_rate_t` - Coding Rate LoRa

Enumeración para los valores de coding rate en la modulación LoRa, permitiendo seleccionar entre las opciones estándar de 4/5 a 4/8.

```c
/**
 * @brief Enumeración para valores de coding rate (CR) en modulación LoRa.
 */
typedef enum {
    CR_4_5 = 0x01,
    CR_4_6 = 0x02,
    CR_4_7 = 0x03,
    CR_4_8 = 0x04
} lora_coding_rate_t;
```

#### `fsk_config_t` - Configuración de Parámetros FSK/GFSK

Estructura que encapsula todos los parámetros configurables para la comunicación FSK/GFSK. Se pasa a `SX1262_FSK_ApplyConfig()`, que además conmuta el chip de LoRa a modo GFSK.

```c
typedef struct {
    uint32_t frequency;              // Hz (default: 915000000)
    uint32_t bitrate;                // Bit rate en bps (default: 50000)
    float freq_dev;                  // Desviación de frecuencia en kHz (default: 25.0)
    fsk_shaping_t shaping;           // Shaping gaussiano BT (default: LORA_FSK_SHAPING_NONE)
    fsk_rx_bandwidth_t rx_bandwidth; // RX bandwidth (default: FSK_RXBW_156_2_KHZ)
    int8_t tx_power;                 // -9 to 22 dBm (default: 22)
    uint16_t preamble_len;           // Longitud de preámbulo en bytes (default: 16)
    uint8_t fsk_sync_word[8];        // Sync word (hasta 8 bytes; default: {0x12, 0xAD})
    uint8_t sync_word_len;           // Longitud del sync word (default: 2)
    bool fixed_length;               // Longitud fija vs variable (default: false)
    uint8_t payload_len;             // Longitud de payload en modo fijo (ignorado en variable)
    fsk_crc_type_t crc_type;         // Tipo de CRC (default: FSK_CRC_2_BYTE)
    bool whitening;                  // Habilitar whitening (default: true)
    bool config_pending;             // true si hay cambios sin aplicar
} fsk_config_t;
```

#### `fsk_shaping_t` - Shaping Gaussiano FSK

```c
typedef enum {
    LORA_FSK_SHAPING_NONE   = 0x00,
    LORA_FSK_SHAPING_BT_0_3 = 0x08,
    LORA_FSK_SHAPING_BT_0_5 = 0x09,
    LORA_FSK_SHAPING_BT_0_7 = 0x0A,
    LORA_FSK_SHAPING_BT_1_0 = 0x0B
} fsk_shaping_t;
```

#### `fsk_rx_bandwidth_t` - Ancho de Banda de Recepción FSK

El chip solo admite este conjunto discreto de valores (datasheet SX1262 §13.4.9). Van desde `FSK_RXBW_4_8_KHZ` hasta `FSK_RXBW_467_0_KHZ`.

```c
typedef enum {
    FSK_RXBW_4_8_KHZ   = 0x1F,
    FSK_RXBW_5_8_KHZ   = 0x17,
    /* ... valores intermedios ... */
    FSK_RXBW_156_2_KHZ = 0x1A,   // Por defecto
    FSK_RXBW_187_2_KHZ = 0x12,
    FSK_RXBW_234_3_KHZ = 0x0A,
    FSK_RXBW_312_0_KHZ = 0x19,
    FSK_RXBW_373_6_KHZ = 0x11,
    FSK_RXBW_467_0_KHZ = 0x09
} fsk_rx_bandwidth_t;
```

#### `fsk_crc_type_t` - Tipo de CRC FSK

```c
typedef enum {
    FSK_CRC_OFF        = 0x01,
    FSK_CRC_1_BYTE     = 0x00,
    FSK_CRC_2_BYTE     = 0x02,
    FSK_CRC_1_BYTE_INV = 0x04,
    FSK_CRC_2_BYTE_INV = 0x06
} fsk_crc_type_t;
```

---

#### Banderas `volatile` de Evento

Las banderas de evento implementan el patrón **productor-consumidor** entre el ISR (productor) y el main loop (consumidor). Están declaradas `volatile` para que el compilador no las optimice fuera de la RAM.

```c
extern volatile uint8_t SX1262_LoRa_TxDoneFlag;  // 1 cuando TX IT completó (TxDone o Timeout)
extern volatile uint8_t SX1262_LoRa_RxDoneFlag;  // 1 cuando RX IT completó (RxDone, Timeout o error)
```

| Bandera | Activada por | Consumida por | Función de consumo |
|---------|-------------|--------------|--------------------|
| `SX1262_LoRa_TxDoneFlag` | `SX1262_IRQ_Handler()` cuando `TxActive == 1` | Main loop | `SX1262_LoRa_GetTransmitStatus()` |
| `SX1262_LoRa_RxDoneFlag` | `SX1262_IRQ_Handler()` cuando `TxActive == 0` | Main loop | `SX1262_LoRa_GetReceivedPacket()` |

> [!WARNING]
> Siempre poner la bandera a `0` **antes** de llamar a la función de consumo, para evitar perder un nuevo evento que pudiera ocurrir durante el procesamiento SPI.

> [!NOTE]
> Ambas banderas son **compartidas por LoRa y FSK**: una operación IT iniciada con `SX1262_FSK_StartTransmitIT()` / `SX1262_FSK_StartReceiveIT()` señaliza en las mismas banderas y se consume con `SX1262_FSK_GetTransmitStatus()` / `SX1262_FSK_GetReceivedPacket()`. Conservan el prefijo `LoRa_` por compatibilidad con la API existente.

---

### 2. Funciones Públicas

#### `SX1262_Init()` - Inicialización del Driver

Inicializa el driver, realiza un reset hardware, despierta el chip y aplica la configuración por defecto. Todas las demás funciones retornan `SX1262_NOT_INITIALIZED` si esta función no se ha llamado exitosamente.

```c
SX1262_Status_t SX1262_Init(
	SPI_HandleTypeDef* hspi,
    GPIO_TypeDef*      nss_port,
    uint16_t           nss_pin,
    GPIO_TypeDef*      busy_port,
    uint16_t           busy_pin,
    GPIO_TypeDef*      dio_port,
    uint16_t           dio_pin,
    GPIO_TypeDef*      rst_port,
    uint16_t           rst_pin
);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `hspi` | `SPI_HandleTypeDef*` | Handle del periférico SPI configurado |
| `nss_port` | `GPIO_TypeDef*` | Puerto GPIO del pin NSS (Chip Select) |
| `nss_pin` | `uint16_t` | Pin GPIO para NSS |
| `busy_port` | `GPIO_TypeDef*` | Puerto GPIO del pin BUSY |
| `busy_pin` | `uint16_t` | Pin GPIO para BUSY |
| `dio_port` | `GPIO_TypeDef*` | Puerto GPIO del pin DIO1 (interrupción) |
| `dio_pin` | `uint16_t` | Pin GPIO para DIO1 |
| `rst_port` | `GPIO_TypeDef*` | Puerto GPIO del pin RST |
| `rst_pin` | `uint16_t` | Pin GPIO para RST |

**Retorna:** `SX1262_OK` si la inicialización fue exitosa, `SX1262_ERROR` si algún parámetro es NULL o la comunicación SPI falla.

---

#### `SX1262_LoRa_Transmit()` - Transmisión Bloqueante

Transmite un buffer de datos por LoRa. La función es **bloqueante**: hace polling en DIO1 hasta que el chip sube `TxDone` o expira el timeout de software (calculado dinámicamente como `ToA × 1.5 + 100 ms`).

```c
SX1262_Status_t SX1262_LoRa_Transmit(uint8_t* data, uint8_t length);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `data` | `uint8_t*` | Puntero al buffer de datos a transmitir |
| `length` | `uint8_t` | Longitud de los datos (máximo 255 bytes) |

**Retorna:** `SX1262_OK` si TX_DONE fue confirmado, `SX1262_TIMEOUT` si el timeout de software o el chip reportan timeout, `SX1262_ERROR` en caso de fallo SPI o IRQ inesperado, `SX1262_NOT_INITIALIZED` si el módulo no fue inicializado.

**Secuencia interna:**
1. Standby RC → configura base address del buffer (TX=0x00, RX=0x00).
2. Escribe el payload al buffer interno del chip.
3. Actualiza `SET_PACKET_PARAMS` con la longitud real del payload.
4. Limpia IRQ, habilita `TxDone | Timeout` en DIO1.
5. Inicia TX con `SET_TX` (timeout de chip = 0, desactivado).
6. Polling en DIO1 hasta evento o timeout de software.
7. Lee y limpia el registro IRQ; evalúa `TX_DONE` vs `TIMEOUT`.

---

#### `SX1262_LoRa_StartTransmitIT()` - Iniciar Transmisión No Bloqueante

Inicia la transmisión de un paquete y **retorna inmediatamente**. El CPU queda libre para ejecutar otras tareas. El evento de finalización se señaliza mediante `SX1262_LoRa_TxDoneFlag`, activada por el ISR cuando DIO1 genera el flanco de subida.

```c
SX1262_Status_t SX1262_LoRa_StartTransmitIT(uint8_t* data, uint8_t length);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `data` | `uint8_t*` | Puntero al buffer de datos a transmitir |
| `length` | `uint8_t` | Longitud de los datos (máximo 255 bytes) |

**Retorna:** `SX1262_OK` si el chip entró en modo TX, `SX1262_TX_BUSY` si ya hay una transmisión IT en curso, `SX1262_ERROR` si falla SPI o `data` es NULL, `SX1262_NOT_INITIALIZED` si el módulo no fue inicializado.

**Secuencia interna:**
1. Standby RC.
2. `SetBufferBaseAddress` (TX=0x00, RX=0x00).
3. `WriteBuffer` — escribe el payload en el chip.
4. `SetPacketParams` — actualiza la longitud real del payload.
5. `ClearIRQ` — limpia IRQs pendientes.
6. `SetDioIrqParams` — enruta `TX_DONE | TIMEOUT` a DIO1.
7. `SX1262_TxActive = 1` — arma el semáforo (antes de `SetTx` para evitar pérdida de IRQ inmediata).
8. `SetTx` con timeout de chip = `0x000000` (sin timeout de chip; el timeout lo gestiona el main loop).
9. Retorno inmediato — sin polling.

> [!WARNING]
> No llamar si `SX1262_LoRa_TxDoneFlag` aún no ha sido consumida. Usar el estado de la máquina de estados del main loop para garantizar que la TX anterior ha terminado.

---

#### `SX1262_LoRa_GetTransmitStatus()` - Confirmar Resultado de TX

Verifica y consume el evento `TX_DONE` después de que `SX1262_LoRa_TxDoneFlag == 1`. Lee el registro IRQ del chip para distinguir entre `TX_DONE` real y `TIMEOUT` interno, limpia el registro y libera el semáforo `SX1262_TxActive`.

```c
SX1262_Status_t SX1262_LoRa_GetTransmitStatus(void);
```

**Retorna:** `SX1262_OK` si `TX_DONE` fue confirmado, `SX1262_TIMEOUT` si el chip reporta timeout interno, `SX1262_ERROR` si la condición de IRQ es inesperada o falla SPI, `SX1262_NOT_INITIALIZED` si el módulo no fue inicializado.

> [!WARNING]
> Llamar **solo desde el main loop** cuando `SX1262_LoRa_TxDoneFlag == 1`. **Nunca desde el ISR** — realiza comunicación SPI.

**Secuencia interna:**
1. Lee `GetIrqStatus`.
2. Limpia el registro IRQ (`ClearIRQ`) — siempre, independientemente del resultado.
3. Libera `SX1262_TxActive = 0`.
4. Evalúa `TIMEOUT` (prioridad) y luego `TX_DONE`.

---

#### `SX1262_LoRa_AbortTransmit()` - Cancelar Transmisión

Cancela la transmisión IT en curso, regresa el chip a **Standby RC** y libera el semáforo `SX1262_TxActive`. Útil para implementar timeouts de software sin bloquear el CPU.

```c
SX1262_Status_t SX1262_LoRa_AbortTransmit(void);
```

**Retorna:** `SX1262_OK` si el chip volvió a Standby, `SX1262_ERROR` si falla la escritura SPI.

**Secuencia interna:**
1. `SetStandby RC` — detiene la transmisión activa.
2. `ClearIRQ` — limpia IRQs residuales.
3. `SX1262_TxActive = 0` — libera el semáforo para nuevas operaciones.

---

#### `SX1262_LoRa_Receive()` - Recepción Bloqueante

Pone el chip en modo recepción y espera un paquete válido. **Bloqueante** con timeout configurable.

```c
SX1262_Status_t SX1262_LoRa_Receive(uint8_t* data, uint8_t max_length, uint8_t* length, uint32_t timeout_ms);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `data` | `uint8_t*` | Buffer donde se almacenarán los datos recibidos |
| `max_length` | `uint8_t` | Capacidad del buffer `data` en bytes. Si el paquete es mayor, no se escribe nada y se retorna `SX1262_RX_BUFFER_TOO_SMALL` |
| `length` | `uint8_t*` | Puntero donde se escribirá la longitud del paquete recibido |
| `timeout_ms` | `uint32_t` | Tiempo máximo de espera en ms. `0` = espera indefinida |

**Retorna:** `SX1262_OK` si se recibió un paquete válido, `SX1262_TIMEOUT` si expiró el tiempo, `SX1262_INVALID_PARAM` si `data`/`length` son NULL o `max_length` es 0, `SX1262_RX_BUFFER_TOO_SMALL` si el paquete no cabe en `data` (`*length` trae el tamaño real), `SX1262_ERROR` si hubo error CRC o fallo de IRQ, `SX1262_NOT_INITIALIZED` si el módulo no fue inicializado.

> [!WARNING]
> **Cambio incompatible en 2.0.0:** se añadió el parámetro `max_length` en segunda posición. Actualiza todas las llamadas: `SX1262_LoRa_Receive(buf, &len, 5000)` → `SX1262_LoRa_Receive(buf, sizeof(buf), &len, 5000)`.

**Conversión de timeout:** el timeout interno del chip trabaja en ticks de 15.625 µs. La librería convierte automáticamente: `ticks = timeout_ms * 64`. Si `timeout_ms = 0`, se usa `0xFFFFFF` (modo continuo).

**Secuencia interna:**
1. Standby RC → limpia IRQ.
2. Habilita `RxDone | Timeout | CRC_ERR` en DIO1.
3. Inicia RX con `SET_RX` y el timeout calculado.
4. Polling en DIO1 con un soft-timeout adicional de `timeout_ms + 100 ms`.
5. Lee registro IRQ: verifica `RxDone` y ausencia de `Timeout` y `CRC_ERR`.
6. Lee `GET_RX_BUFFER_STATUS` para obtener tamaño y offset del paquete.
7. Lee el payload del buffer interno con `READ_BUFFER`.
8. Limpia IRQ.

---

#### `SX1262_LoRa_StartReceiveIT()` - Iniciar Recepción No Bloqueante

Pone el chip en modo RX continuo y **retorna inmediatamente**. El evento de recepción se señaliza mediante la bandera `volatile uint8_t SX1262_LoRa_RxDoneFlag`, que es activada desde el ISR. Requiere que DIO1 esté configurado como EXTI en CubeMX.

```c
SX1262_Status_t SX1262_LoRa_StartReceiveIT(void);
```

**Retorna:** `SX1262_OK` si el chip entró en modo RX correctamente, `SX1262_ERROR` si falla alguna escritura SPI, `SX1262_NOT_INITIALIZED` si el módulo no fue inicializado.

**Secuencia interna:**
1. Standby RC.
2. Limpia registro IRQ (ClearIRQ).
3. Habilita `RxDone | Timeout | CRC_ERR | HeaderErr` en DIO1 (SetDioIrqParams).
4. Inicia RX con timeout `0xFFFFFF` (modo continuo) → `SET_RX`.
5. Retorno inmediato — sin polling.

---

#### `SX1262_LoRa_GetReceivedPacket()` - Leer Paquete Recibido

Lee el payload del buffer interno del SX1262. Debe llamarse **solo** cuando `SX1262_LoRa_RxDoneFlag == 1` (desde el main loop, no desde el ISR).

```c
SX1262_Status_t SX1262_LoRa_GetReceivedPacket(uint8_t* data, uint8_t max_length, uint8_t* length);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `data` | `uint8_t*` | Buffer donde se almacenará el payload recibido |
| `max_length` | `uint8_t` | Capacidad del buffer `data` en bytes. El emisor decide el tamaño (hasta 255 bytes), así que este límite es la única protección contra desbordamiento de RAM |
| `length` | `uint8_t*` | Puntero donde se escribirá la longitud del paquete recibido |

**Retorna:** `SX1262_OK` si se leyó un paquete válido, `SX1262_INVALID_PARAM` si `data`/`length` son NULL o `max_length` es 0, `SX1262_TIMEOUT` si el IRQ indica timeout del chip, `SX1262_RX_BUFFER_TOO_SMALL` si el paquete no cabe en `data` (`*length` trae el tamaño real; el buffer no se toca), `SX1262_ERROR` si hay error CRC, header inválido o fallo SPI.

> [!WARNING]
> **Cambio incompatible en 2.0.0:** se añadió el parámetro `max_length` en segunda posición. Actualiza tus llamadas: `SX1262_LoRa_GetReceivedPacket(buf, &len)` → `SX1262_LoRa_GetReceivedPacket(buf, sizeof(buf), &len)`.

**Secuencia interna:**
1. Lee registro IRQ (`GetIrqStatus`).
2. Verifica bits: `RxDone` presente, `Timeout` y `CRC_ERR` ausentes.
3. Lee `GetRxBufferStatus` para obtener tamaño y offset del paquete.
4. Lee payload con `ReadBuffer`.
5. Limpia registro IRQ.

---

#### `SX1262_LoRa_AbortReceive()` - Cancelar Recepción

Cancela la recepción en curso y regresa el chip al modo Standby. Útil para implementar un timeout de software sin bloquear el CPU.

```c
SX1262_Status_t SX1262_LoRa_AbortReceive(void);
```

**Retorna:** `SX1262_OK` si el chip volvió a Standby, `SX1262_ERROR` si falla la escritura SPI.

---

#### `SX1262_LoRa_ChannelActivityDetection()` - Detección de Actividad en el Canal (CAD)

Ejecuta un CAD **bloqueante** y reporta si hay una señal LoRa presente. El chip busca la correlación con un preámbulo LoRa durante 2 símbolos y vuelve a Standby RC. Es mucho más rápido y barato en energía que armar una RX completa, por lo que es la primitiva habitual para *listen-before-talk*: comprobar que el canal está libre antes de transmitir.

```c
SX1262_Status_t SX1262_LoRa_ChannelActivityDetection(bool *activity_detected);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `activity_detected` | `bool*` | Se escribe `true` si se detectó actividad, `false` si el canal está libre. Solo válido si retorna `SX1262_OK` |

**Retorna:** `SX1262_OK` si el CAD se completó, `SX1262_INVALID_PARAM` si `activity_detected` es NULL, `SX1262_NOT_INITIALIZED` si no se inicializó, `SX1262_TX_BUSY` si hay una TX IT en vuelo, `SX1262_TIMEOUT` si DIO1 no subió en el plazo calculado, `SX1262_ERROR` si no se ha llamado a `SX1262_LoRa_ApplyConfig()` o ante fallos de SPI.

**Notas de comportamiento:**
- El tiempo de espera se calcula internamente a partir del SF y el BW activos, así que la llamada nunca se cuelga aunque DIO1 no suba.
- La sensibilidad (`cadDetPeak` / `cadDetMin`) se deriva del SF configurado según la nota de aplicación Semtech AN1200.48.
- El CAD fuerza **Standby RC**, cancelando cualquier RX continuo armado con `SX1262_LoRa_StartReceiveIT()`. Rearma la recepción tras el CAD si quieres seguir escuchando.
- Requiere modo LoRa: el CAD no existe en GFSK.

---

#### `SX1262_IRQ_Handler()` - Manejador de Interrupción

Función **liviana** (dispatcher) que debe llamarse desde `HAL_GPIO_EXTI_Callback()` cuando el pin DIO1 genera una interrupción. Determina sin comunicación SPI si el evento corresponde a una TX o RX activa, usando el semáforo interno `SX1262_TxActive`, y activa la bandera correspondiente.

```c
void SX1262_IRQ_Handler(void);
```

**Lógica de despacho:**

```c
if (SX1262_TxActive)
    SX1262_LoRa_TxDoneFlag = 1;  // Evento TX → consumir con SX1262_LoRa_GetTransmitStatus()
else
    SX1262_LoRa_RxDoneFlag = 1;  // Evento RX → consumir con SX1262_LoRa_GetReceivedPacket()
```

> [!WARNING]
> Esta función está diseñada para ejecutarse en contexto de **interrupción (ISR)**. Regla crítica: **sin SPI, sin HAL calls bloqueantes, sin printf**. Toda comunicación SPI se realiza únicamente en el main loop mediante `SX1262_LoRa_GetTransmitStatus()` o `SX1262_LoRa_GetReceivedPacket()`.

**TX y RX son mutuamente excluyentes** en el chip SX1262, por lo que el semáforo `SX1262_TxActive` es suficiente para un despacho seguro desde el ISR.

---

#### `SX1262_LoRa_ApplyConfig()` - Aplicar Configuración LoRa

Aplica una nueva configuración de modulación y red al chip. Puede llamarse en cualquier momento después de `SX1262_Init()`. No reinicia el chip: solo pone el módulo en Standby RC y reconfigura los registros necesarios.

```c
SX1262_Status_t SX1262_LoRa_ApplyConfig(const lora_config_t *config);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `config` | `const lora_config_t*` | Puntero a la estructura de configuración (no se modifica) |

**Retorna:** `SX1262_OK` si todos los parámetros fueron escritos correctamente, `SX1262_ERROR` si falla alguna escritura SPI o `config` es NULL, `SX1262_NOT_INITIALIZED` si el módulo no fue inicializado.

**Parámetros aplicados en orden:**
1. Frecuencia RF (conversión entera 64-bit sin punto flotante).
2. PA Config + TX Params (potencia y rampa de 40 µs).
3. Modulation Params: SF, BW, CR y LDRO calculado automáticamente.
4. Sync Word (registros 0x0740–0x0741).
5. Packet Params: preámbulo, header explícito, longitud dummy, CRC on, IQ.


#### `SX1262_LoRa_GetConfig()` - Obtener Configuración Actual

Permite obtener una copia de la configuración LoRa actualmente aplicada al chip. La función retorna una estructura `lora_config_t` con los parámetros que fueron aplicados en el último `SX1262_LoRa_ApplyConfig()`. No realiza comunicación SPI, ya que la librería mantiene una copia local de la configuración aplicada. El campo `config_pending` de la copia siempre se establece en `false`, ya que esta función solo refleja la configuración que ya está activa en el chip.

```c
SX1262_Status_t SX1262_LoRa_GetConfig(lora_config_t *config);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `config` | `lora_config_t*` | Puntero a la estructura donde se almacenará la configuración actual |

#### `SX1262_LoRa_GetPacketStatus()` - Obtener RSSI y SNR del último paquete

Obtiene el RSSI y el SNR del último paquete LoRa recibido en **una sola lectura SPI**. Usa el comando `GetPacketStatus` (0x14), que devuelve ambas métricas en la misma respuesta: `RSSI [dBm] = -RssiPkt / 2` y `SNR [dB] = SnrPkt / 4` (datasheet SX1262 §13.5.3). Debe llamarse justo después de una recepción exitosa.

```c
SX1262_Status_t SX1262_LoRa_GetPacketStatus(int16_t *rssi_dbm, int8_t *snr_db);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `rssi_dbm` | `int16_t*` | Puntero donde se almacenará el RSSI en dBm, o NULL para omitirlo |
| `snr_db` | `int8_t*` | Puntero donde se almacenará el SNR en dB (puede ser negativo), o NULL para omitirlo |

**Retorna:** `SX1262_OK` si se obtuvieron las métricas solicitadas, `SX1262_INVALID_PARAM` si **ambos** punteros son NULL, `SX1262_NOT_INITIALIZED` si no se inicializó, `SX1262_ERROR` ante fallos de SPI.

> [!TIP]
> Si necesitas ambas métricas, usa esta función en lugar de llamar a `SX1262_LoRa_GetRSSI()` y `SX1262_LoRa_GetSNR()` por separado: ahorras una transacción SPI.

#### `SX1262_LoRa_GetRSSI()` - Obtener RSSI del último paquete recibido

Obtiene el RSSI (Received Signal Strength Indicator) del último paquete recibido, en dBm. Es un **envoltorio** sobre `SX1262_LoRa_GetPacketStatus()`.

```c
SX1262_Status_t SX1262_LoRa_GetRSSI(int16_t *rssi_dbm);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `rssi_dbm` | `int16_t*` | Puntero donde se almacenará el valor de RSSI en dBm |

**Retorna:** `SX1262_OK`, `SX1262_INVALID_PARAM` si `rssi_dbm` es NULL, `SX1262_NOT_INITIALIZED` o `SX1262_ERROR`.

#### `SX1262_LoRa_GetSNR()` - Obtener SNR del último paquete recibido

Obtiene el SNR (Signal-to-Noise Ratio) del último paquete recibido, en dB. Es un **envoltorio** sobre `SX1262_LoRa_GetPacketStatus()`. El valor puede ser negativo.

```c
SX1262_Status_t SX1262_LoRa_GetSNR(int8_t *snr_db);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `snr_db` | `int8_t*` | Puntero donde se almacenará el valor de SNR en dB |

**Retorna:** `SX1262_OK`, `SX1262_INVALID_PARAM` si `snr_db` es NULL, `SX1262_NOT_INITIALIZED` o `SX1262_ERROR`.

#### `SX1262_GetRSSIInst()` - Obtener RSSI instantáneo del canal

Obtiene el RSSI instantáneo del canal, medido en el momento de la llamada. Usa el comando `GetRssiInst` (0x15): `RSSI [dBm] = -RssiInst / 2` (datasheet §13.5.2). A diferencia de `SX1262_LoRa_GetRSSI()`, **no depende de una recepción previa**: mide la potencia presente en el canal en ese instante, por lo que es la función adecuada para un escáner de espectro o para detectar canal ocupado. Funciona igual en LoRa y en FSK.

```c
SX1262_Status_t SX1262_GetRSSIInst(int16_t *rssi_dbm);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `rssi_dbm` | `int16_t*` | Puntero donde se almacenará el RSSI en dBm |

**Retorna:** `SX1262_OK`, `SX1262_INVALID_PARAM` si `rssi_dbm` es NULL, `SX1262_NOT_INITIALIZED` o `SX1262_ERROR`.

> [!IMPORTANT]
> El comando solo es válido con el chip en **modo RX**. Debe llamarse tras iniciar una recepción (`SX1262_LoRa_StartReceiveIT()` o `SX1262_FSK_StartReceiveIT()`); en STDBY o SLEEP la lectura no es significativa.

---

### 3. Funciones FSK/GFSK

La API FSK es simétrica a la LoRa. Solo se documentan aquí las diferencias; el comportamiento de banderas, ISR y timeouts es idéntico al descrito para LoRa.

> [!IMPORTANT]
> Todas las funciones `SX1262_FSK_*` requieren haber llamado antes a `SX1262_FSK_ApplyConfig()`, que conmuta el chip a modo GFSK. LoRa y FSK son mutuamente excluyentes.

| Función | Firma | Equivalente LoRa |
|---------|-------|------------------|
| Transmisión bloqueante | `SX1262_FSK_Transmit(uint8_t* data, uint8_t length)` | `SX1262_LoRa_Transmit` |
| Iniciar TX no bloqueante | `SX1262_FSK_StartTransmitIT(uint8_t* data, uint8_t length)` | `SX1262_LoRa_StartTransmitIT` |
| Confirmar resultado TX | `SX1262_FSK_GetTransmitStatus(void)` | `SX1262_LoRa_GetTransmitStatus` |
| Cancelar TX | `SX1262_FSK_AbortTransmit(void)` | `SX1262_LoRa_AbortTransmit` |
| Recepción bloqueante | `SX1262_FSK_Receive(uint8_t* data, uint8_t max_length, uint8_t* length, uint32_t timeout_ms)` | `SX1262_LoRa_Receive` |
| Iniciar RX no bloqueante | `SX1262_FSK_StartReceiveIT(void)` | `SX1262_LoRa_StartReceiveIT` |
| Leer paquete recibido | `SX1262_FSK_GetReceivedPacket(uint8_t* data, uint8_t max_length, uint8_t* length)` | `SX1262_LoRa_GetReceivedPacket` |
| Cancelar RX | `SX1262_FSK_AbortReceive(void)` | `SX1262_LoRa_AbortReceive` |
| Aplicar configuración | `SX1262_FSK_ApplyConfig(fsk_config_t *config)` | `SX1262_LoRa_ApplyConfig` |
| Obtener configuración | `SX1262_FSK_GetConfig(fsk_config_t *config)` | `SX1262_LoRa_GetConfig` |

Las funciones FSK comparten las mismas banderas (`SX1262_LoRa_TxDoneFlag` / `SX1262_LoRa_RxDoneFlag`), el mismo `SX1262_IRQ_Handler()` y los mismos códigos de retorno (`SX1262_Status_t`) que su contraparte LoRa. La recepción FSK también protege contra desbordamiento vía `max_length` y retorna `SX1262_RX_BUFFER_TOO_SMALL` cuando corresponde.

> [!NOTE]
> `SX1262_FSK_GetConfig()` devuelve los valores por defecto cacheados durante `SX1262_Init()` si aún no se ha aplicado ninguna configuración FSK.

---

### 4. Funciones de Diagnóstico

#### `SX1262_GetLastIrqStatus()` - Último registro IRQ de RX

Retorna el último registro IRQ leído al consumir un evento de recepción. **No realiza comunicación SPI**: devuelve el valor cacheado durante la última llamada a `SX1262_LoRa_GetReceivedPacket()` / `SX1262_FSK_GetReceivedPacket()`. Pensado para diagnosticar por qué una recepción no devolvió `SX1262_OK`: los bits se interpretan con las macros `SX126X_IRQ_*` (`RX_DONE`, `CRC_ERR`, `TIMEOUT`, etc.).

```c
uint16_t SX1262_GetLastIrqStatus(void);
```

**Retorna:** El registro IRQ (bits `SX126X_IRQ_*`), o `0` si aún no hubo ninguna RX.

---

### 5. Funciones de Gestión de Energía

#### `SX1262_SetSleep()` - Entrar en Modo Reposo

Pone el módulo SX1262 en modo Sleep para consumo energético mínimo. En este modo el chip detiene la mayoría de sus funciones y el consumo cae a niveles de microamperios.

```c
SX1262_Status_t SX1262_SetSleep(uint8_t sleep_config);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `sleep_config` | `uint8_t` | Configuración del modo sleep (bit 0: Warm/Cold, bit 2: RTC wakeup) |

**Valores recomendados:**
- `SX126X_SLEEP_START_WARM` (0x00): Mantiene la configuración en la memoria de retención. Es el modo recomendado para poder operar rápidamente tras despertar.
- `SX126X_SLEEP_START_COLD` (0x01): El chip se apaga completamente. Al despertar se debe re-inicializar o aplicar la configuración nuevamente.

#### `SX1262_Wakeup()` - Despertar del Modo Reposo

Despierta el chip desde el modo Sleep realizando una secuencia de flancos en la línea NSS. Esta función debe llamarse antes de intentar cualquier comunicación SPI tras un `SetSleep`.

```c
SX1262_Status_t SX1262_Wakeup(void);
```

**Retorna:** `SX1262_OK` si el chip despertó y liberó el pin BUSY correctamente, `SX1262_TIMEOUT` si el chip no responde.

---

## Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

### [2.0.0] - 21-07-2026

Versión mayor: añade la modulación FSK/GFSK completa, CAD y telemetría instantánea, e introduce **cambios incompatibles** en varias firmas y en el enum de estados.

#### Added
- **Modulación FSK/GFSK completa**, con API simétrica a la de LoRa:
  - Transmisión: `SX1262_FSK_Transmit()`, `SX1262_FSK_StartTransmitIT()`, `SX1262_FSK_GetTransmitStatus()`, `SX1262_FSK_AbortTransmit()`.
  - Recepción: `SX1262_FSK_Receive()`, `SX1262_FSK_StartReceiveIT()`, `SX1262_FSK_GetReceivedPacket()`, `SX1262_FSK_AbortReceive()`.
  - Configuración: `SX1262_FSK_ApplyConfig()` (conmuta el chip a GFSK) y `SX1262_FSK_GetConfig()`.
  - Nuevas estructuras y enumeraciones: `fsk_config_t`, `fsk_shaping_t`, `fsk_rx_bandwidth_t`, `fsk_crc_type_t`.
  - Parámetros configurables: bit rate, desviación de frecuencia, RX bandwidth, shaping gaussiano (BT), sync word de hasta 8 bytes, tipo de CRC, whitening y longitud fija/variable.
- **Detección de actividad en el canal (CAD)** para LoRa: `SX1262_LoRa_ChannelActivityDetection()`, primitiva *listen-before-talk* con sensibilidad derivada del SF (Semtech AN1200.48).
- **Telemetría de señal ampliada**:
  - `SX1262_LoRa_GetPacketStatus()`: RSSI y SNR del último paquete en una sola lectura SPI. `SX1262_LoRa_GetRSSI()` y `SX1262_LoRa_GetSNR()` pasan a ser envoltorios sobre esta función.
  - `SX1262_GetRSSIInst()`: RSSI instantáneo del canal (comando `GetRssiInst` 0x15), independiente de una recepción previa. Útil para escáner de espectro o detección de canal ocupado. Válido en LoRa y FSK.
- **Diagnóstico de recepción**: `SX1262_GetLastIrqStatus()` expone el último registro IRQ leído para diagnosticar por qué una RX no devolvió `SX1262_OK`.
- **Protección contra desbordamiento en recepción**: nuevo parámetro `max_length` en las funciones de RX y nuevo código `SX1262_RX_BUFFER_TOO_SMALL`, que preserva el buffer y devuelve el tamaño real del paquete.
- Nuevos códigos de retorno: `SX1262_INVALID_PARAM`, `SX1262_RX_NO_PACKET` y `SX1262_RX_BUFFER_TOO_SMALL`, con validación de parámetros en toda la API pública.
- Presets Meshtastic listos para usar como `const lora_config_t`: `LongSlow`, `LongFast`, `MediumSlow`, `MediumFast`, `ShortSlow`, `ShortFast`.

#### Changed
- **[BREAKING]** `SX1262_LoRa_Receive()` añade el parámetro `max_length`:
  `SX1262_LoRa_Receive(data, length, timeout_ms)` → `SX1262_LoRa_Receive(data, max_length, length, timeout_ms)`.
- **[BREAKING]** `SX1262_LoRa_GetReceivedPacket()` añade el parámetro `max_length`:
  `SX1262_LoRa_GetReceivedPacket(data, length)` → `SX1262_LoRa_GetReceivedPacket(data, max_length, length)`.
- **[BREAKING]** El enum `SX1262_Status_t` se renumeró al insertar `SX1262_INVALID_PARAM = 4`: `SX1262_RX_BUSY` pasó de 4 a 5 y `SX1262_TX_BUSY` de 5 a 6. El código que compare contra el nombre del enum no requiere cambios; el que compare contra el valor entero sí.
- **[BREAKING]** El campo `coding_rate` de `lora_config_t` cambió de `uint8_t` a `lora_coding_rate_t`.
- `SX1262_LoRa_ApplyConfig()` ahora recibe `const lora_config_t *` (antes `lora_config_t *`).
- **Portabilidad sin edición manual**: la librería incluye `"main.h"` internamente y toma automáticamente el encabezado HAL de la familia STM32 generado por CubeMX. Ya no es necesario editar el `#include` del HAL a mano.
- Presets Meshtastic pasan a ser `const` (const-correctness).
- Refactor interno: lógica de TX/RX y de aborto unificada entre LoRa y FSK; conversión de bandwidth y comandos comunes extraídos.

#### Fixed
- Configuración de `PayloadLength` en FSK de longitud variable.

---

### [1.6.0] - 27-04-2026

#### Changed
- Renombradas todas las funciones y variables públicas específicas de LoRa para incluir el prefijo `LoRa` (p. ej. `SX1262_Transmit` → `SX1262_LoRa_Transmit`). Las funciones genéricas del chip (`SX1262_Init`, `SX1262_SetSleep`, `SX1262_Wakeup`, `SX1262_IRQ_Handler`) no cambian de nombre.
  - `SX1262_Transmit` → `SX1262_LoRa_Transmit`
  - `SX1262_StartTransmitIT` → `SX1262_LoRa_StartTransmitIT`
  - `SX1262_GetTransmitStatus` → `SX1262_LoRa_GetTransmitStatus`
  - `SX1262_AbortTransmit` → `SX1262_LoRa_AbortTransmit`
  - `SX1262_Receive` → `SX1262_LoRa_Receive`
  - `SX1262_StartReceiveIT` → `SX1262_LoRa_StartReceiveIT`
  - `SX1262_GetReceivedPacket` → `SX1262_LoRa_GetReceivedPacket`
  - `SX1262_AbortReceive` → `SX1262_LoRa_AbortReceive`
  - `SX1262_ApplyConfig` → `SX1262_LoRa_ApplyConfig`
  - `SX1262_GetRSSI` → `SX1262_LoRa_GetRSSI`
  - `SX1262_GetSNR` → `SX1262_LoRa_GetSNR`
  - `SX1262_GetConfig` → `SX1262_LoRa_GetConfig`
  - `SX1262_TxDoneFlag` → `SX1262_LoRa_TxDoneFlag`
  - `SX1262_RxDoneFlag` → `SX1262_LoRa_RxDoneFlag`
  - Variable interna `SX1262_CurrentConfig` → `SX1262_LoRa_CurrentConfig`
- Preparación para la implementación futura de FSK: la separación de namespaces por modulación permite añadir funciones `SX1262_FSK_*` con una arquitectura simétrica y sin ambigüedad respecto a las funciones LoRa.

---

### [1.5.0] - 19-04-2026

#### Added
- Preset de configuración para red Meshtastic: `LORA_NETWORK_MESHTASTIC` con Sync Word 0x2B, compatible con dispositivos que usan el firmware Meshtastic.

#### Changed
- Documentación: la tabla de *Pines requeridos* fue dividida en dos subtablas diferenciadas — **Bus SPI** (VCC, GND, SCK, MISO, MOSI, ANT) y **GPIO de Control** (NSS/CS, RST, BUSY, DIO1) — para mayor claridad en la configuración de CubeMX.

---

### [1.4.0] - 17-04-2026

#### Added
- Gestión de estados de bajo consumo (Sleep Mode):
  - `SX1262_SetSleep()`: Pone el transceptor en reposo profundo. Soporta modos Warm Start (retención de configuración) y Cold Start.
  - `SX1262_Wakeup()`: Secuencia de despertar mediante NSS para reanudar operaciones tras el modo Sleep.
  - Macros de configuración: `SX126X_SLEEP_START_WARM`, `SX126X_SLEEP_START_COLD` y `SX126X_SLEEP_RTC_WAKEUP`.
- Documentación detallada de la nueva API en el README.

### [1.3.0] - 05-04-2026

#### Added
- Transmisión no bloqueante basada en interrupciones EXTI (patrón productor-consumidor):
  - `SX1262_StartTransmitIT()`: Inicia la transmisión de un paquete y retorna inmediatamente sin bloquear el CPU. El chip entra en modo TX y levanta DIO1 al terminar.
  - `SX1262_GetTransmitStatus()`: Debe invocarse desde el main loop al detectar `SX1262_TxDoneFlag`. Lee el registro IRQ del chip, verifica `TX_DONE` vs `TIMEOUT`, limpia el registro y libera el semáforo `SX1262_TxActive`.
  - `SX1262_AbortTransmit()`: Cancela la transmisión en curso y regresa el chip a Standby RC. Libera `SX1262_TxActive`. Permite implementar timeouts de software sin bloquear el CPU.
  - `volatile uint8_t SX1262_TxDoneFlag`: Bandera productor-consumidor entre el ISR y el main loop para eventos TX.
  - `static volatile uint8_t SX1262_TxActive`: Semáforo interno que indica si hay una TX IT en curso. Permite que el dispatcher `SX1262_IRQ_Handler()` decida qué bandera activar sin leer el chip.
- `SX1262_IRQ_Handler()` actualizado como **dispatcher TX/RX**: si `SX1262_TxActive == 1` activa `SX1262_TxDoneFlag`; si es `0` activa `SX1262_RxDoneFlag`. El mismo EXTI en DIO1 sirve para ambos modos sin ningún cambio en CubeMX.
- Nuevos códigos de retorno en `SX1262_Status_t`: `SX1262_TX_BUSY` (5) y `SX1262_RX_BUSY` (4).

---

### [1.2.0] - 03-04-2026

#### Added
- Recepción no bloqueante basada en interrupciones EXTI:
  - `SX1262_StartReceiveIT()`: Pone el chip en RX continuo y retorna de forma inmediata sin bloquear el CPU.
  - `SX1262_GetReceivedPacket()`: Lee el payload del buffer interno del SX1262, debe invocarse desde el main loop al detectar `SX1262_RxDoneFlag`.
  - `SX1262_AbortReceive()`: Cancela la recepción en curso y regresa el chip a Standby RC. Permite implementar timeouts de software sin bloquear el CPU.
  - `SX1262_IRQ_Handler()`: Función liviana para invocar desde `HAL_GPIO_EXTI_Callback()`. Solo activa `SX1262_RxDoneFlag` — sin SPI en el ISR.
  - `volatile uint8_t SX1262_RxDoneFlag`: Bandera productor-consumidor entre el ISR y el main loop.
- Documentación de configuración EXTI en CubeMX para el pin DIO1: modo, NVIC, prioridad y ejemplo de callback.

---

### [1.1.0] - 01-04-2026

#### Added
- Función `SX1262_GetRSSI()`:
  Permite obtener el RSSI (Received Signal Strength Indicator) del último paquete recibido, en dBm. El valor se calcula a partir del comando GetPacketStatus según el datasheet del SX1262.

- Función `SX1262_GetSNR()`:
  Permite obtener el SNR (Signal-to-Noise Ratio) del último paquete recibido, en dB. El valor se extrae del mismo comando GetPacketStatus y puede ser negativo.

- Función `SX1262_GetConfig()`:
  Retorna una copia de la configuración LoRa actualmente aplicada al chip, sin realizar comunicación SPI. El campo config_pending de la copia siempre es false.

---

### [1.0.1] - 30-03-2026

#### Fixed
- `config_pending` ahora se verifica correctamente en `SX1262_Transmit()` y `SX1262_Receive()`. La bandera existía en la estructura pero ninguna función la comprobaba.
- Timeout de TX ahora es parametrizable. Anteriormente, `SX1262_Transmit` tenía un timeout fijo de 5 segundos, a diferencia de `SX1262_Receive` que sí recibía `timeout_ms` como parámetro.
- El IRQ status de TX no se verificaba. Tras `TxDone` se lee el registro IRQ pero no se comprobaba si el bit `SX126X_IRQ_TX_DONE` estaba activo ni si ocurrió `SX126X_IRQ_TIMEOUT`.
- Asignación redundante de `SX1262_Initialized` en `SX1262_Init`. No era un bug funcional, pero es redundante.

---

### [1.0.0] - 28-03-2026

#### Added
- Versión inicial de la librería SX1262 para STM32.
- Funciones públicas: `SX1262_Init()`, `SX1262_Transmit()`, `SX1262_Receive()` y `SX1262_ApplyConfig()`.
- Cálculo automático de LDRO según SF y BW (sección 6.1.1.4 del datasheet).
- Soporte de SyncWord por modo de red: privado (0x12), público LoRaWAN (0x34) y Meshtastic (0x2B), con valor personalizado.
- Configuración automática de DIO2 como RF Switch (`SET_DIO2_AS_RF_SWITCH_CTRL`).
- Sistema de gestión de errores con códigos de retorno específicos (`SX1262_Status_t`).
- Prevención del flag OVR mediante `HAL_SPI_TransmitReceive` en lecturas SPI.
- Documentación completa con comentarios Doxygen, ejemplos de uso y tabla de pinout.
