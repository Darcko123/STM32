# SX1262 LoRa Library for STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-1.4.1-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/SX1262)
[![Protocol](https://img.shields.io/badge/Protocol-LoRa-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/SX1262)

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
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`SX1262_Status_t` - Estados de Retorno](#sx1262_status_t---estados-de-retorno)
      - [`lora_config_t` - Configuración de Parámetros LoRa](#lora_config_t---configuración-de-parámetros-lora)
      - [`lora_network_mode_t` - Modos de Red LoRa](#lora_network_mode_t---modos-de-red-lora)
      - [`lora_signal_bandwidth_t` - Ancho de Banda LoRa](#lora_signal_bandwidth_t---ancho-de-banda-lora)
      - [`lora_coding_rate_t` - Coding Rate LoRa](#lora_coding_rate_t---coding-rate-lora)
      - [Banderas `volatile` de Evento](#banderas-volatile-de-evento)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`SX1262_Init()` - Inicialización del Driver](#sx1262_init---inicialización-del-driver)
      - [`SX1262_Transmit()` - Transmisión Bloqueante](#sx1262_transmit---transmisión-bloqueante)
      - [`SX1262_StartTransmitIT()` - Iniciar Transmisión No Bloqueante](#sx1262_starttransmitit---iniciar-transmisión-no-bloqueante)
      - [`SX1262_GetTransmitStatus()` - Confirmar Resultado de TX](#sx1262_gettransmitstatus---confirmar-resultado-de-tx)
      - [`SX1262_AbortTransmit()` - Cancelar Transmisión](#sx1262_aborttransmit---cancelar-transmisión)
      - [`SX1262_Receive()` - Recepción Bloqueante](#sx1262_receive---recepción-bloqueante)
      - [`SX1262_StartReceiveIT()` - Iniciar Recepción No Bloqueante](#sx1262_startreceiveit---iniciar-recepción-no-bloqueante)
      - [`SX1262_GetReceivedPacket()` - Leer Paquete Recibido](#sx1262_getreceivedpacket---leer-paquete-recibido)
      - [`SX1262_AbortReceive()` - Cancelar Recepción](#sx1262_abortreceive---cancelar-recepción)
      - [`SX1262_IRQ_Handler()` - Manejador de Interrupción](#sx1262_irq_handler---manejador-de-interrupción)
      - [`SX1262_ApplyConfig()` - Aplicar Configuración LoRa](#sx1262_applyconfig---aplicar-configuración-lora)
      - [`SX1262_GetConfig()` - Obtener Configuración Actual](#sx1262_getconfig---obtener-configuración-actual)
      - [`SX1262_GetRSSI()` - Obtener RSSI del último paquete recibido](#sx1262_getrssi---obtener-rssi-del-último-paquete-recibido)
      - [`SX1262_GetSNR()` - Obtener SNR del último paquete recibido](#sx1262_getsnr---obtener-snr-del-último-paquete-recibido)
      - [`SX1262_SetSleep()` - Entrar en Modo Reposo](#sx1262_setsleep---entrar-en-modo-reposo)
      - [`SX1262_Wakeup()` - Despertar del Modo Reposo](#sx1262_wakeup---despertar-del-modo-reposo)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.4.0\] - 17-04-2026](#140---17-04-2026)
      - [Added](#added)
    - [\[1.3.0\] - 05-04-2026](#130---05-04-2026)
      - [Added](#added-1)
    - [\[1.2.0\] - 03-04-2026](#120---03-04-2026)
      - [Added](#added-2)
    - [\[1.1.0\] - 01-04-2026](#110---01-04-2026)
      - [Added](#added-3)
    - [\[1.0.1\] - 30-03-2026](#101---30-03-2026)
      - [Fixed](#fixed)
    - [\[1.0.0\] - 28-03-2026](#100---28-03-2026)
      - [Added](#added-4)

## Descripción
Librería desarrollada en C para la interfaz con el módulo transceptor LoRa **Semtech SX1262** utilizando microcontroladores STM32. Proporciona funciones para configurar parámetros de comunicación, transmitir y recibir datos, y manejar eventos de interrupción. La librería está diseñada para ser fácil de usar, eficiente y compatible con la mayoría de las series STM32 (F1, F4, etc.) utilizando HAL. Soporta configuraciones avanzadas de LoRa como Spreading Factor, Bandwidth, Coding Rate y potencia de transmisión. Ideal para proyectos de IoT, sensores remotos y redes de baja potencia.

---

### Características
- **Comunicación SPI**: Abstracción de comandos (Write/Read registers, buffers) con manejo robusto del flag OVR.
- **Configuración completa de parámetros LoRa**: Frecuencia (433/868/915 MHz), Spreading Factor (SF5-12), Bandwidth (7.8-500 kHz), Coding Rate (4/5 a 4/8).
- **Transmisión bloqueante** (`SX1262_Transmit`): Espera por IRQ vía polling en DIO1 (TxDone + Timeout), con timeout de software calculado a partir del Time on Air real.
- **Transmisión no bloqueante** (`SX1262_StartTransmitIT` + `SX1262_GetTransmitStatus`): El CPU no se bloquea. El chip notifica TX_DONE vía EXTI en DIO1; la bandera `SX1262_TxDoneFlag` señaliza el evento. Incluye `SX1262_AbortTransmit()` para timeouts de software.
- **Recepción bloqueante** (`SX1262_Receive`): Espera en polling hasta RxDone, con timeout configurable.
- **Recepción no bloqueante** (`SX1262_StartReceiveIT` + `SX1262_GetReceivedPacket`): El chip notifica vía EXTI en DIO1. El CPU no se bloquea; la bandera `SX1262_RxDoneFlag` señaliza el evento. Incluye `SX1262_AbortReceive()` para timeouts de software.
- **Dispatcher ISR unificado** (`SX1262_IRQ_Handler`): Un único callback EXTI en DIO1 maneja tanto TX como RX. El semáforo interno `SX1262_TxActive` decide qué bandera activar, sin realizar ninguna comunicación SPI desde el ISR.
- **Soporte para redes públicas, privadas y Meshtastic**: SyncWord configurable por modo o valor personalizado.
- **LDRO automático**: Calcula y activa LowDataRateOptimize dinámicamente según SF y BW, siguiendo la sección 6.1.1.4 del datasheet.
- **Manejo robusto de hardware**: Busy polling con timeout, reset hardware, wakeup desde Sleep.
- **Portabilidad**: Compatible con múltiples familias STM32 mediante la capa HAL. Solo requiere cambiar el `#include` del encabezado HAL.
- **Potencia TX**: Hasta +22 dBm, con rampa configurable.
- **CRC automático habilitado** en todos los paquetes transmitidos.
- **Gestión robusta de errores**: Códigos de retorno específicos para timeout, error de SPI, inicialización no completada, RX ocupado y TX ocupado.

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

Para utilizar las funciones no bloqueantes (`SX1262_StartTransmitIT` / `SX1262_StartReceiveIT`), el pin DIO1 debe configurarse como entrada con interrupción externa (EXTI) en lugar de entrada GPIO simple. El chip SX1262 mantiene DIO1 en bajo en reposo y genera un **flanco de subida** al completar una operación (TxDone, RxDone, Timeout, etc.).

> [!NOTE]
> La misma configuración EXTI en DIO1 sirve tanto para transmisión como para recepción no bloqueante. El dispatcher `SX1262_IRQ_Handler()` determina automáticamente qué bandera activar (`SX1262_TxDoneFlag` o `SX1262_RxDoneFlag`) según el semáforo interno `SX1262_TxActive`.

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
> `SX1262_IRQ_Handler()` actúa como **dispatcher**: si `SX1262_TxActive == 1` activa `SX1262_TxDoneFlag`; si es `0` activa `SX1262_RxDoneFlag`. Toda comunicación SPI (leer IRQ status, leer/escribir buffer) se realiza únicamente en el main loop, **nunca dentro del ISR**.

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
2. Ajusta el `#include` del encabezado HAL en `SX1262.h` según tu familia STM32:
   ```c
   // STM32F1xx:
   #include "stm32f1xx_hal.h"
   // STM32F4xx:
   #include "stm32f4xx_hal.h"
   ```
3. Incluye la librería en tu `main.c` o archivo principal:
   ```c
   #include "SX1262.h"
   ```
4. Configura SPI y GPIOs en CubeMX (ver secciones anteriores).
5. Genera código y compila.

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

SX1262_Status_t status = SX1262_Transmit(mensaje, len);
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
    if (SX1262_StartTransmitIT(tx_buf, tx_len) == SX1262_OK)
    {
        tx_busy       = true;
        tx_start_tick = HAL_GetTick();
    }
}

/* BLOQUE 2 — Procesar TX_DONE activado por el ISR */
if (SX1262_TxDoneFlag)
{
    SX1262_TxDoneFlag = 0;  // Consumir bandera — SIEMPRE primero
    SX1262_Status_t result = SX1262_GetTransmitStatus();

    if (result == SX1262_OK)     { /* TX_DONE confirmado   */ }
    else if (result == SX1262_TIMEOUT) { /* Timeout interno del chip */ }
    else                         { /* Condición inesperada */ }

    tx_busy      = false;
    last_tx_tick = HAL_GetTick();
}

/* BLOQUE 3 (opcional) — Timeout de software */
if (tx_busy && (HAL_GetTick() - tx_start_tick) >= TX_SW_TIMEOUT_MS)
{
    SX1262_AbortTransmit();  // Standby RC + Clear IRQ + TxActive = 0
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
SX1262_Status_t status = SX1262_Receive(rx_buffer, &rx_len, 5000);

if (status == SX1262_OK) {
    // rx_buffer[0..rx_len-1] contiene los datos recibidos
} else if (status == SX1262_TIMEOUT) {
    // No se recibió ningún paquete en el tiempo indicado
} else if (status == SX1262_ERROR) {
    // Error CRC u otro error de recepción
}

// Recepción continua bloqueante indefinida (timeout_ms = 0)
status = SX1262_Receive(rx_buffer, &rx_len, 0);
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
        SX1262_IRQ_Handler();  // Solo activa SX1262_RxDoneFlag — sin SPI
    }
}

// ---------------------------------------------------------------
// En USER CODE BEGIN 2  (setup, antes del while)
// ---------------------------------------------------------------
SX1262_StartReceiveIT();   // Pone el chip en RX continuo y retorna inmediatamente

// ---------------------------------------------------------------
// En while(1)
// ---------------------------------------------------------------
if (SX1262_RxDoneFlag)
{
    SX1262_RxDoneFlag = 0;                              // Consumir bandera
    status = SX1262_GetReceivedPacket(rx_buffer, &rx_len); // Leer payload

    if (status == SX1262_OK)
    {
        // Procesar rx_buffer[0..rx_len-1]
    }
    else if (status == SX1262_TIMEOUT) { /* timeout interno del chip */ }
    else if (status == SX1262_ERROR)   { /* CRC u otro error */         }

    SX1262_StartReceiveIT();  // Re-enganche: volver a escuchar
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

SX1262_Status_t status = SX1262_ApplyConfig(&mi_config);
```

> [!NOTE]
> `SX1262_ApplyConfig()` puede llamarse en cualquier momento después de la inicialización para cambiar los parámetros sin reiniciar el chip. Todas las funciones de transmisión y recepción (`SX1262_Transmit`, `SX1262_StartTransmitIT`, `SX1262_Receive`, `SX1262_StartReceiveIT`) siempre trabajan con la última configuración aplicada. Si hay cambios pendientes sin aplicar (`config_pending == true`), retornan `SX1262_ERROR`.

### 7. Ejemplo de Transmisión y Recepción Completa

![Transmisión y Recepción](/Librerias/SX1262/Images/TransmisionRecepcion.png)

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
    SX1262_OK             = 0, /**< Operación exitosa */
    SX1262_ERROR          = 1, /**< Error en la operación */
    SX1262_TIMEOUT        = 2, /**< Timeout en la operación */
    SX1262_NOT_INITIALIZED = 3, /**< Módulo no inicializado */
    SX1262_RX_BUSY        = 4, /**< Módulo en modo RX IT — ya hay recepción activa */
    SX1262_TX_BUSY        = 5  /**< Módulo en modo TX IT — ya hay transmisión activa */
} SX1262_Status_t;
```

| Valor | Código | Significado |
|-------|--------|-------------|
| `SX1262_OK` | 0 | Operación completada sin errores |
| `SX1262_ERROR` | 1 | Error de SPI, parámetro inválido o condición inesperada de IRQ |
| `SX1262_TIMEOUT` | 2 | Timeout de software o timeout interno del chip |
| `SX1262_NOT_INITIALIZED` | 3 | `SX1262_Init()` no se llamó o falló |
| `SX1262_RX_BUSY` | 4 | Se intentó iniciar RX IT cuando ya había una en curso |
| `SX1262_TX_BUSY` | 5 | Se intentó iniciar TX IT cuando ya había una en curso |

#### `lora_config_t` - Configuración de Parámetros LoRa

Estructura que encapsula todos los parámetros configurables para la comunicación LoRa. Se pasa a `SX1262_ApplyConfig()` para aplicar cambios al chip.

```c
typedef struct {
	uint32_t frequency;		            // Hz (default: 915000000)
	uint8_t spreading_factor;	        // 5 to 12 (default: 7)
	lora_signal_bandwidth_t bandwidth;  // BW_125_KHZ, BW_250_KHZ, BW_500_KHZ...
	uint8_t coding_rate;                // CR_4_5, CR_4_6, CR_4_7, CR_4_8 (default: CR_4_5)
	int8_t tx_power;                    // -9 to 22 dBm (default: 20)
	uint16_t preamble_len;	            // Default: 12
	bool iq_inverted;	                // IQ inversion (default: false/normal)
	lora_network_mode_t network_mode;   // Sync word: LORA_NETWORK_PRIVATE / PUBLIC / MESHTASTIC
	uint8_t lora_sync_word;             // Custom sync word (distinto de 0 tiene prioridad sobre network_mode)
	bool config_pending;	            // true if changes not yet applied
} lora_config_t;
```

**Prioridad del Sync Word:** si `lora_sync_word != 0`, tiene prioridad absoluta sobre `network_mode`.

> [!IMPORTANT]
> Si `config_pending == true`, las funciones `SX1262_Transmit()`, `SX1262_StartTransmitIT()`, `SX1262_Receive()` y `SX1262_StartReceiveIT()` retornarán `SX1262_ERROR`. Debes llamar a `SX1262_ApplyConfig()` antes de operar.

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
}lora_coding_rate_t;
```

---

#### Banderas `volatile` de Evento

Las banderas de evento implementan el patrón **productor-consumidor** entre el ISR (productor) y el main loop (consumidor). Están declaradas `volatile` para que el compilador no las optimice fuera de la RAM.

```c
extern volatile uint8_t SX1262_TxDoneFlag;  // 1 cuando TX IT completó (TxDone o Timeout)
extern volatile uint8_t SX1262_RxDoneFlag;  // 1 cuando RX IT completó (RxDone, Timeout o error)
```

| Bandera | Activada por | Consumida por | Función de consumo |
|---------|-------------|--------------|--------------------|
| `SX1262_TxDoneFlag` | `SX1262_IRQ_Handler()` cuando `TxActive == 1` | Main loop | `SX1262_GetTransmitStatus()` |
| `SX1262_RxDoneFlag` | `SX1262_IRQ_Handler()` cuando `TxActive == 0` | Main loop | `SX1262_GetReceivedPacket()` |

> [!WARNING]
> Siempre poner la bandera a `0` **antes** de llamar a la función de consumo, para evitar perder un nuevo evento que pudiera ocurrir durante el procesamiento SPI.

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

#### `SX1262_Transmit()` - Transmisión Bloqueante

Transmite un buffer de datos por LoRa. La función es **bloqueante**: hace polling en DIO1 hasta que el chip sube `TxDone` o expira el timeout de software (calculado dinámicamente como `ToA × 1.5 + 100 ms`).

```c
SX1262_Status_t SX1262_Transmit(uint8_t* data, uint8_t length);
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

#### `SX1262_StartTransmitIT()` - Iniciar Transmisión No Bloqueante

Inicia la transmisión de un paquete y **retorna inmediatamente**. El CPU queda libre para ejecutar otras tareas. El evento de finalización se señaliza mediante `SX1262_TxDoneFlag`, activada por el ISR cuando DIO1 genera el flanco de subida.

```c
SX1262_Status_t SX1262_StartTransmitIT(uint8_t* data, uint8_t length);
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
> No llamar si `SX1262_TxDoneFlag` aún no ha sido consumida. Usar el estado de la máquina de estados del main loop para garantizar que la TX anterior ha terminado.

---

#### `SX1262_GetTransmitStatus()` - Confirmar Resultado de TX

Verifica y consume el evento `TX_DONE` después de que `SX1262_TxDoneFlag == 1`. Lee el registro IRQ del chip para distinguir entre `TX_DONE` real y `TIMEOUT` interno, limpia el registro y libera el semáforo `SX1262_TxActive`.

```c
SX1262_Status_t SX1262_GetTransmitStatus(void);
```

**Retorna:** `SX1262_OK` si `TX_DONE` fue confirmado, `SX1262_TIMEOUT` si el chip reporta timeout interno, `SX1262_ERROR` si la condición de IRQ es inesperada o falla SPI, `SX1262_NOT_INITIALIZED` si el módulo no fue inicializado.

> [!WARNING]
> Llamar **solo desde el main loop** cuando `SX1262_TxDoneFlag == 1`. **Nunca desde el ISR** — realiza comunicación SPI.

**Secuencia interna:**
1. Lee `GetIrqStatus`.
2. Limpia el registro IRQ (`ClearIRQ`) — siempre, independientemente del resultado.
3. Libera `SX1262_TxActive = 0`.
4. Evalúa `TIMEOUT` (prioridad) y luego `TX_DONE`.

---

#### `SX1262_AbortTransmit()` - Cancelar Transmisión

Cancela la transmisión IT en curso, regresa el chip a **Standby RC** y libera el semáforo `SX1262_TxActive`. Útil para implementar timeouts de software sin bloquear el CPU.

```c
SX1262_Status_t SX1262_AbortTransmit(void);
```

**Retorna:** `SX1262_OK` si el chip volvió a Standby, `SX1262_ERROR` si falla la escritura SPI.

**Secuencia interna:**
1. `SetStandby RC` — detiene la transmisión activa.
2. `ClearIRQ` — limpia IRQs residuales.
3. `SX1262_TxActive = 0` — libera el semáforo para nuevas operaciones.

---

#### `SX1262_Receive()` - Recepción Bloqueante

Pone el chip en modo recepción y espera un paquete válido. **Bloqueante** con timeout configurable.

```c
SX1262_Status_t SX1262_Receive(uint8_t* data, uint8_t* length, uint32_t timeout_ms);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `data` | `uint8_t*` | Buffer donde se almacenarán los datos recibidos |
| `length` | `uint8_t*` | Puntero donde se escribirá la longitud del paquete recibido |
| `timeout_ms` | `uint32_t` | Tiempo máximo de espera en ms. `0` = espera indefinida |

**Retorna:** `SX1262_OK` si se recibió un paquete válido, `SX1262_TIMEOUT` si expiró el tiempo, `SX1262_ERROR` si hubo error CRC o fallo de IRQ, `SX1262_NOT_INITIALIZED` si el módulo no fue inicializado.

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

#### `SX1262_StartReceiveIT()` - Iniciar Recepción No Bloqueante

Pone el chip en modo RX continuo y **retorna inmediatamente**. El evento de recepción se señaliza mediante la bandera `volatile uint8_t SX1262_RxDoneFlag`, que es activada desde el ISR. Requiere que DIO1 esté configurado como EXTI en CubeMX.

```c
SX1262_Status_t SX1262_StartReceiveIT(void);
```

**Retorna:** `SX1262_OK` si el chip entró en modo RX correctamente, `SX1262_ERROR` si falla alguna escritura SPI, `SX1262_NOT_INITIALIZED` si el módulo no fue inicializado.

**Secuencia interna:**
1. Standby RC.
2. Limpia registro IRQ (ClearIRQ).
3. Habilita `RxDone | Timeout | CRC_ERR | HeaderErr` en DIO1 (SetDioIrqParams).
4. Inicia RX con timeout `0xFFFFFF` (modo continuo) → `SET_RX`.
5. Retorno inmediato — sin polling.

---

#### `SX1262_GetReceivedPacket()` - Leer Paquete Recibido

Lee el payload del buffer interno del SX1262. Debe llamarse **solo** cuando `SX1262_RxDoneFlag == 1` (desde el main loop, no desde el ISR).

```c
SX1262_Status_t SX1262_GetReceivedPacket(uint8_t* data, uint8_t* length);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `data` | `uint8_t*` | Buffer donde se almacenará el payload recibido |
| `length` | `uint8_t*` | Puntero donde se escribirá la longitud del paquete recibido |

**Retorna:** `SX1262_OK` si se leyó un paquete válido, `SX1262_TIMEOUT` si el IRQ indica timeout del chip, `SX1262_ERROR` si hay error CRC, header inválido o fallo SPI.

**Secuencia interna:**
1. Lee registro IRQ (`GetIrqStatus`).
2. Verifica bits: `RxDone` presente, `Timeout` y `CRC_ERR` ausentes.
3. Lee `GetRxBufferStatus` para obtener tamaño y offset del paquete.
4. Lee payload con `ReadBuffer`.
5. Limpia registro IRQ.

---

#### `SX1262_AbortReceive()` - Cancelar Recepción

Cancela la recepción en curso y regresa el chip al modo Standby. Útil para implementar un timeout de software sin bloquear el CPU.

```c
SX1262_Status_t SX1262_AbortReceive(void);
```

**Retorna:** `SX1262_OK` si el chip volvió a Standby, `SX1262_ERROR` si falla la escritura SPI.

---

#### `SX1262_IRQ_Handler()` - Manejador de Interrupción

Función **liviana** (dispatcher) que debe llamarse desde `HAL_GPIO_EXTI_Callback()` cuando el pin DIO1 genera una interrupción. Determina sin comunicación SPI si el evento corresponde a una TX o RX activa, usando el semáforo interno `SX1262_TxActive`, y activa la bandera correspondiente.

```c
void SX1262_IRQ_Handler(void);
```

**Lógica de despacho:**

```c
if (SX1262_TxActive)
    SX1262_TxDoneFlag = 1;  // Evento TX → consumir con SX1262_GetTransmitStatus()
else
    SX1262_RxDoneFlag = 1;  // Evento RX → consumir con SX1262_GetReceivedPacket()
```

> [!WARNING]
> Esta función está diseñada para ejecutarse en contexto de **interrupción (ISR)**. Regla crítica: **sin SPI, sin HAL calls bloqueantes, sin printf**. Toda comunicación SPI se realiza únicamente en el main loop mediante `SX1262_GetTransmitStatus()` o `SX1262_GetReceivedPacket()`.

**TX y RX son mutuamente excluyentes** en el chip SX1262, por lo que el semáforo `SX1262_TxActive` es suficiente para un despacho seguro desde el ISR.

---

#### `SX1262_ApplyConfig()` - Aplicar Configuración LoRa

Aplica una nueva configuración de modulación y red al chip. Puede llamarse en cualquier momento después de `SX1262_Init()`. No reinicia el chip: solo pone el módulo en Standby RC y reconfigura los registros necesarios.

```c
SX1262_Status_t SX1262_ApplyConfig(lora_config_t *config);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `config` | `lora_config_t*` | Puntero a la estructura de configuración |

**Retorna:** `SX1262_OK` si todos los parámetros fueron escritos correctamente, `SX1262_ERROR` si falla alguna escritura SPI o `config` es NULL, `SX1262_NOT_INITIALIZED` si el módulo no fue inicializado.

**Parámetros aplicados en orden:**
1. Frecuencia RF (conversión entera 64-bit sin punto flotante).
2. PA Config + TX Params (potencia y rampa de 40 µs).
3. Modulation Params: SF, BW, CR y LDRO calculado automáticamente.
4. Sync Word (registros 0x0740–0x0741).
5. Packet Params: preámbulo, header explícito, longitud dummy, CRC on, IQ.


#### `SX1262_GetConfig()` - Obtener Configuración Actual

Permite obtener una copia de la configuración LoRa actualmente aplicada al chip. La función retorna una estructura `lora_config_t` con los parámetros que fueron aplicados en el último `SX1262_ApplyConfig()`. No realiza comunicación SPI, ya que la librería mantiene una copia local de la configuración aplicada. El campo `config_pending` de la copia siempre se establece en `false`, ya que esta función solo refleja la configuración que ya está activa en el chip.

```c
SX1262_Status_t SX1262_GetConfig(lora_config_t *config);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `config` | `lora_config_t*` | Puntero a la estructura donde se almacenará la configuración actual |

#### `SX1262_GetRSSI()` - Obtener RSSI del último paquete recibido

Permite obtener el RSSI (Received Signal Strength Indicator) del último paquete recibido, en dBm. El valor se calcula a partir del comando `GetPacketStatus` según el datasheet del SX1262.

```c
SX1262_Status_t SX1262_GetRSSI(int16_t *rssi_dbm);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `rssi_dbm` | `int16_t*` | Puntero donde se almacenará el valor de RSSI en dBm |


#### `SX1262_GetSNR()` - Obtener SNR del último paquete recibido

Permite obtener el SNR (Signal-to-Noise Ratio) del último paquete recibido, en dB. El valor se extrae del mismo comando `GetPacketStatus` y puede ser negativo.

```c 
SX1262_Status_t SX1262_GetSNR(int8_t *snr_db);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `snr_db` | `int8_t*` | Puntero donde se almacenará el valor de SNR en dB |

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