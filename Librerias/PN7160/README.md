# Librería para el controlador NFC PN7160 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32%20(HAL)-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)
[![Protocol](https://img.shields.io/badge/Protocol-SPI%20%7C%20I2C%20%7C%20NCI%202.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/PN7160)

---

## Tabla de Contenidos
- [Librería para el controlador NFC PN7160 en STM32](#librería-para-el-controlador-nfc-pn7160-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Bus SPI](#bus-spi)
    - [Bus I2C](#bus-i2c)
    - [GPIO de Control](#gpio-de-control)
    - [Configuración EXTI para IRQ (modo por interrupción)](#configuración-exti-para-irq-modo-por-interrupción)
  - [Configuración SPI](#configuración-spi)
  - [Configuración I2C](#configuración-i2c)
  - [Selección del transporte](#selección-del-transporte)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Leer el UID de un tag (bloqueante)](#2-leer-el-uid-de-un-tag-bloqueante)
    - [3. Intercambiar datos con un tag](#3-intercambiar-datos-con-un-tag)
    - [4. Esperar la retirada del tag](#4-esperar-la-retirada-del-tag)
    - [5. Descubrimiento por interrupción](#5-descubrimiento-por-interrupción)
    - [6. Emulación de tarjeta](#6-emulación-de-tarjeta)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`PN7160_Status_t` - Estados de Retorno](#pn7160_status_t---estados-de-retorno)
      - [`PN7160_Mode_t` - Modo RF](#pn7160_mode_t---modo-rf)
      - [`PN7160_Tag_t` - Datos del Tag](#pn7160_tag_t---datos-del-tag)
      - [`PN7160_Info_t` - Identificación del Chip](#pn7160_info_t---identificación-del-chip)
      - [Constantes](#constantes)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`PN7160_Init()` - Inicialización](#pn7160_init---inicialización)
      - [`PN7160_DeInit()` - Desinicialización](#pn7160_deinit---desinicialización)
      - [`PN7160_GetInfo()` - Modelo y Firmware](#pn7160_getinfo---modelo-y-firmware)
      - [`PN7160_SetMode()` - Lector o Emulación de Tarjeta](#pn7160_setmode---lector-o-emulación-de-tarjeta)
      - [`PN7160_SetCardUid()` - UID en Modo Tarjeta](#pn7160_setcarduid---uid-en-modo-tarjeta)
      - [`PN7160_WaitTag()` - Esperar un Tag](#pn7160_waittag---esperar-un-tag)
      - [`PN7160_Transceive()` - Intercambio de Datos](#pn7160_transceive---intercambio-de-datos)
      - [`PN7160_ReleaseTag()` - Liberar el Tag](#pn7160_releasetag---liberar-el-tag)
      - [`PN7160_WaitTagRemoval()` - Esperar la Retirada](#pn7160_waittagremoval---esperar-la-retirada)
      - [`PN7160_MfcAuth()` / `MfcRead()` / `MfcWrite()` - MIFARE Classic](#pn7160_mfcauth--mfcread--mfcwrite---mifare-classic)
    - [3. Descubrimiento por Interrupción](#3-descubrimiento-por-interrupción)
      - [`PN7160_StartDiscoveryIT()` - Arrancar el Descubrimiento](#pn7160_startdiscoveryit---arrancar-el-descubrimiento)
      - [`PN7160_IRQ_Handler()` - Manejador de Interrupción](#pn7160_irq_handler---manejador-de-interrupción)
      - [`PN7160_Process()` - Atender el Descubrimiento](#pn7160_process---atender-el-descubrimiento)
      - [`PN7160_TagDetectedCallback()` - Callback de Tag Detectado](#pn7160_tagdetectedcallback---callback-de-tag-detectado)
  - [Limitaciones conocidas](#limitaciones-conocidas)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.0.0\] - 25-11-2026](#100---25-11-2026)
      - [Added](#added)

---

## Descripción

Driver en C para el controlador NFC **NXP PN7160 / PN7161** sobre **STM32 HAL**, conectado por **SPI** o **I2C** (se elige en compilación). Implementa la pila **NCI 2.0** a nivel de comandos (reset, inicialización, descubrimiento RF, activación, paquetes de datos y créditos), así que la aplicación solo trabaja con tags, UIDs y buffers de datos.

Soporta los dos papeles del chip:

- **Lector/escritor**: sondea NFC-A, NFC-B, NFC-F y NFC-V y entrega la tecnología, el protocolo, el UID y, en NFC-A, el ATQA y el SAK.
- **Emulación de tarjeta T4T** (ISO-DEP sobre NFC-A): el PN7160 se presenta ante un lector externo y el host responde los APDU.

La librería es un **driver NCI**: no interpreta los datos. No incluye NDEF ni lógica por tipo de tag; `PN7160_Transceive()` envía y recibe los bytes tal cual (APDU, comandos T2T, etc.). La excepción es MIFARE Classic: su formato TAG-CMD es propietario de NXP, así que el driver lo encapsula en `PN7160_MfcAuth()`, `PN7160_MfcRead()` y `PN7160_MfcWrite()`.

> [!NOTE]
> El datasheet ([PN7160_PN7161.pdf](PN7160_PN7161.pdf)) no documenta la trama SPI ni los comandos NCI. Esos detalles salen de los manuales de usuario **UM11495** (PN7160) y **UM11577**, y vienen comentados en el `.h`.

---

## Características

- **Inicialización NCI completa**: `PN7160_Init()` hace el reset por hardware con VEN, `CORE_RESET`, `CORE_INIT` (NCI 2.0), activa las extensiones propietarias de NXP (MIFARE Classic como protocolo `0x80`) y configura el discover map (ISO-DEP → interfaz ISO-DEP, MIFARE → interfaz TAG-CMD, el resto Frame RF).
- **Verificación del chip**: valida el model ID del `CORE_RESET_NTF` (`0x61` PN7160, `0x71` PN7161) y guarda la versión de firmware, disponible con `PN7160_GetInfo()`.
- **Reloj RF configurable**: `CLOCK_SEL_CFG` se lee y solo se escribe en EEPROM si difiere de `PN7160_CLOCK_SEL_CFG` (cristal de 27.12 MHz por defecto), así que no se gasta EEPROM en cada arranque.
- **Lectura de tags NFC-A/B/F/V**: `PN7160_WaitTag()` devuelve tecnología, protocolo, UID de 4/7/10 bytes (NFC-A), NFCID0 (B), NFCID2 (F) o UID (V), más ATQA y SAK en NFC-A (p. ej. distingue MIFARE Classic 1K de 4K).
- **Intercambio de datos**: `PN7160_Transceive()` segmenta los envíos según el *Max Data Packet Payload Size*, gestiona los créditos NCI y reensambla la respuesta, con protección de tamaño del buffer de recepción.
- **MIFARE Classic**: `PN7160_MfcAuth()` (clave A o B), `PN7160_MfcRead()` y `PN7160_MfcWrite()` arman y validan los TAG-CMD de NXP; rechazan cualquier tag que no sea MIFARE Classic.
- **Emulación de tarjeta T4T**: `PN7160_SetMode(PN7160_MODE_CARD)` rutea ISO-DEP al host; con `PN7160_SetCardUid()` se puede fijar el NFCID1 anunciado (4, 7 o 10 bytes) o volver al UID dinámico.
- **Detección de retirada**: `PN7160_WaitTagRemoval()` espera a que el tag o el lector externo salga del campo.
- **Dos modos de espera**:
  - **Bloqueante** (`PN7160_WaitTag`) con timeout, leyendo la línea IRQ por polling.
  - **Por interrupción** (`PN7160_StartDiscoveryIT` + `PN7160_IRQ_Handler` + `PN7160_Process` + `PN7160_TagDetectedCallback`), sin acceso al bus dentro de la ISR y con callback `__weak` al estilo HAL.
- **Bajo consumo**: `PN7160_ReleaseTag()` apaga el campo RF al terminar y `PN7160_DeInit()` deja el chip en *Hard Power Down* (10.5 µA típ.).
- **Timeouts redefinibles**: `PN7160_TIMEOUT_MS`, `PN7160_RF_TIMEOUT_MS`, `PN7160_REMOVAL_MS` y `PN7160_BOOT_MS` se pueden cambiar antes de incluir `PN7160.h`.
- **SPI o I2C**: el transporte se elige con `PN7160_TRANSPORT_SPI` / `PN7160_TRANSPORT_I2C`; la pila NCI es la misma para ambos y solo cambia `PN7160_Init()`.
- **Portabilidad**: `#include "main.h"` toma el encabezado HAL de la familia STM32 generado por CubeMX.

---

## Pinout y Conexiones

### Bus SPI

| Pin PN7160 | Dirección (desde el STM32) | Descripción | Configuración CubeMX |
|------------|----------------------------|-------------|----------------------|
| **VBAT** | Alimentación | Alimentación principal y del frontal RF | N/A |
| **VDD(PAD)** | Alimentación | Nivel de los pines del host (3.3 V con STM32) | N/A |
| **GND** | Tierra | **Común** con el STM32 | N/A |
| **SCK** | Output | Reloj SPI (≤ 7 MHz) | Periférico SPI |
| **MOSI** | Output | Datos STM32 → PN7160 | Periférico SPI |
| **MISO** | Input | Datos PN7160 → STM32 | Periférico SPI |

### Bus I2C

| Pin PN7160 | Dirección (desde el STM32) | Descripción | Configuración CubeMX |
|------------|----------------------------|-------------|----------------------|
| **VBAT**, **VDD(PAD)**, **GND** | Alimentación | Igual que en SPI. VDD(PAD) siempre presente: HIF3/HIF4 no son *fail-safe* | N/A |
| **SDA** (HIF3) | Bidireccional | Datos I2C, pull-up a VDD(PAD) | Periférico I2C |
| **SCL** (HIF4) | Output | Reloj I2C, pull-up a VDD(PAD); el PN7160 puede alargarlo (*clock stretching*) | Periférico I2C |
| **I2C_ADR0** (HIF1), **I2C_ADR1** (HIF2) | Nivel fijo | Eligen la dirección (ver [Configuración I2C](#configuración-i2c)). Átalos a GND o a VDD(PAD): tienen pull-up interno, así que al aire leen `1` | N/A |

En I2C no hay NSS (HIF1 pasa a ser I2C_ADR0); VEN, IRQ y DWL_REQ se conectan igual que en SPI.

> [!IMPORTANT]
> SDA y SCL necesitan **pull-ups externos** a VDD(PAD) (típicamente 2.2–4.7 kΩ a 100/400 kHz; el datasheet caracteriza los tiempos con 2 kΩ). Los pull-ups internos del GPIO del STM32 (~40 kΩ) son demasiado débiles.

### GPIO de Control

| Pin PN7160 | Dirección | Descripción | Tipo GPIO | Nivel inicial | Pull | Etiqueta sugerida |
|------------|-----------|-------------|-----------|---------------|------|-------------------|
| **NSS** (HIF1) | Output | Chip Select del SPI. **Solo SPI** | `GPIO_Output` push-pull | HIGH | No pull | `NFC_NSS` |
| **VEN** | Output | Enable / reset; en bajo = *Hard Power Down* | `GPIO_Output` push-pull | LOW | No pull | `NFC_VEN` |
| **IRQ** | Input | El PN7160 lo sube cuando tiene un paquete NCI para leer | **Bloqueante:** `GPIO_Input` · **IT:** `GPIO_EXTI` flanco de subida | — | No pull | `NFC_IRQ` |
| **DWL_REQ** | Output o GND | Petición de descarga de firmware | — | **LOW** | — | — |

> [!IMPORTANT]
> - **DWL_REQ debe estar en bajo** al subir VEN; en alto el chip arranca en modo descarga de firmware y `PN7160_Init()` falla.
> - **VBAT y VDD(PAD) deben estar presentes antes de subir VEN.** La librería controla VEN; no la subas a mano antes de `PN7160_Init()`.
> - La **emulación de tarjeta requiere VBAT > 2.8 V** (datasheet, *Listener mode*).

### Configuración EXTI para IRQ (modo por interrupción)

Solo hace falta para `PN7160_StartDiscoveryIT()`. El modo bloqueante funciona con IRQ como entrada simple, pero también con EXTI: `PN7160_WaitTag()` lee el nivel del pin igual.

1. En *Pinout & Configuration*, pon el pin IRQ en `GPIO_EXTIx`.
2. En *GPIO*: **External Interrupt Mode with Rising edge trigger detection**, *No pull-up and no pull-down*, etiqueta `NFC_IRQ`.
3. En *System Core → NVIC*, habilita la línea `EXTIx` correspondiente. No le asignes prioridad 0 si SysTick también la usa.
4. En `main.c`, redirige el callback del HAL a la librería:

```c
/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == NFC_IRQ_Pin)
    {
        PN7160_IRQ_Handler();   /* Solo activa una bandera: sin acceso al bus en la ISR */
    }
}
/* USER CODE END 4 */
```

---

## Configuración SPI

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | Full-Duplex Master | — |
| **Hardware NSS Signal** | Disable | NSS por GPIO, lo maneja la librería |
| **Frame Format** | Motorola | — |
| **Data Size** | 8 Bits | — |
| **First Bit** | MSB First | — |
| **Prescaler (Baud Rate)** | SCK **≤ 7 MHz** | Periodo mínimo de SCK de 142 ns (datasheet) |
| **Clock Polarity (CPOL)** | Low | Modo SPI 0 |
| **Clock Phase (CPHA)** | 1 Edge | Modo SPI 0 |
| **CRC Calculation** | Disabled | — |

> [!NOTE]
> `PN7160_Init()` no valida la configuración del SPI. Si `Init` devuelve `PN7160_TIMEOUT` o `PN7160_ERROR`, revisa primero el modo SPI, la velocidad, DWL_REQ y la alimentación.

La trama SPI es la de los manuales de NXP: con NSS en bajo, el byte `0x7F` precede a la escritura de un paquete NCI y `0xFF` a su lectura. Cada respuesta se espera a que IRQ pase a alto antes de leerla.

---

## Configuración I2C

En CubeMX, *Connectivity → I2Cx*:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | I2C | El STM32 actúa como maestro; la librería usa `HAL_I2C_Master_Transmit/Receive` |
| **I2C Speed Mode** | Standard Mode (100 kHz) o Fast Mode (400 kHz) | El PN7160 **no** admite Fast-mode Plus (1 MHz). Admite High-speed (3.4 MHz), pero el periférico I2C del STM32 no, así que en la práctica el máximo es 400 kHz |
| **I2C Clock Speed (Hz)** | 100000 o 400000 | — |
| **Fast Mode Duty Cycle** | Duty cycle Tlow/Thigh = 2 | Solo en Fast Mode y en familias con el I2C antiguo (F1/F2/F4/L1) |
| **Clock No Stretch Mode** | Disabled | El PN7160 usa *clock stretching* como handshake. Como maestro, el STM32 siempre lo respeta; el parámetro solo afecta en modo esclavo |
| **Primary Address Length** | 7-bit | El PN7160 solo admite direccionamiento de 7 bits (no 10 bits) |
| **Dual Address Acknowledged** | Disabled | — |
| **General Call Address Detection** | Disabled | El PN7160 no admite *General call* |
| **GPIO (SDA/SCL)** | Alternate Function Open Drain, No pull | Pull-ups externos (ver [Bus I2C](#bus-i2c)) |

La dirección se fija con `PN7160_I2C_ADDRESS` (formato HAL, 7 bits ya desplazada; cabecera fija `01010` + HIF2 + HIF1, datasheet tabla 14):

| HIF2 (ADR1) | HIF1 (ADR0) | `PN7160_I2C_ADDRESS` |
|-------------|-------------|----------------------|
| GND | GND | `0x50` (por defecto) |
| GND | VDD(PAD) | `0x52` |
| VDD(PAD) | GND | `0x54` |
| VDD(PAD) | VDD(PAD) o al aire | `0x56` |

Si tu placa no usa `0x50`, redefínela en `main.h` junto a la macro del transporte:

```c
/* USER CODE BEGIN Private defines */
#define PN7160_TRANSPORT_I2C
#define PN7160_I2C_ADDRESS  0x56U   /* HIF1 y HIF2 en alto */
/* USER CODE END Private defines */
```

> [!NOTE]
> `PN7160_Init()` no valida la configuración del I2C. Si devuelve `PN7160_ERROR` (NACK en la dirección) o `PN7160_TIMEOUT`, revisa primero la dirección (HIF1/HIF2), los pull-ups, que VDD(PAD) esté presente, DWL_REQ y que el módulo sea de la variante I2C.

La trama I2C no lleva byte de dirección como la SPI: se escribe el paquete NCI tal cual y se lee en dos lecturas (cabecera de 3 bytes y payload). Igual que en SPI, cada respuesta se espera a que IRQ suba; no hay espera fija tras escribir. Si una escritura no recibe ACK (el chip despertando de *Standby* con su dirección), se reintenta una vez tras `PN7160_I2C_RETRY_MS`.

---

## Selección del transporte

El PN7160 sale de fábrica con la interfaz fijada (SPI o I2C según el número de pedido), así que se compila solo la que corresponde. **I2C es el transporte por defecto**: si no defines nada, `PN7160.h` define `PN7160_TRANSPORT_I2C`. Para SPI, define `PN7160_TRANSPORT_SPI` para todo el proyecto; si defines las dos, `PN7160.h` da `#error`:

- **En `main.h`** (recomendado, sobrevive a la regeneración de CubeMX):
  ```c
  /* USER CODE BEGIN Private defines */
  #define PN7160_TRANSPORT_SPI
  /* USER CODE END Private defines */
  ```
- **Como símbolo del compilador**: *Project → Properties → C/C++ Build → Settings → MCU GCC Compiler → Preprocessor → Define symbols* → `PN7160_TRANSPORT_SPI` (en Debug **y** Release).

No la definas solo antes del `#include` en `main.c`: `PN7160.c` no la vería y no compilaría.

---

## Instalación

1. Copia `PN7160.c` y `PN7160.h` a tu proyecto (ej: `Librerias/PN7160/`).
2. Incluye la librería en tu `main.c`:
   ```c
   #include "PN7160.h"
   ```
3. Elige el transporte como se describe en [Selección del transporte](#selección-del-transporte).
4. Configura el SPI o el I2C y los GPIO como se describe en [Pinout y Conexiones](#pinout-y-conexiones) y [Configuración SPI](#configuración-spi) / [Configuración I2C](#configuración-i2c).
5. Opcional: redefine los timeouts o `PN7160_CLOCK_SEL_CFG` antes del `#include` (o como símbolo del compilador) si tu placa lo necesita.
6. Genera código y compila.

---

## Uso Básico

### 1. Inicialización

**Con `PN7160_TRANSPORT_SPI`:**

```c
/* Después de HAL_Init(), SystemClock_Config(), MX_GPIO_Init() y MX_SPI1_Init() */
if (PN7160_Init(&hspi1,
                NFC_NSS_GPIO_Port, NFC_NSS_Pin,
                NFC_VEN_GPIO_Port, NFC_VEN_Pin,
                NFC_IRQ_GPIO_Port, NFC_IRQ_Pin) != PN7160_OK)
{
    Error_Handler();
}
```

**Con `PN7160_TRANSPORT_I2C`** (sin NSS):

```c
/* Después de HAL_Init(), SystemClock_Config(), MX_GPIO_Init() y MX_I2C1_Init() */
if (PN7160_Init(&hi2c1,
                NFC_VEN_GPIO_Port, NFC_VEN_Pin,
                NFC_IRQ_GPIO_Port, NFC_IRQ_Pin) != PN7160_OK)
{
    Error_Handler();
}
```

A partir de aquí el código es idéntico para ambos transportes:

```c
PN7160_Info_t info;
PN7160_GetInfo(&info);
printf("Modelo 0x%02X, firmware %02X.%02X.%02X\r\n",
       info.modelo, info.firmware[0], info.firmware[1], info.firmware[2]);
```

Tras `Init` el chip queda en **modo lector** y en reposo (`RFST_IDLE`), sin campo RF.

### 2. Leer el UID de un tag (bloqueante)

```c
PN7160_Tag_t tag;

while (1)
{
    PN7160_Status_t st = PN7160_WaitTag(&tag, 1000, 0);   /* 1 s, sin dejarlo activado */

    if (st == PN7160_OK)
    {
        printf("Tech 0x%02X, protocolo 0x%02X, UID:", tag.tecnologia, tag.protocolo);
        for (uint8_t i = 0; i < tag.uidLen; i++)
        {
            printf(" %02X", tag.uid[i]);
        }
        printf("  ATQA %02X %02X, SAK %02X\r\n", tag.atqa[0], tag.atqa[1], tag.sak);
        HAL_Delay(500);
    }
    else if (st != PN7160_TIMEOUT)
    {
        Error_Handler();
    }
}
```

Con `keepActive = 0`, `WaitTag` vuelve a `RFST_IDLE` antes de regresar: el siguiente `WaitTag` vuelve a detectar la misma tarjeta si sigue en el campo.

### 3. Intercambiar datos con un tag

Con `keepActive = 1` el tag queda activado para `PN7160_Transceive()`. El formato depende del protocolo (ver [`PN7160_Transceive()`](#pn7160_transceive---intercambio-de-datos)):

```c
PN7160_Tag_t tag;
uint8_t      rx[64];
uint16_t     rxLen;

if (PN7160_WaitTag(&tag, 5000, 1) == PN7160_OK)
{
    if (tag.protocolo == PN7160_PROT_T2T)            /* NTAG, Ultralight */
    {
        const uint8_t read[] = { 0x30, 0x04 };       /* READ desde la página 4 */
        rxLen = sizeof(rx);
        if (PN7160_Transceive(read, sizeof(read), rx, &rxLen) == PN7160_OK
            && rxLen == 17 && rx[16] == 0x00)        /* 16 bytes + estado NCI */
        {
            /* rx[0..15] = páginas 4 a 7 */
        }
    }
    else if (tag.protocolo == PN7160_PROT_ISODEP)    /* APDU completos */
    {
        const uint8_t selectPpse[] = { 0x00, 0xA4, 0x04, 0x00, 0x0E,
                                       '2','P','A','Y','.','S','Y','S','.','D','D','F','0','1', 0x00 };
        rxLen = sizeof(rx);
        if (PN7160_Transceive(selectPpse, sizeof(selectPpse), rx, &rxLen) == PN7160_OK)
        {
            /* rx termina con SW1 SW2 (90 00 = OK) */
        }
    }
    else if (tag.protocolo == PN7160_PROT_MIFARE)    /* MIFARE Classic: wrappers TAG-CMD */
    {
        const uint8_t claveA[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
        uint8_t bloque[16];
        if (PN7160_MfcAuth(1, 0, claveA) == PN7160_OK      /* sector 1 = bloques 4 a 7 */
            && PN7160_MfcRead(4, bloque) == PN7160_OK)
        {
            /* bloque[0..15] = bloque 4 */
        }
    }

    PN7160_ReleaseTag();                             /* Apaga el campo RF */
}
```

### 4. Esperar la retirada del tag

```c
if (PN7160_WaitTag(&tag, 5000, 1) == PN7160_OK)
{
    /* ... Transceive ... */

    if (PN7160_WaitTagRemoval(10000) == PN7160_TIMEOUT)
    {
        printf("El tag sigue en el campo\r\n");
    }
    /* En ambos casos el PN7160 queda en RFST_IDLE */
}
```

> [!WARNING]
> `PN7160_WaitTagRemoval()` solo funciona **después de `WaitTag(..., keepActive = 1)`**. Tras un `WaitTag` con `keepActive = 0` devuelve `PN7160_ERROR`.

### 5. Descubrimiento por interrupción

Requiere la [configuración EXTI](#configuración-exti-para-irq-modo-por-interrupción). El descubrimiento es de un solo disparo: se rearma desde el callback.

```c
/* USER CODE BEGIN 0 */
void PN7160_TagDetectedCallback(const PN7160_Tag_t* tag)
{
    printf("Tag con UID de %u bytes\r\n", tag->uidLen);
    /* Con keepActive = 1 aquí se puede llamar a PN7160_Transceive() */
    PN7160_StartDiscoveryIT(0);                      /* Rearma para el siguiente tag */
}
/* USER CODE END 0 */

/* Tras PN7160_Init() */
PN7160_StartDiscoveryIT(0);

while (1)
{
    if (PN7160_Process() != PN7160_OK)
    {
        PN7160_StartDiscoveryIT(0);                  /* Un error termina el descubrimiento */
    }
    /* ... resto del bucle principal ... */
}
```

> [!TIP]
> Si rearmas desde el callback con `keepActive = 0`, el mismo tag se volverá a detectar mientras siga en el campo. Añade un retardo o compara el UID con el anterior si solo quieres un evento por tarjeta.

### 6. Emulación de tarjeta

```c
const uint8_t uid[4] = { 0x12, 0x34, 0x56, 0x78 };
PN7160_SetCardUid(uid, sizeof(uid));                 /* Opcional: UID fijo */
PN7160_SetMode(PN7160_MODE_CARD);

PN7160_Tag_t lector;
uint8_t      capdu[261];
uint16_t     len;

if (PN7160_WaitTag(&lector, 10000, 1) == PN7160_OK)  /* Un lector externo nos activó */
{
    len = sizeof(capdu);
    PN7160_Status_t st = PN7160_Transceive(NULL, 0, capdu, &len);    /* Primer C-APDU */

    while (st == PN7160_OK)
    {
        const uint8_t rapdu[] = { 0x90, 0x00 };      /* Respuesta: OK */
        len = sizeof(capdu);
        st = PN7160_Transceive(rapdu, sizeof(rapdu), capdu, &len);   /* R-APDU -> siguiente C-APDU */
    }

    PN7160_ReleaseTag();                             /* El lector se fue o hubo error */
}

PN7160_SetMode(PN7160_MODE_READER);                  /* De vuelta a lector */
```

En modo tarjeta el orden de `Transceive` es inverso: con `txLen = 0` se recibe el primer C-APDU y en cada llamada siguiente `tx` es la R-APDU y `rx` el siguiente C-APDU.

---

## API Reference

### 1. Tipos de Datos

#### `PN7160_Status_t` - Estados de Retorno

```c
typedef enum {
    PN7160_OK               = 0,
    PN7160_ERROR            = 1,
    PN7160_TIMEOUT          = 2,
    PN7160_NOT_INITIALIZED  = 3,
    PN7160_INVALID_PARAM    = 4
} PN7160_Status_t;
```

| Valor | Código | Significado |
|-------|--------|-------------|
| `PN7160_OK` | 0 | Operación completada sin errores |
| `PN7160_ERROR` | 1 | Fallo del bus, respuesta NCI inesperada, model ID desconocido, tag ausente o buffer de recepción insuficiente |
| `PN7160_TIMEOUT` | 2 | IRQ no se activó a tiempo, el bus no respondió, no apareció ningún tag o timeout RF |
| `PN7160_NOT_INITIALIZED` | 3 | `PN7160_Init()` no se llamó o falló |
| `PN7160_INVALID_PARAM` | 4 | Puntero `NULL`, modo o longitud de UID no válidos |

#### `PN7160_Mode_t` - Modo RF

| Valor | Código | Descripción |
|-------|--------|-------------|
| `PN7160_MODE_READER` | 0 | Lector/escritor: sondea NFC-A/B/F/V. Modo por defecto tras `Init` |
| `PN7160_MODE_CARD` | 1 | Emulación de tarjeta T4T (ISO-DEP sobre NFC-A); el host responde los APDU |

#### `PN7160_Tag_t` - Datos del Tag

```c
typedef struct {
    uint8_t tecnologia;
    uint8_t protocolo;
    uint8_t uid[10];
    uint8_t uidLen;
    uint8_t atqa[2];
    uint8_t sak;
} PN7160_Tag_t;
```

| Campo | Descripción |
|-------|-------------|
| `tecnologia` | RF Tech & Mode: `PN7160_TEC_NFC_A` (`0x00`), `_NFC_B` (`0x01`), `_NFC_F` (`0x02`), `_NFC_V` (`0x06`); `PN7160_TEC_NFC_A_ESCUCHA` (`0x80`) en modo tarjeta |
| `protocolo` | `PN7160_PROT_T1T` (`0x01`), `_T2T` (`0x02`), `_T3T` (`0x03`), `_ISODEP` (`0x04`), `_NFCDEP` (`0x05`), `_T5T` (`0x06`), `_MIFARE` (`0x80`, MIFARE Classic) |
| `uid` | NFCID1 / NFCID0 / NFCID2 / UID según la tecnología, en el orden de NCI (NFC-V llega LSB primero) |
| `uidLen` | Bytes válidos de `uid`: 4/7/10 (A), 4 (B), 8 (F), 8 (V); 0 en modo tarjeta |
| `atqa` | SENS_RES de NFC-A en el orden de NCI (p. ej. `04 00` Classic 1K, `02 00` Classic 4K, `44 00` NTAG); `00 00` si no aplica |
| `sak` | SEL_RES de NFC-A (p. ej. `0x08` Classic 1K, `0x18` Classic 4K, `0x00` T2T, `0x20` ISO-DEP); 0 si no aplica (otra tecnología, T1T o modo tarjeta) |

#### `PN7160_Info_t` - Identificación del Chip

| Campo | Descripción |
|-------|-------------|
| `modelo` | Model ID: `0x61` PN7160, `0x71` PN7161 |
| `firmware[3]` | Versión de firmware, en el orden del `CORE_RESET_NTF` (bytes 10 a 12) |

#### Constantes

Todas se pueden redefinir antes de incluir `PN7160.h`.

| Macro | Valor | Descripción |
|-------|-------|-------------|
| `PN7160_TIMEOUT_MS` | 100 | Timeout del bus (SPI/I2C) y de la espera de IRQ por cada comando/respuesta, en ms |
| `PN7160_RF_TIMEOUT_MS` | 1000 | Espera de cada paquete de datos o de créditos en `PN7160_Transceive()`, en ms |
| `PN7160_REMOVAL_MS` | 600 | Ventana de `PN7160_WaitTagRemoval()`: si en ese tiempo no se reactiva nada, se da por retirado. Debe cubrir al menos un ciclo de sondeo |
| `PN7160_BOOT_MS` | 3 | Espera tras subir VEN (tboot = 2.5 ms máx.), en ms |
| `PN7160_I2C_ADDRESS` | `0x50` | Solo I2C. Dirección en formato HAL según HIF1/HIF2 (`0x50`/`0x52`/`0x54`/`0x56`) |
| `PN7160_I2C_RETRY_MS` | 10 | Solo I2C. Espera antes de reintentar una escritura sin ACK (despertar de *Standby*), en ms |
| `PN7160_CLOCK_SEL_CFG` | `0x08` | Valor de `CLOCK_SEL_CFG` (A0 03). `0x08` = cristal de 27.12 MHz. Para PLL con reloj externo, toma la codificación del UM11495 |

---

### 2. Funciones Públicas

#### `PN7160_Init()` - Inicialización

```c
PN7160_Status_t PN7160_Init(SPI_HandleTypeDef* hspi,
                            GPIO_TypeDef* nssPort, uint16_t nssPin,
                            GPIO_TypeDef* venPort, uint16_t venPin,
                            GPIO_TypeDef* irqPort, uint16_t irqPin);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `hspi` | `SPI_HandleTypeDef*` | Handle del SPI ya inicializado por CubeMX |
| `nssPort`, `nssPin` | `GPIO_TypeDef*`, `uint16_t` | Pin NSS (HIF1) |
| `venPort`, `venPin` | `GPIO_TypeDef*`, `uint16_t` | Pin VEN (reset / Hard Power Down) |
| `irqPort`, `irqPin` | `GPIO_TypeDef*`, `uint16_t` | Pin IRQ |

Secuencia: VEN en bajo ≥ 10 µs y en alto, espera `PN7160_BOOT_MS`, `CORE_RESET` (valida el model ID), `CORE_INIT`, activación de extensiones NXP, lectura de `CLOCK_SEL_CFG` (si difiere, lo escribe en EEPROM y repite el reset) y `RF_DISCOVER_MAP` en modo lector. Si ya estaba inicializado, desinicializa y vuelve a empezar.

**Retorna**: `PN7160_OK`; `PN7160_INVALID_PARAM` si `hspi` o algún puerto es `NULL`; `PN7160_TIMEOUT` si IRQ no se activó o el SPI no respondió; `PN7160_ERROR` si falla el bus, una respuesta NCI no es la esperada o el chip no es un PN7160/PN7161.

Con `PN7160_TRANSPORT_I2C` la firma no lleva NSS:

```c
PN7160_Status_t PN7160_Init(I2C_HandleTypeDef* hi2c,
                            GPIO_TypeDef* venPort, uint16_t venPin,
                            GPIO_TypeDef* irqPort, uint16_t irqPin);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `hi2c` | `I2C_HandleTypeDef*` | Handle del I2C ya inicializado por CubeMX |
| `venPort`, `venPin` | `GPIO_TypeDef*`, `uint16_t` | Pin VEN (reset / Hard Power Down) |
| `irqPort`, `irqPin` | `GPIO_TypeDef*`, `uint16_t` | Pin IRQ |

La secuencia NCI es la misma. Cambia solo el transporte: sin byte de dirección (la dirección `PN7160_I2C_ADDRESS` con R/W indica escritura o lectura), cada paquete se lee en dos lecturas (cabecera y payload) y una escritura sin ACK se reintenta una vez tras `PN7160_I2C_RETRY_MS`.

**Retorna**: igual que la versión SPI (`PN7160_INVALID_PARAM` si `hi2c` o algún puerto es `NULL`). `PN7160_ERROR` incluye el NACK en la dirección (dirección equivocada o chip sin alimentar).

---

#### `PN7160_DeInit()` - Desinicialización

```c
PN7160_Status_t PN7160_DeInit(void);
```

Pone VEN en bajo (*Hard Power Down*), cancela el descubrimiento por interrupción y olvida el handle del bus. No envía comandos NCI. El UID fijado con `PN7160_SetCardUid()` se conserva.

**Retorna**: `PN7160_OK`, o `PN7160_NOT_INITIALIZED` si no estaba inicializado.

---

#### `PN7160_GetInfo()` - Modelo y Firmware

```c
PN7160_Status_t PN7160_GetInfo(PN7160_Info_t* info);
```

Copia el model ID y la versión de firmware guardados del último `CORE_RESET_NTF`. No habla con el chip.

**Retorna**: `PN7160_OK`, `PN7160_INVALID_PARAM` si `info` es `NULL`, o `PN7160_NOT_INITIALIZED`.

---

#### `PN7160_SetMode()` - Lector o Emulación de Tarjeta

```c
PN7160_Status_t PN7160_SetMode(PN7160_Mode_t mode);
```

Reconfigura el discover map (y en modo tarjeta, el ruteo de ISO-DEP al host y `LA_SEL_INFO`, más `LA_NFCID1` si se fijó un UID). El modo se aplica en el siguiente `PN7160_WaitTag()` o `PN7160_StartDiscoveryIT()`. Desactiva cualquier tag activado y cancela el descubrimiento IT en curso.

**Retorna**: `PN7160_OK`; `PN7160_INVALID_PARAM` si `mode` no es válido; `PN7160_NOT_INITIALIZED`; `PN7160_TIMEOUT` o `PN7160_ERROR` ante fallos de bus o NCI. En error se conserva el modo anterior, aunque la configuración del chip puede quedar a medias.

---

#### `PN7160_SetCardUid()` - UID en Modo Tarjeta

```c
PN7160_Status_t PN7160_SetCardUid(const uint8_t* uid, uint8_t len);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `uid` | `const uint8_t*` | NFCID1 a anunciar, en el orden de NCI; puede ser `NULL` si `len = 0` |
| `len` | `uint8_t` | 4, 7 o 10 = UID fijo; 0 = UID dinámico generado por el PN7160 |

Guarda el UID y lo envía como `LA_NFCID1` en cada `PN7160_SetMode(PN7160_MODE_CARD)`. Si ya está en modo tarjeta, lo aplica en el acto. Si nunca se llama, no se envía `LA_NFCID1` y el chip usa su UID dinámico. No valida el contenido: según ISO/IEC 14443-3, un UID de 4 bytes que empieza por `0x08` se interpreta como aleatorio y `uid[0] = 0x88` (cascade tag) no es válido.

**Retorna**: `PN7160_OK`; `PN7160_INVALID_PARAM` si `len` no es 0/4/7/10 o `uid` es `NULL` con `len > 0`; `PN7160_NOT_INITIALIZED`; `PN7160_TIMEOUT` / `PN7160_ERROR` si estaba en modo tarjeta y falló al aplicarlo (el UID queda guardado y se reintenta en el siguiente `SetMode`).

---

#### `PN7160_WaitTag()` - Esperar un Tag

```c
PN7160_Status_t PN7160_WaitTag(PN7160_Tag_t* tag, uint32_t timeoutMs, uint8_t keepActive);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `tag` | `PN7160_Tag_t*` | Destino de los datos del tag |
| `timeoutMs` | `uint32_t` | Tiempo máximo de espera, en ms |
| `keepActive` | `uint8_t` | 0 = volver a `RFST_IDLE` al terminar; 1 = dejar el tag activado para `PN7160_Transceive()` |

En modo lector sondea NFC-A/B/F/V; en modo tarjeta espera a que un lector externo active al PN7160 (`tag->uidLen = 0`). Si hay varios tags, se queda con el primero. Si una llamada anterior dejó un tag activado, lo desactiva primero.

**Retorna**: `PN7160_OK` con `*tag` válido; `PN7160_INVALID_PARAM` si `tag` es `NULL`; `PN7160_NOT_INITIALIZED`; `PN7160_TIMEOUT` si no apareció ningún tag; `PN7160_ERROR` ante fallos de bus o NCI.

---

#### `PN7160_Transceive()` - Intercambio de Datos

```c
PN7160_Status_t PN7160_Transceive(const uint8_t* tx, uint16_t txLen, uint8_t* rx, uint16_t* rxLen);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `tx` | `const uint8_t*` | Datos a enviar; puede ser `NULL` si `txLen = 0` |
| `txLen` | `uint16_t` | Bytes a enviar; 0 = solo recibir |
| `rx` | `uint8_t*` | Buffer de recepción |
| `rxLen` | `uint16_t*` | Entrada: capacidad de `rx`. Salida: bytes guardados |

Requiere un tag/lector activado con `keepActive = 1`. El formato de los datos depende de la interfaz que eligió el discover map:

| Protocolo | Interfaz | Formato |
|-----------|----------|---------|
| ISO-DEP (`0x04`) | ISO-DEP | APDU completos, sin bytes extra |
| T2T, T5T, ... | Frame RF | `rx` termina con un **byte de estado NCI** (`0x00` = OK) que no viene del tag |
| MIFARE Classic (`0x80`) | TAG-CMD | Formato propietario NXP (`0x10` = intercambio de datos, `0x40` = autenticación; ver UM11495). Usar mejor [`PN7160_MfcAuth()` / `MfcRead()` / `MfcWrite()`](#pn7160_mfcauth--mfcread--mfcwrite---mifare-classic) |

**Retorna**: `PN7160_OK`; `PN7160_INVALID_PARAM` si `rx` o `rxLen` son `NULL`, o `tx` es `NULL` con `txLen > 0`; `PN7160_NOT_INITIALIZED`; `PN7160_TIMEOUT` si pasaron `PN7160_RF_TIMEOUT_MS` sin paquete o el chip reportó timeout RF; `PN7160_ERROR` si no hay tag activado, el tag/lector se fue, hubo otro error RF o la respuesta no cupo en `rx` (`*rxLen` = bytes que sí cupieron).

---

#### `PN7160_ReleaseTag()` - Liberar el Tag

```c
PN7160_Status_t PN7160_ReleaseTag(void);
```

Desactiva el tag/lector, apaga el campo RF y vuelve a `RFST_IDLE`. También cancela el descubrimiento por interrupción. Si ya estaba en reposo, no hace nada. Úsala tras el último `Transceive()` o después de un `Transceive()` que falló porque el tag se fue.

**Retorna**: `PN7160_OK`, `PN7160_NOT_INITIALIZED`, `PN7160_TIMEOUT` o `PN7160_ERROR`.

---

#### `PN7160_WaitTagRemoval()` - Esperar la Retirada

```c
PN7160_Status_t PN7160_WaitTagRemoval(uint32_t timeoutMs);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `timeoutMs` | `uint32_t` | Tiempo máximo de espera de la retirada, en ms (se revisa entre ciclos) |

No hace un *presence check* por protocolo: repite el descubrimiento hasta que en `PN7160_REMOVAL_MS` no se activa nada. Cuenta cualquier tag/lector, no solo el original, y el campo RF se apaga en cada ciclo, así que el tag pierde su estado (p. ej. la aplicación ISO-DEP seleccionada).

**Retorna**: `PN7160_OK` si se retiró; `PN7160_NOT_INITIALIZED`; `PN7160_TIMEOUT` si seguía en el campo; `PN7160_ERROR` si no había tag activado o falla el bus/NCI. En `OK` y `TIMEOUT` queda en `RFST_IDLE`.

---

#### `PN7160_MfcAuth()` / `MfcRead()` / `MfcWrite()` - MIFARE Classic

```c
PN7160_Status_t PN7160_MfcAuth(uint8_t sector, uint8_t keyB, const uint8_t key[6]);
PN7160_Status_t PN7160_MfcRead(uint8_t block, uint8_t out[16]);
PN7160_Status_t PN7160_MfcWrite(uint8_t block, const uint8_t data[16]);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `sector` | `uint8_t` | Sector a autenticar: 0-15 (1K) o 0-39 (4K) |
| `keyB` | `uint8_t` | 0 = clave A; 1 = clave B |
| `key` | `const uint8_t[6]` | Clave de 6 bytes (de fábrica `FF FF FF FF FF FF`) |
| `block` | `uint8_t` | Bloque absoluto: 0-63 (1K) o 0-255 (4K) |
| `out` / `data` | `uint8_t[16]` | Bloque leído / a escribir |

Envuelven el formato TAG-CMD propietario de NXP sobre `PN7160_Transceive()`; el PN7160 hace Crypto1 y el CRC. Requieren un tag `PN7160_PROT_MIFARE` activado con `keepActive = 1`; con otro protocolo devuelven `PN7160_ERROR` sin mandar nada por RF.

| Función | TAG-CMD enviado | Respuesta esperada |
|---------|-----------------|--------------------|
| `MfcAuth` | `40 <sector> <10 \| 90> <clave[6]>` (clave A / B embebida) | `40 00` |
| `MfcRead` | `10 30 <bloque>` | `10 <16 bytes> 00` |
| `MfcWrite` | `10 A0 <bloque>`, después `10 <16 bytes>` | `10 … 00` en cada fase |

- En 1K el sector `s` ocupa los bloques `4s` a `4s+3`; en 4K los sectores 32-39 tienen 16 bloques. El último bloque de cada sector es el *trailer* (claves y access bits).
- Si la clave es incorrecta, la tarjeta pasa a HALT: hay que volver a activarla con `PN7160_WaitTag()` antes de probar otra clave.
- **Cuidado con `MfcWrite` en un trailer**: unos access bits mal formados dejan el sector inaccesible para siempre. El bloque 0 es de solo lectura en tarjetas genuinas.

**Retorna**: `PN7160_OK`; `PN7160_INVALID_PARAM` si un puntero es `NULL`, `sector > 39` o `keyB > 1`; `PN7160_NOT_INITIALIZED`; `PN7160_TIMEOUT` como `Transceive()`; `PN7160_ERROR` si no hay tag MIFARE Classic activado, la clave es incorrecta, el sector no está autenticado, los access bits lo impiden o falla el bus.

---

### 3. Descubrimiento por Interrupción

#### `PN7160_StartDiscoveryIT()` - Arrancar el Descubrimiento

```c
PN7160_Status_t PN7160_StartDiscoveryIT(uint8_t keepActive);
```

Versión no bloqueante de `PN7160_WaitTag()`, sin timeout: arranca el descubrimiento y regresa. El tag se entrega en `PN7160_TagDetectedCallback()`, llamado desde `PN7160_Process()`. Es de un solo disparo y se cancela con `PN7160_ReleaseTag()`, `PN7160_SetMode()` o `PN7160_WaitTag()`.

`keepActive` = 1 deja el tag activado para usar `PN7160_Transceive()` (incluso dentro del callback) hasta `PN7160_ReleaseTag()` o el siguiente descubrimiento.

**Retorna**: `PN7160_OK`, `PN7160_NOT_INITIALIZED`, `PN7160_TIMEOUT` o `PN7160_ERROR`.

---

#### `PN7160_IRQ_Handler()` - Manejador de Interrupción

```c
void PN7160_IRQ_Handler(void);
```

Se llama desde `HAL_GPIO_EXTI_Callback()`. Solo activa una bandera; no usa el bus (SPI/I2C).

> [!WARNING]
> Se ejecuta en contexto de **interrupción**. Todo el acceso al bus lo hace `PN7160_Process()` en el bucle principal.

---

#### `PN7160_Process()` - Atender el Descubrimiento

```c
PN7160_Status_t PN7160_Process(void);
```

Llamar en el bucle principal, **nunca desde una ISR**. Si no hubo interrupción regresa de inmediato; si la hubo, lee los paquetes pendientes y, al activarse un tag, cierra el descubrimiento y llama al callback. Solo bloquea lo que dura un intercambio comando/respuesta (`PN7160_TIMEOUT_MS` como máximo).

**Retorna**: `PN7160_OK` si no había nada que hacer, el descubrimiento sigue o se entregó un tag; `PN7160_NOT_INITIALIZED`; `PN7160_TIMEOUT` o `PN7160_ERROR` si falló el bus o una NTF no es válida. En error el descubrimiento termina sin callback: vuelve a llamar a `PN7160_StartDiscoveryIT()`.

---

#### `PN7160_TagDetectedCallback()` - Callback de Tag Detectado

```c
void PN7160_TagDetectedCallback(const PN7160_Tag_t* tag);
```

Definida como `__weak` en la librería; redefínela en la aplicación. Se ejecuta dentro de `PN7160_Process()` (no en la ISR), así que puede llamar a `PN7160_Transceive()`, `PN7160_ReleaseTag()` o `PN7160_StartDiscoveryIT()`. El puntero `tag` es válido hasta el siguiente `PN7160_StartDiscoveryIT()`.

---

## Limitaciones conocidas

- **Una sola instancia**: el estado vive en variables `static` del `.c`, así que solo se maneja un PN7160 por firmware.
- **Un transporte por firmware**: SPI e I2C conviven en el mismo código fuente, pero se compila uno. Tener un PN7160 por SPI y otro por I2C a la vez exigiría pasar el estado a un handle por instancia (`PN7160_Handle_t*` en cada función) y separar el callback `__weak`; no está implementado.
- **Un tag a la vez**: con varios tags en el campo se toma el primero; no hay API para recorrer el resto.
- **Sin NDEF**: no interpreta mensajes NDEF; se construyen sobre `PN7160_Transceive()`.
- **Sin P2P**: el datasheet indica que el soporte P2P solo existe hasta el firmware 12.50.0A.
- **TXLDO / RF sin configurar**: `Init` solo escribe `CLOCK_SEL_CFG`; la configuración de alimentación del transmisor y RF queda como venga en la EEPROM del módulo.
- **MIFARE Classic sin *value blocks***: no hay operaciones de *value block* (increment/decrement/restore/transfer) ni claves guardadas en la EEPROM del PN7160; se pueden enviar con `PN7160_Transceive()`.
- **`PN7160_SetCardUid()` sin validar en hardware**: el envío de `LA_NFCID1` (sobre todo con longitud 0) es NCI 2.0 estándar pero el datasheet no lo documenta.
- `PN7160_Transceive()` y `PN7160_WaitTagRemoval()` son siempre bloqueantes; solo el descubrimiento tiene versión por interrupción.

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](../../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [1.0.0] - 25-11-2026

Primera versión del driver del controlador NFC PN7160/PN7161 por SPI o I2C (NCI 2.0).

#### Added
- **Inicialización NCI**: `PN7160_Init()` con reset por VEN, `CORE_RESET`/`CORE_INIT`, extensiones NXP, `CLOCK_SEL_CFG` configurable y discover map; `PN7160_DeInit()` deja el chip en *Hard Power Down*.
- **Identificación del chip**: `PN7160_GetInfo()` devuelve el model ID (PN7160/PN7161) y la versión de firmware.
- **Detección de tags**: `PN7160_WaitTag()` para NFC-A/B/F/V con tecnología, protocolo, UID, ATQA y SAK.
- **Intercambio de datos**: `PN7160_Transceive()` con segmentación, control de créditos y protección del buffer de recepción; `PN7160_ReleaseTag()` apaga el campo RF.
- **Emulación de tarjeta T4T**: `PN7160_SetMode()` entre lector y tarjeta, y `PN7160_SetCardUid()` para fijar el NFCID1 anunciado.
- **MIFARE Classic**: `PN7160_MfcAuth()`, `PN7160_MfcRead()` y `PN7160_MfcWrite()` encapsulan el formato TAG-CMD propietario de NXP (clave A/B embebida, bloques de 16 bytes).
- **Retirada del tag**: `PN7160_WaitTagRemoval()` espera a que el tag o el lector salga del campo.
- **Transporte I2C o SPI**: I2C por defecto; `PN7160_TRANSPORT_SPI` compila el transporte SPI (`#error` si se definen ambos); `PN7160_I2C_ADDRESS` y `PN7160_I2C_RETRY_MS` para I2C.
- **Descubrimiento por interrupción**: `PN7160_StartDiscoveryIT()`, `PN7160_IRQ_Handler()`, `PN7160_Process()` y el callback `__weak` `PN7160_TagDetectedCallback()`.

---
