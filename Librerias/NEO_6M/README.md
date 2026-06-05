# Librería para el módulo NEO-6M en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-1.1.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/NEO_6M)
[![Protocol](https://img.shields.io/badge/Protocol-UART-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/NEO_6M)

---

## Tabla de Contenidos
- [Librería para el módulo NEO-6M en STM32](#librería-para-el-módulo-neo-6m-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
  - [Configuración UART](#configuración-uart)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Integrar el Callback UART](#2-integrar-el-callback-uart)
    - [3. Obtener Datos GPS](#3-obtener-datos-gps)
    - [4. Convertir a DMS](#4-convertir-a-dms)
    - [5. Convertir a UTM](#5-convertir-a-utm)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`NEO6M_GPS_Status_t` - Estados de Retorno](#neo6m_gps_status_t---estados-de-retorno)
      - [`NEO6M_GPS_Data_t` - Estructura de Datos](#neo6m_gps_data_t---estructura-de-datos)
      - [`NEO6M_GPS_DMS_t` - Coordenadas en Formato DMS](#neo6m_gps_dms_t---coordenadas-en-formato-dms)
      - [`NEO6M_GPS_UTM_t` - Coordenadas en Formato UTM](#neo6m_gps_utm_t---coordenadas-en-formato-utm)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`NEO6M_GPS_Init()` - Inicialización del Driver](#neo6m_gps_init---inicialización-del-driver)
      - [`NEO6M_GPS_DeInit()` - Desinicialización del Driver](#neo6m_gps_deinit---desinicialización-del-driver)
      - [`NEO6M_GPS_Get()` - Lectura de Datos GPS](#neo6m_gps_get---lectura-de-datos-gps)
      - [`NEO6M_GPS_DecimalToDMS()` - Conversión a Grados/Minutos/Segundos](#neo6m_gps_decimaltodms---conversión-a-gradosminutossegundos)
      - [`NEO6M_GPS_DecimalToUTM()` - Conversión a Proyección UTM](#neo6m_gps_decimaltoutm---conversión-a-proyección-utm)
      - [`NEO6M_GPS_UART_RxCpltCallback()` - Callback de Recepción UART](#neo6m_gps_uart_rxcpltcallback---callback-de-recepción-uart)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.1.0\] - 05-06-2026](#110---05-06-2026)
      - [Added](#added)
      - [Fixed](#fixed)
      - [Changed](#changed)
    - [\[1.0.0\] - 01-06-2026](#100---01-06-2026)
      - [Added](#added-1)

---

## Descripción

Librería desarrollada en C para la interfaz con el módulo GPS **u-blox NEO-6M** utilizando microcontroladores STM32. Proporciona funciones para recibir y parsear sentencias NMEA estándar (`$GPGGA`, `$GPGLL`, `$GPRMC`) a través de UART mediante recepción controlada por interrupciones, convirtiendo las coordenadas del formato NMEA (DDmm.mmmm) a grados decimales. La librería está diseñada para ser fácil de usar, eficiente y compatible con la mayoría de las series STM32 (F1, F4, etc.) utilizando HAL.

Ideal para aplicaciones de geolocalización, rastreo GPS, drones y sistemas de navegación embebida.

---

## Características

- **Comunicación UART por interrupciones**: Recepción byte a byte sin bloquear el CPU, controlada por interrupciones HAL.
- **Soporte de sentencias NMEA**: Parseo de `$GPGGA` (fix, satélites, altitud, hora), `$GPGLL` (posición y hora) y `$GPRMC` (posición, estado y hora).
- **Validación de checksum**: Verificación XOR del checksum NMEA antes de procesar cualquier sentencia, descartando datos corruptos.
- **Conversión automática de coordenadas**: Convierte del formato NMEA (DDmm.mmmm) a grados decimales con signo (negativo para hemisferios S y W).
- **Datos UTC completos**: Extrae hora, minutos, segundos y milisegundos del campo de tiempo NMEA.
- **Hora local estimada**: Calcula automáticamente la hora local (`localHour`) a partir de la hora UTC y la longitud GPS.
- **Conversión a DMS**: Convierte coordenadas decimales al formato Grados/Minutos/Segundos (`NEO6M_GPS_DecimalToDMS()`).
- **Conversión a UTM**: Proyección Universal Transversa de Mercator completa con soporte de excepciones de zona Noruega/Svalbard (`NEO6M_GPS_DecimalToUTM()`).
- **Manejo robusto de errores**: Códigos de retorno específicos para parámetro inválido, timeout o módulo no inicializado (`NEO6M_GPS_Status_t`).
- **Portabilidad**: Compatible con múltiples familias STM32 mediante la capa HAL. Solo requiere cambiar el `#include` del encabezado HAL en `NEO_6M.h`.
- **Instancia única**: El driver gestiona un único módulo NEO-6M por proyecto.

---

## Pinout y Conexiones

### Pines requeridos

| Pin NEO-6M | Dirección | Descripción | Tipo GPIO | Observaciones |
|------------|-----------|-------------|-----------|---------------|
| **VCC**    | Alimentación | 3.3V a 5V | N/A | Los módulos comerciales incluyen regulador LDO — verificar la hoja de datos del módulo |
| **GND**    | Tierra    | — | N/A | — |
| **TX**     | Output    | Transmisión NMEA hacia STM32 | `UART_RX` | Conectar al pin RX del periférico UART del STM32 |
| **RX**     | Input     | Recepción de comandos desde STM32 | `UART_TX` | Opcional — solo necesario si se envían comandos de configuración al módulo |

> [!NOTE]
> La mayoría de los módulos comerciales NEO-6M integran un regulador de voltaje LDO y operan con alimentación de 5V y niveles lógicos de 3.3V. Verificar la hoja de datos específica del módulo antes de conectar.

> [!WARNING]
> El módulo NEO-6M requiere una antena externa activa o pasiva para recibir señal satelital. Sin antena, el módulo no generará sentencias de posición válidas.

---

## Configuración UART

Configura tu periférico UART en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | Asynchronous | — |
| **Baud Rate** | 9600 | Velocidad por defecto del NEO-6M |
| **Word Length** | 8 Bits | — |
| **Stop Bits** | 1 | — |
| **Parity** | None | 8N1 |
| **UART Global Interrupt** | Enabled | **Requerido** para la recepción por interrupciones |

> [!IMPORTANT]
> Habilitar la interrupción global del periférico UART en la pestaña **NVIC Settings** de CubeMX es obligatorio. Sin ella, `HAL_UART_Receive_IT()` no generará el callback `HAL_UART_RxCpltCallback()` y el driver no recibirá datos.

---

## Instalación

1. Copia `NEO_6M.c` y `NEO_6M.h` a tu proyecto (ej: `Librerias/NEO_6M/`).
2. Incluye la librería en tu `main.c` o archivo principal:
   ```c
   #include "NEO_6M.h"
   ```
3. Configura UART y GPIOs en CubeMX (ver sección anterior).
4. Genera código y compila.

---

## Uso Básico

### 1. Inicialización

```c
// En main() después de HAL_Init() y MX_USARTx_UART_Init()
NEO6M_GPS_Status_t status = NEO6M_GPS_Init(&huart1);

if (status != NEO6M_GPS_OK) {
    Error_Handler();  // Handle NULL — verificar el handle UART
}
```

### 2. Integrar el Callback UART

Dentro del bloque `/* USER CODE BEGIN 4 */` en `main.c`, implementa:

```c
/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    NEO6M_GPS_UART_RxCpltCallback(huart);
}
/* USER CODE END 4 */
```

> [!IMPORTANT]
> Esta llamada en el callback es **obligatoria** para que el driver acumule los bytes NMEA. Sin ella, `NEO6M_GPS_Get()` siempre devolverá datos en cero.

> [!NOTE]
> La librería rearma automáticamente la interrupción UART byte a byte dentro de `NEO6M_GPS_UART_RxCpltCallback()`. No es necesario volver a llamar a `HAL_UART_Receive_IT()` manualmente.

### 3. Obtener Datos GPS

```c
NEO6M_GPS_Data_t gps;

if (NEO6M_GPS_Get(&gps) == NEO6M_GPS_OK) {
    // Coordenadas en grados decimales con signo
    // gps.latitude  / gps.northSouth  →  ej. -19.4326, 'S'  (negativo si hemisferio S)
    // gps.longitude / gps.eastWest    →  ej. -99.1332, 'W'  (negativo si hemisferio W)
    // gps.altitude                    →  ej. 2240.0 m
    // gps.hours / gps.minutes / gps.seconds / gps.milliseconds → hora UTC
    // gps.localHour                   →  hora local estimada por longitud
    // gps.fixQuality                  →  0 = sin fix, 1 = GPS fix, 2 = DGPS
    // gps.numSatellites               →  número de satélites en vista
    // gps.posStatus                   →  'A' = válido, 'V' = vacío (GPRMC)
}
```

### 4. Convertir a DMS

```c
NEO6M_GPS_DMS_t dmsLat, dmsLon;

NEO6M_GPS_DecimalToDMS(gps.latitude,  gps.northSouth, &dmsLat);
NEO6M_GPS_DecimalToDMS(gps.longitude, gps.eastWest,   &dmsLon);
// dmsLat  →  ej. 19° 25' 57" N
// dmsLon  →  ej. 99° 7' 59" W
```

### 5. Convertir a UTM

```c
NEO6M_GPS_UTM_t utm;

if (NEO6M_GPS_DecimalToUTM(gps.latitude, gps.longitude, &utm) == NEO6M_GPS_OK) {
    // utm.zoneNumber  →  ej. 14
    // utm.zone        →  ej. 'Q'
    // utm.easting     →  ej. 486239 m
    // utm.northing    →  ej. 2150619 m
}
```

---

## API Reference

### 1. Tipos de Datos

#### `NEO6M_GPS_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería, permitiendo una gestión robusta de errores y estados del módulo.

```c
typedef enum {
    NEO6M_GPS_OK              = 0,  /**< Operación exitosa */
    NEO6M_GPS_ERROR           = 1,  /**< Error en la operación */
    NEO6M_TIMEOUT             = 2,  /**< Timeout en la operación */
    NEO6M_GPS_NOT_INITIALIZED = 3,  /**< Módulo no inicializado */
    NEO6M_GPS_INVALID_PARAM   = 4   /**< Parámetro inválido */
} NEO6M_GPS_Status_t;
```

| Valor | Código | Significado |
|-------|--------|-------------|
| `NEO6M_GPS_OK` | 0 | Operación completada sin errores |
| `NEO6M_GPS_ERROR` | 1 | Error en la operación |
| `NEO6M_TIMEOUT` | 2 | Timeout en la operación |
| `NEO6M_GPS_NOT_INITIALIZED` | 3 | `NEO6M_GPS_Init()` no se llamó o falló |
| `NEO6M_GPS_INVALID_PARAM` | 4 | Parámetro nulo o inválido |

---

#### `NEO6M_GPS_Data_t` - Estructura de Datos

Estructura para almacenar el último fix GPS parseado.

```c
typedef struct {
    double   latitude;        /**< Latitud en grados decimales con signo   */
    double   longitude;       /**< Longitud en grados decimales con signo  */
    char     northSouth;      /**< Hemisferio: 'N' o 'S'                   */
    char     eastWest;        /**< Hemisferio: 'E' o 'W'                   */
    uint8_t  hours;           /**< Hora UTC (GPGGA/GPRMC)                  */
    uint8_t  minutes;         /**< Minutos UTC (GPGGA/GPRMC)               */
    uint8_t  seconds;         /**< Segundos UTC (GPGGA/GPRMC)              */
    uint16_t milliseconds;    /**< Milisegundos UTC (GPGGA/GPRMC)          */
    uint8_t  localHour;       /**< Hora local estimada a partir de UTC y longitud */
    char     posStatus;       /**< Estado GPRMC: 'A' = válido, 'V' = vacío */
    float    altitude;        /**< Altitud en metros (GPGGA)               */
    uint8_t  fixQuality;      /**< Calidad del fix (GPGGA)                 */
    uint8_t  numSatellites;   /**< Número de satélites en vista (GPGGA)    */
} NEO6M_GPS_Data_t;
```

| Campo | Tipo | Rango | Unidad | Descripción |
|-------|------|-------|--------|-------------|
| `latitude` | `double` | -90.0 – +90.0 | ° | Latitud en grados decimales (negativo = Sur) |
| `longitude` | `double` | -180.0 – +180.0 | ° | Longitud en grados decimales (negativo = Oeste) |
| `northSouth` | `char` | 'N' / 'S' | — | Hemisferio norte o sur |
| `eastWest` | `char` | 'E' / 'W' | — | Hemisferio este u oeste |
| `hours` | `uint8_t` | 0 – 23 | h | Hora UTC |
| `minutes` | `uint8_t` | 0 – 59 | min | Minutos UTC |
| `seconds` | `uint8_t` | 0 – 59 | s | Segundos UTC |
| `milliseconds` | `uint16_t` | 0 – 990 | ms | Milisegundos UTC (resolución 10 ms) |
| `localHour` | `uint8_t` | 0 – 23 | h | Hora local estimada por longitud (huso ≈ longitud / 15°) |
| `posStatus` | `char` | 'A' / 'V' | — | Estado de posición GPRMC |
| `altitude` | `float` | -500 – +9000 | m | Altitud sobre el nivel del mar (GPGGA) |
| `fixQuality` | `uint8_t` | 0 – 2 | — | 0 = sin fix, 1 = GPS fix, 2 = DGPS (GPGGA) |
| `numSatellites` | `uint8_t` | 0 – 12 | — | Número de satélites en vista (GPGGA) |

---

#### `NEO6M_GPS_DMS_t` - Coordenadas en Formato DMS

Estructura para almacenar una coordenada en formato Grados, Minutos y Segundos.

```c
typedef struct {
    uint8_t degrees;    /**< Grados enteros de la coordenada                */
    uint8_t minutes;    /**< Minutos enteros de la coordenada               */
    uint8_t seconds;    /**< Segundos enteros de la coordenada              */
    char    hemisphere; /**< Hemisferio: 'N', 'S', 'E' o 'W'               */
} NEO6M_GPS_DMS_t;
```

| Campo | Tipo | Descripción |
|-------|------|-------------|
| `degrees` | `uint8_t` | Grados enteros |
| `minutes` | `uint8_t` | Minutos enteros (0 – 59) |
| `seconds` | `uint8_t` | Segundos enteros (0 – 59) |
| `hemisphere` | `char` | Hemisferio: `'N'`, `'S'`, `'E'` o `'W'` |

---

#### `NEO6M_GPS_UTM_t` - Coordenadas en Formato UTM

Estructura para almacenar una coordenada en la proyección Universal Transversa de Mercator.

```c
typedef struct {
    uint32_t easting;    /**< Coordenada Este en metros                     */
    uint32_t northing;   /**< Coordenada Norte en metros                    */
    char     zone;       /**< Letra de banda de latitud UTM (ej: 'T')       */
    uint16_t zoneNumber; /**< Número de zona UTM (ej: 30)                   */
} NEO6M_GPS_UTM_t;
```

| Campo | Tipo | Descripción |
|-------|------|-------------|
| `easting` | `uint32_t` | Coordenada Este en metros (incluye falso este de 500 000 m) |
| `northing` | `uint32_t` | Coordenada Norte en metros (incluye falso norte de 10 000 000 m en hemisferio sur) |
| `zone` | `char` | Letra de banda de latitud (C – X, sin I ni O) |
| `zoneNumber` | `uint16_t` | Número de zona UTM (1 – 60) |

---

### 2. Funciones Públicas

#### `NEO6M_GPS_Init()` - Inicialización del Driver

Inicializa el driver NEO6M_GPS, limpia los buffers internos y habilita la recepción por interrupciones UART. Todas las demás funciones retornan `NEO6M_GPS_NOT_INITIALIZED` si esta función no se ha llamado exitosamente.

```c
NEO6M_GPS_Status_t NEO6M_GPS_Init(UART_HandleTypeDef* huart);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `huart` | `UART_HandleTypeDef*` | Puntero al handle del periférico UART configurado |

**Retorna**: `NEO6M_GPS_OK` si la inicialización fue exitosa, `NEO6M_GPS_INVALID_PARAM` si `huart` es NULL.

**Secuencia interna:**

1. Validación del parámetro `huart`.
2. Limpieza del buffer de recepción y de la estructura de datos GPS.
3. Habilitación de la recepción UART por interrupciones (`HAL_UART_Receive_IT`, 1 byte).
4. Activación de la bandera de inicialización.

---

#### `NEO6M_GPS_DeInit()` - Desinicialización del Driver

Desinicializa el driver NEO6M_GPS, aborta la recepción UART por interrupciones (`HAL_UART_AbortReceive_IT`) y libera el handle UART interno.

```c
NEO6M_GPS_Status_t NEO6M_GPS_DeInit(void);
```

**Retorna**: Siempre retorna `NEO6M_GPS_OK`.

---

#### `NEO6M_GPS_Get()` - Lectura de Datos GPS

Copia los últimos datos GPS parseados a la estructura del llamador. Los datos reflejan la última sentencia NMEA válida recibida.

```c
NEO6M_GPS_Status_t NEO6M_GPS_Get(NEO6M_GPS_Data_t* data);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `data` | `NEO6M_GPS_Data_t*` | Puntero a la estructura donde se copiarán los datos GPS | N/A |

**Retorna**: `NEO6M_GPS_OK` si los datos fueron copiados correctamente, `NEO6M_GPS_NOT_INITIALIZED` si el driver no está inicializado, `NEO6M_GPS_INVALID_PARAM` si `data` es NULL.

---

#### `NEO6M_GPS_DecimalToDMS()` - Conversión a Grados/Minutos/Segundos

Convierte una coordenada en grados decimales al formato DMS (Grados, Minutos, Segundos).

```c
NEO6M_GPS_Status_t NEO6M_GPS_DecimalToDMS(double decimalDegrees, char hemisphere, NEO6M_GPS_DMS_t* dms);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `decimalDegrees` | `double` | Coordenada en grados decimales |
| `hemisphere` | `char` | Hemisferio de la coordenada: `'N'`, `'S'`, `'E'` o `'W'` |
| `dms` | `NEO6M_GPS_DMS_t*` | Puntero a la estructura donde se escribirá el resultado |

**Retorna**: `NEO6M_GPS_OK` si la conversión fue exitosa, `NEO6M_GPS_INVALID_PARAM` si `dms` es NULL.

---

#### `NEO6M_GPS_DecimalToUTM()` - Conversión a Proyección UTM

Convierte latitud y longitud en grados decimales a la proyección Universal Transversa de Mercator (UTM) usando el elipsoide WGS84. Incluye las excepciones de zona para Noruega y Svalbard.

```c
NEO6M_GPS_Status_t NEO6M_GPS_DecimalToUTM(double latitude, double longitude, NEO6M_GPS_UTM_t* utm);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `latitude` | `double` | Latitud en grados decimales | -80.0 – +84.0 |
| `longitude` | `double` | Longitud en grados decimales | -180.0 – +180.0 |
| `utm` | `NEO6M_GPS_UTM_t*` | Puntero a la estructura donde se escribirá el resultado | N/A |

**Retorna**: `NEO6M_GPS_OK` si la conversión fue exitosa, `NEO6M_GPS_INVALID_PARAM` si `utm` es NULL o las coordenadas están fuera de rango.

> [!NOTE]
> Requiere que el proyecto esté enlazado con la biblioteca matemática (`-lm`). En STM32CubeIDE esto es habitual; si se usa un Makefile personalizado, añadir la flag al linker.

---

#### `NEO6M_GPS_UART_RxCpltCallback()` - Callback de Recepción UART

Manejador de recepción UART — debe llamarse desde `HAL_UART_RxCpltCallback()` en `main.c`. Acumula bytes en el buffer interno hasta detectar un salto de línea (`\n`), momento en que valida el checksum NMEA y, si es correcto, parsea la sentencia para actualizar la estructura `NEO6M_GPS_Data_t`. Rearma la interrupción automáticamente.

```c
void NEO6M_GPS_UART_RxCpltCallback(UART_HandleTypeDef* huart);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `huart` | `UART_HandleTypeDef*` | Handle UART pasado por el callback HAL |

> [!WARNING]
> Esta función se ejecuta en contexto de **interrupción (ISR)**. No agregar código bloqueante ni llamadas HAL que no sean seguras en ISR dentro de `HAL_UART_RxCpltCallback()`.

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](../../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.  
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [1.1.0] - 05-06-2026

#### Added

- Nuevo campo `localHour` en `NEO6M_GPS_Data_t`: hora local estimada automáticamente a partir de la hora UTC y la longitud GPS (huso horario ≈ longitud / 15°).
- Nueva estructura `NEO6M_GPS_DMS_t` para almacenar coordenadas en formato Grados/Minutos/Segundos.
- Nueva estructura `NEO6M_GPS_UTM_t` para almacenar coordenadas en la proyección Universal Transversa de Mercator (WGS84).
- Nueva función pública `NEO6M_GPS_DecimalToDMS()`: convierte grados decimales a formato DMS con hemisferio.
- Nueva función pública `NEO6M_GPS_DecimalToUTM()`: convierte latitud/longitud a UTM con soporte de excepciones de zona Noruega/Svalbard.
- Documentación Doxygen completa (`@brief`, `@param`, `@return`, `@details`) en todas las funciones públicas y privadas.

#### Fixed

- Las coordenadas `latitude` y `longitude` ahora llevan signo: negativo para hemisferios Sur (`'S'`) y Oeste (`'W'`), positivo para Norte (`'N'`) y Este (`'E'`).
- Corrección del cálculo de huso horario en `UTCtoLocalTime()`: se usa el valor con signo de `longitude` para determinar el desfase correcto en ambos hemisferios.

#### Changed

- Los includes `<stdio.h>`, `<stdlib.h>` y `<string.h>` se movieron de `NEO_6M.c` a `NEO_6M.h`.
- Se añadió `<math.h>` en `NEO_6M.h` para soporte de `fabs()`, `sin()`, `cos()`, `tan()` y `sqrt()` requeridos por las nuevas funciones de conversión.

---

### [1.0.0] - 01-06-2026

#### Added

- Implementación inicial del driver NEO6M_GPS para STM32 con interfaz UART por interrupciones.
- Recepción byte a byte controlada por interrupciones HAL (`HAL_UART_Receive_IT`).
- Validación de checksum XOR de sentencias NMEA antes de parsear.
- Parseo de sentencias `$GPGGA`, `$GPGLL` y `$GPRMC`.
- Conversión automática de coordenadas del formato NMEA (DDmm.mmmm) a grados decimales.
- Extracción de tiempo UTC completo: horas, minutos, segundos y milisegundos.
- Funciones públicas: `NEO6M_GPS_Init()`, `NEO6M_GPS_DeInit()`, `NEO6M_GPS_Get()` y `NEO6M_GPS_UART_RxCpltCallback()`.
- Definición de tipos: `NEO6M_GPS_Status_t` y `NEO6M_GPS_Data_t`.
- Manejo de errores: parámetro inválido, timeout y módulo no inicializado.
