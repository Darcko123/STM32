# Librería para el módulo NEO-6M en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-2.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/NEO_6M)
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
    - [6. Distancia y Rumbo a un Destino](#6-distancia-y-rumbo-a-un-destino)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`NEO_6M_Status_t` - Estados de Retorno](#neo_6m_status_t---estados-de-retorno)
      - [`NEO_6M_Data_t` - Estructura de Datos](#neo_6m_data_t---estructura-de-datos)
      - [`NEO_6M_DMS_t` - Coordenadas en Formato DMS](#neo_6m_dms_t---coordenadas-en-formato-dms)
      - [`NEO_6M_UTM_t` - Coordenadas en Formato UTM](#neo_6m_utm_t---coordenadas-en-formato-utm)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`NEO_6M_Init()` - Inicialización del Driver](#neo_6m_init---inicialización-del-driver)
      - [`NEO_6M_DeInit()` - Desinicialización del Driver](#neo_6m_deinit---desinicialización-del-driver)
      - [`NEO_6M_Get()` - Lectura de Datos GPS](#neo_6m_get---lectura-de-datos-gps)
      - [`NEO_6M_DecimalToDMS()` - Conversión a Grados/Minutos/Segundos](#neo_6m_decimaltodms---conversión-a-gradosminutossegundos)
      - [`NEO_6M_DecimalToUTM()` - Conversión a Proyección UTM](#neo_6m_decimaltoutm---conversión-a-proyección-utm)
      - [`NEO_6M_Distance()` - Distancia entre Dos Puntos](#neo_6m_distance---distancia-entre-dos-puntos)
      - [`NEO_6M_Bearing()` - Rumbo entre Dos Puntos](#neo_6m_bearing---rumbo-entre-dos-puntos)
      - [`NEO_6M_UART_RxCpltCallback()` - Callback de Recepción UART](#neo_6m_uart_rxcpltcallback---callback-de-recepción-uart)
      - [`NEO_6M_UART_ErrorCallback()` - Callback de Error UART](#neo_6m_uart_errorcallback---callback-de-error-uart)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[2.0.0\] - 11-11-2026](#200---11-11-2026)
      - [Added](#added)
      - [Changed](#changed)
      - [Fixed](#fixed)
    - [\[1.1.0\] - 05-06-2026](#110---05-06-2026)
      - [Added](#added-1)
      - [Fixed](#fixed-1)
      - [Changed](#changed-1)
    - [\[1.0.0\] - 01-06-2026](#100---01-06-2026)
      - [Added](#added-2)

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
- **Conversión a DMS**: Convierte coordenadas decimales al formato Grados/Minutos/Segundos (`NEO_6M_DecimalToDMS()`).
- **Conversión a UTM**: Proyección Universal Transversa de Mercator completa con soporte de excepciones de zona Noruega/Svalbard (`NEO_6M_DecimalToUTM()`).
- **Distancia y rumbo**: Distancia sobre la superficie terrestre (haversine) y rumbo inicial entre dos coordenadas (`NEO_6M_Distance()`, `NEO_6M_Bearing()`).
- **Datos de navegación**: HDOP, velocidad sobre el suelo en km/h, rumbo sobre el suelo y fecha UTC (GPGGA/GPRMC).
- **Manejo robusto de errores**: Códigos de retorno específicos para parámetro inválido, timeout (sin sentencias NMEA válidas) o módulo no inicializado (`NEO_6M_Status_t`). Recuperación automática de la recepción tras un overrun UART.
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

> [!NOTE]
> El parser NMEA usa `strtod()`/`atoi()` (no `sscanf()`), así que **no** necesita `-u _scanf_float` con newlib-nano.

---

## Uso Básico

### 1. Inicialización

```c
// En main() después de HAL_Init() y MX_USARTx_UART_Init()
NEO_6M_Status_t status = NEO_6M_Init(&huart1);

if (status != NEO_6M_OK) {
    Error_Handler();  // Handle NULL — verificar el handle UART
}
```

### 2. Integrar el Callback UART

Dentro del bloque `/* USER CODE BEGIN 4 */` en `main.c`, implementa:

```c
/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    NEO_6M_UART_RxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    NEO_6M_UART_ErrorCallback(huart);
}
/* USER CODE END 4 */
```

> [!IMPORTANT]
> Ambas llamadas son **obligatorias**. Sin `NEO_6M_UART_RxCpltCallback()` el driver no recibe nada y `NEO_6M_Get()` devuelve `NEO_6M_TIMEOUT`. Sin `NEO_6M_UART_ErrorCallback()`, el primer overrun de la UART (ORE) detiene la recepción para siempre: el GPS emite sin pausa, así que basta con que otra interrupción retrase la lectura ~2 ms.

> [!NOTE]
> La librería rearma automáticamente la interrupción UART byte a byte dentro de `NEO_6M_UART_RxCpltCallback()`. No es necesario volver a llamar a `HAL_UART_Receive_IT()` manualmente.

### 3. Obtener Datos GPS

```c
NEO_6M_Data_t gps;

// NEO_6M_TIMEOUT: no llegan sentencias válidas (GPS desconectado, sin alimentación...)
// Sin fix sí llegan: fixQuality == 0 / posStatus == 'V' y la posición es la última conocida
if (NEO_6M_Get(&gps) == NEO_6M_OK && gps.fixQuality > 0) {
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
NEO_6M_DMS_t dmsLat, dmsLon;

NEO_6M_DecimalToDMS(gps.latitude,  gps.northSouth, &dmsLat);
NEO_6M_DecimalToDMS(gps.longitude, gps.eastWest,   &dmsLon);
// dmsLat  →  ej. 19° 25' 57" N
// dmsLon  →  ej. 99° 7' 59" W
```

### 5. Convertir a UTM

```c
NEO_6M_UTM_t utm;

if (NEO_6M_DecimalToUTM(gps.latitude, gps.longitude, &utm) == NEO_6M_OK) {
    // utm.zoneNumber  →  ej. 14
    // utm.zone        →  ej. 'Q'
    // utm.easting     →  ej. 486239 m
    // utm.northing    →  ej. 2150619 m
}
```

### 6. Distancia y Rumbo a un Destino

```c
const double destLat = 19.4326, destLon = -99.1332;   // Destino en grados decimales
double distM, rumbo;

if (NEO_6M_Distance(gps.latitude, gps.longitude, destLat, destLon, &distM) == NEO_6M_OK &&
    NEO_6M_Bearing (gps.latitude, gps.longitude, destLat, destLon, &rumbo) == NEO_6M_OK) {
    // distM  →  metros hasta el destino
    // rumbo  →  0° = Norte, 90° = Este (sentido horario), rango [0, 360)
}
```

---

## API Reference

### 1. Tipos de Datos

#### `NEO_6M_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las funciones de la librería, permitiendo una gestión robusta de errores y estados del módulo.

```c
typedef enum {
    NEO_6M_OK              = 0,  /**< Operación exitosa */
    NEO_6M_ERROR           = 1,  /**< Error en la operación */
    NEO_6M_TIMEOUT         = 2,  /**< Timeout en la operación */
    NEO_6M_NOT_INITIALIZED = 3,  /**< Módulo no inicializado */
    NEO_6M_INVALID_PARAM   = 4   /**< Parámetro inválido */
} NEO_6M_Status_t;
```

| Valor | Código | Significado |
|-------|--------|-------------|
| `NEO_6M_OK` | 0 | Operación completada sin errores |
| `NEO_6M_ERROR` | 1 | Error en la operación |
| `NEO_6M_TIMEOUT` | 2 | `NEO_6M_Get()`: ninguna sentencia NMEA válida en los últimos `NEO_6M_DATA_TIMEOUT_MS` (3000 ms por defecto, redefinible antes de incluir el `.h` o con `-D`) |
| `NEO_6M_NOT_INITIALIZED` | 3 | `NEO_6M_Init()` no se llamó o falló |
| `NEO_6M_INVALID_PARAM` | 4 | Parámetro nulo o inválido |

---

#### `NEO_6M_Data_t` - Estructura de Datos

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
    float    hdop;            /**< Dilución horizontal de precisión (GPGGA) */
    float    speedKmh;        /**< Velocidad sobre el suelo en km/h (GPRMC) */
    float    course;          /**< Rumbo sobre el suelo en grados (GPRMC)  */
    uint8_t  day;             /**< Día UTC (GPRMC)                         */
    uint8_t  month;           /**< Mes UTC (GPRMC)                         */
    uint8_t  year;            /**< Año UTC, dos dígitos (GPRMC)            */
} NEO_6M_Data_t;
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
| `hdop` | `float` | 0.5 – 99.99 | — | Precisión horizontal del fix: < 2 bueno, > 5 pobre (GPGGA) |
| `speedKmh` | `float` | ≥ 0 | km/h | Velocidad sobre el suelo, convertida de nudos × 1.852 (GPRMC) |
| `course` | `float` | 0 – 360 | ° | Rumbo sobre el suelo; solo fiable en movimiento (GPRMC) |
| `day` | `uint8_t` | 1 – 31 | — | Día UTC (GPRMC) |
| `month` | `uint8_t` | 1 – 12 | — | Mes UTC (GPRMC) |
| `year` | `uint8_t` | 0 – 99 | — | Año UTC en dos dígitos, 26 = 2026 (GPRMC) |

---

#### `NEO_6M_DMS_t` - Coordenadas en Formato DMS

Estructura para almacenar una coordenada en formato Grados, Minutos y Segundos.

```c
typedef struct {
    uint8_t degrees;    /**< Grados enteros de la coordenada                */
    uint8_t minutes;    /**< Minutos enteros de la coordenada               */
    uint8_t seconds;    /**< Segundos enteros de la coordenada              */
    char    hemisphere; /**< Hemisferio: 'N', 'S', 'E' o 'W'               */
} NEO_6M_DMS_t;
```

| Campo | Tipo | Descripción |
|-------|------|-------------|
| `degrees` | `uint8_t` | Grados enteros |
| `minutes` | `uint8_t` | Minutos enteros (0 – 59) |
| `seconds` | `uint8_t` | Segundos enteros (0 – 59) |
| `hemisphere` | `char` | Hemisferio: `'N'`, `'S'`, `'E'` o `'W'` |

---

#### `NEO_6M_UTM_t` - Coordenadas en Formato UTM

Estructura para almacenar una coordenada en la proyección Universal Transversa de Mercator.

```c
typedef struct {
    uint32_t easting;    /**< Coordenada Este en metros                     */
    uint32_t northing;   /**< Coordenada Norte en metros                    */
    char     zone;       /**< Letra de banda de latitud UTM (ej: 'T')       */
    uint16_t zoneNumber; /**< Número de zona UTM (ej: 30)                   */
} NEO_6M_UTM_t;
```

| Campo | Tipo | Descripción |
|-------|------|-------------|
| `easting` | `uint32_t` | Coordenada Este en metros (incluye falso este de 500 000 m) |
| `northing` | `uint32_t` | Coordenada Norte en metros (incluye falso norte de 10 000 000 m en hemisferio sur) |
| `zone` | `char` | Letra de banda de latitud (C – X, sin I ni O) |
| `zoneNumber` | `uint16_t` | Número de zona UTM (1 – 60) |

---

### 2. Funciones Públicas

#### `NEO_6M_Init()` - Inicialización del Driver

Inicializa el driver NEO-6M, limpia los buffers internos y habilita la recepción por interrupciones UART. Todas las demás funciones retornan `NEO_6M_NOT_INITIALIZED` si esta función no se ha llamado exitosamente.

```c
NEO_6M_Status_t NEO_6M_Init(UART_HandleTypeDef* huart);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `huart` | `UART_HandleTypeDef*` | Puntero al handle del periférico UART configurado |

**Retorna**: `NEO_6M_OK` si la inicialización fue exitosa, `NEO_6M_INVALID_PARAM` si `huart` es NULL, `NEO_6M_ERROR` si `HAL_UART_Receive_IT` falla (el driver queda sin inicializar).

Llamarla de nuevo sin `NEO_6M_DeInit()` es válido: aborta la recepción previa y reinicia el driver.

**Secuencia interna:**

1. Validación del parámetro `huart`.
2. Aborto de una recepción previa, si la hay (`NEO_6M_DeInit()`).
3. Limpieza del buffer de recepción y de la estructura de datos GPS.
4. Habilitación de la recepción UART por interrupciones (`HAL_UART_Receive_IT`, 1 byte); si la HAL falla se retorna `NEO_6M_ERROR`.
5. Activación de la bandera de inicialización.

---

#### `NEO_6M_DeInit()` - Desinicialización del Driver

Desinicializa el driver NEO-6M, aborta la recepción UART por interrupciones (`HAL_UART_AbortReceive_IT`) y libera el handle UART interno.

```c
NEO_6M_Status_t NEO_6M_DeInit(void);
```

**Retorna**: Siempre retorna `NEO_6M_OK`.

---

#### `NEO_6M_Get()` - Lectura de Datos GPS

Copia los últimos datos GPS parseados a la estructura del llamador. Los datos reflejan la última sentencia NMEA válida recibida.

```c
NEO_6M_Status_t NEO_6M_Get(NEO_6M_Data_t* data);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `data` | `NEO_6M_Data_t*` | Puntero a la estructura donde se copiarán los datos GPS | N/A |

**Retorna**: `NEO_6M_OK` si los datos fueron copiados correctamente, `NEO_6M_TIMEOUT` si no llega ninguna sentencia válida desde hace más de `NEO_6M_DATA_TIMEOUT_MS` o aún no ha llegado ninguna desde `Init` (los datos se copian igualmente), `NEO_6M_NOT_INITIALIZED` si el driver no está inicializado, `NEO_6M_INVALID_PARAM` si `data` es NULL.

Cuando el módulo pierde el fix, los campos vacíos de la sentencia NMEA no sobrescriben los datos: `latitude`/`longitude`, la hora y `altitude` conservan el último valor conocido. `fixQuality` y `numSatellites` sí se actualizan siempre (0 sin fix), y `posStatus` pasa a `'V'`.

Es seguro llamarla dentro de una sección crítica: restaura el estado previo de las interrupciones (`PRIMASK`).

---

#### `NEO_6M_DecimalToDMS()` - Conversión a Grados/Minutos/Segundos

Convierte una coordenada en grados decimales al formato DMS (Grados, Minutos, Segundos).

```c
NEO_6M_Status_t NEO_6M_DecimalToDMS(double decimalDegrees, char hemisphere, NEO_6M_DMS_t* dms);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `decimalDegrees` | `double` | Coordenada en grados decimales |
| `hemisphere` | `char` | Hemisferio de la coordenada: `'N'`, `'S'`, `'E'` o `'W'` |
| `dms` | `NEO_6M_DMS_t*` | Puntero a la estructura donde se escribirá el resultado |

**Retorna**: `NEO_6M_OK` si la conversión fue exitosa, `NEO_6M_INVALID_PARAM` si `dms` es NULL o `|decimalDegrees| > 180` (o NaN).

---

#### `NEO_6M_DecimalToUTM()` - Conversión a Proyección UTM

Convierte latitud y longitud en grados decimales a la proyección Universal Transversa de Mercator (UTM) usando el elipsoide WGS84. Incluye las excepciones de zona para Noruega y Svalbard.

```c
NEO_6M_Status_t NEO_6M_DecimalToUTM(double latitude, double longitude, NEO_6M_UTM_t* utm);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `latitude` | `double` | Latitud en grados decimales | -80.0 – +84.0 |
| `longitude` | `double` | Longitud en grados decimales | -180.0 – +180.0 |
| `utm` | `NEO_6M_UTM_t*` | Puntero a la estructura donde se escribirá el resultado | N/A |

**Retorna**: `NEO_6M_OK` si la conversión fue exitosa, `NEO_6M_INVALID_PARAM` si `utm` es NULL o las coordenadas están fuera de rango (o son NaN).

> [!NOTE]
> Requiere que el proyecto esté enlazado con la biblioteca matemática (`-lm`). En STM32CubeIDE esto es habitual; si se usa un Makefile personalizado, añadir la flag al linker.

---

#### `NEO_6M_Distance()` - Distancia entre Dos Puntos

Calcula la distancia sobre la superficie terrestre entre dos coordenadas con la fórmula de haversine, usando una Tierra esférica de radio `NEO_6M_EARTH_RADIUS_M` (6 371 008.8 m, redefinible antes de incluir el `.h` o con `-D`). El error frente al elipsoide WGS84 es < 0.5 %, muy por debajo del error de posición del propio GPS.

```c
NEO_6M_Status_t NEO_6M_Distance(double lat1, double lon1, double lat2, double lon2, double* meters);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `lat1`, `lon1` | `double` | Latitud y longitud del punto 1 en grados decimales | -90.0 – +90.0 / -180.0 – +180.0 |
| `lat2`, `lon2` | `double` | Latitud y longitud del punto 2 en grados decimales | -90.0 – +90.0 / -180.0 – +180.0 |
| `meters` | `double*` | Puntero donde se escribirá la distancia en metros | N/A |

**Retorna**: `NEO_6M_OK` si el cálculo fue exitoso, `NEO_6M_INVALID_PARAM` si `meters` es NULL o alguna coordenada está fuera de rango (o es NaN).

---

#### `NEO_6M_Bearing()` - Rumbo entre Dos Puntos

Calcula el rumbo inicial (azimut) del punto 1 al punto 2 sobre el círculo máximo: 0° = Norte, 90° = Este, en sentido horario. Es el rumbo de salida; en trayectos largos el rumbo cambia a lo largo del camino. Si ambos puntos coinciden el rumbo no está definido y se devuelve 0.

```c
NEO_6M_Status_t NEO_6M_Bearing(double lat1, double lon1, double lat2, double lon2, double* degrees);
```

| Parámetro | Tipo | Descripción | Rango |
|-----------|------|-------------|-------|
| `lat1`, `lon1` | `double` | Latitud y longitud de origen en grados decimales | -90.0 – +90.0 / -180.0 – +180.0 |
| `lat2`, `lon2` | `double` | Latitud y longitud de destino en grados decimales | -90.0 – +90.0 / -180.0 – +180.0 |
| `degrees` | `double*` | Puntero donde se escribirá el rumbo en grados | [0, 360) |

**Retorna**: `NEO_6M_OK` si el cálculo fue exitoso, `NEO_6M_INVALID_PARAM` si `degrees` es NULL o alguna coordenada está fuera de rango (o es NaN).

---

#### `NEO_6M_UART_RxCpltCallback()` - Callback de Recepción UART

Manejador de recepción UART — debe llamarse desde `HAL_UART_RxCpltCallback()` en `main.c`. Acumula bytes en el buffer interno hasta detectar un salto de línea (`\n`), momento en que valida el checksum NMEA y, si es correcto, parsea la sentencia para actualizar la estructura `NEO_6M_Data_t`. Rearma la interrupción automáticamente.

```c
void NEO_6M_UART_RxCpltCallback(UART_HandleTypeDef* huart);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `huart` | `UART_HandleTypeDef*` | Handle UART pasado por el callback HAL |

> [!WARNING]
> Esta función se ejecuta en contexto de **interrupción (ISR)**. No agregar código bloqueante ni llamadas HAL que no sean seguras en ISR dentro de `HAL_UART_RxCpltCallback()`.

---

#### `NEO_6M_UART_ErrorCallback()` - Callback de Error UART

Manejador de errores UART — debe llamarse desde `HAL_UART_ErrorCallback()` en `main.c`. Descarta la sentencia en curso (ya está corrupta) y, si la HAL abortó la recepción por un overrun (ORE), limpia el flag y la rearma. Con errores no bloqueantes (ruido, framing) la HAL sigue recibiendo y solo se descarta la sentencia. Ignora handles UART que no sean el del driver.

```c
void NEO_6M_UART_ErrorCallback(UART_HandleTypeDef* huart);
```

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `huart` | `UART_HandleTypeDef*` | Handle UART pasado por el callback HAL |

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](../../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.  
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [2.0.0] - 11-11-2026

Unifica el prefijo de toda la API a `NEO_6M_`, añade cálculo de distancia y rumbo, nuevos campos de navegación y un parser NMEA más robusto. Incluye **cambios incompatibles**: hay que renombrar todos los identificadores de la librería en el código de usuario.

#### Added

- **Distancia entre coordenadas**: `NEO_6M_Distance()` calcula los metros sobre la superficie terrestre entre dos puntos (haversine, Tierra esférica de radio `NEO_6M_EARTH_RADIUS_M`).
- **Rumbo entre coordenadas**: `NEO_6M_Bearing()` calcula el rumbo inicial del punto 1 al punto 2 en grados [0, 360), útil para navegar hacia un destino.
- **Recuperación tras errores UART**: `NEO_6M_UART_ErrorCallback()`, a llamar desde `HAL_UART_ErrorCallback()`; descarta la sentencia corrupta y rearma la recepción tras un overrun (ORE).
- **Detección de GPS sin datos**: `NEO_6M_Get()` devuelve `NEO_6M_TIMEOUT` si no llega ninguna sentencia válida en `NEO_6M_DATA_TIMEOUT_MS` (3000 ms por defecto, configurable); los datos se copian igualmente.
- **Nuevos campos en `NEO_6M_Data_t`**: `hdop` (GPGGA), `speedKmh`, `course`, `day`, `month` y `year` (GPRMC).
- Macros configurables `NEO_6M_DATA_TIMEOUT_MS` y `NEO_6M_EARTH_RADIUS_M` (`#ifndef`, redefinibles con `-D`).

#### Changed

- **[BREAKING]** Prefijo `NEO6M_GPS_` → `NEO_6M_` en todos los tipos, funciones, valores de enum y macros públicos. Basta con buscar y reemplazar:
  - `NEO6M_GPS_Init()` / `DeInit()` / `Get()` / `DecimalToDMS()` / `DecimalToUTM()` / `UART_RxCpltCallback()` → `NEO_6M_Init()` / `NEO_6M_DeInit()` / ...
  - `NEO6M_GPS_Status_t`, `NEO6M_GPS_Data_t`, `NEO6M_GPS_DMS_t`, `NEO6M_GPS_UTM_t` → `NEO_6M_Status_t`, `NEO_6M_Data_t`, `NEO_6M_DMS_t`, `NEO_6M_UTM_t`.
  - `NEO6M_GPS_OK`, `NEO6M_GPS_ERROR`, `NEO6M_GPS_NOT_INITIALIZED`, `NEO6M_GPS_INVALID_PARAM` → `NEO_6M_OK`, ...; **ojo**: `NEO6M_TIMEOUT` (sin `_GPS`) → `NEO_6M_TIMEOUT`.
  - `NEO6M_GPS_RX_BUFFER_SIZE`, `NEO6M_GPS_NMEA_MAX_LEN` → `NEO_6M_RX_BUFFER_SIZE`, `NEO_6M_NMEA_MAX_LEN`. Los valores numéricos del enum no cambian.
- **[BREAKING]** `NEO_6M.h` ya no incluye `<stdio.h>`, `<stdlib.h>`, `<string.h>` ni `<math.h>` (vuelven al `.c`). El código de usuario que dependía de ellos a través del `.h` debe incluirlos por su cuenta.
- **[BREAKING]** `NEO_6M_Get()` puede devolver `NEO_6M_TIMEOUT` (antes siempre `NEO6M_GPS_OK` con el driver inicializado). Si se comprobaba `== NEO_6M_OK`, ahora sin GPS conectado no se entra en el bloque.
- El parser NMEA usa `strtod()`/`atoi()` por campos en lugar de `sscanf()`: ya no hace falta `-u _scanf_float` con newlib-nano.
- `NEO_6M_Init()` es reentrante (llamarla sin `DeInit` reinicia el driver) y devuelve `NEO_6M_ERROR` si `HAL_UART_Receive_IT` falla.
- `NEO_6M_DecimalToDMS()` devuelve `NEO_6M_INVALID_PARAM` si `|decimalDegrees| > 180` o NaN; `NEO_6M_DecimalToUTM()` también rechaza NaN.

#### Fixed

- Al perder el fix, los campos vacíos de las sentencias NMEA ya no ponen a cero la posición, la hora ni la altitud: se conserva el último valor conocido (`fixQuality`, `numSatellites` y `posStatus` sí se actualizan).
- Las sentencias de entre 76 y 82 caracteres (máximo NMEA 0183) se rechazaban: `NEO_6M_NMEA_MAX_LEN` pasa de 75 a 82.
- Un overrun UART detenía la recepción para siempre; ahora se recupera con `NEO_6M_UART_ErrorCallback()`, y `NEO_6M_Init()` limpia el flag ORE pendiente antes de armar la recepción.
- `NEO_6M_Get()` restaura el `PRIMASK` previo en lugar de rehabilitar las interrupciones incondicionalmente, así que es seguro llamarla dentro de una sección crítica.

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
