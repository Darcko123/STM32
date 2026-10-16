# Librería para el módulo LIS3DH en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
[![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/LIS3DH)
[![Protocol](https://img.shields.io/badge/Protocol-I2C-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/LIS3DH)

---

## Tabla de Contenidos
- [Librería para el módulo LIS3DH en STM32](#librería-para-el-módulo-lis3dh-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
  - [Configuración I2C](#configuración-i2c)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Leer Aceleración](#2-leer-aceleración)
    - [3. Configurar Escala y Tasa de Muestreo](#3-configurar-escala-y-tasa-de-muestreo)
    - [4. Descripción del Sensor](#4-descripción-del-sensor)
    - [5. Desinicialización](#5-desinicialización)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`LIS3DH_Status_t` - Estados de Retorno](#lis3dh_status_t---estados-de-retorno)
      - [`LIS3DH_Accel_t` - Datos de Aceleración](#lis3dh_accel_t---datos-de-aceleración)
      - [`LIS3DH_Range_t` - Escala Completa](#lis3dh_range_t---escala-completa)
      - [`LIS3DH_DataRate_t` - Tasa de Muestreo](#lis3dh_datarate_t---tasa-de-muestreo)
      - [`LIS3DH_Sensor_t` - Descripción del Sensor](#lis3dh_sensor_t---descripción-del-sensor)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`LIS3DH_Init()` - Inicialización del Driver](#lis3dh_init---inicialización-del-driver)
      - [`LIS3DH_DeInit()` - Desinicialización del Driver](#lis3dh_deinit---desinicialización-del-driver)
      - [`LIS3DH_GetAccel()` - Aceleración en g](#lis3dh_getaccel---aceleración-en-g)
      - [`LIS3DH_GetEvent()` - Aceleración en m/s²](#lis3dh_getevent---aceleración-en-ms)
      - [`LIS3DH_SetRange()` / `LIS3DH_GetRange()` - Escala Completa](#lis3dh_setrange--lis3dh_getrange---escala-completa)
      - [`LIS3DH_SetDataRate()` / `LIS3DH_GetDataRate()` - Tasa de Muestreo](#lis3dh_setdatarate--lis3dh_getdatarate---tasa-de-muestreo)
      - [`LIS3DH_GetSensor()` - Descripción del Sensor](#lis3dh_getsensor---descripción-del-sensor)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[1.0.0\] - 15-10-2026](#100---15-10-2026)
      - [Added](#added)

---

## Descripción
Librería desarrollada en C para la interfaz con el acelerómetro de 3 ejes **STMicroelectronics LIS3DH** utilizando microcontroladores STM32 y la capa HAL. Permite leer la aceleración en los ejes X, Y y Z en unidades `g` o en m/s², configurar la escala completa (±2g a ±16g) y la tasa de muestreo, y obtener una descripción del sensor (rango, resolución y periodo de muestreo) según su configuración actual.

Ideal para detección de inclinación, orientación, vibración y movimiento en proyectos de IoT, wearables y robótica.

---

## Características
- **Comunicación I2C**: Lecturas multibyte con auto-incremento de dirección (`LIS3DH_AUTO_INC`).
- **Verificación del dispositivo**: Comprueba el registro `WHO_AM_I` (`0x33`) durante la inicialización.
- **Alta resolución por defecto**: Tras `LIS3DH_Init()` el sensor queda a 400 Hz, modo alta resolución (12 bits), escala ±2g y *Block Data Update* activado.
- **Conversión automática**: La conversión a `g` lee el modo (alta resolución / normal / bajo consumo) y la escala directamente de los registros, y aplica la sensibilidad de la tabla 4 del datasheet. Cambiar la escala no requiere ajustar nada en el código del usuario.
- **Unidades SI**: `LIS3DH_GetEvent()` entrega la aceleración en m/s² (gravedad estándar `9.80665`).
- **Escala y tasa de muestreo configurables**: `LIS3DH_SetRange()` / `LIS3DH_SetDataRate()` modifican solo sus bits (lectura-modificación-escritura), conservando el resto de la configuración.
- **Descripción del sensor**: `LIS3DH_GetSensor()` calcula rango, resolución y periodo de muestreo a partir de la configuración real del chip.
- **Interrupción de dato listo**: La inicialización enruta DRDY1 al pin **INT1**.
- **Manejo robusto de errores**: Códigos de retorno específicos para error de I2C, timeout, módulo no inicializado o parámetro inválido (`LIS3DH_Status_t`).
- **Instancia única**: El driver gestiona un único sensor LIS3DH por proyecto.

---

## Pinout y Conexiones
### Pines requeridos

| Pin LIS3DH | Dirección | Descripción | Tipo GPIO | Observaciones |
|------------|-----------|-------------|-----------|---------------|
| **VCC**    | Alimentación | 1.71V a 3.6V (chip) | N/A | Comprobar si el módulo incluye regulador antes de usar 5V |
| **GND**    | Tierra    | — | N/A | — |
| **SCL**    | Input (open-drain) | Línea de reloj I2C | `I2C_SCL` | Requiere resistencia pull-up (típicamente 4.7kΩ) |
| **SDA**    | Input/Output (open-drain) | Línea de datos I2C | `I2C_SDA` | Requiere resistencia pull-up (típicamente 4.7kΩ) |
| **SDO/SA0** | Input | Bit menos significativo de la dirección I2C | N/A | GND → `0x18`, VCC → `0x19` |
| **CS**     | Input | Selección de interfaz | N/A | Debe estar en alto (VCC) para usar I2C |
| **INT1**   | Output | Interrupción de dato listo (DRDY1) | `GPIO_EXTI` | Opcional; la librería no la utiliza internamente |

> [!NOTE]
> La mayoría de los módulos comerciales ya integran las resistencias pull-up en SDA/SCL y fijan CS en alto.

---

## Configuración I2C
Configura tu periférico I2C en CubeMX/STM32CubeIDE:

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Mode** | I2C | — |
| **I2C Speed Mode** | Standard Mode / Fast Mode | Soporta 100 kHz y 400 kHz |
| **I2C Clock Speed** | 100000 o 400000 Hz | Depende del requerimiento |

> [!NOTE]
> La librería usa por defecto la dirección **0x18** (SA0 = GND), definida como `LIS3DH_ADDRESS (0x18 << 1)` porque la capa HAL espera la dirección desplazada 1 bit. Si SA0 está conectado a VCC, cambia la definición a `(0x19 << 1)` en `LIS3DH.h`.

---

## Instalación
1. Copia `LIS3DH.c` y `LIS3DH.h` a tu proyecto (ej: `Inc/LIS3DH.h`, `Src/LIS3DH.c`).
2. Incluye la librería en tu código fuente:
   ```c
   #include "LIS3DH.h"
   ```
3. Configura I2C en CubeMX (ver sección anterior).
4. Genera código y compila.

---

## Uso Básico

### 1. Inicialización
```c
// En main() después de HAL_Init() y MX_I2C1_Init()
if (LIS3DH_Init(&hi2c1) != LIS3DH_OK) {
    Error_Handler();  // Sin respuesta, WHO_AM_I incorrecto o handle NULL
}
```

### 2. Leer Aceleración
```c
LIS3DH_Accel_t accel;

// En unidades 'g'
if (LIS3DH_GetAccel(&accel) == LIS3DH_OK) {
    printf("X: %.3f g, Y: %.3f g, Z: %.3f g\n", accel.x, accel.y, accel.z);
}

// En m/s²
if (LIS3DH_GetEvent(&accel) == LIS3DH_OK) {
    printf("X: %.2f, Y: %.2f, Z: %.2f m/s^2\n", accel.x, accel.y, accel.z);
}
```

### 3. Configurar Escala y Tasa de Muestreo
```c
LIS3DH_SetRange(LIS3DH_RANGE_8G);          // ±8g
LIS3DH_SetDataRate(LIS3DH_DATARATE_100HZ); // 100 Hz

LIS3DH_Range_t range;
LIS3DH_DataRate_t rate;
LIS3DH_GetRange(&range);
LIS3DH_GetDataRate(&rate);
```

### 4. Descripción del Sensor
```c
LIS3DH_Sensor_t sensor;

if (LIS3DH_GetSensor(&sensor) == LIS3DH_OK) {
    printf("%s: ±%.2f m/s^2, resolución %.4f m/s^2, periodo %lu us\n",
           sensor.name, sensor.max_value, sensor.resolution, sensor.min_delay);
}
```

### 5. Desinicialización
```c
// Libera el handle de I2C y marca el módulo como no inicializado
LIS3DH_DeInit();
```

---

## API Reference

### 1. Tipos de Datos

#### `LIS3DH_Status_t` - Estados de Retorno

```c
typedef enum {
    LIS3DH_OK              = 0,    /**< Operación exitosa */
    LIS3DH_ERROR           = 1,    /**< Error en la operación */
    LIS3DH_TIMEOUT         = 2,    /**< Timeout en la operación */
    LIS3DH_NOT_INITIALIZED = 3,    /**< Módulo no inicializado */
    LIS3DH_INVALID_PARAM   = 4     /**< Parámetro inválido */
} LIS3DH_Status_t;
```

|Valor|Código|Significado|
|---|---|---|
|`LIS3DH_OK`|0|Operación completada sin errores|
|`LIS3DH_ERROR`|1|Error de I2C o `WHO_AM_I` incorrecto|
|`LIS3DH_TIMEOUT`|2|Timeout de la transacción I2C|
|`LIS3DH_NOT_INITIALIZED`|3|`LIS3DH_Init()` no se llamó o falló|
|`LIS3DH_INVALID_PARAM`|4|Puntero NULL o valor de enumeración fuera de rango|

---

#### `LIS3DH_Accel_t` - Datos de Aceleración

```c
typedef struct {
    float x;    /**< Valor de aceleración en el eje X. */
    float y;    /**< Valor de aceleración en el eje Y. */
    float z;    /**< Valor de aceleración en el eje Z. */
} LIS3DH_Accel_t;
```

La unidad depende de la función que la rellene: `g` con `LIS3DH_GetAccel()`, m/s² con `LIS3DH_GetEvent()`.

---

#### `LIS3DH_Range_t` - Escala Completa

Bits FS[1:0] de `CTRL_REG4`.

|Valor|Código|Escala|
|---|---|---|
|`LIS3DH_RANGE_2G`|0|±2g|
|`LIS3DH_RANGE_4G`|1|±4g|
|`LIS3DH_RANGE_8G`|2|±8g|
|`LIS3DH_RANGE_16G`|3|±16g|

---

#### `LIS3DH_DataRate_t` - Tasa de Muestreo

Bits ODR[3:0] de `CTRL_REG1`.

|Valor|Código|Frecuencia|
|---|---|---|
|`LIS3DH_DATARATE_POWERDOWN`|0|Apagado|
|`LIS3DH_DATARATE_1HZ`|1|1 Hz|
|`LIS3DH_DATARATE_10HZ`|2|10 Hz|
|`LIS3DH_DATARATE_25HZ`|3|25 Hz|
|`LIS3DH_DATARATE_50HZ`|4|50 Hz|
|`LIS3DH_DATARATE_100HZ`|5|100 Hz|
|`LIS3DH_DATARATE_200HZ`|6|200 Hz|
|`LIS3DH_DATARATE_400HZ`|7|400 Hz|
|`LIS3DH_DATARATE_LOWPOWER_1K6HZ`|8|1.6 kHz (solo modo bajo consumo)|
|`LIS3DH_DATARATE_1K344HZ_LP5K376`|9|1.344 kHz (normal/alta resolución) o 5.376 kHz (bajo consumo)|

> [!NOTE]
> La librería no expone una función para activar el modo bajo consumo (`LIS3DH_Init()` deja el sensor en alta resolución), por lo que `LIS3DH_DATARATE_LOWPOWER_1K6HZ` solo tiene sentido si el bit LPen de `CTRL_REG1` se activa por otros medios.

---

#### `LIS3DH_Sensor_t` - Descripción del Sensor

```c
typedef struct {
    const char *name;       /**< Nombre del sensor. */
    int32_t     version;    /**< Versión del driver. */
    float       max_value;  /**< Valor máximo medible (m/s²). */
    float       min_value;  /**< Valor mínimo medible (m/s²). */
    float       resolution; /**< Resolución (m/s² por dígito). */
    uint32_t    min_delay;  /**< Periodo de muestreo en µs (0 si está apagado). */
} LIS3DH_Sensor_t;
```

---

### 2. Funciones Públicas

#### `LIS3DH_Init()` - Inicialización del Driver

```c
LIS3DH_Status_t LIS3DH_Init(I2C_HandleTypeDef* hi2c);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`hi2c`|`I2C_HandleTypeDef*`|Puntero al handle de I2C|

**Retorna**: `LIS3DH_OK` si la inicialización fue exitosa, `LIS3DH_ERROR` si falla la comunicación o `WHO_AM_I` no es `0x33`, `LIS3DH_INVALID_PARAM` si el handle es NULL.

**Secuencia interna:**

1. Espera 5 ms (tiempo de arranque del sensor).
2. Lee `WHO_AM_I` y verifica que sea `0x33`.
3. `CTRL_REG1 = 0x77`: ejes X, Y, Z habilitados, 400 Hz, LPen = 0.
4. `CTRL_REG4 = 0x88`: BDU activado, escala ±2g, alta resolución.
5. `CTRL_REG3 = 0x10`: interrupción DRDY1 en el pin INT1.
6. `TEMP_CFG_REG = 0x80`: ADC auxiliar habilitado.

---

#### `LIS3DH_DeInit()` - Desinicialización del Driver

Libera el handle de I2C y marca el módulo como no inicializado. No modifica los registros del sensor.

```c
LIS3DH_Status_t LIS3DH_DeInit(void);
```

**Retorna**: Siempre `LIS3DH_OK`.

---

#### `LIS3DH_GetAccel()` - Aceleración en g

```c
LIS3DH_Status_t LIS3DH_GetAccel(LIS3DH_Accel_t *accel);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`accel`|`LIS3DH_Accel_t*`|Estructura donde se guardan los valores en `g`|

**Retorna**: `LIS3DH_OK`, `LIS3DH_ERROR`, `LIS3DH_TIMEOUT`, `LIS3DH_NOT_INITIALIZED` o `LIS3DH_INVALID_PARAM` si `accel` es NULL.

Lee `CTRL_REG1..CTRL_REG4` para conocer modo y escala, lee 6 bytes desde `OUT_X_L` y escala los datos (little-endian, justificados a la izquierda) según la sensibilidad del datasheet.

---

#### `LIS3DH_GetEvent()` - Aceleración en m/s²

```c
LIS3DH_Status_t LIS3DH_GetEvent(LIS3DH_Accel_t *accel);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`accel`|`LIS3DH_Accel_t*`|Estructura donde se guardan los valores en m/s²|

**Retorna**: Igual que `LIS3DH_GetAccel()`, de la que reutiliza la lectura multiplicando por `LIS3DH_GRAVITY_STANDARD`.

---

#### `LIS3DH_SetRange()` / `LIS3DH_GetRange()` - Escala Completa

```c
LIS3DH_Status_t LIS3DH_SetRange(LIS3DH_Range_t range);
LIS3DH_Status_t LIS3DH_GetRange(LIS3DH_Range_t *range);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`range`|`LIS3DH_Range_t` / `LIS3DH_Range_t*`|Escala a configurar / puntero donde se guarda la escala actual|

**Retorna**: `LIS3DH_OK`, `LIS3DH_ERROR`, `LIS3DH_TIMEOUT`, `LIS3DH_NOT_INITIALIZED` o `LIS3DH_INVALID_PARAM` (escala fuera de rango o puntero NULL).

`LIS3DH_SetRange()` solo modifica FS[1:0] de `CTRL_REG4` (conserva BDU y HR) y espera 15 ms para que la nueva escala se estabilice.

---

#### `LIS3DH_SetDataRate()` / `LIS3DH_GetDataRate()` - Tasa de Muestreo

```c
LIS3DH_Status_t LIS3DH_SetDataRate(LIS3DH_DataRate_t dataRate);
LIS3DH_Status_t LIS3DH_GetDataRate(LIS3DH_DataRate_t *dataRate);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`dataRate`|`LIS3DH_DataRate_t` / `LIS3DH_DataRate_t*`|Tasa a configurar / puntero donde se guarda la tasa actual|

**Retorna**: `LIS3DH_OK`, `LIS3DH_ERROR`, `LIS3DH_TIMEOUT`, `LIS3DH_NOT_INITIALIZED` o `LIS3DH_INVALID_PARAM` (tasa fuera de rango o puntero NULL).

`LIS3DH_SetDataRate()` solo modifica ODR[3:0] de `CTRL_REG1` (conserva LPen y la habilitación de ejes).

---

#### `LIS3DH_GetSensor()` - Descripción del Sensor

```c
LIS3DH_Status_t LIS3DH_GetSensor(LIS3DH_Sensor_t *sensor);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`sensor`|`LIS3DH_Sensor_t*`|Estructura donde se guarda la descripción|

**Retorna**: `LIS3DH_OK`, `LIS3DH_ERROR`, `LIS3DH_TIMEOUT`, `LIS3DH_NOT_INITIALIZED` o `LIS3DH_INVALID_PARAM` si `sensor` es NULL.

Calcula, a partir de la configuración leída del chip: `max_value`/`min_value` como la escala completa en m/s², `resolution` como la sensibilidad en m/s² por dígito y `min_delay` como el periodo de muestreo en µs (0 si está apagado).

---

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/../LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.  
El formato está basado en [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

### [1.0.0] - 15-10-2026

Primera versión del driver del acelerómetro LIS3DH para STM32 por I2C.

#### Added
- **Inicialización**: `LIS3DH_Init()` verifica `WHO_AM_I` y configura 400 Hz, alta resolución, ±2g, BDU y DRDY1 en INT1; `LIS3DH_DeInit()` libera el driver.
- **Lectura en g**: `LIS3DH_GetAccel()` con conversión automática según el modo y la escala leídos del chip.
- **Lectura en m/s²**: `LIS3DH_GetEvent()`, para trabajar directamente en unidades SI.
- **Escala completa**: `LIS3DH_SetRange()` / `LIS3DH_GetRange()` (±2g, ±4g, ±8g, ±16g).
- **Tasa de muestreo**: `LIS3DH_SetDataRate()` / `LIS3DH_GetDataRate()` (apagado a 5.376 kHz).
- **Descripción del sensor**: `LIS3DH_GetSensor()` devuelve rango, resolución y periodo de muestreo de la configuración actual.
- Tipos `LIS3DH_Status_t`, `LIS3DH_Accel_t`, `LIS3DH_Range_t`, `LIS3DH_DataRate_t` y `LIS3DH_Sensor_t`.

---
