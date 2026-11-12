# Librería para el módulo DHT11 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-2.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/DHT11)
[![Protocol](https://img.shields.io/badge/Protocol-Single--Wire-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/DHT11)

---

## Tabla de Contenidos
- [Librería para el módulo DHT11 en STM32](#librería-para-el-módulo-dht11-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Pinout y Conexiones](#pinout-y-conexiones)
    - [Pines requeridos](#pines-requeridos)
    - [Rangos operativos](#rangos-operativos)
  - [Configuración del Timer y GPIO](#configuración-del-timer-y-gpio)
    - [1. Timer para temporización en microsegundos](#1-timer-para-temporización-en-microsegundos)
    - [2. Pin de datos GPIO](#2-pin-de-datos-gpio)
  - [Instalación](#instalación)
  - [Uso Básico](#uso-básico)
    - [1. Inicialización](#1-inicialización)
    - [2. Obtener Datos](#2-obtener-datos)
    - [3. Desinicialización](#3-desinicialización)
    - [Ejemplo completo](#ejemplo-completo)
  - [Funcionamiento del Protocolo](#funcionamiento-del-protocolo)
    - [Teoría de inicialización del sensor](#teoría-de-inicialización-del-sensor)
    - [Señal de inicio y respuesta](#señal-de-inicio-y-respuesta)
    - [Transmisión de Datos y Formato de Trama](#transmisión-de-datos-y-formato-de-trama)
    - [Secuencia de Comunicación](#secuencia-de-comunicación)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`DHT11_Status_t` - Estados de Retorno](#dht11_status_t---estados-de-retorno)
      - [`DHT11_Data_t` - Estructura de Datos](#dht11_data_t---estructura-de-datos)
    - [2. Funciones Públicas](#2-funciones-públicas)
      - [`DHT11_Init()` - Inicialización del Driver](#dht11_init---inicialización-del-driver)
      - [`DHT11_Get()` - Lectura de Datos](#dht11_get---lectura-de-datos)
      - [`DHT11_DeInit()` - Desinicialización del Driver](#dht11_deinit---desinicialización-del-driver)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [\[2.0.0\] - 12-11-2026](#200---12-11-2026)
      - [Added](#added)
      - [Changed](#changed)
      - [Fixed](#fixed)
    - [\[1.0.0\] - 21-01-2026](#100---21-01-2026)
      - [Added](#added-1)

---

## Descripción
Librería desarrollada en C para la interfaz con el sensor de temperatura y humedad **DHT11** mediante comunicación digital de un solo cable (*single-wire*), con temporización precisa en microsegundos basada en un Timer de hardware. Permite obtener la temperatura (°C) y la humedad relativa (%RH) en sistemas embebidos basados en microcontroladores STM32 utilizando HAL.

Ideal para aplicaciones de monitoreo ambiental, estaciones meteorológicas y proyectos de IoT de bajo costo.

---

## Características
- **Comunicación single-wire**: Protocolo digital implementado en un único pin de datos, que la librería conmuta dinámicamente entre salida y entrada.
- **Temporización por Timer de hardware**: Retardos y timeouts en microsegundos medidos con el contador de un Timer a 1 MHz.
- **Gestión robusta de errores**: Códigos de retorno específicos para timeout, checksum, parámetros inválidos y estado de inicialización (`DHT11_Status_t`).
- **Respeto del intervalo mínimo**: `DHT11_Get()` devuelve la última medición válida si se llama antes de `DHT11_MIN_INTERVAL_MS` (2 s), sin consultar al sensor.
- **Temperaturas negativas**: Interpreta el bit de signo de los DHT11 de rango -20 °C a 60 °C.
- **Timers de 16 o 32 bits**: Manejo correcto del desbordamiento del contador con el ARR al máximo en ambos casos.
- **Documentación profesional**: Código documentado con estándar Doxygen.
- **Portabilidad**: `DHT11.h` incluye `main.h`, por lo que funciona en cualquier familia STM32 generada con CubeMX sin editar el encabezado.
- **Instancia única**: El driver gestiona un único sensor DHT11 por proyecto.

> [!NOTE]
> Los retardos se implementan como espera activa sobre el contador del Timer, por lo que `DHT11_Get()` es **bloqueante** (ver [`DHT11_Get()`](#dht11_get---lectura-de-datos)).

---

## Pinout y Conexiones
### Pines requeridos

| Pin DHT11 | Dirección | Descripción | Tipo GPIO | Observaciones |
|-----------|-----------|-------------|-----------|---------------|
| **VCC**   | Alimentación | 3.3V a 5V | N/A | — |
| **GND**   | Tierra    | — | N/A | — |
| **DATA**  | Input/Output | Línea de datos single-wire | `GPIO_Output` / `GPIO_Input` (conmutado por la librería) | Cualquier pin digital. Pull-up recomendada de 4.7 kΩ a VCC |

> [!NOTE]
> En configuraciones con líneas de datos largas o alta impedancia, se recomienda conectar una resistencia de pull-up de 4.7 kΩ entre VCC y el pin de datos para garantizar niveles lógicos adecuados durante la fase de alta impedancia. Cuando el pin está en modo entrada, la librería activa además el pull-up interno del STM32 (`GPIO_PULLUP`).

> [!IMPORTANT]
> El orden físico de los pines varía entre el sensor suelto y los módulos comerciales; verifica la serigrafía de tu módulo antes de conectarlo.

### Rangos operativos

| Característica | Rango |
|----------------|-------|
| Rango de Temperatura | 0°C a 50°C (versiones nuevas: -20°C a 60°C) |
| Precisión de Temperatura | ±2°C |
| Rango de Humedad | 20% a 90% RH |
| Precisión de Humedad | ±5% RH |
| Intervalo entre lecturas | 2 s en la librería (`DHT11_MIN_INTERVAL_MS`) |
| Voltaje Operativo | 3.3V a 5V |
| Formato de datos | Digital (single-wire) |

> [!WARNING]
> La versión anterior de este README indicaba un *sampling rate* de 1 lectura por segundo (1 Hz), mientras que la librería y su documentación imponen un mínimo de ~2 s entre lecturas. Se toma como referencia el valor de la librería (2000 ms); confirma el valor con la hoja de datos de tu sensor si necesitas leer más rápido.

---

## Configuración del Timer y GPIO

### 1. Timer para temporización en microsegundos

La comunicación con el DHT11 requiere medir intervalos en microsegundos. Configura un Timer en modo contador con resolución de 1 µs (1 MHz).

La frecuencia del contador se determina mediante:

$$
f_{timer} = \frac{f_{clk}}{(Prescaler + 1)}
$$

Donde:

- $f_{clk}$: Frecuencia de reloj del Timer
- $Prescaler$: Valor del prescaler
- $f_{timer}$: Frecuencia resultante del contador

Para lograr 1 MHz, el prescaler se calcula como:

$$
Prescaler = \frac{f_{clk}}{1 MHz} - 1
$$

**Ejemplo para `STM32F411` a `100MHz`:**

$$
Prescaler = \frac{100MHz}{1 MHz} - 1 = 99
$$

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Timer** | TIM2 | Timer de 32 bits en el STM32F411 |
| **Clock Source** | Internal Clock | — |
| **Prescaler** | 99 | Para 1 MHz con reloj de 100 MHz |
| **Counter Mode** | Up | — |
| **Counter Period (ARR)** | 0xFFFFFFFF | `0xFFFF` si usas un Timer de 16 bits |
| **Internal Clock Division** | No Division | — |

> [!IMPORTANT]
> El Counter Period (ARR) **debe** estar al valor máximo: `0xFFFF` en Timers de 16 bits o `0xFFFFFFFF` (2³² − 1) en Timers de 32 bits. La librería calcula las diferencias de tiempo truncadas a 16 bits; cualquier otro ARR rompe el manejo del desbordamiento.

> [!NOTE]
> Con un Timer de 32 bits el contador tarda 2³² µs (≈ 71.5 minutos) en desbordar. Como las diferencias se calculan en 16 bits, cada retardo o timeout individual queda limitado a 65 535 µs, suficiente para todas las fases del protocolo (la más larga es la señal de inicio de 18 ms).

### 2. Pin de datos GPIO

El pin de datos se configura dinámicamente como salida push-pull durante la señal de inicio y como entrada con pull-up durante la recepción. La configuración de CubeMX solo define su estado inicial.

**Ejemplo de configuración para PA1 en STM32CubeMX:**

| Parámetro | Valor Inicial | Notas |
|-----------|---------------|-------|
| **GPIO Mode** | Output Push Pull | Modo inicial |
| **GPIO Pull-up/Pull-down** | Pull-up | Mantiene nivel alto en modo entrada |
| **Maximum Output Speed** | Low | Suficiente para el protocolo DHT11 |
| **User Label** | DHT11_DATA | Para identificación en código |

> [!NOTE]
> La librería modificará dinámicamente el modo del pin entre salida y entrada según la fase de comunicación.

---

## Instalación
1. Copia `DHT11.c` y `DHT11.h` a tu proyecto (ej: `Inc/DHT11.h`, `Src/DHT11.c`).
2. Incluye la librería en tu código fuente:
   ```c
   #include "DHT11.h"
   ```
3. Configura el Timer y el GPIO en CubeMX (ver sección anterior).
4. Genera código y compila.

> [!NOTE]
> `DHT11.h` incluye `main.h` (generado por CubeMX) y `<stdint.h>`. No incluye `<stdio.h>`: si usas `printf`, inclúyelo tú mismo.

---

## Uso Básico

### 1. Inicialización
```c
// En main() después de HAL_Init(), MX_GPIO_Init() y MX_TIM2_Init()
DHT11_Status_t status = DHT11_Init(&htim2, GPIOB, GPIO_PIN_9);

if (status != DHT11_OK) {
    Error_Handler();  // Parámetro inválido o el timer no arrancó
}

HAL_Delay(1000);  // Estabilización del sensor tras el encendido
```

`DHT11_Init()` arranca el Timer con `HAL_TIM_Base_Start()` si aún no está en marcha, por lo que no es necesario arrancarlo antes.

### 2. Obtener Datos
```c
// Declarar estructura de datos
DHT11_Data_t Environment;

// Leer los datos del sensor
if (DHT11_Get(&Environment) == DHT11_OK)
{
    // Uso de los datos: Environment.temperatura, Environment.humedad
    printf("Temp: %.1f C, Humedad: %.1f %%RH\n",
           Environment.temperatura, Environment.humedad);
}
```

### 3. Desinicialización
```c
// Libera el timer y el pin y marca el módulo como no inicializado
DHT11_DeInit();
```

### Ejemplo completo

```c
/* USER CODE BEGIN PV */
DHT11_Data_t Environment;
/* USER CODE END PV */

int main(void)
{
    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim2);     // Opcional: DHT11_Init() lo arranca si no está en marcha

    if (DHT11_Init(&htim2, GPIOB, GPIO_PIN_9) != DHT11_OK)
    {
        // Led de usuario enciende para indicar error
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        while (1);
    }

    HAL_Delay(1000);    // Delay para estabilización
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // Lectura de sensor
        DHT11_Status_t status = DHT11_Get(&Environment);

        if (status != DHT11_OK)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_Delay(1000);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        }

        HAL_Delay(2000);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}
```

> [!NOTE]
> El ejemplo usa el pin `PB9` para datos y el LED de `PC13` (activo en bajo) como indicador de estado. La tabla de CubeMX anterior usa `PA1` solo como ejemplo: usa el pin que hayas configurado.

---

## Funcionamiento del Protocolo

### Teoría de inicialización del sensor

El DHT11 usa un protocolo maestro-esclavo sobre una sola línea de datos que, en reposo, permanece en **nivel alto** gracias a la resistencia de pull-up. El arranque sigue este orden:

1. **Alimentación**: Se aplica VCC (3.3V a 5V) al sensor.
2. **Espera tras el encendido**: Según la hoja de datos del DHT11, el sensor pasa por un estado inestable durante aproximadamente **1 s** después de alimentarlo; en ese tiempo no debe enviarse ninguna señal de inicio. La librería **no** aplica esta espera internamente: debe hacerla la aplicación (el ejemplo usa `HAL_Delay(1000)` tras `DHT11_Init()`).
3. **Señal de inicio (MCU → sensor)**: El microcontrolador configura el pin como salida, lo lleva a **nivel bajo durante al menos 18 ms** para que el sensor detecte la petición, lo pone en **alto durante 20 µs** y cambia el pin a **entrada** para liberar el bus.
4. **Respuesta del sensor (sensor → MCU)**: El DHT11 toma el bus y responde con un pulso **bajo de 80 µs** seguido de un pulso **alto de 80 µs**, indicando que comenzará a transmitir.
5. **Transmisión**: El sensor envía la trama de 40 bits y, al terminar, libera el bus, que vuelve a nivel alto.

| Fase | Quién controla el bus | Duración | Constante en la librería |
|------|-----------------------|----------|--------------------------|
| Espera tras el encendido | — | ≈ 1 s | No aplica (responsabilidad de la aplicación) |
| Pulso de inicio (bajo) | MCU | ≥ 18 ms | `DHT11_START_SIGNAL_US` = 18000 |
| Pulso de inicio (alto) | MCU | 20 µs | `DHT11_START_WAIT_US` = 20 |
| Respuesta (bajo) | Sensor | 80 µs | Timeout: `DHT11_RESPONSE_TIMEOUT_US` = 100 |
| Respuesta (alto) | Sensor | 80 µs | Timeout: `DHT11_RESPONSE_TIMEOUT_US` = 100 |

> [!NOTE]
> Además del primer arranque, el sensor necesita un intervalo mínimo entre lecturas consecutivas; la librería lo respeta con `DHT11_MIN_INTERVAL_MS` (2000 ms), devolviendo la última medición guardada si se llama antes.

### Señal de inicio y respuesta

**Señal de inicio generada por el microcontrolador:**

![StartSignal](/Librerias/DHT11/Images/StartSignal.jpg)

```c
static DHT11_Status_t dht11_StartCommunication(void)
{
    dht11_SetPinOutput();                                       // Configurar el pin como salida
    HAL_GPIO_WritePin(DHT11_Port, DHT11_Pin, GPIO_PIN_RESET);   // Pin bajo
    dht11_DelayUs(DHT11_START_SIGNAL_US);                       // Esperar por 18 ms
    HAL_GPIO_WritePin(DHT11_Port, DHT11_Pin, GPIO_PIN_SET);     // Pin alto
    dht11_DelayUs(DHT11_START_WAIT_US);                         // Esperar 20 us
    dht11_SetPinInput();                                        // Configurar el pin como entrada

    dht11_DelayUs(2);       // Pequeña espera para estabilización del pin

    return DHT11_OK;
}
```

**Señal de respuesta del sensor DHT11:**

![ResponseSignal](/Librerias/DHT11/Images/ResponseSignal.jpg)

```c
static DHT11_Status_t dht11_CheckResponse(void)
{
    uint32_t start = 0;

    // Esperar señal de bajo inicial (sensor toma control del bus)
    start = __HAL_TIM_GET_COUNTER(DHT11_htim);
    while(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))      // Esperar mientras esté ALTO
    {
        if((uint16_t)(__HAL_TIM_GET_COUNTER(DHT11_htim) - start) > DHT11_RESPONSE_TIMEOUT_US)
            return DHT11_TIMEOUT;
    }

    // Verificar primer pulso bajo (~80μs)
    if(!(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin)))
    {
        // Esperar transición a alto (~80μs)
        start = __HAL_TIM_GET_COUNTER(DHT11_htim);
        while(!HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))
        {
            if((uint16_t)(__HAL_TIM_GET_COUNTER(DHT11_htim) - start) > DHT11_RESPONSE_TIMEOUT_US)
                return DHT11_TIMEOUT;
        }

        // Esperar segundo pulso bajo (~80μs)
        start = __HAL_TIM_GET_COUNTER(DHT11_htim);
        while(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))
        {
            if((uint16_t)(__HAL_TIM_GET_COUNTER(DHT11_htim) - start) > DHT11_RESPONSE_TIMEOUT_US)
                return DHT11_TIMEOUT;
        }

        return DHT11_OK;
    }

    return DHT11_ERROR;
}
```

### Transmisión de Datos y Formato de Trama

Tras la respuesta, el DHT11 transmite una trama de 40 bits (5 bytes), MSB primero, con codificación por ancho de pulso. Cada bit comienza con un pulso bajo de referencia de 50 µs seguido de un pulso alto cuya duración codifica el valor:

- **Bit '0'**: Pulso alto de 26-28 µs
- **Bit '1'**: Pulso alto de aproximadamente 70 µs

La librería espera el flanco de subida, aguarda `DHT11_BIT_READ_DELAY_US` (40 µs) y lee el pin: si sigue en alto el bit es `1`, si ya bajó es `0`. Cada espera de flanco tiene un timeout de `DHT11_BIT_TIMEOUT_US` (150 µs).

**Formato para bit '0' (26-28 µs en alto)**

![SignalFor0](/Librerias/DHT11/Images/SignalFor0.jpg)

**Formato para bit '1' (≈70 µs en alto)**

![SignalFor1](/Librerias/DHT11/Images/SignalFor1.jpg)

**Estructura de la trama de datos (40 bits):**

| Byte | Contenido | Descripción |
|------|-----------|-------------|
| 1    | Humedad Entera | Parte entera de humedad relativa (0-100%) |
| 2    | Humedad Decimal | Parte decimal de humedad relativa |
| 3    | Temperatura Entera | Parte entera de temperatura (°C) |
| 4    | Temperatura Decimal | Bits 0-6: parte decimal de temperatura (°C). Bit 7: signo (`1` = negativa, solo en versiones de -20°C a 60°C) |
| 5    | Checksum | Suma de verificación (8 bits LSB) |

La verificación de integridad se realiza mediante el byte de checksum, que corresponde a los 8 bits menos significativos de la suma de los primeros cuatro bytes:

$$
Checksum = (HumedadEntera + HumedadDecimal + TemperaturaEntera + TemperaturaDecimal) \bmod 256
$$

El checksum se calcula con el byte 4 completo, incluido el bit de signo. La librería ignora ese bit al convertir el valor y, si está en `1`, devuelve la temperatura negativa (por ejemplo, `0x00` / `0x81` → -0.1 °C).

```c
static DHT11_Status_t dht11_ReadByte(uint8_t* byte)
{
    uint8_t i = 0;
    uint32_t start;

    if (byte == NULL)
        return DHT11_ERROR;

    for (uint8_t j = 0; j < 8; j++)
    {
        // Esperar inicio del bit (transición a alto)
        start = __HAL_TIM_GET_COUNTER(DHT11_htim);
        while(!(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin)))
        {
            if((uint16_t)(__HAL_TIM_GET_COUNTER(DHT11_htim) - start) > DHT11_BIT_TIMEOUT_US)
                return DHT11_TIMEOUT;
        }

        // Esperar tiempo crítico para diferenciar 0 de 1
        dht11_DelayUs(DHT11_BIT_READ_DELAY_US);

        // Determinar valor del bit según nivel del pin
        if(!(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin)))
        {
            i &= ~(1 << (7 - j));   // Escribe 0
        }
        else
        {
            i |=  (1 << (7 - j));   // Escribe 1
        }

        // Esperar fin del bit (transición a bajo)
        start = __HAL_TIM_GET_COUNTER(DHT11_htim);
        while(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))
        {
            if((uint16_t)(__HAL_TIM_GET_COUNTER(DHT11_htim) - start) > DHT11_BIT_TIMEOUT_US)
                return DHT11_TIMEOUT;
        }
    }

    *byte = i;
    return DHT11_OK;
}
```

### Secuencia de Comunicación

Fragmento de `DHT11_Get()` que encadena las fases anteriores:

```c
    // --- SECUENCIA DE COMUNICACIÓN ---

    // 1. Iniciar comunicación
    status = dht11_StartCommunication();
    if (status != DHT11_OK)
        return status;

    // 2. Verificar respuesta del sensor
    status = dht11_CheckResponse();
    if (status != DHT11_OK)
        return status;

    // 3. Leer los 5 bytes de datos
    status = dht11_ReadByte(&humedad_entero);
    if (status != DHT11_OK)
        return status;

    status = dht11_ReadByte(&humedad_decimal);
    if (status != DHT11_OK)
        return status;

    status = dht11_ReadByte(&temp_entero);
    if (status != DHT11_OK)
        return status;

    status = dht11_ReadByte(&temp_decimal);
    if (status != DHT11_OK)
        return status;

    status = dht11_ReadByte(&checksum);
    if (status != DHT11_OK)
        return status;

    // 4. Verificar integridad de datos mediante checksum
    if ((uint8_t)(humedad_entero + humedad_decimal + temp_entero + temp_decimal) != checksum)
        return DHT11_CHECKSUM_ERROR;

    // 5. Convertir bytes a valores flotantes
    environment->humedad = (float)humedad_entero + ((float)humedad_decimal / 10.0f);
    // Bit 7 del byte decimal de temperatura = signo (DHT11 nuevos, rango -20 a 60 °C)
    environment->temperatura = (float)temp_entero + ((float)(temp_decimal & 0x7FU) / 10.0f);
    if (temp_decimal & 0x80U)
        environment->temperatura = -environment->temperatura;

    // 6. Guardar medición y marca de tiempo para respetar el intervalo mínimo
    DHT11_LastData  = *environment;
    DHT11_LastTick  = HAL_GetTick();
    DHT11_HasData   = 1U;
```

---

## API Reference

### 1. Tipos de Datos

#### `DHT11_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las operaciones del driver DHT11.

```c
typedef enum {
    DHT11_OK              = 0,  /**< Operación exitosa */
    DHT11_ERROR           = 1,  /**< Error en la operación */
    DHT11_TIMEOUT         = 2,  /**< Timeout en la operación */
    DHT11_NOT_INITIALIZED = 3,  /**< Módulo no inicializado */
    DHT11_INVALID_PARAM   = 4,  /**< Parámetro inválido */
    DHT11_CHECKSUM_ERROR  = 5   /**< Error en el checksum */
} DHT11_Status_t;
```

|Valor|Código|Significado|
|---|---|---|
|`DHT11_OK`|0|Operación completada sin errores|
|`DHT11_ERROR`|1|Respuesta inválida del sensor o el timer no pudo arrancar|
|`DHT11_TIMEOUT`|2|El sensor no respondió dentro del tiempo esperado|
|`DHT11_NOT_INITIALIZED`|3|`DHT11_Init()` no se llamó o falló|
|`DHT11_INVALID_PARAM`|4|Parámetro nulo o pin inválido|
|`DHT11_CHECKSUM_ERROR`|5|Trama recibida corrupta (checksum inválido)|

---

#### `DHT11_Data_t` - Estructura de Datos

Estructura que almacena las mediciones de temperatura y humedad obtenidas del sensor.

```c
typedef struct {
    float humedad;      /**< Humedad relativa (%RH) */
    float temperatura;  /**< Temperatura en grados Celsius (°C) */
} DHT11_Data_t;
```

| Campo | Tipo | Rango | Unidad | Descripción |
|-------|------|-------|--------|-------------|
| `humedad` | `float` | 20 - 90 | %RH | Humedad relativa |
| `temperatura` | `float` | 0 - 50 (-20 - 60 en versiones nuevas) | °C | Temperatura |

---

### 2. Funciones Públicas

#### `DHT11_Init()` - Inicialización del Driver

Configura e inicializa el driver DHT11 con el Timer y el pin de datos especificados. Descarta cualquier medición guardada en caché.

```c
DHT11_Status_t DHT11_Init(TIM_HandleTypeDef* htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`htim`|`TIM_HandleTypeDef*`|Puntero al handle del Timer (1 MHz, 1 µs por cuenta, ARR al máximo). Si no está en marcha, la función lo arranca|
|`GPIOx`|`GPIO_TypeDef*`|Puerto GPIO del pin de datos|
|`GPIO_PIN`|`uint16_t`|Pin GPIO del pin de datos|

**Retorna**: `DHT11_OK` si la inicialización fue exitosa, `DHT11_INVALID_PARAM` si `htim`, `htim->Instance` o `GPIOx` es `NULL` o `GPIO_PIN` es `0`, `DHT11_ERROR` si no se pudo arrancar el Timer.

**Secuencia interna:**

1. Validación de parámetros.
2. Arranque del Timer con `HAL_TIM_Base_Start()` si no está en marcha.
3. Registro del Timer y del pin, descarte de la caché y marca del driver como inicializado.

> [!NOTE]
> `DHT11_Init()` no se comunica con el sensor ni aplica la espera de ~1 s tras el encendido; ver [Teoría de inicialización del sensor](#teoría-de-inicialización-del-sensor).

---

#### `DHT11_Get()` - Lectura de Datos

Realiza la secuencia completa de comunicación con el sensor DHT11 y obtiene las mediciones de temperatura y humedad.

```c
DHT11_Status_t DHT11_Get(DHT11_Data_t* environment);
```

|Parámetro|Tipo|Descripción|
|---|---|---|
|`environment`|`DHT11_Data_t*`|Puntero a la estructura donde se guardarán los datos|

**Retorna**: `DHT11_OK` si la lectura fue exitosa (datos válidos en `environment`), `DHT11_NOT_INITIALIZED` si el driver no fue inicializado, `DHT11_INVALID_PARAM` si `environment` es `NULL`, `DHT11_TIMEOUT` si el sensor no responde a tiempo, `DHT11_CHECKSUM_ERROR` si la trama está corrupta, `DHT11_ERROR` si la respuesta del sensor es inválida.

**Secuencia interna:**

1. Si no han pasado `DHT11_MIN_INTERVAL_MS` desde la última lectura válida, devuelve la medición guardada con `DHT11_OK`.
2. Señal de inicio (18 ms en bajo + 20 µs en alto) y cambio del pin a entrada.
3. Verificación de la respuesta del sensor (80 µs bajo + 80 µs alto).
4. Lectura de los 5 bytes de la trama.
5. Verificación del checksum.
6. Conversión a °C y %RH (incluido el signo de temperatura) y actualización de la caché.

> [!CAUTION]
> - El DHT11 necesita ~2 s entre lecturas. Si se llama antes de `DHT11_MIN_INTERVAL_MS` (2000 ms) desde la última lectura válida, no se consulta al sensor y se devuelve la medición guardada con `DHT11_OK`.
> - Requiere inicialización previa con `DHT11_Init()`.
> - Función **bloqueante**: la señal de inicio ocupa 18 ms y la trama de 40 bits unos milisegundos más. La versión anterior de este README indicaba "~4 ms de duración típica", cifra que no incluye la señal de inicio; la duración total real no está medida en esta documentación.

---

#### `DHT11_DeInit()` - Desinicialización del Driver

Libera el Timer y el pin y marca el driver como no inicializado. **No detiene el Timer**, ya que puede estar compartido con otros módulos. Tras llamarla, es necesario invocar nuevamente `DHT11_Init()` antes de realizar lecturas.

```c
DHT11_Status_t DHT11_DeInit(void);
```

**Retorna**: Siempre retorna `DHT11_OK`.

---

## Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.  
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [2.0.0] - 12-11-2026

Alinea la API con la convención de librerías del repo, añade caché de lecturas, soporte de temperaturas negativas y validación de parámetros. Incluye **cambios incompatibles** en la función de lectura, la estructura de datos y los valores de `DHT11_Status_t`.

#### Added
- **Desinicialización**: `DHT11_DeInit()` libera el timer y el pin y marca el driver como no inicializado. No detiene el timer, por si está compartido.
- **Caché de lecturas**: si `DHT11_Get()` se llama antes de `DHT11_MIN_INTERVAL_MS` (2000 ms) desde la última lectura válida, devuelve la medición guardada con `DHT11_OK` sin consultar al sensor. `DHT11_Init()` descarta la caché.
- **Temperaturas negativas**: el bit 7 del byte decimal de temperatura se interpreta como signo (DHT11 de rango -20 °C a 60 °C).
- **Validación de parámetros**: nuevo estado `DHT11_INVALID_PARAM`; `DHT11_Init()` lo devuelve si `htim`, `htim->Instance` o `GPIOx` es `NULL` o `GPIO_PIN` es `0`, y `DHT11_Get()` si `environment` es `NULL`.
- **Arranque del timer**: `DHT11_Init()` arranca el timer con `HAL_TIM_Base_Start()` si no está en marcha y devuelve `DHT11_ERROR` si falla.

#### Changed
- **[BREAKING]** La función de lectura se renombra y cambia de tipo:
  `DHT11_Read_Data(DHT11_Handles_t* values)` → `DHT11_Get(DHT11_Data_t* environment)`.
- **[BREAKING]** `DHT11_Handles_t` pasa a `DHT11_Data_t` y sus campos se renombran:
  `.Temperature` → `.temperatura`, `.Humidity` → `.humedad`.
- **[BREAKING]** Cambian los valores numéricos de `DHT11_Status_t`: `DHT11_NOT_INITIALIZED` pasa de `4` a `3`, `DHT11_CHECKSUM_ERROR` de `3` a `5`, y `4` es ahora `DHT11_INVALID_PARAM`. Solo afecta a código que compare con números en lugar de los nombres.
- **[BREAKING]** `DHT11.h` incluye `main.h` en lugar de `stm32f4xx_hal.h` y `stdio.h`, así funciona en cualquier familia STM32 sin editarlo. Si tu código usaba `printf` gracias a `DHT11.h`, incluye `<stdio.h>` tú mismo.
- Las funciones auxiliares de configuración del pin pasan a ser `static` (ya no se exportan).

#### Fixed
- Desbordamiento del timer en retardos y timeouts: las diferencias se calculan en 16 bits, correcto con timers de 16 y 32 bits si el ARR está al máximo (`0xFFFF` / `0xFFFFFFFF`).

---

### [1.0.0] - 21-01-2026

#### Added
- Versión inicial de la librería DHT11 para STM32.
- Comunicación single-wire con temporización precisa mediante Timers de hardware.
- Funciones públicas: `DHT11_Init()` y `DHT11_Get()`.
- Sistema de gestión de errores con tipo `DHT11_Status_t` (OK, ERROR, TIMEOUT,
  CHECKSUM_ERROR, NOT_INITIALIZED).
- Verificación de integridad de datos mediante checksum de 8 bits.
- Estructura `DHT11_Data_t` para almacenamiento de temperatura y humedad.
- Documentación completa con comentarios Doxygen, ejemplos de uso y diagramas
  de señal del protocolo.
