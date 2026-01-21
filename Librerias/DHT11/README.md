# DHT11 Library

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/DHT11)

---

## Tabla de Contenidos
- [DHT11 Library](#dht11-library)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Requisitos](#requisitos)
  - [Datos del Sensor](#datos-del-sensor)
    - [Rangos operativos](#rangos-operativos)
    - [Funcionamiento del Protocolo](#funcionamiento-del-protocolo)
      - [Secuencia de Inicialización](#secuencia-de-inicialización)
      - [Transmisión de Datos y Formato de Trama](#transmisión-de-datos-y-formato-de-trama)
      - [Secuencia de Comunicación](#secuencia-de-comunicación)
  - [Ejemplo de Implementación](#ejemplo-de-implementación)
    - [1. Configuración del Timer para Temporización Precisa](#1-configuración-del-timer-para-temporización-precisa)
      - [Ejemplo para `STM32F411` a `100MHz`](#ejemplo-para-stm32f411-a-100mhz)
    - [2. Configuración del Pin de Datos GPIO](#2-configuración-del-pin-de-datos-gpio)
    - [Declaración e Inicialización del Driver](#declaración-e-inicialización-del-driver)
    - [Ejemplo completo de uso](#ejemplo-completo-de-uso)
  - [API Reference](#api-reference)
    - [1. Tipos de Datos](#1-tipos-de-datos)
      - [`DHT11_Status_t` - Estados de Retorno](#dht11_status_t---estados-de-retorno)
      - [`DHT11_Handles_t` - Estructura de Mediciones](#dht11_handles_t---estructura-de-mediciones)
    - [2. Funciones del Driver](#2-funciones-del-driver)
      - [`DHT11_Init()` - Inicialización del Driver](#dht11_init---inicialización-del-driver)
      - [`DHT11_Read_Data()` - Lectura de Mediciones](#dht11_read_data---lectura-de-mediciones)
  - [📄 Licencia](#-licencia)
  - [Changelog](#changelog)
    - [Versión 1.0.0](#versión-100)

## Descripción
Librería desarrollada en C para la interfaz con el sensor **DHT11** mediante comunicación digital de un solo cable (*single-wire*), implementando temporización precisa mediante Timers de hardware. Este driver permite monitorear las condiciones ambientales obteniendo mediciones de temperatura y humedad relativa en sistemas embebidos basados en microcontroladores STM32.

---

## Características
- **Comunicación single-wire**: Protocolo digital implementado en un único pin de datos.
- **Temporización no bloqueante**: Utilización de Timers de hardware para garantizar tiempos precisos sin ocupar la CPU.
- **Gestión robusta de errores**: Sistema completo con códigos de retorno específicos para timeout, checksum y estados de inicialización.
- **Documentación profesional**: Código documentado con estándar Doxygen, facilitando mantenimiento e integración.
- **Portabilidad**: Diseñada para ser compatible con múltiples familias STM32 mediante la capa HAL.

---

## Requisitos

- STM32CubeIDE o STM32CubeMX.
- Biblioteca HAL correspondiente a tu microcontrolador STM32.
- Módulo DHT11 conectado a cualquier pin digital.

## Datos del Sensor

### Rangos operativos
| Característica | Rango |
|----------------|-------|
| Rango de Temperatura | 0°C to 50°C |
| Precisión de Temperatura | ±2°C |
| Rango de Humedad | 20% a 90% RH |
| Precisión de Humedad | ±5% RH |
| Sampling Rate | 1 lectura por segundo (1Hz) |
| Voltage Operativo | 3.3V a 5V |
| Data Format | Digital (1-wire) |

### Funcionamiento del Protocolo

#### Secuencia de Inicialización

El proceso de inicialización del DHT11 sigue un protocolo maestro-esclavo. El microcontrolador (*maestro*) inicia la comunicación configurando el pin de datos como salida y generando un pulso de inicio (*start pulse*): nivel bajo durante 18 ms seguido de un nivel alto durante 20 µs, tras lo cual cambia a modo entrada.

El sensor responde con una señal de reconocimiento (*acknowledge signal*): un pulso bajo de 80 µs seguido de un pulso alto de 80 µs, indicando que está listo para la transmisión de datos.

**Señal de inicio generada por el microcontrolador:**

![StartSignal](/Librerias/DHT11/Images/StartSignal.jpg)

```C
static DHT11_Status_t DHT11_Start_Communication(void)
{
	Set_DHT11Pin_Output();							// Configurar el pin como salida
	HAL_GPIO_WritePin(DHT11_Port, DHT11_Pin, 0);	// Pin bajo
	delayUs(DHT11_START_SIGNAL_US);					// Esperar por 18 ms		
	HAL_GPIO_WritePin(DHT11_Port, DHT11_Pin, 1);	// Pin alto
	delayUs(DHT11_START_WAIT_US);					// Esperar 20 us
	Set_DHT11Pin_Input();							// Configurar el pin como entrada

	delayUs(2);		// Pequeña espera para estabilización del pin

	return DHT11_OK;
}
```

**Señal de respuesta del sensor DHT11:**

![ResponseSignal](/Librerias/DHT11/Images/ResponseSignal.jpg)

```C
static DHT11_Status_t DHT11_Check_Response(void)
{   
	uint32_t start = 0;

	// Esperar señal de bajo inicial (sensor toma control del bus)
	start = __HAL_TIM_GET_COUNTER(DHT11_tim);
	while(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))		// Esperar mientras esté ALTO
	{
		if((__HAL_TIM_GET_COUNTER(DHT11_tim) - start) > DHT11_RESPONSE_TIMEOUT_US)
			return DHT11_TIMEOUT;
	}

	// Verificar primer pulso bajo (~80μs)
	if(!(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin)))
	{
		// Esperar transición a alto (~80μs)
		start = __HAL_TIM_GET_COUNTER(DHT11_tim);
		while(!HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))
		{
			if((__HAL_TIM_GET_COUNTER(DHT11_tim) - start) > DHT11_RESPONSE_TIMEOUT_US)
				return DHT11_TIMEOUT;
		}

		// Esperar segundo pulso bajo (~80μs)
		start = __HAL_TIM_GET_COUNTER(DHT11_tim);
		while(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))
		{
			if((__HAL_TIM_GET_COUNTER(DHT11_tim) - start) > DHT11_RESPONSE_TIMEOUT_US)
				return DHT11_TIMEOUT;
		}

		return DHT11_OK;
	}

	return DHT11_ERROR;
}
```

**Resumen de la secuencia de inicialización:**

- Pulso de inicio del maestro: Nivel bajo por 18 ms, luego alto por 20 µs
- Cambio a modo entrada: Microcontrolador libera el bus para el sensor
- Respuesta del sensor: Pulso bajo (80 µs) + pulso alto (80 µs)
- Verificación temporal: Timeout de 100 µs para detección de fallos

> [!NOTE]
> En configuraciones con líneas de datos largas o alta impedancia, se recomienda conectar una resistencia de pull-up de 4.7 kΩ entre VCC y el pin de datos para garantizar niveles lógicos adecuados durante la fase de alta impedancia.

#### Transmisión de Datos y Formato de Trama

Tras la secuencia de inicialización, el DHT11 transmite una trama de 40 bits (5 bytes) con codificación Manchester-like. Cada bit se representa mediante un pulso bajo de referencia de 50 µs seguido de un pulso alto cuya duración codifica el valor:

- **Bit '0'**: Pulso alto de 26-28 µs
- **Bit '1'**: Pulso alto de aproximadamente 70 µs

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
| 4    | Temperatura Decimal | Parte decimal de temperatura (°C) |
| 5    | Checksum | Suma de verificación (8 bits LSB) |

La verificación de integridad se realiza mediante el byte de checksum, que corresponde a los 8 bits menos significativos de la suma de los primeros cuatro bytes:

$Checksum=(HumedadEntera+HumedadDecimal+TemperaturaEntera+TemperaturaDecimal) mod 256$

```C
static DHT11_Status_t DHT11_Read_Byte(uint8_t* byte)
{
	uint8_t i = 0;
	uint32_t start;

	if (byte == NULL)
		return DHT11_ERROR;

	for (uint8_t j = 0; j < 8; j++)
	{
		// Esperar inicio del bit (transición a alto)
		start = __HAL_TIM_GET_COUNTER(DHT11_tim);
		while(!(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin)))
		{
			if((__HAL_TIM_GET_COUNTER(DHT11_tim) - start) > DHT11_BIT_TIMEOUT_US)
				return DHT11_TIMEOUT;
		}

		// Esperar tiempo crítico para diferenciar 0 de 1
		delayUs(DHT11_BIT_READ_DELAY_US);

		// Determinar valor del bit según nivel del pin
		if(!(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin)))
		{
			i &= ~(1 << (7 - j));	// Escribe 0
		}
		else
		{
			i |=  (1 << (7 - j));   // Escribe 1
		}

		// Esperar fin del bit (transición a bajo)
		start = __HAL_TIM_GET_COUNTER(DHT11_tim);
		while(HAL_GPIO_ReadPin(DHT11_Port, DHT11_Pin))
		{
			if((__HAL_TIM_GET_COUNTER(DHT11_tim) - start) > DHT11_BIT_TIMEOUT_US)
				return DHT11_TIMEOUT;
		}
	}

	*byte = i;
	return DHT11_OK;
}
```

#### Secuencia de Comunicación

```C
    // --- SECUENCIA DE COMUNICACIÓN ---

	// 1. Iniciar comunicación
	status = DHT11_Start_Communication();
	if (status != DHT11_OK)
		return status;

	// 2. Verificar respuesta del sensor
	status = DHT11_Check_Response();
	if (status != DHT11_OK)
		return status;

	// 3. Leer los 5 bytes de datos
	status = DHT11_Read_Byte(&humedad_entero);
	if (status != DHT11_OK)
		return status;

	status = DHT11_Read_Byte(&humedad_decimal);
	if (status != DHT11_OK)
		return status;

	status = DHT11_Read_Byte(&temp_entero);
	if (status != DHT11_OK)
		return status;

	status = DHT11_Read_Byte(&temp_decimal);
	if (status != DHT11_OK)
		return status;

	status = DHT11_Read_Byte(&checksum);
	if (status != DHT11_OK)
		return status;

	// 4. Verificar integridad de datos mediante checksum
	if ((uint8_t)(humedad_entero + humedad_decimal + temp_entero + temp_decimal) != checksum)
		return DHT11_CHECKSUM_ERROR;

	// 5. Convertir bytes a valores flotantes
	values->Humidity = (float)humedad_entero + ((float)humedad_decimal / 10.0f);
	values->Temperature = (float)temp_entero + ((float)temp_decimal / 10.0f);
```

## Ejemplo de Implementación

### 1. Configuración del Timer para Temporización Precisa

La comunicación con el DHT11 requiere medición precisa de intervalos en microsegundos. Para esto, debe configurarse un periférico Timer en modo contador (counter mode) con resolución de 1 µs.

**Cálculo de parámetros del Timer:**
La frecuencia del contador del Timer se determina mediante:
$$
f_{timer} = \frac{f_{clk}}{(Prescaler + 1)}
$$

Donde:

- $f_{clk}$: Frecuencia del reloj del sistema (System Clock)
- $Prescaler$: Valor del prescaler (Prescaler)
- $f_{timer}$: Frecuencia resultante del contador
​
 : Frecuencia resultante del contador

Para lograr una resolución de 1 µs (1 MHz), el prescaler se calcula como:

$$
Prescaler = \frac{f_{CLK}}{1 MHz} - 1
$$

#### Ejemplo para `STM32F411` a `100MHz`
$$
Prescaler = \frac{100MHz}{1 MHz} - 1 = 99
$$

El período del contador (Counter Period) debe configurarse al valor máximo para evitar desbordamientos durante las mediciones. Para un Timer de 32 bits:

$$
Counter Period=2^{32} −1=0xFFFFFFFF
$$

**Configuración recomendada**

| Característica       | Valor                |
|----------------------|----------------------|
| Timer                | TIM2                 |
| Clock Source         | Internal Clock       |
| Prescaler            | 99                   |
| Counter Mode         | UP                   |
| Counter Period       | 0xFFFFFFFF           |
| Internal Clock Division       | NO DIVISION |

Esta configuración permite mediciones de hasta $2^{32}$ µs (aproximadamente 71.5 minutos) sin desbordamiento, suficiente para todas las fases de comunicación del DHT11.

### 2. Configuración del Pin de Datos GPIO

El pin de datos requiere configuración dinámica como salida push-pull durante la fase de inicialización y como entrada con pull-up durante la recepción de datos. La librería maneja automáticamente estos cambios de modo.

**Ejemplo de configuración para PA1 en STM32CubeMX: **

|Parámetro | Valor Inicial | Nota |
|----------|----------------|------|
| GPIO Mode | Output Push Pull | Modo inicial |
| GPIO Pull-up/Pull-down | Pull-up | Mantiene nivel alto cuando en entrada |
| Maximum Output Speed | Low | Suficiente para protocolo DHT11 |
| User Label | DHT11_DATA | Para identificación en código |

> [!NOTE]
> La librería modificará dinámicamente el modo del pin entre salida y entrada según la fase de comunicación.

### Declaración e Inicialización del Driver

Declare una variable de tipo `DHT11_Handles_t` para almacenar las mediciones y llame a la función de inicialización con los parámetros adecuados:

```C
/* USER CODE BEGIN PV */
DHT11_Handles_t Environment;
/* USER CODE END PV */
```

### Ejemplo completo de uso

```C
/* USER CODE BEGIN PV */

DHT11_Handles_t Environment;

/* USER CODE END PV */

int main()
{
    /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start(&htim2);

    if(DHT11_Init(&htim2, GPIOB, GPIO_PIN_9) != DHT11_OK)
    {
        // Led de usuario enciende para indicar error
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        while(1);
    }

    HAL_Delay(1000);	// Delay para estabilización
    /* USER CODE END 2 */

    /* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
        // Lectura de sensor
		DHT11_Status_t status = DHT11_Read_Data(&Environment);

		if(status != DHT11_OK)
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

## API Reference

### 1. Tipos de Datos

#### `DHT11_Status_t` - Estados de Retorno

Enumeración que define todos los códigos de retorno posibles para las operaciones del driver DHT11.

```C
/**
 * @brief Enumeración para estados de retorno del DHT11.
 */
typedef enum {
	DHT11_OK = 0,				/** Operación exitosa */
	DHT11_ERROR = 1,			/** Error en la operación */
	DHT11_TIMEOUT = 2,			/** Timeout en la operación */
	DHT11_CHECKSUM_ERROR = 3,	/** Error en el checksum */
	DHT11_NOT_INITIALIZED = 4	/** Sensor no inicializado */
}DHT11_Status_t;
```

#### `DHT11_Handles_t` - Estructura de Mediciones
Estructura que almacena las mediciones de temperatura y humedad obtenidas del sensor.

```C
/*
 * @brief Estructura para almacenar mediciones del sensor DHT11.
 * @note Temperatura en grados Celsius (°C), humedad en porcentaje (%RH).
 */
typedef struct {
    float Temperature;          /**< Valor de temperatura en °C */
    float Humidity;             /**< Valor de humedad relativa en % */
} DHT11_Handles_t;
```

### 2. Funciones del Driver

---

#### `DHT11_Init()` - Inicialización del Driver
Configura e inicializa el driver DHT11 con los periféricos hardware especificados.

```C
DHT11_Status_t DHT11_Init(
    TIM_HandleTypeDef* htim, 
    GPIO_TypeDef* GPIOx, 
    uint16_t GPIO_PIN
    )
```

**Parámetros:**
- `htim`: Puntero a la estructura del manejador TIM.
- `GPIO_TypeDef`: Puntero al puerto GPIO.
- `GPIO_PIN`: Pin GPIO del puerto.

**Retorna:** 
- `DHT11_Status_t` indicando el estado de la operación.

---

#### `DHT11_Read_Data()` - Lectura de Mediciones

Realiza la secuencia completa de comunicación con el sensor DHT11 y obtiene las mediciones de temperatura y humedad.

```C
DHT11_Status_t DHT11_Read_Data(
    DHT11_Values* values
    );
```

**Parámetros:**
- `values`: Puntero a la estructura `DHT11_Values` donde se almacenarán los datos leídos.

**Retorna:** 
- `DHT11_OK`: Lectura exitosa, datos válidos en values
- `DHT11_NOT_INITIALIZED`: Driver no inicializado previamente
- `DHT11_TIMEOUT`: Sensor no responde dentro del tiempo esperado
- `DHT11_CHECKSUM_ERROR`: Datos corruptos (checksum inválido)
- `DHT11_ERROR`: Error durante la comunicación

>[!CAUTION]
>- No debe llamarse más frecuentemente que cada 1 segundo (limitación del DHT11)
>- Requiere inicialización previa con DHT11_Init()
>- Función bloqueante (~4ms de duración típica)
---

## 📄 Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog

### Versión 1.0.0
- Versión inicial de la librería DHT11 para STM32.
- Implementación de funciones básicas para inicialización y lectura de datos.
- Sistema de gestión de errores.
- Documentación completa con ejemplos de uso.