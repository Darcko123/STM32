# Interrupción por Timer

[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![TIM](https://img.shields.io/badge/Protocol-TIM-yellow)](https://es.wikipedia.org/wiki/Temporizador)

## Tabla de contenidos
- [Interrupción por Timer](#interrupción-por-timer)
  - [Tabla de contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características Principales](#características-principales)
  - [Componentes Requeridos](#componentes-requeridos)
  - [Configuración del Proyecto en STM32CubeIDE](#configuración-del-proyecto-en-stm32cubeide)
    - [1. Crear un Nuevo Proyecto](#1-crear-un-nuevo-proyecto)
    - [2. Configuración de los Timers](#2-configuración-de-los-timers)
    - [3. Cálculo de los Parámetros del Timer](#3-cálculo-de-los-parámetros-del-timer)
      - [a) Determinación de la Frecuencia de Interrupción](#a-determinación-de-la-frecuencia-de-interrupción)
      - [b) Fuente de Reloj (Clock Source)](#b-fuente-de-reloj-clock-source)
      - [c) Procedimiento de Cálculo](#c-procedimiento-de-cálculo)
      - [d) Ejemplo alternativo (STM32F103)](#d-ejemplo-alternativo-stm32f103)
      - [e) Configuración Final](#e-configuración-final)
    - [4. Configuración de Pines](#4-configuración-de-pines)
    - [5. Generar el Código](#5-generar-el-código)
    - [6. Conexiones Físicas](#6-conexiones-físicas)
  - [Implementación de Interrupción](#implementación-de-interrupción)
    - [Parámetros de `HAL_TIM_Base_Start_IT()`:](#parámetros-de-hal_tim_base_start_it)
    - [Callback: `HAL_TIM_PeriodElapsedCallback()`](#callback-hal_tim_periodelapsedcallback)
  - [Licencia](#licencia)

---

## Descripción

Este proyecto implementa un sistema de parpadeo (*blink*) para **tres LEDs independientes**, utilizando **interrupciones generadas por los temporizadores** `TIM2`, `TIM3` y `TIM4` en una placa de desarrollo **STM32F411CEU6 (Black Pill)**.  

El propósito de este tutorial es guiar paso a paso en la configuración de los temporizadores en **modo interrupción**, cada uno operando a diferentes frecuencias, así como la generación del código base mediante **STM32CubeIDE**.  
Además, se explica el uso de las funciones de la **HAL (Hardware Abstraction Layer)** para implementar el control de los LEDs utilizando eventos temporizados.

---

## Características Principales

- ✅ Configuración del reloj **HSE** para operación a máxima velocidad.  
- ✅ Habilitación del modo **Serial Wire Debug (SWD)** para depuración.  
- ✅ Configuración de **GPIOs** para controlar tres salidas digitales (LEDs).  
- ✅ Configuración de **Timers** en modo **interrupción periódica**.  
- ✅ Explicación detallada del uso de funciones HAL como `HAL_TIM_Base_Start_IT()` y del manejo de interrupciones mediante `HAL_TIM_PeriodElapsedCallback()`.  
- ✅ Ejemplo funcional de ejecución concurrente de múltiples temporizadores con diferentes frecuencias.

---

## Componentes Requeridos

- **STM32CubeIDE** – [Descargar desde STMicroelectronics](https://www.st.com/en/development-tools/stm32cubeide.html)  
- **Placa de desarrollo**: STM32F411CEU6 (Black Pill) o compatible.  
- **Programador/depurador**: ST-Link V2.  
- **3 LEDs** (cualquier color).  
- **3 resistencias de 330 Ω** (para limitar corriente). 
  
> [!TIP]  
> Es recomendable conectar cada LED a un pin GPIO distinto y emplear resistencias en serie para proteger las salidas del microcontrolador.  
> Este ejemplo puede adaptarse fácilmente a otros microcontroladores de la familia **STM32F4xx** modificando los timers disponibles y los pines asignados. 

---

## Configuración del Proyecto en STM32CubeIDE

### 1. Crear un Nuevo Proyecto

Siga los pasos del tutorial [Blink](/PerifericosBasicos/HelloWorld/README.md) para crear un nuevo proyecto destinado al microcontrolador `STM32F411CEU6`, incluyendo la configuración del reloj principal del sistema (HSE y PLL).

---

### 2. Configuración de los Timers

Para la configuración de los temporizadores, acceda a:  
`Pinout & Configuration` → `Timers` → `TIM2`.

- En la opción **Clock Source**, cambie de `Disable` a `Internal Clock`.  
- En el caso de `TIM3` y `TIM4`, seleccione directamente **Internal Clock**, ya que no disponen de una fuente de reloj configurable adicional.

Cada temporizador controlará un LED diferente, operando a una frecuencia específica.  
Para lograr esto, se deben ajustar los valores de **Prescaler (PSC)** y **Auto-Reload (ARR)** según la frecuencia deseada.

---



### 3. Cálculo de los Parámetros del Timer

La frecuencia de actualización del timer, es decir, la frecuencia con la que se genera la interrupción, se calcula mediante la siguiente fórmula:

$F_{out} = \frac{F_{clk}}{(Prescaler + 1) \times (CounterPeriod + 1)}$

**Donde:**
- $F_{out}$: Frecuencia de salida deseada (Hz).  
- $F_{clk}$: Frecuencia del reloj del timer (Hz).  
- `Prescaler`: Valor del divisor de frecuencia (PSC).  
- `Counter Period`: Valor de recarga automática (ARR).

---

#### a) Determinación de la Frecuencia de Interrupción

El tiempo entre interrupciones depende del parpadeo deseado para cada LED:

| LED | Periodo (T) | Frecuencia (f) |
|-----|--------------|----------------|
| LED 1 | 200 ms | 5 Hz |
| LED 2 | 500 ms | 2 Hz |
| LED 3 | 750 ms | 1.33 Hz |

Recordando que:

$f = \frac{1}{T}$

---

#### b) Fuente de Reloj (Clock Source)

Los temporizadores `TIM2`, `TIM3` y `TIM4` del **STM32F411CEU6** se alimentan desde el bus **APB1**.

![Bus de Datos](/PerifericosBasicos/TimerInterrupt/Images/BusDatos.png)

El bus **APB1** tiene una frecuencia máxima de **50 MHz**, pero los temporizadores asociados pueden recibir el doble de esa frecuencia (**100 MHz**) cuando el prescaler del bus es distinto de 1.

![Timer Clocks](/PerifericosBasicos/TimerInterrupt/Images/TimerClocks.png)

> [!NOTE]  
> La frecuencia del reloj de los timers varía entre modelos de microcontroladores.  
> Consulte siempre la hoja de datos (datasheet) correspondiente para obtener la información exacta de su dispositivo.

---

#### c) Procedimiento de Cálculo

Dado que se conoce $F_{out}$ y $F_{clk} = 100\,MHz$, se puede despejar el valor de $CounterPeriod$:

$(CounterPeriod + 1) = \frac{F_{clk}}{F_{out} \times (Prescaler + 1)}$

Para simplificar los cálculos, fijamos:

$(Prescaler + 1) = 10000$

Sustituyendo los valores:

| LED | $F_{out}$ | $(CounterPeriod + 1)$ | Valor aproximado |
|------|----------------|--------------------------|------------------|
| LED 1 | 5 Hz | $\frac{100\,MHz}{(10000)(5)}$ | 2000 − 1 |
| LED 2 | 2 Hz | $\frac{100\,MHz}{(10000)(2)}$ | 5000 − 1 |
| LED 3 | 1.33 Hz | $\frac{100\,MHz}{(10000)(1.33)}$ | 7500 − 1 |

---

#### d) Ejemplo alternativo (STM32F103)

Para ilustrar la diferencia entre familias, consideremos el microcontrolador `STM32F103`, cuya frecuencia máxima de reloj para los timers es **72 MHz**.  
Supongamos que fijamos el valor de `Counter Period` y deseamos calcular el **Prescaler**:

$Prescaler = \frac{F_{clk}}{F_{out} \times (CounterPeriod + 1)} - 1$

Con un `Counter Period` de **10,000**:

| LED | $F_{out}$ | $(Prescaler + 1)$ | Valor aproximado |
|------|----------------|----------------------|------------------|
| LED 1 | 5 Hz | $\frac{72\,MHz}{(10000)(5)}$ | 1440 − 1 |
| LED 2 | 2 Hz | $\frac{72\,MHz}{(10000)(2)}$ | 3600 − 1 |
| LED 3 | 1.33 Hz | $\frac{72\,MHz}{(10000)(1.33)}$ | 5400 − 1 |

---

#### e) Configuración Final

| Parámetro | **TIM2** | **TIM3** | **TIM4** |
|------------|-----------|-----------|-----------|
| **Prescaler** | 10000 − 1 | 10000 − 1 | 10000 − 1 |
| **Counter Mode** | Up | Up | Up |
| **Counter Period** | 2000 − 1 | 5000 − 1 | 7500 − 1 |

---

> [!TIP]  
> Es recomendable mantener valores de **Prescaler** y **Counter Period** dentro de los límites de 16 bits (0–65535) cuando sea posible, para garantizar compatibilidad con todos los timers de la serie **STM32F4xx**.  
> Además, recuerde verificar en **Clock Configuration** que el bus APB1 se encuentre correctamente configurado para suministrar la frecuencia de reloj esperada a los temporizadores.

---

### 4. Configuración de Pines

Se deben seleccionar tres pines distintos configurados como **salida digital** para controlar los LEDs.  
Deje la configuración predeterminada de los GPIO, asegurándose de que el modo esté establecido en **Output Push-Pull**.

En este ejemplo, se seleccionan los siguientes pines del puerto **GPIOB**:
- `PB12`
- `PB13`
- `PB14`

De esta forma, el *pinout* del proyecto quedará como se muestra a continuación:

![Pinout](/PerifericosBasicos/TimerInterrupt/Images/Pinout.png)

### 5. Generar el Código

Una vez realizada toda la configuración, haga clic en el ícono de la **llave amarilla** para generar el código base del proyecto.

![Generar_Codigo](/PerifericosBasicos/HelloWorld/images/CodeGeneration.png)

### 6. Conexiones Físicas

Cada LED debe conectarse a su respectivo pin GPIO con una resistencia limitadora de 330 Ω en serie.  
Por ejemplo:

| LED | Pin GPIO | Resistencia | Conexión |
|------|-----------|--------------|-----------|
| LED 1 | PB12 | 330 Ω | Ánodo → PB12, Cátodo → GND |
| LED 2 | PB13 | 330 Ω | Ánodo → PB13, Cátodo → GND |
| LED 3 | PB14 | 330 Ω | Ánodo → PB14, Cátodo → GND |

> [!TIP]  
> Es recomendable emplear resistencias entre 220 Ω y 470 Ω dependiendo de la intensidad de brillo deseada del LED.  
> Si el LED no enciende, verifique la polaridad y la configuración de salida del pin correspondiente.

---

## Implementación de Interrupción

A continuación se presenta el código principal del programa:

``` C
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);    // Inicia TIM2 en modo interrupción

  HAL_TIM_Base_Start_IT(&htim3);    // Inicia TIM3 en modo interrupción

  HAL_TIM_Base_Start_IT(&htim4);    // Inicia TIM4 en modo interrupción
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Callback ejecutado cuando un Timer genera una interrupción.
  * @param  htim: puntero al manejador del Timer que generó la interrupción.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Instance == TIM2)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	}

	if(htim -> Instance == TIM3)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	}

	if(htim -> Instance == TIM4)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	}
}
/* USER CODE END 4 */
```

### Parámetros de `HAL_TIM_Base_Start_IT()`:

Esta función inicializa el Timer en modo interrupción y comienza su conteo.

```C
HAL_TIM_Base_Start_IT(
   *htim,     //Puntero al Handler del Timer
);
```

Cuando el contador del Timer alcanza el valor de Auto-Reload (ARR) configurado, se genera una interrupción que activa el callback correspondiente.

### Callback: `HAL_TIM_PeriodElapsedCallback()`

La función `HAL_TIM_PeriodElapsedCallback()` se ejecuta automáticamente cada vez que el Timer alcanza su valor máximo y provoca una interrupción.

Dentro de esta función:
- Se compara el puntero `htim` con la instancia (`Instance`) de cada Timer (`TIM2`, `TIM3`, `TIM4`).
- Dependiendo del Timer que generó la interrupción, se alterna el estado del LED correspondiente mediante la función `HAL_GPIO_TogglePin()`.

Este enfoque permite controlar múltiples eventos temporizados en paralelo utilizando un único callback.

> [!TIP]
> Dentro de esta función también pueden implementarse otras tareas, como actualizar variables, iniciar conversiones ADC, enviar datos por UART o activar otras rutinas periódicas.


## Licencia

Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.