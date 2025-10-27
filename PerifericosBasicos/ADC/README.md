# ADC en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![ADC](https://img.shields.io/badge/Protocol-ADC-yellow)](https://es.wikipedia.org/wiki/Convertidor_anal%C3%B3gico-digital)

Este documento presenta una guía completa para la implementación del **ADC (Analog-to-Digital Converter)** en microcontroladores STM32, utilizando **STM32CubeIDE**.
Incluye explicaciones detalladas sobre la configuración del periférico y ejemplos prácticos que facilitan la comprensión y aplicación del ADC en proyectos embebidos. Además, se realiza una comparación entre los modos de operación bloqueante, no bloqueante e interrupción.

---

## Índice

- [ADC en STM32](#adc-en-stm32)
  - [Índice](#índice)
  - [Descripción General](#descripción-general)
  - [Características Principales](#características-principales)
  - [Componentes Requeridos](#componentes-requeridos)
  - [Polling](#polling)
    - [Ventajas](#ventajas)
    - [Configuración del Proyecto en STM32CubeIDE](#configuración-del-proyecto-en-stm32cubeide)
      - [1. Crear un Nuevo Proyecto](#1-crear-un-nuevo-proyecto)
      - [2. Configuración de pines](#2-configuración-de-pines)
        - [ADC Scan Conversion Mode](#adc-scan-conversion-mode)
        - [ADC Continouse Conversion Mode](#adc-continouse-conversion-mode)
      - [3. Generar el Código](#3-generar-el-código)
    - [Implementación de Recepción de Datos](#implementación-de-recepción-de-datos)
      - [Explicación del Funcionamiento de la Lógica dentro del `while`](#explicación-del-funcionamiento-de-la-lógica-dentro-del-while)
        - [1. Inicio de la Conversión ADC](#1-inicio-de-la-conversión-adc)
        - [2. Epera por la Finalización de la conversión (Polling)](#2-epera-por-la-finalización-de-la-conversión-polling)
        - [3. Lectura del Valor Digital Convertido](#3-lectura-del-valor-digital-convertido)
        - [4. Detención del ADC](#4-detención-del-adc)
  - [Interrupción](#interrupción)
    - [Configuración del Proyecto en STM32CubeIDE](#configuración-del-proyecto-en-stm32cubeide-1)
      - [1. Crear un Nuevo Proyecto](#1-crear-un-nuevo-proyecto-1)
      - [2. Configuración de Pines](#2-configuración-de-pines-1)
      - [3. Generar el Código](#3-generar-el-código-1)
    - [Implementación de Recepción de Datos](#implementación-de-recepción-de-datos-1)
  - [DMA](#dma)
    - [Configuración del Proyecto en STM32CubeIDE](#configuración-del-proyecto-en-stm32cubeide-2)
      - [1. Crear un Nuevo Proyecto](#1-crear-un-nuevo-proyecto-2)
      - [2. Configuración de Pines](#2-configuración-de-pines-2)
      - [3. Generar el Código](#3-generar-el-código-2)
    - [Implementación de Recepción de Datos](#implementación-de-recepción-de-datos-2)
      - [Explicación detallada](#explicación-detallada)
  - [Interrupción vs DMA](#interrupción-vs-dma)
  - [ADC Multicanal DMA Circular](#adc-multicanal-dma-circular)
    - [Configuración del Proyecto en STM32CubeIDE](#configuración-del-proyecto-en-stm32cubeide-3)
      - [1. Crear un Nuevo Proyecto](#1-crear-un-nuevo-proyecto-3)
      - [2. Configuración de Pines](#2-configuración-de-pines-3)
    - [Configuración del DMA](#configuración-del-dma)
      - [3. Generar el Código](#3-generar-el-código-3)
    - [Implementación de Recepción de Datos](#implementación-de-recepción-de-datos-3)
      - [Explicación del Funcionamiento](#explicación-del-funcionamiento)
      - [Nota personal](#nota-personal)
  - [Distribución](#distribución)
  - [Licencia](#licencia)

## Descripción General
El objetivo de este tutorial es guiar paso a paso en la configuración del entorno de desarrollo, la generación del código base mediante **STM32CubeIDE**, y la implementación de diferentes métodos de uso del ADC utilizando las funciones de la capa de abstracción de hardware (HAL).

---

## Características Principales

- ✅ Configuración completa del protocolo ADC.
- ✅ Ejemplos funcionales de recepción.
- ✅ Comparación entre modos bloqueantes, no bloqueantes e interrupciones.
- ✅ Compatible con cualquier microcontrolador STM32 que disponga de ADC.
---

## Componentes Requeridos

- **STM32CubeIDE**: [Descargar desde STMicroelectronics](https://www.st.com/en/development-tools/stm32cubeide.html)
- **Placa de desarrollo**: STM32F411CEU6 (Black Pill) o compatible.  
- **Programador/depurador**: ST-Link V2.  
- **Fuente de alimentación**: 5 V.
- **Convertidor TTL a USB** (cualquier modelo es válido).
- **Potenciometros**.

Ejemplo de convertidor sugerido:

![TTL_A_USB](/PerifericosBasicos/UART/images/TTL_USB.png)

## Polling

El modo **Polling** del ADC consiste en que el microcontrolador inicia una conversión y espera hasta que esta finalice. Una vez completada, se lee el valor digital resultante del ADC.  
Este método es sencillo de implementar; sin embargo, es bloqueante, ya que mantiene ocupado al CPU durante el proceso de espera. Por esta razón, no es la opción más eficiente para aplicaciones en tiempo real.

### Ventajas
El modo **Polling** es fácil de configurar y resulta adecuado cuando únicamente se requiere realizar la lectura de un canal analógico.

### Configuración del Proyecto en STM32CubeIDE

#### 1. Crear un Nuevo Proyecto
Siga los pasos del tutorial [Blink](/PerifericosBasicos/HelloWorld/README.md) para crear un nuevo proyecto para el `STM32F411CEU6`, incluyendo la configuración de reloj.

#### 2. Configuración de pines
Para configurar el pin correspondiente, acceda a:  
`Pinout & Configuration` → `Analog` → `ADC1` → **Habilitar canal 0** (`IN0`).

Deje las configuraciones con los valores predeterminados, es decir:

**ADCs Common Settings**
- `Mode`: Independent Mode

**ADC Settings**
- `Data Alignment`: Right alignment
- `Scan Conversion Mode`: Disable 
- `Continous Conversion Mode`: Disable 
- `Discontinous Conversion Mode`: Disable

##### ADC Scan Conversion Mode
Este modo permite escanear un grupo de canales analógicos. Se activa automáticamente cuando se configuran múltiples canales.  
Se realiza una conversión por cada canal del grupo; tras finalizar una conversión, el ADC pasa automáticamente al siguiente canal.  
Si el `Continuous Conversion Mode` está habilitado, el proceso no se detiene en el último canal, sino que vuelve al primero y repite el ciclo de forma continua.

##### ADC Continouse Conversion Mode
En el modo `Continuous Conversion Mode`, el ADC inicia una nueva conversión inmediatamente después de que la anterior haya finalizado.  
Este método es más eficiente cuando se requiere realizar conversiones continuas. En el caso de un solo canal, este se convertirá repetidamente de forma automática.

> [!NOTE]  
> El ADC del microcontrolador **STM32F103** no cuenta con la opción para configurar la resolución. Por defecto, trabaja a **12 bits**.

![Polling](/PerifericosBasicos/ADC/Images/ConfPolling.png)

De forma predeterminada, se utiliza el pin **ADC0** como entrada del ADC.

La visualización de la información se realizará mediante **UART**. La configuración de los pines puede consultarse en el apartado [UART](/PerifericosBasicos/UART/README.md).

Por lo tanto, el *pinout* del proyecto quedaría como se muestra a continuación:  
![PinoutPolling](/PerifericosBasicos/ADC/Images/PinoutPolling.png)

#### 3. Generar el Código
Haga clic en el icono de la **llave amarilla** para generar el código base.  
![Generar_Codigo](/PerifericosBasicos/HelloWorld/images/CodeGeneration.png)

### Implementación de Recepción de Datos

```C
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint32_t ADC_VAL;

char buffer[15];
/* USER CODE END PV */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 2 */
    HAL_ADC_Start(&hadc1);
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while(1)
    {
        HAL_ADC_Start(&hadc1);
	      if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
	      {
            ADC_VAL = HAL_ADC_GetValue(&hadc1);
	      }
	      HAL_ADC_Stop(&hadc1);

	      sprintf(buffer, "ADC: %lu\r\n", ADC_VAL);

        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

        HAL_Delay(500);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

```

Resultado:

![ResultadoPolling](/PerifericosBasicos/ADC/Images/ResultPolling.png)

#### Explicación del Funcionamiento de la Lógica dentro del `while`
Dentro del bucle principal while(1), se ejecuta continuamente el proceso de conversión analógico-digital (ADC) mediante el modo Polling. A continuación, se detalla cada paso de la lógica implementada:

##### 1. Inicio de la Conversión ADC
```C
HAL_ADC_Start(&hadc1);
```
- **Propósito**: Inicia el proceso de conversión del ADC en el periférico `ADC1`.

- **Funcionamiento**: Esta función prepara al ADC para comenzar una nueva conversión. En este modo, el ADC espera una señal de inicio para realizar la conversión del voltaje analógico presente en el pin configurado (en este caso, el canal IN0).

##### 2. Epera por la Finalización de la conversión (Polling)

```C
if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
{
  ...
}
```
- **Propósito**: Espera activamente a que la conversión ADC finalice.
- **Parámetros**:
  - `&hadc1`: Referencia al manejador del ADC.
  - `1000`: Tiempo máximo de espera en milisegundos antes de timeout.

- **Funcionamiento**:
  - Esta función es **bloqueante**, lo que significa que el microcontrolador permanece en este punto hasta que la conversión termine o se alcance el tiempo máximo de espera.
  - Si la conversión se completa dentro del tiempo especificado, la función retorna `HAL_OK` y el programa continúa.

##### 3. Lectura del Valor Digital Convertido
```C
ADC_VAL = HAL_ADC_GetValue(&hadc1);
```
- **Propósito**: Lee el resultado de la conversión ADC.
- **Funcionamiento**:
  - Una vez finalizada la conversión, el valor digital resultante (de 12 bits en el STM32F103) se almacena en el registro de datos del ADC.
  - `HAL_ADC_GetValue()` recupera este valor y lo guarda en la variable ADC_VAL.

##### 4. Detención del ADC
```C
HAL_ADC_Stop(&hadc1);
```
- **Propósito**: Detiene el periférico ADC.
- **Funcionamiento**:
  - Aunque en este ejemplo el ADC se reinicia en cada iteración, detenerlo después de cada conversión puede ahorrar energía en aplicaciones donde no se requieren conversiones continuas.

## Interrupción

En el modo **Interrupción**, el ADC notifica al CPU únicamente cuando se completa una conversión.  
Esto elimina la necesidad de realizar sondeos constantes (*polling*) y permite que el CPU se concentre en otras tareas hasta que se active la interrupción.  
Es una opción adecuada cuando el muestreo es ocasional o cuando se busca optimizar el tiempo de procesamiento en comparación con el método bloqueante.

### Configuración del Proyecto en STM32CubeIDE

#### 1. Crear un Nuevo Proyecto

Siga los pasos del tutorial [Blink](/PerifericosBasicos/HelloWorld/README.md) para crear un nuevo proyecto para el microcontrolador `STM32F411CEU6`, incluyendo la configuración del reloj.

#### 2. Configuración de Pines

Para la configuración del pin, acceda a:  
`Pinout & Configuration` → `Analog` → `ADC1` → **Habilitar canal 0** (`IN0`).

**ADCs Common Settings**
- `Mode`: Independent Mode  

**ADC Settings**
- `Data Alignment`: Right alignment  
- `Scan Conversion Mode`: Disable  
- `Continuous Conversion Mode`: Enabled  
- `Discontinuous Conversion Mode`: Disable  

A continuación, diríjase al menú superior y configure:

**NVIC Settings**
- `ADC1 and ADC2 Global Interrupt`: Enabled  

![Interrupt](/PerifericosBasicos/ADC/Images/ConfInterrupt.png)

![NVIC](/PerifericosBasicos/ADC/Images/NVICInterrupt.png)

La visualización de los datos se realizará mediante **UART**, por lo que la configuración deberá ser la misma que en el modo *Polling*.

#### 3. Generar el Código

Haga clic en el icono de la **llave amarilla** para generar el código base.  
![Generar_Codigo](/PerifericosBasicos/HelloWorld/images/CodeGeneration.png)

### Implementación de Recepción de Datos

```C
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint32_t ADC_VAL;
char buffer[15];
/* USER CODE END PV */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 2 */
    HAL_ADC_Start_IT(&hadc1);
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while(1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief  Callback de finalización de conversión del ADC.
  * @param  hadc: puntero al manejador del ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  ADC_VAL = HAL_ADC_GetValue(&hadc1);

  sprintf(buffer, "ADC: %lu\r\n", ADC_VAL);

  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
}
```

Dentro del **callback**, se lee el valor convertido utilizando la función `HAL_ADC_GetValue()`, la cual devuelve el valor obtenido por el ADC según la resolución configurada previamente.

Una vez completada la conversión, los datos quedan almacenados en la variable `ADC_VAL`, y posteriormente son enviados mediante la interfaz UART para su visualización.

Dado que el ADC se encuentra configurado con una resolución de 12 bits, el valor máximo posible será:

$2^{12} = 4095$

En el caso de un ADC con una resolución mayor, por ejemplo de 16 bits, el valor máximo alcanzaría:

$2^{16} = 65,536$

## DMA

En el modo **DMA (Direct Memory Access)**, el ADC envía los datos convertidos directamente a la memoria sin intervención del CPU.  
Esto permite realizar transferencias de datos continuas con una sobrecarga mínima, ya que el CPU puede encargarse de otras tareas mientras el DMA gestiona el movimiento de datos en segundo plano.

### Configuración del Proyecto en STM32CubeIDE

#### 1. Crear un Nuevo Proyecto
Siga los pasos del tutorial [Blink](/PerifericosBasicos/HelloWorld/README.md) para crear un nuevo proyecto para el `STM32F411CEU6`, incluyendo la configuración de reloj.

#### 2. Configuración de Pines

Para la configuración del pin, acceda a:  
`Pinout & Configuration` → `Analog` → `ADC1` → **Habilitar canal 0** (`IN0`).

**ADCs Common Settings**
- `Mode`: Independent Mode  

**ADC Settings**
- `Data Alignment`: Right alignment  
- `Scan Conversion Mode`: Disable  
- `Continuous Conversion Mode`: Enabled  
- `Discontinuous Conversion Mode`: Disable  

A continuación, diríjase al menú superior:

![DMA](/PerifericosBasicos/ADC/Images/ConfDMA.png)

Como se observa en la imagen anterior, es necesario habilitar el `Continous Conversion Mode`.  
Esto garantiza que cada conversión inicie inmediatamente después de finalizar la anterior, permitiendo que el proceso se ejecute de forma ininterrumpida y la interrupción se active con una frecuencia constante.  
Para un solo canal, este método asegura que el mismo canal se convierta de manera continua.

![DMA_Settings](/PerifericosBasicos/ADC/Images/DMASettigns.png)

Luego, acceda a la sección **DMA Settings** y agregue una solicitud para el ADC.  
Configure el modo en **Circular** para que el DMA se reactive automáticamente después de cada transferencia.  
Si se selecciona el modo **Normal**, el DMA se detendrá tras una sola transferencia y será necesario reiniciarlo manualmente.

A diferencia del microcontrolador `STM32F103`, el `STM32F446` incluye la opción **DMA Continuous Request**, que permite mantener la solicitud de transferencia activa entre conversiones.

La diferencia entre ambos microcontroladores se resume en la siguiente tabla:

| Parámetro | **STM32F103** | **STM32F446** |
|------------|---------------|---------------|
| `Mode` | Independent Mode | Independent Mode |
| `Clock Prescaler` | X | PCLK2 divided by 6 |
| `Resolution` | X | 12 Bits |
| `Data Alignment` | Right Alignment | Right Alignment |
| `Scan Conversion Mode` | Disable | Disable |
| `Continuous Conversion Mode` | Enabled | Enabled |
| `Discontinuous Conversion Mode` | Disable | Disable |
| `DMA Continuous Request` | X | Enabled |
| `End Of Conversion Selection` | X | EOC flag at the end of single channel conversion |

La visualización de los datos se realizará nuevamente mediante **UART**, por lo que la configuración será idéntica a la utilizada en el modo *Polling*.

#### 3. Generar el Código

Haga clic en el icono de la **llave amarilla** para generar el código base.  
![Generar_Codigo](/PerifericosBasicos/HelloWorld/images/CodeGeneration.png)

### Implementación de Recepción de Datos

```C
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint16_t ADC_VAL;

char buffer[15];
/* USER CODE END PV */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 2 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_VAL, 1);
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while(1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Callback que se ejecuta cuando se completa una conversión del ADC mediante DMA.
  * @param  hadc: puntero al manejador del ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    sprintf(buffer, "ADC: %u\r\n", ADC_VAL);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
}
/* USER CODE END 4 */

```
#### Explicación detallada

1. Inicialización del **DMA** con el **ADC**
   
    La función:
    ```C
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_VAL, 1);
    ```
    inicia el ADC en modo DMA, indicando que los datos convertidos se almacenarán en la dirección de memoria correspondiente a ADC_VAL.

    El tercer parámetro (1) especifica la cantidad de muestras que se transferirán en cada ciclo.

2. Tipo de dato y conversión de puntero.
   
    Se define `ADC_VAL` como una variable de 16 bits, ya que la resolución configurada del ADC es de 12 bits (`ADC_RESOLUTION_12B`).

    Sin embargo, la función `HAL_ADC_Start_DMA()` requiere un puntero a una variable de 32 bits (uint32_t*), por lo que se realiza un cast de tipo:
    ```C
    (uint32_t*)ADC_VAL
    ```
    Esto garantiza compatibilidad con la firma de la función sin alterar el valor real del dato.

3. Callback de conversión completa

    Cuando el DMA transfiere exitosamente un valor desde el ADC, se ejecuta automáticamente la función de callback:

    ```C
    void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
    ```

    Dentro de esta función, el valor almacenado en `ADC_VAL` se formatea en texto y se transmite por UART para su visualización.

    Es importante destacar que, dado que la transferencia es gestionada por hardware, esta función no bloquea la ejecución del programa.

Definimos `ADC_VAL` cómo una variable de 16-bits y lo casteamos cómo una variable de 32 bits, ya que la función` HAL_ADC_Start_DMA` espera un puntero a uint32_t* en su firma, pero cuando el ADC está configurado con resolución de 12 bits (`ADC_RESOLUTION_12B`), los valores reales que genera son de 16 bits.

## Interrupción vs DMA

La elección entre los modos **Interrupción** y **DMA (Direct Memory Access)** depende principalmente de la frecuencia de muestreo requerida y del nivel de carga permitido sobre el CPU.

- Utilice el **modo de interrupción** cuando las conversiones sean esporádicas, el número de canales sea reducido y la carga del procesador no represente un problema.  
- Utilice el **modo DMA** cuando se requiera capturar datos de manera continua o procesar grandes volúmenes de información con alta eficiencia, reduciendo al mínimo la intervención del CPU.

| Característica | **Interrupt Mode** | **DMA Mode** |
|----------------|--------------------|---------------|
| **Participación del CPU** | El CPU maneja cada conversión mediante la rutina de servicio de interrupción (*ISR*) | El CPU permanece libre; el DMA transfiere los datos directamente a memoria |
| **Eficiencia** | Moderada. Adecuada para pocas conversiones | Alta. Ideal para conversiones continuas o de gran volumen |
| **Complejidad de implementación** | Sencillo de configurar | Más complejo, requiere definir canales y buffers de DMA |
| **Latencia** | Ligero retraso debido al manejo de la interrupción | Muy baja, ya que las transferencias se realizan por hardware |
| **Mejor caso de uso** | Muestreos ocasionales o de baja frecuencia | Muestreos rápidos o procesamiento intensivo de datos |

En resumen, el **modo de interrupción** es apropiado para tareas simples o de baja frecuencia, mientras que el **modo DMA** es la opción recomendada para sistemas embebidos que requieren un flujo constante de datos analógicos con mínima carga sobre el procesador.


## ADC Multicanal DMA Circular

En el modo **ADC multicanal**, se recomienda el uso del **DMA en modo circular**, ya que este permite realizar múltiples conversiones consecutivas sin intervención del CPU.  
El DMA reinicia automáticamente el proceso una vez completada la transferencia de todos los canales configurados, reduciendo significativamente la carga de procesamiento y permitiendo una adquisición continua y eficiente de datos.

---

### Configuración del Proyecto en STM32CubeIDE

#### 1. Crear un Nuevo Proyecto

Siga los pasos descritos en el tutorial [Blink](/PerifericosBasicos/HelloWorld/README.md) para crear un nuevo proyecto para el microcontrolador `STM32F411CEU6`, incluyendo la configuración del reloj.

#### 2. Configuración de Pines
Para configurar los pines correspondientes a los canales del ADC, acceda a:  
`Pinout & Configuration` → `Analog` → `ADC1` → **Habilitar canal 0** (`IN0`), **Habilitar canal 1** (`IN1`) y **Habilitar canal 2** (`IN2`).
 
**ADCs Common Settings**
- `Mode`: Independent Mode

**ADC Settings**
- `Data Alignment`: Right alignment
- `Scan Conversion Mode`: Enabled 
- `Continous Conversion Mode`: Enabled
- `Discontinous Conversion Mode`: Disable

**ADC Regular ConversionMode**
- `Enable Regular Conversions`: Enabled
- `Number of Conversion`: 3
- `External Trigger Conversion Source`: Regular Conversion launched by Software
- `Rank 1`: 1
  - `Channel`: Channel 0
  - `Sampling Time`: 55.5 Cycles
- `Rank 2`: 1
  - `Channel`: Channel 0
  - `Sampling Time`: 55.5 Cycles
- `Rank 3`: 1
  - `Channel`: Channel 0
  - `Sampling Time`: 55.5 Cycles

---

Al habilitar el **Scan Conversion Mode**, el ADC recorrerá automáticamente los canales en el orden definido (Rank 1 → Rank 2 → Rank 3).  
El **Continuous Conversion Mode** garantiza que, al finalizar la última conversión, el proceso se reinicie de forma automática, permitiendo una lectura cíclica sin intervención manual.

![DMA](/PerifericosBasicos/ADC/Images/ConfDMAMultichannel.png)

---

### Configuración del DMA

A continuación, diríjase a la sección **DMA Settings** y agregue una solicitud para el ADC.  
Configure el modo en **Circular** para que el DMA se reactive automáticamente después de cada secuencia de conversiones.  
Si se selecciona el modo **Normal**, el DMA se detendrá al finalizar la transferencia y deberá reiniciarse manualmente.

![DMA_Settings](/PerifericosBasicos/ADC/Images/DMASettigns.png)

A diferencia del microcontrolador `STM32F103`, el `STM32F446` permite habilitar la opción **DMA Continuous Request**, la cual mantiene activa la comunicación entre el ADC y el DMA, asegurando un flujo continuo de datos.

La diferencia entre ambos microcontroladores se muestra a continuación:

| Parametro | **STM32F103** | **STM32F446** |
|-----------|-------------|-------------|
| `Mode` | Independent Mode | Independent Mode |
| `Clock Prescaler` | X | PCLK2 divided by 6 |
| `Resolution` | X | 12 Bits |
| `Data Aligmen`t | Right Aligment | Right Aligment |
| `Scan Conversion Mode` | Enabled | Enabled |
| `Continous Conversion Mode` | Enabled | Enabled |
| `Discontinous Conversion Mode` | Disable | Disable |
| `DMA Continous Request` | X | Enabled |
| `End Of Conversion Selection` | X | EOC flag the end of all conversion |

Al igual que en los modos anteriores, la visualización de los resultados se realiza mediante **UART**, por lo que debe configurarse de la misma forma que en el modo *Polling*.

#### 3. Generar el Código

Haga clic en el icono de la **llave amarilla** para generar el código base.  
![Generar_Codigo](/PerifericosBasicos/HelloWorld/images/CodeGeneration.png)

---

### Implementación de Recepción de Datos
```C
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint16_t ADC_VAL[3];            // Arreglo para almacenar los valores de los tres canales
char buffer[55];                // Buffer de texto para transmisión por UART
volatile uint8_t adc_ready = 0; // Bandera para indicar que los datos están listos
/* USER CODE END PV */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 2 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_VAL, 3); // Inicio del ADC en modo DMA
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while(1)
    {
        if(adc_ready == 1)
        {
            adc_ready = 0;

            sprintf(buffer, "ADC1: %u\tADC2: %u\tADC3: %u\n\n\r", ADC_VAL[0], ADC_VAL[1], ADC_VAL[2]);

            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

            HAL_Delay(500);
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Callback ejecutado cuando el DMA completa la conversión de todos los canales.
  * @param  hadc: puntero al manejador del ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    adc_ready = 1;
}

/* USER CODE END 4 */
```

#### Explicación del Funcionamiento

1. Secuencia de conversión
   
   El ADC recorre automáticamente los tres canales configurados (`IN0`, `IN1`, `IN2`) en el orden definido por sus ranks.

2. Transferencia de datos con DMA

    Cada valor digitalizado se almacena en el arreglo ADC_VAL[3] sin intervención del CPU.
    Gracias al modo Circular, el proceso se repite indefinidamente.

3. Callback de conversión completa:

    Una vez que el DMA transfiere los datos, se activa el callback `HAL_ADC_ConvCpltCallback()`, el cual establece la bandera `adc_ready` para indicar que los datos están listos para su uso o transmisión.

Esta configuración multicanal con DMA en modo circular es ideal para aplicaciones que requieren adquisición simultánea de múltiples señales analógicas, como sensores ambientales, controladores de potencia o sistemas de adquisición de datos embebidos.

#### Nota personal
Esta configuración es más eficaz para un microcontrolador `STM32F446` que para un `STM32F103`. Revisar proyecto STM32 para más detalles.

## Distribución
La estructura del repositorio es:

```text
ADC/
├── images
├── README.md
├── ADC_Polling/
├── ADC_DMA/
├── ADC_Interrupt/
└── ADC_Multicanal/
```

Cada carpeta incluye un proyecto ejemplo creado con **STM32CubeIDE**, listo para su importación.

---

## Licencia

Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.

