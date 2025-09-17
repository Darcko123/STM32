# Comunicación UART en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![UART](https://img.shields.io/badge/Protocol-UART-yellow)](https://es.wikipedia.org/wiki/Universal_Asynchronous_Receiver-Transmitter)

Este documento presenta una guía completa para la implementación de la comunicación **UART (Universal Asynchronous Receiver/Transmitter)** en microcontroladores **STM32**, utilizando **STM32CubeIDE**. Se proporciona una explicación detallada desde la configuración inicial hasta ejemplos prácticos de transmisión de datos.

---

## Índice

- [Comunicación UART en STM32](#comunicación-uart-en-stm32)
  - [Índice](#índice)
  - [Descripción General](#descripción-general)
  - [Características Principales](#características-principales)
  - [Componentes Requeridos](#componentes-requeridos)
  - [Configuración del Proyecto en STM32CubeIDE](#configuración-del-proyecto-en-stm32cubeide)
    - [1. Crear un Nuevo Proyecto](#1-crear-un-nuevo-proyecto)
    - [2.Configuración de pines](#2configuración-de-pines)
    - [3. Generar el Código](#3-generar-el-código)
    - [4. Conexiones Físicas](#4-conexiones-físicas)
  - [Implementación de Transmisión de Datos](#implementación-de-transmisión-de-datos)
    - [Transmisión Básica](#transmisión-básica)
    - [Transmisión de Números](#transmisión-de-números)
    - [Transmisión con Colores en Terminal](#transmisión-con-colores-en-terminal)
  - [Trasnsmisión de datos con casteo](#trasnsmisión-de-datos-con-casteo)
  - [Próximas Implementaciones](#próximas-implementaciones)
  - [Licencia](#licencia)

---

## Descripción General

El objetivo de este tutorial es guiar paso a paso en la configuración del entorno de desarrollo, la generación del código base con **STM32CubeIDE** y la implementación de diversas formas de comunicación utilizando las funciones de la capa de abstracción de hardware (HAL).  
Actualmente se cubre la **transmisión de datos**, y se prevé incluir próximamente la **recepción de datos**, incorporando el uso de **interrupciones** y **DMA**.

---

## Características Principales

- ✅ Configuración completa del protocolo UART.  
- ✅ Ejemplos visuales para su uso en una terminal serial.

---

## Componentes Requeridos

- **STM32CubeIDE**: [Descargar desde STMicroelectronics](https://www.st.com/en/development-tools/stm32cubeide.html)  
- **Placa de desarrollo**: STM32F411CEU6 (Black Pill) o compatible.  
- **Programador/depurador**: ST-Link V2.  
- **Fuente de alimentación**: 5 V.  
- **Convertidor TTL a USB** (cualquier modelo es válido).  

Ejemplo de convertidor sugerido:
![TTL_A_USB](/PerifericosBasicos/UART/images/TTL_USB.png)

## Configuración del Proyecto en STM32CubeIDE

### 1. Crear un Nuevo Proyecto
Siga los pasos del tutorial [Blink](/PerifericosBasicos/HelloWorld/README.md) para crear un nuevo proyecto para el `STM32F411CEU6`, incluyendo la configuración de reloj.

### 2.Configuración de pines
- Para la configuración del pin acceder a  `Pinout & Configuration` > `Connectivity` > `USART1`.
- En `Mode`, cambiar de `Disable` a `Asynchronus`.

>[!Note]
> El microcontrolador `STM32F411CEU6` no dispone de UART nativo, únicamente **USART**. Este protocolo admite comunicación síncrona, pero en este tutorial se emplea en modo asíncrono, funcionando como UART.

Parámetros predeterminados:

**Basic Parameters**
- `Baud Rate`: 115200
- `Word Lenght`: 8 Bits(including Parity)
- `Parity`: None
- `Stop`: 1

**Advanced Parameters**
- `Data Direction`: Receive and Transmit
- `Over Sampling`: 16 Samples

![Parameter Settings](/PerifericosBasicos/UART/images/ParameterSettings.png)

Adicionalmente se configuraran automaticamente los pines `PA9` y `PA10` cómo:
- `PA9`: USART1_TX
- `PA10`: USART1_RX

De manera que el pinout quedaría de la siguiente manera:

![Pinout](/PerifericosBasicos/UART/images/Pinout.png)

### 3. Generar el Código
Haga clic en el icono de la llave amarilla para generar el código base.  
![Generar_Codigo](/PerifericosBasicos/HelloWorld/images/CodeGeneration.png)

### 4. Conexiones Físicas

| STM32F411 | TTL |
| --------- | --- |
| `PA9` TX  | RX  |
| `PA10` RX | TX  |
| `GND`     | GND |

---

## Implementación de Transmisión de Datos

### Transmisión Básica
Ejemplo de envío de una cadena cada segundo:

```C
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint8_t data[] = "Hello world\n";
/* USER CODE END PV */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Transmit(&huart1, data, strlen(data), 1000);
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```

Parámetros de `HAL_UART_Transmit`:

```C
HAL_UART_Transmit(
    &huart,     //Handler del UART
    pData,      //Puntero a los datos a enviar
    Size,       //Tamaño de los datos a enviar
    Timeout     //Tiempo de espera
    );
```

### Transmisión de Números

Para enviar números se usa `sprintf()` para convertirlos a cadena:

```C
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint8_t number = 123;
uint8_t numarray[4];
/* USER CODE END PV */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    sprintf(numarray, "%d\n", number);
    HAL_UART_Transmit(&huart1, numarray, strlen(numarray), 1000);
    HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```

Configure una terminal serial con los mismos parámetros de STM32MX (por ejemplo, PuTTY).  

![SerialTerminal](/PerifericosBasicos/UART/images/SerialTerminal.png)

>[!Note]
>El puerto cambiará dependiendo de cada dispositivo. Ingresar a `Administrador de dispositivos` > `Puertos (COM y LPT)` y veríficar el puerto en el que aparece `USB Serial Port`

Resultado:  
![Result](/PerifericosBasicos/UART/images/HelloWorld.png)

Recordar además que cómo en C, se pueden agregar tabulaciones, saltos de linea y regreso en la misma línea, todo esto con los comandos `/n`, `/t`, `/r`.

De manera que modificando un poco el código, el array de datos podría quedar:

```C
uint8_t data[] = "Hello\tworld\n\r";
```

Y en terminal se podría ver:

![HelloWordTab](/PerifericosBasicos/UART/images/HelloWorldTab.png)

### Transmisión con Colores en Terminal
Se pueden agregar colores utilizando secuencias ANSI:

```C
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_RESET   "\x1b[0m"
/* USER CODE END PD */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
int main()
{
  while (1)
  {
	  //Rojo
	  sprintf(numarray, ANSI_COLOR_RED"%d\n\r", number);
	  HAL_UART_Transmit(&huart1, numarray, strlen(numarray), 1000);

	  //Verde
	  sprintf(numarray, ANSI_COLOR_GREEN"%d\n\r", number);
	  HAL_UART_Transmit(&huart1, numarray, strlen(numarray), 1000);

	  //Amarillo
	  sprintf(numarray, ANSI_COLOR_YELLOW"%d\n\r", number);
	  HAL_UART_Transmit(&huart1, numarray, strlen(numarray), 1000);

	  //Azúl
	  sprintf(numarray, ANSI_COLOR_BLUE"%d\n\r", number);
	  HAL_UART_Transmit(&huart1, numarray, strlen(numarray), 1000);

	  //Reset
	  sprintf(numarray, ANSI_COLOR_RESET"%d\n\r", number);
	  HAL_UART_Transmit(&huart1, numarray, strlen(numarray), 1000);

	  HAL_Delay(5000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```

![TerminalColor](/PerifericosBasicos/UART/images/TerminalCOlor.png)

## Trasnsmisión de datos con casteo

Para enviar variables que no sean `uint8_t`, realice un casteo para evitar advertencias:

```C
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
int number = 65535;
char numberBuffer[7];
/* USER CODE END PV */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    sprintf(numberBuffer, "%d\n\r", number);
    HAL_UART_Transmit(&huart1, (uint8_t*)numberBuffer, strlen(numberBuffer), 1000);
    HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```

## Próximas Implementaciones

Actualmente se cubre únicamente la **transmisión de datos**. Próximamente se añadirán:

- Recepción de datos básica.
- Recepción con DMA.
- Transmisión y recepción mediante interrupciones.

La estructura de carpetas prevista será:

```
UART/
├── images
├── README.md
├── UART_Transmit/
├── UART_Transmit_Interrupt/
├── UART_Receive/
├── UART_Receive_DMA/
└── UART_Receive_Interrupt/
```

Cada carpeta incluirá un proyecto ejemplo creado con **STM32CubeIDE**, listo para su importación.

---

## Licencia

Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.