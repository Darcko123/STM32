#  UART (Universal Asynchronous Receiver-Transmitter)

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![UART](https://img.shields.io/badge/Protocol-UART-yellow)](https://es.wikipedia.org/wiki/Universal_Asynchronous_Receiver-Transmitter)

## Descripción
Este proyecto implementa una serie de programas explicando el funcionamiento de uno de los protocolos de comunicación más usados; el UART.

El objetivo de este tutorial es guiar paso a paso en la configuración del entorno de desarrollo, la generación del código base con STM32CubeIDE y la implementación de formas de comunicación utilizando diferentes funciones de la capa de abstracción de hardware (HAL).

## Características Principales
- ✅ Configuración del protocolo
- ✅ Opcione visuales para el uso en terminal

## Componentes Requeridos
- STM32CubeIDE (descargable desde [STMicroelectronics](https://www.st.com/en/development-tools/stm32cubeide.html))
- Placa STM32F411CEU6 (Black Pill) o compatible.
- ST-Link V2 (programador/depurador).
- Fuente de alimentación de 5V.
- TTL a USB
  
Se sugiere el uso de un TTL a USB cómo el siguiente modelo, sin embargo, cualquier otro cumple con la misma función:

![TTL_A_USB](/PerifericosBasicos/UART/images/TTL_USB.png)

# Configuración del Proyecto en STM32CubeIDE

## 1. Crear un Nuevo Proyecto
Sigue los mismos pasos del tutorial [Blink](/PerifericosBasicos/HelloWorld/README.md) para crear un nuevo proyecto para el `STM32F411CEU6` así como la configuración de reloj.

## 2.Configuración de pines
- Para la configuración del pin acceder a  `Pinout & Configuration` > `Connectivity` > `USART1`.
- En `Mode`, cambiar de `Disable` a `Asynchronus`.

>[!Note]
> Si se está utilizando el microcontrolador `STM32F411CEU6`, este no cuenta con UART, Únicamente cuenta con USART, la principal diferencia es que este protocolo admite comunicación síncroma, pero no es el objeto del tutorial, de manera que aunque admite esta configuración, solo se verá el UART.

Por predeteminado se configuran los parametros cómo:

__Basic Parameters__
- `Baud Rate`: 115200
- `Word Lenght`: 8 Bits(including Parity)
- `Parity`: None
- `Stop`: 1

__Advanced Parameters__
- `Data Direction`: Receive and Transmit
- `Over Sampling`: 16 Samples

![Parameter Settings](/PerifericosBasicos/UART/images/ParameterSettings.png)

Adicionalmente se configuraran automaticamente los pines `PA9` y `PA10` cómo:
- `PA9`: USART1_TX
- `PA10`: USART1_RX

De manera que el pinout quedaría de la siguiente manera:

![Pinout](/PerifericosBasicos/UART/images/Pinout.png)

## 3. Generar el Código
Haz clic en el icono de la llave de la tuerca amarilla para generar el código base.

![Generar_Codigo](/PerifericosBasicos/HelloWorld/images/CodeGeneration.png)

## 6. Conexiones Físicas

| `STM32F411` | `TTL` |
| --- | --- |
| `PA9`: TX | RX |
| `PA10`: RX | TX |
| GND | GND |

# Implementación de Transmisión de datos

## Transmisión básica

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

Aquí definimos un vector (data) que contiene la cadena que se enviará al UART. Luego, llamamos a la función HAL_UART_Transmit en el bucle while cada 1 segundo.

La función recibe los parámetros:

```C
HAL_UART_Transmit(
    &huart,     //Handler del UART
    pData,      //Puntero a los datos a enviar
    Size,       //Tamaño de los datos a enviar
    Timeout     //Tiempo de espera
    );
```

## Transmisión de números

Esta función sólo transmite los datos en formato ascii. Para enviar el número, primero debemos convertir cada dígito del número al formato de caracteres y luego enviar los datos al UART.

Para esto, ocuparemos la función `sprintf()` de la librería básica de c `stdio.h`.

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

Para veríficar los cambios, se debe configurar una ventana de terminal serial con las mismas características que se configuraron en STM32MX. Se sugiere el uso de Putty para su visualización, sin embargo, puede ser cualquier terminal de su preferencia.

De esta manera, la configuración de la terminal quedaría de la siguiente manera:

![SerialTerminal](/PerifericosBasicos/UART/images/SerialTerminal.png)

>[!Note]
>El puerto cambiará dependiendo de cada dispositivo. Ingresar a `Administrador de dispositivos` > `Puertos (COM y LPT)` y veríficar el puerto en el que aparece `USB Serial Port`

### Resultado
![Result](/PerifericosBasicos/UART/images/HelloWorld.png)

Recordar además que cómo en C, se pueden agregar tabulaciones, saltos de linea y regreso en la misma línea, todo esto con los comandos `/n`, `/t`, `/r`.

De manera que modificando un poco el código, el array de datos podría quedar:

```C
uint8_t data[] = "Hello\tworld\n\r";
```

Y en terminal se podría ver:

![HelloWordTab](/PerifericosBasicos/UART/images/HelloWorldTab.png)

Adicionalmente, y cómo función totalmente extra, se pueden agregar colores a termianl, incorporando los siguientes extractos de código:

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
