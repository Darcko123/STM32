# Control de Matriz de LEDs con MAX7219 en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)
[![Version](https://img.shields.io/badge/Version-2.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/PWM_Module)

---

## Tabla de Contenidos
- [Control de Matriz de LEDs con MAX7219 en STM32](#control-de-matriz-de-leds-con-max7219-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Requisitos](#requisitos)
  - [Configuración de Periférico](#configuración-de-periférico)
    - [SPI](#spi)
    - [GPIO](#gpio)
  - [Conexión de Hardware](#conexión-de-hardware)
  - [Ejemplo de uso](#ejemplo-de-uso)
    - [**Ejemplo básico**](#ejemplo-básico)
  - [API Reference](#api-reference)
    - [Tipo de dato para gestión de errores](#tipo-de-dato-para-gestión-de-errores)
    - [Funciones Básicas](#funciones-básicas)
      - [`MAX7219_Init()`](#max7219_init)
      - [`MAX7219_ClearDisplay()`](#max7219_cleardisplay)
      - [`MAX7219_ScrollString()`](#max7219_scrollstring)
      - [`MAX7219_PrintString()`](#max7219_printstring)
  - [Notas](#notas)
  - [📄 Licencia](#-licencia)
  - [Changelog](#changelog)
    - [Versión 2.0.0](#versión-200)
    - [Versión 1.1.0](#versión-110)
    - [Versión 1.0.0](#versión-100)

## Descripción
Esta librería permite controlar una matriz de LEDs mediante el controlador **MAX7219** utilizando comunicación **SPI** en un microcontrolador STM32. La implementación permite visualizar caracteres, desplazarlos y manipular la matriz de LEDs de manera eficiente.

Esta librería está basada en el código de **Controllerstech**:
[Cómo desplazar cadenas en una pantalla de matriz de puntos](https://controllerstech.com/how-to-scroll-string-on-dot-matrix-display/#info_box)

---

## Características
- Comunicación mediante SPI con STM32 HAL.
- Control de múltiples dispositivos MAX7219 en cascada.
- Funciones para desplazar cadenas de texto (scroll) y mostrar texto estático.
- Conjunto de fuentes de caracteres 8x8.
- Gestión de buffer de columnas para actualizaciones eficientes.
- Configuración sencilla del periférico SPI y GPIO para CS.
- Sistema de gestión de errores con tipo `MAX7219_Status_t`.
- Documentación completa con comentarios Doxygen.

---

## Requisitos

- STM32CubeIDE o STM32CubeMX.
- Biblioteca HAL correspondiente a tu microcontrolador STM32.
- Módulo MAX7219 conectado mediante SPI.

## Configuración de Periférico

### SPI

| Parametro | Valor |
|-----------|-------|
| Mode      | Half-Duplex Master |
| Frame Format | Motorola |
| Data Size | 16 Bits |
| First Bit | MSB First |
| Prescale | 32 | 
| Clock Polarity | High |
| Clock Phase | 1 Edge |

### GPIO

| Parametro | Valor |
|-----------|-------|
| GPIO Output Level | HIGH |
| GPIO Mode | Output Pushh Pull |
| Maximim Output Speed | Low |

## Conexión de Hardware

| MAX7219 | STM32   |
|---------|---------|
| VCC     | 5V/3.3V |
| GND     | GND     |
| DIN     | SPI_MOSI |
| CS      | GPIO_CS |
| CLK     | SPI_SCK |

## Ejemplo de uso

### **Ejemplo básico**

```c
#include "MAX7219.h"

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 2 */

  if(MAX7219_Init(&hspi1, GPIOA, GPIO_PIN_6) != MAX7219_OK)
  {
	  sprintf(buffer, "%sError inicializando MAX7219%s\r\n", COLOR_RED, COLOR_RESET);
	  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
	  while(1);
  }

  if(MAX7219_ClearDisplay() != MAX7219_OK)
  {
	  sprintf(buffer, "%sError Limpiando MAX7219%s\r\n", COLOR_RED, COLOR_RESET);
	  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
  }

  if(MAX7219_PrintString("HOLA") != MAX7219_OK)
  {
	  sprintf(buffer, "%sError mandando cadena%s\r\n", COLOR_RED, COLOR_RESET);
	  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(MAX7219_ScrollString("HOLA MUNDO", 100) != MAX7219_OK)
	  {
		  sprintf(buffer, "%sError mandando cadena%s\r\n", COLOR_RED, COLOR_RESET);
		  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
	  }
	  HAL_Delay(200);

	  MAX7219_ClearDisplay();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```

## API Reference

### Tipo de dato para gestión de errores

```c
/**
 * @brief Enumeración para estados de retorno del MAX7219.
 */
typedef enum {
	MAX7219_OK = 0,				/** Operación exitosa */
	MAX7219_ERROR = 1,			/** Error en la operación */
	MAX7219_TIMEOUT = 2,		/** Timeout en la operación */
	MAX7219_NOT_INITIALIZED = 3	/** Sensor no inicializado */
}MAX7219_Status_t;
```

### Funciones Básicas

---

#### `MAX7219_Init()`
Inicializa el módulo MAX7219 con los parámetros de SPI y GPIO para CS.

```c
MAX7219_Status_t MAX7219_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN);
```

**Parámetros:**
- `hspi`: Puntero a la estructura de manejo del periférico SPI.
- `GPIOx`: Puerto GPIO donde está conectado el pin CS.
- `GPIO_PIN`: Pin GPIO donde está conectado el pin CS.

**Retorna:**
- `MAX7219_Status_t` indicando el estado de la operación.

---

#### `MAX7219_ClearDisplay()`
Limpia la pantalla de la matriz de LEDs.

```c
MAX7219_Status_t MAX7219_ClearDisplay(void);
```

**Retorna:**
- `MAX7219_Status_t` indicando el estado de la operación.

---

#### `MAX7219_ScrollString()`
Desplaza una cadena de texto en la matriz de LEDs.

```c
MAX7219_Status_t MAX7219_ScrollString(char* string, uint32_t delay_ms);
```

**Parámetros:**
- `string`: Cadena de texto a desplazar.
- `delay_ms`: Retraso entre cada paso de desplazamiento en milisegundos.

**Retorna:**
- `MAX7219_Status_t` indicando el estado de la operación.

---

#### `MAX7219_PrintString()`
Muestra una cadena de texto estática en la matriz de LEDs.

```c
MAX7219_Status_t MAX7219_PrintString(char* string);
```

**Parámetros:**
- `string`: Cadena de texto a mostrar.

**Retorna:**
- `MAX7219_Status_t` indicando el estado de la operación.

---

## Notas
- El número de dispositivos MAX7219 en cascada se define con `NUM_DEV` en **MAX7219.h**.

---

## 📄 Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog

### Versión 2.0.0
- Implementación completa de sistema de gestión de errores con tipo MAX7219_Status_t
- Validación exhaustiva de parámetros en todas las funciones públicas
- Conversión de funciones críticas a privadas (`max7219_cmd`, `max7219_write`, `flushBuffer`, `ShiftLeft`, `ShiftRight`, `shiftchar`)
- Diferenciación entre errores generales, timeouts SPI y módulo no inicializado
- Mejora en la función de inicialización para recibir parámetros de GPIO del pin CS
- Implementación de bandera de verificación de inicialización del módulo
- Control de punteros NULL en todas las funciones públicas
- Cambio de nombres de funciones para seguir convención de nomenclatura consistente
- Documentación mejorada con comentarios Doxygen completos

### Versión 1.1.0
- Parametrización del puerto SPI en la función de inicialización

### Versión 1.0.0
- Versión inicial con control básico de matriz de LEDs usando MAX7219
- Soporte para múltiples dispositivos en cascada
- Funciones de desplazamiento de texto (scroll) y visualización estática
- Conjunto completo de fuentes de caracteres 8x8
- Gestión de buffer de columnas para actualizaciones eficientes
- Configuración básica de SPI para comunicación con MAX7219