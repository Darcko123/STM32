# MPU6050 STM32 Acelerómetro y Giroscopio I2C Librería

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)
[![Version](https://img.shields.io/badge/Version-2.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/MPU6050)

---

## Tabla de Contenidos
- [MPU6050 STM32 Acelerómetro y Giroscopio I2C Librería](#mpu6050-stm32-acelerómetro-y-giroscopio-i2c-librería)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [**Descripción General**](#descripción-general)
  - [**Características**](#características)
  - [**Requisitos**](#requisitos)
  - [**Ejemplo de Uso**](#ejemplo-de-uso)
  - [API Reference](#api-reference)
    - [Tipo de dato para gestión de errores](#tipo-de-dato-para-gestión-de-errores)
    - [**Funciones Principales**](#funciones-principales)
      - [`MPU6050_Init()`](#mpu6050_init)
      - [`MPU6050_Read_Accel()`](#mpu6050_read_accel)
      - [`MPU6050_Read_Gyro()`](#mpu6050_read_gyro)
  - [Conexión de hardware](#conexión-de-hardware)
  - [📄 Licencia](#-licencia)
  - [Changelog](#changelog)
    - [Version 2.0.1](#version-201)
    - [Version 2.0.0](#version-200)
    - [Versión 1.2.0](#versión-120)
    - [Versión 1.1.0](#versión-110)
    - [Versión 1.0.0](#versión-100)

## **Descripción General**
Esta librería proporciona una interfaz para comunicarse con un acelerómetro y giroscopio utilizando el protocolo I2C en microcontroladores STM32. Permite la inicialización del sensor, la lectura de datos de aceleración en los ejes X, Y y Z, la lectura de datos del giroscopio en los ejes X, Y y Z, y la obtención de valores interpretados en unidades físicas.

## **Características**
- Comunicación mediante protocolo I2C.
- Lectura de datos de aceleración en los tres ejes (X, Y, Z).
- Conversión de datos en valores físicos (m/s² o g).
- Configuración inicial y reinicio del sensor.

## **Requisitos**
- STM32CubeIDE
- Librería HAL para STM32
- Protocolo I2C habilitado en el microcontrolador

## **Ejemplo de Uso**
```c
#include "MPU6050.h"

int main(void) {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;

    MPU6050_init();
    
    while (1) {
        MPU6050_Read_Accel(&acc_x, &acc_y, &acc_z);
        printf("X: %.2f, Y: %.2f, Z: %.2f\n", acc_x, acc_y, acc_z);

        MPU6050_Read_Gyro(&gyro_x, &gyro_y, &gyro_z);
        printf("X: %.2f, Y: %.2f, Z: %.2f\n", gyro_x, gyro_y, gyro_z);

        HAL_Delay(1000);
    }
}
```

## API Reference

### Tipo de dato para gestión de errores
```c
typedef enum {
    MPU6050_OK = 0,         /**< Operación exitosa */
    MPU6050_ERROR = 1,      /**< Error en la operación */
    MPU6050_TIMEOUT = 2,    /**< Timeout en la operación */
    MPU6050_NOT_INITIALIZED = 3	/** Sensor no inicializado */
}MPU6050_Status_t;
```

### **Funciones Principales**

---

#### `MPU6050_Init()`
Inicializa el módulo configurando los registros necesarios mediante I2C.
```c
MPU6050_Status_t MPU6050_Init(I2C_HandleTypeDef* hi2c);
```
**Parámetros:**
  - `I2C_HandleTypeDef* hi2c`: Puntero a la estructura de manejo del I2C.

**Retorna:**
  - `MPU6050_Status_t`: Estado de la operación (éxito o error).

---

#### `MPU6050_Read_Accel()`

```c
MPU6050_Status_t MPU6050_Read_Accel(
                  float *Ax,  /**< Puntero a valor de aceleración en eje X */
                  float *Ay,  /**< Puntero a valor de aceleración en eje Y */
                  float *Az   /**< Puntero a valor de aceleración en eje Z */
                );
```

**Parámetros:**
  - `float *Ax`: Puntero para almacenar el valor de aceleración en el eje X.
  - `float *Ay`: Puntero para almacenar el valor de aceleración en el eje Y.
  - `float *Az`: Puntero para almacenar el valor de aceleración en el eje Z.

**Retorna:**
  - `MPU6050_Status_t`: Estado de la operación (éxito o error).

#### `MPU6050_Read_Gyro()`

```c
MPU6050_Status_t MPU6050_Read_Gyro(
                  float *Gx,  /**< Puntero a valor de giroscopio en eje X */
                  float *Gy,  /**< Puntero a valor de giroscopio en eje Y */
                  float *Gz   /**< Puntero a valor de giroscopio en eje Z */
                );
```

**Parámetros:**
  - `float *Gx`: Puntero para almacenar el valor del giroscopio en el eje X.
  - `float *Gy`: Puntero para almacenar el valor del giroscopio en el eje Y.
  - `float *Gz`: Puntero para almacenar el valor del giroscopio en el eje Z.

**Retorna:**
  - `MPU6050_Status_t`: Estado de la operación (éxito o error).

## Conexión de hardware

| MPU6050 | STM32   |
|---------|---------|
| VCC     | 5V/3.3V |
| GND     | GND     |
| SDA     | I2C_SDA |
| SCL     | I2C_SCL |

>[!NOTE]
>Asegúrate de que la dirección I2C del MPU6050 coincida con la definida en el código (`0xD0` por defecto).

---

## 📄 Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog
### Version 2.0.1
- Eliminación de warnings en la compilación por variables no usadas.

### Version 2.0.0
- Se agregó gestión de errores mediante el tipo `MPU6050_Status_t`.
- Las funciones principales ahora retornan el estado de la operación.
- Se añadieron comentarios detallados para cada función y parámetro.

### Versión 1.2.0
- Se corrigieron errores menores en la documentación.
- Se incluye manejador de la interfaz I2C como parámetro en la función de inicialización `MPU6050_Init()`.

### Versión 1.1.0
- Cambio en nombre del archivo de `"accelerometer.h"` a `"MPU6050.h"` para reflejar mejor el contenido de la librería.
- Cambio en nombre de funciones de `accelerometer_read()` a `MPU6050_Read_Accel()` y `gyroscope_read()` a `MPU6050_Read_Gyro()` para mayor claridad.
- Cambio en el nombre de los punteros de entrada de `*x`, `*y`, `*z` a `*Ax`, `*Ay`, `*Az` para acelerómetro y `*Gx`, `*Gy`, `*Gz` para giroscopio, mejorando la legibilidad del código.

### Versión 1.0.0
- Versión inicial de la librería con funciones básicas de inicialización y lectura de datos.
- Soporte para lectura de aceleración y giroscopio en los tres ejes.
