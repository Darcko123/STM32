# Librería para el Acelerómetro en STM32

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

## **Archivos de la Librería**
- `MPU6050.h`: Archivo de cabecera con declaraciones de funciones y definiciones.
- `MPU6050.c`: Implementación de las funciones para el manejo del acelerómetro y giroscopio.

## **Funciones Principales**

### `void MPU6050_init(void)`
Inicializa el módulo configurando los registros necesarios mediante I2C.

### `void MPU6050_Read_Accel(float *Ax, float *Ay, float *Az);`
Obtiene los valores de aceleración en los ejes X, Y y Z.
- **Parámetros:**
  - `float *Ax`: Puntero para almacenar el valor de aceleración en el eje X.
  - `float *Ay`: Puntero para almacenar el valor de aceleración en el eje Y.
  - `float *Az`: Puntero para almacenar el valor de aceleración en el eje Z.

### `voidMPU6050_Read_Gyro(float *Gx, float *Gy, float *Gz)`
Obtiene los valores del giroscopio en los ejes X, Y y Z.
- **Parámetros:**
  - `float *Gx`: Puntero para almacenar el valor del giroscopio en el eje X.
  - `float *Gy`: Puntero para almacenar el valor del giroscopio en el eje Y.
  - `float *Gz`: Puntero para almacenar el valor del giroscopio en el eje Z.

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

## **Notas Importantes**
- Asegúrate de configurar correctamente los pines I2C en STM32CubeMX.
- Verifica la dirección I2C del acelerómetro en la hoja de datos del sensor.

## **Documentación Adicional**
Consulta la hoja de datos oficial del acelerómetro para más detalles sobre los registros y configuraciones.

## **Autor**
- **Nombre:** Daniel Ruiz
- **Fecha:** Sep 26, 2024
- **Versión:** 1.1
