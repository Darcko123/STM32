# Librería SI7021 para STM32

## Descripción
Esta librería permite la comunicación con el sensor **SI7021** para la medición de **temperatura** y **humedad relativa** mediante el protocolo **I2C** en microcontroladores STM32.

El sensor SI7021 es ampliamente utilizado en aplicaciones de monitoreo ambiental, HVAC y sistemas embebidos debido a su precisión y facilidad de integración.

---

## Características
- Comunicación mediante protocolo **I2C**.
- Medición de **temperatura** en grados Celsius (°C).
- Medición de **humedad relativa** (%HR).
- Función de inicialización con **reset por hardware**.
- Conversión de datos crudos mediante las fórmulas especificadas en el **datasheet del SI7021**.

---

## Dependencias
- **STM32 HAL Library** (Hardware Abstraction Layer).
- Configuración adecuada del periférico I2C en STM32CubeIDE.

---

## Archivos
- **si7021.h**: Archivo de cabecera con prototipos de funciones y definiciones.
- **si7021.c**: Implementación de las funciones de inicialización y lectura de datos.

---

## Funciones Principales

### 1. `void si7021_init(void)`
- **Descripción:** Inicializa el sensor enviando un comando de **reset**.
- **Uso:** Esta función debe ser llamada antes de realizar lecturas.
- **Parámetros:** Ninguno.
- **Retorno:** Ninguno.

### 2. `void get_si7021(float *temp, float *humid)`
- **Descripción:** Obtiene los valores de temperatura y humedad del sensor.
- **Parámetros:**
   - `float *temp`: Puntero donde se almacenará la temperatura en °C.
   - `float *humid`: Puntero donde se almacenará la humedad en %HR.
- **Retorno:** Ninguno.

---

## Ejemplo de Uso

```c
#include "si7021.h"

float temperatura = 0.0;
float humedad = 0.0;

int main(void)
{
    HAL_Init();
    si7021_init();

    while(1)
    {
        get_si7021(&temperatura, &humedad);
        printf("Temperatura: %.2f °C\n", temperatura);
        printf("Humedad: %.2f %%\n", humedad);
        HAL_Delay(1000);
    }
}
```

---

## Notas Importantes
- Asegúrate de configurar correctamente el **I2C** en tu microcontrolador STM32.
- Revisa las direcciones I2C específicas del sensor en tu aplicación.
- Consulta el **datasheet del SI7021** para ajustes avanzados.

---

## Autor
- **Daniel Ruiz**
- **Fecha:** Diciembre 28, 2024
- **Versión:** 1.0

---