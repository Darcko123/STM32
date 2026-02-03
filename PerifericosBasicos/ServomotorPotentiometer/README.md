# Control de Ángulo de un Servomotor mediante Potenciómetro usando STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)

---

## Tabla de Contenidos
- [Control de Ángulo de un Servomotor mediante Potenciómetro usando STM32](#control-de-ángulo-de-un-servomotor-mediante-potenciómetro-usando-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Introducción](#introducción)
  - [Características Principales](#características-principales)
  - [Configuración del Proyecto](#configuración-del-proyecto)
    - [Configuración del Timer](#configuración-del-timer)
    - [Configuración del ADC](#configuración-del-adc)
  - [Implementación](#implementación)
    - [Mapeo de Valores (ADC a Ángulo)](#mapeo-de-valores-adc-a-ángulo)
  - [Funcionamiento General](#funcionamiento-general)
  - [Licencia](#licencia)

## Introducción
Este proyecto demuestra cómo controlar el ángulo de un servomotor en función de la posición de un potenciómetro utilizando un microcontrolador STM32F103 y la HAL de STM32CubeIDE. Se hace uso de un ADC de 12 bits para leer el valor del potenciómetro y un temporizador para generar la señal PWM que controla al servomotor. El valor analógico se mapea a un ángulo de 0° a 180°, y el correspondiente ciclo de trabajo PWM se ajusta dinámicamente.

## Características Principales
- ✅ Control preciso del ángulo del servomotor (0°–180°)
- ✅ Mapeo eficiente de valores del ADC a valores PWM
- ✅ Compatible con la mayoría de servomotores estándar
- ✅ Uso de periféricos HAL de STM32 para facilitar la portabilidad

## Configuración del Proyecto

### Configuración del Timer
El Timer es usado para generar la señal PWM al servomotor. La configuración utilizada es:

| Parametro | Valor |
|-----------|-------|
| Frecuencia de Reloj | 72 Mhz |
| TIM | 1 |
| Canal | Channel1 |
| PSC | 144 |
| ARR | 9,999 |

Esta configuración permite generar un PWM con un periodo de 20 ms (50 Hz), adecuado para controlar servomotores de hobby.
> [!NOTE]
> Para más detalles sobre la configuración del Timer, consulta este repositorio complementario:🔗 [ServomotorWithPWM](https://github.com/Darcko123/STM32/tree/main/PerifericosBasicos/ServomotorWithPWM)


### Configuración del ADC
Se utiliza el canal `ADC_IN0`, con las siguientes configuraciones:

| Parametro | Valor |
|-----------|-------|
| Modo de Conversión Continua | Habilitado |
| Resolución | 12 bits (0 - 4095) | 
| Modo de escaneo | Deshabilitado | 
| Disparo Externo | Ninguno | 

El valor del ADC representa la posición del potenciómetro y se actualiza de forma continua.

> [!NOTE]
> Para más detalles sobre la configuración del ADC, consulta este repositorio complementario:🔗 [ADC](https://github.com/Darcko123/STM32/tree/main/PerifericosBasicos/ADC)

## Implementación

### Mapeo de Valores (ADC a Ángulo)

Para convertir el valor del ADC (0–4095) a un ángulo (0°–180°), se utiliza una función map() que implementa una interpolación lineal:

```C
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
```
Esta función se basa en la ecuación general de una recta que pasa por dos puntos conocidos, aplicando el siguiente razonamiento matemático:

Sea una recta definida por dos puntos A(x₁, y₁) y B(x₂, y₂). La pendiente M se calcula como:

$M = \frac{(y_2 - y_1)}{(x_2 - x_1)}$

Para un punto genérico (X, Y), se tiene:

$Y = \frac{(X - x_2)(y_2 - y_1)}{x_2 - x_1} + y_1$

Este mismo principio se aplica en la función `map()`, lo que permite obtener el valor correspondiente en el nuevo rango.

## Funcionamiento General
1. El ADC lee continuamente el valor del potenciómetro.
2. El valor leído se mapea a un ángulo entre 0° y 180°.
3. El ángulo se convierte a un valor de ciclo de trabajo del PWM (duty cycle).
4. Se actualiza el pulso PWM generado por el Timer para controlar el servomotor.

---

## Licencia
Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.