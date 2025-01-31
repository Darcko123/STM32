# Control de Servomotor con STM32 y PWM

## Introducción
Este documento explica la configuración y cálculo de parámetros para el control de un servomotor utilizando un microcontrolador STM32. Se detalla el cálculo de la frecuencia de trabajo del PWM, la configuración del timer y la conversión de ángulos a valores de comparación para generar los pulsos correctos.

## Frecuencia de Trabajo del Servomotor
Un servomotor típico opera con una frecuencia de 50 Hz, lo que significa que el periodo de la señal PWM es:

$\[ T = \frac{1}{frecuencia} = \frac{1}{50Hz} = 0.02s = 20ms \]$

## Configuración del Timer
El periodo del PWM se define mediante la siguiente ecuación:

$\[ T_{PWM} = \frac{(CounterPeriod + 1) (Prescaler + 1)}{F_{clk}} \]$

En un STM32F411, la frecuencia del reloj del bus APB1, donde se encuentran los timers, es de 100 MHz. Para simplificar los cálculos, tomamos:

- `CounterPeriod = 9,999 + 1 = 10,000`
- `T_PWM = 20 ms`

Sustituyendo en la ecuación:

$\[ 20ms = \frac{(10,000) (Prescaler + 1)}{100MHz} \]$

Resolviendo para el prescaler:

$\[ (Prescaler + 1) = \frac{(20ms)(100MHz)}{10,000} = 200 \]$

Por lo tanto, los valores de configuración del timer son:

- **Frecuencia del reloj del microcontrolador:** 100 MHz
- **Prescaler:** 200
- **Counter Period:** 10,000

### Ejemplo de Cálculo para STM32F103
Si tomamos un microcontrolador **STM32F103**, donde la frecuencia máxima del bus APB1 es **72 MHz** y mantenemos el mismo `CounterPeriod = 10,000`, el cálculo del prescaler sería:

$\[ 20ms = \frac{(10,000) (Prescaler + 1)}{72MHz} \]$

Resolviendo:

$\[ (Prescaler + 1) = \frac{(20ms)(72MHz)}{10,000} \]$

$\[ (Prescaler + 1) = \frac{(0.02s)(72,000,000Hz)}{10,000} \]$

$\[ (Prescaler + 1) = 144 \]$

Por lo que el prescaler adecuado para un STM32F103 sería **143** (recordando que el valor del prescaler en STM32 es `Prescaler + 1`).

## Cálculo del Tiempo de Pulso
El prescaler divide la frecuencia del reloj:

$\[ F_{timer} = \frac{100,000,000Hz}{200} = 500,000Hz \]$

Cada "tick" del timer dura:

$\[ T_{tick} = \frac{1}{500,000Hz} = 2us \]$

El periodo del PWM se obtiene como:

$\[ T_{PWM} = 10,000 \times 2us = 20ms \]$

Esto coincide con la señal estándar de un servomotor.

## Conversión de Ángulo a Pulso
El ancho del pulso se traduce a valores del timer mediante la ecuación:

$\[ ValorDeComparación = \frac{AnchoDelPulsoEnms}{T_{tick}} \]$

Para los valores estándar de un servomotor:

- **0° (0.5 ms):**
  $\[ (0.5 \times 10^{-3}) / (2 \times 10^{-6}) = 250 \]$

- **180° (2.5 ms):**
  $\[ (2.5 \times 10^{-3}) / (2 \times 10^{-6}) = 1250 \]$

Por lo tanto:

- `250` representa **0°** (pulso de 0.5 ms)
- `1250` representa **180°** (pulso de 2.5 ms)

## Ecuación para Mapear Ángulos
Para calcular el valor del pulso en función del ángulo, utilizamos la ecuación de la recta:

$\[ pulse = m \times angulo + b \]$

Donde:

- **m** es la pendiente:
  $\[ m = \frac{1250 - 250}{180 - 0} = \frac{1000}{180} \approx 5.56 \]$
- **b** es la intersección con el eje Y, que en este caso es 250 (valor para 0°).

Por lo que la ecuación final es:

$\[ pulse = 5.56 \times angulo + 250 \]$

Este cálculo permite convertir un ángulo de 0 a 180 grados en el valor de comparación correspondiente para el PWM del servomotor en STM32.

---
**Notas:**
- Se debe ajustar el timer y los valores en función de la frecuencia de trabajo del microcontrolador.
- La ecuación lineal permite una conversión precisa del ángulo a pulsos de control del servo.

