# Control de Servomotor con STM32 y PWM
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)

---

## Tabla de Contenidos
- [Control de Servomotor con STM32 y PWM](#control-de-servomotor-con-stm32-y-pwm)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características Principales](#características-principales)
  - [Requisitos](#requisitos)
  - [Introducción Teórica](#introducción-teórica)
    - [Frecuencia de Trabajo del Servomotor](#frecuencia-de-trabajo-del-servomotor)
    - [Configuración del Temporizador (Timer)](#configuración-del-temporizador-timer)
      - [Ejemplo de Cálculo para STM32F103](#ejemplo-de-cálculo-para-stm32f103)
    - [Cálculo del Tiempo de Pulso](#cálculo-del-tiempo-de-pulso)
    - [Conversión de Ángulo a Valor de Comparación (CCR)](#conversión-de-ángulo-a-valor-de-comparación-ccr)
    - [Ecuación General para el Mapeo de Ángulos](#ecuación-general-para-el-mapeo-de-ángulos)
  - [Implementación](#implementación)
    - [Inicialización del PWM](#inicialización-del-pwm)
    - [Función para el Control de Posición (`set_servo_angle`)](#función-para-el-control-de-posición-set_servo_angle)
      - [Uso de la función](#uso-de-la-función)
    - [Verificación con Osciloscopio](#verificación-con-osciloscopio)
      - [Posición a 0° (CCR = 250)](#posición-a-0-ccr--250)
      - [Posición a 180° (CCR = 1250)](#posición-a-180-ccr--1250)
      - [Análisis de las mediciones:](#análisis-de-las-mediciones)
  - [Licencia](#licencia)

---

## Descripción
Este documento describe la metodología para configurar y controlar un servomotor mediante modulación por ancho de pulsos (PWM) utilizando un microcontrolador de la familia STM32. Se explican en detalle los cálculos para establecer la frecuencia de trabajo, la configuración de los registros del temporizador (Timer) y el mapeo de ángulos de posición a valores de comparación (CCR), garantizando la generación de pulsos precisos para el posicionamiento del servomotor.

## Características Principales
- **Posicionamiento Angular Preciso**: Control del servomotor en grados con alta resolución.
- **Compatibilidad Amplia**: Implementación adaptable a la mayoría de servomotores estándar de 50 Hz.
- **Eficiencia en Código**: Utiliza las bibliotecas Hardware Abstraction Layer (HAL) de STM32CubeIDE para una implementación robusta y portable.

---

## Requisitos
- STM32CubeIDE o STM32CubeMX.
- Biblioteca HAL correspondiente a tu microcontrolador STM32.
- Sermotor
- Fuente de Alimentación de 5V

## Introducción Teórica

### Frecuencia de Trabajo del Servomotor
Los servomotores de tipo estándar (como los SG90 o MG90S) operan con una señal PWM de referencia de 50 Hz. Esta frecuencia corresponde a un periodo de señal de:

$T = \frac{1}{f} = \frac{1}{50 \text{ Hz}} = 0.02 \text{ s} = 20 \text{ ms}$

La precisión en el control depende críticamente de mantener este periodo constante.

### Configuración del Temporizador (Timer)
El periodo de la señal PWM generada por el microcontrolador se determina mediante la configuración del temporizador, siguiendo la ecuación:

$T_{PWM} = \frac{(\text{ARR} + 1) \times (\text{PSC} + 1)}{F_{\text{clk}}}$

Donde:
- $T_{PWM}$ es el periodo deseado de la señal PWM (ej. 20 ms).
- $\text{ARR}$ (Auto-Reload Register) es el valor del "Counter Period".
- $\text{PSC}$ (Prescaler) es el divisor de la frecuencia del reloj.
- $F_{\text{clk}}$ es la frecuencia del reloj que alimenta al temporizador.

Para un STM32F411 (con el reloj del bus APB1 a 100 MHz), seleccionando un valor de $ARR = 9,999$ (lo que da un periodo de recarga de 10,000 ciclos) y un $T_{PWM}$ objetivo de 20 ms, el cálculo del prescaler es:

$20 \text{ ms} = \frac{(10,000) \times (\text{PSC} + 1)}{100 \text{ MHz}}$

Resolviendo para $\text{PSC}$:

$(\text{PSC} + 1) = \frac{(20 \times 10^{-3}) \times (100 \times 10^{6})}{10,000} = 200$

**Parámetros de configuración resultantes para STM32F411**:

- **Frecuencia de reloj del Timer:** 100 MHz
- **Prescaler (PSC):** 199
- **Periodo del Auto-Reload (ARR):** 9,999

> [!Note]
> En las bibliotecas HAL, el parámetro `Counter Period` se corresponde con el valor ARR. El valor del prescaler que se carga en el registro es `PSC = (divisor deseado) - 1`.

#### Ejemplo de Cálculo para STM32F103
Para un microcontrolador **STM32F103** (con frecuencia APB1 típica de 72 MHz), manteniendo el mismo ARR (10,000 ciclos) y periodo objetivo (20 ms), el cálculo se ajusta:

$20 \text{ ms} = \frac{(10,000) \times (\text{PSC} + 1)}{72 \text{ MHz}}$

$(\text{PSC} + 1) = \frac{(0.02) \times (72 \times 10^{6})}{10,000} = 144$

Por lo tanto, el valor a cargar en el registro del prescaler sería:
$\text{PSC} = 144 - 1 = 143$

### Cálculo del Tiempo de Pulso
El prescaler divide la frecuencia del reloj:

$F_{timer} = \frac{100,000,000Hz}{200} = 500,000Hz$

Cada "tick" del timer dura:

$T_{tick} = \frac{1}{500,000Hz} = 2us$

El periodo del PWM se obtiene como:

$T_{PWM} = 10,000 \times 2us = 20ms$

Esto coincide con la señal estándar de un servomotor.

### Conversión de Ángulo a Valor de Comparación (CCR)
El control de posición se logra variando el ancho del pulso dentro del periodo fijo de 20 ms. Este ancho de pulso debe convertirse en un valor numérico que el temporizador pueda comparar, conocido como **Capture/Compare Register (CCR)**. La relación se deriva del tiempo por ciclo del temporizador ($T_{tick}$):

$\text{CCR} = \frac{\text{Ancho de Pulso}[\text{s}]}{T_{tick}}$

Para la configuración establecida en el STM32F411 ($T_{tick} = 2 \mu\text{s}$), los valores de CCR para los límites de operación típicos de un servomotor son:

- Posición a 0° (ancho de pulso = 0.5 ms):
    $\text{CCR}_{0^\circ} = \frac{0.5 \times 10^{-3} \text{s}}{2 \times 10^{-6} \text{s}} = 250$

- Posición a 180° (ancho de pulso = 2.5 ms):
    $\text{CCR}_{180^\circ} = \frac{2.5 \times 10^{-3} \text{s}}{2 \times 10^{-6} \text{s}} = 1250$

**Resumen de Mapeo**
| Ángulo | Ancho de Pulso | Valor de CCR (Capture/Compare Register) |
|--------|----------------|-----------------------------------------|
| 0° | 0.5 ms | 250 |
| 180° | 2.5 ms | 1250 |

### Ecuación General para el Mapeo de Ángulos
Para obtener el valor de CCR correspondiente a cualquier ángulo intermedio (de 0° a 180°), se puede establecer una relación lineal. Esta conversión se modela mediante la ecuación de una recta:

$\text{CCR}(\theta) = m \cdot \theta + b$

Donde:

- $\theta$ es el ángulo deseado en grados.
- $m$ es la pendiente (incremento de CCR por grado).
- **b** es la intersección con el eje Y, que en este caso es 250 (valor para 0°).

Con los puntos de calibración $(0, 250)$ y $(180, 1250)$, los parámetros se calculan como:

1. **Pendiente (m):**

    $m = \frac{\text{CCR}{180^\circ} - \text{CCR}{0^\circ}}{180^\circ - 0^\circ} = \frac{1250 - 250}{180} = \frac{1000}{180} \approx 5.5556$

2. **Ordenada al origen (b):**

    $b = \text{CCR}_{0^\circ} = 250$

Aplicación: Esta ecuación permite al programa convertir un ángulo objetivo (ej., 90°) directamente en el valor entero que debe escribirse en el registro CCR del temporizador para generar el ancho de pulso preciso, logrando un control de posición en lazo abierto.

---

## Implementación
Esta sección detalla la implementación del control del servomotor en código C utilizando las bibliotecas HAL de STM32CubeIDE. Se asume que el temporizador (Timer) y su canal PWM han sido configurados previamente en STM32CubeMX con los parámetros calculados:

| Parametro | Valor |
|-----------|-------|
| Frecuencia de Reloj | 100 Mhz |
| TIM | 1 |
| Canal | Channel1 |
| PSC | 199 |
| ARR | 9,999 |

### Inicialización del PWM
Una vez configurado el hardware en CubeMX y generado el código de inicialización, es necesario iniciar la generación de la señal PWM en el canal correspondiente del temporizador. Esto se realiza típicamente en la función `main()`, después de la inicialización de todos los periféricos.

```C
/* USER CODE BEGIN 2 */
  // Inicia la generación de PWM en el Timer 1, Canal 1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
/* USER CODE END 2 */
```

> [!Note]
> Asegúrese de que la variable `htim` (manejador del Timer) y el canal (`TIM_CHANNEL_X`) correspondan a su configuración específica en STM32CubeMX.

### Función para el Control de Posición (`set_servo_angle`)
Para posicionar el servomotor en un ángulo específico, se implementa una función que calcula y establece el valor de comparación (CCR) en el registro correspondiente del temporizador, según la ecuación de mapeo derivada.

```C
/**
  * @brief  Establece el ángulo de posición del servomotor.
  * @param  htim   Puntero al manejador de la estructura TIM_HandleTypeDef.
  * @param  channel Canal del Timer configurado para PWM (e.g., TIM_CHANNEL_1).
  * @param  angle  Ángulo deseado, en un rango de 0 a 180 grados.
  * @retval None
  *
  * @detail La función mapea linealmente el ángulo al valor del registro CCR
  *         utilizando la ecuación: CCR(θ) = (5.5556 * θ) + 250.
  *         Los límites prácticos son:
  *           0°   -> CCR = 250  (pulso de 0.5 ms)
  *           180° -> CCR = 1250 (pulso de 2.5 ms)
  */
void set_servo_angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle)
{
  // 1. Cálculo del valor de comparación (CCR) mediante mapeo lineal.
  //    Se utiliza un factor de conversión en punto flotante para mayor precisión.
  uint16_t pulse = (uint16_t)( (5.5556f * (float)angle) + 250.0f );

  // 2. Asignación del valor calculado al registro de comparación del canal.
  //    Esta macro de HAL actualiza el CCR y modifica inmediatamente el ancho del pulso.
  __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}
```

#### Uso de la función
```C
// Ejemplo: Mover el servomotor a 90 grados.
set_servo_angle(&htim1, TIM_CHANNEL_1, 90);

// Ejemplo: Mover el servomotor a 0 grados (posición inicial).
set_servo_angle(&htim1, TIM_CHANNEL_1, 0);
```

### Verificación con Osciloscopio
La correcta generación de la señal PWM puede verificarse conectando un osciloscopio al pin de control del servomotor. Las siguientes capturas muestran los parámetros clave para las posiciones límite:

#### Posición a 0° (CCR = 250)

![0Degrees](/PerifericosBasicos/ServomotorWithPWM/Images/0Degrees.jpg)

**Mediciones características:**
- **Frecuencia:** 50.0 Hz (constante)
- **Periodo (T):** 20.0 ms
- **Ancho de pulso (Width):** 0.50 ms
- **Duty Cycle:** 2.5% (0.5 ms / 20 ms)

*La señal cumple con el estándar de 0.5 ms de pulso para la posición mínima del servomotor.*

#### Posición a 180° (CCR = 1250)

![180Degrees](/PerifericosBasicos/ServomotorWithPWM/Images/180Degrees.jpg)

**Mediciones características:**
- **Frecuencia:** 50.0 Hz (constante)
- **Periodo (T):** 20.0 ms
- **Ancho de pulso (Width):** 2.50 ms
- **Duty Cycle:** 12.5% (2.5 ms / 20 ms)

*La señal cumple con el estándar de 2.5 ms de pulso para la posición máxima del servomotor.*

#### Análisis de las mediciones:

1. **Frecuencia y Periodo:**
    Ambas capturas confirman que la configuración del temporizador genera una señal PWM estable de 50 Hz (20 ms de periodo), independientemente del ángulo. Esto es fundamental para el correcto funcionamiento del servomotor.

2. **Ancho de pulso:**
    Las mediciones de 0.5 ms y 2.5 ms coinciden exactamente con los cálculos teóricos para 0° y 180°, validando la precisión de la ecuación de mapeo y la configuración del `T_{tick}` (2 µs).

3. **Duty Cycle Variable:**
    Aunque la frecuencia es fija, el ciclo de trabajo varía entre ~2.5% y 12.5%. Es este cambio en el ancho del pulso dentro del periodo fijo lo que el servomotor interpreta como una posición angular.

---

## Licencia
Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.
