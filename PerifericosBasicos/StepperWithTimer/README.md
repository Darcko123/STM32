# Control de motor a pasos con STM32 y Timer
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)


## Introducción
Este proyecto implementa el control preciso de un motor paso a paso (28BYJ-48) utilizando un microcontrolador STM32F103. Incluye configuraciones para diferentes modos de operación (full-step y half-step), control de velocidad (RPM) mediante temporizador hardware, y movimiento angular preciso.

## Características Principales
- ✅ Control en modos full-step y half-step
- ✅ Velocidad configurable mediante Timer hardware
- ✅ Movimiento por ángulos exactos
- ✅ Compatible con motores 28BYJ-48 con driver ULN2003
- ✅ Implementación eficiente usando HAL de STM32CubeIDE

### Componentes Requeridos
- Placa STM32F103 (Blue Pill u otra compatible)
- Motor paso a paso 28BYJ-48
- Driver ULN2003
- Fuente de alimentación 5V
- Cables de conexión

### Diagrama de Conexiones
STM32F103   |   ULN2003
---------      --------
PA1        ->    IN1  
PA2        ->    IN2  
PA3        ->    IN3  
PA4        ->    IN4  
GND        ->    GND  
5V         ->    Vcc (alimentación del motor)

## Modo de operación

Un motor a pasos típico como lo es el 28BYJ-48 con el circuito integrado ULN200 cuenta con diferentes modos de uso con diferentes objetivos, los cuales son:

### - Full Drive
En este modo se activan 2 electroimanes del estator al mismo tiempo, lo que provoca una activación de de 2 fases simultaneamente, por lo tanto, genera un **mayor torque**. El motor tarda 2048 pasos en completar una revolución, lo que significa 2048/4 pasos es igual a 512 secuencias. La tabla de activación es la siguiente:

| `Step` | `A` | `B` | `C` | `D` |
| --- | --- | --- | --- | --- |
| `1` | 1 | 1 | 0 | 0 |
| `2` | 0 | 1 | 1 | 0 |
| `3` | 0 | 0 | 1 | 1 |
| `4` | 1 | 0 | 0 | 1 |

### - Half Drive
En este modo se activan 1 y 2 fases alternativamente. De esta manera se aumenta la resolución angular del motor, pero **reduce el torque**. En esta configuración el motor tarda 4096 pasos en completar una revolución, lo que equivale a 4096/8 pasos igual a 512 secuencias. La tabla de activación es la siguiente:

| `Step` | `A` | `B` | `C` | `D` |
| --- | --- | --- | --- | --- |
| `1` | 1 | 0 | 0 | 0 |
| `2` | 1 | 1 | 0 | 0 |
| `3` | 0 | 1 | 0 | 0 |
| `4` | 0 | 1 | 1 | 0 |
| `5` | 0 | 0 | 1 | 0 |
| `6` | 0 | 0 | 1 | 1 |
| `7` | 0 | 0 | 0 | 1 |
| `8` | 1 | 0 | 0 | 1 |

## Configuración de Timer

Para generar un retardo preciso en microsegundos, se utiliza uno de los Timers generales del STM32F103 (por ejemplo, TIM1 o TIM2) en modo de **contador ascendente continuo**, configurando adecuadamente su prescaler y auto-reload register (ARR).

La frecuencia del reloj del bus APB1 en el STM32F103 es de 72 MHz, y es la frecuencia base para muchos timers. El timer cuenta desde 0 hasta el valor en el ARR, incrementándose con cada "tick" del reloj del timer, cuya duración depende del prescaler:

$\ T_{tick} = \frac{1}{F_{clk}} * (Prescaler + 1) \$

$\ T_{delay} = (ARR + 1) * T_{tick}\$

Para lograr un delay en microsegundos, necesitamos que cada tick del timer equivalga a 1 µs:

$\ 1 us = 1 MHz\$
$\ Prescaler = \frac{F_{clk}}{1 MHz} -1  = \frac{72,000,000}{1,000,000} - 1 = 71\$

Prescaler = 71 nos da un incremento de 1 us.


De este modo, un contador que llegue a x representará un retardo de x microsegundos.

De manera que:

- `Prescaler = 71`
- `CounterPeriod(ARR) = 0xFFFF`
- `Counter Mode = UP`
- `Clock Source = Internal Clock`
Obtenemos:

### Función para el control del delay en microsegundos

``` C
void delay (uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}
```

> [!NOTE]
> NOTA: El Timer debe estar inicializado y en ejecución antes de usar delay_us(). Por ejemplo:
>```C
> HAL_TIM_Base_Start(&htim1);
>```
> - Para evitar bloqueos o comportamientos erráticos:
>   - No usar retardos muy largos (mayores a 65,535 µs) con un contador de 16 bits.
>   - Evitar usar este tipo de retardo en tareas críticas donde se requiera bajo consumo o multitarea.

### Control de RPM
``` C
void stepper_set_rpm(int rpm)
{
    delay(60000000/(stepsperrev*rpm));
}
```

## Funciones principales
``` C
// Gira un ángulo específico
void stepper_step_angle(float angle, int direction, int rpm);

// Control en modo half-step
void stepper_half_drive(int step);

// Control en modo full-step
void stepper_full_drive(int step);
```

### Ejemplo de movimiento:
``` C
// Gira 90° en sentido horario a 10 RPM
stepper_step_angle(90, CLOCKWISE, 10);

// Gira 180° en sentido antihorario a 5 RPM
stepper_step_angle(180, COUNTER_CLOCKWISE, 5);
```

# 📝 Licencia
Este proyecto está bajo la licencia MIT.
Eres libre de usar, modificar y distribuir este código para cualquier propósito con o sin fines comerciales.

Código y explicación adaptadas de: [CONTROLLERSTECH](https://controllerstech.com/interface-stepper-motor-with-stm32/)