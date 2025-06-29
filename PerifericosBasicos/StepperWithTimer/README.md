# Control de motor a pasos con STM32 y Timer

## Introducción
Este documento explica la configuración y cálculo de parámetros para el control de un motor a pasos utilizando un microcontrolador STM32. Se detalla el cálculo de la frecuencia de trabajo del timer como delay para el control de las RPM del motor a pasos.

## Configuración de activación de las bobinas del motor a pasos
Un motor a pasos típico como lo es el 28BYJ-48 con el circuito integrado ULN200 cuenta con diferentes modos de uso con diferentes objetivos, los cuales son:

### - Full Drive
En este modo se activan 2 electroimanes del estator al mismo tiempo, lo que provoca una activación de de 2 fases simultaneamente, por lo tanto, genera un **mayor torque**. El motor tarda 2048 pasos en completar una revolución, lo que significa 2048/4 pasos es igual a 512 secuencias. La tabla de activación es la siguiente:

| `Step` | `A` | `B` | `C` | `D` |
| --- | --- | --- | --- |
| `1` | 1 | 1 | 0 | 0 |
| `2` | 0 | 1 | 1 | 0 |
| `3` | 0 | 0 | 1 | 1 |
| `4` | 1 | 0 | 0 | 1 |

### - Half Drive
En este modo se activan 1 y 2 fases alternativamente. De esta manera se aumenta la resolución angular del motor, pero **reduce el torque**. En esta configuración el motor tarda 4096 pasos en completar una revolución, lo que equivale a 4096/8 pasos igual a 512 secuencias. La tabla de activación es la siguiente:

| `Step` | `A` | `B` | `C` | `D` |
| --- | --- | --- | --- |
| `1` | 1 | 0 | 0 | 0 |
| `2` | 1 | 1 | 0 | 0 |
| `3` | 0 | 1 | 0 | 0 |
| `4` | 0 | 1 | 1 | 0 |
| `5` | 0 | 0 | 1 | 0 |
| `6` | 0 | 0 | 1 | 1 | 
| `7` | 0 | 0 | 0 | 1 |
| `8` | 1 | 0 | 0 | 1 |