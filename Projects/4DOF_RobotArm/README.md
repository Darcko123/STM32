# 4DOF KUKA Robot Arm 

[![STM32](https://img.shields.io/badge/Platform-STM32F411-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![I2C](https://img.shields.io/badge/Protocol-I2C-yellow)](https://es.wikipedia.org/wiki/I2C)
[![UART](https://img.shields.io/badge/Protocol-UART-yellow)](https://es.wikipedia.org/wiki/Universal_Asynchronous_Receiver-Transmitter)
[![PCA9685](https://img.shields.io/badge/Drive-PCA9685-green)](https://github.com/Darcko123/STM32/tree/main/Librerias/PWM_Module)


## Tabla de Contenidos
- [4DOF KUKA Robot Arm](#4dof-kuka-robot-arm)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Introducción Teórica](#introducción-teórica)
    - [Dimensiones de robot KUKA LBR IISY 4DOF a escala](#dimensiones-de-robot-kuka-lbr-iisy-4dof-a-escala)
    - [Límites de Articulaciones](#límites-de-articulaciones)
    - [Parámetros Denavit-Hartenberg](#parámetros-denavit-hartenberg)
  - [Características Principales](#características-principales)
  - [Licencia](#licencia)

## Descripción
Este proyecto consiste en la implementación del control con cinemática directa e inversa de un brazo robótico KUKA de 4 grados de libertad (4DOF) utilizando un microcontrolador STM32F411. El sistema permite controlar la posición y orientación del efector final del brazo mediante comandos enviados a través de una interfaz UART, y utiliza el módulo PCA9685 para manejar los servomotores que accionan las articulaciones del brazo.

![Modelo 3D del Robot](/Projects/4DOF_RobotArm/Images/Modelo3DRobot.png)

> [!NOTE]
> El modelo 3D se adquirió desde [Brazo robótico 4DOF (basado en Cobot KUKA)](https://cults3d.com/es/modelo-3d/artilugios/4dof-robotic-arm-cobot-kuka-based)

## Introducción Teórica
El brazo robótico KUKA de 4DOF es un manipulador que consta de cuatro articulaciones que permiten movimientos en tres dimensiones espaciales. La cinemática directa se utiliza para calcular la posición del efector final a partir de los ángulos de las articulaciones, mientras que la cinemática inversa permite determinar los ángulos necesarios para alcanzar una posición deseada del efector final.

Esta se puede calcular a través de las matrices de transformación homogénea, que describen la relación entre los sistemas de coordenadas de cada articulación y el efector final. La implementación de estos cálculos en el microcontrolador STM32F411 permite un control preciso y eficiente del brazo robótico.

Matemáticamente la formula se representa de la siguiente manera:

$$
H04 = H01 * H12 * H23 * H34
$$

Donde cada matriz de transformación homogénea $H_{ij}$ se define en función de los parámetros DH (Denavit-Hartenberg) del robot.

Cada matriz de transformación homogénea $T_\frac{i-1}{i}$ se define como:

$$
T_\frac{i-1}{i} = \begin{bmatrix}
\cos(\theta_i) & -\sin(\theta_i)\cos(\alpha_i) & \sin(\theta_i)\sin(\alpha_i) & a_i\cos(\theta_i) \\
\sin(\theta_i) & \cos(\theta_i)\cos(\alpha_i) & -\cos(\theta_i)\sin(\alpha_i) & a_i\sin(\theta_i) \\
0 & \sin(\alpha_i) & \cos(\alpha_i) & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Los parametros $\alpha_i$, $a_i$, $d_i$ y $\theta_i$ están relacionadas al eslabón i:
  - $a_i$: Longitud del eslabón
  - $\alpha_i$: Ángulo entre el eje z del eslabón i-1 y el eje z del eslabón i
  - $d_i$: Desplazamiento a lo largo del eje z del eslabón i
  - $\theta_i$: Ángulo entre el eje x del eslabón i-1 y el eje x del eslabón i

La matriz $T_\frac{i-1}{i}$ es función de una sola variable de junta (tipicamente) y 3 de los 4 parametros son constantes para cada eslabón determinado. En el caso de una articulación de *revoluta* $\theta_i$ es la variable, mientras qye $d_i$ es la variables para una junta *prismática*

### Dimensiones de robot KUKA LBR IISY 4DOF a escala
Las dimensiones geómetricas del modelo son:

| Parámetro | Símbolo | Valor (mm) | Descripción |
|-----------|---------|------------|-------------|
| Altura Base | H0 | 45.0 | Altura desde el suelo |
| Offset Vertical | L1 | 30.0 | Joint1 a plano Joint2 |
| Brazo Proximal | L2 |	87.0 |Hombro (Shoulder) |
| Brazo Distal | L3 | 70.0 | Codo (Elbow) |
| Antebrazo | L4 | 44.0 | Muñeca (Wrist) |
| Pinza | LGRIP | 45.0 | Longitud de agarre |

### Límites de Articulaciones

| Articulación | Mínimo (°) | Máximo (°) | Rango Total |
|-----------|---------|------------|-------------|
| Base (Joint 1) | 0.0 | 180.0 | 180.0 |
| Hombro (Joint 2) | 0.0 | 180.0 | 180.0 |
| Codo (Joint 3) | 0.0 | 180.0 |180.0 |
| Muñeca (Joint 4) | 0.0 | 180.0 | 180.0 |

### Parámetros Denavit-Hartenberg
>[!NOTE]
> - $d_1 = Base + Eslabón_1 = 45mm + 30mm = 75mm$
> - $\alpha_1 = -90°$ ya que sufre una rotación para un cambio de eje
> - $\alpha_2 = \alpha_3 = \alpha_4 = 0°$ ya que son paralelos

La fórmula para la Transformación D-H es:

$$
T_\frac{i-1}{i} = rot_z(\theta_i) \times trans_z(d_i) \times trans_x(a_i) \times rot_x(\alpha_i)
$$

La tabla con los valores paramtrizables es:

| $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|-----|------------|-------|-------|------------|
|  **1**  | $\theta_1$* | H0 + L1 | 0 | $-\pi/2$ |
|  **2**  | -$\theta_2$* | 0 | L2 | 0° |
|  **3** | $\theta_3$* | 0  | L3 | 0° |
|  **4**  | $\theta_4$* | 0  | L4 | 0° |

Para lo cuál, la tabla de este modelo quedaría como:

| $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|-----|------------|-------|-------|------------|
|  **1**  | $\theta_1$* | 75 mm | 0 mm | -90° |
|  **2**  | -$\theta_2$* | 0 mm | 87 mm | 0° |
|  **3** | $\theta_3$* | 0 mm | 70 mm | 0° |
|  **4**  | $\theta_4$* | 0 mm | 44 mm | 0° |

>[!NOTE]
> `*` para diferenciar constantes de variables de junta

## Características Principales
- Control de 4 servomotores mediante el módulo PCA9685.
- Cálculo de cinemática directa e inversa para posicionamiento del efector final
- Interfaz de comunicación UART para recibir comandos de posición.
- Implementación en microcontrolador STM32F411.
- Estructura modular del código para facilitar futuras expansiones y mantenibilidad.

## Licencia

Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.