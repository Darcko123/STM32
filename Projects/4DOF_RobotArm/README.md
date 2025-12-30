# 4DOF KUKA Robot Arm 

[![STM32](https://img.shields.io/badge/Platform-STM32F411-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![I2C](https://img.shields.io/badge/Protocol-I2C-yellow)](https://es.wikipedia.org/wiki/I2C)
[![UART](https://img.shields.io/badge/Protocol-UART-yellow)](https://es.wikipedia.org/wiki/Universal_Asynchronous_Receiver-Transmitter)
[![PCA9685](https://img.shields.io/badge/Drive-PCA9685-green)](https://github.com/Darcko123/STM32/tree/main/Librerias/PWM_Module)
[![Version](https://img.shields.io/badge/FK_Version-2.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Projects/4DOF_RobotArm/CinematicaDirectaKUKA4DOF)


## Tabla de Contenidos
- [4DOF KUKA Robot Arm](#4dof-kuka-robot-arm)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Introducción Teórica](#introducción-teórica)
    - [Dimensiones del Robot KUKA LBR IISY 4DOF a Escala](#dimensiones-del-robot-kuka-lbr-iisy-4dof-a-escala)
    - [Límites de Articulaciones](#límites-de-articulaciones)
    - [Parámetros Denavit-Hartenberg](#parámetros-denavit-hartenberg)
      - [Fórmula general de transformación DH:](#fórmula-general-de-transformación-dh)
      - [Matriz de transformación homogénea:](#matriz-de-transformación-homogénea)
      - [Tabla DH del robot KUKA 4DOF:](#tabla-dh-del-robot-kuka-4dof)
  - [Características Generales del Sistema](#características-generales-del-sistema)
  - [Arquitectura del Sistema](#arquitectura-del-sistema)
    - [Diagrama de Bloques General](#diagrama-de-bloques-general)
  - [Requisitos del Sistema](#requisitos-del-sistema)
    - [Hardware Requerido](#hardware-requerido)
    - [Software Requerido](#software-requerido)
  - [Configuración del Hardaware](#configuración-del-hardaware)
    - [Configuración de I2C](#configuración-de-i2c)
    - [Configuración de UART](#configuración-de-uart)
    - [Conexiones de Hardware](#conexiones-de-hardware)
  - [Cinemática Directa](#cinemática-directa)
    - [Descripción](#descripción-1)
    - [Características](#características)
    - [Teoría Matemática](#teoría-matemática)
      - [Cinemática Directa (Forward Kinematics):](#cinemática-directa-forward-kinematics)
      - [Posición del end-effector:](#posición-del-end-effector)
    - [Implementación](#implementación)
      - [Funciones Principales](#funciones-principales)
      - [Algoritmo de Transformación DH](#algoritmo-de-transformación-dh)
      - [Cálculo de Cinemática Directa](#cálculo-de-cinemática-directa)
    - [Control de Movimiento](#control-de-movimiento)
      - [Movimiento Suave con Interpolación](#movimiento-suave-con-interpolación)
      - [Actualización de Servos](#actualización-de-servos)
    - [Secuencia de Prueba Automática](#secuencia-de-prueba-automática)
    - [Interfaz de Usuario (v2.0)](#interfaz-de-usuario-v20)
      - [Menú Principal](#menú-principal)
      - [Funciones del Menú](#funciones-del-menú)
      - [Ingreso Manual de Ángulos](#ingreso-manual-de-ángulos)
      - [Interfaz con Colores ANSI](#interfaz-con-colores-ansi)
      - [Funciones de la Interfaz](#funciones-de-la-interfaz)
    - [Validación y Debugging](#validación-y-debugging)
    - [Changelog](#changelog)
      - [Versión 2.0.0 (Diciembre 2025)](#versión-200-diciembre-2025)
    - [Versión FK 1.0.0 (Diciembre 2025)](#versión-fk-100-diciembre-2025)
  - [Licencia](#licencia)

## Descripción
Este repositorio contiene una implementación completa del sistema de control para un brazo robótico KUKA LBR IISY de 4 grados de libertad (4DOF) a escala, utilizando el microcontrolador STM32F411. El proyecto está dividido en tres módulos principales que abarcan desde el control básico hasta trayectorias avanzadas.

El sistema implementa control de servomotores mediante I2C (PCA9685), interfaz de usuario por UART con colores ANSI, y cálculos de cinemática directa e inversa en tiempo real.

![Modelo 3D del Robot](/Projects/4DOF_RobotArm/Images/Modelo3DRobot.png)

> [!NOTE]
> El modelo 3D se adquirió desde [Brazo robótico 4DOF (basado en Cobot KUKA)](https://cults3d.com/es/modelo-3d/artilugios/4dof-robotic-arm-cobot-kuka-based)

## Introducción Teórica

### Dimensiones del Robot KUKA LBR IISY 4DOF a Escala

El brazo robótico está compuesto por cuatro articulaciones de revolución con las siguientes dimensiones físicas:

| Parámetro | Símbolo | Valor (mm) | Descripción |
|-----------|---------|------------|-------------|
| Altura Base | H0 | 45.0 | Altura desde el suelo hasta joint 1 |
| Offset Vertical | L1 | 30.0 | Joint 1 a plano de joint 2 |
| Brazo Proximal | L2 | 87.0 | Hombro (Shoulder) |
| Brazo Distal | L3 | 70.0 | Codo (Elbow) |
| Antebrazo | L4 | 44.0 | Muñeca (Wrist) |
| Pinza | LGRIP | 45.0 | Longitud de agarre |

**Alcance total del robot:** 
- Máximo radial: L2 + L3 + L4 = 201 mm
- Altura máxima: H0 + L1 + L2 + L3 + L4 = 276 mm

### Límites de Articulaciones

Cada articulación tiene límites de seguridad para prevenir colisiones y daños mecánicos:

| Articulación | Mínimo (°) | Máximo (°) | Rango Total |
|--------------|-----------|-----------|-------------|
| Base (Joint 1) | 0.0 | 180.0 | 180.0 |
| Hombro (Joint 2) | 0.0 | 180.0 | 180.0 |
| Codo (Joint 3) | 0.0 | 180.0 | 180.0 |
| Muñeca (Joint 4) | 0.0 | 180.0 | 180.0 |

> [!WARNING]
> Los límites de articulaciones deben respetarse estrictamente. El sistema implementa validación automática pero movimientos forzados pueden dañar los servomotores.

### Parámetros Denavit-Hartenberg

La convención Denavit-Hartenberg (DH) es un método sistemático para describir la geometría de robots articulados mediante cuatro parámetros por eslabón:

- **$a_i$**: Longitud del eslabón (distancia entre ejes Z)
- **$\alpha_i$**: Torsión del eslabón (ángulo entre ejes Z)
- **$d_i$**: Offset del eslabón (distancia entre ejes X)
- **$\theta_i$**: Ángulo de la articulación (variable para robots)

#### Fórmula general de transformación DH:

$$
T_\frac{i-1}{i} = rot_z(\theta_i) \times trans_z(d_i) \times trans_x(a_i) \times rot_x(\alpha_i)
$$

#### Matriz de transformación homogénea:

$$
T_\frac{i-1}{i} = \begin{bmatrix}
\cos(\theta_i) & -\sin(\theta_i)\cos(\alpha_i) & \sin(\theta_i)\sin(\alpha_i) & a_i\cos(\theta_i) \\
\sin(\theta_i) & \cos(\theta_i)\cos(\alpha_i) & -\cos(\theta_i)\sin(\alpha_i) & a_i\sin(\theta_i) \\
0 & \sin(\alpha_i) & \cos(\alpha_i) & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

La matriz $T_\frac{i-1}{i}$ es función de una sola variable de junta (tipicamente) y 3 de los 4 parametros son constantes para cada eslabón determinado. En el caso de una articulación de *revoluta* $\theta_i$ es la variable, mientras que $d_i$ es la variables para una junta *prismática*|

#### Tabla DH del robot KUKA 4DOF:
>[!NOTE]
> - $d_1 = Base + Eslabón_1 = 45mm + 30mm = 75mm$
> - $\alpha_1 = -90°$ ya que sufre una rotación para un cambio de eje
> - $\alpha_2 = \alpha_3 = \alpha_4 = 0°$ ya que son paralelos

La fórmula para la Transformación D-H es:

$$
T_\frac{i-1}{i} = rot_z(\theta_i) \times trans_z(d_i) \times trans_x(a_i) \times rot_x(\alpha_i)
$$

La tabla con los valores parametrizables es:

| $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|-----|------------|-------|-------|------------|
|  **1**  | $\theta_1$* | H0 + L1 | 0 | $-\pi/2$ |
|  **2**  | - $\theta_2$* | 0 | L2 | 0° |
|  **3** | $\theta_3$* | 0  | L3 | 0° |
|  **4**  | $\theta_4$* | 0  | L4 | 0° |

Para lo cuál, la tabla de este modelo quedaría como:

| $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|-----|------------|-------|-------|------------|
|  **1**  | $\theta_1$* | 75 mm | 0 mm | -90° |
|  **2**  | - $\theta_2$* | 0 mm | 87 mm | 0° |
|  **3** | $\theta_3$* | 0 mm | 70 mm | 0° |
|  **4**  | $\theta_4$* | 0 mm | 44 mm | 0° |

>[!NOTE]
> `*` indica variable de junta. El signo negativo en $\theta_2$ compensa la orientación del servo.

## Características Generales del Sistema

Características compartidas por todos los proyectos:

**Hardware:**
- Control de 4 servomotores mediante PCA9685 (I2C)
- Microcontrolador STM32F411 (100 MHz, FPU)
- Comunicación UART a 115200 baudios
- Movimiento suave con interpolación lineal
- Actualización de servos a 50 Hz

**Software:**
- Interfaz de usuario con colores ANSI
- Validación de límites de articulaciones
- Sistema de estados para control de flujo
- Callback UART no bloqueante
- Timeout de inactividad (3 minutos)
- Manejo de errores robusto

**Funcionalidades:**
- Posición HOME segura
- Visualización de posición actual
- Debugging detallado por UART
- Secuencias de prueba automáticas

---

## Arquitectura del Sistema

### Diagrama de Bloques General

```text
┌──────────────────────────────────────────┐
│       Computadora (PC/Laptop)            │
│   Terminal Serial con soporte ANSI       │
│  (PuTTY, Tera Term, Terminal Linux)      │
└──────────────────┬───────────────────────┘
                   │ UART 115200 bps
┌──────────────────▼───────────────────────┐
│      Microcontrolador STM32F411          │
│  ┌────────────────────────────────────┐  │
│  │   Aplicación de Control            │  │
│  │  ┌──────────────────────────────┐  │  │
│  │  │ Módulo de Interfaz Usuario   │  │  │
│  │  │  • Menú UART                 │  │  │
│  │  │  • Validación de entrada     │  │  │
│  │  │  • Colores ANSI              │  │  │
│  │  └──────────────┬───────────────┘  │  │
│  │                 │                  │  │
│  │  ┌──────────────▼───────────────┐  │  │
│  │  │ Módulo de Cinemática         │  │  │
│  │  │  • Cinemática Directa (FK)   │  │  │
│  │  │  • Cinemática Inversa (IK)   │  │  │
│  │  │  • Planificador Trayectorias │  │  │
│  │  └──────────────┬───────────────┘  │  │
│  │                 │                  │  │
│  │  ┌──────────────▼───────────────┐  │  │
│  │  │ Módulo de Control Servos     │  │  │
│  │  │  • Interpolación de ángulos  │  │  │
│  │  │  • Validación de límites     │  │  │
│  │  │  • Actualización periódica   │  │  │
│  │  └──────────────┬───────────────┘  │  │
│  └─────────────────┼──────────────────┘  │
│                    │                     │
│    ┌───────────────▼────────────────┐    │
│    │       Periféricos HAL          │    │
│    │  • I2C1 → PCA9685              │    │
│    │  • UART1 → Interface           │    │
│    │  • SysTick → Timing            │    │
│    └───────────────┬────────────────┘    │
└────────────────────┼─────────────────────┘
                     │ I2C 100 kHz
┌────────────────────▼─────────────────────┐
│       Controlador PCA9685                │
│   (16 canales PWM, 12-bit, 50 Hz)        │
└────────────────────┬─────────────────────┘
                     │ PWM Signal
         ┌───────────┼───────────┐───────────┐
         │           │           │           │
┌────────▼───┐  ┌────▼───┐  ┌────▼───┐  ┌────▼───┐
│ Servo Base │  │ Servo  │  │ Servo  │  │ Servo  │
│  (Joint 1) │  │Hombro  │  │ Codo   │  │Muñeca  │
│            │  │(Joint2)│  │(Joint3)│  │(Joint4)│
└────────────┘  └────────┘  └────────┘  └────────┘
```

## Requisitos del Sistema

### Hardware Requerido

| Componente | Especificación | Cantidad |
|------------|---------------|----------|
| Microcontrolador | STM32F411 (Nucleo/BlackPill) | 1 |
| Módulo PWM | PCA9685 (16 canales I2C) | 1 |
| Servomotores | MG90S/SG90 (180°) | 4 |
| Convertidor USB | TTL to USB (CH340/CP2102) | 1 |
| Fuente alimentación | 5V 2-3A para servos | 1 |
| Regulador | 3.3V para STM32 | 1 |
| Cables | Dupont M-M, M-F | Varios |
| Programador | ST-Link V2/V3 | 1 |

### Software Requerido
| Software | Versión | Propósito |
|----------|---------|-----------|
| STM32CubeIDE | ≥ 1.12.0 | IDE de desarrollo |
| STM32CubeMX | ≥ 6.8.0 | Configuración de periféricos |
| Terminal Serial | Cualquiera con ANSI | Monitorización (PuTTY, Tera Term) |
| MATLAB | ≥ R2021a | Simulación (opcional) |
| Toolbox **[Simple Robotics Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/80137-simple-robotics-toolbox)** | N/A | Cinemática y visualización |

---

## Configuración del Hardaware
- Microcontrolador STM32F411
### Configuración de I2C
| Parámetro | Valor |
|-----------|-------|
| Modo | I2C Master |
| Velocidad | Standard Mode (100 kHz) |
| Direccionamiento | 7-bit |
| Dirección PCA9685 | 0x40 (por defecto) |

### Configuración de UART
| Parámetro | Valor |
|-----------|-------|
| Baud Rate | 115200 bps |
| Word Length | 8 bits |
| Stop Bits | 1 |
| Parity | None |
| Flow Control | None |
| Modo | Asíncrono |

### Conexiones de Hardware
| Componente | Pin STM32F411 | Pin PCA9685 / Servomotor |
|------------|---------------|--------------------------|
| SDA (I2C)  | PB7           | SDA                      |
| SCL (I2C)  | PB6           | SCL                      |
| TX (UART)  | PA2           | RX (PC)                  |
| RX (UART)  | PA3           | TX (PC)                  |
| PWM Servo 1 | Canal 0       | Servo Base               |
| PWM Servo 2 | Canal 1       | Servo Hombro             |
| PWM Servo 3 | Canal 2       | Servo Codo               |
| PWM Servo 4 | Canal 3       | Servo Muñeca             |
| 5V Power   | VCC           | VCC (Servos)             |
| GND        | GND           | GND                      |

> [!IMPORTANT]
> El módulo PCA9685 debe alimentarse con fuente externa de 5V. NO conectar directamente al STM32.

---

## Cinemática Directa

### Descripción
El proyecto de **Cinemática Directa** implementa el control básico del brazo robótico calculando la posición del end-effector a partir de los ángulos de las articulaciones. Este es el proyecto fundamental que establece la base para los desarrollos posteriores.

**Ubicación:** `CinematicaDirectaKUKA4DOF/`

### Características

**Versión 1.0:**
- Cálculo de cinemática directa mediante parámetros DH
- Control de 4 servomotores con PCA9685
- Movimiento suave con interpolación lineal
- Secuencia de prueba automática
- Validación de límites
- Debugging por UART

**Versión 2.0 (Actual):**
- **Interfaz de usuario interactiva** con menú UART
- **Sistema de colores ANSI** para mejor UX
- **Ingreso manual de ángulos** con validación
- **Visualización de posición actual**
- **Control de secuencia** (iniciar/detener)
- **Timeout de inactividad** (3 minutos)
- **Callback UART no bloqueante**

### Teoría Matemática

#### Cinemática Directa (Forward Kinematics): 

La cinemática directa calcula la posición y orientación del end-effector dados los ángulos de las articulaciones:

$$
T_4 = T_1 \times T_2 \times T_3 \times T_4
$$

Donde cada matriz $T_\frac{i-1}{i}$ se calcula usando los parámetros DH del eslabón $i$.

#### Posición del end-effector:

Dada la matriz de transformación final $T_4$:

$$
T_{04} = \begin{bmatrix}
r_{11} & r_{12} & r_{13} & p_x \\
r_{21} & r_{22} & r_{23} & p_y \\
r_{31} & r_{32} & r_{33} & p_z \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

La posición es: $\mathbf{p} = [p_x, p_y, p_z]^T$

La orientación (pitch) es: $\beta = \arctan2(-r_{31}, \sqrt{r_{11}^2 + r_{21}^2})$

### Implementación

#### Funciones Principales
```c
/**
 * @brief Construye matriz de transformación DH estándar.
 * 
 * @details
 * La convención Denavit-Hartenberg (DH) es un método sistemático para
 * describir la geometría de robots articulados. Cada transformación
 * se define por 4 parámetros:
 * 1. a: distancia a lo largo de X desde Z(i-1) a Zi
 * 2. α: ángulo alrededor de X desde Z(i-1) a Zi
 * 3. d: distancia a lo largo de Z desde X(i-1) a Xi
 * 4. θ: ángulo alrededor de Z desde X(i-1) a Xi
 * 
 * La matriz de transformación homogénea resultante es:
 * 
 *     [ cosθ   -sinθ·cosα   sinθ·sinα   a·cosθ ]
 *     [ sinθ    cosθ·cosα  -cosθ·sinα   a·sinθ ]
 * A = [  0        sinα         cosα        d   ]
 *     [  0          0           0          1   ]
 * 
 * @param a Longitud del eslabón (distancia entre ejes Z)
 * @param alpha Torsión del eslabón (ángulo entre ejes Z)
 * @param d Offset del eslabón (distancia entre ejes X)
 * @param theta Ángulo de la articulación (variable para robots)
 * @param[out] T Matriz de salida 4x4
 * 
 * @note Las funciones trigonométricas usan float (cosf, sinf) para eficiencia.
 */
void DH_Matrix(float a, float alpha, float d, float theta, float T[4][4]);

/**
 * @brief Calcula cinemática directa del robot.
 * 
 * @details
 * La cinemática directa calcula la posición y orientación del
 * end-effector (pinza) dados los ángulos de las articulaciones.
 * 
 * FLUJO DE CÁLCULO:
 * 1. Calcular matrices DH individuales para cada articulación
 * 2. Multiplicar secuencialmente: T04 = T01 × T12 × T23 × T34
 * 
 * DONDE:
 * - T01: Transformación de base (0) a hombro (1)
 * - T12: Transformación de hombro (1) a codo (2)
 * - T23: Transformación de codo (2) a muñeca (3)
 * - T34: Transformación de muñeca (3) a pinza (4)
 * 
 * @param q1 Ángulo de la base en radianes (rotación alrededor de Z0)
 * @param q2 Ángulo del hombro en radianes
 * @param q3 Ángulo del codo en radianes
 * @param q4 Ángulo de la muñeca en radianes
 * @param[out] T Matriz de transformación 4x4 (base a end-effector)
 * 
 * @note Los ángulos deben estar en radianes, no en grados.
 */
void ForwardKinematics(float q1, float q2, float q3, float q4, float T[4][4]);

/**
 * @brief Extrae posición y orientación (pitch) del end-effector.
 * 
 * @details
 * De una matriz de transformación homogénea 4x4, extrae:
 * 1. Posición (x, y, z): elementos [0][3], [1][3], [2][3]
 * 2. Orientación (pitch): del vector de aproximación (columna Z)
 * 
 * CÁLCULO DEL PITCH:
 * El pitch (β) se calcula a partir de la matriz de rotación R:
 *   β = atan2(-r20, sqrt(r00² + r10²))
 * Donde r_ij son elementos de R (primeras 3 filas/columnas).
 * 
 * @param T Matriz de transformación 4x4
 * @param[out] x Coordenada X del end-effector en mm
 * @param[out] y Coordenada Y del end-effector en mm
 * @param[out] z Coordenada Z del end-effector en mm
 * @param[out] pitch Ángulo de inclinación (pitch) en radianes
 * 
 * @note El pitch es útil para mantener la pinza nivelada.
 */
void GetEndEffectorPose(float T[4][4], float* x, float* y, float* z, float* pitch);

```

#### Algoritmo de Transformación DH
La función DH_Matrix() implementa la transformación DH estándar:

```C
/**
 * @brief Construye matriz de transformación DH estándar.
 *
 * @param a Longitud del eslabón (distancia entre ejes Z)
 * @param alpha Torsión del eslabón (ángulo entre ejes Z)
 * @param d Offset del eslabón (distancia entre ejes X)
 * @param theta Ángulo de la articulación (variable para robots)
 * @param[out] T Matriz de salida 4x4
 */ 
void DH_Matrix(float a, float alpha, float d, float theta, float T[4][4])
{
	// Pre-cálculo de funciones trigonométricas (optimización)
	float ct = cosf(theta);
	float st = sinf(theta);
	float ca = cosf(alpha);
	float sa = sinf(alpha);

	// Matriz DH estándar
	T[0][0] = ct;       T[0][1] = -st * ca;  T[0][2] =  st * sa;  T[0][3] = a * ct;
	T[1][0] = st;       T[1][1] =  ct * ca;  T[1][2] = -ct * sa;  T[1][3] = a * st;
	T[2][0] = 0.0f;     T[2][1] =  sa;       T[2][2] =  ca;       T[2][3] = d;
	T[3][0] = 0.0f;     T[3][1] = 0.0f;      T[3][2] = 0.0f;      T[3][3] = 1.0f;
}
```

**Características de implementación:**
- Uso de `float` para optimización de FPU
- Funciones trigonométricas con sufijo `f` (cosf, sinf)
- Pre-cálculo de valores para eficiencia

#### Cálculo de Cinemática Directa
La cinemática directa se calcula multiplicando secuencialmente las matrices DH:
```C
/**
 * @brief Calcula cinemática directa del robot.
 * 
 * @param q1 Ángulo de la base en radianes (rotación alrededor de Z0)
 * @param q2 Ángulo del hombro en radianes
 * @param q3 Ángulo del codo en radianes
 * @param q4 Ángulo de la muñeca en radianes
 * @param[out] T Matriz de transformación 4x4 (base a end-effector)
 * 
 * @note Los ángulos deben estar en radianes, no en grados.
 */
void ForwardKinematics(float q1, float q2, float q3, float q4, float T[4][4])
{
	// Matrices de transformación individuales
	float T01[4][4], T12[4][4], T23[4][4], T34[4][4];

	// Matrices intermedias (optimización de multiplicaciones)
	float T02[4][4], T03[4][4];

	// Calcular matrices DH individuales para cada articulación
	DH_Matrix(a1, alpha1, d1, q1, T01);  // Base a hombro
	DH_Matrix(a2, alpha2, d2,-q2, T12);  // Hombro a codo
	DH_Matrix(a3, alpha3, d3, q3, T23);  // Codo a muñeca
	DH_Matrix(a4, alpha4, d4, q4, T34);  // Muñeca a pinza

	// Multiplicar matrices secuencialmente (cadena cinemática)
	MatMul4x4(T01, T12, T02);   // T02 = base a codo
	MatMul4x4(T02, T23, T03);   // T03 = base a muñeca
	MatMul4x4(T03, T34, T);     // T = base a pinza (end-effector)
}
```

**Flujo de cálculo:**
1. Calcular 4 matrices DH individuales
2. Multiplicar secuencialmente de izquierda a derecha
3. Resultado final en matriz T (4x4)

### Control de Movimiento

#### Movimiento Suave con Interpolación
El sistema implementa interpolación lineal para movimientos suaves:
```C
/**
 * @brief Mueve el robot a una configuración específica (no bloqueante).
 * 
 * @param q1 Ángulo objetivo de la base en grados
 * @param q2 Ángulo objetivo del hombro en grados
 * @param q3 Ángulo objetivo del codo en grados
 * @param q4 Ángulo objetivo de la muñeca en grados
 * @param duration_ms Duración total del movimiento en milisegundos
 * 
 * @note Para movimiento síncrono, usar MoveRobotBlocking()
 */
void MoveRobot(float q1, float q2, float q3, float q4, uint32_t duration_ms)
{
	// 1. LIMITAR ÁNGULOS A RANGOS PERMITIDOS
	// Previene daños mecánicos y configuraciones peligrosas
	q1 = ConstrainAngle(q1, Q1_MIN, Q1_MAX);
	q2 = ConstrainAngle(q2, Q2_MIN, Q2_MAX);
	q3 = ConstrainAngle(q3, Q3_MIN, Q3_MAX);
	q4 = ConstrainAngle(q4, Q4_MIN, Q4_MAX);

	// 2. APLICAR OFFSETS DE CALIBRACIÓN
	// Compensa desalineaciones mecánicas de los servos
	float servo1_angle = q1 + SERVO1_OFFSET;
	float servo2_angle = q2 + SERVO2_OFFSET;
	float servo3_angle = q3 + SERVO3_OFFSET;
	float servo4_angle = q4 + SERVO4_OFFSET;

	// 3. CONFIGURAR MOVIMIENTOS SUAVES
	// Interpolación lineal entre posición actual y objetivo
	PCA9685_SetSmoothAngle(&SERVO_BASE, servo1_angle, duration_ms);
	PCA9685_SetSmoothAngle(&SERVO_SHOULDER, servo2_angle, duration_ms);
	PCA9685_SetSmoothAngle(&SERVO_ELBOW, servo3_angle, duration_ms);
	PCA9685_SetSmoothAngle(&SERVO_WRIST, servo4_angle, duration_ms);

	// 4. ACTUALIZAR ÁNGULOS ACTUALES
	// Estos se actualizarán gradualmente durante el movimiento
	current_q1 = q1;
	current_q2 = q2;
	current_q3 = q3;
	current_q4 = q4;
}
```

**Interpolación lineal:**
$$
ángulo(t) = ánguloInicial + (ánguloFinal - ánguloInicial) \times (\frac{t}{duracion})
$$

Donde `t` es el tiempo transcurrido desde el inicio del movimiento.

#### Actualización de Servos

```c
/**
 * @brief Actualiza interpolación de todos los servos
 * @note Debe llamarse periódicamente en el loop principal
 */
void UpdateServos(void)
{
    PCA9685_UpdateSmoothServo(&SERVO_BASE);
    PCA9685_UpdateSmoothServo(&SERVO_SHOULDER);
    PCA9685_UpdateSmoothServo(&SERVO_ELBOW);
    PCA9685_UpdateSmoothServo(&SERVO_WRIST);
}
```

**Función de actualización interna del PCA9685:**
```c
/**
 * @brief Actualiza el movimiento suave del servomotor (debe llamarse periódicamente)
 * @param servo Puntero a la estructura Servo_Smooth_t
 * @return true si el servomotor alcanzó el ángulo objetivo, false si aún está en movimiento
 */
bool PCA9685_UpdateSmoothServo(Servo_Smooth_t* servo)
{
    if (!servo->isMoving) {
        return true; // El movimiento ya ha terminado
    }

    uint32_t currentTime = HAL_GetTick();

    // Verificar si ha pasado el intervalo de actualización
    if ((currentTime - servo->lastUpdateTime) >= servo->updateInterval) {

        // Calcular el nuevo ángulo
        float newAngle = servo->currentAngle + servo->stepSize;

        // Verificar si hemos alcanzado o superado el objetivo
        if ((servo->stepSize > 0 && newAngle >= servo->targetAngle) ||
            (servo->stepSize < 0 && newAngle <= servo->targetAngle)) {
            newAngle = servo->targetAngle;
            servo->isMoving = false;
        }

        // Actualizar el ángulo actual
        servo->currentAngle = newAngle;

        // Enviar el nuevo ángulo al servomotor
        PCA9685_SetServoAngle(servo->channel, newAngle);

        // Actualizar el tiempo de la última actualización
        servo->lastUpdateTime = currentTime;

        return !servo->isMoving; // Devuelve true si el movimiento ha terminado
    }

    return false; // Aún no es tiempo de actualizar
}
```

### Secuencia de Prueba Automática
El sistema incluye una secuencia de prueba preprogramada que demuestra todas las capacidades:

Cada movimiento incluye cálculo y envío de la posición del end-effector por UART.

```c
void TestSequence(void)
{
    // Máquina de estados para secuencia automática
    switch(secuenciaStep) {
        case 0:  // HOME (0°, 0°, 0°, 0°)
        case 2:  // Rotar base (135°, 0°, 0°, 0°)
        case 4:  // Levantar hombro (135°, 45°, 0°, 0°)
        case 6:  // Flexionar codo (135°, 45°, 60°, 0°)
        case 8:  // Ajustar muñeca (135°, 45°, 60°, -30°)
        case 10: // Volver HOME (0°, 0°, 0°, 0°)
            // Ejecutar movimiento y calcular FK
            MoveRobot(q1, q2, q3, q4, 2000);
            ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2), 
                            DEG2RAD(q3), DEG2RAD(q4), T);
            GetEndEffectorPose(T, &x, &y, &z, &pitch);
            // Mostrar información por UART
            break;
            
        case 1: case 3: case 5: case 7: case 9:
            // Estados de espera entre movimientos
            break;
            
        case 11: // Secuencia completada
            DetenerSecuencia();
            break;
    }
}
```

**Movimientos de la secuencia:**

| Paso | Descripción | Q1 | Q2 | Q3 | Q4 | Tiempo |
|------|-------------|----|----|----|----|--------|
| 1 | Posición HOME | 0° | 0° | 0° | 0° | 2s |
| 2 | Rotar base | 135° | 0° | 0° | 0° | 2s |
| 3 | Levantar hombro | 135° | 45° | 0° | 0° | 2s |
| 4 | Flexionar codo | 135° | 45° | 60° | 0° | 2s |
| 5 | Ajustar muñeca | 135° | 45° | 60° | -30° | 2s |
| 6 | Retorno HOME | 0° | 0° | 0° | 0° | 2s |

### Interfaz de Usuario (v2.0)

#### Menú Principal

La versión 2.0 incorpora un menú interactivo completo con las siguientes opciones:

```text
==================================================
     KUKA LBR IISY 4 DOF ROBOT ARM
     Control System v2.1
     Cinematica Directa DH
==================================================

========== MENU PRINCIPAL ==========

  1. Ejecutar secuencia automatica
  2. Ingresar angulos manualmente
  3. Posicion HOME
  4. Ver posicion actual
  5. DETENER secuencia (si está activa)

====================================

[ROBOT] Seleccione una opcion (1-5):
```

#### Funciones del Menú

**Opción 1: Ejecutar secuencia automática**
- Inicia TestSequence() en loop no bloqueante
- Muestra cada movimiento con datos de FK
- Puede detenerse con opción 5
- Reinicia automáticamente al finalizar

**Opción 2: Ingresar ángulos manualmente**
```text
--- INGRESO MANUAL DE ANGULOS ---

Rango valido: 0 a 180 grados

[1/4] Angulo Base (Q1): 90_
[2/4] Angulo Hombro (Q2): 45_
[3/4] Angulo Codo (Q3): 60_
[4/4] Angulo Muñeca (Q4): 0_

--- RESUMEN DE ANGULOS ---

Base:   90.0 grados
Hombro: 45.0 grados
Codo:   60.0 grados
Muñeca: 0.0 grados

ℹ Moviendo robot...
✓ Movimiento completado!
```

**Opción 3: Posición HOME**
- Ejecuta HomePosition() de forma bloqueante
- Lleva todas las articulaciones a 0°
- Confirmación visual del movimiento

**Opción 4: Ver posición actual**
```text
--- POSICION ACTUAL DEL ROBOT ---

Angulos de las articulaciones:
  Base (Q1):   90.0 grados
  Hombro (Q2): 45.0 grados
  Codo (Q3):   60.0 grados
  Muñeca (Q4): 0.0 grados

Posicion End-Effector:
  X = 123.45 mm
  Y = 67.89 mm
  Z = 145.67 mm
  Pitch = 15.00 grados

Presione ENTER para volver al menu...
```

**Opción 5: Detener secuencia**
- Solo disponible si secuencia está activa
- Detiene TestSequence() en paso actual
- Mantiene última posición alcanzada

#### Ingreso Manual de Ángulos

El sistema implementa un proceso guiado para el ingreso de ángulos:

```c
/**
 * @brief Solicita ángulos al usuario y mueve el robot
 * @details Proceso secuencial con validación en cada paso
 */
void SolicitarAngulos(void)
{
    // Proceso guiado:
    // 1. Solicita Base (Q1) con validación
    // 2. Solicita Hombro (Q2) con validación
    // 3. Solicita Codo (Q3) con validación
    // 4. Solicita Muñeca (Q4) con validación
    // 5. Muestra resumen
    // 6. Ejecuta movimiento
    // 7. Muestra posición alcanzada
}
```

**Validación en tiempo real:**
```c
float ConvertirStringAFloat(char* str)
{
    float resultado = 0.0f;
    float decimal = 0.0f;
    int signo = 1;
    bool despuesDelPunto = false;
    int decimales = 0;
    
    // Eliminar espacios
    while(*str == ' ') str++;
    
    // Detectar signo
    if(*str == '-') { signo = -1; str++; }
    else if(*str == '+') str++;
    
    // Parsear dígitos
    while(*str != '\0' && *str != '\r' && *str != '\n') {
        if(*str >= '0' && *str <= '9') {
            if(!despuesDelPunto)
                resultado = resultado * 10.0f + (*str - '0');
            else {
                decimal = decimal * 10.0f + (*str - '0');
                decimales++;
            }
        } else if(*str == '.' || *str == ',') {
            despuesDelPunto = true;
        }
        str++;
    }
    
    // Agregar parte decimal
    while(decimales > 0) {
        decimal /= 10.0f;
        decimales--;
    }
    
    return (resultado + decimal) * signo;
}
```

**Características:**
- Validación en tiempo real contra límites definidos
- Mensajes de error claros y específicos
- Indicador de progreso [1/4], [2/4], [3/4], [4/4]
- Resumen antes de ejecutar el movimiento
- Confirmación visual de finalización

#### Interfaz con Colores ANSI

El sistema utiliza códigos ANSI para una interfaz visual mejorada. Los colores se definen en el archivo `ANSII_Codes.h`:

```c
// Colores de texto
#define COLOR_RESET         "\033[0m"
#define COLOR_BOLD          "\033[1m"
#define COLOR_DIM           "\033[2m"
#define COLOR_UNDERLINE     "\033[4m"

// Colores básicos
#define COLOR_BLACK         "\033[30m"
#define COLOR_RED           "\033[31m"
#define COLOR_GREEN         "\033[32m"
#define COLOR_YELLOW        "\033[33m"
#define COLOR_BLUE          "\033[34m"
#define COLOR_MAGENTA       "\033[35m"
#define COLOR_CYAN          "\033[36m"
#define COLOR_WHITE         "\033[37m"

// Colores brillantes
#define COLOR_BRIGHT_BLACK  "\033[90m"
#define COLOR_BRIGHT_RED    "\033[91m"
#define COLOR_BRIGHT_GREEN  "\033[92m"
#define COLOR_BRIGHT_YELLOW "\033[93m"
#define COLOR_BRIGHT_BLUE   "\033[94m"
#define COLOR_BRIGHT_MAGENTA "\033[95m"
#define COLOR_BRIGHT_CYAN   "\033[96m"
#define COLOR_BRIGHT_WHITE  "\033[97m"

// Control de cursor y pantalla
#define CLEAR_SCREEN        "\033[2J"
#define CLEAR_LINE          "\033[2K"
#define CLEAR_TO_END        "\033[0J"
#define CLEAR_TO_LINE_END   "\033[0K"
#define CURSOR_HOME         "\033[H"
#define CURSOR_SAVE         "\033[s"
#define CURSOR_RESTORE      "\033[u"
#define CURSOR_HIDE         "\033[?25l"
#define CURSOR_SHOW         "\033[?25h"
```

**Esquema de colores del proyecto:**
```c
#define HEADER_COLOR        COLOR_BRIGHT_CYAN
#define MENU_COLOR          COLOR_BRIGHT_GREEN
#define OPTION_COLOR        COLOR_CYAN
#define WARNING_COLOR       COLOR_BRIGHT_YELLOW
#define ERROR_COLOR         COLOR_BRIGHT_RED
#define SUCCESS_COLOR       COLOR_BRIGHT_GREEN
#define INFO_COLOR          COLOR_BRIGHT_BLUE
#define INPUT_COLOR         COLOR_BRIGHT_WHITE
#define VALUE_COLOR         COLOR_YELLOW
#define SEPARATOR_COLOR     COLOR_BRIGHT_MAGENTA
#define ROBOT_COLOR         COLOR_BRIGHT_CYAN
```

#### Funciones de la Interfaz

```c
/**
 * @brief Limpia la pantalla del terminal
 */
void LimpiarPantalla(void);

/**
 * @brief Muestra el encabezado del sistema
 */
void MostrarEncabezado(void);

/**
 * @brief Muestra una línea separadora con color
 */
void MostrarSeparador(const char* color);

/**
 * @brief Muestra un mensaje de éxito
 */
void MostrarExito(const char* mensaje);

/**
 * @brief Muestra un mensaje de error
 */
void MostrarError(const char* mensaje);

/**
 * @brief Muestra un mensaje de advertencia
 */
void MostrarAdvertencia(const char* mensaje);

/**
 * @brief Muestra un mensaje informativo
 */
void MostrarInfo(const char* mensaje);

/**
 * @brief Callback de recepción UART no bloqueante
 * @details Maneja caracteres individuales, backspace y enter
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief Convierte una cadena a float
 * @details Soporta signos, decimales y punto/coma
 */
float ConvertirStringAFloat(char* str);
```

### Validación y Debugging

**Validación de límites:**
```c
float ConstrainAngle(float angle, float min, float max)
{
    if (angle < min) {
        sprintf(buffer, "%s⚠ Angulo %.1f° limitado a %.1f°%s\r\n",
                WARNING_COLOR, angle, min, COLOR_RESET);
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
        return min;
    }
    if (angle > max) {
        sprintf(buffer, "%s⚠ Angulo %.1f° limitado a %.1f°%s\r\n",
                WARNING_COLOR, angle, max, COLOR_RESET);
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
        return max;
    }
    return angle;
}
```

**Debugging detallado:**
```c
void MostrarPosicionActual(void)
{
    float T[4][4];
    float x, y, z, pitch;
    
    // Calcular FK
    ForwardKinematics(DEG2RAD(current_q1), DEG2RAD(current_q2),
                     DEG2RAD(current_q3), DEG2RAD(current_q4), T);
    GetEndEffectorPose(T, &x, &y, &z, &pitch);
    
    // Mostrar con formato colorizado
    LimpiarPantalla();
    MostrarEncabezado();
    
    sprintf(buffer, "%s%s--- POSICION ACTUAL DEL ROBOT ---%s\r\n\r\n",
            HEADER_COLOR, COLOR_BOLD, COLOR_RESET);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
    
    // Ángulos de articulaciones
    sprintf(buffer, "%sAngulos de las articulaciones:%s\r\n",
            INFO_COLOR, COLOR_RESET);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
    
    sprintf(buffer, "  %sBase (Q1):%s   %s%.1f%s grados\r\n",
            COLOR_CYAN, COLOR_RESET, VALUE_COLOR, current_q1, COLOR_RESET);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
    
    // ... similar para Q2, Q3, Q4 ...
    
    // Posición end-effector
    sprintf(buffer, "\r\n%sPosicion End-Effector:%s\r\n",
            INFO_COLOR, COLOR_RESET);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
    
    sprintf(buffer, "  %sX =%s %s%.2f%s mm\r\n",
            COLOR_GREEN, COLOR_RESET, VALUE_COLOR, x, COLOR_RESET);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
    
    // ... similar para Y, Z, Pitch ...
}
```

**Callback UART no bloqueante:**
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(RxByte == '\r' || RxByte == '\n') {
        // ENTER presionado
        if(indx > 0) {
            RxData[indx] = '\0';
            newMessageReady = true;
            sprintf(buffer, "\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 10);
        } else if (estadoMenu == MENU_PRINCIPAL) {
            // ENTER sin datos -> refrescar menú
            MostrarMenuPrincipal();
        }
    } else if(RxByte == 127 || RxByte == 8) {
        // Backspace o DEL
        if(indx > 0) {
            indx--;
            RxData[indx] = 0;
            HAL_UART_Transmit(&huart1, (uint8_t*)"\b \b", 3, 10);
        }
    } else if(indx < BUFFER_SIZE - 1) {
        // Carácter normal
        RxData[indx++] = RxByte;
        HAL_UART_Transmit(&huart1, &RxByte, 1, 10); // Echo
    }
    
    // Reiniciar recepción
    HAL_UART_Receive_IT(&huart1, &RxByte, 1);
}
```

### Changelog

#### Versión 2.0.0 (Diciembre 2025)

**Nuevas Características:**
- **Interfaz de usuario interactiva por UART** con menú principal
- **Sistema de colores ANSI** para mejorar la experiencia visual
- **Ingreso manual de ángulos** con validación en tiempo real
- **Visualización de posición actual** con formato mejorado
- **Control de secuencia automática** con opción de inicio/detención
- **Archivo ANSII_Codes.h** para gestión centralizada de colores
- **Callback UART no bloqueante** con manejo de buffer seguro
- **Sistema de timeout** por inactividad (3 minutos)
- **Estados de menú** (MENU_PRINCIPAL, MENU_ANGULOS, EJECUTANDO_SECUENCIA)

**Mejoras:**
- **Función ConvertirStringAFloat()** para parsing robusto de entrada
- **Funciones de interfaz** (MostrarExito, MostrarError, MostrarAdvertencia, MostrarInfo)
- **LimpiarPantalla()** para control de terminal mejorado
- **MostrarEncabezado()** con diseño profesional
- **Validación mejorada** de límites con mensajes específicos
- **Echo de caracteres** en UART para retroalimentación inmediata
- **Soporte de backspace** para corrección de entrada

**Cambios Técnicos:**
- Incremento de tamaño de buffer a 250 caracteres
- Implementación de máquina de estados para control de flujo
- Variables volátiles para manejo de interrupciones UART
- Buffer circular para recepción UART
- Gestión de última interacción para timeout

**Estructura de Código:**
```
Nuevas funciones v2.0:
├── MostrarMenuPrincipal()
├── ProcesarOpcionMenu()
├── SolicitarAngulos()
├── MostrarPosicionActual()
├── IniciarSecuenciaAutomatica()
├── DetenerSecuencia()
├── ConvertirStringAFloat()
├── LimpiarPantalla()
├── MostrarEncabezado()
├── MostrarSeparador()
├── MostrarExito()
├── MostrarError()
├── MostrarAdvertencia()
├── MostrarInfo()
└── HAL_UART_RxCpltCallback() [mejorado]

Nuevas variables v2.0:
├── EstadoMenu_t (typedef enum)
├── estadoMenu
├── secuenciaStep
├── secuenciaLastMove
├── secuenciaActiva
├── ultimaInteraccion
├── RxData[BUFFER_SIZE]
├── RxByte
├── indx
└── newMessageReady
```

**Archivos Modificados:**
- `main.c` - Versión 2.0 con sistema de menú completo
- `README.md` - Actualizado con nueva documentación

**Archivos Nuevos:**
- `ANSII_Codes.h` - Definiciones de códigos ANSI para colores y control de terminal

### Versión FK 1.0.0 (Diciembre 2025)

**Características Iniciales:**
- Implementación de cinemática directa con parámetros DH
- Control de 4 servomotores mediante PCA9685
- Movimiento suave con interpolación lineal
- Secuencia de prueba automática predefinida
- Validación de límites de articulaciones
- Comunicación UART para debugging
- Cálculo de posición del end-effector
- Funciones de conversión radianes/grados

**Funciones Principales v1.0:**
```
├── DH_Matrix()
├── MatMul4x4()
├── ForwardKinematics()
├── GetEndEffectorPose()
├── DEG2RAD()
├── RAD2DEG()
├── ConstrainAngle()
├── MoveRobot()
├── MoveRobotBlocking()
├── UpdateServos()
├── HomePosition()
└── TestSequence()
```

**Arquitectura v1.0:**
- Módulo de cinemática directa
- Módulo de control de servos (PCA9685)
- Comunicación UART básica
- Sistema de interpolación de movimientos

## Licencia

Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.