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
  - [Arquitectura del Sistema](#arquitectura-del-sistema)
  - [Implementación y Rendimiento](#implementación-y-rendimiento)
  - [Requisitos del Sistema](#requisitos-del-sistema)
    - [Hardware Requerido](#hardware-requerido)
    - [Software Requerido](#software-requerido)
  - [Descripción Funcional del Sistema](#descripción-funcional-del-sistema)
    - [Diagrama de flujo del procesamiento general](#diagrama-de-flujo-del-procesamiento-general)
  - [Configuración del Hardaware](#configuración-del-hardaware)
    - [Configuración de I2C](#configuración-de-i2c)
    - [Configuración de UART](#configuración-de-uart)
    - [Conexiones de Hardware](#conexiones-de-hardware)
  - [Cinemática Directa](#cinemática-directa)
    - [Funciones Principales](#funciones-principales)
    - [Algoritmo de Transformación DH](#algoritmo-de-transformación-dh)
    - [Cálculo de Matriz de Transformación](#cálculo-de-matriz-de-transformación)
    - [Movimiento Suave con Interpolación](#movimiento-suave-con-interpolación)
    - [Secuencia de Prueba Automática](#secuencia-de-prueba-automática)
    - [Módulo PCA9685 para Control de Servos](#módulo-pca9685-para-control-de-servos)
    - [Validación y Debugging](#validación-y-debugging)
      - [Validación de Límites](#validación-de-límites)
      - [Debuggging por UART](#debuggging-por-uart)
  - [Próximos Desarrollos](#próximos-desarrollos)
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

## Arquitectura del Sistema
El sistema está compuesto por los siguientes módulos principales:
- **Módulo de Comunicación UART**: Gestiona la recepción de comandos de posición desde una interfaz externa.
- **Módulo de Cálculo de Cinemática**: Realiza los cálculos de cinemática directa e inversa.
- **Módulo de Control de Servomotores**: Utiliza el PCA9685 para controlar los servomotores del brazo robótico.
- **Módulo Principal**: Coordina la interacción entre los módulos y gestiona el flujo de datos.  
- **Módulo de Configuración**: Define los parámetros iniciales y las constantes del sistema.
- **Módulo de Utilidades**: Proporciona funciones auxiliares para el manejo de datos y conversiones necesarias.

## Implementación y Rendimiento

## Requisitos del Sistema

### Hardware Requerido
- **STM32CubeIDE** (última versión recomendada)
- **ST-Link V2/V3** (programador/depurador)
- **Placa STM32F411**
- **Módulo PCA9685**
- **Fuente de alimentación 3.3V/5V**
- **TTL a USB Converter** (para comunicación UART con PC)
- **Servomotores** (4 unidades)
- **Cables de conexión** (I2C, UART, alimentación)

### Software Requerido
- **STM32CubeMX** para configuración de periféricos
- **Terminal serial** para monitorización (opcional)
- **MATLAB**
- Toolbox **[Simple Robotics Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/80137-simple-robotics-toolbox)**

## Descripción Funcional del Sistema
El flujo general del sistema es el siguiente:
1. **Inicialización**: Al encender el sistema, se inicializan todos los módulos, incluyendo la configuración del PCA9685 y la UART.
2. **Recepción de Comandos**: El sistema espera la recepción de comandos a través de la interfaz UART. Los comandos especifican la posición deseada del efector final en coordenadas espaciales (X, Y, Z).
3. **Cálculo de Cinemática Inversa**: Una vez recibido un comando, el módulo de cinemática inversa calcula los ángulos necesarios para cada articulación del brazo robótico para alcanzar la posición deseada.
4. **Control de Servomotores**: Los ángulos calculados se envían al módulo de control de servomotores, que ajusta las posiciones de los servos utilizando el PCA9685.
5. **Cinemática Directa y Verificación**: El módulo de cinemática directa puede ser utilizado para verificar la posición actual del efector final después del movimiento.

### Diagrama de flujo del procesamiento general
```text
        ┌─────────────────────────────────────────┐
        │          Computadora (PC)               │
        │         Monitorización UART             │
        └─────────────────────┬───────────────────┘
                              │
        ┌─────────────────────▼───────────────────┐
        │         Microcontrolador STM32          │
        │  ┌───────────────────────────────────┐  │
        │  │      Lógica de Control            │  │
        │  │  • Cinemática Directa             │  │
        │  │  • Interpolación                  │  │
        │  │  • Validación de Límites          │  │
        │  └───────────────┬───────────────────┘  │
        │                  │                      │
        │    ┌─────────────▼─────────────┐        │
        │    │         Periféricos       │        │
        │    │  • I2C → PCA9685          │        │
        │    │  • UART → Debugging       │        │
        │    └─────────────┬─────────────┘        │
        └──────────────────┼──────────────────────┘
                           │
        ┌──────────────────▼──────────────────┐
        │          Controlador PCA9685        │
        │         (16 canales PWM)            │
        └──────────────────┬──────────────────┘
                           │
               ┌───────────▼───────────┐
               │     Servomotores      │
               │  • Base (Joint 1)     │
               │  • Hombro (Joint 2)   │
               │  • Codo (Joint 3)     │
               │  • Muñeca (Joint 4)   │
               └───────────────────────┘
```

## Configuración del Hardaware
- Microcontrolador STM32F411
### Configuración de I2C
|I2C Mode   | Configuration |
|-----------|----------------|
| I2C Speed Mode | Standard mode |
| I2C Clock Speed (Hz) | 100 kHz |

### Configuración de UART
|UART Mode   | Configuration |
|-----------|----------------|
| Baud Rate | 115200 bps |

### Conexiones de Hardware
| Componente | Pin STM32F411 | Pin PCA9685 / Servomotor |
|------------|---------------|--------------------------|
| SDA (I2C)  | PB7           | SDA                      |
| SCL (I2C)  | PB6           | SCL                      |
| TX (UART)  | PA2           | RX (PC)                  |
| RX (UART)  | PA3           | TX (PC)                  |

## Cinemática Directa

### Funciones Principales
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

### Algoritmo de Transformación DH
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

### Cálculo de Matriz de Transformación
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

### Movimiento Suave con Interpolación
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

### Secuencia de Prueba Automática
El sistema incluye una secuencia de prueba preprogramada que demuestra todas las capacidades:

1. Movimiento 1: Posición home (0°, 0°, 0°, 0°)
2. Movimiento 2: Rotación de base a 135°
3. Movimiento 3: Hombro a 45°
4. Movimiento 4: Codo a 60°
5. Movimiento 5: Muñeca a -30°
6. Movimiento 6: Retorno a posición home

Cada movimiento incluye cálculo y envío de la posición del end-effector por UART.

### Módulo PCA9685 para Control de Servos
El control de servos se realiza mediante el módulo PCA9685 con las siguientes características:
- Frecuencia PWM: 50 Hz (20 ms período)
- Resolución: 12 bits (4096 pasos)
- Canales: 16 canales independientes
- Comunicación: I2C a 100 kHz
- Interpolación: Suavizado de movimientos integrado

```C
// Inicialización del PCA9685
PCA9685_Init(&hi2c1, 50);  // 50 Hz para servos estándar

// Configuración de servos con interpolación
PCA9685_InitSmoothServo(&SERVO_BASE, 0, 0, 20);      // Canal 0, actualización cada 20ms
PCA9685_InitSmoothServo(&SERVO_SHOULDER, 1, 0, 20);  // Canal 1
PCA9685_InitSmoothServo(&SERVO_ELBOW, 2, 0, 20);     // Canal 2
PCA9685_InitSmoothServo(&SERVO_WRIST, 3, 0, 20);     // Canal 3
```

### Validación y Debugging
El sistema incluye funciones de validación y debugging para monitorear el estado del robot:

#### Validación de Límites
```C
float ConstrainAngle(float angle, float min, float max)
{
    if (angle < min) return min;
    if (angle > max) return max;
    return angle;
}
```

#### Debuggging por UART
El sistema envía información detallada por UART:
- Ángulos actuales de articulaciones
- Posición XYZ del end-effector
- Orientación (pitch)
- Estado de movimiento
- Errores o advertencias

**Ejemplo de salida:**
```text
Movimiento 1: Home
Angulos: [0.0, 0.0, 0.0, 0.0] grados
Posicion End-Effector:
  X = 201.00 mm
  Y = 0.00 mm
  Z = 75.00 mm
  Pitch = 0.00 grados
```

## Próximos Desarrollos
1. Cinemática Inversa (Próxima carpeta):
   1. Cálculo de ángulos para posición deseada
   2. Solución de múltiples configuraciones
   3. Selección de solución óptima

1. Trayectorias por Interpolación:
   1. Movimientos en línea recta (Cartesianos)
   2. Interpolación de joint-space
   3. Planificación de trayectorias suaves
   4. Avoidance de obstáculos básico

1. Mejoras Adicionales:
   1. Control mediante interfaz gráfica
   2. Algoritmos de seguimiento de trayectoria
   3. Integración de sensor de fuerza
   4. Modo enseñanza por demostración

## Licencia

Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.