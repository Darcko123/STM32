/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Control de brazo robótico escala KUKA LBR IISY 4DOF con cinemática directa
 * @author			: Daniel Ruiz
 * @date			: December, 2025
 * @version		: 1.0
 ******************************************************************************
 * @description
 * Este programa implementa el control completo de un brazo robótico de 4 grados
 * de libertad (4DOF) mediante servomotores controlados por un módulo PCA9685.
 *
 * CARACTERÍSTICAS PRINCIPALES:
 * - Cinemática directa usando parámetros Denavit-Hartenberg (DH)
 * - Movimiento suave no bloqueante con interpolación de ángulos
 * - Control de servos mediante I2C (PCA9685)
 * - Comunicación UART para monitoreo y debugging
 * - Secuencia de prueba automática
 * - Validación de límites de articulaciones
 *
 * ARQUITECTURA DEL SISTEMA:
 * 1. STM32 (Nucleo/Discovery) como controlador principal
 * 2. PCA9685 como controlador de servos (16 canales, I2C)
 * 3. 4x Servomotores MG90S/SG90 para las articulaciones
 * 4. UART para interfaz con PC (115200 baudios)
 *
 * ARTICULACIONES DEL ROBOT:
 * 1. Joint 1: Base (Yaw) - Rotación horizontal
 * 2. Joint 2: Hombro (Shoulder) - Movimiento vertical
 * 3. Joint 3: Codo (Elbow) - Movimiento vertical
 * 4. Joint 4: Muñeca (Wrist) - Inclinación pitch
 *
 * PARÁMETROS FÍSICOS (en mm):
 * - H0:    45.0  Altura de la base
 * - L1:    30.0  Offset vertical del joint1 al plano del joint2
 * - L2:    87.0  Brazo proximal (hombro)
 * - L3:    70.0  Brazo distal (codo)
 * - L4:    44.0  Antebrazo (muñeca)
 * - LGRIP: 45.0  Longitud de la pinza (no implementada en este código)
 *
 * @attention
 * Copyright (c) 2025 STMicroelectronics.
 * Todos los derechos reservados.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "PCA9685_PWMModule.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ============================================================================
// PARÁMETROS FÍSICOS DEL ROBOT (en milímetros)
// ============================================================================

#define H0      45.0f   /**< Altura de la base desde el suelo */
#define L1      30.0f   /**< Offset vertical del joint1 al plano del joint2 */
#define L2      87.0f   /**< Longitud del brazo proximal (hombro) */
#define L3      70.0f   /**< Longitud del brazo distal (codo) */
#define L4      44.0f   /**< Longitud del antebrazo (muñeca) */
#define LGRIP   45.0f   /**< Longitud de la pinza (no implementada) */

// ============================================================================
// LÍMITES DE ARTICULACIONES (en grados)
// ============================================================================

#define Q1_MIN   0.0f   /**< Límite mínimo de la base (joint 1) */
#define Q1_MAX 180.0f   /**< Límite máximo de la base (joint 1) */
#define Q2_MIN   0.0f   /**< Límite mínimo del hombro (joint 2) */
#define Q2_MAX 180.0f   /**< Límite máximo del hombro (joint 2) */
#define Q3_MIN   0.0f   /**< Límite mínimo del codo (joint 3) */
#define Q3_MAX 180.0f   /**< Límite máximo del codo (joint 3) */
#define Q4_MIN   0.0f   /**< Límite mínimo de la muñeca (joint 4) */
#define Q4_MAX 180.0f   /**< Límite máximo de la muñeca (joint 4) */

// ============================================================================
// OFFSETS DE CALIBRACIÓN DE SERVOS
// ============================================================================

#define SERVO1_OFFSET  0.0f   /**< Offset de calibración para servo 1 (base) */
#define SERVO2_OFFSET  0.0f   /**< Offset de calibración para servo 2 (hombro) */
#define SERVO3_OFFSET  0.0f   /**< Offset de calibración para servo 3 (codo) */
#define SERVO4_OFFSET  0.0f   /**< Offset de calibración para servo 4 (muñeca) */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// ============================================================================
// MAPEO DE SERVOMOTORES
// ============================================================================

Servo_Smooth_t SERVO_BASE;      /**< Servo canal 0 - Joint 1 (Yaw, rotación base) */
Servo_Smooth_t SERVO_SHOULDER;  /**< Servo canal 1 - Joint 2 (Shoulder, hombro) */
Servo_Smooth_t SERVO_ELBOW;     /**< Servo canal 2 - Joint 3 (Elbow, codo) */
Servo_Smooth_t SERVO_WRIST;     /**< Servo canal 3 - Joint 4 (Wrist, muñeca) */

// ============================================================================
// PARÁMETROS DENAVIT-HARTENBERG (DH)
// ============================================================================
// NOTA: theta es variable (q1, q2, q3, q4)

// Joint 1: Base rotation (Yaw) - Rotación alrededor de Z0
static const float a1 = 0.0f;
static const float alpha1 = -M_PI/2.0f;  // -90° para pasar de Z a X
static const float d1 = H0 + L1;         // Offset vertical total

// Joint 2: Shoulder - Rotación en el nuevo Z (plano vertical)
static const float a2 = L2;                  /**< Longitud del eslabón 2 (hombro) */
static const float alpha2 = 0.0f;            /**< Torsión del eslabón 2 */
static const float d2 = 0.0f;                /**< Offset del eslabón 2 */

// Joint 3: Elbow - Rotación en Z (plano vertical)
static const float a3 = L3;                  /**< Longitud del eslabón 3 (codo) */
static const float alpha3 = 0.0f;            /**< Torsión del eslabón 3 */
static const float d3 = 0.0f;                /**< Offset del eslabón 3 */

// Joint 4: Wrist Pitch - Rotación en Z (plano vertical)
static const float a4 = L4;                  /**< Longitud del eslabón 4 (muñeca) */
static const float alpha4 = 0.0f;            /**< Torsión del eslabón 4 */
static const float d4 = 0.0f;                /**< Offset del eslabón 4 */

// Ángulos theta de prueba para cada Joint
float q1, q2, q3, q4;

// ============================================================================
// CONFIGURACIÓN ACTUAL DEL ROBOT
// ============================================================================

float current_q1 = 0.0f; 	/**< Ángulo actual de la base (joint1) en grados */
float current_q2 = 0.0f;  	/**< Ángulo actual del hombro (joint2) en grados */
float current_q3 = 0.0f;  	/**< Ángulo actual del codo (joint3) en grados */
float current_q4 = 0.0f;  	/**< Ángulo actual de la muñeca (joint4) en grados */

/**
 * @brief Buffer para mensajes de debugging por UART
 * @details Capacidad: 200 caracteres para mensajes formateados
 */
char buffer[200];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/**
 * @brief Calcula la matriz de transformación DH estándar.
 * @param a Longitud del eslabón (a)
 * @param alpha Torsión del eslabón (alpha)
 * @param d Offset del eslabón (d)
 * @param theta Ángulo de la articulación (theta)
 * @param[out] T Matriz de transformación 4x4 de salida
 * @retval None
 */
void DH_Matrix(float a, float alpha, float d, float theta, float T[4][4]);

/**
 * @brief Multiplica dos matrices 4x4.
 * @param A Primera matriz 4x4
 * @param B Segunda matriz 4x4
 * @param[out] R Matriz resultado 4x4
 * @retval None
 */
void MatMul4x4(float A[4][4], float B[4][4], float R[4][4]);

/**
 * @brief Calcula la cinemática directa del robot.
 * @param q1 Ángulo de la base en radianes
 * @param q2 Ángulo del hombro en radianes
 * @param q3 Ángulo del codo en radianes
 * @param q4 Ángulo de la muñeca en radianes
 * @param[out] T Matriz de transformación 4x4 (base a end-effector)
 * @retval None
 */
void ForwardKinematics(float q1, float q2, float q3, float q4, float T[4][4]);

/**
 * @brief Extrae posición y orientación del end-effector.
 * @param T Matriz de transformación 4x4
 * @param[out] x Coordenada X en mm
 * @param[out] y Coordenada Y en mm
 * @param[out] z Coordenada Z en mm
 * @param[out] pitch Ángulo de inclinación en radianes
 * @retval None
 */
void GetEndEffectorPose(float T[4][4], float* x, float* y, float* z, float* pitch);

/**
 * @brief Convierte grados a radianes.
 * @param angle Ángulo en grados
 * @return Ángulo en radianes
 */
float DEG2RAD(float angle);

/**
 * @brief Convierte radianes a grados.
 * @param angle Ángulo en radianes
 * @return Ángulo en grados
 */
float RAD2DEG(float angle);

/**
 * @brief Limita un ángulo a un rango específico.
 * @param angle Ángulo a limitar
 * @param min Límite mínimo
 * @param max Límite máximo
 * @return Ángulo limitado
 */
float ConstrainAngle(float angle, float min, float max);

/**
 * @brief Mueve el robot a una configuración específica (no bloqueante).
 * @param q1 Ángulo objetivo de la base en grados
 * @param q2 Ángulo objetivo del hombro en grados
 * @param q3 Ángulo objetivo del codo en grados
 * @param q4 Ángulo objetivo de la muñeca en grados
 * @param duration_ms Duración del movimiento en milisegundos
 * @retval None
 */
void MoveRobot(float q1, float q2, float q3, float q4, uint32_t duration_ms);

/**
 * @brief Mueve el robot y espera hasta completar el movimiento (bloqueante).
 * @param q1 Ángulo objetivo de la base en grados
 * @param q2 Ángulo objetivo del hombro en grados
 * @param q3 Ángulo objetivo del codo en grados
 * @param q4 Ángulo objetivo de la muñeca en grados
 * @param duration_ms Duración del movimiento en milisegundos
 * @retval None
 */
void MoveRobotBlocking(float q1, float q2, float q3, float q4, uint32_t duration_ms);

/**
 * @brief Actualiza los servos (debe llamarse en el loop principal).
 * @retval None
 */
void UpdateServos(void);

/**
 * @brief Lleva el robot a la posición home.
 * @retval None
 */
void HomePosition(void);

/**
 * @brief Ejecuta una secuencia de prueba del robot.
 * @details Realiza movimientos predefinidos y muestra información por UART.
 * @retval None
 */
void TestSequence(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_Delay(100);	// Esperar inicialización del PCA9685

	// Inicializar PCA9685 a 50Hz (servos estándar)
	PCA9685_Init(&hi2c1, 50);
	HAL_Delay(100);	// Pequeño delay para estabilización

	// Configurar servos con interpolación suave
	PCA9685_InitSmoothServo(&SERVO_BASE, 0, 0, 20);  		// Canal 0, inicio en 0°, actualización cada 20ms
	PCA9685_InitSmoothServo(&SERVO_SHOULDER, 1, 0, 20);  	// Canal 1, inicio en 0°, actualización cada 20ms
	PCA9685_InitSmoothServo(&SERVO_ELBOW, 2, 0, 20);  	// Canal 2, inicio en 0°, actualización cada 20ms
	PCA9685_InitSmoothServo(&SERVO_WRIST, 3, 0, 20);		// Canal 3, inicio en 0°, actualización cada 20ms

	// Mensaje de Inicio
	HAL_UART_Transmit(&huart1, (uint8_t*)"==== KUKA LBR IISY 4 DOF ====\r\n\n",
			strlen("==== KUKA LBR IISY 4 DOF ====\r\n\n"), 1000);

	// Ir a posición home
	HomePosition();
	HAL_Delay(2000);  // Esperar estabilización en posición home

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		UpdateServos(); // Actualiza interpolación de ángulos

		// Ejemplo: ejecutar secuencia de prueba cada 10 segundos
		static uint32_t lastTest = 0;
		if (HAL_GetTick() - lastTest > 10000)
		{
			TestSequence();
			lastTest = HAL_GetTick();
		}

		HAL_Delay(10);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ============================================================================
// IMPLEMENTACIÓN DE CINEMÁTICA DIRECTA
// ============================================================================

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

/**
 * @brief Multiplica dos matrices 4x4.
 * 
 * @details
 * Realiza la multiplicación matricial estándar C = A × B.
 * Para matrices homogéneas 4x4, esto combina transformaciones.
 * 
 * ALGORITMO:
 * Para cada elemento C[i][j]:
 *   C[i][j] = Σ (A[i][k] × B[k][j]) para k = 0..3
 * 
 * @param A Primera matriz 4x4
 * @param B Segunda matriz 4x4
 * @param[out] R Matriz resultado 4x4 (R = A × B)
 * 
 * @note Se usa un buffer temporal para evitar sobreescritura accidental.
 */
void MatMul4x4(float A[4][4], float B[4][4], float R[4][4])
{
	float temp[4][4]; // Buffer temporal para evitar aliasing

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			temp[i][j] = 0.0f;  // Inicializar elemento
			for (int k = 0; k < 4; k++) {
				temp[i][j] += A[i][k] * B[k][j];
			}
		}
	}
	// Copiar resultado
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			R[i][j] = temp[i][j];
		}
	}
}

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
void GetEndEffectorPose(float T[4][4], float* x, float* y, float* z, float* pitch)
{
	// Extraer posición (columna de traslación, fila 3)
	*x = T[0][3];
	*y = T[1][3];
	*z = T[2][3];

	// Extraer pitch del vector de aproximación (columna Z de la matriz de rotación)
	// pitch = atan2(-r20, sqrt(r00^2 + r10^2))
	*pitch = atan2f(-T[2][0], sqrtf(T[0][0]*T[0][0] + T[1][0]*T[1][0]));
}

// ============================================================================
// FUNCIONES DE UTILIDAD
// ============================================================================


/**
 * @brief Convierte grados a radianes.
 * 
 * @param angle Ángulo en grados
 * @return Ángulo en radianes
 * 
 * @note Factor de conversión: π radianes = 180°
 */
float DEG2RAD(float angle)
{
	return angle * (M_PI / 180.0f);
}

/**
 * @brief Convierte radianes a grados.
 * 
 * @param angle Ángulo en radianes
 * @return Ángulo en grados
 */
float RAD2DEG(float angle)
{
	return angle * (180.0f / M_PI);
}

/**
 * @brief Limita un ángulo a un rango específico.
 * 
 * @details
 * Esta función asegura que los ángulos de las articulaciones
 * se mantengan dentro de límites seguros para prevenir:
 * 1. Daño a los servomotores
 * 2. Colisiones mecánicas
 * 3. Configuraciones singulares
 * 
 * @param angle Ángulo a limitar (en grados)
 * @param min Límite mínimo permitido
 * @param max Límite máximo permitido
 * @return Ángulo limitado al rango [min, max]
 */
float ConstrainAngle(float angle, float min, float max)
{
	if (angle < min) return min;
	if (angle > max) return max;
	return angle;
}

// ============================================================================
// CONTROL DE MOVIMIENTO
// ============================================================================

/**
 * @brief Mueve el robot a una configuración específica (no bloqueante).
 * 
 * @details
 * Esta función planifica un movimiento suave hacia la configuración objetivo.
 * Características:
 * - Limita ángulos a rangos seguros
 * - Aplica offsets de calibración
 * - Usa interpolación suave para evitar movimientos bruscos
 * - No bloquea la ejecución (asíncrona)
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

/**
 * @brief Mueve el robot y espera hasta completar el movimiento (bloqueante).
 * 
 * @details
 * Versión síncrona de MoveRobot(). Útil para secuencias donde se necesita
 * garantizar que una posición se alcance antes de continuar.
 * 
 * FLUJO:
 * 1. Inicia movimiento (igual que MoveRobot)
 * 2. Espera en loop hasta que todos los servos terminen
 * 3. Continúa ejecución
 * 
 * @param q1 Ángulo objetivo de la base en grados
 * @param q2 Ángulo objetivo del hombro en grados
 * @param q3 Ángulo objetivo del codo en grados
 * @param q4 Ángulo objetivo de la muñeca en grados
 * @param duration_ms Duración del movimiento en milisegundos
 * 
 * @note Bloquea la ejecución durante duration_ms aproximadamente
 */
void MoveRobotBlocking(float q1, float q2, float q3, float q4, uint32_t duration_ms)
{
	// Iniciar movimiento
	MoveRobot(q1, q2, q3, q4, duration_ms);

	// Esperar la duración + margen
	uint32_t start = HAL_GetTick();
	uint32_t timeout = duration_ms + 500; // 500ms de margen

	while ((HAL_GetTick() - start) < timeout) {
		UpdateServos(); // Actualizar interpolación
		HAL_Delay(20);  // Delay moderado (coincide con update_ms de los servos)
	}
}

/**
 * @brief Actualiza los servos (debe llamarse en el loop principal).
 * 
 * @details
 * Esta función actualiza la posición de todos los servos usando
 * interpolación lineal. Debe llamarse frecuentemente en el loop
 * principal para un movimiento suave.
 * 
 * FUNCIONAMIENTO:
 * Para cada servo con movimiento suave habilitado:
 * 1. Calcula posición intermedia basada en tiempo transcurrido
 * 2. Actualiza PWM del PCA9685
 * 3. Marca como "no moviendo" cuando alcanza posición objetivo
 * 
 * @retval None
 * 
 * @note Llamar en cada iteración del loop principal
 * @note Tiempo entre llamadas afecta suavidad del movimiento
 */
void UpdateServos(void)
{
	PCA9685_UpdateSmoothServo(&SERVO_BASE);
	PCA9685_UpdateSmoothServo(&SERVO_SHOULDER);
	PCA9685_UpdateSmoothServo(&SERVO_ELBOW);
	PCA9685_UpdateSmoothServo(&SERVO_WRIST);
}

// ============================================================================
// POSICIONES Y SECUENCIAS
// ============================================================================

/**
 * @brief Lleva el robot a la posición home.
 * 
 * @details
 * Posición home definida como:
 * - Base: 90° (centrada)
 * - Hombro: 0° (extendido horizontalmente)
 * - Codo: 0° (extendido)
 * - Muñeca: 0° (horizontal)
 * 
 * Esta posición suele ser:
 * 1. Segura (sin colisiones)
 * 2. Conocida (punto de referencia)
 * 3. De baja energía (extendido)
 * 
 * @retval None
 * 
 * @note Usa movimiento bloqueante para garantizar posición
 */
void HomePosition(void)
{
	// Posición home: base centrada, brazo extendido horizontalmente
	MoveRobotBlocking(0.0f, 0.0f, 0.0f, 0.0f, 2000);
}

/**
 * @brief Secuencia de prueba del robot.
 * 
 * @details
 * Ejecuta una secuencia predefinida de movimientos para:
 * 1. Verificar funcionamiento de todas las articulaciones
 * 2. Demostrar capacidades del robot
 * 3. Probar límites de movimiento
 * 4. Mostrar información de debugging por UART
 * 
 * SECUENCIA:
 * 1. Home → 2. Base derecha → 3. Levantar brazo → 
 * 4. Flexionar codo → 5. Ajustar muñeca → 6. Volver a home
 * 
 * @retval None
 * 
 * @note Incluye cálculo y envío de posición por UART
 * @note Se ejecuta automáticamente cada 10 segundos en el ejemplo
 */
void TestSequence(void)
{
	static uint8_t step = 0;
	static uint32_t lastMove = 0;
	uint32_t now = HAL_GetTick();

	switch(step) {
		case 0: // MOVIMIENTO 1: Posición home
			if (now - lastMove > 100)
			{

				q1 = 0.0f;
				q2 = 0.0f;
				q3 = 0.0f;
				q4 = 0.0f;

				MoveRobot(q1, q2, q3, q4, 2000);
				lastMove = now;
				step++;

				// Calcular y mostrar cinemática directa
				float T[4][4];
				float x, y, z, pitch;

				ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2),
						DEG2RAD(q3), DEG2RAD(q4), T);
				GetEndEffectorPose(T, &x, &y, &z, &pitch);

				int len = sprintf(buffer,
						"Movimiento 1: Home\r\n"
						"Angulos: [%.1f, %.1f, %.1f, %.1f] grados\r\n"
						"Posicion End-Effector:\r\n"
						"  X = %.2f mm\r\n"
						"  Y = %.2f mm\r\n"
						"  Z = %.2f mm\r\n"
						"  Pitch = %.2f grados\r\n\r\n",
						current_q1, current_q2, current_q3, current_q4,
						x, y, z, RAD2DEG(pitch));
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 1000);
			}
			break;

		case 1: // Esperar 3 segundos
			if (now - lastMove > 3000)
			{
				lastMove = now;
				step++;
			}
			break;

		case 2: // MOVIMIENTO 2: Rotar base
			if (now - lastMove > 100)
			{
				q1 = 135.0f;
				q2 = 0.0f;
				q3 = 0.0f;
				q4 = 0.0f;

				MoveRobot(q1, q2, q3, q4, 2000);
				lastMove = now;
				step++;

				// Calcular y mostrar cinemática directa
				float T[4][4];
				float x, y, z, pitch;

				ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2),
						DEG2RAD(q3), DEG2RAD(q4), T);
				GetEndEffectorPose(T, &x, &y, &z, &pitch);

				int len = sprintf(buffer,
						"Movimiento 2: Rotar base\r\n"
						"Angulos: [%.1f, %.1f, %.1f, %.1f] grados\r\n"
						"Posicion End-Effector:\r\n"
						"  X = %.2f mm\r\n"
						"  Y = %.2f mm\r\n"
						"  Z = %.2f mm\r\n"
						"  Pitch = %.2f grados\r\n\r\n",
						current_q1, current_q2, current_q3, current_q4,
						x, y, z, RAD2DEG(pitch));
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 1000);
			}
			break;

		case 3: // Esperar 2 segundos
			if (now - lastMove > 2000)
			{
				lastMove = now;
				step++;
			}
			break;

		case 4: // MOVIMIENTO 3: Levantar brazo
			if (now - lastMove > 100)
			{

				q1 = 135.0f;
				q2 = 45.0f;
				q3 = 0.0f;
				q4 = 0.0f;

				MoveRobot(q1, q2, q3, q4, 2000);
				lastMove = now;
				step++;

				// Calcular y mostrar cinemática directa
				float T[4][4];
				float x, y, z, pitch;

				ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2),
						DEG2RAD(q3), DEG2RAD(q4), T);
				GetEndEffectorPose(T, &x, &y, &z, &pitch);

				int len = sprintf(buffer,
						"Movimiento 3: Hombro a 45 grados\r\n"
						"Angulos: [%.1f, %.1f, %.1f, %.1f] grados\r\n"
						"Posicion End-Effector:\r\n"
						"  X = %.2f mm\r\n"
						"  Y = %.2f mm\r\n"
						"  Z = %.2f mm\r\n"
						"  Pitch = %.2f grados\r\n\r\n",
						current_q1, current_q2, current_q3, current_q4,
						x, y, z, RAD2DEG(pitch));
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 1000);
			}
			break;

		case 5: // Esperar 2 segundos
			if (now - lastMove > 2000)
			{
				lastMove = now;
				step++;
			}
			break;

		case 6: // MOVIMIENTO 4: Flexionar codo
			if (now - lastMove > 100)
			{
				q1 = 135.0f;
				q2 = 45.0f;
				q3 = 60.0f;
				q4 = 0.0f;

				MoveRobot(q1, q2, q3, q4, 2000);
				lastMove = now;
				step++;

				// Calcular y mostrar cinemática directa
				float T[4][4];
				float x, y, z, pitch;

				ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2),
						DEG2RAD(q3), DEG2RAD(q4), T);
				GetEndEffectorPose(T, &x, &y, &z, &pitch);

				int len = sprintf(buffer,
						"Movimiento 4: Codo 60 grados\r\n"
						"Angulos: [%.1f, %.1f, %.1f, %.1f] grados\r\n"
						"Posicion End-Effector:\r\n"
						"  X = %.2f mm\r\n"
						"  Y = %.2f mm\r\n"
						"  Z = %.2f mm\r\n"
						"  Pitch = %.2f grados\r\n\r\n",
						current_q1, current_q2, current_q3, current_q4,
						x, y, z, RAD2DEG(pitch));
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 1000);
			}
			break;

		case 7: // Esperar 2 segundos
			if (now - lastMove > 2000)
			{
				lastMove = now;
				step++;
			}
			break;

		case 8: // MOVIMIENTO 5: Ajustar muñeca
			if (now - lastMove > 100)
			{

				q1 = 135.0f;
				q2 = 45.0f;
				q3 = 60.0f;
				q4 = -30.0f;

				MoveRobot(q1, q2, q3, q4, 2000);
				lastMove = now;
				step++;

				// Calcular y mostrar cinemática directa
				float T[4][4];
				float x, y, z, pitch;

				ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2),
						DEG2RAD(q3), DEG2RAD(q4), T);
				GetEndEffectorPose(T, &x, &y, &z, &pitch);

				int len = sprintf(buffer,
						"Movimiento 5: Muneca -30 grados\r\n"
						"Angulos: [%.1f, %.1f, %.1f, %.1f] grados\r\n"
						"Posicion End-Effector:\r\n"
						"  X = %.2f mm\r\n"
						"  Y = %.2f mm\r\n"
						"  Z = %.2f mm\r\n"
						"  Pitch = %.2f grados\r\n\r\n",
						current_q1, current_q2, current_q3, current_q4,
						x, y, z, RAD2DEG(pitch));
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 1000);
			}
			break;

		case 9: // Esperar 1.5 segundos
			if (now - lastMove > 1500)
			{
				lastMove = now;
				step++;
			}
			break;

		case 10: // MOVIMIENTO 6: Volver a home
			if (now - lastMove > 100)
			{
				q1 = 0.0f;
				q2 = 0.0f;
				q3 = 0.0f;
				q4 = 0.0f;

				MoveRobot(q1, q2, q3, q4, 2000);
				lastMove = now;
				step++;

				// Calcular y mostrar cinemática directa
				float T[4][4];
				float x, y, z, pitch;

				ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2),
						DEG2RAD(q3), DEG2RAD(q4), T);
				GetEndEffectorPose(T, &x, &y, &z, &pitch);

				int len = sprintf(buffer,
						"Movimiento 1: Home\r\n"
						"Angulos: [%.1f, %.1f, %.1f, %.1f] grados\r\n"
						"Posicion End-Effector:\r\n"
						"  X = %.2f mm\r\n"
						"  Y = %.2f mm\r\n"
						"  Z = %.2f mm\r\n"
						"  Pitch = %.2f grados\r\n\r\n",
						current_q1, current_q2, current_q3, current_q4,
						x, y, z, RAD2DEG(pitch));
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 1000);
			}
			break;

		case 11: // Esperar y mostrar información final
			if (now - lastMove > 2500)
			{
				// Reiniciar secuencia
				step = 0;
				lastMove = now;
			}
			break;
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
