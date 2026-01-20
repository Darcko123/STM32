/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           	: main.c
 * @brief          	: Control de brazo robótico escala KUKA LBR IISY 4DOF con cinemática directa
 * @author			: Daniel Ruiz
 * @date			: December, 2025
 * @version			: 2.1
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
 * - Interfaz de usuario con colores ANSI y limpieza de pantalla
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
#include "ctype.h"
#include "stdbool.h"
#include "PCA9685_PWMModule.h"
#include "ANSII_Codes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Estado del menú de usuario
typedef enum {
	MENU_PRINCIPAL,
	MENU_ANGULOS,
	EJECUTANDO_SECUENCIA
} EstadoMenu_t;

// Estados de error del sistema
typedef enum {
	ROBOT_OK,
	ROBOT_ERROR_PCA9685_INIT,
	ROBOT_ERROR_SERVO_INIT,
	ROBOT_ERROR_SERVO_MOVE,
	ROBOT_ERROR_I2C_COMM,
	ROBOT_ERROR_UART
} RobotError_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ============================================================================
// DELAY PARA SECUENCIA
// ============================================================================
#define DELAY_TEST_SEQUENCE 5000

// ============================================================================
// PARÁMETROS FÍSICOS DEL ROBOT (en milímetros)
// ============================================================================

#define H0      45.0f	/**< Altura de la base desde el suelo */
#define L1      30.0f	/**< Offset vertical del joint1 al plano del joint2 */
#define L2      87.0f	/**< Longitud del brazo proximal (hombro) */
#define L3      70.0f	/**< Longitud del brazo distal (codo) */
#define L4      44.0f	/**< Longitud del antebrazo (muñeca) */
#define LGRIP   45.0f	/**< Longitud de la pinza (no implementada) */

// ============================================================================
// LÍMITES DE ARTICULACIONES (en grados)
// ============================================================================

#define Q1_MIN   0.0f	/**< Límite mínimo de la base (joint 1) */
#define Q1_MAX 180.0f	/**< Límite máximo de la base (joint 1) */
#define Q2_MIN   0.0f	/**< Límite mínimo del hombro (joint 2) */
#define Q2_MAX 180.0f	/**< Límite máximo del hombro (joint 2) */
#define Q3_MIN   0.0f	/**< Límite mínimo del codo (joint 3) */
#define Q3_MAX 180.0f	/**< Límite máximo del codo (joint 3) */
#define Q4_MIN   0.0f	/**< Límite mínimo de la muñeca (joint 4) */
#define Q4_MAX 180.0f	/**< Límite máximo de la muñeca (joint 4) */

// ============================================================================
// OFFSETS DE CALIBRACIÓN DE SERVOS
// ============================================================================

#define SERVO1_OFFSET  0.0f		/**< Offset de calibración para servo 1 (base) */
#define SERVO2_OFFSET  0.0f		/**< Offset de calibración para servo 2 (hombro) */
#define SERVO3_OFFSET  0.0f		/**< Offset de calibración para servo 3 (codo) */
#define SERVO4_OFFSET  0.0f		/**< Offset de calibración para servo 4 (muñeca) */

// ============================================================================
// CONFIGURACIÓN UART
// ============================================================================

#define BUFFER_SIZE 50		/**< Tamaño del buffer de recepción UART */

// Estilos para la interfaz
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

#define ERROR_COLOR_PCA9685 COLOR_BRIGHT_RED
#define ERROR_COLOR_SERVO   COLOR_BRIGHT_MAGENTA
#define ERROR_COLOR_I2C     COLOR_BRIGHT_YELLOW
#define ERROR_COLOR_SYSTEM  COLOR_BRIGHT_RED

// Límite de reintentos para errores I2C
#define I2C_MAX_RETRIES 3
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

Servo_Smooth_t SERVO_BASE;		/**< Servo canal 0 - Joint 1 (Yaw, rotación base) */
Servo_Smooth_t SERVO_SHOULDER;	/**< Servo canal 1 - Joint 2 (Shoulder, hombro) */
Servo_Smooth_t SERVO_ELBOW;		/**< Servo canal 2 - Joint 3 (Elbow, codo) */
Servo_Smooth_t SERVO_WRIST;		/**< Servo canal 3 - Joint 4 (Wrist, muñeca) */

// ============================================================================
// PARÁMETROS DENAVIT-HARTENBERG (DH)
// ============================================================================
// NOTA: theta es variable (q1, q2, q3, q4)

// Joint 1: Base rotation (Yaw) - Rotación alrededor de Z0
static const float a1 = 0.0f;
static const float alpha1 = -M_PI/2.0f; 	// -90° para pasar de Z a X
static const float d1 = H0 + L1;			// Offset vertical total

// Joint 2: Shoulder - Rotación en el nuevo Z (plano vertical)
static const float a2 = L2;					/**< Longitud del eslabón 2 (hombro) */
static const float alpha2 = 0.0f;			/**< Torsión del eslabón 2 */
static const float d2 = 0.0f;				/**< Offset del eslabón 2 */

// Joint 3: Elbow - Rotación en Z (plano vertical)
static const float a3 = L3;					/**< Longitud del eslabón 3 (codo) */
static const float alpha3 = 0.0f;			/**< Torsión del eslabón 3 */
static const float d3 = 0.0f;				/**< Offset del eslabón 3 */

// Joint 4: Wrist Pitch - Rotación en Z (plano vertical)
static const float a4 = L4;					/**< Longitud del eslabón 4 (muñeca) */
static const float alpha4 = 0.0f;			/**< Torsión del eslabón 4 */
static const float d4 = 0.0f;				/**< Offset del eslabón 4 */

// Ángulos theta de prueba para cada Joint
float q1, q2, q3, q4;

// ============================================================================
// CONFIGURACIÓN ACTUAL DEL ROBOT
// ============================================================================

float current_q1 = 0.0f;	/**< Ángulo actual de la base (joint1) en grados */
float current_q2 = 0.0f;	/**< Ángulo actual del hombro (joint2) en grados */
float current_q3 = 0.0f;	/**< Ángulo actual del codo (joint3) en grados */
float current_q4 = 0.0f;	/**< Ángulo actual de la muñeca (joint4) en grados */

/**
 * @brief Buffer para mensajes de debugging por UART
 * @details Capacidad: 250 caracteres para mensajes formateados
 */
char buffer[250];

// ============================================================================
// RECEPCIÓN DE DATOS POR UART
// ============================================================================
uint8_t RxData[BUFFER_SIZE];	// Buffer de recepción
uint8_t RxByte;					// Buffer para recibir 1 byte
volatile uint8_t indx = 0;		// Manejador del índice de la cadena recibida
volatile bool newMessageReady = false;	//Señal de listo

EstadoMenu_t estadoMenu = MENU_PRINCIPAL;

// ============================================================================
// VARIABLES DE ESTADO Y ERROR
// ============================================================================

RobotError_t robotErrorState = ROBOT_OK;  /**< Estado actual de error del robot */
uint8_t i2cRetryCount = 0;                /**< Contador de reintentos I2C */
bool systemInitialized = false;           /**< Flag de sistema inicializado */

// ===========================================================================
// Variables para la secuencia de prueba automática
// ===========================================================================
uint8_t secuenciaStep = 0;
uint32_t secuenciaLastMove = 0;
bool secuenciaActiva = false;

// ===========================================================================
// Timeout de inactividad
// ===========================================================================
uint32_t ultimaInteraccion = 0;
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
PCA9685_Status_t MoveRobot(float q1, float q2, float q3, float q4, uint32_t duration_ms);

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

/**
 * @brief Inicia la secuencia automática del robot.
 * 
 */
void IniciarSecuenciaAutomatica(void);

/**
 * @brief Detiene la secuencia automática del robot.
 *
 */
void DetenerSecuencia(void);

/**
 * @brief Muestra el menú principal del robot.
 * @details Envía el menú por UART.
 * @retval None
 */
void MostrarMenuPrincipal(void);

/**
 * @brief Procesa la opción seleccionada en el menú.
 * @details Lee la entrada del usuario y ejecuta la acción correspondiente.
 * @retval None
 */
void ProcesarOpcionMenu(void);

/**
 * @brief Solicita al usuario los ángulos de los servos.
 * @retval None
 */
void SolicitarAngulos(void);

/**
 * @brief Muestra la posición actual del robot.
 * @details Envía la posición actual del robot por UART.
 * @retval None
 */
void MostrarPosicionActual(void);

/**
 * @brief Muestra el estado actual del sistema
 *
 */
void MostrarEstadoSistema(void);

/**
 * @brief Convierte una cadena de caracteres a un número flotante.
 * 
 * @param str 
 * @return float 
 */
float ConvertirStringAFloat(char* str);

/**
 * @brief Maneja errores del PCA9685.
 * @param error Código de error del PCA9685.
 * @param operation Operación que falló.
 * @retval None
 */
void HandlePCA9685Error(PCA9685_Status_t error, const char* operation);

/**
 * @brief Muestra un mensaje de error específico.
 * @param errorType Tipo de error.
 * @param operation Operación que falló.
 * @param details Detalles adicionales (opcional).
 * @retval None
 */
void MostrarErrorRobot(RobotError_t errorType, const char* operation, const char* details);

/**
 * @brief Reintenta la inicialización del sistema.
 * @retval true si se reinició exitosamente, false en caso contrario.
 */
bool ReintentarInicializacion(void);

/**
 * @brief Verifica el estado del sistema.
 * @retval true si el sistema está operativo, false si hay errores críticos.
 */
bool VerificarEstadoSistema(void);

/**
 * @brief Realiza una prueba de comunicación I2C.
 * @retval true si la comunicación es exitosa, false en caso contrario.
 */
bool TestI2CComunicacion(void);

/**
 * @brief Inicializa los servos con validación de errores.
 *
 * @return PCA9685_Status_t
 */
PCA9685_Status_t InicializarServosConValidacion(void);
// ===========================================================================
// FUNCIONES DE INTERFAZ DE USUARIO
// ===========================================================================

/**
 * @brief Limpia la pantalla del terminal.
 * @retval None
 */
void LimpiarPantalla(void);

/**
 * @brief Muestra el encabezado del sistema.
 * @retval None
 */
void MostrarEncabezado(void);

/**
 * @brief Muestra una línea separadora con color.
 * @param color Código de color ANSI
 * @retval None
 */
void MostrarSeparador(const char* color);

/**
 * @brief Muestra un mensaje de éxito.
 * @param mensaje Texto del mensaje
 * @retval None
 */
void MostrarExito(const char* mensaje);

/**
 * @brief Muestra un mensaje de error.
 * @param mensaje Texto del mensaje
 * @retval None
 */
void MostrarError(const char* mensaje);

/**
 * @brief Muestra un mensaje de advertencia.
 * @param mensaje Texto del mensaje
 * @retval None
 */
void MostrarAdvertencia(const char* mensaje);

/**
 * @brief Muestra un mensaje informativo.
 * @param mensaje Texto del mensaje
 * @retval None
 */
void MostrarInfo(const char* mensaje);

void MostrarSugerenciaError(RobotError_t errorType);

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
	PCA9685_Status_t pcaStatus;
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

	memset(RxData, 0, 20);
	HAL_UART_Receive_IT(&huart1, &RxByte, 1);
	HAL_Delay(100);	// Esperar inicialización del PCA9685

	// Limpiar pantalla terminal y mostrar encabezado
	LimpiarPantalla();
	MostrarEncabezado();

	MostrarInfo("Inicializando sistema robótico...");


	// Inicializar PCA9685 a 50Hz (servos estándar)
	pcaStatus = PCA9685_Init(&hi2c1, 50);
	if(pcaStatus != PCA9685_OK)
	{
		HandlePCA9685Error(pcaStatus, "Inicialización PCA9685");

		if (!ReintentarInicializacion())
		{
			MostrarErrorRobot(ROBOT_ERROR_PCA9685_INIT, "Inicialización",
					"No se pudo inicializar el controlador PWM");
			while(1) { HAL_Delay(1000); }
		}
	}
	else
	{
		MostrarExito("PCA9685 Inicializado correctamente");
	}
	HAL_Delay(100);	// Pequeño delay para estabilización



	// Inicializar servos con validación
	pcaStatus = InicializarServosConValidacion();
	if (pcaStatus != PCA9685_OK)
	{
		HandlePCA9685Error(pcaStatus, "Inicialización de servos");

		if (!ReintentarInicializacion())
		{
			MostrarErrorRobot(ROBOT_ERROR_SERVO_INIT, "Inicialización servos",
					"No se pudo inicializar los servomotores");
		}
	}
	else
	{
		MostrarExito("Servos inicializados correctamente");
		systemInitialized = true;
	}

	// Ir a posición home con validación
	MostrarInfo("Moviendo a posición HOME...");
	if (systemInitialized)
	{
		HomePosition();
	}

	HAL_Delay(2000);  // Esperar estabilización en posición home

	// Mostrar menú inicial si todo está bien
	if (VerificarEstadoSistema())
	{
		MostrarMenuPrincipal();
	}
	else
	{
		MostrarAdvertencia("Sistema operando en modo limitado debido a errores");
		MostrarMenuPrincipal();
	}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		// Verificar estado del sistema periódicamente
		static uint32_t lastSystemCheck = 0;
		if (HAL_GetTick() - lastSystemCheck > 5000) // Cada 5 segundos
		{
			if (!VerificarEstadoSistema())
			{
				MostrarAdvertencia("Advertencia: Sistema detectó errores previos");
			}
			lastSystemCheck = HAL_GetTick();
		}

		if (systemInitialized)
		{
			UpdateServos();

			if (secuenciaActiva)
			{
				TestSequence();
			}

			// Timeout de inactividad
			if (!secuenciaActiva && estadoMenu != MENU_ANGULOS &&
					(HAL_GetTick() - ultimaInteraccion > 180000))	// Minutos
			{
				MostrarAdvertencia("Timeout de inactividad. Volviendo a HOME...");
				if (systemInitialized)
				{
					HomePosition();
				}
				ultimaInteraccion = HAL_GetTick();
			}
		}

		if (newMessageReady)
		{
			newMessageReady = false;
			ProcesarOpcionMenu();

			// Limpiar buffer
			memset(RxData, 0, BUFFER_SIZE);
			indx = 0;
		}

		HAL_Delay(10);	// Pequeño delay para evitar sobrecarga CPU
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ============================================================================
// FUNCIONES DE MANEJO DE ERRORES
// ============================================================================

/**
 * @brief Maneja errores del PCA9685.
 */
void HandlePCA9685Error(PCA9685_Status_t error, const char* operation)
{
	char errorMsg[100];
	const char* errorType;
	RobotError_t robotError;

	// Validar que realmente sea un error
	if (error == PCA9685_OK)
	{
		// No debería llamarse a esta función con PCA9685_OK
		return;
	}

	// Determinar tipo de error y estado del robot
	switch(error)
	{
	case PCA9685_TIMEOUT:
		errorType = "Timeout I2C";
		robotError = ROBOT_ERROR_I2C_COMM;
		break;

	case PCA9685_ERROR:
	default:  // Manejar cualquier otro error no especificado
		errorType = "Error general";
		robotError = ROBOT_ERROR_PCA9685_INIT;
		break;
	}

	// Actualizar estado global de error
	robotErrorState = robotError;

	// Crear mensaje de error
	sprintf(errorMsg, "%s en %s (Código: %d)", errorType, operation, error);
	MostrarErrorRobot(robotError, operation, errorMsg);

	// Incrementar contador de reintentos solo para errores I2C
	if (robotError == ROBOT_ERROR_I2C_COMM)
	{
		i2cRetryCount++;

		if (i2cRetryCount >= I2C_MAX_RETRIES)
		{
			sprintf(buffer, "%s%sERROR CRÍTICO: Se excedió el límite de reintentos I2C%s\r\n",
					COLOR_BRIGHT_RED, COLOR_BOLD, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			sprintf(buffer, "%sPor favor, revise la conexión I2C y reinicie el sistema.%s\r\n",
					WARNING_COLOR, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			systemInitialized = false;
		}
	}
}

/**
 * @brief Muestra un mensaje de error específico del robot.
 */
void MostrarErrorRobot(RobotError_t errorType, const char* operation, const char* details)
{
	const char* color;
	const char* errorDesc;

	// Determinar color y descripción según el tipo de error
	switch(errorType)
	{
	case ROBOT_ERROR_PCA9685_INIT:
		color = ERROR_COLOR_PCA9685;
		errorDesc = "ERROR PCA9685";
		break;

	case ROBOT_ERROR_SERVO_INIT:
		color = ERROR_COLOR_SERVO;
		errorDesc = "ERROR SERVO";
		break;

	case ROBOT_ERROR_SERVO_MOVE:
		color = ERROR_COLOR_SERVO;
		errorDesc = "ERROR MOVIMIENTO";
		break;

	case ROBOT_ERROR_I2C_COMM:
		color = ERROR_COLOR_I2C;
		errorDesc = "ERROR I2C";
		break;

	case ROBOT_ERROR_UART:
		color = ERROR_COLOR_SYSTEM;
		errorDesc = "ERROR UART";
		break;

	default:
		color = ERROR_COLOR_SYSTEM;
		errorDesc = "ERROR SISTEMA";
		break;
	}

	sprintf(buffer, "%s%s[%s] %s%s\r\n",
			color, COLOR_BOLD, errorDesc, operation, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	if (details != NULL)
	{
		sprintf(buffer, "%sDetalles: %s%s\r\n",
				color, details, COLOR_RESET);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
	}

	// Mostrar sugerencia de solución según el error
	MostrarSugerenciaError(errorType);
}

/**
 * @brief Muestra sugerencias para resolver errores.
 */
void MostrarSugerenciaError(RobotError_t errorType)
{
	sprintf(buffer, "%sSugerencia: ", INFO_COLOR);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	switch(errorType)
	{
	case ROBOT_ERROR_I2C_COMM:
	case ROBOT_ERROR_PCA9685_INIT:
		sprintf(buffer, "Verifique conexiones I2C, alimentación y direcciones.%s\r\n", COLOR_RESET);
		break;

	case ROBOT_ERROR_SERVO_INIT:
	case ROBOT_ERROR_SERVO_MOVE:
		sprintf(buffer, "Revise servomotores, voltaje y límites mecánicos.%s\r\n", COLOR_RESET);
		break;

	default:
		sprintf(buffer, "Reinicie el sistema o verifique configuraciones.%s\r\n", COLOR_RESET);
		break;
	}
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

/**
 * @brief Reintenta la inicialización del sistema.
 */
bool ReintentarInicializacion(void)
{
    PCA9685_Status_t status;
    
    // Verificar si ya hemos excedido los reintentos
    if (i2cRetryCount >= I2C_MAX_RETRIES)
    {
        MostrarError("Límite máximo de reintentos alcanzado");
        return false;
    }
    
    // Verificar si el sistema ya está inicializado
    if (systemInitialized)
    {
        MostrarInfo("Sistema ya está inicializado");
        return true;
    }
    
    // Mostrar mensaje de reintento
    sprintf(buffer, "%sReintentando inicialización (%d/%d)...%s\r\n",
            WARNING_COLOR, i2cRetryCount + 1, I2C_MAX_RETRIES, COLOR_RESET);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
    
    // Pequeño delay para estabilización
    HAL_Delay(1000);
    
    // Reintentar inicialización PCA9685
    status = PCA9685_Init(&hi2c1, 50);
    if (status == PCA9685_OK)
    {
        // Reinicializar servos
        status = InicializarServosConValidacion();
        if (status == PCA9685_OK)
        {
            MostrarExito("Reinicialización exitosa!");
            i2cRetryCount = 0;  // Reiniciar contador
            robotErrorState = ROBOT_OK;
            systemInitialized = true;
            return true;
        }
        else
        {
            HandlePCA9685Error(status, "Reinicialización de servos");
            return false;
        }
    }
    else
    {
        HandlePCA9685Error(status, "Reintento de inicialización PCA9685");
        return false;
    }
}

/**
 * @brief Verifica el estado del sistema.
 */
bool VerificarEstadoSistema(void)
{
	if (robotErrorState != ROBOT_OK && i2cRetryCount >= I2C_MAX_RETRIES)
	{
		return false;
	}
	return systemInitialized;
}

/**
 * @brief Inicializa servos con validación de errores.
 */
PCA9685_Status_t InicializarServosConValidacion(void)
{
	PCA9685_Status_t status;

	// Configurar servos con manejo de errores individual
	status = PCA9685_InitSmoothServo(&SERVO_BASE, 0, 0, 20);
	if (status != PCA9685_OK)
	{
		HandlePCA9685Error(status, "Inicializar servo BASE");
		return status;
	}

	status = PCA9685_InitSmoothServo(&SERVO_SHOULDER, 1, 0, 20);
	if (status != PCA9685_OK)
	{
		HandlePCA9685Error(status, "Inicializar servo HOMBRO");
		return status;
	}

	status = PCA9685_InitSmoothServo(&SERVO_ELBOW, 2, 0, 20);
	if (status != PCA9685_OK)
	{
		HandlePCA9685Error(status, "Inicializar servo CODO");
		return status;
	}

	status = PCA9685_InitSmoothServo(&SERVO_WRIST, 3, 0, 20);
	if (status != PCA9685_OK)
	{
		HandlePCA9685Error(status, "Inicializar servo MUÑECA");
		return status;
	}

	return PCA9685_OK;
}

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
PCA9685_Status_t MoveRobot(float q1, float q2, float q3, float q4, uint32_t duration_ms)
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

	PCA9685_Status_t status;

	// 3. CONFIGURAR MOVIMIENTOS SUAVES
	// Interpolación lineal entre posición actual y objetivo
	status = PCA9685_SetSmoothAngle(&SERVO_BASE, servo1_angle, duration_ms);
	if (status != PCA9685_OK)
	{
		HandlePCA9685Error(status, "Mover servo BASE");
		robotErrorState = ROBOT_ERROR_SERVO_MOVE;
		return status;
	}

	status = PCA9685_SetSmoothAngle(&SERVO_SHOULDER, servo2_angle, duration_ms);
	if (status != PCA9685_OK)
	{
		HandlePCA9685Error(status, "Mover servo HOMBRO");
		robotErrorState = ROBOT_ERROR_SERVO_MOVE;
		return status;
	}

	status = PCA9685_SetSmoothAngle(&SERVO_ELBOW, servo3_angle, duration_ms);
	if (status != PCA9685_OK)
	{
		HandlePCA9685Error(status, "Mover servo CODO");
		robotErrorState = ROBOT_ERROR_SERVO_MOVE;
		return status;
	}

	status = PCA9685_SetSmoothAngle(&SERVO_WRIST, servo4_angle, duration_ms);
	if (status != PCA9685_OK)
	{
		HandlePCA9685Error(status, "Mover servo MUÑECA");
		robotErrorState = ROBOT_ERROR_SERVO_MOVE;
		return status;
	}

	// 4. ACTUALIZAR ÁNGULOS ACTUALES
	// Estos se actualizarán gradualmente durante el movimiento
	current_q1 = q1;
	current_q2 = q2;
	current_q3 = q3;
	current_q4 = q4;

	return PCA9685_OK;
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
	if (!systemInitialized) return;

	bool allStopped = true;

	allStopped &= PCA9685_UpdateSmoothServo(&SERVO_BASE);
	allStopped &= PCA9685_UpdateSmoothServo(&SERVO_SHOULDER);
	allStopped &= PCA9685_UpdateSmoothServo(&SERVO_ELBOW);
	allStopped &= PCA9685_UpdateSmoothServo(&SERVO_WRIST);

	// Si hubo un error durante la actualización, se maneja dentro de PCA9685_UpdateSmoothServo
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
	if (!VerificarEstadoSistema())
	{
		MostrarErrorRobot(robotErrorState, "Home Position",
				"Sistema no está completamente operativo");
		return;
	}

	PCA9685_Status_t status = MoveRobot(0.0f, 0.0f, 0.0f, 0.0f, 2000);
	if (status == PCA9685_OK)
	{
		// Esperar movimiento
		uint32_t start = HAL_GetTick();
		while ((HAL_GetTick() - start) < 2500)
		{
			UpdateServos();
			HAL_Delay(20);
		}
	}
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
	uint32_t now = HAL_GetTick();
	float T[4][4];
	float x, y, z, pitch;
	int len;

	switch(secuenciaStep) {
	case 0: // MOVIMIENTO 1: Posición home
		if (now - secuenciaLastMove > 100)
		{
			q1 = 0.0f;
			q2 = 0.0f;
			q3 = 0.0f;
			q4 = 0.0f;

			MoveRobot(q1, q2, q3, q4, 2000);
			secuenciaLastMove = now;
			secuenciaStep++;

			ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2),
					DEG2RAD(q3), DEG2RAD(q4), T);
			GetEndEffectorPose(T, &x, &y, &z, &pitch);

			// Mostrar movimiento con color
			sprintf(buffer, "%s%s[Movimiento 1: Home]%s\r\n", 
					SUCCESS_COLOR, COLOR_BOLD, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			len = sprintf(buffer,
					"Angulos: [%s%.1f%s, %s%.1f%s, %s%.1f%s, %s%.1f%s] grados\r\n"
					"Posicion End-Effector:\r\n"
					"  %sX =%s %s%.2f%s mm\r\n"
					"  %sY =%s %s%.2f%s mm\r\n"
					"  %sZ =%s %s%.2f%s mm\r\n"
					"  %sPitch =%s %s%.2f%s grados\r\n\r\n",
					VALUE_COLOR, current_q1, COLOR_RESET,
					VALUE_COLOR, current_q2, COLOR_RESET,
					VALUE_COLOR, current_q3, COLOR_RESET,
					VALUE_COLOR, current_q4, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, x, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, y, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, z, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, RAD2DEG(pitch), COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 1000);
		}
		break;

	case 1: // Esperar 3 segundos
		if (now - secuenciaLastMove > DELAY_TEST_SEQUENCE)
		{
			secuenciaLastMove = now;
			secuenciaStep++;
		}
		break;

	case 2: // MOVIMIENTO 2: Rotar base
		if (now - secuenciaLastMove > 100)
		{
			q1 = 135.0f;
			q2 = 0.0f;
			q3 = 0.0f;
			q4 = 0.0f;

			MoveRobot(q1, q2, q3, q4, 2000);
			secuenciaLastMove = now;
			secuenciaStep++;

			ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2),
					DEG2RAD(q3), DEG2RAD(q4), T);
			GetEndEffectorPose(T, &x, &y, &z, &pitch);

			sprintf(buffer, "%s%s[Movimiento 2: Rotar base]%s\r\n", 
					SUCCESS_COLOR, COLOR_BOLD, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			len = sprintf(buffer,
					"Angulos: [%s%.1f%s, %s%.1f%s, %s%.1f%s, %s%.1f%s] grados\r\n"
					"Posicion End-Effector:\r\n"
					"  %sX =%s %s%.2f%s mm\r\n"
					"  %sY =%s %s%.2f%s mm\r\n"
					"  %sZ =%s %s%.2f%s mm\r\n"
					"  %sPitch =%s %s%.2f%s grados\r\n\r\n",
					VALUE_COLOR, current_q1, COLOR_RESET,
					VALUE_COLOR, current_q2, COLOR_RESET,
					VALUE_COLOR, current_q3, COLOR_RESET,
					VALUE_COLOR, current_q4, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, x, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, y, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, z, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, RAD2DEG(pitch), COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 1000);
		}
		break;

	case 3: // Esperar 2 segundos
		if (now - secuenciaLastMove > DELAY_TEST_SEQUENCE)
		{
			secuenciaLastMove = now;
			secuenciaStep++;
		}
		break;

	case 4: // MOVIMIENTO 3: Levantar brazo
		if (now - secuenciaLastMove > 100)
		{

			q1 = 135.0f;
			q2 = 45.0f;
			q3 = 0.0f;
			q4 = 0.0f;

			MoveRobot(q1, q2, q3, q4, 2000);
			secuenciaLastMove = now;
			secuenciaStep++;

			ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2),
					DEG2RAD(q3), DEG2RAD(q4), T);
			GetEndEffectorPose(T, &x, &y, &z, &pitch);

			sprintf(buffer, "%s%s[Movimiento 3: Hombro a 45 grados]%s\r\n", 
					SUCCESS_COLOR, COLOR_BOLD, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			len = sprintf(buffer,
					"Angulos: [%s%.1f%s, %s%.1f%s, %s%.1f%s, %s%.1f%s] grados\r\n"
					"Posicion End-Effector:\r\n"
					"  %sX =%s %s%.2f%s mm\r\n"
					"  %sY =%s %s%.2f%s mm\r\n"
					"  %sZ =%s %s%.2f%s mm\r\n"
					"  %sPitch =%s %s%.2f%s grados\r\n\r\n",
					VALUE_COLOR, current_q1, COLOR_RESET,
					VALUE_COLOR, current_q2, COLOR_RESET,
					VALUE_COLOR, current_q3, COLOR_RESET,
					VALUE_COLOR, current_q4, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, x, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, y, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, z, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, RAD2DEG(pitch), COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 1000);
		}
		break;

	case 5: // Esperar 2 segundos
		if (now - secuenciaLastMove > DELAY_TEST_SEQUENCE)
		{
			secuenciaLastMove = now;
			secuenciaStep++;
		}
		break;

	case 6: // MOVIMIENTO 4: Flexionar codo
		if (now - secuenciaLastMove > 100)
		{
			q1 = 135.0f;
			q2 = 45.0f;
			q3 = 60.0f;
			q4 = 0.0f;

			MoveRobot(q1, q2, q3, q4, 2000);
			secuenciaLastMove = now;
			secuenciaStep++;

			ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2),
					DEG2RAD(q3), DEG2RAD(q4), T);
			GetEndEffectorPose(T, &x, &y, &z, &pitch);

			sprintf(buffer, "%s%s[Movimiento 4: Codo 60 grados]%s\r\n", 
					SUCCESS_COLOR, COLOR_BOLD, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			len = sprintf(buffer,
					"Angulos: [%s%.1f%s, %s%.1f%s, %s%.1f%s, %s%.1f%s] grados\r\n"
					"Posicion End-Effector:\r\n"
					"  %sX =%s %s%.2f%s mm\r\n"
					"  %sY =%s %s%.2f%s mm\r\n"
					"  %sZ =%s %s%.2f%s mm\r\n"
					"  %sPitch =%s %s%.2f%s grados\r\n\r\n",
					VALUE_COLOR, current_q1, COLOR_RESET,
					VALUE_COLOR, current_q2, COLOR_RESET,
					VALUE_COLOR, current_q3, COLOR_RESET,
					VALUE_COLOR, current_q4, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, x, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, y, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, z, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, RAD2DEG(pitch), COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 1000);
		}
		break;

	case 7: // Esperar 2 segundos
		if (now - secuenciaLastMove > DELAY_TEST_SEQUENCE)
		{
			secuenciaLastMove = now;
			secuenciaStep++;
		}
		break;

	case 8: // MOVIMIENTO 5: Ajustar muñeca
		if (now - secuenciaLastMove > 100)
		{

			q1 = 135.0f;
			q2 = 45.0f;
			q3 = 60.0f;
			q4 = -30.0f;

			MoveRobot(q1, q2, q3, q4, 2000);
			secuenciaLastMove = now;
			secuenciaStep++;

			ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2),
					DEG2RAD(q3), DEG2RAD(q4), T);
			GetEndEffectorPose(T, &x, &y, &z, &pitch);

			sprintf(buffer, "%s%s[Movimiento 5: Muñeca -30 grados]%s\r\n", 
					SUCCESS_COLOR, COLOR_BOLD, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			len = sprintf(buffer,
					"Angulos: [%s%.1f%s, %s%.1f%s, %s%.1f%s, %s%.1f%s] grados\r\n"
					"Posicion End-Effector:\r\n"
					"  %sX =%s %s%.2f%s mm\r\n"
					"  %sY =%s %s%.2f%s mm\r\n"
					"  %sZ =%s %s%.2f%s mm\r\n"
					"  %sPitch =%s %s%.2f%s grados\r\n\r\n",
					VALUE_COLOR, current_q1, COLOR_RESET,
					VALUE_COLOR, current_q2, COLOR_RESET,
					VALUE_COLOR, current_q3, COLOR_RESET,
					VALUE_COLOR, current_q4, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, x, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, y, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, z, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, RAD2DEG(pitch), COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 1000);
		}
		break;

	case 9: // Esperar 1.5 segundos
		if (now - secuenciaLastMove > DELAY_TEST_SEQUENCE)
		{
			secuenciaLastMove = now;
			secuenciaStep++;
		}
		break;

	case 10: // MOVIMIENTO 6: Volver a home
		if (now - secuenciaLastMove > 100)
		{
			q1 = 0.0f;
			q2 = 0.0f;
			q3 = 0.0f;
			q4 = 0.0f;

			MoveRobot(q1, q2, q3, q4, 2000);
			secuenciaLastMove = now;
			secuenciaStep++;

			ForwardKinematics(DEG2RAD(q1), DEG2RAD(q2),
					DEG2RAD(q3), DEG2RAD(q4), T);
			GetEndEffectorPose(T, &x, &y, &z, &pitch);

			sprintf(buffer, "%s%s[Movimiento 6: Volver a Home]%s\r\n", 
					SUCCESS_COLOR, COLOR_BOLD, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			len = sprintf(buffer,
					"Angulos: [%s%.1f%s, %s%.1f%s, %s%.1f%s, %s%.1f%s] grados\r\n"
					"Posicion End-Effector:\r\n"
					"  %sX =%s %s%.2f%s mm\r\n"
					"  %sY =%s %s%.2f%s mm\r\n"
					"  %sZ =%s %s%.2f%s mm\r\n"
					"  %sPitch =%s %s%.2f%s grados\r\n\r\n",
					VALUE_COLOR, current_q1, COLOR_RESET,
					VALUE_COLOR, current_q2, COLOR_RESET,
					VALUE_COLOR, current_q3, COLOR_RESET,
					VALUE_COLOR, current_q4, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, x, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, y, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, z, COLOR_RESET,
					COLOR_GREEN, COLOR_RESET, VALUE_COLOR, RAD2DEG(pitch), COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 1000);
		}
		break;

	case 11: // Esperar y mostrar información final
		if (now - secuenciaLastMove > DELAY_TEST_SEQUENCE)
		{
			// Secuencia completada
			LimpiarPantalla();
			MostrarEncabezado();
			sprintf(buffer, "%s%s=== SECUENCIA COMPLETADA CON EXITO ===%s\r\n\r\n", 
					SUCCESS_COLOR, COLOR_BOLD, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			MostrarExito("Todos los movimientos se ejecutaron correctamente");
			sprintf(buffer, "\r\n%sPresione ENTER para volver al menu...%s", 
					COLOR_DIM, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			// Detener y volver al menú
			DetenerSecuencia();
			estadoMenu = MENU_PRINCIPAL;
		}
		break;
	}
}


// ============================================================================
// FUNCIONES DEL MENÚ UART
// ============================================================================

/**
 * @brief Muestra el menú principal por UART
 */
void MostrarMenuPrincipal(void)
{
	LimpiarPantalla();
	MostrarEncabezado();

	// Mostrar estado del sistema si hay errores
	if (robotErrorState != ROBOT_OK)
	{
		sprintf(buffer, "%s%s[ADVERTENCIA: SISTEMA CON ERRORES]%s\r\n",
				WARNING_COLOR, COLOR_BOLD, COLOR_RESET);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

		sprintf(buffer, "%sEstado: %s%s%s\r\n\r\n",
				INFO_COLOR, ERROR_COLOR,
				(i2cRetryCount >= I2C_MAX_RETRIES) ? "CRÍTICO" : "LIMITADO",
						COLOR_RESET);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
	}

	sprintf(buffer, "%s%s========== MENU PRINCIPAL ==========%s\r\n", 
			MENU_COLOR, COLOR_BOLD, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
	sprintf(buffer, "\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "  %s1.%s %sEjecutar secuencia automatica%s\r\n", 
			OPTION_COLOR, COLOR_RESET, COLOR_BRIGHT_GREEN, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "  %s2.%s %sIngresar angulos manualmente%s\r\n", 
			OPTION_COLOR, COLOR_RESET, COLOR_BRIGHT_BLUE, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "  %s3.%s %sPosicion HOME%s\r\n", 
			OPTION_COLOR, COLOR_RESET, COLOR_BRIGHT_CYAN, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "  %s4.%s %sVer posicion actual%s\r\n", 
			OPTION_COLOR, COLOR_RESET, COLOR_BRIGHT_YELLOW, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	if (secuenciaActiva)
	{
		sprintf(buffer, "  %s5.%s %sDETENER secuencia%s\r\n", 
				OPTION_COLOR, COLOR_RESET, COLOR_BRIGHT_RED, COLOR_RESET);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
	}

	sprintf(buffer, "\r\n%s====================================%s\r\n", 
			MENU_COLOR, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "\r\n%s[%sROBOT%s]%s Seleccione una opcion (1-%d): %s", 
			COLOR_BRIGHT_MAGENTA, ROBOT_COLOR, COLOR_BRIGHT_MAGENTA, 
			COLOR_RESET, secuenciaActiva ? 5 : 4, INPUT_COLOR);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	ultimaInteraccion = HAL_GetTick();
}

/**
 * @brief Muestra la posición actual del robot
 */
void MostrarPosicionActual(void)
{
	float T[4][4];
	float x, y, z, pitch;

	// Calcular cinemática directa
	ForwardKinematics(DEG2RAD(current_q1), DEG2RAD(current_q2),
			DEG2RAD(current_q3), DEG2RAD(current_q4), T);
	GetEndEffectorPose(T, &x, &y, &z, &pitch);

	LimpiarPantalla();
	MostrarEncabezado();

	sprintf(buffer, "%s%s--- POSICION ACTUAL DEL ROBOT ---%s\r\n\r\n", 
			HEADER_COLOR, COLOR_BOLD, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "%sAngulos de las articulaciones:%s\r\n", 
			INFO_COLOR, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "  %sBase (Q1):%s   %s%.1f%s grados\r\n", 
			COLOR_CYAN, COLOR_RESET, VALUE_COLOR, current_q1, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "  %sHombro (Q2):%s %s%.1f%s grados\r\n", 
			COLOR_CYAN, COLOR_RESET, VALUE_COLOR, current_q2, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "  %sCodo (Q3):%s   %s%.1f%s grados\r\n", 
			COLOR_CYAN, COLOR_RESET, VALUE_COLOR, current_q3, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "  %sMuñeca (Q4):%s %s%.1f%s grados\r\n\r\n", 
			COLOR_CYAN, COLOR_RESET, VALUE_COLOR, current_q4, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "%sPosicion End-Effector:%s\r\n", 
			INFO_COLOR, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "  %sX =%s %s%.2f%s mm\r\n", 
			COLOR_GREEN, COLOR_RESET, VALUE_COLOR, x, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "  %sY =%s %s%.2f%s mm\r\n", 
			COLOR_GREEN, COLOR_RESET, VALUE_COLOR, y, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "  %sZ =%s %s%.2f%s mm\r\n", 
			COLOR_GREEN, COLOR_RESET, VALUE_COLOR, z, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "  %sPitch =%s %s%.2f%s grados\r\n\r\n", 
			COLOR_GREEN, COLOR_RESET, VALUE_COLOR, RAD2DEG(pitch), COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "%sPresione ENTER para volver al menu...%s", 
			COLOR_DIM, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

/**
 * @brief Convierte un string a float (implementación simple)
 */
float ConvertirStringAFloat(char* str)
{
	float resultado = 0.0f;
	float decimal = 0.0f;
	int signo = 1;
	int idx = 0;
	bool despuesDelPunto = false;
	int decimales = 0;

	// Eliminar espacios al inicio
	while(str[idx] == ' ') idx++;

	// Verificar signo
	if(str[idx] == '-')
	{
		signo = -1;
		idx++;
	}
	else if(str[idx] == '+')
	{
		idx++;
	}

	// Convertir dígitos
	while(str[idx] != '\0' && str[idx] != '\r' && str[idx] != '\n')
	{
		if(str[idx] >= '0' && str[idx] <= '9')
		{
			if(!despuesDelPunto)
			{
				resultado = resultado * 10.0f + (str[idx] - '0');
			}
			else
			{
				decimales++;
				decimal = decimal * 10.0f + (str[idx] - '0');
			}
		}
		else if(str[idx] == '.' || str[idx] == ',')
		{
			despuesDelPunto = true;
		}
		idx++;
	}

	// Agregar parte decimal
	while(decimales > 0)
	{
		decimal /= 10.0f;
		decimales--;
	}

	resultado += decimal;
	return resultado * signo;
}

/**
 * @brief Solicita ángulos al usuario y mueve el robot
 */
void SolicitarAngulos(void)
{
	static uint8_t paso = 0;
	static float q1_temp = 0.0f, q2_temp = 0.0f, q3_temp = 0.0f, q4_temp = 0.0f;

	switch(paso)
	{
	case 0:
		LimpiarPantalla();
		MostrarEncabezado();

		sprintf(buffer, "%s%s--- INGRESO MANUAL DE ANGULOS ---%s\r\n\r\n", 
				HEADER_COLOR, COLOR_BOLD, COLOR_RESET);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

		sprintf(buffer, "%sRango valido:%s %s%.0f a %.0f grados%s\r\n\r\n", 
				INFO_COLOR, COLOR_RESET, VALUE_COLOR, Q1_MIN, Q1_MAX, COLOR_RESET);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

		sprintf(buffer, "%s[1/4]%s Angulo Base (Q1): %s", 
				COLOR_BRIGHT_MAGENTA, COLOR_RESET, INPUT_COLOR);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
		paso = 1;
		ultimaInteraccion = HAL_GetTick();
		break;

	case 1:
		q1_temp = ConvertirStringAFloat((char*)RxData);
		if (q1_temp < Q1_MIN || q1_temp > Q1_MAX)
		{
			MostrarError("Angulo fuera de rango!");
			sprintf(buffer, "%sRango permitido: %.0f-%.0f grados%s\r\n", 
					WARNING_COLOR, Q1_MIN, Q1_MAX, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
			sprintf(buffer, "%s[1/4]%s Angulo Base (Q1): %s", 
					COLOR_BRIGHT_MAGENTA, COLOR_RESET, INPUT_COLOR);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
		}
		else
		{
			sprintf(buffer, "\r\n%s[2/4]%s Angulo Hombro (Q2): %s", 
					COLOR_BRIGHT_MAGENTA, COLOR_RESET, INPUT_COLOR);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
			paso = 2;
			ultimaInteraccion = HAL_GetTick();
		}
		break;

	case 2:
		q2_temp = ConvertirStringAFloat((char*)RxData);
		if (q2_temp < Q2_MIN || q2_temp > Q2_MAX)
		{
			MostrarError("Angulo fuera de rango!");
			sprintf(buffer, "%sRango permitido: %.0f-%.0f grados%s\r\n", 
					WARNING_COLOR, Q2_MIN, Q2_MAX, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
			sprintf(buffer, "%s[2/4]%s Angulo Hombro (Q2): %s", 
					COLOR_BRIGHT_MAGENTA, COLOR_RESET, INPUT_COLOR);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
		}
		else
		{
			sprintf(buffer, "\r\n%s[3/4]%s Angulo Codo (Q3): %s", 
					COLOR_BRIGHT_MAGENTA, COLOR_RESET, INPUT_COLOR);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
			paso = 3;
			ultimaInteraccion = HAL_GetTick();
		}
		break;

	case 3:
		q3_temp = ConvertirStringAFloat((char*)RxData);
		if (q3_temp < Q3_MIN || q3_temp > Q3_MAX)
		{
			MostrarError("Angulo fuera de rango!");
			sprintf(buffer, "%sRango permitido: %.0f-%.0f grados%s\r\n", 
					WARNING_COLOR, Q3_MIN, Q3_MAX, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
			sprintf(buffer, "%s[3/4]%s Angulo Codo (Q3): %s", 
					COLOR_BRIGHT_MAGENTA, COLOR_RESET, INPUT_COLOR);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
		}
		else
		{
			sprintf(buffer, "\r\n%s[4/4]%s Angulo Muñeca (Q4): %s", 
					COLOR_BRIGHT_MAGENTA, COLOR_RESET, INPUT_COLOR);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
			paso = 4;
			ultimaInteraccion = HAL_GetTick();
		}
		break;

	case 4:
		q4_temp = ConvertirStringAFloat((char*)RxData);
		if (q4_temp < Q4_MIN || q4_temp > Q4_MAX)
		{
			MostrarError("Angulo fuera de rango!");
			sprintf(buffer, "%sRango permitido: %.0f-%.0f grados%s\r\n", 
					WARNING_COLOR, Q4_MIN, Q4_MAX, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
			sprintf(buffer, "%s[4/4]%s Angulo Muñeca (Q4): %s", 
					COLOR_BRIGHT_MAGENTA, COLOR_RESET, INPUT_COLOR);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
		}
		else
		{
			// Mostrar resumen
			LimpiarPantalla();
			MostrarEncabezado();

			sprintf(buffer, "%s%s--- RESUMEN DE ANGULOS ---%s\r\n\r\n", 
					HEADER_COLOR, COLOR_BOLD, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			sprintf(buffer, "%sBase:%s   %s%.1f%s grados\r\n", 
					COLOR_CYAN, COLOR_RESET, VALUE_COLOR, q1_temp, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			sprintf(buffer, "%sHombro:%s %s%.1f%s grados\r\n", 
					COLOR_CYAN, COLOR_RESET, VALUE_COLOR, q2_temp, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			sprintf(buffer, "%sCodo:%s   %s%.1f%s grados\r\n", 
					COLOR_CYAN, COLOR_RESET, VALUE_COLOR, q3_temp, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			sprintf(buffer, "%sMuñeca:%s %s%.1f%s grados\r\n\r\n", 
					COLOR_CYAN, COLOR_RESET, VALUE_COLOR, q4_temp, COLOR_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

			// Mover robot
			MostrarInfo("Moviendo robot...");
			MoveRobotBlocking(q1_temp, q2_temp, q3_temp, q4_temp, 3000);
			MostrarExito("Movimiento completado!");

			// Mostrar posición alcanzada
			MostrarPosicionActual();

			// Reiniciar y volver al menú
			paso = 0;
			ultimaInteraccion = HAL_GetTick();
			estadoMenu = MENU_PRINCIPAL;
		}
		break;
	}
}

/**
 * @brief Procesa la opción seleccionada del menú
 */
void ProcesarOpcionMenu(void)
{
	if(estadoMenu == MENU_PRINCIPAL)
	{
		// Convertir primer carácter a número
		int opcion = RxData[0] - '0';

		switch(opcion)
		{
		case 1: // Secuencia automática
			if (!secuenciaActiva)
			{
				LimpiarPantalla();
				MostrarEncabezado();
				MostrarInfo("Iniciando secuencia automatica...");
				sprintf(buffer, "%s(Presione 5 en el menu para detener)%s\r\n\r\n", 
						WARNING_COLOR, COLOR_RESET);
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
				IniciarSecuenciaAutomatica();
				estadoMenu = EJECUTANDO_SECUENCIA;
			}else
			{
				MostrarAdvertencia("Secuencia ya esta en ejecucion!");
				MostrarMenuPrincipal();
			}
			break;

		case 2: // Ingresar ángulos
			if (secuenciaActiva)
			{
				MostrarError("Debe detener la secuencia primero (opcion 5)");
				MostrarMenuPrincipal();
			}else
			{
				estadoMenu = MENU_ANGULOS;
				SolicitarAngulos();
			}
			break;

		case 3: // HOME
			if (secuenciaActiva)
			{
				MostrarError("Debe detener la secuencia primero (opcion 5)");
				MostrarMenuPrincipal();
			}else
			{
				LimpiarPantalla();
				MostrarEncabezado();
				MostrarInfo("Moviendo a posicion HOME...");
				HomePosition();
				MostrarExito("Robot en posicion HOME");
				MostrarMenuPrincipal();
			}
			break;

		case 4: // Ver posición actual
			MostrarPosicionActual();
			// No mostramos menú aquí, se espera ENTER en MostrarPosicionActual
			break;

		case 5: // Detener secuencia
			if(secuenciaActiva)
			{
				DetenerSecuencia();
				LimpiarPantalla();
				MostrarEncabezado();
				MostrarExito("Secuencia detenida!");
				estadoMenu = MENU_PRINCIPAL;
				MostrarMenuPrincipal();
			}
			else
			{
				MostrarAdvertencia("No hay secuencia en ejecucion.");
				MostrarMenuPrincipal();
			}
			break;

		case 6: // Mostrar estado del sistema
			MostrarEstadoSistema();
			break;

		default:
			MostrarError("Opcion invalida. Intente de nuevo.");
			MostrarMenuPrincipal();
			break;
		}
	}
	else if(estadoMenu == MENU_ANGULOS)
	{
		SolicitarAngulos();
	}
	else if (estadoMenu == EJECUTANDO_SECUENCIA)
	{
		// Durante la secuencia, solo permitir ver menú o detener
		int opcion = RxData[0] - '0';
		if(opcion == 5)
		{
			DetenerSecuencia();
			LimpiarPantalla();
			MostrarEncabezado();
			MostrarExito("Secuencia detenida!");
			estadoMenu = MENU_PRINCIPAL;
			MostrarMenuPrincipal();
		}
		else
		{
			MostrarAdvertencia("Secuencia en ejecucion. Presione 5 para detener.");
		}
	}
}

/**
 * @brief Inicia la secuencia automática
 */
void IniciarSecuenciaAutomatica(void)
{
	secuenciaActiva = true;
	secuenciaStep = 0;
	secuenciaLastMove = HAL_GetTick();
}

/**
 * @brief Detiene la secuencia automática
 */
void DetenerSecuencia(void)
{
	secuenciaActiva = false;
	secuenciaStep = 0;
}

// ============================================================================
// FUNCIONES DE INTERFAZ DE USUARIO
// ============================================================================

/**
 * @brief Limpia la pantalla del terminal
 */
void LimpiarPantalla(void)
{
	sprintf(buffer, "%s%s", CLEAR_SCREEN, CURSOR_HOME);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

/**
 * @brief Muestra el encabezado del sistema
 */
void MostrarEncabezado(void)
{
	MostrarSeparador(SEPARATOR_COLOR);

	sprintf(buffer, "%s%s     KUKA LBR IISY 4 DOF ROBOT ARM%s\r\n", 
			ROBOT_COLOR, COLOR_BOLD, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "%s     Control System v2.1%s\r\n", 
			HEADER_COLOR, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "%s     Cinematica Directa DH%s\r\n", 
			INFO_COLOR, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	MostrarSeparador(SEPARATOR_COLOR);
	sprintf(buffer, "\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

/**
 * @brief Muestra el estado actual del sistema
 *
 */
void MostrarEstadoSistema(void)
{
	LimpiarPantalla();
	MostrarEncabezado();

	sprintf(buffer, "%s%s--- ESTADO DEL SISTEMA ---%s\r\n\r\n",
			HEADER_COLOR, COLOR_BOLD, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "%sInicialización:%s %s%s%s\r\n",
			COLOR_CYAN, COLOR_RESET,
			systemInitialized ? SUCCESS_COLOR : ERROR_COLOR,
					systemInitialized ? "COMPLETA" : "FALLIDA",
							COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "%sEstado PCA9685:%s %s%s%s\r\n",
			COLOR_CYAN, COLOR_RESET,
			(robotErrorState == ROBOT_OK) ? SUCCESS_COLOR : ERROR_COLOR,
					(robotErrorState == ROBOT_OK) ? "OK" : "ERROR",
							COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "%sReintentos I2C:%s %s%d/%d%s\r\n",
			COLOR_CYAN, COLOR_RESET,
			VALUE_COLOR, i2cRetryCount, I2C_MAX_RETRIES, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	sprintf(buffer, "%sEstado general:%s %s%s%s\r\n\r\n",
			COLOR_CYAN, COLOR_RESET,
			VerificarEstadoSistema() ? SUCCESS_COLOR : ERROR_COLOR,
					VerificarEstadoSistema() ? "OPERATIVO" : "CON ERRORES",
							COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

	if (robotErrorState != ROBOT_OK)
	{
		sprintf(buffer, "%sÚltimo error reportado:%s\r\n",
				WARNING_COLOR, COLOR_RESET);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

		// Aquí podrías guardar el último mensaje de error para mostrarlo
	}

	sprintf(buffer, "%sPresione ENTER para volver al menú...%s",
			COLOR_DIM, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

/**
 * @brief Muestra una línea separadora con color
 */
void MostrarSeparador(const char* color)
{
	sprintf(buffer, "%s==================================================%s\r\n", 
			color, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

/**
 * @brief Muestra un mensaje de éxito
 */
void MostrarExito(const char* mensaje)
{
	sprintf(buffer, "%s✓ %s%s\r\n", SUCCESS_COLOR, mensaje, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

/**
 * @brief Muestra un mensaje de error
 */
void MostrarError(const char* mensaje)
{
	sprintf(buffer, "%s✗ %s%s\r\n", ERROR_COLOR, mensaje, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

/**
 * @brief Muestra un mensaje de advertencia
 */
void MostrarAdvertencia(const char* mensaje)
{
	sprintf(buffer, "\r\n%s⚠ %s%s\r\n", WARNING_COLOR, mensaje, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

/**
 * @brief Muestra un mensaje informativo
 */
void MostrarInfo(const char* mensaje)
{
	sprintf(buffer, "%sℹ %s%s\r\n", INFO_COLOR, mensaje, COLOR_RESET);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}



/**
 * @brief  Callback de recepción UART
 * @param  huart: Handle del UART
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Verificar si es ENTER (carriage return o line feed)
	if(RxByte == '\r' || RxByte == '\n')
	{
		// Marcar que hay un mensaje completo
		if(indx > 0) // Solo si hay datos
		{
			RxData[indx] = '\0'; // Null terminator
			newMessageReady = true;

			// Echo del ENTER
			sprintf(buffer, "\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 10);
		}
		else if (estadoMenu == MENU_PRINCIPAL)
		{
			// Si se presiona ENTER sin datos, refrescar menú
			MostrarMenuPrincipal();
		}
	}
	else if(RxByte == 127 || RxByte == 8)	// Backspace o DEL
	{
		// Eliminar último carácter
		if(indx > 0)
		{
			indx--;
			RxData[indx] = 0;
			// Echo del backspace
			HAL_UART_Transmit(&huart1, (uint8_t*)"\b \b", 3, 10);
		}
	}
	else if(indx < BUFFER_SIZE - 1)  // Protección contra desbordamiento
	{
		// Almacenar el byte recibido
		RxData[indx] = RxByte;
		indx++;

		// Echo del carácter recibido (con color de input)
		HAL_UART_Transmit(&huart1, &RxByte, 1, 10);
	}
	// Reiniciar recepción para el siguiente byte
	HAL_UART_Receive_IT(&huart1, &RxByte, 1);
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
