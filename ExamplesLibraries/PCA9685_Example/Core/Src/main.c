/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PCA9685.h"
#include "ANSII_Codes.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Parámetros de la batería de pruebas -- */
#define PCA_CANAL_SERVO     0U         /**< Canal con el servo conectado (prueba visual) */
#define PCA_CANAL_EXTREMOS  15U        /**< Canal SIN servo: recibe ciclos de trabajo del 0 al 100 % */
#define PCA_FREQ_SERVO      50U        /**< Frecuencia de trabajo para servos [Hz] */
#define PCA_PERIODO_MS      20U        /**< Intervalo entre pasos del movimiento suave [ms] */
#define PCA_MOV_MS          2000U      /**< Duración de cada movimiento suave [ms] */
/* Retraso tolerado sobre la duración pedida: cada paso suma la escritura I2C y el +1 ms
 * de HAL_Delay. Ajustar si la placa falla con un I2C más lento */
#define PCA_MARGEN_MS       300U
#define PCA_I2C_TIMEOUT     100U       /**< Timeout de las relecturas directas por HAL [ms] */

/* Cuentas OFF que escribe SetServoAngle para 0°, 90° y 180° (0.5 / 1.5 / 2.5 ms a 50 Hz) */
#define PCA_SERVO_0         102U
#define PCA_SERVO_90        307U
#define PCA_SERVO_180       511U

/* PRE_SCALE que escribe Init a 50 Hz: 25 MHz / (4096 * 50) truncado */
#define PCA_PRESCALE_50HZ   122U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PCA_CHECK(x)  do { if ((x) != PCA9685_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define PCA_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
PCA9685_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t PCA9685_LeerReg(uint8_t reg);
static void PCA9685_CheckCanal(uint8_t ch, uint16_t on, uint16_t off);
static void PCA9685_CheckModo(uint8_t prescale);
static void PCA9685_CheckDuracion(uint32_t t0, uint32_t durationMs);
static void PCA9685_TestSuite(void);
static void PCA9685_TestPreInit(void);
static void PCA9685_TestRegistros(void);
static void PCA9685_TestPWM(void);
static void PCA9685_TestServo(void);
static void PCA9685_TestSmoothMove(void);
static void PCA9685_TestSmoothServo(void);
static void PCA9685_TestErrores(void);
static void PCA9685_TestFrecuencia(void);
static void PCA9685_TestDeInit(void);
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
  printf(CLEAR_SCREEN CURSOR_HOME COLOR_BOLD COLOR_CYAN "PCA9685 TEST" COLOR_RESET "\r\n");
  PCA9685_TestPreInit();

  status = PCA9685_Init(&hi2c1, PCA_FREQ_SERVO);
  if(status != PCA9685_OK)
  {
	  Error_Handler();
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  PCA9685_TestSuite();
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

/* printf por USART1 (PA9): syscalls.c redirige _write a esta función débil */
int __io_putchar(int ch)
{
	uint8_t c = (uint8_t)ch;
	HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
	return ch;
}

/* La librería no tiene lecturas: se relee el chip directamente por HAL para
 * comprobar lo que de verdad quedó escrito en sus registros */
static uint8_t PCA9685_LeerReg(uint8_t reg)
{
	uint8_t v;

	if (HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &v, 1, PCA_I2C_TIMEOUT) != HAL_OK)
	{
		Error_Handler();
	}
	return v;
}

/* LEDn_ON_L..LEDn_OFF_H de un canal en una sola lectura de 4 bytes (requiere auto-incremento) */
static void PCA9685_CheckCanal(uint8_t ch, uint16_t on, uint16_t off)
{
	uint8_t r[4];

	if (HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, PCA9685_LED0_ON_L + 4U * ch, I2C_MEMADD_SIZE_8BIT,
						 r, sizeof(r), PCA_I2C_TIMEOUT) != HAL_OK)
	{
		Error_Handler();
	}
	if ((uint16_t)(r[0] | (r[1] << 8)) != on || (uint16_t)(r[2] | (r[3] << 8)) != off)
	{
		Error_Handler();
	}
}

/* Tras un Init: PRE_SCALE programado, auto-incremento activo y fuera de sleep (oscilador en marcha) */
static void PCA9685_CheckModo(uint8_t prescale)
{
	const uint8_t mode1 = PCA9685_LeerReg(PCA9685_MODE1);
	const uint8_t pre   = PCA9685_LeerReg(PCA9685_PRE_SCALE);

	printf("MODE1=0x%02X PRE_SCALE=%u\r\n", mode1, pre);
	if (pre != prescale ||
		(mode1 & (1U << PCA9685_MODE1_AI_BIT)) == 0U ||
		(mode1 & (1U << PCA9685_MODE1_SLEEP_BIT)) != 0U)
	{
		Error_Handler();
	}
}

/* Un movimiento suave nunca dura menos de lo pedido, y como mucho PCA_MARGEN_MS más */
static void PCA9685_CheckDuracion(uint32_t t0, uint32_t durationMs)
{
	const uint32_t dt = HAL_GetTick() - t0;

	printf("Duracion: %lu ms (pedido %lu ms)\r\n", dt, durationMs);
	if (dt < durationMs || dt > durationMs + PCA_MARGEN_MS)
	{
		Error_Handler();
	}
}

static void PCA9685_TestSuite(void)
{
	PCA9685_TestRegistros();
	HAL_Delay(2000);

	PCA9685_TestPWM();
	HAL_Delay(2000);

	PCA9685_TestServo();
	HAL_Delay(2000);

	PCA9685_TestSmoothMove();
	HAL_Delay(2000);

	PCA9685_TestSmoothServo();
	HAL_Delay(2000);

	PCA9685_TestErrores();
	HAL_Delay(2000);

	PCA9685_TestFrecuencia();
	HAL_Delay(2000);

	PCA9685_TestDeInit();
	HAL_Delay(2000);
}

static void PCA9685_TestPreInit(void)
{
	Servo_Smooth_t s;

	/* -- Antes de Init: todo devuelve NOT_INITIALIZED sin tocar el bus (SmoothMove sale
	 *    en el primer SetServoAngle, sin esperar la duración del movimiento) -- */
	PCA_EXPECT(PCA9685_SetPWM(PCA_CANAL_SERVO, 0, PCA_SERVO_90), PCA9685_NOT_INITIALIZED);
	PCA_EXPECT(PCA9685_SetServoAngle(PCA_CANAL_SERVO, 90.0f), PCA9685_NOT_INITIALIZED);
	PCA_EXPECT(PCA9685_InitSmoothServo(&s, PCA_CANAL_SERVO, 90.0f, PCA_PERIODO_MS), PCA9685_NOT_INITIALIZED);
	PCA_EXPECT(PCA9685_SmoothMove(PCA_CANAL_SERVO, 0.0f, 180.0f, PCA_MOV_MS, PCA_PERIODO_MS), PCA9685_NOT_INITIALIZED);

	/* DeInit sobre un driver sin inicializar es inofensivo */
	PCA_CHECK(PCA9685_DeInit());
}

static void PCA9685_TestRegistros(void)
{
	/* -- Init a 50 Hz: el chip debe tener el PRE_SCALE de 50 Hz, AI = 1 y SLEEP = 0 -- */
	printf(COLOR_YELLOW "[Registros]" COLOR_RESET "\r\n");
	PCA9685_CheckModo(PCA_PRESCALE_50HZ);
}

static void PCA9685_TestPWM(void)
{
	/* -- SetPWM en los 16 canales con ON y OFF distintos por canal. Se escriben todos antes de
	 *    releer, así una escritura en el canal equivocado pisa a otro y se detecta. El pulso es
	 *    siempre de 1.5 ms (OFF - ON = 307): un servo en cualquier canal se queda centrado -- */
	printf(COLOR_YELLOW "[PWM]" COLOR_RESET "\r\n");
	for (uint8_t ch = 0; ch < 16U; ch++)
	{
		PCA_CHECK(PCA9685_SetPWM(ch, ch * 8U, ch * 8U + PCA_SERVO_90));
	}
	for (uint8_t ch = 0; ch < 16U; ch++)
	{
		PCA9685_CheckCanal(ch, ch * 8U, ch * 8U + PCA_SERVO_90);
		printf("Canal %2u ON=%4u OFF=%4u\r\n", ch, ch * 8U, ch * 8U + PCA_SERVO_90);
	}

	/* -- Extremos del rango 0..4095 en ON y en OFF -- */
	PCA_CHECK(PCA9685_SetPWM(PCA_CANAL_EXTREMOS, 0, 4095));
	PCA9685_CheckCanal(PCA_CANAL_EXTREMOS, 0, 4095);
	PCA_CHECK(PCA9685_SetPWM(PCA_CANAL_EXTREMOS, 4095, 0));
	PCA9685_CheckCanal(PCA_CANAL_EXTREMOS, 4095, 0);
	printf(COLOR_GREEN "PWM OK" COLOR_RESET "\r\n");
}

static void PCA9685_TestServo(void)
{
	static const float    angulos[] = { 0.0f, 90.0f, 180.0f };
	static const uint16_t cuentas[] = { PCA_SERVO_0, PCA_SERVO_90, PCA_SERVO_180 };

	/* -- SetServoAngle: 0°, 90° y 180° dan 0.5, 1.5 y 2.5 ms. El servo debe recorrer todo su giro -- */
	printf(COLOR_YELLOW "[Servo]" COLOR_RESET "\r\n");
	for (uint8_t i = 0; i < 3U; i++)
	{
		PCA_CHECK(PCA9685_SetServoAngle(PCA_CANAL_SERVO, angulos[i]));
		PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, cuentas[i]);
		printf("%3u grados -> OFF=%u\r\n", (unsigned)angulos[i], cuentas[i]);
		HAL_Delay(1000);
	}

	/* -- Fuera de 0..180 se recorta al extremo sin devolver error -- */
	PCA_CHECK(PCA9685_SetServoAngle(PCA_CANAL_SERVO, -10.0f));
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_0);
	HAL_Delay(1000);
	PCA_CHECK(PCA9685_SetServoAngle(PCA_CANAL_SERVO, 200.0f));
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_180);
	HAL_Delay(1000);
}

static void PCA9685_TestSmoothMove(void)
{
	uint32_t t0;

	/* -- SmoothMove bloqueante: 0° -> 180° y vuelta en PCA_MOV_MS cada uno, sin tirones.
	 *    Debe durar lo pedido y terminar exactamente en el ángulo final -- */
	printf(COLOR_YELLOW "[SmoothMove]" COLOR_RESET "\r\n");
	t0 = HAL_GetTick();
	PCA_CHECK(PCA9685_SmoothMove(PCA_CANAL_SERVO, 0.0f, 180.0f, PCA_MOV_MS, PCA_PERIODO_MS));
	PCA9685_CheckDuracion(t0, PCA_MOV_MS);
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_180);

	t0 = HAL_GetTick();
	PCA_CHECK(PCA9685_SmoothMove(PCA_CANAL_SERVO, 180.0f, 0.0f, PCA_MOV_MS, PCA_PERIODO_MS));
	PCA9685_CheckDuracion(t0, PCA_MOV_MS);
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_0);

	/* -- Duración menor que un intervalo: salto inmediato al ángulo final -- */
	t0 = HAL_GetTick();
	PCA_CHECK(PCA9685_SmoothMove(PCA_CANAL_SERVO, 0.0f, 90.0f, PCA_PERIODO_MS - 1U, PCA_PERIODO_MS));
	PCA9685_CheckDuracion(t0, 0);
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_90);
}

static void PCA9685_TestSmoothServo(void)
{
	Servo_Smooth_t s;
	uint32_t t0;

	/* -- InitSmoothServo coloca el servo en el ángulo inicial, parado -- */
	printf(COLOR_YELLOW "[SmoothServo]" COLOR_RESET "\r\n");
	PCA_CHECK(PCA9685_InitSmoothServo(&s, PCA_CANAL_SERVO, 0.0f, PCA_PERIODO_MS));
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_0);
	if (s.isMoving)
	{
		Error_Handler();
	}
	HAL_Delay(1000);

	/* -- SetSmoothAngle + UpdateSmoothServo en bucle: 0° -> 180° sin bloquear. El bucle podría
	 *    atender otras tareas entre llamadas; el timeout evita colgarse si nunca termina -- */
	t0 = HAL_GetTick();
	PCA_CHECK(PCA9685_SetSmoothAngle(&s, 180.0f, PCA_MOV_MS));
	if (!s.isMoving)
	{
		Error_Handler();
	}
	while (!PCA9685_UpdateSmoothServo(&s))
	{
		if (HAL_GetTick() - t0 > PCA_MOV_MS + PCA_MARGEN_MS)
		{
			Error_Handler();
		}
	}
	PCA9685_CheckDuracion(t0, PCA_MOV_MS);
	if (s.lastError != PCA9685_OK || s.isMoving || s.currentAngle != 180.0f)
	{
		Error_Handler();
	}
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_180);

	/* -- Ya parado: Update devuelve true enseguida -- */
	if (!PCA9685_UpdateSmoothServo(&s))
	{
		Error_Handler();
	}

	/* -- Duración 0: salto directo al objetivo (recortado a 0..180), sin quedar en movimiento -- */
	PCA_CHECK(PCA9685_SetSmoothAngle(&s, -50.0f, 0));
	if (s.isMoving || s.currentAngle != 0.0f)
	{
		Error_Handler();
	}
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_0);
	HAL_Delay(1000);

	PCA_CHECK(PCA9685_SetSmoothAngle(&s, 90.0f, 0));
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_90);
}

static void PCA9685_TestErrores(void)
{
	Servo_Smooth_t s;

	/* -- SetPWM / SetServoAngle: canal > 15 o tiempos > 4095 se rechazan sin escribir nada
	 *    (el canal del servo conserva su valor) -- */
	printf(COLOR_YELLOW "[Errores]" COLOR_RESET "\r\n");
	PCA_CHECK(PCA9685_SetServoAngle(PCA_CANAL_SERVO, 90.0f));
	PCA_EXPECT(PCA9685_SetPWM(16, 0, PCA_SERVO_90), PCA9685_INVALID_PARAM);
	PCA_EXPECT(PCA9685_SetPWM(PCA_CANAL_SERVO, 4096, 0), PCA9685_INVALID_PARAM);
	PCA_EXPECT(PCA9685_SetPWM(PCA_CANAL_SERVO, 0, 4096), PCA9685_INVALID_PARAM);
	PCA_EXPECT(PCA9685_SetServoAngle(16, 90.0f), PCA9685_INVALID_PARAM);
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_90);

	/* -- Funciones de movimiento suave: devuelven PCA9685_ERROR (no INVALID_PARAM) -- */
	PCA_EXPECT(PCA9685_InitSmoothServo(NULL, PCA_CANAL_SERVO, 90.0f, PCA_PERIODO_MS), PCA9685_ERROR);
	PCA_EXPECT(PCA9685_InitSmoothServo(&s, 16, 90.0f, PCA_PERIODO_MS), PCA9685_ERROR);
	PCA_EXPECT(PCA9685_InitSmoothServo(&s, PCA_CANAL_SERVO, -1.0f, PCA_PERIODO_MS), PCA9685_ERROR);
	PCA_EXPECT(PCA9685_InitSmoothServo(&s, PCA_CANAL_SERVO, 181.0f, PCA_PERIODO_MS), PCA9685_ERROR);
	PCA_EXPECT(PCA9685_InitSmoothServo(&s, PCA_CANAL_SERVO, 90.0f, 0), PCA9685_ERROR);
	PCA_EXPECT(PCA9685_SetSmoothAngle(NULL, 90.0f, PCA_MOV_MS), PCA9685_ERROR);
	PCA_EXPECT(PCA9685_SmoothMove(16, 0.0f, 180.0f, PCA_MOV_MS, PCA_PERIODO_MS), PCA9685_ERROR);
	PCA_EXPECT(PCA9685_SmoothMove(PCA_CANAL_SERVO, 0.0f, 180.0f, PCA_MOV_MS, 0), PCA9685_ERROR);
	/* Update con NULL devuelve true para no colgar el bucle de quien lo llama */
	if (!PCA9685_UpdateSmoothServo(NULL))
	{
		Error_Handler();
	}
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_90);

	/* -- Init con handle NULL o frecuencia fuera de 24..1526 Hz: se rechaza sin tocar el chip
	 *    ni el estado, el driver sigue funcionando a 50 Hz -- */
	PCA_EXPECT(PCA9685_Init(NULL, PCA_FREQ_SERVO), PCA9685_INVALID_PARAM);
	PCA_EXPECT(PCA9685_Init(&hi2c1, 23), PCA9685_INVALID_PARAM);
	PCA_EXPECT(PCA9685_Init(&hi2c1, 1527), PCA9685_INVALID_PARAM);
	PCA9685_CheckModo(PCA_PRESCALE_50HZ);
	PCA_CHECK(PCA9685_SetServoAngle(PCA_CANAL_SERVO, 90.0f));
	printf(COLOR_GREEN "Errores OK" COLOR_RESET "\r\n");
}

static void PCA9685_TestFrecuencia(void)
{
	/* PRE_SCALE esperado: 25 MHz / (4096 * f) truncado; los extremos los fija la librería */
	static const struct { uint16_t f; uint8_t pre; } tabla[] = {
		{ 1000U, 6U },
		{ 24U,   0xFFU },
		{ 1526U, 0x03U },
		{ PCA_FREQ_SERVO, PCA_PRESCALE_50HZ },   /* Restaura la frecuencia de trabajo */
	};

	/* -- Antes de cambiar la frecuencia se dejan las 16 salidas a nivel bajo (ON = OFF = 0):
	 *    un servo no debe recibir PWM a 24 Hz ni a 1 kHz -- */
	printf(COLOR_YELLOW "[Frecuencia]" COLOR_RESET "\r\n");
	for (uint8_t ch = 0; ch < 16U; ch++)
	{
		PCA_CHECK(PCA9685_SetPWM(ch, 0, 0));
	}

	/* -- Re-Init a otras frecuencias, incluidos los límites 24 y 1526 Hz. Init no borra los canales -- */
	for (uint8_t i = 0; i < sizeof(tabla) / sizeof(tabla[0]); i++)
	{
		PCA_CHECK(PCA9685_Init(&hi2c1, tabla[i].f));
		printf("%4u Hz -> ", tabla[i].f);
		PCA9685_CheckModo(tabla[i].pre);
		PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, 0);
	}

	PCA_CHECK(PCA9685_SetServoAngle(PCA_CANAL_SERVO, 90.0f));
	printf(COLOR_GREEN "Frecuencia OK" COLOR_RESET "\r\n");
}

static void PCA9685_TestDeInit(void)
{
	Servo_Smooth_t s;

	/* -- DeInit libera el handle: las llamadas posteriores devuelven NOT_INITIALIZED.
	 *    Repetirlo sigue devolviendo OK -- */
	printf(COLOR_YELLOW "[DeInit]" COLOR_RESET "\r\n");
	PCA_CHECK(PCA9685_DeInit());
	PCA_CHECK(PCA9685_DeInit());
	PCA_EXPECT(PCA9685_SetPWM(PCA_CANAL_SERVO, 0, PCA_SERVO_0), PCA9685_NOT_INITIALIZED);
	PCA_EXPECT(PCA9685_SetServoAngle(PCA_CANAL_SERVO, 0.0f), PCA9685_NOT_INITIALIZED);
	PCA_EXPECT(PCA9685_InitSmoothServo(&s, PCA_CANAL_SERVO, 0.0f, PCA_PERIODO_MS), PCA9685_NOT_INITIALIZED);
	PCA_EXPECT(PCA9685_SmoothMove(PCA_CANAL_SERVO, 90.0f, 0.0f, PCA_MOV_MS, PCA_PERIODO_MS), PCA9685_NOT_INITIALIZED);

	/* -- DeInit no toca el chip: sigue generando PWM y el servo se queda a 90° -- */
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_90);

	/* -- Re-Init: el driver debe poder arrancar de nuevo tras un DeInit -- */
	PCA_CHECK(PCA9685_Init(&hi2c1, PCA_FREQ_SERVO));
	PCA9685_CheckModo(PCA_PRESCALE_50HZ);
	PCA_CHECK(PCA9685_SetServoAngle(PCA_CANAL_SERVO, 90.0f));
	PCA9685_CheckCanal(PCA_CANAL_SERVO, 0, PCA_SERVO_90);
	printf(COLOR_GREEN "Re-Init OK" COLOR_RESET "\r\n");
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
  /* Si la UART aún no está inicializada, HAL_UART_Transmit devuelve BUSY sin bloquear */
  printf(COLOR_BOLD COLOR_RED "\r\nERROR: prueba fallida" COLOR_RESET "\r\n");
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  HAL_Delay(200);
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
