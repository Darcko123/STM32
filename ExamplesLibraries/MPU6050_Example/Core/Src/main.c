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
#include "MPU6050.h"
#include "ANSII_Codes.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Una lectura completa del sensor: aceleración, giroscopio y temperatura */
typedef struct {
	MPU6050_Accel_t a;
	MPU6050_Gyro_t  g;
	float           t;
} MPU_Muestra_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Parámetros de la batería de pruebas -- */
#define MPU_MUESTRAS        10U        /**< Lecturas seguidas del test de muestras */
#define MPU_SENTINEL        1000.0f    /**< Fuera de escala (±2 g, ±250 dps): delata campos que Read no escribió */

/* Rango de funcionamiento del sensor */
#define MPU_TEMP_MIN        (-40.0f)
#define MPU_TEMP_MAX        (85.0f)

/* En reposo: |a| ~ 1 g y giro ~ 0 dps. Tolerancias amplias porque el sensor viene sin
 * calibrar (el datasheet da ±20 dps de offset del giroscopio); ajustar si la placa falla quieta */
#define MPU_ACCEL_MOD_MIN   0.8f       /**< |a| mínimo en reposo [g] */
#define MPU_ACCEL_MOD_MAX   1.2f       /**< |a| máximo en reposo [g] */
#define MPU_GYRO_REPOSO     30.0f      /**< |giro| máximo por eje en reposo [dps] */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MPU_CHECK(x)  do { if ((x) != MPU6050_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define MPU_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
MPU6050_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MPU6050_Centinela(MPU_Muestra_t* m);
static void MPU6050_Leer(MPU_Muestra_t* m);
static void MPU6050_CheckReposo(const MPU_Muestra_t* m);
static void MPU6050_CheckSinTocar(const MPU_Muestra_t* m);
static void MPU6050_CheckDormido(void);
static void MPU6050_CheckDespierto(void);
static void MPU6050_PrintVal(const char* nombre, float v);
static void MPU6050_Print(const MPU_Muestra_t* m);
static void MPU6050_TestSuite(void);
static void MPU6050_TestPreInit(void);
static void MPU6050_TestLectura(void);
static void MPU6050_TestMuestras(void);
static void MPU6050_TestSleep(void);
static void MPU6050_TestErrores(void);
static void MPU6050_TestDeInit(void);
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
  printf(CLEAR_SCREEN CURSOR_HOME COLOR_BOLD COLOR_CYAN "MPU6050 TEST" COLOR_RESET "\r\n");
  MPU6050_TestPreInit();

  status = MPU6050_Init(&hi2c1);
  if(status != MPU6050_OK)
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
	  MPU6050_TestSuite();
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
  hi2c1.Init.ClockSpeed = 400000;
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

/* Rellena la muestra con el centinela para detectar campos que Read no escribió */
static void MPU6050_Centinela(MPU_Muestra_t* m)
{
	m->a.x = m->a.y = m->a.z = MPU_SENTINEL;
	m->g.x = m->g.y = m->g.z = MPU_SENTINEL;
	m->t = MPU_SENTINEL;
}

/* Lectura completa: las tres llamadas deben ir bien, escribir todos los campos
 * (el centinela está fuera de escala) y dar una temperatura plausible */
static void MPU6050_Leer(MPU_Muestra_t* m)
{
	MPU6050_Centinela(m);
	MPU_CHECK(MPU6050_Read_Accel(&m->a));
	MPU_CHECK(MPU6050_Read_Gyro(&m->g));
	MPU_CHECK(MPU6050_Read_Temp(&m->t));

	if (m->a.x == MPU_SENTINEL || m->a.y == MPU_SENTINEL || m->a.z == MPU_SENTINEL ||
		m->g.x == MPU_SENTINEL || m->g.y == MPU_SENTINEL || m->g.z == MPU_SENTINEL ||
		m->t < MPU_TEMP_MIN || m->t > MPU_TEMP_MAX)
	{
		Error_Handler();
	}
}

/* Placa quieta: solo actúa la gravedad (|a| ~ 1 g en cualquier orientación) y no hay giro.
 * Detecta factores de escala o bytes mal combinados que un simple rango no vería */
static void MPU6050_CheckReposo(const MPU_Muestra_t* m)
{
	const float mod2 = m->a.x * m->a.x + m->a.y * m->a.y + m->a.z * m->a.z;

	if (mod2 < MPU_ACCEL_MOD_MIN * MPU_ACCEL_MOD_MIN || mod2 > MPU_ACCEL_MOD_MAX * MPU_ACCEL_MOD_MAX ||
		m->g.x < -MPU_GYRO_REPOSO || m->g.x > MPU_GYRO_REPOSO ||
		m->g.y < -MPU_GYRO_REPOSO || m->g.y > MPU_GYRO_REPOSO ||
		m->g.z < -MPU_GYRO_REPOSO || m->g.z > MPU_GYRO_REPOSO)
	{
		Error_Handler();
	}
}

/* Tras un Read rechazado la muestra debe seguir intacta */
static void MPU6050_CheckSinTocar(const MPU_Muestra_t* m)
{
	if (m->a.x != MPU_SENTINEL || m->a.y != MPU_SENTINEL || m->a.z != MPU_SENTINEL ||
		m->g.x != MPU_SENTINEL || m->g.y != MPU_SENTINEL || m->g.z != MPU_SENTINEL ||
		m->t != MPU_SENTINEL)
	{
		Error_Handler();
	}
}

/* Dormido el sensor deja de muestrear: dos lecturas separadas deben ser idénticas */
static void MPU6050_CheckDormido(void)
{
	MPU_Muestra_t m1, m2;

	HAL_Delay(50);
	MPU6050_Leer(&m1);
	HAL_Delay(100);
	MPU6050_Leer(&m2);
	if (memcmp(&m1, &m2, sizeof(m1)) != 0)
	{
		Error_Handler();
	}
	MPU6050_Print(&m1);
}

/* Despierto, el ruido del sensor hace que dos lecturas separadas nunca coincidan */
static void MPU6050_CheckDespierto(void)
{
	MPU_Muestra_t m1, m2;

	HAL_Delay(100);    /* Arranque del giroscopio tras salir de sleep (~30 ms) */
	MPU6050_Leer(&m1);
	HAL_Delay(50);
	MPU6050_Leer(&m2);
	if (memcmp(&m1, &m2, sizeof(m1)) == 0)
	{
		Error_Handler();
	}
	MPU6050_Print(&m2);
}

/* El proyecto no enlaza -u _printf_float: se imprime en centésimas con enteros */
static void MPU6050_PrintVal(const char* nombre, float v)
{
	const int32_t c = (int32_t)(v * 100.0f);
	printf("%s%s%ld.%02ld ", nombre, (c < 0) ? "-" : "", labs(c) / 100, labs(c) % 100);
}

static void MPU6050_Print(const MPU_Muestra_t* m)
{
	printf("A[g] ");
	MPU6050_PrintVal("x=", m->a.x);
	MPU6050_PrintVal("y=", m->a.y);
	MPU6050_PrintVal("z=", m->a.z);
	printf("| G[dps] ");
	MPU6050_PrintVal("x=", m->g.x);
	MPU6050_PrintVal("y=", m->g.y);
	MPU6050_PrintVal("z=", m->g.z);
	printf("| ");
	MPU6050_PrintVal("T=", m->t);
	printf("C\r\n");
}

static void MPU6050_TestSuite(void)
{
	MPU6050_TestLectura();
	HAL_Delay(2000);

	MPU6050_TestMuestras();
	HAL_Delay(2000);

	MPU6050_TestSleep();
	HAL_Delay(2000);

	MPU6050_TestErrores();
	HAL_Delay(2000);

	MPU6050_TestDeInit();
	HAL_Delay(2000);
}

static void MPU6050_TestPreInit(void)
{
	MPU_Muestra_t m;

	/* -- Antes de Init: todo devuelve NOT_INITIALIZED sin tocar el bus ni la muestra -- */
	MPU6050_Centinela(&m);
	MPU_EXPECT(MPU6050_Read_Accel(&m.a), MPU6050_NOT_INITIALIZED);
	MPU_EXPECT(MPU6050_Read_Gyro(&m.g), MPU6050_NOT_INITIALIZED);
	MPU_EXPECT(MPU6050_Read_Temp(&m.t), MPU6050_NOT_INITIALIZED);
	MPU_EXPECT(MPU6050_Set_Sleep_Mode(1), MPU6050_NOT_INITIALIZED);
	MPU6050_CheckSinTocar(&m);

	/* DeInit sobre un driver sin inicializar es inofensivo */
	MPU_CHECK(MPU6050_DeInit());
}

static void MPU6050_TestLectura(void)
{
	MPU_Muestra_t m;

	/* -- Lectura única con la placa QUIETA: |a| ~ 1 g, giro ~ 0 dps y temperatura plausible -- */
	printf(COLOR_YELLOW "[Lectura]" COLOR_RESET "\r\n");
	MPU6050_Leer(&m);
	MPU6050_CheckReposo(&m);
	MPU6050_Print(&m);
}

static void MPU6050_TestMuestras(void)
{
	MPU_Muestra_t m;

	/* -- Lecturas seguidas cada ~0.5 s: ninguna debe fallar. Inclina la placa: el eje que
	 *    apunta hacia abajo marca ~1 g; al girarla el giroscopio marca la velocidad angular -- */
	printf(COLOR_YELLOW "[Muestras]" COLOR_RESET "\r\n");
	for (uint8_t i = 1; i <= MPU_MUESTRAS; i++)
	{
		MPU6050_Leer(&m);
		printf("%2u/%u  ", i, MPU_MUESTRAS);
		MPU6050_Print(&m);
		HAL_Delay(500);
	}
}

static void MPU6050_TestSleep(void)
{
	/* -- Sleep: el I2C sigue respondiendo pero las lecturas se congelan -- */
	printf(COLOR_YELLOW "[Sleep]" COLOR_RESET "\r\n");
	MPU_CHECK(MPU6050_Set_Sleep_Mode(1));
	MPU6050_CheckDormido();

	/* -- Despertar: solo cambia el bit SLEEP, las lecturas vuelven a variar -- */
	MPU_CHECK(MPU6050_Set_Sleep_Mode(0));
	MPU6050_CheckDespierto();
}

static void MPU6050_TestErrores(void)
{
	MPU_Muestra_t m;

	/* -- Punteros de salida NULL y enable fuera de {0, 1} -- */
	printf(COLOR_YELLOW "[Errores]" COLOR_RESET "\r\n");
	MPU_EXPECT(MPU6050_Read_Accel(NULL), MPU6050_INVALID_PARAM);
	MPU_EXPECT(MPU6050_Read_Gyro(NULL), MPU6050_INVALID_PARAM);
	MPU_EXPECT(MPU6050_Read_Temp(NULL), MPU6050_INVALID_PARAM);
	MPU_EXPECT(MPU6050_Set_Sleep_Mode(2), MPU6050_INVALID_PARAM);

	/* -- Init con handle NULL: se rechaza antes de tocar el estado, el driver sigue funcionando -- */
	MPU_EXPECT(MPU6050_Init(NULL), MPU6050_INVALID_PARAM);
	MPU6050_Leer(&m);
	printf(COLOR_GREEN "Errores OK" COLOR_RESET "\r\n");
}

static void MPU6050_TestDeInit(void)
{
	MPU_Muestra_t m;

	/* -- DeInit libera el handle: las llamadas posteriores devuelven NOT_INITIALIZED
	 *    sin tocar el bus ni la muestra. Repetirlo sigue devolviendo OK -- */
	printf(COLOR_YELLOW "[DeInit]" COLOR_RESET "\r\n");
	MPU_CHECK(MPU6050_DeInit());
	MPU_CHECK(MPU6050_DeInit());
	MPU6050_Centinela(&m);
	MPU_EXPECT(MPU6050_Read_Accel(&m.a), MPU6050_NOT_INITIALIZED);
	MPU_EXPECT(MPU6050_Read_Gyro(&m.g), MPU6050_NOT_INITIALIZED);
	MPU_EXPECT(MPU6050_Read_Temp(&m.t), MPU6050_NOT_INITIALIZED);
	MPU_EXPECT(MPU6050_Set_Sleep_Mode(0), MPU6050_NOT_INITIALIZED);
	MPU6050_CheckSinTocar(&m);

	/* -- Re-Init: el driver debe poder arrancar de nuevo tras un DeInit -- */
	MPU_CHECK(MPU6050_Init(&hi2c1));
	MPU6050_Leer(&m);

	/* -- Init sobre un driver ya inicializado y dormido: reescribe PWR_MGMT_1 y lo despierta -- */
	MPU_CHECK(MPU6050_Set_Sleep_Mode(1));
	MPU6050_CheckDormido();
	MPU_CHECK(MPU6050_Init(&hi2c1));
	MPU6050_CheckDespierto();
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
