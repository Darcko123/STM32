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
#include "SI7021.h"
#include "ANSII_Codes.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Parámetros de la batería de pruebas -- */
#define SI_MUESTRAS     10U        /**< Lecturas seguidas del test de muestras */
#define SI_SENTINEL     1000.0f    /**< Valor fuera de rango para detectar campos que Get no escribió */

/* Rango plausible: el de funcionamiento del sensor (la fórmula del datasheet puede pasarse un poco de 0-100 %HR) */
#define SI_TEMP_MIN     (-40.0f)
#define SI_TEMP_MAX     (125.0f)
#define SI_HUM_MIN      (-2.0f)
#define SI_HUM_MAX      (102.0f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SI_CHECK(x)  do { if ((x) != SI7021_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define SI_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SI7021_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void SI7021_CheckData(const SI7021_Data_t* d);
static void SI7021_CheckSinTocar(const SI7021_Data_t* d);
static void SI7021_Print(const SI7021_Data_t* d);
static void SI7021_TestSuite(void);
static void SI7021_TestPreInit(void);
static void SI7021_TestLectura(void);
static void SI7021_TestMuestras(void);
static void SI7021_TestErrores(void);
static void SI7021_TestDeInit(void);
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
  printf(CLEAR_SCREEN CURSOR_HOME COLOR_BOLD COLOR_CYAN "SI7021 TEST" COLOR_RESET "\r\n");
  SI7021_TestPreInit();

  status = SI7021_Init(&hi2c1);
  if(status != SI7021_OK)
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
	  SI7021_TestSuite();
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

/* La lectura debe caer en el rango del sensor; el centinela (fuera de rango) delata un campo sin escribir */
static void SI7021_CheckData(const SI7021_Data_t* d)
{
	if (d->temperatura < SI_TEMP_MIN || d->temperatura > SI_TEMP_MAX ||
		d->humedad < SI_HUM_MIN || d->humedad > SI_HUM_MAX)
	{
		Error_Handler();
	}
}

/* Tras un Get rechazado la estructura debe seguir intacta */
static void SI7021_CheckSinTocar(const SI7021_Data_t* d)
{
	if (d->temperatura != SI_SENTINEL || d->humedad != SI_SENTINEL)
	{
		Error_Handler();
	}
}

/* El proyecto no enlaza -u _printf_float: se imprime en centésimas con enteros */
static void SI7021_Print(const SI7021_Data_t* d)
{
	const int32_t t = (int32_t)(d->temperatura * 100.0f);
	const int32_t h = (int32_t)(d->humedad * 100.0f);

	printf("T = %s%ld.%02ld C | HR = %s%ld.%02ld %%\r\n",
		   (t < 0) ? "-" : "", labs(t) / 100, labs(t) % 100,
		   (h < 0) ? "-" : "", labs(h) / 100, labs(h) % 100);
}

static void SI7021_TestSuite(void)
{
	SI7021_TestLectura();
	HAL_Delay(2000);

	SI7021_TestMuestras();
	HAL_Delay(2000);

	SI7021_TestErrores();
	HAL_Delay(2000);

	SI7021_TestDeInit();
	HAL_Delay(2000);
}

static void SI7021_TestPreInit(void)
{
	SI7021_Data_t data = { SI_SENTINEL, SI_SENTINEL };

	/* -- Antes de Init: Get devuelve NOT_INITIALIZED sin tocar el bus ni la estructura -- */
	SI_EXPECT(SI7021_Get(&data), SI7021_NOT_INITIALIZED);
	SI7021_CheckSinTocar(&data);

	/* DeInit sobre un driver sin inicializar es inofensivo */
	SI_CHECK(SI7021_DeInit());
}

static void SI7021_TestLectura(void)
{
	SI7021_Data_t data = { SI_SENTINEL, SI_SENTINEL };

	/* -- Lectura única: Get rellena temperatura y humedad con valores plausibles -- */
	printf(COLOR_YELLOW "[Lectura]" COLOR_RESET "\r\n");
	SI_CHECK(SI7021_Get(&data));
	SI7021_CheckData(&data);
	SI7021_Print(&data);
}

static void SI7021_TestMuestras(void)
{
	SI7021_Data_t data;

	/* -- Lecturas seguidas cada ~0.5 s: ninguna debe fallar ni salirse de rango.
	 *    Si soplas o acercas un dedo al sensor, la humedad y la temperatura deben subir -- */
	printf(COLOR_YELLOW "[Muestras]" COLOR_RESET "\r\n");
	for (uint8_t i = 1; i <= SI_MUESTRAS; i++)
	{
		data.temperatura = SI_SENTINEL;
		data.humedad     = SI_SENTINEL;
		SI_CHECK(SI7021_Get(&data));
		SI7021_CheckData(&data);
		printf("%2u/%u  ", i, SI_MUESTRAS);
		SI7021_Print(&data);
		HAL_Delay(500);
	}
}

static void SI7021_TestErrores(void)
{
	SI7021_Data_t data = { SI_SENTINEL, SI_SENTINEL };

	/* -- Get con puntero NULL -- */
	printf(COLOR_YELLOW "[Errores]" COLOR_RESET "\r\n");
	SI_EXPECT(SI7021_Get(NULL), SI7021_INVALID_PARAM);

	/* -- Init con handle NULL: se rechaza antes de tocar el estado, el driver sigue funcionando -- */
	SI_EXPECT(SI7021_Init(NULL), SI7021_INVALID_PARAM);
	SI_CHECK(SI7021_Get(&data));
	SI7021_CheckData(&data);
	printf(COLOR_GREEN "Errores OK" COLOR_RESET "\r\n");
}

static void SI7021_TestDeInit(void)
{
	SI7021_Data_t data = { SI_SENTINEL, SI_SENTINEL };

	/* -- DeInit libera el handle: las llamadas posteriores deben devolver NOT_INITIALIZED
	 *    sin tocar el bus ni la estructura. Repetirlo sigue devolviendo OK -- */
	printf(COLOR_YELLOW "[DeInit]" COLOR_RESET "\r\n");
	SI_CHECK(SI7021_DeInit());
	SI_CHECK(SI7021_DeInit());
	SI_EXPECT(SI7021_Get(&data), SI7021_NOT_INITIALIZED);
	SI7021_CheckSinTocar(&data);

	/* -- Re-Init: el driver debe poder arrancar de nuevo tras un DeInit -- */
	SI_CHECK(SI7021_Init(&hi2c1));
	SI_CHECK(SI7021_Get(&data));
	SI7021_CheckData(&data);

	/* -- Init sobre un driver ya inicializado: reenvía el reset y sigue funcionando -- */
	data.temperatura = SI_SENTINEL;
	data.humedad     = SI_SENTINEL;
	SI_CHECK(SI7021_Init(&hi2c1));
	SI_CHECK(SI7021_Get(&data));
	SI7021_CheckData(&data);
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
