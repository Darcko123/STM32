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
#include "BMP280.h"
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
#define BMP_MUESTRAS        10U        /**< Lecturas seguidas del test de muestras */
#define BMP_SENTINEL        (-1000.0f) /**< Fuera de rango en los tres campos: delata campos que Get no escribió */

/* Rango de funcionamiento del sensor (datasheet) */
#define BMP_TEMP_MIN        (-40.0f)
#define BMP_TEMP_MAX        (85.0f)
#define BMP_PRES_MIN        (300.0f)   /**< [hPa] */
#define BMP_PRES_MAX        (1100.0f)  /**< [hPa] */
/* Altitud que da la fórmula de la librería en ese rango de presión (-698 m .. 9164 m) */
#define BMP_ALT_MIN         (-750.0f)  /**< [m] */
#define BMP_ALT_MAX         (9200.0f)  /**< [m] */

/* Con la placa quieta, variación máxima entre muestras (filtro IIR x16 activo).
 * Ajustar si falla sin tocarla; soplarle o tocarla con el dedo debe hacerlo fallar */
#define BMP_PRES_RUIDO      0.5f       /**< [hPa] */
#define BMP_TEMP_RUIDO      0.5f       /**< [°C] */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BMP_CHECK(x)  do { if ((x) != BMP280_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define BMP_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
BMP280_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void BMP280_Centinela(BMP280_Data_t* d);
static void BMP280_Leer(BMP280_Data_t* d);
static void BMP280_CheckSinTocar(const BMP280_Data_t* d);
static void BMP280_PrintVal(const char* nombre, float v, const char* unidad);
static void BMP280_Print(const BMP280_Data_t* d);
static void BMP280_TestSuite(void);
static void BMP280_TestPreInit(void);
static void BMP280_TestLectura(void);
static void BMP280_TestMuestras(void);
static void BMP280_TestErrores(void);
static void BMP280_TestReInit(void);
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
  printf(CLEAR_SCREEN CURSOR_HOME COLOR_BOLD COLOR_CYAN "BMP280 TEST" COLOR_RESET "\r\n");
  BMP280_TestPreInit();

  status = BMP280_Init(&hi2c1);
  if(status != BMP280_OK)
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
	  BMP280_TestSuite();
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

/* Rellena la muestra con el centinela para detectar campos que Get no escribió */
static void BMP280_Centinela(BMP280_Data_t* d)
{
	d->temperatura = d->presion = d->altitud = BMP_SENTINEL;
}

/* Lectura completa: Get debe ir bien y los tres campos quedar dentro del rango del
 * sensor (el centinela está fuera, así que también detecta campos sin escribir) */
static void BMP280_Leer(BMP280_Data_t* d)
{
	BMP280_Centinela(d);
	BMP_CHECK(BMP280_Get(d));

	if (d->temperatura < BMP_TEMP_MIN || d->temperatura > BMP_TEMP_MAX ||
		d->presion     < BMP_PRES_MIN || d->presion     > BMP_PRES_MAX ||
		d->altitud     < BMP_ALT_MIN  || d->altitud     > BMP_ALT_MAX)
	{
		Error_Handler();
	}
}

/* Tras un Get rechazado la muestra debe seguir intacta */
static void BMP280_CheckSinTocar(const BMP280_Data_t* d)
{
	if (d->temperatura != BMP_SENTINEL || d->presion != BMP_SENTINEL || d->altitud != BMP_SENTINEL)
	{
		Error_Handler();
	}
}

/* El proyecto no enlaza -u _printf_float: se imprime en centésimas con enteros */
static void BMP280_PrintVal(const char* nombre, float v, const char* unidad)
{
	const int32_t c = (int32_t)(v * 100.0f);
	printf("%s%s%ld.%02ld %s", nombre, (c < 0) ? "-" : "", labs(c) / 100, labs(c) % 100, unidad);
}

static void BMP280_Print(const BMP280_Data_t* d)
{
	BMP280_PrintVal("T=", d->temperatura, "C | ");
	BMP280_PrintVal("P=", d->presion, "hPa | ");
	BMP280_PrintVal("Alt=", d->altitud, "m\r\n");
}

static void BMP280_TestSuite(void)
{
	BMP280_TestLectura();
	HAL_Delay(2000);

	BMP280_TestMuestras();
	HAL_Delay(2000);

	BMP280_TestErrores();
	HAL_Delay(2000);

	BMP280_TestReInit();
	HAL_Delay(2000);
}

static void BMP280_TestPreInit(void)
{
	BMP280_Data_t d;

	/* -- Antes de Init: Get devuelve NOT_INITIALIZED sin tocar el bus ni la muestra.
	 *    El puntero NULL se comprueba antes que el estado: INVALID_PARAM -- */
	BMP280_Centinela(&d);
	BMP_EXPECT(BMP280_Get(&d), BMP280_NOT_INITIALIZED);
	BMP280_CheckSinTocar(&d);
	BMP_EXPECT(BMP280_Get(NULL), BMP280_INVALID_PARAM);
}

static void BMP280_TestLectura(void)
{
	BMP280_Data_t d;

	/* -- Lectura única: temperatura, presión y altitud dentro del rango del sensor -- */
	printf(COLOR_YELLOW "[Lectura]" COLOR_RESET "\r\n");
	BMP280_Leer(&d);
	BMP280_Print(&d);
}

static void BMP280_TestMuestras(void)
{
	BMP280_Data_t d;
	float pmin = 0.0f, pmax = 0.0f, tmin = 0.0f, tmax = 0.0f;

	/* -- Lecturas seguidas cada ~0.5 s con la placa QUIETA: ninguna debe fallar.
	 *    La presión debe variar (el sensor está en modo normal, midiendo) pero poco:
	 *    un salto mayor que el ruido delata bytes mal combinados o la compensación -- */
	printf(COLOR_YELLOW "[Muestras]" COLOR_RESET "\r\n");
	for (uint8_t i = 1; i <= BMP_MUESTRAS; i++)
	{
		BMP280_Leer(&d);
		if (i == 1U)
		{
			pmin = pmax = d.presion;
			tmin = tmax = d.temperatura;
		}
		pmin = fminf(pmin, d.presion);
		pmax = fmaxf(pmax, d.presion);
		tmin = fminf(tmin, d.temperatura);
		tmax = fmaxf(tmax, d.temperatura);

		printf("%2u/%u  ", i, BMP_MUESTRAS);
		BMP280_Print(&d);
		HAL_Delay(500);
	}

	BMP280_PrintVal("dP=", pmax - pmin, "hPa | ");
	BMP280_PrintVal("dT=", tmax - tmin, "C\r\n");
	if (pmax == pmin || pmax - pmin > BMP_PRES_RUIDO || tmax - tmin > BMP_TEMP_RUIDO)
	{
		Error_Handler();
	}
}

static void BMP280_TestErrores(void)
{
	BMP280_Data_t d;

	/* -- Puntero de salida NULL -- */
	printf(COLOR_YELLOW "[Errores]" COLOR_RESET "\r\n");
	BMP_EXPECT(BMP280_Get(NULL), BMP280_INVALID_PARAM);

	/* -- Init con handle NULL: se rechaza antes de tocar el estado, el driver sigue funcionando -- */
	BMP_EXPECT(BMP280_Init(NULL), BMP280_INVALID_PARAM);
	BMP280_Leer(&d);
	printf(COLOR_GREEN "Errores OK" COLOR_RESET "\r\n");
}

static void BMP280_TestReInit(void)
{
	BMP280_Data_t d;

	/* -- Re-Init: sobre un driver ya inicializado vuelve a detectar el sensor, hace soft
	 *    reset y reconfigura. Las lecturas siguientes deben seguir siendo válidas -- */
	printf(COLOR_YELLOW "[Re-Init]" COLOR_RESET "\r\n");
	BMP_CHECK(BMP280_Init(&hi2c1));
	BMP280_Leer(&d);
	BMP280_Print(&d);
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
