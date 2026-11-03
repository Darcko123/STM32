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
#include "MAX7219.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Parámetros de la batería de pruebas -- */
#define TEST_NUM_DEV     4U                   /**< Módulos 8x8 en cascada (1 a MAX7219_MAX_DEV) */
#define TEST_WIDTH       (TEST_NUM_DEV * 8U)  /**< Columnas totales de la matriz */
#define TEST_SCROLL_MS   60U                  /**< Retardo entre pasos del scroll */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_CHECK(x)  do { if ((x) != MAX7219_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define MAX_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
MAX7219_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
static void MAX7219_TestSuite(void);
static void MAX7219_TestPrint(void);
static void MAX7219_TestScroll(void);
static void MAX7219_TestPixeles(void);
static void MAX7219_TestBarrido(void);
static void MAX7219_TestSleep(void);
static void MAX7219_TestErrores(void);
static void MAX7219_TestPreInit(void);
static void MAX7219_TestDeInit(void);
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  MAX7219_TestPreInit();

  status = MAX7219_Init(&hspi2, GPIOB, GPIO_PIN_1, TEST_NUM_DEV);
  if (status != MAX7219_OK)
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
	  MAX7219_TestSuite();
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void MAX7219_TestSuite(void)
{
	MAX7219_TestPrint();
	HAL_Delay(2000);

	MAX7219_TestScroll();
	HAL_Delay(2000);

	MAX7219_TestPixeles();
	HAL_Delay(2000);

	MAX7219_TestBarrido();
	HAL_Delay(2000);

	MAX7219_TestSleep();
	HAL_Delay(2000);

	MAX7219_TestErrores();
	HAL_Delay(2000);

	MAX7219_TestDeInit();
	HAL_Delay(2000);
}

static void MAX7219_TestPrint(void)
{
	/* -- Texto estático: cada carácter ocupa 7 columnas, con 4 módulos caben 4 caracteres.
	 *    PrintString no borra las columnas que no escribe: se limpia antes de cada texto -- */
	MAX_CHECK(MAX7219_ClearDisplay());
	MAX_CHECK(MAX7219_PrintString("1234"));
	HAL_Delay(1500);

	MAX_CHECK(MAX7219_ClearDisplay());
	MAX_CHECK(MAX7219_PrintString("Hola"));
	HAL_Delay(1500);

	/* -- Un solo carácter: debe quedar pegado al borde izquierdo y el resto apagado -- */
	MAX_CHECK(MAX7219_ClearDisplay());
	MAX_CHECK(MAX7219_PrintString("A"));
}

static void MAX7219_TestScroll(void)
{
	/* -- Scroll: el texto entra por la derecha y se desplaza hacia la izquierda -- */
	MAX_CHECK(MAX7219_ClearDisplay());
	MAX_CHECK(MAX7219_ScrollString("Matriz de LEDs  ", TEST_SCROLL_MS));
}

static void MAX7219_TestPixeles(void)
{
	/* Con el texto legible, x = 0 es la columna derecha (por donde entra el scroll)
	 * e y = 0 la fila inferior */
	const int w = (int)TEST_WIDTH;

	/* -- Marco: bordes superior/inferior a lo ancho de todos los módulos y laterales -- */
	MAX_CHECK(MAX7219_ClearDisplay());
	for (int x = 0; x < w; x++)
	{
		MAX_CHECK(MAX7219_SetPixel(x, 0, 1));
		MAX_CHECK(MAX7219_SetPixel(x, 7, 1));
	}
	for (int y = 1; y < 7; y++)
	{
		MAX_CHECK(MAX7219_SetPixel(0, y, 1));
		MAX_CHECK(MAX7219_SetPixel(w - 1, y, 1));
	}

	/* -- Una diagonal por módulo: debe verse la misma "\" en cada uno (orden de la cascada) -- */
	for (int dev = 0; dev < (int)TEST_NUM_DEV; dev++)
	{
		for (int i = 1; i < 7; i++)
		{
			MAX_CHECK(MAX7219_SetPixel(dev * 8 + i, i, 1));
		}
	}
	HAL_Delay(1500);

	/* -- Apagado: se borra la diagonal del primer módulo, el resto debe quedar igual -- */
	for (int i = 1; i < 7; i++)
	{
		MAX_CHECK(MAX7219_SetPixel(i, i, 0));
	}

	/* -- Fuera de rango: a diferencia de un recorte, SetPixel lo rechaza -- */
	MAX_EXPECT(MAX7219_SetPixel(-1, 0, 1), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_SetPixel(w, 0, 1), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_SetPixel(0, -1, 1), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_SetPixel(0, 8, 1), MAX7219_INVALID_PARAM);
}

static void MAX7219_TestBarrido(void)
{
	/* -- Barrido de columnas desde x = 0: debe avanzar de derecha a izquierda sin saltos
	 *    al pasar de un módulo al siguiente, y luego apagarse en el mismo orden -- */
	MAX_CHECK(MAX7219_ClearDisplay());
	for (uint8_t pasada = 0; pasada < 2; pasada++)
	{
		const uint8_t state = (pasada == 0) ? 1 : 0;

		for (int x = 0; x < (int)TEST_WIDTH; x++)
		{
			for (int y = 0; y < 8; y++)
			{
				MAX_CHECK(MAX7219_SetPixel(x, y, state));
			}
			HAL_Delay(30);
		}
	}
}

static void MAX7219_TestSleep(void)
{
	/* -- Set_Sleep: la matriz se apaga 1.5 s y al despertar vuelve la misma imagen
	 *    (los registros de dígito se conservan en shutdown, no hace falta redibujar) -- */
	MAX_CHECK(MAX7219_ClearDisplay());
	MAX_CHECK(MAX7219_PrintString("Zzz"));
	HAL_Delay(1000);

	MAX_CHECK(MAX7219_Set_Sleep(1));
	HAL_Delay(1500);
	MAX_CHECK(MAX7219_Set_Sleep(0));
}

static void MAX7219_TestErrores(void)
{
	/* -- Cadenas NULL -- */
	MAX_EXPECT(MAX7219_PrintString(NULL), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_ScrollString(NULL, TEST_SCROLL_MS), MAX7219_INVALID_PARAM);

	/* -- Init con parámetros inválidos: se rechaza antes de tocar el estado del driver -- */
	MAX_EXPECT(MAX7219_Init(NULL, GPIOB, GPIO_PIN_1, TEST_NUM_DEV), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_Init(&hspi2, NULL, GPIO_PIN_1, TEST_NUM_DEV), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_Init(&hspi2, GPIOB, 0, TEST_NUM_DEV), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_Init(&hspi2, GPIOB, GPIO_PIN_1, 0), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_Init(&hspi2, GPIOB, GPIO_PIN_1, MAX7219_MAX_DEV + 1), MAX7219_INVALID_PARAM);

	/* El driver sigue operativo tras los rechazos */
	MAX_CHECK(MAX7219_ClearDisplay());
	MAX_CHECK(MAX7219_PrintString("Err"));
}

static void MAX7219_TestPreInit(void)
{
	/* -- Antes de Init: todo debe devolver NOT_INITIALIZED sin tocar el bus -- */
	MAX_EXPECT(MAX7219_ClearDisplay(), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_PrintString("X"), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_ScrollString("X", TEST_SCROLL_MS), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_SetPixel(0, 0, 1), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_Set_Sleep(1), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_DeInit(), MAX7219_NOT_INITIALIZED);
}

static void MAX7219_TestDeInit(void)
{
	/* -- DeInit apaga la matriz y libera la configuración: las llamadas posteriores
	 *    deben devolver NOT_INITIALIZED sin tocar el bus -- */
	MAX_CHECK(MAX7219_DeInit());
	MAX_EXPECT(MAX7219_DeInit(), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_ClearDisplay(), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_PrintString("X"), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_SetPixel(0, 0, 1), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_Set_Sleep(0), MAX7219_NOT_INITIALIZED);
	HAL_Delay(1500);

	/* -- Re-Init: el driver debe poder arrancar de nuevo tras un DeInit -- */
	MAX_CHECK(MAX7219_Init(&hspi2, GPIOB, GPIO_PIN_1, TEST_NUM_DEV));
	MAX_CHECK(MAX7219_PrintString("OK"));
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
