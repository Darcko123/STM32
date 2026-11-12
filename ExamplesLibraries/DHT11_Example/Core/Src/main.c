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
#include "DHT11.h"
#include "ANSII_Codes.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Conexión del sensor -- */
#define DHT_PUERTO          GPIOB
#define DHT_PIN             GPIO_PIN_7     /**< Pin de datos del DHT11 */
#define DHT_PIN_LIBRE       GPIO_PIN_14     /**< Pin SIN conectar: el test de timeout lo usa como "sensor ausente" */

/* -- Parámetros de la batería de pruebas -- */
#define DHT_ARRANQUE_MS     1000U      /**< El DHT11 necesita ~1 s tras alimentarse antes de la primera lectura */
#define DHT_PERIODO_MS      DHT11_MIN_INTERVAL_MS  /**< Entre lecturas: por debajo, Get devuelve la caché sin tocar el bus */
#define DHT_MUESTRAS        10U        /**< Lecturas seguidas del test de muestras */
#define DHT_SENTINEL        1000.0f    /**< Fuera de escala: delata campos que Get no escribió */

/* Rango de funcionamiento del DHT11 según datasheet (temperatura: DHT11 nuevos con bit de signo) */
#define DHT_TEMP_MIN        -20.0f
#define DHT_TEMP_MAX        60.0f
#define DHT_HUM_MIN         20.0f
#define DHT_HUM_MAX         90.0f

/* Variación máxima entre dos lecturas separadas DHT_PERIODO_MS (el ambiente cambia despacio);
 * subir si la habitación tiene corrientes de aire o se sopla sobre el sensor */
#define DHT_TEMP_RUIDO      2.0f       /**< [°C] */
#define DHT_HUM_RUIDO       5.0f       /**< [%HR] */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DHT_CHECK(x)  do { if ((x) != DHT11_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define DHT_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
DHT11_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void DHT11_Centinela(DHT11_Data_t* m);
static void DHT11_Leer(DHT11_Data_t* m);
static void DHT11_CheckSinTocar(const DHT11_Data_t* m);
static void DHT11_Print(const DHT11_Data_t* m);
static void DHT11_TestSuite(void);
static void DHT11_TestPreInit(void);
static void DHT11_TestLectura(void);
static void DHT11_TestMuestras(void);
static void DHT11_TestErrores(void);
static void DHT11_TestTimeout(void);
static void DHT11_TestDeInit(void);
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  printf(CLEAR_SCREEN CURSOR_HOME COLOR_BOLD COLOR_CYAN "DHT11 TEST" COLOR_RESET "\r\n");
  DHT11_TestPreInit();

  /* La librería arranca el TIM2 (1 MHz, 1 us por cuenta) si aún no está en marcha */
  status = DHT11_Init(&htim2, DHT_PUERTO, DHT_PIN);
  if(status != DHT11_OK)
  {
	  Error_Handler();
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  HAL_Delay(DHT_ARRANQUE_MS);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  DHT11_TestSuite();
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
static void DHT11_Centinela(DHT11_Data_t* m)
{
	m->temperatura = DHT_SENTINEL;
	m->humedad    = DHT_SENTINEL;
}

/* Lectura completa: debe ir bien, escribir los dos campos y dar valores dentro del rango del
 * sensor. Se escribe como !(v >= min && v <= max) para que un NaN también falle */
static void DHT11_Leer(DHT11_Data_t* m)
{
	DHT11_Centinela(m);
	DHT_CHECK(DHT11_Get(m));

	if (!(m->temperatura >= DHT_TEMP_MIN && m->temperatura <= DHT_TEMP_MAX) ||
		!(m->humedad    >= DHT_HUM_MIN  && m->humedad    <= DHT_HUM_MAX))
	{
		Error_Handler();
	}
}

/* Tras un Get rechazado la muestra debe seguir intacta */
static void DHT11_CheckSinTocar(const DHT11_Data_t* m)
{
	if (m->temperatura != DHT_SENTINEL || m->humedad != DHT_SENTINEL)
	{
		Error_Handler();
	}
}

/* El proyecto no enlaza -u _printf_float: se imprime en décimas con enteros
 * (el DHT11 resuelve 0.1; la temperatura puede ser negativa, la humedad no) */
static void DHT11_Print(const DHT11_Data_t* m)
{
	const uint32_t t = (uint32_t)(fabsf(m->temperatura) * 10.0f + 0.5f);
	const uint32_t h = (uint32_t)(m->humedad * 10.0f + 0.5f);

	printf("T=%s%lu.%lu C  H=%lu.%lu %%\r\n", (m->temperatura < 0.0f) ? "-" : "",
		   t / 10, t % 10, h / 10, h % 10);
}

static void DHT11_TestSuite(void)
{
	DHT11_TestLectura();
	HAL_Delay(DHT_PERIODO_MS);

	DHT11_TestMuestras();
	HAL_Delay(DHT_PERIODO_MS);

	DHT11_TestErrores();
	HAL_Delay(DHT_PERIODO_MS);

	DHT11_TestTimeout();
	HAL_Delay(DHT_PERIODO_MS);

	DHT11_TestDeInit();
	HAL_Delay(DHT_PERIODO_MS);
}

static void DHT11_TestPreInit(void)
{
	DHT11_Data_t m;

	/* -- Antes de Init: Get devuelve NOT_INITIALIZED sin tocar el bus ni la muestra
	 *    (aunque el puntero sea NULL: la comprobación de inicialización va primero) -- */
	DHT11_Centinela(&m);
	DHT_EXPECT(DHT11_Get(&m), DHT11_NOT_INITIALIZED);
	DHT_EXPECT(DHT11_Get(NULL), DHT11_NOT_INITIALIZED);
	DHT11_CheckSinTocar(&m);
}

static void DHT11_TestLectura(void)
{
	DHT11_Data_t m;

	/* -- Lectura única: temperatura y humedad dentro del rango del DHT11 -- */
	printf(COLOR_YELLOW "[Lectura]" COLOR_RESET "\r\n");
	DHT11_Leer(&m);
	DHT11_Print(&m);

	/* -- Get inmediato (< DHT11_MIN_INTERVAL_MS): no toca el bus, devuelve al instante
	 *    la misma medición guardada -- */
	DHT11_Data_t r;
	const uint32_t t0 = HAL_GetTick();
	DHT11_Leer(&r);
	if (r.temperatura != m.temperatura || r.humedad != m.humedad || HAL_GetTick() - t0 > 1U)
	{
		Error_Handler();
	}
}

static void DHT11_TestMuestras(void)
{
	DHT11_Data_t m, ant = {0};

	/* -- Lecturas seguidas cada DHT_PERIODO_MS: ninguna debe fallar (checksum incluido) y
	 *    entre dos consecutivas el ambiente no puede haber cambiado más que el ruido tolerado.
	 *    Un soplido sobre el sensor sube la humedad: no lo hagas durante el test -- */
	printf(COLOR_YELLOW "[Muestras]" COLOR_RESET "\r\n");
	for (uint8_t i = 1; i <= DHT_MUESTRAS; i++)
	{
		DHT11_Leer(&m);
		if (i > 1 && (fabsf(m.temperatura - ant.temperatura) > DHT_TEMP_RUIDO ||
					  fabsf(m.humedad - ant.humedad) > DHT_HUM_RUIDO))
		{
			Error_Handler();
		}
		printf("%2u/%u  ", i, DHT_MUESTRAS);
		DHT11_Print(&m);
		ant = m;
		HAL_Delay(DHT_PERIODO_MS);
	}
}

static void DHT11_TestErrores(void)
{
	DHT11_Data_t m;
	TIM_HandleTypeDef sin_instancia = {0};

	/* -- Puntero de salida NULL, y Init con timer, instancia, puerto NULL o pin 0: se rechazan
	 *    con INVALID_PARAM sin tocar el bus ni la configuración vigente -- */
	printf(COLOR_YELLOW "[Errores]" COLOR_RESET "\r\n");
	DHT11_Centinela(&m);
	DHT_EXPECT(DHT11_Get(NULL), DHT11_INVALID_PARAM);
	DHT_EXPECT(DHT11_Init(NULL, DHT_PUERTO, DHT_PIN), DHT11_INVALID_PARAM);
	DHT_EXPECT(DHT11_Init(&sin_instancia, DHT_PUERTO, DHT_PIN), DHT11_INVALID_PARAM);
	DHT_EXPECT(DHT11_Init(&htim2, NULL, DHT_PIN), DHT11_INVALID_PARAM);
	DHT_EXPECT(DHT11_Init(&htim2, DHT_PUERTO, 0U), DHT11_INVALID_PARAM);
	DHT11_CheckSinTocar(&m);

	/* El driver conserva la configuración anterior y sigue leyendo */
	DHT11_Leer(&m);
	printf(COLOR_GREEN "Errores OK" COLOR_RESET "\r\n");
}

static void DHT11_TestTimeout(void)
{
	DHT11_Data_t m;

	/* -- Sensor ausente: apuntando a un pin libre (pull-up, siempre en alto) el sensor nunca
	 *    responde y Get debe agotar el tiempo con TIMEOUT sin escribir la muestra.
	 *    También prueba que Init descarta la caché: si no, Get devolvería OK con la medición
	 *    del pin anterior -- */
	printf(COLOR_YELLOW "[Timeout]" COLOR_RESET "\r\n");
	DHT11_Centinela(&m);
	DHT_CHECK(DHT11_Init(&htim2, DHT_PUERTO, DHT_PIN_LIBRE));
	DHT_EXPECT(DHT11_Get(&m), DHT11_TIMEOUT);
	DHT11_CheckSinTocar(&m);

	/* -- Volver al pin real: Init reconfigura el driver y la lectura vuelve a funcionar -- */
	DHT_CHECK(DHT11_Init(&htim2, DHT_PUERTO, DHT_PIN));
	DHT11_Leer(&m);
	printf(COLOR_GREEN "Timeout OK" COLOR_RESET "\r\n");
}

static void DHT11_TestDeInit(void)
{
	DHT11_Data_t m;

	/* -- DeInit libera el driver: Get debe devolver NOT_INITIALIZED sin tocar el bus
	 *    ni la muestra. Repetirlo sigue devolviendo OK -- */
	printf(COLOR_YELLOW "[DeInit]" COLOR_RESET "\r\n");
	DHT11_Centinela(&m);
	DHT_CHECK(DHT11_DeInit());
	DHT_CHECK(DHT11_DeInit());
	DHT_EXPECT(DHT11_Get(&m), DHT11_NOT_INITIALIZED);
	DHT11_CheckSinTocar(&m);

	/* -- Re-Init tras DeInit: el driver debe poder arrancar de nuevo -- */
	DHT_CHECK(DHT11_Init(&htim2, DHT_PUERTO, DHT_PIN));
	DHT11_Leer(&m);
	HAL_Delay(DHT_PERIODO_MS);

	/* -- Init sobre un driver ya inicializado (el timer sigue en marcha) también se acepta -- */
	DHT_CHECK(DHT11_Init(&htim2, DHT_PUERTO, DHT_PIN));
	DHT11_Leer(&m);
	DHT11_Print(&m);
	printf(COLOR_GREEN "DeInit OK" COLOR_RESET "\r\n");
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
