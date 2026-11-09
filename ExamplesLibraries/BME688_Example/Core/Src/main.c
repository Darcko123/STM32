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
#include "BME688.h"
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
#define BME_MUESTRAS        10U        /**< Lecturas seguidas del test de muestras */
#define BME_PERIODO_MS      1000U      /**< Entre muestras: el calentador a 300 °C autocalienta el sensor */
#define BME_SENTINEL        (-1000.0f) /**< Fuera de rango en todos los campos: delata campos que Get no escribió */

/* Rango de funcionamiento del sensor (datasheet) */
#define BME_TEMP_MIN        (-40.0f)
#define BME_TEMP_MAX        (85.0f)
#define BME_PRES_MIN        (300.0f)   /**< [hPa] */
#define BME_PRES_MAX        (1100.0f)  /**< [hPa] */
#define BME_HUM_MIN         (0.0f)     /**< [%RH] la librería acota a 0-100 */
#define BME_HUM_MAX         (100.0f)
/* Resistencia de gas: la fórmula del BME688 da 1.4 kOhm .. 102 MOhm, fuera de eso hay bytes mal leídos */
#define BME_GAS_MIN         (1.0e3f)   /**< [Ohm] */
#define BME_GAS_MAX         (10.1e8f)   /**< [Ohm] */

/* Con la placa quieta, variación máxima entre muestras (forced mode, filtro IIR off).
 * La temperatura sube algo por el calentador: ajustar si falla sin tocarla */
#define BME_PRES_RUIDO      0.5f       /**< [hPa] */
#define BME_TEMP_RUIDO      1.0f       /**< [°C] */
#define BME_HUM_RUIDO       3.0f       /**< [%RH] */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BME_CHECK(x)  do { if ((x) != BME688_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define BME_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)
/* Dentro de [min, max]; escrito así para que un NaN también falle */
#define BME_EN_RANGO(v, min, max)  ((v) >= (min) && (v) <= (max))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
BME688_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void BME688_Centinela(BME688_Data_t* d);
static void BME688_Leer(BME688_Data_t* d);
static void BME688_CheckSinTocar(const BME688_Data_t* d);
static void BME688_PrintVal(const char* nombre, float v, const char* unidad);
static void BME688_Print(const BME688_Data_t* d);
static void BME688_TestSuite(void);
static void BME688_TestPreInit(void);
static void BME688_TestLectura(void);
static void BME688_TestMuestras(void);
static void BME688_TestSleep(void);
static void BME688_TestErrores(void);
static void BME688_TestDeInit(void);
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
  printf(CLEAR_SCREEN CURSOR_HOME COLOR_BOLD COLOR_CYAN "BME688 TEST" COLOR_RESET "\r\n");
  BME688_TestPreInit();

  status = BME688_Init(&hi2c1);
  if(status != BME688_OK)
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
	  BME688_TestSuite();
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
static void BME688_Centinela(BME688_Data_t* d)
{
	d->temperatura = d->presion = d->humedad = d->gas = BME_SENTINEL;
	d->gas_valido  = 0xFFU;
}

/* Lectura completa: Get debe ir bien, los cuatro campos quedar dentro del rango del
 * sensor (el centinela está fuera) y el calentador haberse estabilizado */
static void BME688_Leer(BME688_Data_t* d)
{
	BME688_Centinela(d);
	BME_CHECK(BME688_Get(d));

	if (!BME_EN_RANGO(d->temperatura, BME_TEMP_MIN, BME_TEMP_MAX) ||
		!BME_EN_RANGO(d->presion,     BME_PRES_MIN, BME_PRES_MAX) ||
		!BME_EN_RANGO(d->humedad,     BME_HUM_MIN,  BME_HUM_MAX)  ||
		!BME_EN_RANGO(d->gas,         BME_GAS_MIN,  BME_GAS_MAX)  ||
		d->gas_valido != 1U)
	{
		BME688_Print(d);
		Error_Handler();
	}
}

/* Tras un Get rechazado la muestra debe seguir intacta */
static void BME688_CheckSinTocar(const BME688_Data_t* d)
{
	if (d->temperatura != BME_SENTINEL || d->presion != BME_SENTINEL ||
		d->humedad != BME_SENTINEL || d->gas != BME_SENTINEL || d->gas_valido != 0xFFU)
	{
		Error_Handler();
	}
}

/* El proyecto no enlaza -u _printf_float: se imprime en centésimas con enteros */
static void BME688_PrintVal(const char* nombre, float v, const char* unidad)
{
	const int32_t c = (int32_t)(v * 100.0f);
	printf("%s%s%ld.%02ld %s", nombre, (c < 0) ? "-" : "", labs(c) / 100, labs(c) % 100, unidad);
}

static void BME688_Print(const BME688_Data_t* d)
{
	BME688_PrintVal("T=", d->temperatura, "C | ");
	BME688_PrintVal("P=", d->presion, "hPa | ");
	BME688_PrintVal("H=", d->humedad, "%RH | ");
	/* En kOhm: en Ohm las centésimas desbordan int32 por encima de ~21 MOhm */
	BME688_PrintVal("Gas=", d->gas / 1000.0f, "kOhm");
	printf("%s\r\n", (d->gas_valido == 1U) ? "" : " (no valido)");
}

static void BME688_TestSuite(void)
{
	BME688_TestLectura();
	HAL_Delay(2000);

	BME688_TestMuestras();
	HAL_Delay(2000);

	BME688_TestSleep();
	HAL_Delay(2000);

	BME688_TestErrores();
	HAL_Delay(2000);

	BME688_TestDeInit();
	HAL_Delay(2000);
}

static void BME688_TestPreInit(void)
{
	BME688_Data_t d;

	/* -- Antes de Init: todo devuelve NOT_INITIALIZED sin tocar el bus ni la muestra.
	 *    El puntero NULL se comprueba antes que el estado: INVALID_PARAM -- */
	BME688_Centinela(&d);
	BME_EXPECT(BME688_Get(&d), BME688_NOT_INITIALIZED);
	BME688_CheckSinTocar(&d);
	BME_EXPECT(BME688_Get(NULL), BME688_INVALID_PARAM);
	BME_EXPECT(BME688_Sleep(), BME688_NOT_INITIALIZED);
	BME_EXPECT(BME688_DeInit(), BME688_NOT_INITIALIZED);
}

static void BME688_TestLectura(void)
{
	BME688_Data_t d;

	/* -- Lectura única: T, P, H y gas dentro del rango del sensor y gas válido -- */
	printf(COLOR_YELLOW "[Lectura]" COLOR_RESET "\r\n");
	BME688_Leer(&d);
	BME688_Print(&d);
}

static void BME688_TestMuestras(void)
{
	BME688_Data_t d, anterior;
	float pmin = 0.0f, pmax = 0.0f, tmin = 0.0f, tmax = 0.0f, hmin = 0.0f, hmax = 0.0f;
	uint8_t cambios = 0U;

	/* -- Lecturas seguidas cada BME_PERIODO_MS con la placa QUIETA: ninguna debe fallar.
	 *    Cada Get dispara una medición forced nueva, así que las muestras deben cambiar
	 *    (al menos el gas), pero poco: un salto mayor que el ruido delata bytes mal
	 *    combinados o la compensación. Soplarle debe subir la humedad y bajar el gas -- */
	printf(COLOR_YELLOW "[Muestras]" COLOR_RESET "\r\n");
	for (uint8_t i = 1; i <= BME_MUESTRAS; i++)
	{
		BME688_Leer(&d);
		if (i == 1U)
		{
			pmin = pmax = d.presion;
			tmin = tmax = d.temperatura;
			hmin = hmax = d.humedad;
		}
		else if (d.temperatura != anterior.temperatura || d.presion != anterior.presion ||
				 d.humedad != anterior.humedad || d.gas != anterior.gas)
		{
			cambios++;
		}
		anterior = d;

		pmin = fminf(pmin, d.presion);
		pmax = fmaxf(pmax, d.presion);
		tmin = fminf(tmin, d.temperatura);
		tmax = fmaxf(tmax, d.temperatura);
		hmin = fminf(hmin, d.humedad);
		hmax = fmaxf(hmax, d.humedad);

		printf("%2u/%u  ", i, BME_MUESTRAS);
		BME688_Print(&d);
		HAL_Delay(BME_PERIODO_MS);
	}

	BME688_PrintVal("dP=", pmax - pmin, "hPa | ");
	BME688_PrintVal("dT=", tmax - tmin, "C | ");
	BME688_PrintVal("dH=", hmax - hmin, "%RH\r\n");
	if (cambios == 0U || pmax - pmin > BME_PRES_RUIDO ||
		tmax - tmin > BME_TEMP_RUIDO || hmax - hmin > BME_HUM_RUIDO)
	{
		Error_Handler();
	}
}

static void BME688_TestSleep(void)
{
	BME688_Data_t d;

	/* -- Sleep: tras cada Get el sensor ya vuelve solo a sleep, así que pedirlo (y repetirlo)
	 *    debe devolver OK sin esperar -- */
	printf(COLOR_YELLOW "[Sleep]" COLOR_RESET "\r\n");
	BME_CHECK(BME688_Sleep());
	BME_CHECK(BME688_Sleep());

	/* -- Despertar: Get lo saca de sleep sin Init, conserva configuración y calibración -- */
	BME688_Leer(&d);
	BME688_Print(&d);
	printf(COLOR_GREEN "Sleep OK" COLOR_RESET "\r\n");
}

static void BME688_TestErrores(void)
{
	BME688_Data_t d;

	/* -- Puntero de salida NULL -- */
	printf(COLOR_YELLOW "[Errores]" COLOR_RESET "\r\n");
	BME_EXPECT(BME688_Get(NULL), BME688_INVALID_PARAM);

	/* -- Init con handle NULL: se rechaza antes de tocar el estado, el driver sigue funcionando -- */
	BME_EXPECT(BME688_Init(NULL), BME688_INVALID_PARAM);
	BME688_Leer(&d);
	printf(COLOR_GREEN "Errores OK" COLOR_RESET "\r\n");
}

static void BME688_TestDeInit(void)
{
	BME688_Data_t d;

	/* -- DeInit deja el sensor en sleep y libera el handle: las llamadas posteriores
	 *    devuelven NOT_INITIALIZED sin tocar el bus ni la muestra. Repetirlo también -- */
	printf(COLOR_YELLOW "[DeInit]" COLOR_RESET "\r\n");
	BME_CHECK(BME688_DeInit());
	BME_EXPECT(BME688_DeInit(), BME688_NOT_INITIALIZED);
	BME688_Centinela(&d);
	BME_EXPECT(BME688_Get(&d), BME688_NOT_INITIALIZED);
	BME_EXPECT(BME688_Sleep(), BME688_NOT_INITIALIZED);
	BME688_CheckSinTocar(&d);

	/* -- Re-Init: el driver debe poder arrancar de nuevo tras un DeInit -- */
	BME_CHECK(BME688_Init(&hi2c1));
	BME688_Leer(&d);

	/* -- Init sobre un driver ya inicializado: vuelve a detectar, resetear y configurar -- */
	BME_CHECK(BME688_Init(&hi2c1));
	BME688_Leer(&d);
	BME688_Print(&d);
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
