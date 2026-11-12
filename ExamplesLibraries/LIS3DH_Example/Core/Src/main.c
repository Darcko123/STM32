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
#include "LIS3DH.h"
#include "ANSII_Codes.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Parámetros de la batería de pruebas -- */
#define LIS_MUESTRAS        10U        /**< Lecturas seguidas del test de muestras */
#define LIS_SENTINEL        1000.0f    /**< Fuera de escala (±16 g): delata campos que la lectura no escribió */
#define LIS_FLOAT_TOL       0.001f     /**< Tolerancia al comparar los campos float de GetSensor */

/* En reposo: |a| ~ 1 g en cualquier orientación. Tolerancia amplia porque el sensor viene
 * sin calibrar (el datasheet da ±40 mg de offset en cero-g); ajustar si la placa falla quieta */
#define LIS_ACCEL_MOD_MIN   0.8f       /**< |a| mínimo en reposo [g] */
#define LIS_ACCEL_MOD_MAX   1.2f       /**< |a| máximo en reposo [g] */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LIS_CHECK(x)  do { if ((x) != LIS3DH_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define LIS_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
LIS3DH_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void LIS3DH_Centinela(LIS3DH_Accel_t* a);
static void LIS3DH_Leer(LIS3DH_Accel_t* a);
static void LIS3DH_CheckReposo(const LIS3DH_Accel_t* a, float g);
static void LIS3DH_CheckSinTocar(const LIS3DH_Accel_t* a);
static void LIS3DH_CheckConfig(LIS3DH_Range_t range, LIS3DH_DataRate_t odr);
static void LIS3DH_CheckCongelado(void);
static void LIS3DH_CheckVivo(void);
static void LIS3DH_PrintVal(const char* nombre, float v);
static void LIS3DH_Print(const char* unidad, const LIS3DH_Accel_t* a);
static void LIS3DH_TestSuite(void);
static void LIS3DH_TestPreInit(void);
static void LIS3DH_TestLectura(void);
static void LIS3DH_TestMuestras(void);
static void LIS3DH_TestEscalas(void);
static void LIS3DH_TestTasas(void);
static void LIS3DH_TestPowerDown(void);
static void LIS3DH_TestErrores(void);
static void LIS3DH_TestDeInit(void);
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
  printf(CLEAR_SCREEN CURSOR_HOME COLOR_BOLD COLOR_CYAN "LIS3DH TEST" COLOR_RESET "\r\n");
  LIS3DH_TestPreInit();

  status = LIS3DH_Init(&hi2c1);
  if(status != LIS3DH_OK)
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
	  LIS3DH_TestSuite();
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

/* Rellena la muestra con el centinela para detectar campos que la lectura no escribió */
static void LIS3DH_Centinela(LIS3DH_Accel_t* a)
{
	a->x = a->y = a->z = LIS_SENTINEL;
}

/* Lectura en g: debe ir bien y escribir los tres ejes (el centinela está fuera de escala) */
static void LIS3DH_Leer(LIS3DH_Accel_t* a)
{
	LIS3DH_Centinela(a);
	LIS_CHECK(LIS3DH_GetAccel(a));
	if (a->x == LIS_SENTINEL || a->y == LIS_SENTINEL || a->z == LIS_SENTINEL)
	{
		Error_Handler();
	}
}

/* Placa quieta: solo actúa la gravedad, |a| ~ 1 g (g = 1 para GetAccel, g = 9.81 para GetEvent).
 * Detecta factores de escala o bytes mal combinados que un simple rango no vería */
static void LIS3DH_CheckReposo(const LIS3DH_Accel_t* a, float g)
{
	const float mod2 = a->x * a->x + a->y * a->y + a->z * a->z;
	const float min  = LIS_ACCEL_MOD_MIN * g;
	const float max  = LIS_ACCEL_MOD_MAX * g;

	if (mod2 < min * min || mod2 > max * max)
	{
		Error_Handler();
	}
}

/* Tras una llamada rechazada la muestra debe seguir intacta */
static void LIS3DH_CheckSinTocar(const LIS3DH_Accel_t* a)
{
	if (a->x != LIS_SENTINEL || a->y != LIS_SENTINEL || a->z != LIS_SENTINEL)
	{
		Error_Handler();
	}
}

/* La escala y la tasa leídas del sensor deben coincidir con las esperadas */
static void LIS3DH_CheckConfig(LIS3DH_Range_t range, LIS3DH_DataRate_t odr)
{
	LIS3DH_Range_t    r;
	LIS3DH_DataRate_t d;

	LIS_CHECK(LIS3DH_GetRange(&r));
	LIS_CHECK(LIS3DH_GetDataRate(&d));
	if (r != range || d != odr)
	{
		Error_Handler();
	}
}

/* En power-down el sensor deja de muestrear: dos lecturas separadas deben ser idénticas */
static void LIS3DH_CheckCongelado(void)
{
	LIS3DH_Accel_t a1, a2;

	HAL_Delay(50);
	LIS3DH_Leer(&a1);
	HAL_Delay(100);
	LIS3DH_Leer(&a2);
	if (memcmp(&a1, &a2, sizeof(a1)) != 0)
	{
		Error_Handler();
	}
	LIS3DH_Print("g", &a1);
}

/* Muestreando a 400 Hz en 12 bits, el ruido (~3 mg rms) hace que dos lecturas nunca coincidan */
static void LIS3DH_CheckVivo(void)
{
	LIS3DH_Accel_t a1, a2;

	HAL_Delay(50);     /* Arranque tras power-down: 1/ODR + 1 ms */
	LIS3DH_Leer(&a1);
	HAL_Delay(50);
	LIS3DH_Leer(&a2);
	if (memcmp(&a1, &a2, sizeof(a1)) == 0)
	{
		Error_Handler();
	}
	LIS3DH_Print("g", &a2);
}

/* El proyecto no enlaza -u _printf_float: se imprime en milésimas con enteros (alta resolución = 1 mg) */
static void LIS3DH_PrintVal(const char* nombre, float v)
{
	const int32_t c = (int32_t)(v * 1000.0f);
	printf("%s%s%ld.%03ld ", nombre, (c < 0) ? "-" : "", labs(c) / 1000, labs(c) % 1000);
}

static void LIS3DH_Print(const char* unidad, const LIS3DH_Accel_t* a)
{
	printf("A[%s] ", unidad);
	LIS3DH_PrintVal("x=", a->x);
	LIS3DH_PrintVal("y=", a->y);
	LIS3DH_PrintVal("z=", a->z);
	printf("\r\n");
}

static void LIS3DH_TestSuite(void)
{
	LIS3DH_TestLectura();
	HAL_Delay(2000);

	LIS3DH_TestMuestras();
	HAL_Delay(2000);

	LIS3DH_TestEscalas();
	HAL_Delay(2000);

	LIS3DH_TestTasas();
	HAL_Delay(2000);

	LIS3DH_TestPowerDown();
	HAL_Delay(2000);

	LIS3DH_TestErrores();
	HAL_Delay(2000);

	LIS3DH_TestDeInit();
	HAL_Delay(2000);
}

static void LIS3DH_TestPreInit(void)
{
	LIS3DH_Accel_t    a;
	LIS3DH_Range_t    r;
	LIS3DH_DataRate_t d;
	LIS3DH_Sensor_t   s;

	/* -- Antes de Init: todo devuelve NOT_INITIALIZED sin tocar el bus ni la muestra -- */
	LIS3DH_Centinela(&a);
	LIS_EXPECT(LIS3DH_GetAccel(&a), LIS3DH_NOT_INITIALIZED);
	LIS_EXPECT(LIS3DH_GetEvent(&a), LIS3DH_NOT_INITIALIZED);
	LIS_EXPECT(LIS3DH_SetRange(LIS3DH_RANGE_2G), LIS3DH_NOT_INITIALIZED);
	LIS_EXPECT(LIS3DH_GetRange(&r), LIS3DH_NOT_INITIALIZED);
	LIS_EXPECT(LIS3DH_SetDataRate(LIS3DH_DATARATE_400HZ), LIS3DH_NOT_INITIALIZED);
	LIS_EXPECT(LIS3DH_GetDataRate(&d), LIS3DH_NOT_INITIALIZED);
	LIS_EXPECT(LIS3DH_GetSensor(&s), LIS3DH_NOT_INITIALIZED);
	LIS3DH_CheckSinTocar(&a);

	/* DeInit sobre un driver sin inicializar es inofensivo */
	LIS_CHECK(LIS3DH_DeInit());
}

static void LIS3DH_TestLectura(void)
{
	LIS3DH_Accel_t a;

	/* -- Configuración de Init: ±2 g a 400 Hz -- */
	printf(COLOR_YELLOW "[Lectura]" COLOR_RESET "\r\n");
	LIS3DH_CheckConfig(LIS3DH_RANGE_2G, LIS3DH_DATARATE_400HZ);

	/* -- Lectura única con la placa QUIETA: |a| ~ 1 g en GetAccel y ~ 9.81 m/s² en GetEvent -- */
	LIS3DH_Leer(&a);
	LIS3DH_CheckReposo(&a, 1.0f);
	LIS3DH_Print("g", &a);

	LIS3DH_Centinela(&a);
	LIS_CHECK(LIS3DH_GetEvent(&a));
	LIS3DH_CheckReposo(&a, LIS3DH_GRAVITY_STANDARD);
	LIS3DH_Print("m/s2", &a);
}

static void LIS3DH_TestMuestras(void)
{
	LIS3DH_Accel_t a;

	/* -- Lecturas seguidas cada ~0.5 s: ninguna debe fallar. Inclina la placa: el eje que
	 *    apunta hacia abajo marca ~1 g y los otros dos ~0 g -- */
	printf(COLOR_YELLOW "[Muestras]" COLOR_RESET "\r\n");
	for (uint8_t i = 1; i <= LIS_MUESTRAS; i++)
	{
		LIS3DH_Leer(&a);
		printf("%2u/%u  ", i, LIS_MUESTRAS);
		LIS3DH_Print("g", &a);
		HAL_Delay(500);
	}
}

static void LIS3DH_TestEscalas(void)
{
	/* Datasheet, tabla 4: fondo de escala y sensibilidad en alta resolución [mg/dígito] */
	static const uint8_t rango_g[4] = { 2, 4, 8, 16 };
	static const uint8_t sens_mg[4] = { 1, 2, 4, 12 };
	LIS3DH_Accel_t  a;
	LIS3DH_Sensor_t s;

	/* -- Escalas: GetRange devuelve la escala escrita, GetAccel sigue marcando ~1 g (la
	 *    conversión se ajusta sola) y GetSensor da el fondo de escala y la resolución -- */
	printf(COLOR_YELLOW "[Escalas]" COLOR_RESET "\r\n");
	for (uint8_t r = LIS3DH_RANGE_2G; r <= LIS3DH_RANGE_16G; r++)
	{
		const float max = rango_g[r] * LIS3DH_GRAVITY_STANDARD;
		const float res = sens_mg[r] * LIS3DH_GRAVITY_STANDARD / 1000.0f;

		LIS_CHECK(LIS3DH_SetRange((LIS3DH_Range_t)r));
		LIS3DH_CheckConfig((LIS3DH_Range_t)r, LIS3DH_DATARATE_400HZ);
		LIS3DH_Leer(&a);
		LIS3DH_CheckReposo(&a, 1.0f);

		LIS_CHECK(LIS3DH_GetSensor(&s));
		if (s.name == NULL || strcmp(s.name, "LIS3DH") != 0 ||
			fabsf(s.max_value - max) > LIS_FLOAT_TOL || fabsf(s.min_value + max) > LIS_FLOAT_TOL ||
			fabsf(s.resolution - res) > LIS_FLOAT_TOL)
		{
			Error_Handler();
		}

		printf("+-%2u g  ", rango_g[r]);
		LIS3DH_Print("g", &a);
	}

	/* Restaura la escala de trabajo para el resto de los tests */
	LIS_CHECK(LIS3DH_SetRange(LIS3DH_RANGE_2G));
	printf(COLOR_GREEN "Escalas OK" COLOR_RESET "\r\n");
}

static void LIS3DH_TestTasas(void)
{
	/* Periodo esperado en GetSensor [us]: 1e6 / ODR (0 en power-down). En alta resolución
	 * ODR 8 (1.6 kHz) no es válido (solo bajo consumo), pero el registro se escribe igual */
	static const uint32_t periodo_us[10] = { 0, 1000000, 100000, 40000, 20000, 10000, 5000, 2500, 625, 744 };
	LIS3DH_Sensor_t s;

	/* -- Tasas: GetDataRate devuelve las 10 ODR escritas sin tocar la escala, y GetSensor
	 *    da su periodo -- */
	printf(COLOR_YELLOW "[Tasas]" COLOR_RESET "\r\n");
	for (uint8_t d = LIS3DH_DATARATE_POWERDOWN; d <= LIS3DH_DATARATE_1K344HZ_LP5K376; d++)
	{
		LIS_CHECK(LIS3DH_SetDataRate((LIS3DH_DataRate_t)d));
		LIS3DH_CheckConfig(LIS3DH_RANGE_2G, (LIS3DH_DataRate_t)d);
		LIS_CHECK(LIS3DH_GetSensor(&s));
		if (s.min_delay != periodo_us[d])
		{
			Error_Handler();
		}
		printf("ODR %u -> %lu us\r\n", d, s.min_delay);
	}

	/* Restaura la tasa de trabajo para el resto de los tests */
	LIS_CHECK(LIS3DH_SetDataRate(LIS3DH_DATARATE_400HZ));
	printf(COLOR_GREEN "Tasas OK" COLOR_RESET "\r\n");
}

static void LIS3DH_TestPowerDown(void)
{
	/* -- Power-down: el I2C sigue respondiendo pero las lecturas se congelan -- */
	printf(COLOR_YELLOW "[PowerDown]" COLOR_RESET "\r\n");
	LIS_CHECK(LIS3DH_SetDataRate(LIS3DH_DATARATE_POWERDOWN));
	LIS3DH_CheckCongelado();

	/* -- Volver a 400 Hz: las lecturas vuelven a variar -- */
	LIS_CHECK(LIS3DH_SetDataRate(LIS3DH_DATARATE_400HZ));
	LIS3DH_CheckVivo();
}

static void LIS3DH_TestErrores(void)
{
	LIS3DH_Accel_t a;

	/* -- Punteros de salida NULL y escala/tasa fuera de rango -- */
	printf(COLOR_YELLOW "[Errores]" COLOR_RESET "\r\n");
	LIS_EXPECT(LIS3DH_GetAccel(NULL), LIS3DH_INVALID_PARAM);
	LIS_EXPECT(LIS3DH_GetEvent(NULL), LIS3DH_INVALID_PARAM);
	LIS_EXPECT(LIS3DH_GetRange(NULL), LIS3DH_INVALID_PARAM);
	LIS_EXPECT(LIS3DH_GetDataRate(NULL), LIS3DH_INVALID_PARAM);
	LIS_EXPECT(LIS3DH_GetSensor(NULL), LIS3DH_INVALID_PARAM);
	LIS_EXPECT(LIS3DH_SetRange((LIS3DH_Range_t)(LIS3DH_RANGE_16G + 1)), LIS3DH_INVALID_PARAM);
	LIS_EXPECT(LIS3DH_SetDataRate((LIS3DH_DataRate_t)(LIS3DH_DATARATE_1K344HZ_LP5K376 + 1)), LIS3DH_INVALID_PARAM);

	/* -- Los Set rechazados no deben haber tocado la configuración -- */
	LIS3DH_CheckConfig(LIS3DH_RANGE_2G, LIS3DH_DATARATE_400HZ);

	/* -- Init con handle NULL: se rechaza antes de tocar el estado, el driver sigue funcionando -- */
	LIS_EXPECT(LIS3DH_Init(NULL), LIS3DH_INVALID_PARAM);
	LIS3DH_Leer(&a);
	printf(COLOR_GREEN "Errores OK" COLOR_RESET "\r\n");
}

static void LIS3DH_TestDeInit(void)
{
	LIS3DH_Accel_t    a;
	LIS3DH_Range_t    r;
	LIS3DH_DataRate_t d;
	LIS3DH_Sensor_t   s;

	/* -- Se deja una configuración distinta de la de Init para comprobar luego que Init la restaura -- */
	printf(COLOR_YELLOW "[DeInit]" COLOR_RESET "\r\n");
	LIS_CHECK(LIS3DH_SetRange(LIS3DH_RANGE_16G));
	LIS_CHECK(LIS3DH_SetDataRate(LIS3DH_DATARATE_10HZ));

	/* -- DeInit libera el handle: las llamadas posteriores devuelven NOT_INITIALIZED
	 *    sin tocar el bus ni la muestra. Repetirlo sigue devolviendo OK -- */
	LIS_CHECK(LIS3DH_DeInit());
	LIS_CHECK(LIS3DH_DeInit());
	LIS3DH_Centinela(&a);
	LIS_EXPECT(LIS3DH_GetAccel(&a), LIS3DH_NOT_INITIALIZED);
	LIS_EXPECT(LIS3DH_GetEvent(&a), LIS3DH_NOT_INITIALIZED);
	LIS_EXPECT(LIS3DH_SetRange(LIS3DH_RANGE_2G), LIS3DH_NOT_INITIALIZED);
	LIS_EXPECT(LIS3DH_GetRange(&r), LIS3DH_NOT_INITIALIZED);
	LIS_EXPECT(LIS3DH_SetDataRate(LIS3DH_DATARATE_400HZ), LIS3DH_NOT_INITIALIZED);
	LIS_EXPECT(LIS3DH_GetDataRate(&d), LIS3DH_NOT_INITIALIZED);
	LIS_EXPECT(LIS3DH_GetSensor(&s), LIS3DH_NOT_INITIALIZED);
	LIS3DH_CheckSinTocar(&a);

	/* -- Re-Init: arranca de nuevo y reescribe CTRL_REG1/CTRL_REG4 (vuelve a ±2 g y 400 Hz) -- */
	LIS_CHECK(LIS3DH_Init(&hi2c1));
	LIS3DH_CheckConfig(LIS3DH_RANGE_2G, LIS3DH_DATARATE_400HZ);
	HAL_Delay(50);     /* Primera muestra a 400 Hz tras el cambio de ODR */
	LIS3DH_Leer(&a);
	LIS3DH_CheckReposo(&a, 1.0f);
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
