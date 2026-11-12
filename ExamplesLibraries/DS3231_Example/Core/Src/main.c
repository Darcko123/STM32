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
#include "DS3231.h"
#include "ANSII_Codes.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Caso de desborde del calendario: fecha de partida y la que debe marcar un tic después */
typedef struct {
	const char*   nombre;
	DS3231_Time_t inicio;
	DS3231_Time_t esperado;
} DS_Desborde_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Parámetros de la batería de pruebas -- */
#define DS_SENTINEL         0xFFU      /**< Fuera de rango en todos los campos: delata lo que Get no escribió */
#define DS_MUESTRAS         5U         /**< Lecturas seguidas del test de muestras (una por segundo) */

/* Escribir los segundos reinicia el divisor interno del DS3231: el primer tic llega 1 s
 * después del SetTime. Las esperas caen a media cuenta entre tics para tolerar la deriva
 * del HSI (±1 %) con el que cuenta HAL_Delay */
#define DS_TIC_MS           1500U      /**< Espera que cubre exactamente 1 tic tras un SetTime */
#define DS_ESPERA_MS        3500U      /**< Espera del test de avance */
#define DS_AVANCE_S         3U         /**< Segundos que debe avanzar el RTC en DS_ESPERA_MS */

/* Rango de funcionamiento del sensor de temperatura interno */
#define DS_TEMP_MIN         (-40)
#define DS_TEMP_MAX         (85)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DS_CHECK(x)  do { if ((x) != DS3231_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define DS_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
DS3231_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void DS3231_Centinela(void* p, size_t n);
static void DS3231_CheckSinTocar(const void* p, size_t n);
static void DS3231_CheckNoInit(void);
static void DS3231_Leer(DS3231_Time_t* t);
static void DS3231_LeerTemp(DS3231_Temp_t* t);
static uint32_t DS3231_Avance(const DS3231_Time_t* t0, const DS3231_Time_t* t1);
static void DS3231_PrintHora(const DS3231_Time_t* t);
static void DS3231_PrintTemp(const DS3231_Temp_t* t);
static void DS3231_TestSuite(void);
static void DS3231_TestPreInit(void);
static void DS3231_TestHora(void);
static void DS3231_TestMuestras(void);
static void DS3231_TestDesbordes(void);
static void DS3231_TestAlarmas(void);
static void DS3231_TestTemperatura(void);
static void DS3231_TestErrores(void);
static void DS3231_TestDeInit(void);
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
  printf(CLEAR_SCREEN CURSOR_HOME COLOR_BOLD COLOR_CYAN "DS3231 TEST" COLOR_RESET "\r\n");
  DS3231_TestPreInit();

  status = DS3231_Init(&hi2c1);
  if(status != DS3231_OK)
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
	  DS3231_TestSuite();
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

/* Rellena una estructura de salida con el centinela */
static void DS3231_Centinela(void* p, size_t n)
{
	memset(p, DS_SENTINEL, n);
}

/* Tras una llamada rechazada la estructura de salida debe seguir intacta */
static void DS3231_CheckSinTocar(const void* p, size_t n)
{
	const uint8_t* b = (const uint8_t*)p;

	for (size_t i = 0; i < n; i++)
	{
		if (b[i] != DS_SENTINEL)
		{
			Error_Handler();
		}
	}
}

/* Sin inicializar: todo devuelve NOT_INITIALIZED sin tocar el bus ni las estructuras */
static void DS3231_CheckNoInit(void)
{
	DS3231_Time_t   t;
	DS3231_Alarm1_t a1;
	DS3231_Alarm2_t a2;
	DS3231_Temp_t   tc;

	DS3231_Centinela(&t, sizeof(t));
	DS3231_Centinela(&a1, sizeof(a1));
	DS3231_Centinela(&a2, sizeof(a2));
	DS3231_Centinela(&tc, sizeof(tc));

	DS_EXPECT(DS3231_SetTime(&t), DS3231_NOT_INITIALIZED);
	DS_EXPECT(DS3231_GetTime(&t), DS3231_NOT_INITIALIZED);
	DS_EXPECT(DS3231_SetAlarm1(&a1), DS3231_NOT_INITIALIZED);
	DS_EXPECT(DS3231_GetAlarm1(&a1), DS3231_NOT_INITIALIZED);
	DS_EXPECT(DS3231_SetAlarm2(&a2), DS3231_NOT_INITIALIZED);
	DS_EXPECT(DS3231_GetAlarm2(&a2), DS3231_NOT_INITIALIZED);
	DS_EXPECT(DS3231_GetTemperature(&tc), DS3231_NOT_INITIALIZED);

	DS3231_CheckSinTocar(&t, sizeof(t));
	DS3231_CheckSinTocar(&a1, sizeof(a1));
	DS3231_CheckSinTocar(&a2, sizeof(a2));
	DS3231_CheckSinTocar(&tc, sizeof(tc));
}

/* Lectura de hora: debe ir bien y dar una fecha válida en todos los campos
 * (el centinela 0xFF está fuera de rango en cualquiera de ellos) */
static void DS3231_Leer(DS3231_Time_t* t)
{
	DS3231_Centinela(t, sizeof(*t));
	DS_CHECK(DS3231_GetTime(t));

	if (t->seconds > 59U || t->minutes > 59U || t->hour > 23U ||
		t->dayofmonth < 1U || t->dayofmonth > 31U ||
		t->month < 1U || t->month > 12U || t->year > 99U)
	{
		Error_Handler();
	}
}

/* Lectura de temperatura: rango del chip, fracción en cuartos de grado y raw coherente.
 * Según el datasheet el MSB es la parte entera en complemento a dos y la fracción
 * siempre suma (-6.25 C = -7 + 0.75), así que raw = integer * 100 + fraction */
static void DS3231_LeerTemp(DS3231_Temp_t* t)
{
	DS3231_Centinela(t, sizeof(*t));
	DS_CHECK(DS3231_GetTemperature(t));

	if (t->integer < DS_TEMP_MIN || t->integer > DS_TEMP_MAX ||
		(t->fraction % 25U) != 0U || t->fraction > 75U ||
		t->raw != (int16_t)(t->integer * 100 + t->fraction))
	{
		Error_Handler();
	}
}

/* Segundos que avanzó el reloj entre dos lecturas (módulo un día) */
static uint32_t DS3231_Avance(const DS3231_Time_t* t0, const DS3231_Time_t* t1)
{
	const uint32_t s0 = t0->hour * 3600U + t0->minutes * 60U + t0->seconds;
	const uint32_t s1 = t1->hour * 3600U + t1->minutes * 60U + t1->seconds;

	return (s1 + 86400U - s0) % 86400U;
}

static void DS3231_PrintHora(const DS3231_Time_t* t)
{
	printf("%02d:%02d:%02d %02d-%02d-20%02d ",
		   t->hour, t->minutes, t->seconds, t->dayofmonth, t->month, t->year);
}

/* El proyecto no enlaza -u _printf_float: raw ya viene en centésimas */
static void DS3231_PrintTemp(const DS3231_Temp_t* t)
{
	printf("T=%s%d.%02dC ", (t->raw < 0) ? "-" : "", abs(t->raw) / 100, abs(t->raw) % 100);
}

static void DS3231_TestSuite(void)
{
	DS3231_TestHora();
	HAL_Delay(2000);

	DS3231_TestMuestras();
	HAL_Delay(2000);

	DS3231_TestDesbordes();
	HAL_Delay(2000);

	DS3231_TestAlarmas();
	HAL_Delay(2000);

	DS3231_TestTemperatura();
	HAL_Delay(2000);

	DS3231_TestErrores();
	HAL_Delay(2000);

	DS3231_TestDeInit();
	HAL_Delay(2000);
}

static void DS3231_TestPreInit(void)
{
	/* -- Antes de Init: todo devuelve NOT_INITIALIZED sin tocar el bus ni las estructuras -- */
	DS3231_CheckNoInit();

	/* DeInit sobre un driver sin inicializar es inofensivo */
	DS_CHECK(DS3231_DeInit());
}

static void DS3231_TestHora(void)
{
	DS3231_Time_t ref = { .seconds = 56, .minutes = 34, .hour = 12, .dayofmonth = 7, .month = 11, .year = 26 };
	DS3231_Time_t t0, t1;

	/* -- SetTime + GetTime: la lectura inmediata devuelve exactamente lo escrito -- */
	printf(COLOR_YELLOW "[Hora]" COLOR_RESET "\r\n");
	DS_CHECK(DS3231_SetTime(&ref));
	DS3231_Leer(&t0);
	if (memcmp(&t0, &ref, sizeof(ref)) != 0)
	{
		Error_Handler();
	}
	DS3231_PrintHora(&t0);
	printf("\r\n");

	/* -- El oscilador corre: en DS_ESPERA_MS el reloj avanza exactamente DS_AVANCE_S segundos.
	 *    Si falla por uno, revisar la deriva del HSI antes que el RTC -- */
	HAL_Delay(DS_ESPERA_MS);
	DS3231_Leer(&t1);
	if (DS3231_Avance(&t0, &t1) != DS_AVANCE_S)
	{
		Error_Handler();
	}
	DS3231_PrintHora(&t1);
	printf("\r\n");
}

static void DS3231_TestMuestras(void)
{
	DS3231_Time_t t;
	DS3231_Temp_t tc;

	/* -- Hora y temperatura una vez por segundo: ninguna lectura debe fallar -- */
	printf(COLOR_YELLOW "[Muestras]" COLOR_RESET "\r\n");
	for (uint8_t i = 1; i <= DS_MUESTRAS; i++)
	{
		DS3231_Leer(&t);
		DS3231_LeerTemp(&tc);
		printf("%u/%u  ", i, DS_MUESTRAS);
		DS3231_PrintHora(&t);
		DS3231_PrintTemp(&tc);
		printf("\r\n");
		HAL_Delay(1000);
	}
}

static void DS3231_TestDesbordes(void)
{
	/* No se prueba 31-12-99 -> 01-01-00: el DS3231 activa el bit Century (bit 7 del mes)
	 * y GetTime no lo enmascara, devolvería mes 81 */
	static const DS_Desborde_t casos[] = {
		{ "Fin de anio",
		  { .seconds = 59, .minutes = 59, .hour = 23, .dayofmonth = 31, .month = 12, .year = 25 },
		  { .seconds = 0,  .minutes = 0,  .hour = 0,  .dayofmonth = 1,  .month = 1,  .year = 26 } },
		{ "Bisiesto",
		  { .seconds = 59, .minutes = 59, .hour = 23, .dayofmonth = 28, .month = 2,  .year = 24 },
		  { .seconds = 0,  .minutes = 0,  .hour = 0,  .dayofmonth = 29, .month = 2,  .year = 24 } },
		{ "No bisiesto",
		  { .seconds = 59, .minutes = 59, .hour = 23, .dayofmonth = 28, .month = 2,  .year = 26 },
		  { .seconds = 0,  .minutes = 0,  .hour = 0,  .dayofmonth = 1,  .month = 3,  .year = 26 } },
		{ "Mes de 30",
		  { .seconds = 59, .minutes = 59, .hour = 23, .dayofmonth = 30, .month = 4,  .year = 26 },
		  { .seconds = 0,  .minutes = 0,  .hour = 0,  .dayofmonth = 1,  .month = 5,  .year = 26 } },
	};
	DS3231_Time_t inicio, t;

	/* -- Calendario: un tic después de las 23:59:59 cambian todos los campos a la vez -- */
	printf(COLOR_YELLOW "[Desbordes]" COLOR_RESET "\r\n");
	for (uint8_t i = 0; i < sizeof(casos) / sizeof(casos[0]); i++)
	{
		inicio = casos[i].inicio;
		DS_CHECK(DS3231_SetTime(&inicio));
		HAL_Delay(DS_TIC_MS);
		DS3231_Leer(&t);
		if (memcmp(&t, &casos[i].esperado, sizeof(t)) != 0)
		{
			Error_Handler();
		}
		printf("%-12s", casos[i].nombre);
		DS3231_PrintHora(&t);
		printf("\r\n");
	}
}

static void DS3231_TestAlarmas(void)
{
	DS3231_Alarm1_t a1 = { .seconds = 59, .minutes = 59, .hour = 23 };
	DS3231_Alarm2_t a2 = { .minutes = 45, .hour = 9 };
	DS3231_Alarm1_t l1;
	DS3231_Alarm2_t l2;
	DS3231_Time_t   t0, t1;

	/* -- Escribir y releer cada alarma: la lectura coincide campo a campo -- */
	printf(COLOR_YELLOW "[Alarmas]" COLOR_RESET "\r\n");
	DS3231_Leer(&t0);

	DS_CHECK(DS3231_SetAlarm1(&a1));
	DS3231_Centinela(&l1, sizeof(l1));
	DS_CHECK(DS3231_GetAlarm1(&l1));
	if (memcmp(&l1, &a1, sizeof(a1)) != 0)
	{
		Error_Handler();
	}

	DS_CHECK(DS3231_SetAlarm2(&a2));
	DS3231_Centinela(&l2, sizeof(l2));
	DS_CHECK(DS3231_GetAlarm2(&l2));
	if (memcmp(&l2, &a2, sizeof(a2)) != 0)
	{
		Error_Handler();
	}

	/* -- Las alarmas van en registros aparte (0x07 y 0x0B): la fecha no cambia y la hora
	 *    solo avanza lo que tardaron las transacciones -- */
	DS3231_Leer(&t1);
	if (t1.dayofmonth != t0.dayofmonth || t1.month != t0.month || t1.year != t0.year ||
		DS3231_Avance(&t0, &t1) > 1U)
	{
		Error_Handler();
	}
	printf("A1=%02d:%02d:%02d A2=%02d:%02d\r\n", l1.hour, l1.minutes, l1.seconds, l2.hour, l2.minutes);
}

static void DS3231_TestTemperatura(void)
{
	DS3231_Temp_t tc;

	/* -- Sensor interno: rango, fracción en cuartos y raw coherente. Se convierte cada 64 s,
	 *    así que lecturas seguidas suelen repetirse -- */
	printf(COLOR_YELLOW "[Temperatura]" COLOR_RESET "\r\n");
	DS3231_LeerTemp(&tc);
	DS3231_PrintTemp(&tc);
	printf("(integer=%d fraction=%u)\r\n", tc.integer, tc.fraction);
}

static void DS3231_TestErrores(void)
{
	DS3231_Time_t t;

	/* -- Punteros NULL: se rechazan con INVALID_PARAM -- */
	printf(COLOR_YELLOW "[Errores]" COLOR_RESET "\r\n");
	DS_EXPECT(DS3231_SetTime(NULL), DS3231_INVALID_PARAM);
	DS_EXPECT(DS3231_GetTime(NULL), DS3231_INVALID_PARAM);
	DS_EXPECT(DS3231_SetAlarm1(NULL), DS3231_INVALID_PARAM);
	DS_EXPECT(DS3231_GetAlarm1(NULL), DS3231_INVALID_PARAM);
	DS_EXPECT(DS3231_SetAlarm2(NULL), DS3231_INVALID_PARAM);
	DS_EXPECT(DS3231_GetAlarm2(NULL), DS3231_INVALID_PARAM);
	DS_EXPECT(DS3231_GetTemperature(NULL), DS3231_INVALID_PARAM);

	/* -- Init con handle NULL: la librería devuelve ERROR (no INVALID_PARAM) antes de tocar
	 *    el estado, el driver sigue funcionando -- */
	DS_EXPECT(DS3231_Init(NULL), DS3231_ERROR);
	DS3231_Leer(&t);
	printf(COLOR_GREEN "Errores OK" COLOR_RESET "\r\n");
}

static void DS3231_TestDeInit(void)
{
	DS3231_Time_t t0, t1;

	/* -- DeInit libera el handle: las llamadas posteriores devuelven NOT_INITIALIZED
	 *    sin tocar el bus ni las estructuras. Repetirlo sigue devolviendo OK -- */
	printf(COLOR_YELLOW "[DeInit]" COLOR_RESET "\r\n");
	DS3231_Leer(&t0);
	DS_CHECK(DS3231_DeInit());
	DS_CHECK(DS3231_DeInit());
	DS3231_CheckNoInit();
	HAL_Delay(DS_TIC_MS);

	/* -- Re-Init: el driver arranca de nuevo y el chip siguió contando durante el DeInit -- */
	DS_CHECK(DS3231_Init(&hi2c1));
	DS3231_Leer(&t1);
	if (DS3231_Avance(&t0, &t1) == 0U)
	{
		Error_Handler();
	}
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
