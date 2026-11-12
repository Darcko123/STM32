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
#include "ANSII_Codes.h"
#include "NEO_6M.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Conexión del módulo: el GPS va por USART2 a 9600 baudios 8N1 (válido por defecto
 *    en el NEO-6M); la consola de diagnóstico es USART1 (PA9) -- */

/* -- Parámetros de la batería de pruebas -- */
#define NEO_ARRANQUE_MS     2000U      /**< El módulo tarda ~1-2 s en emitir las primeras sentencias NMEA */
#define NEO_PERIODO_MS      1000U      /**< Entre lecturas del test de muestras (los NMEA salen a 1 Hz) */
#define NEO_MUESTRAS        10U        /**< Lecturas seguidas del test de muestras */
#define NEO_SENTINEL        1000.0     /**< Coordenada fuera de rango: delata campos que Get no escribió */
#define NEO_FIX_TIMEOUT_MS  180000U    /**< Espera máxima de fix: arranque en frío ~30 s a cielo abierto, más con mala visibilidad */

/* Rangos físicos de la posición */
#define NEO_LAT_MAX         90.0       /**< |latitud| máxima [grados decimales] */
#define NEO_LON_MAX         180.0      /**< |longitud| máxima [grados decimales] */
#define NEO_HDOP_MAX        100.0f     /**< El NEO-6M informa HDOP 99.99 sin fix */
#define NEO_RUMBO_MAX       360.0f     /**< Rumbo sobre el suelo [grados] */

/* Puntos de control de Distance/Bearing */
#define NEO_GRADO_M         111195.08  /**< 1 grado de meridiano = R·π/180 con R = 6371008.8 m */
#define NEO_DIST_TOL_M      1.0        /**< Tolerancia de distancia [m] */
#define NEO_RUMBO_TOL       1e-6       /**< Tolerancia de rumbo [grados] */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NEO_CHECK(x)  do { if ((x) != NEO_6M_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define NEO_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
NEO_6M_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void NEO6M_Centinela(NEO_6M_Data_t* m);
static void NEO6M_Leer(NEO_6M_Data_t* m);
static void NEO6M_CheckDatos(const NEO_6M_Data_t* m);
static void NEO6M_CheckSinTocar(const NEO_6M_Data_t* m);
static void NEO6M_Print(const NEO_6M_Data_t* m);
static void NEO6M_TestSuite(void);
static void NEO6M_TestPreInit(void);
static void NEO6M_EsperarFix(void);
static void NEO6M_TestLectura(void);
static void NEO6M_TestMuestras(void);
static void NEO6M_TestConversiones(void);
static void NEO6M_TestErrores(void);
static void NEO6M_TestDeInit(void);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf(CLEAR_SCREEN CURSOR_HOME COLOR_BOLD COLOR_CYAN "NEO6M TEST" COLOR_RESET "\r\n");
  NEO6M_TestPreInit();

  status = NEO_6M_Init(&huart2);
  if(status != NEO_6M_OK)
  {
	  Error_Handler();
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  HAL_Delay(NEO_ARRANQUE_MS);   /* Arranque del módulo: primeras sentencias NMEA */
  NEO6M_EsperarFix();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  NEO6M_TestSuite();
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

/* Enruta el byte recibido hacia el driver GPS: él mismo comprueba que el handle
 * sea el suyo (ignora USART1) y rearma la interrupción automáticamente */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    NEO_6M_UART_RxCpltCallback(huart);
}

/* Un overrun (ORE) aborta la recepción por IT: el driver la rearma */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    NEO_6M_UART_ErrorCallback(huart);
}

/* Rellena la muestra con el centinela para detectar campos que Get no escribió */
static void NEO6M_Centinela(NEO_6M_Data_t* m)
{
	m->latitude  = NEO_SENTINEL;
	m->longitude = NEO_SENTINEL;
}

/* Lectura completa: debe ir bien y dejar datos plausibles en la estructura */
static void NEO6M_Leer(NEO_6M_Data_t* m)
{
	NEO6M_Centinela(m);
	NEO_CHECK(NEO_6M_Get(m));
	NEO6M_CheckDatos(m);
}

/* Coherencia de la estructura: rangos físicos y códigos válidos. Sin fix los
 * campos valen 0 (que también es válido). Las comparaciones de doble se escriben
 * como !(v >= min && v <= max) para que un NaN también falle */
static void NEO6M_CheckDatos(const NEO_6M_Data_t* m)
{
	if (!(m->latitude >= -NEO_LAT_MAX && m->latitude <= NEO_LAT_MAX) ||
		!(m->longitude >= -NEO_LON_MAX && m->longitude <= NEO_LON_MAX) ||
		m->fixQuality > 8 || m->numSatellites > 32 ||
		m->hours > 23 || m->minutes > 59 || m->seconds > 59 ||
		m->localHour > 23 ||
		!(m->hdop >= 0.0f && m->hdop <= NEO_HDOP_MAX) ||
		!(m->speedKmh >= 0.0f) ||
		!(m->course >= 0.0f && m->course <= NEO_RUMBO_MAX) ||
		m->day > 31 || m->month > 12 || m->year > 99 ||
		(m->posStatus != 'A' && m->posStatus != 'V' && m->posStatus != 0) ||
		(m->northSouth != 'N' && m->northSouth != 'S' && m->northSouth != 0) ||
		(m->eastWest != 'E' && m->eastWest != 'W' && m->eastWest != 0))
	{
		Error_Handler();
	}
}

/* Tras un Get rechazado la muestra debe seguir intacta */
static void NEO6M_CheckSinTocar(const NEO_6M_Data_t* m)
{
	if (m->latitude != NEO_SENTINEL || m->longitude != NEO_SENTINEL)
	{
		Error_Handler();
	}
}

/* El proyecto no enlaza -u _printf_float: coordenadas en miligrados con enteros
 * (±90 * 1000 cabe de sobra en int32), altura, velocidad y rumbo en décimas y
 * HDOP en centésimas (estos tres nunca son negativos) */
static void NEO6M_Print(const NEO_6M_Data_t* m)
{
	const int32_t latI = (int32_t)(m->latitude  * 1000.0 + ((m->latitude  < 0.0) ? -0.5 : 0.5));
	const int32_t lonI = (int32_t)(m->longitude * 1000.0 + ((m->longitude < 0.0) ? -0.5 : 0.5));
	const int32_t altI = (int32_t)(m->altitude * 10.0f  + ((m->altitude < 0.0f) ? -0.5f : 0.5f));
	const int32_t hdI  = (int32_t)(m->hdop     * 100.0f + 0.5f);
	const int32_t velI = (int32_t)(m->speedKmh * 10.0f  + 0.5f);
	const int32_t rumI = (int32_t)(m->course   * 10.0f  + 0.5f);
	const char    ns   = (m->northSouth != 0) ? m->northSouth : '-';
	const char    ew   = (m->eastWest   != 0) ? m->eastWest   : '-';
	const char    st   = (m->posStatus  != 0) ? m->posStatus  : '-';

	printf("Lat %s%ld.%03ld %c  Lon %s%ld.%03ld %c | UTC %02u:%02u:%02u.%03u (Local %02u) | Alt %ld.%01ld m | FixQ %u Sat %u | %c\r\n",
		   (latI < 0) ? "-" : "", labs(latI) / 1000, labs(latI) % 1000, ns,
		   (lonI < 0) ? "-" : "", labs(lonI) / 1000, labs(lonI) % 1000, ew,
		   (unsigned)m->hours, (unsigned)m->minutes, (unsigned)m->seconds, (unsigned)m->milliseconds,
		   (unsigned)m->localHour,
		   labs(altI) / 10, labs(altI) % 10,
		   (unsigned)m->fixQuality, (unsigned)m->numSatellites,
		   st);
	printf("       Fecha %02u/%02u/20%02u | HDOP %ld.%02ld | Vel %ld.%01ld km/h | Rumbo %ld.%01ld%s\r\n",
		   (unsigned)m->day, (unsigned)m->month, (unsigned)m->year,
		   hdI / 100, hdI % 100, velI / 10, velI % 10, rumI / 10, rumI % 10,
		   (m->fixQuality == 0U) ? COLOR_RED "  [SIN FIX: posicion no actual]" COLOR_RESET : "");
}

/* -- Espera de fix: sin él el módulo emite NMEA válidos (Get devuelve OK) pero con
 *    fixQuality 0 y la posición a 0 o la última conocida. Muestra el progreso cada
 *    segundo y parpadea el LED lento (1 Hz; el de Error_Handler es 5 Hz). Agotado
 *    NEO_FIX_TIMEOUT_MS sigue sin fix: la batería de pruebas es válida igualmente.
 *    Get con TIMEOUT aquí no es falta de fix sino de NMEA (cableado/baudios) -- */
static void NEO6M_EsperarFix(void)
{
	NEO_6M_Data_t m;
	const uint32_t t0 = HAL_GetTick();

	printf(COLOR_YELLOW "[Esperando fix]" COLOR_RESET " (coloca la antena con vista al cielo)\r\n");
	for (;;)
	{
		NEO_CHECK(NEO_6M_Get(&m));
		const uint32_t s = (HAL_GetTick() - t0) / 1000U;

		if (m.fixQuality != 0U)
		{
			printf("\r\n" COLOR_GREEN "Fix en %lu s (%u satelites)" COLOR_RESET "\r\n",
				   (unsigned long)s, (unsigned)m.numSatellites);
			break;
		}
		if ((HAL_GetTick() - t0) >= NEO_FIX_TIMEOUT_MS)
		{
			printf("\r\n" COLOR_RED "Sin fix tras %lu s: se continua sin posicion" COLOR_RESET "\r\n",
				   (unsigned long)s);
			break;
		}

		const int32_t hdI = (int32_t)(m.hdop * 100.0f + 0.5f);
		printf("\r  %3lu s | Sat %2u | HDOP %ld.%02ld | UTC %02u:%02u:%02u ",
			   (unsigned long)s, (unsigned)m.numSatellites, hdI / 100, hdI % 100,
			   (unsigned)m.hours, (unsigned)m.minutes, (unsigned)m.seconds);
		fflush(stdout);   /* Sin '\n' newlib retiene la línea en el búfer */
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

static void NEO6M_TestSuite(void)
{
	NEO6M_TestLectura();
	HAL_Delay(2000);

	NEO6M_TestMuestras();
	HAL_Delay(2000);

	NEO6M_TestConversiones();
	HAL_Delay(2000);

	NEO6M_TestErrores();
	HAL_Delay(2000);

	NEO6M_TestDeInit();
	HAL_Delay(2000);
}

static void NEO6M_TestPreInit(void)
{
	NEO_6M_Data_t m;
	NEO_6M_UTM_t utm;
	NEO_6M_DMS_t dms;
	double v;

	/* -- Antes de Init: Get devuelve NOT_INITIALIZED sin tocar la muestra (el check
	 *    de inicialización va antes de la copia) y Get con NULL, INVALID_PARAM -- */
	NEO6M_Centinela(&m);
	NEO_EXPECT(NEO_6M_Get(&m), NEO_6M_NOT_INITIALIZED);
	NEO6M_CheckSinTocar(&m);
	NEO_EXPECT(NEO_6M_Get(NULL), NEO_6M_INVALID_PARAM);

	/* -- Las conversiones son matemática pura y no dependen del driver: salida NULL
	 *    y coordenadas fuera de rango o NaN se rechazan. DMS: |grados| > 180
	 *    desbordaría dms.degrees; UTM: lat > 84, lon < -180; Distance/Bearing:
	 *    |lat| > 90, |lon| > 180 en cualquiera de los dos puntos -- */
	NEO_EXPECT(NEO_6M_DecimalToDMS(0.0, 'N', NULL), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_DecimalToDMS(181.0, 'E', &dms), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_DecimalToDMS(NAN, 'E', &dms), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_DecimalToUTM(0.0, 0.0, NULL), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_DecimalToUTM(85.0, 0.0, &utm), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_DecimalToUTM(0.0, -181.0, &utm), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_Distance(0.0, 0.0, 0.0, 0.0, NULL), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_Distance(91.0, 0.0, 0.0, 0.0, &v), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_Distance(0.0, 0.0, 0.0, 181.0, &v), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_Bearing(0.0, 0.0, 0.0, 0.0, NULL), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_Bearing(0.0, NAN, 0.0, 0.0, &v), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_Bearing(0.0, 0.0, -91.0, 0.0, &v), NEO_6M_INVALID_PARAM);

	/* -- DeInit sobre un driver sin inicializar es inofensivo (el handle es NULL y
	 *    HAL_UART_AbortReceive_IT no lo desreferencia) -- */
	NEO_CHECK(NEO_6M_DeInit());
}

static void NEO6M_TestLectura(void)
{
	NEO_6M_Data_t m;

	/* -- Lectura única: datos plausibles con fix o sin él. En interior, el módulo
	 *    emite NMEA válidos con fixQuality 0 y campos a 0 -- */
	printf(COLOR_YELLOW "[Lectura]" COLOR_RESET "\r\n");
	NEO6M_Leer(&m);
	NEO6M_Print(&m);
}

static void NEO6M_TestMuestras(void)
{
	NEO_6M_Data_t m;

	/* -- Lecturas seguidas cada NEO_PERIODO_MS (el módulo emite NMEA a 1 Hz):
	 *    ninguna debe fallar. Colócalo junto a una ventana o sal al exterior:
	 *    con fix la posición y la hora UTC avanzan; sin fix la lectura sigue
	 *    siendo válida (fixQuality 0 y la última posición conocida, o 0 si
	 *    nunca hubo fix). Si se desconecta el GPS, Get devuelve TIMEOUT -- */
	printf(COLOR_YELLOW "[Muestras]" COLOR_RESET "\r\n");
	for (uint8_t i = 1; i <= NEO_MUESTRAS; i++)
	{
		NEO6M_Leer(&m);
		printf("%2u/%u  ", i, NEO_MUESTRAS);
		NEO6M_Print(&m);
		HAL_Delay(NEO_PERIODO_MS);
	}
}

static void NEO6M_TestConversiones(void)
{
	NEO_6M_DMS_t dms;
	NEO_6M_UTM_t utm;
	double rec;

	/* -- DMS exacto: 40.5 grados son 40 grados, 30 minutos y 0 segundos; el
	 *    hemisferio se copia tal cual -- */
	printf(COLOR_YELLOW "[Conversiones]" COLOR_RESET "\r\n");
	NEO_CHECK(NEO_6M_DecimalToDMS(40.5, 'N', &dms));
	if (dms.degrees != 40 || dms.minutes != 30 || dms.seconds != 0 || dms.hemisphere != 'N')
	{
		Error_Handler();
	}

	/* -- Ida y vuelta: reconstruir el decimal desde el DMS debe diferir menos de
	 *    1 segundo de arco (los segundos se truncan, así que el error solo puede
	 *    ser por debajo del valor original). Con Madrid se cubren lat/lon reales -- */
	const double lat = 40.416775;
	const double lon = -3.703790;

	NEO_CHECK(NEO_6M_DecimalToDMS(lat, 'N', &dms));
	rec = dms.degrees + dms.minutes / 60.0 + dms.seconds / 3600.0;
	if (!(fabs(rec - lat) <= 1.0 / 3600.0))
	{
		Error_Handler();
	}

	/* La conversión trabaja sobre el valor absoluto: 3.703790 W -> 3°42'13" W */
	NEO_CHECK(NEO_6M_DecimalToDMS(lon, 'W', &dms));
	rec = dms.degrees + dms.minutes / 60.0 + dms.seconds / 3600.0;
	if (!(fabs(rec - fabs(lon)) <= 1.0 / 3600.0))
	{
		Error_Handler();
	}

	/* -- UTM con punto de control perfecto: el meridiano central de la zona 31
	 *    (3E, en el ecuador) no tiene sesgo de escala y da este 500000 / norte 0;
	 *    la banda del ecuador es N. En 75N 15E la excepción de Svalbard cambia
	 *    la zona a 33 (no la 32) y la banda es X -- */
	NEO_CHECK(NEO_6M_DecimalToUTM(0.0, 3.0, &utm));
	if (!(utm.easting >= 499999U && utm.easting <= 500001U) ||
		utm.northing > 1U || utm.zoneNumber != 31 || utm.zone != 'N')
	{
		Error_Handler();
	}

	NEO_CHECK(NEO_6M_DecimalToUTM(75.0, 15.0, &utm));
	if (utm.zoneNumber != 33 || utm.zone != 'X')
	{
		Error_Handler();
	}

	/* -- Distance: 1 grado a lo largo de un meridiano mide R·π/180; un punto
	 *    consigo mismo (Madrid), 0 -- */
	NEO_CHECK(NEO_6M_Distance(0.0, 0.0, 1.0, 0.0, &rec));
	if (!(fabs(rec - NEO_GRADO_M) <= NEO_DIST_TOL_M))
	{
		Error_Handler();
	}

	NEO_CHECK(NEO_6M_Distance(lat, lon, lat, lon, &rec));
	if (!(rec <= NEO_DIST_TOL_M))
	{
		Error_Handler();
	}

	/* -- Bearing desde el origen hacia los cuatro puntos cardinales (0, 90, 180,
	 *    270) y 0 con puntos coincidentes (rumbo indefinido) -- */
	static const double destino[4][2] = { {1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0} };
	for (uint8_t i = 0; i < 4U; i++)
	{
		NEO_CHECK(NEO_6M_Bearing(0.0, 0.0, destino[i][0], destino[i][1], &rec));
		if (!(fabs(rec - 90.0 * i) <= NEO_RUMBO_TOL))
		{
			Error_Handler();
		}
	}

	NEO_CHECK(NEO_6M_Bearing(lat, lon, lat, lon, &rec));
	if (rec != 0.0)
	{
		Error_Handler();
	}

	printf(COLOR_GREEN "Conversiones OK" COLOR_RESET "\r\n");
}

static void NEO6M_TestErrores(void)
{
	NEO_6M_Data_t m;

	/* -- Punteros de salida NULL e Init con handle NULL: se rechazan con
	 *    INVALID_PARAM sin tocar la muestra ni el estado del driver -- */
	printf(COLOR_YELLOW "[Errores]" COLOR_RESET "\r\n");
	NEO6M_Centinela(&m);
	NEO_EXPECT(NEO_6M_Get(NULL), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_DecimalToDMS(0.0, 'S', NULL), NEO_6M_INVALID_PARAM);
	NEO_EXPECT(NEO_6M_Init(NULL), NEO_6M_INVALID_PARAM);
	NEO6M_CheckSinTocar(&m);

	/* El driver conservó el UART y el búfer: sigue leyendo */
	NEO6M_Leer(&m);
	printf(COLOR_GREEN "Errores OK" COLOR_RESET "\r\n");
}

static void NEO6M_TestDeInit(void)
{
	NEO_6M_Data_t m;

	/* -- DeInit aborta la recepción (las sentencias que lleguen mientras se
	 *    pierden): repetirlo sigue devolviendo OK, y Get pasa a NOT_INITIALIZED
	 *    sin tocar la muestra -- */
	printf(COLOR_YELLOW "[DeInit]" COLOR_RESET "\r\n");
	NEO_CHECK(NEO_6M_DeInit());
	NEO_CHECK(NEO_6M_DeInit());
	NEO6M_Centinela(&m);
	NEO_EXPECT(NEO_6M_Get(&m), NEO_6M_NOT_INITIALIZED);
	NEO6M_CheckSinTocar(&m);

	/* -- Re-Init: la interrupción se rearma y las lecturas vuelven. El segundo
	 *    Init sin DeInit aborta la RX previa en vez de chocar con HAL_BUSY.
	 *    Init borra los datos: hasta la primera sentencia válida Get devuelve
	 *    TIMEOUT (una sentencia tarda ~70 ms en llegar a 9600 baudios) -- */
	NEO_CHECK(NEO_6M_Init(&huart2));
	NEO_CHECK(NEO_6M_Init(&huart2));
	NEO_EXPECT(NEO_6M_Get(&m), NEO_6M_TIMEOUT);
	HAL_Delay(NEO_ARRANQUE_MS);
	NEO6M_Leer(&m);
	NEO6M_Print(&m);
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
