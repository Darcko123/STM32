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
#include "WS2812.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Tira conectada a PA7 (SPI1 MOSI); los tests asumen 16 LEDs (mínimo 8 para el test de Re-Init) -- */
#define WS2812_NUM_LEDS 16

/* -- Parámetros de la batería de pruebas -- */
#define TEST_BRILLO         32U     /**< Brillo de trabajo (un blanco total a 255 consumiría ~1 A con 16 LEDs) */
#define TEST_PASO_MS        1500U  /**< Tiempo que se deja cada imagen para verla */
#define TEST_BARRIDO_MS     50U     /**< Retardo entre pasos de las animaciones */
#define TEST_SHOW_REPS      100U    /**< Transmisiones seguidas del test de temporización */
#define TEST_SHOW_MIN_MS    55U     /**< 100 x Show de 16 LEDs: 269 bytes a 3.125 MHz = 0.69 ms c/u -> ~70 ms (ajustar si cambia el SPI o los LEDs) */
#define TEST_SHOW_MAX_MS    120U    /**< Cota superior: detecta una transmisión mucho más lenta de lo esperado */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define WS_CHECK(x)  do { if ((x) != WS2812_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define WS_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)
/* Para los tests de valores: la condición debe cumplirse */
#define WS_ASSERT(c)  do { if (!(c)) { Error_Handler(); } } while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
WS2812_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void WS2812_Llenar(uint8_t r, uint8_t g, uint8_t b);
static void WS2812_TestSuite(void);
static void WS2812_TestPreInit(void);
static void WS2812_TestColorHSV(void);
static void WS2812_TestSetPixel(void);
static void WS2812_TestFill(void);
static void WS2812_TestClear(void);
static void WS2812_TestBrillo(void);
static void WS2812_TestShow(void);
static void WS2812_TestErrores(void);
static void WS2812_TestDeInit(void);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf(CLEAR_SCREEN CURSOR_HOME COLOR_BOLD COLOR_CYAN "=== Pruebas WS2812 (%u LEDs) ===" COLOR_RESET "\r\n", (unsigned)WS2812_NUM_LEDS);
  WS2812_TestPreInit();

  status = WS2812_Init(&hspi1, WS2812_NUM_LEDS);
  if(status != WS2812_OK)
  {
	  Error_Handler();
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  /* Init deja el brillo en 255; se baja al de trabajo antes de encender nada */
  WS_ASSERT(WS2812_GetBrightness() == 255U);
  WS_CHECK(WS2812_SetBrightness(TEST_BRILLO));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    WS2812_TestSuite();
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

/* Rellena toda la tira con un color y lo transmite */
static void WS2812_Llenar(uint8_t r, uint8_t g, uint8_t b)
{
	WS_CHECK(WS2812_Fill(r, g, b, 0, 0));
	WS_CHECK(WS2812_Show());
}

/* Con el módulo sin inicializar, todo (salvo ColorHSV y GetBrightness) debe devolver NOT_INITIALIZED */
static void WS2812_CheckSinInit(void)
{
	WS_EXPECT(WS2812_SetPixelColor(0, 1, 2, 3), WS2812_NOT_INITIALIZED);
	WS_EXPECT(WS2812_SetPixelColorPacked(0, 0x010203UL), WS2812_NOT_INITIALIZED);
	WS_EXPECT(WS2812_Fill(1, 2, 3, 0, 0), WS2812_NOT_INITIALIZED);
	WS_EXPECT(WS2812_SetBrightness(10), WS2812_NOT_INITIALIZED);
	WS_EXPECT(WS2812_Clear(), WS2812_NOT_INITIALIZED);
	WS_EXPECT(WS2812_Show(), WS2812_NOT_INITIALIZED);
	WS_EXPECT(WS2812_DeInit(), WS2812_NOT_INITIALIZED);

	/* ColorHSV es una función pura: no depende de Init */
	WS_ASSERT(WS2812_ColorHSV(21845, 255, 255) == 0x00FF00UL);
}

/* Apaga la tira, espera y deja el brillo de trabajo: separa un test del siguiente */
static void WS2812_Siguiente(void)
{
	WS_CHECK(WS2812_Clear());
	WS_CHECK(WS2812_Show());
	HAL_Delay(2000);
}

static void WS2812_TestSuite(void)
{
	WS2812_TestColorHSV();
	WS2812_Siguiente();

	WS2812_TestSetPixel();
	WS2812_Siguiente();

	WS2812_TestFill();
	WS2812_Siguiente();

	WS2812_TestClear();
	WS2812_Siguiente();

	WS2812_TestBrillo();
	WS2812_Siguiente();

	WS2812_TestShow();
	WS2812_Siguiente();

	WS2812_TestErrores();
	WS2812_Siguiente();

	WS2812_TestDeInit();
	WS2812_Siguiente();
}

static void WS2812_TestPreInit(void)
{
	/* -- Antes de Init: nada toca el bus y GetBrightness devuelve 255 -- */
	printf(COLOR_YELLOW "[PreInit]" COLOR_RESET "\r\n");
	WS2812_CheckSinInit();
	WS_ASSERT(WS2812_GetBrightness() == 255U);

	printf(COLOR_GREEN "PreInit OK" COLOR_RESET "\r\n");
}

static void WS2812_TestColorHSV(void)
{
	static const struct { uint16_t hue; uint8_t sat; uint8_t val; uint32_t rgb; } puntos[] = {
		/* -- Los seis vértices del círculo cromático (sat y val al máximo) -- */
		{     0, 255, 255, 0xFF0000UL },   /* rojo */
		{ 10923, 255, 255, 0xFFFF00UL },   /* amarillo */
		{ 21845, 255, 255, 0x00FF00UL },   /* verde */
		{ 32768, 255, 255, 0x00FFFFUL },   /* cian */
		{ 43690, 255, 255, 0x0000FFUL },   /* azul */
		{ 54613, 255, 255, 0xFF00FFUL },   /* magenta */
		{ 65535, 255, 255, 0xFF0000UL },   /* el final de la vuelta redondea a rojo */
		{  5000, 255, 255, 0xFF7500UL },   /* tramo intermedio rojo -> amarillo */
		/* -- Saturación y valor -- */
		{ 12345,   0,   0, 0x000000UL },   /* val = 0: negro */
		{     0,   0, 255, 0xFFFFFFUL },   /* sat = 0: blanco */
		{ 21845,   0, 128, 0x808080UL },   /* sat = 0 ignora el matiz: gris */
		{     0, 255, 128, 0x800000UL },   /* valor a la mitad */
		{     0, 128, 255, 0xFF7F7FUL },   /* saturación a la mitad: rojo mezclado con blanco */
	};

	printf(COLOR_YELLOW "[ColorHSV]" COLOR_RESET "\r\n");
	for (uint8_t i = 0; i < sizeof(puntos) / sizeof(puntos[0]); i++)
	{
		WS_ASSERT(WS2812_ColorHSV(puntos[i].hue, puntos[i].sat, puntos[i].val) == puntos[i].rgb);
	}

	/* -- Arcoíris estático con SetPixelColorPacked: rojo -> verde -> azul -> rojo a lo largo de la tira -- */
	printf("Debe verse un arcoiris completo, y luego girando\r\n");
	WS_CHECK(WS2812_Clear());
	for (uint16_t i = 0; i < WS2812_NUM_LEDS; i++)
	{
		WS_CHECK(WS2812_SetPixelColorPacked(i, WS2812_ColorHSV((uint16_t)((uint32_t)i * 65536UL / WS2812_NUM_LEDS), 255, 255)));
	}
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	for (uint16_t paso = 0; paso < 128U; paso++)
	{
		for (uint16_t i = 0; i < WS2812_NUM_LEDS; i++)
		{
			uint16_t hue = (uint16_t)((uint32_t)i * 65536UL / WS2812_NUM_LEDS + (uint32_t)paso * 1024UL);
			WS_CHECK(WS2812_SetPixelColorPacked(i, WS2812_ColorHSV(hue, 255, 255)));
		}
		WS_CHECK(WS2812_Show());
		HAL_Delay(TEST_BARRIDO_MS / 2U);
	}

	printf(COLOR_GREEN "ColorHSV OK" COLOR_RESET "\r\n");
}

static void WS2812_TestSetPixel(void)
{
	static const uint8_t colores[3][3] = { {255, 0, 0}, {0, 255, 0}, {0, 0, 255} };

	printf(COLOR_YELLOW "[SetPixel]" COLOR_RESET "\r\n");

	/* -- Orden de colores: si rojo y verde salen cambiados, el orden GRB de la tira está mal -- */
	printf("Debe verse: LED 0 rojo, LED 1 verde, LED 2 azul, ultimo LED blanco\r\n");
	WS_CHECK(WS2812_Clear());
	WS_CHECK(WS2812_SetPixelColor(0, 255, 0, 0));
	WS_CHECK(WS2812_SetPixelColor(1, 0, 255, 0));
	WS_CHECK(WS2812_SetPixelColor(2, 0, 0, 255));
	WS_CHECK(WS2812_SetPixelColor(WS2812_NUM_LEDS - 1U, 255, 255, 255));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	/* -- Sobrescribir un píxel no toca a los demás (no hay Clear entre medias) -- */
	printf("Debe verse: igual, pero el LED 0 ahora amarillo\r\n");
	WS_CHECK(WS2812_SetPixelColor(0, 255, 255, 0));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	/* -- El color empaquetado ignora el byte alto (0xFF en bits 31-24 no cambia nada) -- */
	printf("Debe verse: el LED 0 en rojo\r\n");
	WS_CHECK(WS2812_SetPixelColorPacked(0, 0xFFFF0000UL));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	/* -- Persecución de un píxel en rojo, verde y azul: recorre desde el primero hasta el último LED -- */
	printf("Debe verse un LED recorriendo la tira en rojo, verde y azul\r\n");
	for (uint8_t c = 0; c < 3U; c++)
	{
		for (uint16_t i = 0; i < WS2812_NUM_LEDS; i++)
		{
			WS_CHECK(WS2812_Clear());
			WS_CHECK(WS2812_SetPixelColor(i, colores[c][0], colores[c][1], colores[c][2]));
			WS_CHECK(WS2812_Show());
			HAL_Delay(TEST_BARRIDO_MS);
		}
	}

	printf(COLOR_GREEN "SetPixel OK" COLOR_RESET "\r\n");
}

static void WS2812_TestFill(void)
{
	printf(COLOR_YELLOW "[Fill]" COLOR_RESET "\r\n");

	/* -- count = 0: desde first hasta el final de la tira -- */
	printf("Debe verse: toda la tira en rojo\r\n");
	WS_CHECK(WS2812_Clear());
	WS_CHECK(WS2812_Fill(255, 0, 0, 0, 0));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	/* -- Rango interior: solo cuatro LEDs -- */
	printf("Debe verse: solo los LEDs 4-7 en verde\r\n");
	WS_CHECK(WS2812_Clear());
	WS_CHECK(WS2812_Fill(0, 255, 0, 4, 4));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	/* -- count excedente: se recorta en el último LED, sin error (y sin tocar los verdes) -- */
	printf("Debe verse: LEDs 4-7 verdes y 12-15 azules\r\n");
	WS_CHECK(WS2812_Fill(0, 0, 255, 12, 100));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	/* -- count = 0 con first > 0: hasta el final, sobrescribe los azules -- */
	printf("Debe verse: LEDs 4-7 verdes y 8-15 amarillos\r\n");
	WS_CHECK(WS2812_Fill(255, 255, 0, 8, 0));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	/* -- Un solo LED en cada extremo -- */
	printf("Debe verse: lo anterior, con los LEDs 0 y 15 en magenta\r\n");
	WS_CHECK(WS2812_Fill(255, 0, 255, 0, 1));
	WS_CHECK(WS2812_Fill(255, 0, 255, WS2812_NUM_LEDS - 1U, 1));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	printf(COLOR_GREEN "Fill OK" COLOR_RESET "\r\n");
}

static void WS2812_TestClear(void)
{
	printf(COLOR_YELLOW "[Clear]" COLOR_RESET "\r\n");

	printf("Debe verse: toda la tira en blanco tenue\r\n");
	WS2812_Llenar(255, 255, 255);
	HAL_Delay(TEST_PASO_MS);

	/* -- Clear solo borra el buffer: la tira no cambia hasta el siguiente Show -- */
	printf("Debe verse: sigue en blanco (Clear no transmite)\r\n");
	WS_CHECK(WS2812_Clear());
	HAL_Delay(TEST_PASO_MS);

	printf("Debe verse: tira apagada\r\n");
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	/* -- Tras Clear los píxeles quedan en negro, también para los siguientes Set -- */
	printf("Debe verse: solo el LED 3 en blanco\r\n");
	WS_CHECK(WS2812_SetPixelColor(3, 255, 255, 255));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	printf(COLOR_GREEN "Clear OK" COLOR_RESET "\r\n");
}

static void WS2812_TestBrillo(void)
{
	WS_ASSERT(WS2812_GetBrightness() == TEST_BRILLO);
	printf(COLOR_YELLOW "[Brillo]" COLOR_RESET "\r\n");

	/* -- Get devuelve lo último fijado, para los 256 valores (también repetidos) -- */
	for (uint16_t v = 0; v < 256U; v++)
	{
		WS_CHECK(WS2812_SetBrightness((uint8_t)v));
		WS_ASSERT(WS2812_GetBrightness() == (uint8_t)v);
		WS_CHECK(WS2812_SetBrightness((uint8_t)v));
		WS_ASSERT(WS2812_GetBrightness() == (uint8_t)v);
	}

	/* -- SetBrightness reescala lo ya almacenado -- */
	printf("Debe verse: rojo brillante, luego mas tenue\r\n");
	WS_CHECK(WS2812_Clear());
	WS_CHECK(WS2812_SetBrightness(255));
	WS2812_Llenar(255, 0, 0);
	HAL_Delay(TEST_PASO_MS);
	WS_CHECK(WS2812_SetBrightness(TEST_BRILLO));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	/* -- Brillo 0 apaga; el reescalado pierde datos, así que subirlo no recupera el color: hay que rellenar -- */
	printf("Debe verse: tira apagada (brillo 0), sigue apagada al subir a 255, y rojo al rellenar\r\n");
	WS_CHECK(WS2812_SetBrightness(0));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);
	WS_CHECK(WS2812_SetBrightness(255));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);
	WS2812_Llenar(255, 0, 0);
	HAL_Delay(TEST_PASO_MS);

	/* -- "Respiración" de brillo: el magenta se rellena en cada paso -- */
	printf("Debe verse: magenta subiendo y bajando de brillo\r\n");
	for (int16_t b = 0; b <= 255; b += 5)
	{
		WS_CHECK(WS2812_SetBrightness((uint8_t)b));
		WS_CHECK(WS2812_Fill(255, 0, 255, 0, 0));
		WS_CHECK(WS2812_Show());
		HAL_Delay(10);
	}
	for (int16_t b = 255; b >= 0; b -= 5)
	{
		WS_CHECK(WS2812_SetBrightness((uint8_t)b));
		WS_CHECK(WS2812_Fill(255, 0, 255, 0, 0));
		WS_CHECK(WS2812_Show());
		HAL_Delay(10);
	}

	/* Vuelve al brillo de trabajo para los demás tests */
	WS_CHECK(WS2812_SetBrightness(TEST_BRILLO));
	printf(COLOR_GREEN "Brillo OK" COLOR_RESET "\r\n");
}

static void WS2812_TestShow(void)
{
	printf(COLOR_YELLOW "[Show]" COLOR_RESET "\r\n");

	/* -- Show no modifica el buffer: transmitir dos veces seguidas deja la misma imagen -- */
	printf("Debe verse: toda la tira en azul, sin parpadeos\r\n");
	WS2812_Llenar(0, 0, 255);
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	/* -- Temporización: 100 transmisiones de 269 bytes a 3.125 MHz deben tardar ~70 ms; muy por debajo
	 *    indica que Show no envía todos los bytes, muy por encima, un SPI más lento de lo configurado -- */
	uint32_t t0 = HAL_GetTick();
	for (uint16_t i = 0; i < TEST_SHOW_REPS; i++)
	{
		WS_CHECK(WS2812_Show());
	}
	uint32_t dt = HAL_GetTick() - t0;
	printf("%u x Show = %lu ms (esperado %u-%u ms)\r\n", (unsigned)TEST_SHOW_REPS, (unsigned long)dt,
	       (unsigned)TEST_SHOW_MIN_MS, (unsigned)TEST_SHOW_MAX_MS);
	WS_ASSERT(dt >= TEST_SHOW_MIN_MS && dt <= TEST_SHOW_MAX_MS);

	/* -- Un fallo del HAL (SPI ocupado -> HAL_BUSY) se traduce en WS2812_ERROR y el driver se recupera.
	 *    WS2812_TIMEOUT no se puede provocar así: depende de WS2812_SPI_TIMEOUT_MS -- */
	hspi1.State = HAL_SPI_STATE_BUSY_TX;
	WS_EXPECT(WS2812_Show(), WS2812_ERROR);
	hspi1.State = HAL_SPI_STATE_READY;
	WS_CHECK(WS2812_Show());

	printf(COLOR_GREEN "Show OK" COLOR_RESET "\r\n");
}

static void WS2812_TestErrores(void)
{
	SPI_HandleTypeDef h;

	printf(COLOR_YELLOW "[Errores]" COLOR_RESET "\r\n");

	/* -- Índices fuera de la tira: se rechazan sin escribir nada -- */
	WS_EXPECT(WS2812_SetPixelColor(WS2812_NUM_LEDS, 255, 255, 255), WS2812_INVALID_PARAM);
	WS_EXPECT(WS2812_SetPixelColor(0xFFFF, 255, 255, 255), WS2812_INVALID_PARAM);
	WS_EXPECT(WS2812_SetPixelColorPacked(WS2812_NUM_LEDS, 0xFFFFFFUL), WS2812_INVALID_PARAM);
	WS_EXPECT(WS2812_Fill(255, 255, 255, WS2812_NUM_LEDS, 1), WS2812_INVALID_PARAM);
	WS_EXPECT(WS2812_Fill(255, 255, 255, 0xFFFF, 0), WS2812_INVALID_PARAM);

	/* -- count enorme no desborda first + count: se recorta en el último LED -- */
	WS_CHECK(WS2812_Fill(0, 0, 0, WS2812_NUM_LEDS - 1U, 0xFFFF));
	WS_CHECK(WS2812_Fill(0, 0, 0, 0, 0xFFFF));

	/* -- Init con parámetros inválidos: se rechaza antes de tocar el estado del driver -- */
	WS_EXPECT(WS2812_Init(NULL, WS2812_NUM_LEDS), WS2812_INVALID_PARAM);
	WS_EXPECT(WS2812_Init(&hspi1, 0), WS2812_INVALID_PARAM);
	WS_EXPECT(WS2812_Init(&hspi1, WS2812_MAX_LEDS + 1U), WS2812_INVALID_PARAM);

	/* -- Init con un SPI mal configurado (cada caso sobre una copia del handle de SPI1) -- */
	h = hspi1; h.Instance = NULL;
	WS_EXPECT(WS2812_Init(&h, WS2812_NUM_LEDS), WS2812_INVALID_PARAM);
	h = hspi1; h.State = HAL_SPI_STATE_RESET;
	WS_EXPECT(WS2812_Init(&h, WS2812_NUM_LEDS), WS2812_INVALID_PARAM);
	h = hspi1; h.Init.Mode = SPI_MODE_SLAVE;
	WS_EXPECT(WS2812_Init(&h, WS2812_NUM_LEDS), WS2812_INVALID_PARAM);
	h = hspi1; h.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
	WS_EXPECT(WS2812_Init(&h, WS2812_NUM_LEDS), WS2812_INVALID_PARAM);
	h = hspi1; h.Init.DataSize = SPI_DATASIZE_16BIT;
	WS_EXPECT(WS2812_Init(&h, WS2812_NUM_LEDS), WS2812_INVALID_PARAM);
	h = hspi1; h.Init.FirstBit = SPI_FIRSTBIT_LSB;
	WS_EXPECT(WS2812_Init(&h, WS2812_NUM_LEDS), WS2812_INVALID_PARAM);

	/* -- Tras los rechazos el driver sigue operativo y con la misma tira (último LED sí, el siguiente no) -- */
	printf("Debe verse: solo el ultimo LED en verde\r\n");
	WS_CHECK(WS2812_Clear());
	WS_CHECK(WS2812_SetPixelColor(WS2812_NUM_LEDS - 1U, 0, 255, 0));
	WS_EXPECT(WS2812_SetPixelColor(WS2812_NUM_LEDS, 0, 255, 0), WS2812_INVALID_PARAM);
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	printf(COLOR_GREEN "Errores OK" COLOR_RESET "\r\n");
}

static void WS2812_TestDeInit(void)
{
	printf(COLOR_YELLOW "[DeInit]" COLOR_RESET "\r\n");

	/* -- Re-Init sin DeInit previo, con menos LEDs: libera, reasigna, apaga los píxeles y devuelve el brillo a 255 -- */
	printf("Debe verse: tira en rojo, luego LEDs 0-7 apagados (los 8-15 ya no reciben datos y conservan el rojo)\r\n");
	WS_CHECK(WS2812_SetBrightness(100));
	WS2812_Llenar(255, 0, 0);
	HAL_Delay(TEST_PASO_MS);
	WS_CHECK(WS2812_Init(&hspi1, 8));
	WS_ASSERT(WS2812_GetBrightness() == 255U);
	WS_CHECK(WS2812_SetPixelColor(7, 0, 0, 0));
	WS_EXPECT(WS2812_SetPixelColor(8, 0, 0, 0), WS2812_INVALID_PARAM);
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	/* -- Tira de un solo LED: el mínimo permitido -- */
	printf("Debe verse: el LED 0 en azul tenue\r\n");
	WS_CHECK(WS2812_Init(&hspi1, 1));
	WS_CHECK(WS2812_SetBrightness(TEST_BRILLO));
	WS_CHECK(WS2812_SetPixelColor(0, 0, 0, 255));
	WS_EXPECT(WS2812_SetPixelColor(1, 0, 0, 255), WS2812_INVALID_PARAM);
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

	/* -- DeInit: todo vuelve a NOT_INITIALIZED, incluido un segundo DeInit -- */
	WS_CHECK(WS2812_DeInit());
	WS2812_CheckSinInit();

	/* -- Init de nuevo con la tira completa: píxeles en negro y brillo al máximo -- */
	printf("Debe verse: tira completamente apagada\r\n");
	WS_CHECK(WS2812_Init(&hspi1, WS2812_NUM_LEDS));
	WS_ASSERT(WS2812_GetBrightness() == 255U);
	WS_CHECK(WS2812_SetBrightness(TEST_BRILLO));
	WS_CHECK(WS2812_Show());
	HAL_Delay(TEST_PASO_MS);

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
