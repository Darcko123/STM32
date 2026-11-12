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
#include "ILI9341.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Orientación de trabajo de la batería de pruebas (la rotación se restaura a esta) */
#define TEST_ORIENTATION  ILI9341_Orientation_Landscape_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ILI_CHECK(x)  do { if ((x) != ILI9341_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define ILI_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

ILI9341_Status_t status;

/* La librería no expone GetWidth/GetHeight: se lleva aquí la orientación activa
 * para conocer las dimensiones (240x320 en vertical, 320x240 en horizontal) */
static ILI9341_Orientation_t Test_Orientation = ILI9341_Orientation_Portrait_1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t Test_Width(void);
static uint16_t Test_Height(void);
static ILI9341_Status_t Test_Rotate(ILI9341_Orientation_t orientation);
static void ILI9341_TestSuite(void);
static void ILI9341_TestPreInit(void);
static void ILI9341_TestFonts(void);
static void ILI9341_TestPutc(void);
static void ILI9341_TestPrintf(void);
static void ILI9341_TestPixeles(void);
static void ILI9341_TestLineas(void);
static void ILI9341_TestLineasGruesas(void);
static void ILI9341_TestRectangulos(void);
static void ILI9341_TestTriangulos(void);
static void ILI9341_TestCirculos(void);
static void ILI9341_TestElipses(void);
static void ILI9341_TestArc(void);
static void ILI9341_TestColores(void);
static void ILI9341_TestRotacion(void);
static void ILI9341_TestErrores(void);
static void ILI9341_TestDeInit(void);
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
  ILI9341_TestPreInit();

  /* El SPI2 queda a 2 Mbit/s (16 MHz / 8) como pide la librería; Init lo sube a /2 */
  status = ILI9341_Init(&hspi2,
		  	  	  	    ILI_CS_GPIO_Port, ILI_CS_Pin,
						ILI_RES_GPIO_Port, ILI_RES_Pin,
						ILI_DC_GPIO_Port, ILI_DC_Pin);
  if(status != ILI9341_OK)
  {
	  Error_Handler();
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }

  ILI_CHECK(Test_Rotate(TEST_ORIENTATION));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ILI9341_TestSuite();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  HAL_GPIO_WritePin(GPIOB, ILI_CS_Pin|ILI_RES_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ILI_DC_GPIO_Port, ILI_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ILI_CS_Pin ILI_RES_Pin */
  GPIO_InitStruct.Pin = ILI_CS_Pin|ILI_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ILI_DC_Pin */
  GPIO_InitStruct.Pin = ILI_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ILI_DC_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static uint16_t Test_Width(void)
{
	return (Test_Orientation >= ILI9341_Orientation_Landscape_1) ? ILI9341_HEIGHT : ILI9341_WIDTH;
}

static uint16_t Test_Height(void)
{
	return (Test_Orientation >= ILI9341_Orientation_Landscape_1) ? ILI9341_WIDTH : ILI9341_HEIGHT;
}

/* Rotate + registro de la orientación activa (solo si la librería la aceptó) */
static ILI9341_Status_t Test_Rotate(ILI9341_Orientation_t orientation)
{
	ILI9341_Status_t st = ILI9341_Rotate(orientation);

	if (st == ILI9341_OK)
	{
		Test_Orientation = orientation;
	}
	return st;
}

static void ILI9341_TestSuite(void)
{
	ILI9341_TestFonts();
	HAL_Delay(2000);

	ILI9341_TestPutc();
	HAL_Delay(2000);

	ILI9341_TestPrintf();
	HAL_Delay(2000);

	ILI9341_TestPixeles();
	HAL_Delay(2000);

	ILI9341_TestLineas();
	HAL_Delay(2000);

	ILI9341_TestLineasGruesas();
	HAL_Delay(2000);

	ILI9341_TestRectangulos();
	HAL_Delay(2000);

	ILI9341_TestTriangulos();
	HAL_Delay(2000);

	ILI9341_TestCirculos();
	HAL_Delay(2000);

	ILI9341_TestElipses();
	HAL_Delay(2000);

	ILI9341_TestArc();
	HAL_Delay(2000);

	ILI9341_TestColores();
	HAL_Delay(2000);

	ILI9341_TestRotacion();
	HAL_Delay(2000);

	ILI9341_TestErrores();
	HAL_Delay(2000);

	ILI9341_TestDeInit();
	HAL_Delay(2000);
}

static void ILI9341_TestPreInit(void)
{
	/* -- Antes de Init: todo debe devolver NOT_INITIALIZED sin tocar el bus -- */
	ILI_EXPECT(ILI9341_Fill(ILI9341_COLOR_WHITE), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_DrawPixel(0, 0, ILI9341_COLOR_WHITE), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_DrawFastHLine(0, 0, 10, ILI9341_COLOR_WHITE), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_DrawFilledRectangle(0, 0, 9, 9, ILI9341_COLOR_WHITE), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Putc(0, 0, 'X', &LCD_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Rotate(ILI9341_Orientation_Portrait_1), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_TP_ConfigXPT2046(GPIOB, GPIO_PIN_0, NULL, 0), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_DeInit(), ILI9341_NOT_INITIALIZED);
	if (ILI9341_TP_GetState() != NULL)
	{
		Error_Handler();
	}
}

static void ILI9341_TestFonts(void)
{
	const uint16_t bg = ILI9341_COLOR_BLACK;
	uint16_t tw, th;

	/* -- Texto con las tres fuentes y colores distintos; '\n' baja FontHeight + 1 px y vuelve a x inicial -- */
	ILI_CHECK(ILI9341_Fill(bg));
	ILI_CHECK(ILI9341_Puts(0, 0, "ILI9341 TEST", &LCD_Font_16x26, ILI9341_COLOR_WHITE, bg));
	ILI_CHECK(ILI9341_Puts(0, 30, "Font 11x18", &LCD_Font_11x18, ILI9341_COLOR_YELLOW, bg));
	ILI_CHECK(ILI9341_Puts(0, 52, "Font 7x10", &LCD_Font_7x10, ILI9341_COLOR_CYAN, bg));
	ILI_CHECK(ILI9341_Puts(0, 66, "ABCDEFGHIJKLMNOPQRSTUVWXYZ\nabcdefghijklmnopqrstuvwxyz\n0123456789 !?#$%&()*+-=",
						   &LCD_Font_7x10, ILI9341_COLOR_WHITE, bg));

	/* Esquina inferior derecha: verifica GetStringSize y los bordes
	 * (GetStringSize cuenta todos los caracteres: solo sirve para cadenas de una línea) */
	ILI9341_GetStringSize("Fin", &LCD_Font_11x18, &tw, &th);
	ILI_CHECK(ILI9341_Puts(Test_Width() - tw, Test_Height() - th, "Fin",
						   &LCD_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_GREEN));
}

static void ILI9341_TestPutc(void)
{
	const uint16_t bg = ILI9341_COLOR_BLACK;
	const uint16_t w  = Test_Width();
	const uint16_t h  = Test_Height();
	LCD_FontDef_t* f  = &LCD_Font_11x18;

	/* -- Caracteres sueltos colocados a mano, avanzando FontWidth columnas -- */
	ILI_CHECK(ILI9341_Fill(bg));
	for (char c = 'A'; c <= 'J'; c++)
	{
		ILI_CHECK(ILI9341_Putc((uint16_t)(c - 'A') * f->FontWidth, 0, c, f, ILI9341_COLOR_WHITE, bg));
	}

	/* -- Salto automático: la celda no cabe a 3 px del borde derecho -> sale en x = 0, y = 30 + FontHeight -- */
	ILI_CHECK(ILI9341_Putc(w - 3, 30, 'W', f, ILI9341_COLOR_YELLOW, bg));

	/* -- Última celda que cabe, pegada a la esquina inferior derecha -- */
	ILI_CHECK(ILI9341_Putc(w - f->FontWidth, h - f->FontHeight, 'Z', f, ILI9341_COLOR_BLACK, ILI9341_COLOR_GREEN));

	/* -- No imprimibles: se ignoran y devuelven OK sin pintar (no debe verse ningún bloque rojo) -- */
	ILI_CHECK(ILI9341_Putc(0, 80, '\n', f, ILI9341_COLOR_RED, ILI9341_COLOR_RED));
	ILI_CHECK(ILI9341_Putc(0, 80, (char)127, f, ILI9341_COLOR_RED, ILI9341_COLOR_RED));

	/* -- Error: fuente NULL -- */
	ILI_EXPECT(ILI9341_Putc(0, 0, 'X', NULL, ILI9341_COLOR_RED, bg), ILI9341_INVALID_PARAM);
}

static void ILI9341_TestPrintf(void)
{
	const uint16_t bg = ILI9341_COLOR_BLACK;
	const uint16_t w  = Test_Width();
	const uint16_t h  = Test_Height();

	/* -- Alineación dentro de la región [0, w - 1] -- */
	ILI_CHECK(ILI9341_Fill(bg));
	ILI_CHECK(ILI9341_PutsAligned(0, w - 1, 0, ILI9341_ALIGN_LEFT, "Izquierda", &LCD_Font_11x18, ILI9341_COLOR_YELLOW, bg));
	ILI_CHECK(ILI9341_PutsAligned(0, w - 1, 22, ILI9341_ALIGN_CENTER, "Centro", &LCD_Font_11x18, ILI9341_COLOR_CYAN, bg));
	ILI_CHECK(ILI9341_PutsAligned(0, w - 1, 44, ILI9341_ALIGN_RIGHT, "Derecha", &LCD_Font_11x18, ILI9341_COLOR_GREEN, bg));

	/* -- Cadena más ancha que la región (9 x 7 = 63 px > 50 px): se alinea contra x0 aunque pida derecha -- */
	ILI_CHECK(ILI9341_PutsAligned(0, 49, 68, ILI9341_ALIGN_RIGHT, "Mas ancha", &LCD_Font_7x10, ILI9341_COLOR_WHITE, bg));

	/* -- PrintfAligned: formato + alineación en una sola llamada -- */
	ILI_CHECK(ILI9341_PrintfAligned(0, w - 1, 84, ILI9341_ALIGN_CENTER, &LCD_Font_7x10,
									ILI9341_COLOR_WHITE, bg, "%ux%u px  0x%04X", (unsigned)w, (unsigned)h, 0xBEEF));

	/* -- Printf: contador que se sobrescribe sin Fill (Puts pinta el fondo de cada celda) -- */
	for (uint8_t i = 0; i <= 9; i++)
	{
		ILI_CHECK(ILI9341_Printf(0, 110, &LCD_Font_16x26, ILI9341_COLOR_WHITE, bg, "Cuenta: %u", (unsigned)i));
		HAL_Delay(300);
	}
}

static void ILI9341_TestPixeles(void)
{
	const uint16_t w = Test_Width();
	const uint16_t h = Test_Height();

	/* -- Rejilla de puntos cada 8 px: blanca en la mitad izquierda, amarilla en la derecha -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	for (uint16_t y = 0; y < h; y += 8)
	{
		for (uint16_t x = 0; x < w; x += 8)
		{
			ILI_CHECK(ILI9341_DrawPixel(x, y, (x < w / 2) ? ILI9341_COLOR_WHITE : ILI9341_COLOR_YELLOW));
		}
	}

	/* -- Un píxel rojo en cada esquina: deben verse los cuatro, pegados al borde.
	 *    DrawPixel no recorta ni valida: no se prueban coordenadas fuera de pantalla -- */
	ILI_CHECK(ILI9341_DrawPixel(0, 0, ILI9341_COLOR_RED));
	ILI_CHECK(ILI9341_DrawPixel(w - 1, 0, ILI9341_COLOR_RED));
	ILI_CHECK(ILI9341_DrawPixel(0, h - 1, ILI9341_COLOR_RED));
	ILI_CHECK(ILI9341_DrawPixel(w - 1, h - 1, ILI9341_COLOR_RED));
}

static void ILI9341_TestLineas(void)
{
	const uint16_t w = Test_Width();
	const uint16_t h = Test_Height();

	/* -- Lineas: marco, diagonales y ejes centrales (las coordenadas de rectángulo son esquinas inclusivas) -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawRectangle(0, 0, w - 1, h - 1, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawLine(0, 0, w - 1, h - 1, ILI9341_COLOR_RED));
	ILI_CHECK(ILI9341_DrawLine(0, h - 1, w - 1, 0, ILI9341_COLOR_GREEN));
	ILI_CHECK(ILI9341_DrawFastHLine(0, h / 2, w, ILI9341_COLOR_BLUE));
	ILI_CHECK(ILI9341_DrawFastVLine(w / 2, 0, h, ILI9341_COLOR_YELLOW));

	/* -- Recorte: líneas rápidas que sobresalen 20 px por cada lado y con largo negativo (sin error) -- */
	ILI_CHECK(ILI9341_DrawFastHLine(-20, h / 4, w + 40, ILI9341_COLOR_CYAN));
	ILI_CHECK(ILI9341_DrawFastHLine(w - 1, 3 * h / 4, -(int16_t)w, ILI9341_COLOR_MAGENTA));
	ILI_CHECK(ILI9341_DrawFastVLine(w / 4, -20, h + 40, ILI9341_COLOR_CYAN));
	ILI_CHECK(ILI9341_DrawFastVLine(3 * w / 4, h - 1, -(int16_t)h, ILI9341_COLOR_MAGENTA));
}

static void ILI9341_TestLineasGruesas(void)
{
	const uint16_t w = Test_Width();
	const uint16_t h = Test_Height();
	static const uint16_t grosores[] = { 1, 3, 6, 9 };

	/* -- Horizontales con grosor 1, 3, 6 y 9 (0 y 1 equivalen a DrawLine), centradas en su y -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	for (uint8_t i = 0; i < 4; i++)
	{
		ILI_CHECK(ILI9341_DrawThickLine(10, 15 + i * 25, w - 11, 15 + i * 25, grosores[i], ILI9341_COLOR_WHITE));
	}

	/* -- Vertical y diagonales con grosor 5 -- */
	ILI_CHECK(ILI9341_DrawThickLine(w / 2, 115, w / 2, h - 10, 5, ILI9341_COLOR_YELLOW));
	ILI_CHECK(ILI9341_DrawThickLine(10, h - 10, w / 2 - 20, 115, 5, ILI9341_COLOR_ORANGE));
	ILI_CHECK(ILI9341_DrawThickLine(w / 2 + 20, 115, w - 11, h - 10, 5, ILI9341_COLOR_CYAN));
}

static void ILI9341_TestRectangulos(void)
{
	const uint16_t w = Test_Width();
	const uint16_t h = Test_Height();
	const uint16_t q = w / 4;

	/* -- Rectangulos: contorno, relleno, redondeado y redondeado relleno (de izq. a der.).
	 *    El relleno va en dos mitades (rojo arriba, naranja abajo) que deben quedar
	 *    alineadas y sin hueco entre ellas -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawRectangle(0, 0, q - 5, h - 1, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawFilledRectangle(q, 0, 2 * q - 5, h / 2 - 1, ILI9341_COLOR_RED));
	ILI_CHECK(ILI9341_DrawFilledRectangle(q, h / 2, 2 * q - 5, h - 1, ILI9341_COLOR_ORANGE));
	ILI_CHECK(ILI9341_DrawRoundRect(2 * q, 0, 3 * q - 5, h - 1, 15, ILI9341_COLOR_GREEN));
	ILI_CHECK(ILI9341_DrawFilledRoundRect(3 * q, 0, w - 1, h - 1, 15, ILI9341_COLOR_BLUE));
}

static void ILI9341_TestTriangulos(void)
{
	const uint16_t w = Test_Width();
	const uint16_t h = Test_Height();

	/* -- Triangulos: contorno a la izquierda, relleno a la derecha (vértices en los bordes) -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawTriangle(w / 4, 0, 0, h - 1, w / 2 - 5, h - 1, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawFilledTriangle(w / 2 + 4, 0, w - 1, 0, 3 * w / 4, h - 1, ILI9341_COLOR_MAGENTA));
}

static void ILI9341_TestCirculos(void)
{
	const int16_t w = (int16_t)Test_Width();
	const int16_t h = (int16_t)Test_Height();
	/* El radio lo limita el lado corto para que funcione en cualquier orientación */
	const int16_t r = ((w < h) ? w : h) / 4 - 2;

	/* -- Circulos: contorno a la izquierda (toca el borde izq.), relleno a la derecha (toca el borde der.) -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawCircle(r, h / 2, r, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawFilledCircle(w - 1 - r, h / 2, r, ILI9341_COLOR_ORANGE));

	/* -- Recorte: centros sobre el borde superior e inferior, solo se ve media circunferencia (sin error) -- */
	ILI_CHECK(ILI9341_DrawCircle(w / 2, 0, r, ILI9341_COLOR_CYAN));
	ILI_CHECK(ILI9341_DrawFilledCircle(w / 2, h - 1, r, ILI9341_COLOR_CYAN));
}

static void ILI9341_TestElipses(void)
{
	const int16_t w = (int16_t)Test_Width();
	const int16_t h = (int16_t)Test_Height();

	const int16_t rx = w / 4 - 3;
	const int16_t ry = h / 2 - 1;

	/* -- Elipses: contorno a la izquierda, relleno a la derecha (tocan bordes laterales, superior e inferior) -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawEllipse(rx, h / 2, rx, ry, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawFilledEllipse(w - 1 - rx, h / 2, rx, ry, ILI9341_COLOR_CYAN));
}

static void ILI9341_TestArc(void)
{
	const int16_t w = (int16_t)Test_Width();
	const int16_t h = (int16_t)Test_Height();

	const int16_t r1 = ((w < h) ? w : h) / 2 - 2;
	const int16_t r2 = r1 / 2;

	/* -- Arcos: contorno a la izquierda (toca el borde izq.), relleno a la derecha (toca el borde der.) -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawArc(r1, h / 2, r1, r2, 0.0f, 270.0f, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawFilledArc(w - 1 - r1, h / 2, r1, r2, 0.0f, 270.0f, ILI9341_COLOR_GREEN));
}

static void ILI9341_TestColores(void)
{
	static const uint16_t barras[] = {
		ILI9341_COLOR_RED, ILI9341_COLOR_LIME, ILI9341_COLOR_BLUE, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK
	};
	const uint8_t  n  = sizeof(barras) / sizeof(barras[0]);
	const uint16_t w  = Test_Width();
	const uint16_t h  = Test_Height();
	const uint16_t bh = (h - h / 2) / 4;

	/* Color565 debe coincidir con la macro RGB565 */
	if (ILI9341_Color565(255, 128, 64) != RGB565(255, 128, 64))
	{
		Error_Handler();
	}

	/* -- Mitad superior: 5 barras rojo/verde/azul/blanco/negro. Si rojo y azul salen
	 *    intercambiados, el panel está en orden BGR -- */
	for (uint8_t i = 0; i < n; i++)
	{
		const uint16_t x0 = i * w / n;
		const uint16_t x1 = (i + 1) * w / n - 1;
		ILI_CHECK(ILI9341_DrawFilledRectangle(x0, 0, x1, h / 2 - 1, barras[i]));
	}

	/* -- Mitad inferior: degradados de rojo, verde, azul y gris con Color565 -- */
	for (uint16_t x = 0; x < w; x++)
	{
		const uint8_t v = (uint8_t)((uint32_t)x * 255U / (w - 1U));
		ILI_CHECK(ILI9341_DrawFastVLine(x, h / 2, bh, ILI9341_Color565(v, 0, 0)));
		ILI_CHECK(ILI9341_DrawFastVLine(x, h / 2 + bh, bh, ILI9341_Color565(0, v, 0)));
		ILI_CHECK(ILI9341_DrawFastVLine(x, h / 2 + 2 * bh, bh, ILI9341_Color565(0, 0, v)));
		ILI_CHECK(ILI9341_DrawFastVLine(x, h / 2 + 3 * bh, bh, ILI9341_Color565(v, v, v)));
	}
}

static void ILI9341_TestRotacion(void)
{
	static const ILI9341_Orientation_t orientaciones[] = {
		ILI9341_Orientation_Portrait_1,
		ILI9341_Orientation_Landscape_1,
		ILI9341_Orientation_Portrait_2,
		ILI9341_Orientation_Landscape_2,
	};
	static const char* const nombres[] = { "Portrait 1", "Landscape 1", "Portrait 2", "Landscape 2" };

	/* -- Rotación: marco pegado al borde y nombre arriba a la izquierda en cada orientación -- */
	for (uint8_t i = 0; i < 4; i++)
	{
		ILI_CHECK(Test_Rotate(orientaciones[i]));
		ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_NAVY));
		ILI_CHECK(ILI9341_DrawRectangle(0, 0, Test_Width() - 1, Test_Height() - 1, ILI9341_COLOR_WHITE));
		ILI_CHECK(ILI9341_Puts(4, 4, (char*)nombres[i], &LCD_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_NAVY));
		ILI_CHECK(ILI9341_Printf(4, 26, &LCD_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_NAVY,
								 "%ux%u", (unsigned)Test_Width(), (unsigned)Test_Height()));
		HAL_Delay(1500);
	}

	/* Restaura la orientación de trabajo para el resto de los tests */
	ILI_CHECK(Test_Rotate(TEST_ORIENTATION));
}

static void ILI9341_TestErrores(void)
{
	/* -- Punteros NULL y región invertida (x1 < x0) -- */
	ILI_EXPECT(ILI9341_Puts(0, 0, NULL, &LCD_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_Puts(0, 0, "X", NULL, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_Printf(0, 0, &LCD_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK, NULL), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_Printf(0, 0, NULL, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK, "X"), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_PutsAligned(10, 5, 0, ILI9341_ALIGN_LEFT, "X", &LCD_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK), ILI9341_INVALID_PARAM);

	/* -- Init con parámetros inválidos: se rechaza antes de tocar el estado del driver -- */
	ILI_EXPECT(ILI9341_Init(NULL, ILI_CS_GPIO_Port, ILI_CS_Pin, ILI_RES_GPIO_Port, ILI_RES_Pin,
							ILI_DC_GPIO_Port, ILI_DC_Pin), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_Init(&hspi2, NULL, ILI_CS_Pin, ILI_RES_GPIO_Port, ILI_RES_Pin,
							ILI_DC_GPIO_Port, ILI_DC_Pin), ILI9341_INVALID_PARAM);

	/* -- Touch: sin configurar no hay estado, y el XPT2046 exige un puerto CS -- */
	if (ILI9341_TP_GetState() != NULL)
	{
		Error_Handler();
	}
	ILI_EXPECT(ILI9341_TP_ConfigXPT2046(NULL, 0, NULL, 0), ILI9341_INVALID_PARAM);

	/* Nada de lo anterior debe haber roto el driver: sigue dibujando */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_DARKCYAN));
	ILI_CHECK(ILI9341_Puts(4, 4, "Errores OK", &LCD_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_DARKCYAN));
}

static void ILI9341_TestDeInit(void)
{
	/* -- DeInit marca el driver como no inicializado (no apaga el panel, queda la última imagen):
	 *    las llamadas posteriores deben devolver NOT_INITIALIZED sin tocar el bus -- */
	ILI_CHECK(ILI9341_DeInit());
	ILI_EXPECT(ILI9341_DeInit(), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Fill(ILI9341_COLOR_RED), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_DrawPixel(0, 0, ILI9341_COLOR_RED), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Rotate(TEST_ORIENTATION), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Puts(0, 0, "X", &LCD_Font_7x10, ILI9341_COLOR_RED, ILI9341_COLOR_RED), ILI9341_NOT_INITIALIZED);
	if (ILI9341_TP_GetState() != NULL)
	{
		Error_Handler();
	}
	HAL_Delay(1500);

	/* -- Re-Init: el SPI debe volver a 2 Mbit/s (el Init anterior lo subió a /2) y el
	 *    driver debe poder arrancar de nuevo. Init reinicia la geometría a vertical -- */
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	ILI_CHECK(ILI9341_Init(&hspi2,
						   ILI_CS_GPIO_Port, ILI_CS_Pin,
						   ILI_RES_GPIO_Port, ILI_RES_Pin,
						   ILI_DC_GPIO_Port, ILI_DC_Pin));
	Test_Orientation = ILI9341_Orientation_Portrait_1;
	ILI_CHECK(Test_Rotate(TEST_ORIENTATION));
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_Puts(4, 4, "Re-Init OK", &LCD_Font_11x18, ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK));
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
