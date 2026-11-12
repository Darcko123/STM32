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
#include "NV3007.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Parámetros de la batería de pruebas -- */
#define TEST_TITLE_MS         800U   /**< Tiempo en pantalla del cartel de cada test */
#define TEST_FAIL_DWELL_MS    500U   /**< LED encendido cuando una comprobación falla */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NV_CHECK(x)  do { if ((x) != NV3007_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define NV_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

NV3007_Status_t status;

/* Orientación activa del panel, para poder restaurar la que había antes de los
 * tests que cambian la rotación */
static NV3007_Orientation_t Test_Orientation = NV3007_Orientation_Landscape_1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
static void NV3007_TestSuite(void);
static void NV3007_TestFonts(void);
static void NV3007_TestPutc(void);
static void NV3007_TestPixeles(void);
static void NV3007_TestLineas(void);
static void NV3007_TestRectangulos(void);
static void NV3007_TestTriangulos(void);
static void NV3007_TestCirculos(void);
static void NV3007_TestElipses(void);
static void NV3007_TestArc(void);
static void NV3007_TestBitmap(void);
static void NV3007_TestRotacion(void);
static void NV3007_TestOffset(void);
static void NV3007_TestInversion(void);
static void NV3007_TestEncendido(void);
static void NV3007_TestErrores(void);
static void NV3007_TestDeInit(void);
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
  /* La librería no gestiona la retroiluminación: encenderla antes de dibujar */
  HAL_GPIO_WritePin(NV_BL_GPIO_Port, NV_BL_Pin, GPIO_PIN_SET);

  status = NV3007_Init(&hspi2,
		  	  	  	   NV_CS_GPIO_Port, NV_CS_Pin,
					   NV_DC_GPIO_Port, NV_DC_Pin,
					   NV_RESET_GPIO_Port, NV_RESET_Pin);
  if(status != NV3007_OK)
  {
	  Error_Handler();
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }

  NV_CHECK(NV3007_Rotate(Test_Orientation));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  NV3007_TestSuite();
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
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NV_BL_GPIO_Port, NV_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NV_RESET_Pin|NV_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NV_DC_GPIO_Port, NV_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : NV_BL_Pin */
  GPIO_InitStruct.Pin = NV_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NV_BL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NV_RESET_Pin */
  GPIO_InitStruct.Pin = NV_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NV_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NV_DC_Pin NV_CS_Pin */
  GPIO_InitStruct.Pin = NV_DC_Pin|NV_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void NV3007_TestSuite(void)
{
	NV3007_TestFonts();
	HAL_Delay(2000);

	NV3007_TestPutc();
	HAL_Delay(2000);

	NV3007_TestPixeles();
	HAL_Delay(2000);

	NV3007_TestLineas();
	HAL_Delay(2000);

	NV3007_TestRectangulos();
	HAL_Delay(2000);

	NV3007_TestTriangulos();
	HAL_Delay(2000);

	NV3007_TestCirculos();
	HAL_Delay(2000);

	NV3007_TestElipses();
	HAL_Delay(2000);

	NV3007_TestArc();
	HAL_Delay(2000);

	NV3007_TestBitmap();
	HAL_Delay(2000);

	NV3007_TestRotacion();
	HAL_Delay(2000);

	NV3007_TestOffset();
	HAL_Delay(2000);

	NV3007_TestInversion();
	HAL_Delay(2000);

	NV3007_TestEncendido();
	HAL_Delay(2000);

	NV3007_TestErrores();
	HAL_Delay(2000);

	NV3007_TestDeInit();
	HAL_Delay(2000);
}

static void NV3007_TestFonts(void)
{
	const uint16_t bg = NV3007_COLOR_BLACK;
	uint16_t tw, th;

	/* -- Texto con las tres fuentes y colores distintos -- */
	NV_CHECK(NV3007_FillScreen(bg));
	NV_CHECK(NV3007_Puts(0, 0, "NV3007 TEST", &LCD_Font_16x26, NV3007_COLOR_WHITE, bg));
	NV_CHECK(NV3007_Puts(0, 30, "Font 11x18", &LCD_Font_11x18, NV3007_COLOR_YELLOW, bg));
	NV_CHECK(NV3007_Puts(0, 52, "Font 7x10", &LCD_Font_7x10, NV3007_COLOR_CYAN, bg));
	NV_CHECK(NV3007_Puts(0, 66, "ABCDEFGHIJKLMNOPQRSTUVWXYZ\nabcdefghijklmnopqrstuvwxyz\n0123456789 !?#$%&()*+-=",
						 &LCD_Font_7x10, NV3007_COLOR_WHITE, bg));

	/* Esquina inferior derecha: verifica GetStringSize y los bordes */
	NV3007_GetStringSize("Fin", &LCD_Font_11x18, &tw, &th);
	NV_CHECK(NV3007_Puts(NV3007_GetWidth() - tw, NV3007_GetHeight() - th, "Fin",
						 &LCD_Font_11x18, NV3007_COLOR_BLACK, NV3007_COLOR_GREEN));
}

static void NV3007_TestPutc(void)
{
	const uint16_t bg = NV3007_COLOR_BLACK;
	const uint16_t w  = NV3007_GetWidth();
	const uint16_t h  = NV3007_GetHeight();
	LCD_FontDef_t* f  = &LCD_Font_11x18;

	/* -- Caracteres sueltos colocados a mano, avanzando FontWidth columnas -- */
	NV_CHECK(NV3007_FillScreen(bg));
	for (char c = 'A'; c <= 'J'; c++)
	{
		NV_CHECK(NV3007_Putc((uint16_t)(c - 'A') * f->FontWidth, 0, c, f, NV3007_COLOR_WHITE, bg));
	}

	/* -- Salto automático: la celda no cabe a 3 px del borde derecho -> sale en x = 0, y = 30 + FontHeight -- */
	NV_CHECK(NV3007_Putc(w - 3, 30, 'W', f, NV3007_COLOR_YELLOW, bg));

	/* -- Última celda que cabe, pegada a la esquina inferior derecha -- */
	NV_CHECK(NV3007_Putc(w - f->FontWidth, h - f->FontHeight, 'Z', f, NV3007_COLOR_BLACK, NV3007_COLOR_GREEN));

	/* -- No imprimibles: se ignoran y devuelven OK sin pintar (no debe verse ningún bloque rojo) -- */
	NV_CHECK(NV3007_Putc(0, 80, '\n', f, NV3007_COLOR_RED, NV3007_COLOR_RED));
	NV_CHECK(NV3007_Putc(0, 80, (char)127, f, NV3007_COLOR_RED, NV3007_COLOR_RED));

	/* -- Errores: fuente NULL y celda que se sale 1 px por abajo -- */
	NV_EXPECT(NV3007_Putc(0, 0, 'X', NULL, NV3007_COLOR_RED, bg), NV3007_INVALID_PARAM);
	NV_EXPECT(NV3007_Putc(0, h - f->FontHeight + 1, 'X', f, NV3007_COLOR_RED, bg), NV3007_INVALID_PARAM);
}

static void NV3007_TestPixeles(void)
{
	const int16_t w = (int16_t)NV3007_GetWidth();
	const int16_t h = (int16_t)NV3007_GetHeight();

	/* -- Rejilla de puntos cada 4 px: DrawPixel en la mitad izquierda (blanco), WritePixel en la derecha (amarillo) -- */
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_BLACK));
	for (int16_t y = 0; y < h; y += 4)
	{
		for (int16_t x = 0; x < w / 2; x += 4)
		{
			NV_CHECK(NV3007_DrawPixel(x, y, NV3007_COLOR_WHITE));
			NV_CHECK(NV3007_WritePixel(w / 2 + x, y, NV3007_COLOR_YELLOW));
		}
	}

	/* -- Un píxel rojo en cada esquina: deben verse los cuatro, pegados al borde -- */
	NV_CHECK(NV3007_WritePixel(0, 0, NV3007_COLOR_RED));
	NV_CHECK(NV3007_WritePixel(w - 1, 0, NV3007_COLOR_RED));
	NV_CHECK(NV3007_WritePixel(0, h - 1, NV3007_COLOR_RED));
	NV_CHECK(NV3007_WritePixel(w - 1, h - 1, NV3007_COLOR_RED));

	/* -- Fuera de pantalla: se descartan en silencio y devuelven OK -- */
	NV_CHECK(NV3007_DrawPixel(-1, 0, NV3007_COLOR_RED));
	NV_CHECK(NV3007_DrawPixel(0, -1, NV3007_COLOR_RED));
	NV_CHECK(NV3007_WritePixel(w, 0, NV3007_COLOR_RED));
	NV_CHECK(NV3007_WritePixel(0, h, NV3007_COLOR_RED));
}

static void NV3007_TestLineas(void)
{
	const uint16_t w = NV3007_GetWidth();
	const uint16_t h = NV3007_GetHeight();

	/* -- Lineas: marco, diagonales y ejes centrales -- */
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawRectangle(0, 0, w, h, NV3007_COLOR_WHITE));
	NV_CHECK(NV3007_DrawLine(0, 0, w - 1, h - 1, NV3007_COLOR_RED));
	NV_CHECK(NV3007_DrawLine(0, h - 1, w - 1, 0, NV3007_COLOR_GREEN));
	NV_CHECK(NV3007_DrawFastHLine(0, h / 2, w, NV3007_COLOR_BLUE));
	NV_CHECK(NV3007_DrawFastVLine(w / 2, 0, h, NV3007_COLOR_YELLOW));
}

static void NV3007_TestRectangulos(void)
{
	const uint16_t w = NV3007_GetWidth();
	const uint16_t h = NV3007_GetHeight();
	const int16_t  q = w / 4;

	/* -- Rectangulos: contorno, relleno, redondeado y redondeado relleno (de izq. a der.).
	 *    El relleno va en dos mitades: DrawFilledRectangle arriba (rojo) y WriteFilledRectangle
	 *    abajo (naranja), anclado en la esquina inferior derecha con w/h negativos. Las dos
	 *    mitades deben quedar alineadas y sin hueco entre ellas -- */
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawRectangle(0, 0, q - 4, h, NV3007_COLOR_WHITE));
	NV_CHECK(NV3007_DrawFilledRectangle(q, 0, q - 4, h / 2, NV3007_COLOR_RED));
	NV_CHECK(NV3007_WriteFilledRectangle(2 * q - 5, h - 1, -(q - 4), -(h - h / 2), NV3007_COLOR_ORANGE));
	NV_CHECK(NV3007_DrawRoundRect(2 * q, 0, q - 4, h, 15, NV3007_COLOR_GREEN));
	NV_CHECK(NV3007_DrawFilledRoundRect(3 * q, 0, w - 3 * q, h, 15, NV3007_COLOR_BLUE));
}

static void NV3007_TestTriangulos(void)
{
	const uint16_t w = NV3007_GetWidth();
	const uint16_t h = NV3007_GetHeight();

	/* -- Triangulos: contorno a la izquierda, relleno a la derecha (vértices en los bordes) -- */
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawTriangle(w / 4, 0, 0, h - 1, w / 2 - 5, h - 1, NV3007_COLOR_WHITE));
	NV_CHECK(NV3007_DrawFilledTriangle(w / 2 + 4, 0, w - 1, 0, 3 * w / 4, h - 1, NV3007_COLOR_MAGENTA));
}

static void NV3007_TestCirculos(void)
{
	const uint16_t w = NV3007_GetWidth();
	const uint16_t h = NV3007_GetHeight();
	/* El radio lo limita el lado corto para que funcione en cualquier orientación */
	const int16_t  r = ((w < h) ? w : h) / 4 - 2;

	/* -- Circulos: contorno a la izquierda (toca el borde izq.), relleno a la derecha (toca el borde der.) -- */
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawCircle(r, h / 2, r, NV3007_COLOR_WHITE));
	NV_CHECK(NV3007_DrawFilledCircle(w - 1 - r, h / 2, r, NV3007_COLOR_ORANGE));
}

static void NV3007_TestElipses(void)
{
	const uint16_t w = NV3007_GetWidth();
	const uint16_t h = NV3007_GetHeight();

	const int16_t  rx = w / 4 - 3;
	const int16_t  ry = h / 2 - 1;

	/* -- Elipses: contorno a la izquierda, relleno a la derecha (tocan bordes laterales, superior e inferior) -- */
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawEllipse(rx, h / 2, rx, ry, NV3007_COLOR_WHITE));
	NV_CHECK(NV3007_DrawFilledEllipse(w - 1 - rx, h / 2, rx, ry, NV3007_COLOR_CYAN));
}

static void NV3007_TestArc(void)
{
	const uint16_t w = NV3007_GetWidth();
	const uint16_t h = NV3007_GetHeight();

	const int16_t  r1 = ((w < h) ? w : h) / 2 - 2;
	const int16_t  r2 = r1 / 2;

	/* -- Arcos: contorno a la izquierda (toca el borde izq.), relleno a la derecha (toca el borde der.) -- */
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawArc(r1, h / 2, r1, r2, 0.0f, 270.0f, NV3007_COLOR_WHITE));
	NV_CHECK(NV3007_DrawFilledArc(w - 1 - r1, h / 2, r1, r2, 0.0f, 270.0f, NV3007_COLOR_GREEN));
}

static void NV3007_TestBitmap(void)
{
	const uint16_t w = NV3007_GetWidth();
	const uint16_t h = NV3007_GetHeight();

	/* "F" de 12x12: asimétrica para detectar espejos/rotaciones. Ancho no múltiplo
	 * de 8 -> 2 bytes por fila; los 4 bits de relleno (0x0F) van en 1 a propósito:
	 * si aparecen columnas extra a la derecha de la F, el relleno no se ignora. */
	static const uint8_t bmpF[] = {
		0xFF, 0xFF,
		0xFF, 0xFF,
		0xC0, 0x0F,
		0xC0, 0x0F,
		0xFF, 0x0F,
		0xFF, 0x0F,
		0xC0, 0x0F,
		0xC0, 0x0F,
		0xC0, 0x0F,
		0xC0, 0x0F,
		0xC0, 0x0F,
		0xC0, 0x0F,
	};
	const int16_t bw = 12;
	const int16_t bh = 12;

	/* -- Bitmaps: negro sobre blanco en las 4 esquinas (tocan los bordes) -- */
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_WHITE));
	NV_CHECK(NV3007_DrawBitmap(0, 0, bmpF, bw, bh, NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawBitmap(w - bw, 0, bmpF, bw, bh, NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawBitmap(0, h - bh, bmpF, bw, bh, NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawBitmap(w - bw, h - bh, bmpF, bw, bh, NV3007_COLOR_BLACK));

	/* -- Recorte: a medio salir por izquierda, arriba, derecha y abajo (sin error) -- */
	NV_CHECK(NV3007_DrawBitmap(-bw / 2, h / 2, bmpF, bw, bh, NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawBitmap(w / 2, -bh / 2, bmpF, bw, bh, NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawBitmap(w - bw / 2, h / 2, bmpF, bw, bh, NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawBitmap(w / 2, h - bh / 2, bmpF, bw, bh, NV3007_COLOR_BLACK));

	/* -- Color: rojo sobre un recuadro negro en el centro (los bits en 0 no pintan) -- */
	NV_CHECK(NV3007_DrawFilledRectangle(w / 2 - bw, h / 2 - bh, 2 * bw, 2 * bh, NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_DrawBitmap(w / 2 - bw / 2, h / 2 - bh / 2, bmpF, bw, bh, NV3007_COLOR_RED));
}

static void NV3007_TestRotacion(void)
{
	static const NV3007_Orientation_t orientaciones[] = {
		NV3007_Orientation_Portrait_1,
		NV3007_Orientation_Landscape_1,
		NV3007_Orientation_Portrait_2,
		NV3007_Orientation_Landscape_2,
	};
	static const char* const nombres[] = { "Portrait 1", "Landscape 1", "Portrait 2", "Landscape 2" };

	/* -- Rotación: marco pegado al borde y nombre arriba a la izquierda en cada orientación.
	 *    Si el marco no se ve completo en alguna, hay que ajustar su NV3007_OFFSET_* -- */
	for (uint8_t i = 0; i < 4; i++)
	{
		NV_CHECK(NV3007_Rotate(orientaciones[i]));
		NV_CHECK(NV3007_FillScreen(NV3007_COLOR_NAVY));
		NV_CHECK(NV3007_DrawRectangle(0, 0, NV3007_GetWidth(), NV3007_GetHeight(), NV3007_COLOR_WHITE));
		NV_CHECK(NV3007_Puts(4, 4, nombres[i], &LCD_Font_11x18, NV3007_COLOR_WHITE, NV3007_COLOR_NAVY));
		HAL_Delay(1500);
	}

	/* Restaura la orientación de trabajo para el resto de los tests */
	NV_CHECK(NV3007_Rotate(Test_Orientation));
}

static void NV3007_TestOffset(void)
{
	/* -- Offset de GRAM: con SetOffset(0, 0) el marco sale corrido 12-14 px en el eje de 142 px
	 *    (un borde se sale del cristal y en el opuesto queda una franja negra sin pintar). Al
	 *    volver a rotar, NV3007_Rotate() recarga el offset de la tabla y el marco encaja -- */
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_SetOffset(0, 0));
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_NAVY));
	NV_CHECK(NV3007_DrawRectangle(0, 0, NV3007_GetWidth(), NV3007_GetHeight(), NV3007_COLOR_WHITE));
	NV_CHECK(NV3007_Puts(16, 16, "Offset 0,0", &LCD_Font_11x18, NV3007_COLOR_WHITE, NV3007_COLOR_NAVY));
	HAL_Delay(1500);

	NV_CHECK(NV3007_Rotate(Test_Orientation));
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_NAVY));
	NV_CHECK(NV3007_DrawRectangle(0, 0, NV3007_GetWidth(), NV3007_GetHeight(), NV3007_COLOR_WHITE));
	NV_CHECK(NV3007_Puts(16, 16, "Offset ok", &LCD_Font_11x18, NV3007_COLOR_WHITE, NV3007_COLOR_NAVY));
}

static void NV3007_TestInversion(void)
{
	static const uint16_t colores[] = {
		NV3007_COLOR_RED, NV3007_COLOR_LIME, NV3007_COLOR_BLUE, NV3007_COLOR_WHITE, NV3007_COLOR_BLACK
	};
	const uint8_t  n = sizeof(colores) / sizeof(colores[0]);
	const uint16_t w = NV3007_GetWidth();
	const uint16_t h = NV3007_GetHeight();

	/* -- Inversión: 5 franjas rojo/verde/azul/blanco/negro. Con InvertDisplay(true) pasan a
	 *    cian/magenta/amarillo/negro/blanco y con false vuelven a su color original -- */
	for (uint8_t i = 0; i < n; i++)
	{
		const int16_t x0 = i * w / n;
		const int16_t x1 = (i + 1) * w / n;
		NV_CHECK(NV3007_DrawFilledRectangle(x0, 0, x1 - x0, h, colores[i]));
	}
	HAL_Delay(1500);

	NV_CHECK(NV3007_InvertDisplay(true));
	HAL_Delay(1500);
	NV_CHECK(NV3007_InvertDisplay(false));
}

static void NV3007_TestEncendido(void)
{
	/* -- DisplayOff/On: la pantalla se apaga 1.5 s y al encender vuelve la misma imagen
	 *    (la GRAM se conserva en Sleep, no hace falta redibujar) -- */
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_DARKGREEN));
	NV_CHECK(NV3007_Puts(4, 4, "Display OFF/ON", &LCD_Font_7x10, NV3007_COLOR_WHITE, NV3007_COLOR_DARKGREEN));
	HAL_Delay(1000);

	NV_CHECK(NV3007_DisplayOff());
	HAL_Delay(1500);
	NV_CHECK(NV3007_DisplayOn());
}

static void NV3007_TestErrores(void)
{
	/* -- WriteAddrWindow: una ventana que se sale de la GRAM se rechaza sin enviar nada -- */
	NV_EXPECT(NV3007_WriteAddrWindow(0, 0, 1000U, 1U), NV3007_INVALID_PARAM);

	/* -- WriteAddrWindow válida: deja RAMWR abierto con CS bajo. FillScreen usa la misma
	 *    ventana (acierto de caché: solo reenvía RAMWR) y cierra la transacción -- */
	NV_CHECK(NV3007_WriteAddrWindow(0, 0, NV3007_GetWidth(), NV3007_GetHeight()));
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_DARKCYAN));

	/* -- Orientación fuera de rango -- */
	NV_EXPECT(NV3007_Rotate((NV3007_Orientation_t)4), NV3007_INVALID_PARAM);

	NV_CHECK(NV3007_Puts(4, 4, "Errores OK", &LCD_Font_11x18, NV3007_COLOR_WHITE, NV3007_COLOR_DARKCYAN));
}

static void NV3007_TestDeInit(void)
{
	/* -- DeInit apaga el panel y libera la configuración: las llamadas posteriores
	 *    deben devolver NOT_INITIALIZED sin tocar el bus -- */
	NV_CHECK(NV3007_DeInit());
	NV_EXPECT(NV3007_DeInit(), NV3007_NOT_INITIALIZED);
	NV_EXPECT(NV3007_FillScreen(NV3007_COLOR_RED), NV3007_NOT_INITIALIZED);
	NV_EXPECT(NV3007_DrawPixel(0, 0, NV3007_COLOR_RED), NV3007_NOT_INITIALIZED);
	NV_EXPECT(NV3007_Rotate(Test_Orientation), NV3007_NOT_INITIALIZED);
	NV_EXPECT(NV3007_DisplayOn(), NV3007_NOT_INITIALIZED);
	NV_EXPECT(NV3007_Init(NULL,
						  NV_CS_GPIO_Port, NV_CS_Pin,
						  NV_DC_GPIO_Port, NV_DC_Pin,
						  NV_RESET_GPIO_Port, NV_RESET_Pin), NV3007_INVALID_PARAM);
	HAL_Delay(1500);

	/* -- Re-Init: el driver debe poder arrancar de nuevo tras un DeInit -- */
	NV_CHECK(NV3007_Init(&hspi2,
						 NV_CS_GPIO_Port, NV_CS_Pin,
						 NV_DC_GPIO_Port, NV_DC_Pin,
						 NV_RESET_GPIO_Port, NV_RESET_Pin));
	NV_CHECK(NV3007_Rotate(Test_Orientation));
	NV_CHECK(NV3007_FillScreen(NV3007_COLOR_BLACK));
	NV_CHECK(NV3007_Puts(4, 4, "Re-Init OK", &LCD_Font_11x18, NV3007_COLOR_GREEN, NV3007_COLOR_BLACK));
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
