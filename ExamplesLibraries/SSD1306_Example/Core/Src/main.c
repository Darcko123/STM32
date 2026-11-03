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
#include "SSD1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SSD_CHECK(x)  do { if ((x) != SSD1306_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define SSD_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
SSD1306_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void SSD1306_TestSuite(void);
static void SSD1306_TestFonts(void);
static void SSD1306_TestPutc(void);
static void SSD1306_TestPixeles(void);
static void SSD1306_TestLineas(void);
static void SSD1306_TestRectangulos(void);
static void SSD1306_TestTriangulos(void);
static void SSD1306_TestCirculos(void);
static void SSD1306_TestElipses(void);
static void SSD1306_TestArc(void);
static void SSD1306_TestBitmap(void);
static void SSD1306_TestInversion(void);
static void SSD1306_TestScroll(void);
static void SSD1306_TestEncendido(void);
static void SSD1306_TestErrores(void);
static void SSD1306_TestPreInit(void);
static void SSD1306_TestReInit(void);
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
  /* USER CODE BEGIN 2 */
  SSD1306_TestPreInit();

  status = SSD1306_Init(&hi2c1, 128, 64);
  if (status != SSD1306_OK)
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
	  SSD1306_TestSuite();
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

static void SSD1306_TestSuite(void)
{
	SSD1306_TestFonts();
	HAL_Delay(2000);

	SSD1306_TestPutc();
	HAL_Delay(2000);

	SSD1306_TestPixeles();
	HAL_Delay(2000);

	SSD1306_TestLineas();
	HAL_Delay(2000);

	SSD1306_TestRectangulos();
	HAL_Delay(2000);

	SSD1306_TestTriangulos();
	HAL_Delay(2000);

	SSD1306_TestCirculos();
	HAL_Delay(2000);

	SSD1306_TestElipses();
	HAL_Delay(2000);

	SSD1306_TestArc();
	HAL_Delay(2000);

	SSD1306_TestBitmap();
	HAL_Delay(2000);

	SSD1306_TestInversion();
	HAL_Delay(2000);

	SSD1306_TestScroll();
	HAL_Delay(2000);

	SSD1306_TestEncendido();
	HAL_Delay(2000);

	SSD1306_TestErrores();
	HAL_Delay(2000);

	SSD1306_TestReInit();
	HAL_Delay(2000);
}

static void SSD1306_TestFonts(void)
{
	/* -- Texto con las tres fuentes (16x26 cabe 7 caracteres en 128 px) -- */
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_GotoXY(0, 0));
	SSD_CHECK(SSD1306_Puts("SSD1306", &Font_16x26, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_GotoXY(0, 28));
	SSD_CHECK(SSD1306_Puts("Font 11x18", &Font_11x18, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_GotoXY(0, 50));
	SSD_CHECK(SSD1306_Puts("Font 7x10", &Font_7x10, SSD1306_COLOR_WHITE));

	/* Texto negro sobre fondo blanco: Putc pinta la celda completa con el color contrario */
	SSD_CHECK(SSD1306_DrawFilledRectangle(80, 48, 47, 15, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_GotoXY(84, 50));
	SSD_CHECK(SSD1306_Puts("Inv", &Font_7x10, SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestPutc(void)
{
	const uint16_t w    = SSD1306_GetWidth();
	FontDef_t*     f    = &Font_7x10;
	const uint16_t cols = w / f->FontWidth;
	uint16_t       i    = 0;

	/* -- Juego de caracteres completo (' ' .. '~') con Putc, saltando de línea a mano:
	 *    Putc no hace salto automático, avanza CurrentX y nada más -- */
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	for (char c = ' '; c <= '~'; c++, i++)
	{
		if ((i % cols) == 0U)
		{
			SSD_CHECK(SSD1306_GotoXY(0, (i / cols) * f->FontHeight));
		}
		SSD_CHECK(SSD1306_Putc(c, f, SSD1306_COLOR_WHITE));
	}

	/* -- Errores: fuente NULL, cadena NULL y celda que se sale 1 px por la derecha -- */
	SSD_EXPECT(SSD1306_Putc('X', NULL, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);
	SSD_EXPECT(SSD1306_Puts(NULL, f, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);
	SSD_CHECK(SSD1306_GotoXY(w - f->FontWidth + 1, 0));
	SSD_EXPECT(SSD1306_Putc('X', f, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestPixeles(void)
{
	const uint16_t w = SSD1306_GetWidth();
	const uint16_t h = SSD1306_GetHeight();

	/* -- Rejilla de puntos cada 4 px en la mitad izquierda -- */
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	for (uint16_t y = 0; y < h; y += 4)
	{
		for (uint16_t x = 0; x < w / 2; x += 4)
		{
			SSD_CHECK(SSD1306_DrawPixel(x, y, SSD1306_COLOR_WHITE));
		}
	}

	/* -- Un píxel en cada esquina: deben verse los cuatro, pegados al borde -- */
	SSD_CHECK(SSD1306_DrawPixel(0, 0, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawPixel(w - 1, 0, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawPixel(0, h - 1, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawPixel(w - 1, h - 1, SSD1306_COLOR_WHITE));

	/* -- Fuera de pantalla: a diferencia de las primitivas, DrawPixel no recorta y lo rechaza -- */
	SSD_EXPECT(SSD1306_DrawPixel(w, 0, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);
	SSD_EXPECT(SSD1306_DrawPixel(0, h, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestLineas(void)
{
	const uint16_t w = SSD1306_GetWidth();
	const uint16_t h = SSD1306_GetHeight();

	/* -- Lineas: marco, diagonales y ejes centrales -- */
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_DrawRectangle(0, 0, w - 1, h - 1, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawLine(0, 0, w - 1, h - 1, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawLine(0, h - 1, w - 1, 0, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawFastHLine(0, h / 2, w, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawFastVLine(w / 2, 0, h, SSD1306_COLOR_WHITE));

	/* -- Recorte: línea que sobresale 20 px por cada lado y línea con ancho negativo (sin error) -- */
	SSD_CHECK(SSD1306_DrawFastHLine(-20, h / 4, w + 40, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawFastHLine(w - 1, 3 * h / 4, -(int16_t)w, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestRectangulos(void)
{
	const uint16_t w = SSD1306_GetWidth();
	const uint16_t h = SSD1306_GetHeight();

	/* -- Rectangulos: contorno a la izquierda, relleno a la derecha (tocan los bordes).
	 *    Ambos dibujan w + 1 x h + 1 píxeles, por eso se pasa el tamaño - 1 -- */
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_DrawRectangle(0, 0, w / 2 - 5, h - 1, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawFilledRectangle(w / 2 + 4, 0, w / 2 - 5, h - 1, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestTriangulos(void)
{
	const uint16_t w = SSD1306_GetWidth();
	const uint16_t h = SSD1306_GetHeight();

	/* -- Triangulos: contorno a la izquierda, relleno a la derecha (vértices en los bordes) -- */
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_DrawTriangle(w / 4, 0, 0, h - 1, w / 2 - 5, h - 1, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawFilledTriangle(w / 2 + 4, 0, w - 1, 0, 3 * w / 4, h - 1, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestCirculos(void)
{
	const uint16_t w = SSD1306_GetWidth();
	const uint16_t h = SSD1306_GetHeight();
	/* DrawCircle no recorta: el radio lo limita el lado que primero toque el borde */
	const int16_t  r = ((w / 4 < h / 2) ? w / 4 : h / 2) - 1;

	/* -- Circulos: contorno a la izquierda (toca el borde izq.), relleno a la derecha (toca el borde der.) -- */
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_DrawCircle(r, h / 2, r, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawFilledCircle(w - 1 - r, h / 2, r, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestElipses(void)
{
	const uint16_t w = SSD1306_GetWidth();
	const uint16_t h = SSD1306_GetHeight();

	const int16_t  rx = w / 4 - 3;
	const int16_t  ry = h / 2 - 1;

	/* -- Elipses: contorno a la izquierda, relleno a la derecha (tocan bordes laterales, superior e inferior) -- */
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_DrawEllipse(rx, h / 2, rx, ry, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawFilledEllipse(w - 1 - rx, h / 2, rx, ry, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestArc(void)
{
	const uint16_t w = SSD1306_GetWidth();
	const uint16_t h = SSD1306_GetHeight();

	const int16_t  r1 = ((w < h) ? w : h) / 2 - 2;
	const int16_t  r2 = r1 / 2;

	/* -- Arcos: contorno a la izquierda (toca el borde izq.), relleno a la derecha (toca el borde der.) -- */
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_DrawArc(r1, h / 2, r1, r2, 0.0f, 270.0f, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawFilledArc(w - 1 - r1, h / 2, r1, r2, 0.0f, 270.0f, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestBitmap(void)
{
	const uint16_t w = SSD1306_GetWidth();
	const uint16_t h = SSD1306_GetHeight();

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
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_DrawBitmap(0, 0, bmpF, bw, bh, SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_DrawBitmap(w - bw, 0, bmpF, bw, bh, SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_DrawBitmap(0, h - bh, bmpF, bw, bh, SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_DrawBitmap(w - bw, h - bh, bmpF, bw, bh, SSD1306_COLOR_BLACK));

	/* -- Color: blanco sobre un recuadro negro en el centro (los bits en 0 no pintan) -- */
	SSD_CHECK(SSD1306_DrawFilledRectangle(w / 2 - bw, h / 2 - bh, 2 * bw, 2 * bh, SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_DrawBitmap(w / 2 - bw / 2, h / 2 - bh / 2, bmpF, bw, bh, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestInversion(void)
{
	const uint16_t w = SSD1306_GetWidth();
	const uint16_t h = SSD1306_GetHeight();

	/* -- Mitad blanca / mitad negra con texto en cada una -- */
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_DrawFilledRectangle(0, 0, w / 2 - 1, h - 1, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_GotoXY(4, 4));
	SSD_CHECK(SSD1306_Puts("Normal", &Font_7x10, SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_GotoXY(w / 2 + 4, 4));
	SSD_CHECK(SSD1306_Puts("Normal", &Font_7x10, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_UpdateScreen());
	HAL_Delay(1500);

	/* -- InvertDisplay: lo invierte el controlador, el buffer no cambia -- */
	SSD_CHECK(SSD1306_InvertDisplay(1));
	HAL_Delay(1500);
	SSD_CHECK(SSD1306_InvertDisplay(0));
	HAL_Delay(1500);

	/* -- ToggleInvert: invierte el buffer, hace falta UpdateScreen. Se llama dos veces
	 *    para dejar el estado normal: mientras está activo, DrawPixel también invierte el color -- */
	SSD_CHECK(SSD1306_ToggleInvert());
	SSD_CHECK(SSD1306_UpdateScreen());
	HAL_Delay(1500);
	SSD_CHECK(SSD1306_ToggleInvert());
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestScroll(void)
{
	/* -- Scroll por hardware: 4 franjas de texto, una por cada par de páginas (16 px) -- */
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_GotoXY(0, 3));
	SSD_CHECK(SSD1306_Puts("Pagina 0-1", &Font_7x10, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_GotoXY(0, 19));
	SSD_CHECK(SSD1306_Puts("Pagina 2-3", &Font_7x10, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_GotoXY(0, 35));
	SSD_CHECK(SSD1306_Puts("Pagina 4-5", &Font_7x10, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_GotoXY(0, 51));
	SSD_CHECK(SSD1306_Puts("Pagina 6-7", &Font_7x10, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_UpdateScreen());
	HAL_Delay(1000);

	/* Solo las páginas 0-1 se desplazan, el resto debe quedarse quieto */
	SSD_CHECK(SSD1306_ScrollRight(0, 1));
	HAL_Delay(3000);
	SSD_CHECK(SSD1306_Stopscroll());

	/* Al detener el scroll la GRAM queda desplazada: se reenvía el buffer en cada cambio */
	SSD_CHECK(SSD1306_UpdateScreen());
	SSD_CHECK(SSD1306_ScrollLeft(0, 7));
	HAL_Delay(3000);
	SSD_CHECK(SSD1306_Stopscroll());

	SSD_CHECK(SSD1306_UpdateScreen());
	SSD_CHECK(SSD1306_Scrolldiagright(0, 7));
	HAL_Delay(3000);
	SSD_CHECK(SSD1306_Stopscroll());

	SSD_CHECK(SSD1306_UpdateScreen());
	SSD_CHECK(SSD1306_Scrolldiagleft(0, 7));
	HAL_Delay(3000);
	SSD_CHECK(SSD1306_Stopscroll());
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestEncendido(void)
{
	/* -- OFF/ON: la pantalla se apaga 1.5 s y al encender vuelve la misma imagen
	 *    (la GRAM se conserva apagada, no hace falta redibujar) -- */
	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_GotoXY(4, 4));
	SSD_CHECK(SSD1306_Puts("Display OFF/ON", &Font_7x10, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_UpdateScreen());
	HAL_Delay(1000);

	SSD_CHECK(SSD1306_OFF());
	HAL_Delay(1500);
	SSD_CHECK(SSD1306_ON());
}

static void SSD1306_TestErrores(void)
{
	const uint16_t w = SSD1306_GetWidth();
	const uint16_t h = SSD1306_GetHeight();
	static const uint8_t dummy = 0xFF;

	/* -- Coordenadas de origen fuera de pantalla -- */
	SSD_EXPECT(SSD1306_GotoXY(w, 0), SSD1306_INVALID_PARAM);
	SSD_EXPECT(SSD1306_GotoXY(0, h), SSD1306_INVALID_PARAM);
	SSD_EXPECT(SSD1306_DrawRectangle(w, 0, 4, 4, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);
	SSD_EXPECT(SSD1306_DrawFilledRectangle(0, h, 4, 4, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);

	/* -- Radios negativos y bitmap NULL -- */
	SSD_EXPECT(SSD1306_DrawEllipse(w / 2, h / 2, -1, 4, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);
	SSD_EXPECT(SSD1306_DrawFilledEllipse(w / 2, h / 2, 4, -1, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);
	SSD_EXPECT(SSD1306_DrawArc(w / 2, h / 2, -1, 4, 0.0f, 90.0f, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);
	SSD_EXPECT(SSD1306_DrawFilledArc(w / 2, h / 2, 4, -1, 0.0f, 90.0f, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);
	SSD_EXPECT(SSD1306_DrawBitmap(0, 0, NULL, 8, 1, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);

	/* -- DrawBitmap no recorta: un bitmap que se sale 1 px por la derecha se rechaza -- */
	SSD_EXPECT(SSD1306_DrawBitmap(w - 7, 0, &dummy, 8, 1, SSD1306_COLOR_WHITE), SSD1306_INVALID_PARAM);

	/* -- Init con parámetros inválidos: se rechaza antes de tocar el estado del driver -- */
	SSD_EXPECT(SSD1306_Init(NULL, 128, 64), SSD1306_INVALID_PARAM);
	SSD_EXPECT(SSD1306_Init(&hi2c1, 0, 64), SSD1306_INVALID_PARAM);
	SSD_EXPECT(SSD1306_Init(&hi2c1, SSD1306_MAX_WIDTH + 1, 64), SSD1306_INVALID_PARAM);
	SSD_EXPECT(SSD1306_Init(&hi2c1, 128, SSD1306_MAX_HEIGHT + 1), SSD1306_INVALID_PARAM);

	SSD_CHECK(SSD1306_Fill(SSD1306_COLOR_BLACK));
	SSD_CHECK(SSD1306_GotoXY(4, 4));
	SSD_CHECK(SSD1306_Puts("Errores OK", &Font_11x18, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_UpdateScreen());
}

static void SSD1306_TestPreInit(void)
{
	/* -- Antes de Init: todo debe devolver NOT_INITIALIZED sin tocar el bus -- */
	if (SSD1306_GetWidth() != 0U || SSD1306_GetHeight() != 0U)
	{
		Error_Handler();
	}
	SSD_EXPECT(SSD1306_Fill(SSD1306_COLOR_WHITE), SSD1306_NOT_INITIALIZED);
	SSD_EXPECT(SSD1306_DrawPixel(0, 0, SSD1306_COLOR_WHITE), SSD1306_NOT_INITIALIZED);
	SSD_EXPECT(SSD1306_Putc('X', &Font_7x10, SSD1306_COLOR_WHITE), SSD1306_NOT_INITIALIZED);
	SSD_EXPECT(SSD1306_UpdateScreen(), SSD1306_NOT_INITIALIZED);
	SSD_EXPECT(SSD1306_ON(), SSD1306_NOT_INITIALIZED);
	SSD_EXPECT(SSD1306_ScrollRight(0, 7), SSD1306_NOT_INITIALIZED);
}

static void SSD1306_TestReInit(void)
{
	/* -- Re-Init: sobre un driver ya inicializado debe reconfigurar el panel y limpiarlo -- */
	SSD_CHECK(SSD1306_Init(&hi2c1, 128, 64));
	HAL_Delay(1000);
	SSD_CHECK(SSD1306_GotoXY(4, 4));
	SSD_CHECK(SSD1306_Puts("Re-Init OK", &Font_11x18, SSD1306_COLOR_WHITE));
	SSD_CHECK(SSD1306_UpdateScreen());
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
