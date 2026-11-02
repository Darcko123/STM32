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
#include "SSD1681.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SSD_CHECK(x)  do { if ((x) != SSD1681_OK) { Error_Handler(); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
SSD1681_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void SSD1681_TestSuite(void);
static void SSD1681_TestFonts(void);
static void SSD1681_TestLineas(void);
static void SSD1681_TestRectangulos(void);
static void SSD1681_TestTriangulos(void);
static void SSD1681_TestCirculos(void);
static void SSD1681_TestElipses(void);
static void SSD1681_TestArc(void);
static void SSD1681_TestBitmap(void);
static void SSD1681_TestParcial(void);
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
  /* USER CODE BEGIN 2 */
  status = SSD1681_Init(&hspi1,
		  	  	  	  	SSD_CS_GPIO_Port, SSD_CS_Pin,
						SSD_DC_GPIO_Port, SSD_DC_Pin,
						SSD_RES_GPIO_Port, SSD_RES_Pin,
						SSD_BUSY_GPIO_Port, SSD_BUSY_Pin,
						200, 200
						);
  if(status != SSD1681_OK)
  {
	  Error_Handler();
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }

  if(SSD1681_Clear() != SSD1681_OK)
  {
	  Error_Handler();
  }

  /* Una sola vez: el e-paper no debe refrescarse en bucle */
  SSD1681_TestSuite();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SSD_DC_GPIO_Port, SSD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SSD_CS_Pin|SSD_RES_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SSD_BUSY_Pin */
  GPIO_InitStruct.Pin = SSD_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SSD_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SSD_DC_Pin */
  GPIO_InitStruct.Pin = SSD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SSD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SSD_CS_Pin SSD_RES_Pin */
  GPIO_InitStruct.Pin = SSD_CS_Pin|SSD_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void SSD1681_TestSuite(void)
{
	SSD1681_TestFonts();
	HAL_Delay(2000);

	SSD1681_TestLineas();
	HAL_Delay(2000);

	SSD1681_TestRectangulos();
	HAL_Delay(2000);

	SSD1681_TestTriangulos();
	HAL_Delay(2000);

	SSD1681_TestCirculos();
	HAL_Delay(2000);

	SSD1681_TestElipses();
	HAL_Delay(2000);

	SSD1681_TestArc();
	HAL_Delay(2000);

	SSD1681_TestBitmap();
	HAL_Delay(2000);

	SSD1681_TestParcial();

	/* El panel conserva la imagen sin energía: dormirlo al terminar */
	SSD_CHECK(SSD1681_Sleep());
}

static void SSD1681_TestFonts(void)
{
	/* -- Texto con distintas fuentes (16x26 cabe 12 caracteres en 200 px) -- */
	SSD_CHECK(SSD1681_Fill(SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_GotoXY(0, 0));
	SSD_CHECK(SSD1681_Puts("SSD1681 TEST", &Font_16x26, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_GotoXY(0, 34));
	SSD_CHECK(SSD1681_Puts("Font 11x18", &Font_11x18, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_GotoXY(0, 60));
	SSD_CHECK(SSD1681_Puts("Font 7x10", &Font_7x10, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_GotoXY(0, 74));
	SSD_CHECK(SSD1681_Puts("ABCDEFGHIJKLMNOPQRSTUVWXYZ", &Font_7x10, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_GotoXY(0, 86));
	SSD_CHECK(SSD1681_Puts("abcdefghijklmnopqrstuvwxyz", &Font_7x10, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_GotoXY(0, 98));
	SSD_CHECK(SSD1681_Puts("0123456789 !?#$%&()*+-=", &Font_7x10, SSD1681_COLOR_WHITE));
	/* Última fila de la pantalla: verifica el borde inferior */
	SSD_CHECK(SSD1681_GotoXY(0, SSD1681_GetHeight() - Font_11x18.FontHeight));
	SSD_CHECK(SSD1681_Puts("Fin", &Font_11x18, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_UpdateScreen());
}

static void SSD1681_TestLineas(void)
{
	const uint16_t w = SSD1681_GetWidth();
	const uint16_t h = SSD1681_GetHeight();

	/* -- Lineas: marco, diagonales y ejes centrales -- */
	SSD_CHECK(SSD1681_Fill(SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawRectangle(0, 0, w - 1, h - 1, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_DrawLine(0, 0, w - 1, h - 1, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_DrawLine(0, h - 1, w - 1, 0, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_DrawFastHLine(0, h / 2, w, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_DrawFastVLine(w / 2, 0, h, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_UpdateScreen());
}

static void SSD1681_TestRectangulos(void)
{
	const uint16_t w = SSD1681_GetWidth();
	const uint16_t h = SSD1681_GetHeight();

	/* -- Rectangulos: contorno a la izquierda (esquinas), relleno a la derecha (ancho/alto) -- */
	SSD_CHECK(SSD1681_Fill(SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawRectangle(0, 0, w / 2 - 5, h - 1, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_DrawFilledRectangle(w / 2 + 4, 0, w / 2 - 4, h, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_UpdateScreen());
}

static void SSD1681_TestTriangulos(void)
{
	const uint16_t w = SSD1681_GetWidth();
	const uint16_t h = SSD1681_GetHeight();

	/* -- Triangulos: contorno a la izquierda, relleno a la derecha (vértices en los bordes) -- */
	SSD_CHECK(SSD1681_Fill(SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawTriangle(w / 4, 0, 0, h - 1, w / 2 - 5, h - 1, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_DrawFilledTriangle(w / 2 + 4, 0, w - 1, 0, 3 * w / 4, h - 1, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_UpdateScreen());
}

static void SSD1681_TestCirculos(void)
{
	const uint16_t w = SSD1681_GetWidth();
	const uint16_t h = SSD1681_GetHeight();

	const int16_t  r = w / 4 - 3;

	/* -- Circulos: contorno a la izquierda (toca el borde izq.), relleno a la derecha (toca el borde der.) -- */
	SSD_CHECK(SSD1681_Fill(SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawCircle(r, h / 2, r, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_DrawFilledCircle(w - 1 - r, h / 2, r, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_UpdateScreen());
}

static void SSD1681_TestElipses(void)
{
	const uint16_t w = SSD1681_GetWidth();
	const uint16_t h = SSD1681_GetHeight();

	const int16_t  rx = w / 4 - 3;
	const int16_t  ry = h / 2 - 1;

	/* -- Elipses: contorno a la izquierda, relleno a la derecha (tocan bordes laterales, superior e inferior) -- */
	SSD_CHECK(SSD1681_Fill(SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawEllipse(rx, h / 2, rx, ry, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_DrawFilledEllipse(w - 1 - rx, h / 2, rx, ry, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_UpdateScreen());
}

static void SSD1681_TestArc(void)
{
	const uint16_t w = SSD1681_GetWidth();
	const uint16_t h = SSD1681_GetHeight();

	const int16_t  r1 = w / 4 - 3;
	const int16_t  r2 = r1 / 2;

	/* -- Arcos: contorno a la izquierda (toca el borde izq.), relleno a la derecha (toca el borde der.) -- */
	SSD_CHECK(SSD1681_Fill(SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawArc(r1, h / 2, r1, r2, 0.0f, 270.0f, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_DrawFilledArc(w - 1 - r1, h / 2, r1, r2, 0.0f, 270.0f, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_UpdateScreen());
}

static void SSD1681_TestBitmap(void)
{
	const uint16_t w = SSD1681_GetWidth();
	const uint16_t h = SSD1681_GetHeight();

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

	/* -- Parámetro inválido: NULL debe rechazarse -- */
	if (SSD1681_DrawBitmap(0, 0, NULL, bw, bh, SSD1681_COLOR_BLACK) != SSD1681_INVALID_PARAM)
	{
		Error_Handler();
	}

	/* -- Bitmaps: negro sobre blanco en las 4 esquinas (tocan los bordes) -- */
	SSD_CHECK(SSD1681_Fill(SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_DrawBitmap(0, 0, bmpF, bw, bh, SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawBitmap(w - bw, 0, bmpF, bw, bh, SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawBitmap(0, h - bh, bmpF, bw, bh, SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawBitmap(w - bw, h - bh, bmpF, bw, bh, SSD1681_COLOR_BLACK));

	/* -- Recorte: a medio salir por izquierda, arriba, derecha y abajo (sin error) -- */
	SSD_CHECK(SSD1681_DrawBitmap(-bw / 2, h / 2, bmpF, bw, bh, SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawBitmap(w / 2, -bh / 2, bmpF, bw, bh, SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawBitmap(w - bw / 2, h / 2, bmpF, bw, bh, SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawBitmap(w / 2, h - bh / 2, bmpF, bw, bh, SSD1681_COLOR_BLACK));

	/* -- Color: blanco sobre un recuadro negro en el centro (los bits en 0 no pintan) -- */
	SSD_CHECK(SSD1681_DrawFilledRectangle(w / 2 - bw, h / 2 - bh, 2 * bw, 2 * bh, SSD1681_COLOR_BLACK));
	SSD_CHECK(SSD1681_DrawBitmap(w / 2 - bw / 2, h / 2 - bh / 2, bmpF, bw, bh, SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_UpdateScreen());
}

static void SSD1681_TestParcial(void)
{
	char texto[16];

	/* -- Refresco parcial: contador sin parpadeo (la 1a llamada hace un refresco completo) -- */
	SSD_CHECK(SSD1681_Fill(SSD1681_COLOR_WHITE));
	SSD_CHECK(SSD1681_GotoXY(0, 0));
	SSD_CHECK(SSD1681_Puts("Parcial", &Font_16x26, SSD1681_COLOR_BLACK));

	for (uint8_t i = 0; i <= 9; i++)
	{
		snprintf(texto, sizeof(texto), "Cuenta: %u", i);
		SSD_CHECK(SSD1681_DrawFilledRectangle(0, 40, SSD1681_GetWidth(), Font_11x18.FontHeight, SSD1681_COLOR_WHITE));
		SSD_CHECK(SSD1681_GotoXY(0, 40));
		SSD_CHECK(SSD1681_Puts(texto, &Font_11x18, SSD1681_COLOR_BLACK));
		SSD_CHECK(SSD1681_UpdateScreenPartial());
		HAL_Delay(500);
	}
}
/* USER CODE END 4 */

/**z
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
