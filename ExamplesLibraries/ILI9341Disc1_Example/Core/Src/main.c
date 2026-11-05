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
#include "ILI9341_Disc1.h"
#include "ErisRudeus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Parámetros de la batería de pruebas -- */
#define TEST_TOUCH_MS   10000U   /**< Ventana interactiva de ILI9341_TestTouch()                    */
#define TEST_ICON_W        48U   /**< Ancho del icono de prueba para DisplayImage() y BlitImage()   */
#define TEST_ICON_H        48U   /**< Alto del icono de prueba                                      */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ILI_CHECK(x)  do { if ((x) != ILI9341_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define ILI_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi5_tx;

SDRAM_HandleTypeDef hsdram2;

/* USER CODE BEGIN PV */
ILI9341_Status_t status;

/* Icono de prueba (degradado RGB565) para DisplayImage() y BlitImage(); lo rellena ILI9341_TestMakeIcon() */
static uint16_t Test_Icon[TEST_ICON_W * TEST_ICON_H];

/* Bitmaps 1bpp (MSB primero, (w + 7) / 8 bytes por fila) */

/* Carita 16x16: ancho múltiplo de 8 -> 2 bytes por fila */
static const uint8_t bmpSmiley[16U * 2U] = {
	0x07, 0xE0,  0x18, 0x18,  0x20, 0x04,  0x40, 0x02,
	0x4C, 0x32,  0x8C, 0x31,  0x80, 0x01,  0x80, 0x01,
	0x80, 0x01,  0x90, 0x09,  0x88, 0x11,  0x47, 0xE2,
	0x40, 0x02,  0x20, 0x04,  0x18, 0x18,  0x07, 0xE0,
};

/* "F" de 12x12: asimétrica para detectar espejos/rotaciones. Ancho no múltiplo
 * de 8 -> 2 bytes por fila; los 4 bits de relleno (0x0F) van en 1 a propósito:
 * si aparecen columnas extra a la derecha de la F, el relleno no se ignora. */
static const uint8_t bmpF[12U * 2U] = {
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI5_Init(void);
/* USER CODE BEGIN PFP */
static void ILI9341_TestSuite(void);
static void ILI9341_TestMakeIcon(void);
static void ILI9341_TestFonts(void);
static void ILI9341_TestPutc(void);
static void ILI9341_TestTexto(void);
static void ILI9341_TestPixeles(void);
static void ILI9341_TestLineas(void);
static void ILI9341_TestRectangulos(void);
static void ILI9341_TestTriangulos(void);
static void ILI9341_TestCirculos(void);
static void ILI9341_TestElipses(void);
static void ILI9341_TestArc(void);
static void ILI9341_TestBitmap(void);
static void ILI9341_TestRotacion(void);
static void ILI9341_TestImagen(void);
static void ILI9341_TestImageBuffer(void);
static void ILI9341_TestBitmapImageBuffer(void);
static void ILI9341_TestTouch(void);
static void ILI9341_TestErrores(void);
static void ILI9341_TestPreInit(void);
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
  MX_DMA_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_TestPreInit();

  /* SPI5 + I2C3 + SDRAM + DMA2D: el SPI debe llegar lento (2.8 Mbit/s), Init lo sube solo */
  status = ILI9341_Init(&hspi5, &hi2c3, &hsdram2, &hdma2d);
  if (status != ILI9341_OK)
  {
	  Error_Handler();
  }
  else
  {
	  HAL_GPIO_WritePin(LED_Verde_GPIO_Port, LED_Verde_Pin, GPIO_PIN_SET);
  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM2 memory initialization sequence
  */
  hsdram2.Instance = FMC_SDRAM_DEVICE;
  /* hsdram2.Init */
  hsdram2.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram2.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram2.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram2.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram2.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram2.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram2.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram2.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram2.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram2.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram2, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ILI_CS_GPIO_Port, ILI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ILI_RESET_Pin|ILI_DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED_Verde_Pin|LED_Rojo_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ILI_CS_Pin */
  GPIO_InitStruct.Pin = ILI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ILI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ILI_RESET_Pin ILI_DC_Pin */
  GPIO_InitStruct.Pin = ILI_RESET_Pin|ILI_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Verde_Pin LED_Rojo_Pin */
  GPIO_InitStruct.Pin = LED_Verde_Pin|LED_Rojo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void ILI9341_TestSuite(void)
{
	ILI9341_TestFonts();
	HAL_Delay(2000);

	ILI9341_TestPutc();
	HAL_Delay(2000);

	ILI9341_TestTexto();
	HAL_Delay(2000);

	ILI9341_TestPixeles();
	HAL_Delay(2000);

	ILI9341_TestLineas();
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

	ILI9341_TestBitmap();
	HAL_Delay(2000);

	ILI9341_TestRotacion();
	HAL_Delay(2000);

	ILI9341_TestImagen();
	HAL_Delay(2000);

	ILI9341_TestImageBuffer();
	HAL_Delay(2000);

	ILI9341_TestBitmapImageBuffer();
	HAL_Delay(2000);

	ILI9341_TestTouch();
	HAL_Delay(2000);

	ILI9341_TestErrores();
	HAL_Delay(2000);

	ILI9341_TestDeInit();
	HAL_Delay(2000);
}

static void ILI9341_TestMakeIcon(void)
{
	/* Degradado RGB565 de 48x48: R crece con x, G con y, B en diagonal */
	for (uint16_t y = 0; y < TEST_ICON_H; y++)
	{
		for (uint16_t x = 0; x < TEST_ICON_W; x++)
		{
			Test_Icon[(y * TEST_ICON_W) + x] =
					ILI9341_Color565((uint8_t)(x * 5U), (uint8_t)(y * 5U), (uint8_t)(((x + y) * 2U) & 0xFFU));
		}
	}
}

static void ILI9341_TestFonts(void)
{
	const uint16_t bg = ILI9341_COLOR_BLACK;
	uint16_t tw, th;

	/* -- Texto con las tres fuentes y colores distintos (16x26 cabe 15 caracteres en 240 px) -- */
	ILI_CHECK(ILI9341_Fill(bg));
	ILI_CHECK(ILI9341_Puts(0, 0, "ILI9341 TEST", &LCD_Font_16x26, ILI9341_COLOR_WHITE, bg));
	ILI_CHECK(ILI9341_Puts(0, 34, "Font 11x18", &LCD_Font_11x18, ILI9341_COLOR_YELLOW, bg));
	ILI_CHECK(ILI9341_Puts(0, 60, "Font 7x10", &LCD_Font_7x10, ILI9341_COLOR_CYAN, bg));

	/* -- '\n' baja una línea (FontHeight + 1) y vuelve a la columna inicial -- */
	ILI_CHECK(ILI9341_Puts(0, 76, "ABCDEFGHIJKLMNOPQRSTUVWXYZ\nabcdefghijklmnopqrstuvwxyz\n0123456789 !?#$%&()*+-=",
						   &LCD_Font_7x10, ILI9341_COLOR_WHITE, bg));

	/* -- Sin '\n': el texto salta de línea solo al llegar al borde derecho (caben 21 caracteres de 11 px) -- */
	ILI_CHECK(ILI9341_Puts(0, 120, "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ", &LCD_Font_11x18, ILI9341_COLOR_GREEN, bg));

	/* Esquina inferior derecha: verifica GetStringSize y los bordes */
	ILI9341_GetStringSize("Fin", &LCD_Font_11x18, &tw, &th);
	ILI_CHECK(ILI9341_Puts(ILI9341_WIDTH - tw, ILI9341_HEIGHT - th, "Fin",
						   &LCD_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_GREEN));
}

static void ILI9341_TestPutc(void)
{
	const uint16_t bg = ILI9341_COLOR_BLACK;
	const uint16_t w  = ILI9341_WIDTH;
	const uint16_t h  = ILI9341_HEIGHT;
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
	ILI_CHECK(ILI9341_Putc(0, 100, '\n', f, ILI9341_COLOR_RED, ILI9341_COLOR_RED));
	ILI_CHECK(ILI9341_Putc(0, 100, (char)127, f, ILI9341_COLOR_RED, ILI9341_COLOR_RED));

	/* -- Errores: fuente NULL -- */
	ILI_EXPECT(ILI9341_Putc(0, 0, 'X', NULL, ILI9341_COLOR_RED, bg), ILI9341_INVALID_PARAM);
}

static void ILI9341_TestTexto(void)
{
	const uint16_t w  = ILI9341_WIDTH;
	const uint16_t bg = ILI9341_COLOR_WHITE;
	LCD_FontDef_t* f  = &LCD_Font_11x18;
	uint16_t tw, th;

	ILI_CHECK(ILI9341_Fill(bg));

	/* -- Printf con argumentos variádicos (sin %f: el proyecto no enlaza printf de float) -- */
	ILI_CHECK(ILI9341_Printf(4, 4, f, ILI9341_COLOR_BLACK, bg, "%s = %d (0x%04X)", "id", -42, 0xBEEFU));

	/* -- GetStringSize: el recuadro rojo debe rodear la cadena sin dejar holgura -- */
	ILI9341_GetStringSize("Hola ILI9341", &LCD_Font_16x26, &tw, &th);
	ILI_CHECK(ILI9341_Puts(4, 40, "Hola ILI9341", &LCD_Font_16x26, ILI9341_COLOR_BLUE, bg));
	ILI_CHECK(ILI9341_DrawRectangle(3, 39, 4 + tw, 40 + th, ILI9341_COLOR_RED));
	ILI_CHECK(ILI9341_Printf(4, 76, f, ILI9341_COLOR_DARKRED, bg, "size = %u x %u px", tw, th));

	/* -- PutsAligned: las tres alineaciones dentro de la región [4, w - 5] (recuadro gris) -- */
	ILI_CHECK(ILI9341_DrawRectangle(3, 104, w - 4, 182, ILI9341_COLOR_GRAY));
	ILI_CHECK(ILI9341_PutsAligned(4, w - 5, 108, ILI9341_ALIGN_LEFT,   "LEFT",   f, ILI9341_COLOR_BLACK, bg));
	ILI_CHECK(ILI9341_PutsAligned(4, w - 5, 132, ILI9341_ALIGN_CENTER, "CENTER", f, ILI9341_COLOR_BLACK, bg));
	ILI_CHECK(ILI9341_PutsAligned(4, w - 5, 156, ILI9341_ALIGN_RIGHT,  "RIGHT",  f, ILI9341_COLOR_BLACK, bg));

	/* -- PrintfAligned: formato + alineación -- */
	ILI_CHECK(ILI9341_PrintfAligned(4, w - 5, 200, ILI9341_ALIGN_CENTER, f, ILI9341_COLOR_DARKGREEN, bg,
									"paso %u/%u", 1U, 3U));
	ILI_CHECK(ILI9341_PrintfAligned(4, w - 5, 226, ILI9341_ALIGN_RIGHT, f, ILI9341_COLOR_DARKGREEN, bg,
									"listo %u%%", 100U));

	/* -- Errores: región invertida (x1 < x0), cadena NULL y formato NULL -- */
	ILI_EXPECT(ILI9341_PutsAligned(100, 10, 270, ILI9341_ALIGN_LEFT, "X", f, ILI9341_COLOR_BLACK, bg), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_PrintfAligned(100, 10, 270, ILI9341_ALIGN_LEFT, f, ILI9341_COLOR_BLACK, bg, "X"), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_PutsAligned(4, w - 5, 270, ILI9341_ALIGN_LEFT, NULL, f, ILI9341_COLOR_BLACK, bg), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_Printf(4, 270, f, ILI9341_COLOR_BLACK, bg, NULL), ILI9341_INVALID_PARAM);
}

static void ILI9341_TestPixeles(void)
{
	const uint16_t w = ILI9341_WIDTH;
	const uint16_t h = ILI9341_HEIGHT;

	/* -- Rejilla de puntos cada 8 px con un degradado generado con Color565 (R crece con x, G con y) -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	for (uint16_t y = 0; y < h; y += 8)
	{
		for (uint16_t x = 0; x < w; x += 8)
		{
			ILI_CHECK(ILI9341_DrawPixel(x, y, ILI9341_Color565((uint8_t)x, (uint8_t)y, 128U)));
		}
	}

	/* -- Un píxel blanco en cada esquina: deben verse los cuatro, pegados al borde -- */
	ILI_CHECK(ILI9341_DrawPixel(0, 0, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawPixel(w - 1, 0, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawPixel(0, h - 1, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawPixel(w - 1, h - 1, ILI9341_COLOR_WHITE));
}

static void ILI9341_TestLineas(void)
{
	const uint16_t w = ILI9341_WIDTH;
	const uint16_t h = ILI9341_HEIGHT;

	/* -- Lineas: marco, diagonales y ejes centrales -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawRectangle(0, 0, w - 1, h - 1, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawLine(0, 0, w - 1, h - 1, ILI9341_COLOR_RED));
	ILI_CHECK(ILI9341_DrawLine(0, h - 1, w - 1, 0, ILI9341_COLOR_GREEN));
	ILI_CHECK(ILI9341_DrawFastHLine(0, h / 2, w, ILI9341_COLOR_BLUE));
	ILI_CHECK(ILI9341_DrawFastVLine(w / 2, 0, h, ILI9341_COLOR_YELLOW));

	/* -- Líneas con grosor: horizontal, vertical y diagonal -- */
	ILI_CHECK(ILI9341_DrawThickLine(20, 40, w - 21, 40, 8, ILI9341_COLOR_ORANGE));
	ILI_CHECK(ILI9341_DrawThickLine(30, 70, 30, h - 70, 8, ILI9341_COLOR_PINK));
	ILI_CHECK(ILI9341_DrawThickLine(50, h - 40, w - 30, h / 2 + 40, 12, ILI9341_COLOR_GOLD));

	/* -- Recorte: línea que sobresale 20 px por cada lado y línea con ancho negativo (sin error) -- */
	ILI_CHECK(ILI9341_DrawFastHLine(-20, h / 4, w + 40, ILI9341_COLOR_MAGENTA));
	ILI_CHECK(ILI9341_DrawFastHLine(w - 1, 3 * h / 4, -(int16_t)w, ILI9341_COLOR_CYAN));
	ILI_CHECK(ILI9341_DrawFastVLine(w / 4, -20, h + 40, ILI9341_COLOR_LIME));
	ILI_CHECK(ILI9341_DrawFastVLine(3 * w / 4, h - 1, -(int16_t)h, ILI9341_COLOR_VIOLET));
}

static void ILI9341_TestRectangulos(void)
{
	const uint16_t w  = ILI9341_WIDTH;
	const uint16_t h  = ILI9341_HEIGHT;
	const uint16_t cx = w / 2;
	const uint16_t cy = h / 2;

	/* -- Rectangulos en cuatro cuadrantes que tocan los bordes (esquinas inclusivas x0, y0, x1, y1):
	 *    contorno arriba izq., relleno arriba der., redondeado abajo izq. y redondeado relleno abajo der. -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawRectangle(0, 0, cx - 5, cy - 5, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawFilledRectangle(cx + 4, 0, w - 1, cy - 5, ILI9341_COLOR_RED));
	ILI_CHECK(ILI9341_DrawRoundRect(0, cy + 4, cx - 5, h - 1, 20, ILI9341_COLOR_GREEN));
	ILI_CHECK(ILI9341_DrawFilledRoundRect(cx + 4, cy + 4, w - 1, h - 1, 20, ILI9341_COLOR_BLUE));

	/* -- Radios especiales: r = 0 equivale a DrawRectangle (marco blanco dentro del rojo) y un radio mayor
	 *    que la mitad del lado menor se recorta solo (queda una píldora dentro del contorno blanco) -- */
	ILI_CHECK(ILI9341_DrawRoundRect(cx + 14, 10, w - 11, cy - 15, 0, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawFilledRoundRect(10, 50, cx - 15, 100, 100, ILI9341_COLOR_TEAL));
}

static void ILI9341_TestTriangulos(void)
{
	const uint16_t w  = ILI9341_WIDTH;
	const uint16_t h  = ILI9341_HEIGHT;
	const uint16_t cy = h / 2;

	/* -- Triangulos: contorno arriba, relleno abajo (vértices en los bordes) -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawTriangle(w / 2, 0, 0, cy - 10, w - 1, cy - 10, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawFilledTriangle(w / 2, cy + 10, 0, h - 1, w - 1, h - 1, ILI9341_COLOR_MAGENTA));

	/* -- Triángulo degenerado (vértices alineados): se dibuja como una línea y no debe fallar -- */
	ILI_CHECK(ILI9341_DrawFilledTriangle(0, cy, w / 2, cy, w - 1, cy, ILI9341_COLOR_CYAN));
}

static void ILI9341_TestCirculos(void)
{
	const uint16_t w = ILI9341_WIDTH;
	const uint16_t h = ILI9341_HEIGHT;
	const int16_t  r = w / 4 - 3;

	/* -- Circulos: contorno a la izquierda (toca el borde izq.), relleno a la derecha (toca el borde der.) -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawCircle(r, h / 4, r, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawFilledCircle(w - 1 - r, h / 4, r, ILI9341_COLOR_ORANGE));

	/* -- Anillos concéntricos y radio 0 (no debe colgarse ni fallar) -- */
	for (int16_t rr = 10; rr <= 50; rr += 10)
	{
		ILI_CHECK(ILI9341_DrawCircle(w / 2, 3 * h / 4, rr, ILI9341_Color565((uint8_t)(rr * 5), (uint8_t)(255 - rr * 5), 128U)));
	}
	ILI_CHECK(ILI9341_DrawFilledCircle(w / 2, 3 * h / 4, 0, ILI9341_COLOR_RED));
}

static void ILI9341_TestElipses(void)
{
	const uint16_t w = ILI9341_WIDTH;
	const uint16_t h = ILI9341_HEIGHT;

	const int16_t  rx = w / 4 - 3;
	const int16_t  ry = h / 2 - 1;

	/* -- Elipses: contorno a la izquierda, relleno a la derecha (tocan bordes laterales, superior e inferior) -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawEllipse(rx, h / 2, rx, ry, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawFilledEllipse(w - 1 - rx, h / 2, rx, ry, ILI9341_COLOR_CYAN));
}

static void ILI9341_TestArc(void)
{
	const uint16_t w = ILI9341_WIDTH;
	const uint16_t h = ILI9341_HEIGHT;

	const int16_t  r1 = w / 4 - 3;
	const int16_t  r2 = r1 / 2;

	/* -- Arcos: contorno a la izquierda (toca el borde izq.), relleno a la derecha (toca el borde der.) -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawArc(r1, h / 2, r1, r2, 0.0f, 270.0f, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawFilledArc(w - 1 - r1, h / 2, r1, r2, 0.0f, 270.0f, ILI9341_COLOR_GREEN));
}

static void ILI9341_TestBitmap(void)
{
	static const uint16_t colores[] = {
		ILI9341_COLOR_YELLOW, ILI9341_COLOR_RED, ILI9341_COLOR_LIME,
		ILI9341_COLOR_CYAN, ILI9341_COLOR_MAGENTA, ILI9341_COLOR_ORANGE
	};
	const uint16_t w  = ILI9341_WIDTH;
	const uint16_t h  = ILI9341_HEIGHT;
	const uint16_t bw = 12;
	const uint16_t bh = 12;

	/* -- Bitmaps "F": negro sobre blanco en las 4 esquinas (tocan los bordes). A diferencia de otras
	 *    librerías, el fondo es opaco: los bits en 0 se pintan con el color de fondo -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawBitmap1bpp(0, 0, bw, bh, bmpF, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawBitmap1bpp(w - bw, 0, bw, bh, bmpF, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawBitmap1bpp(0, h - bh, bw, bh, bmpF, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawBitmap1bpp(w - bw, h - bh, bw, bh, bmpF, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE));

	/* -- Recorte: a medio salir por la derecha y por abajo (sin error) -- */
	ILI_CHECK(ILI9341_DrawBitmap1bpp(w - bw / 2, h / 2, bw, bh, bmpF, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE));
	ILI_CHECK(ILI9341_DrawBitmap1bpp(w / 2, h - bh / 2, bw, bh, bmpF, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE));

	/* -- Ancho múltiplo de 8: fila de caritas 16x16 en distintos colores sobre fondo azul marino -- */
	for (uint8_t i = 0; i < (uint8_t)(sizeof(colores) / sizeof(colores[0])); i++)
	{
		ILI_CHECK(ILI9341_DrawBitmap1bpp(20 + i * 34, 60, 16, 16, bmpSmiley, colores[i], ILI9341_COLOR_NAVY));
	}

	/* -- Color: rojo sobre un recuadro negro en el centro -- */
	ILI_CHECK(ILI9341_DrawFilledRectangle(w / 2 - bw, h / 2 - bh, w / 2 + bw - 1, h / 2 + bh - 1, ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawBitmap1bpp(w / 2 - bw / 2, h / 2 - bh / 2, bw, bh, bmpF, ILI9341_COLOR_RED, ILI9341_COLOR_BLACK));

	/* -- Totalmente fuera de pantalla y w = 0: no dibujan nada y devuelven OK -- */
	ILI_CHECK(ILI9341_DrawBitmap1bpp(w, 0, bw, bh, bmpF, ILI9341_COLOR_RED, ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawBitmap1bpp(0, h, bw, bh, bmpF, ILI9341_COLOR_RED, ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_DrawBitmap1bpp(0, 0, 0, bh, bmpF, ILI9341_COLOR_RED, ILI9341_COLOR_BLACK));

	/* -- Errores: bitmap NULL -- */
	ILI_EXPECT(ILI9341_DrawBitmap1bpp(0, 0, bw, bh, NULL, ILI9341_COLOR_RED, ILI9341_COLOR_BLACK), ILI9341_INVALID_PARAM);
}

static void ILI9341_TestRotacion(void)
{
	static const ILI9341_Orientation_t orientaciones[] = {
		ILI9341_Orientation_Portrait_1,
		ILI9341_Orientation_Landscape_1,
		ILI9341_Orientation_Portrait_2,
		ILI9341_Orientation_Landscape_2,
	};
	static char* const nombres[] = { "Portrait 1", "Landscape 1", "Portrait 2", "Landscape 2" };

	/* -- Rotación: marco pegado al borde, diagonal y nombre arriba a la izquierda en cada orientación.
	 *    La librería no expone el tamaño activo: las orientaciones impares (apaisadas) intercambian ancho y alto -- */
	for (uint8_t i = 0; i < 4; i++)
	{
		const uint16_t w = (i & 1U) ? ILI9341_HEIGHT : ILI9341_WIDTH;
		const uint16_t h = (i & 1U) ? ILI9341_WIDTH : ILI9341_HEIGHT;

		ILI_CHECK(ILI9341_Rotate(orientaciones[i]));
		ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_NAVY));
		ILI_CHECK(ILI9341_DrawRectangle(0, 0, w - 1, h - 1, ILI9341_COLOR_WHITE));
		ILI_CHECK(ILI9341_DrawLine(0, 0, w - 1, h - 1, ILI9341_COLOR_YELLOW));
		ILI_CHECK(ILI9341_Puts(4, 4, nombres[i], &LCD_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_NAVY));
		HAL_Delay(1500);
	}

	/* Restaura la orientación vertical para el resto de los tests */
	ILI_CHECK(ILI9341_Rotate(ILI9341_Orientation_Portrait_1));
}

static void ILI9341_TestImagen(void)
{
	const uint16_t w = ILI9341_WIDTH;
	const uint16_t h = ILI9341_HEIGHT;

	ILI9341_TestMakeIcon();

	/* -- Imagen a pantalla completa desde flash (ErisRudeus.h, 240x320) enviada por DMA -- */
	ILI_CHECK(ILI9341_DisplayImage(0, 0, IMAGE_DATA_WIDTH, IMAGE_DATA_HEIGHT, image_data));
	HAL_Delay(2000);

	/* -- Icono de 48x48 en las 4 esquinas (tocan los bordes) -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_DARKSLATEGRAY));
	ILI_CHECK(ILI9341_DisplayImage(0, 0, TEST_ICON_W, TEST_ICON_H, Test_Icon));
	ILI_CHECK(ILI9341_DisplayImage(w - TEST_ICON_W, 0, TEST_ICON_W, TEST_ICON_H, Test_Icon));
	ILI_CHECK(ILI9341_DisplayImage(0, h - TEST_ICON_H, TEST_ICON_W, TEST_ICON_H, Test_Icon));
	ILI_CHECK(ILI9341_DisplayImage(w - TEST_ICON_W, h - TEST_ICON_H, TEST_ICON_W, TEST_ICON_H, Test_Icon));

	/* -- Recorte: a medio salir por la derecha y por abajo (sin error) -- */
	ILI_CHECK(ILI9341_DisplayImage(w - TEST_ICON_W / 2, h / 2, TEST_ICON_W, TEST_ICON_H, Test_Icon));
	ILI_CHECK(ILI9341_DisplayImage(w / 2, h - TEST_ICON_H / 2, TEST_ICON_W, TEST_ICON_H, Test_Icon));

	/* -- Totalmente fuera de pantalla y w = 0: no dibujan nada y devuelven OK -- */
	ILI_CHECK(ILI9341_DisplayImage(w, 0, TEST_ICON_W, TEST_ICON_H, Test_Icon));
	ILI_CHECK(ILI9341_DisplayImage(0, h, TEST_ICON_W, TEST_ICON_H, Test_Icon));
	ILI_CHECK(ILI9341_DisplayImage(0, 0, 0, TEST_ICON_H, Test_Icon));

	/* -- Tras DisplayImage() el SPI vuelve a 8 bits: el resto del driver funciona (cuadro amarillo en el centro) -- */
	ILI_CHECK(ILI9341_DrawFilledRectangle(w / 2 - 20, h / 2 - 20, w / 2 + 19, h / 2 + 19, ILI9341_COLOR_YELLOW));

	/* -- Errores: imagen NULL -- */
	ILI_EXPECT(ILI9341_DisplayImage(0, 0, TEST_ICON_W, TEST_ICON_H, NULL), ILI9341_INVALID_PARAM);
}

static void ILI9341_TestImageBuffer(void)
{
	const uint16_t bg = ILI9341_COLOR_DARKSLATEGRAY;
	/* Rejilla de 4 x 4 celdas de 60 px que empieza en y = 80 */
	const uint16_t c0 = 0, c1 = 60, c2 = 120, c3 = 180;
	const uint16_t r0 = 80, r1 = 140, r2 = 200, r3 = 260;
	uint32_t* fb = ILI9341_GetFrameBuffer();

	ILI9341_TestMakeIcon();

	/* -- Todo se dibuja fuera de pantalla, en el back buffer de la SDRAM, y se presenta con Flush() -- */
	if (fb == NULL)
	{
		Error_Handler();
	}
	ILI_CHECK(ILI9341_Fill_ImageBuffer(bg, fb));

	/* -- Texto en el frame buffer: Putc, Puts, Printf, PutsAligned y PrintfAligned -- */
	ILI_CHECK(ILI9341_Putc_ImageBuffer(0, 0, '#', &LCD_Font_16x26, ILI9341_COLOR_RED, fb));
	ILI_CHECK(ILI9341_Puts_ImageBuffer(16, 0, "ImageBuffer", &LCD_Font_16x26, ILI9341_COLOR_YELLOW, fb));
	ILI_CHECK(ILI9341_Printf_ImageBuffer(4, 30, &LCD_Font_7x10, ILI9341_COLOR_WHITE, fb,
										 "Printf %ux%u px", ILI9341_WIDTH, ILI9341_HEIGHT));
	ILI_CHECK(ILI9341_PutsAligned_ImageBuffer(2, ILI9341_WIDTH - 3, 44, ILI9341_ALIGN_RIGHT, "PutsAligned",
											  &LCD_Font_7x10, ILI9341_COLOR_CYAN, fb));
	ILI_CHECK(ILI9341_PrintfAligned_ImageBuffer(2, ILI9341_WIDTH - 3, 58, ILI9341_ALIGN_CENTER,
												&LCD_Font_7x10, ILI9341_COLOR_GREEN, fb, "PrintfAligned %u", 2U));

	/* -- Una primitiva por celda de la rejilla -- */
	for (uint16_t i = 0; i < 9; i++)
	{
		ILI_CHECK(ILI9341_DrawPixel_ImageBuffer(c0 + 4 + i * 6, r0 + 4 + i * 6, ILI9341_COLOR_YELLOW, fb));
	}
	ILI_CHECK(ILI9341_DrawLine_ImageBuffer(c1 + 4, r0 + 4, c1 + 55, r0 + 55, ILI9341_COLOR_CYAN, fb));
	ILI_CHECK(ILI9341_DrawThickLine_ImageBuffer(c2 + 4, r0 + 55, c2 + 55, r0 + 4, 6, ILI9341_COLOR_ORANGE, fb));
	ILI_CHECK(ILI9341_DrawRectangle_ImageBuffer(c3 + 4, r0 + 4, c3 + 55, r0 + 55, ILI9341_COLOR_WHITE, fb));

	ILI_CHECK(ILI9341_DrawFilledRectangle_ImageBuffer(c0 + 4, r1 + 4, c0 + 55, r1 + 55, ILI9341_COLOR_AQUAMARINE, fb));
	ILI_CHECK(ILI9341_DrawRoundRect_ImageBuffer(c1 + 4, r1 + 4, c1 + 55, r1 + 55, 14, ILI9341_COLOR_MAGENTA, fb));
	ILI_CHECK(ILI9341_DrawFilledRoundRect_ImageBuffer(c2 + 4, r1 + 4, c2 + 55, r1 + 55, 14, ILI9341_COLOR_GOLD, fb));
	ILI_CHECK(ILI9341_DrawCircle_ImageBuffer(c3 + 30, r1 + 30, 26, ILI9341_COLOR_LIME, fb));

	ILI_CHECK(ILI9341_DrawFilledCircle_ImageBuffer(c0 + 30, r2 + 30, 26, ILI9341_COLOR_DODGERBLUE, fb));
	ILI_CHECK(ILI9341_DrawTriangle_ImageBuffer(c1 + 30, r2 + 4, c1 + 4, r2 + 55, c1 + 55, r2 + 55, ILI9341_COLOR_PINK, fb));
	ILI_CHECK(ILI9341_DrawFilledTriangle_ImageBuffer(c2 + 30, r2 + 4, c2 + 4, r2 + 55, c2 + 55, r2 + 55, ILI9341_COLOR_VIOLET, fb));
	ILI_CHECK(ILI9341_DrawEllipse_ImageBuffer(c3 + 30, r2 + 30, 26, 16, ILI9341_COLOR_TURQUOISE, fb));

	ILI_CHECK(ILI9341_DrawFilledEllipse_ImageBuffer(c0 + 30, r3 + 30, 16, 26, ILI9341_COLOR_CORAL, fb));
	ILI_CHECK(ILI9341_DrawArc_ImageBuffer(c1 + 30, r3 + 30, 26, 14, 0.0f, 270.0f, ILI9341_COLOR_CHARTREUSE, fb));
	ILI_CHECK(ILI9341_DrawFilledArc_ImageBuffer(c2 + 30, r3 + 30, 26, 14, 0.0f, 270.0f, ILI9341_COLOR_ORANGERED, fb));

	/* -- BlitImage: copia el icono de 48x48 desde la RAM con DMA2D (celda 4,4) -- */
	ILI_CHECK(ILI9341_BlitImage(Test_Icon, c3 + 6, r3 + 6, TEST_ICON_W, TEST_ICON_H, fb));

	/* -- Píxel fuera de límites: se descarta en silencio y devuelve OK -- */
	ILI_CHECK(ILI9341_DrawPixel_ImageBuffer(1000, 1000, ILI9341_COLOR_RED, fb));

	/* -- Flush: intercambia los buffers y envía el frame por DMA SPI -- */
	ILI_CHECK(ILI9341_Flush());
	HAL_Delay(2000);

	/* -- Segundo frame: tras Flush el back buffer es el otro, hay que pedirlo de nuevo -- */
	fb = ILI9341_GetFrameBuffer();
	if (fb == NULL)
	{
		Error_Handler();
	}
	ILI_CHECK(ILI9341_Fill_ImageBuffer(ILI9341_COLOR_DARKMAGENTA, fb));
	ILI_CHECK(ILI9341_PrintfAligned_ImageBuffer(2, ILI9341_WIDTH - 3, 150, ILI9341_ALIGN_CENTER,
												&LCD_Font_16x26, ILI9341_COLOR_WHITE, fb, "FRAME 2 (SDRAM)"));
	ILI_CHECK(ILI9341_Flush());

	/* -- Sync: espera al DMA antes de volver a usar el SPI de forma directa -- */
	ILI_CHECK(ILI9341_Sync());
}

static void ILI9341_TestBitmapImageBuffer(void)
{
	const uint16_t bg = ILI9341_COLOR_DARKSLATEGRAY;
	uint32_t*       fb = ILI9341_GetFrameBuffer();
	const uint16_t* px;

	if (fb == NULL)
	{
		Error_Handler();
	}
	px = (const uint16_t*)fb;   /* Vista RGB565 del frame para releer píxeles */

	ILI_CHECK(ILI9341_Fill_ImageBuffer(bg, fb));
	ILI_CHECK(ILI9341_PutsAligned_ImageBuffer(2, ILI9341_WIDTH - 3, 6, ILI9341_ALIGN_CENTER, "Bitmap1bpp FB",
											  &LCD_Font_11x18, ILI9341_COLOR_WHITE, fb));

	/* -- Carita 16x16 y relectura: fila 0, col 5 es un bit a 1 (fg) y col 0 un bit a 0 (bg) -- */
	ILI_CHECK(ILI9341_DrawBitmap1bpp_ImageBuffer(20, 60, 16, 16, bmpSmiley, ILI9341_COLOR_YELLOW, ILI9341_COLOR_BLACK, fb));
	if ((px[(60U * ILI9341_WIDTH) + 25U] != ILI9341_COLOR_YELLOW) ||
		(px[(60U * ILI9341_WIDTH) + 20U] != ILI9341_COLOR_BLACK))
	{
		Error_Handler();
	}

	/* -- Rejilla de caritas 6 x 4 (estrés de escrituras en SDRAM) -- */
	for (uint8_t i = 0; i < 24; i++)
	{
		ILI_CHECK(ILI9341_DrawBitmap1bpp_ImageBuffer(20 + (i % 6) * 34, 100 + (i / 6) * 26, 16, 16, bmpSmiley,
													 ILI9341_Color565((uint8_t)(i * 10U), 255U, (uint8_t)(255U - i * 10U)),
													 ILI9341_COLOR_NAVY, fb));
	}

	/* -- Ancho no múltiplo de 8: la "F" de 12 px lleva 4 bits de relleno a 1; el píxel justo a su derecha (col 12) no se toca -- */
	ILI_CHECK(ILI9341_DrawBitmap1bpp_ImageBuffer(20, 220, 12, 12, bmpF, ILI9341_COLOR_WHITE, ILI9341_COLOR_DARKRED, fb));
	if (px[(220U * ILI9341_WIDTH) + 32U] != bg)
	{
		Error_Handler();
	}

	/* -- Recorte en la esquina inferior derecha, fuera de pantalla y w = 0: no fallan -- */
	ILI_CHECK(ILI9341_DrawBitmap1bpp_ImageBuffer(ILI9341_WIDTH - 8, ILI9341_HEIGHT - 8, 16, 16, bmpSmiley,
												 ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK, fb));
	ILI_CHECK(ILI9341_DrawBitmap1bpp_ImageBuffer(0, ILI9341_HEIGHT, 16, 16, bmpSmiley,
												 ILI9341_COLOR_RED, ILI9341_COLOR_BLACK, fb));
	ILI_CHECK(ILI9341_DrawBitmap1bpp_ImageBuffer(0, 0, 0, 16, bmpSmiley, ILI9341_COLOR_RED, ILI9341_COLOR_BLACK, fb));

	/* -- Errores: bitmap NULL y buffer NULL -- */
	ILI_EXPECT(ILI9341_DrawBitmap1bpp_ImageBuffer(0, 0, 16, 16, NULL, ILI9341_COLOR_RED, ILI9341_COLOR_BLACK, fb), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_DrawBitmap1bpp_ImageBuffer(0, 0, 16, 16, bmpSmiley, ILI9341_COLOR_RED, ILI9341_COLOR_BLACK, NULL), ILI9341_INVALID_PARAM);

	/* -- Presenta el frame y espera al DMA antes de volver al SPI directo -- */
	ILI_CHECK(ILI9341_Flush());
	HAL_Delay(2000);
	ILI_CHECK(ILI9341_Sync());
}

static void ILI9341_TestTouch(void)
{
	TP_STATE* tp;
	uint32_t  t0;

	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_PrintfAligned(2, ILI9341_WIDTH - 3, 60, ILI9341_ALIGN_CENTER, &LCD_Font_11x18,
									ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK, "TOUCH"));
	ILI_CHECK(ILI9341_PrintfAligned(2, ILI9341_WIDTH - 3, 90, ILI9341_ALIGN_CENTER, &LCD_Font_11x18,
									ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK, "toque la pantalla..."));

	/* -- TP_Config detecta y configura el STMPE811; TP_GetState devuelve el estado interno -- */
	ILI_CHECK(ILI9341_TP_Config());
	tp = ILI9341_TP_GetState();
	if (tp == NULL)
	{
		Error_Handler();
	}

	/* -- Ventana interactiva: cada toque dibuja un círculo rojo y muestra X, Y y Z (presión) -- */
	t0 = HAL_GetTick();
	while ((HAL_GetTick() - t0) < TEST_TOUCH_MS)
	{
		tp = ILI9341_TP_GetState();
		if (tp->TouchDetected)
		{
			ILI_CHECK(ILI9341_DrawFilledCircle((int16_t)tp->X, (int16_t)tp->Y, 12, ILI9341_COLOR_RED));
			ILI_CHECK(ILI9341_Printf(2, ILI9341_HEIGHT - 20, &LCD_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK,
									 "X=%3u Y=%3u Z=%3u", tp->X, tp->Y, tp->Z));
		}
		HAL_Delay(10);
	}
}

static void ILI9341_TestErrores(void)
{
	uint32_t* fb = ILI9341_GetFrameBuffer();
	const uint16_t w = ILI9341_WIDTH;
	const uint16_t h = ILI9341_HEIGHT;

	/* -- Cadena y formato NULL -- */
	ILI_EXPECT(ILI9341_Puts(0, 0, NULL, &LCD_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_Puts(0, 0, "X", NULL, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK), ILI9341_INVALID_PARAM);

	/* -- Radios negativos -- */
	ILI_EXPECT(ILI9341_DrawEllipse(w / 2, h / 2, -1, 4, ILI9341_COLOR_WHITE), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_DrawFilledEllipse(w / 2, h / 2, 4, -1, ILI9341_COLOR_WHITE), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_DrawArc(w / 2, h / 2, -1, 4, 0.0f, 90.0f, ILI9341_COLOR_WHITE), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_DrawFilledArc(w / 2, h / 2, 4, -1, 0.0f, 90.0f, ILI9341_COLOR_WHITE), ILI9341_INVALID_PARAM);

	/* -- Buffer NULL en las variantes de frame buffer, y puntero NULL en BlitImage -- */
	if (fb == NULL)
	{
		Error_Handler();
	}
	ILI_EXPECT(ILI9341_Fill_ImageBuffer(ILI9341_COLOR_RED, NULL), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_DrawPixel_ImageBuffer(0, 0, ILI9341_COLOR_RED, NULL), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_DrawArc_ImageBuffer(w / 2, h / 2, -1, 4, 0.0f, 90.0f, ILI9341_COLOR_RED, fb), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_BlitImage(NULL, 0, 0, TEST_ICON_W, TEST_ICON_H, fb), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_BlitImage(Test_Icon, 0, 0, TEST_ICON_W, TEST_ICON_H, NULL), ILI9341_INVALID_PARAM);

	/* -- Init con handles NULL: se rechaza antes de tocar el estado del driver -- */
	ILI_EXPECT(ILI9341_Init(NULL, &hi2c3, &hsdram2, &hdma2d), ILI9341_INVALID_PARAM);
	ILI_EXPECT(ILI9341_Init(&hspi5, NULL, &hsdram2, &hdma2d), ILI9341_INVALID_PARAM);

	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_DARKCYAN));
	ILI_CHECK(ILI9341_PrintfAligned(2, w - 3, 150, ILI9341_ALIGN_CENTER, &LCD_Font_16x26,
									ILI9341_COLOR_WHITE, ILI9341_COLOR_DARKCYAN, "Errores OK"));
}

static void ILI9341_TestPreInit(void)
{
	/* -- Antes de Init: todo debe devolver NOT_INITIALIZED (o NULL) sin tocar el bus -- */
	ILI_EXPECT(ILI9341_Fill(ILI9341_COLOR_WHITE), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_DrawPixel(0, 0, ILI9341_COLOR_WHITE), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Putc(0, 0, 'X', &LCD_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Puts(0, 0, "X", &LCD_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Rotate(ILI9341_Orientation_Portrait_1), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_DisplayImage(0, 0, IMAGE_DATA_WIDTH, IMAGE_DATA_HEIGHT, image_data), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Flush(), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Sync(), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_TP_Config(), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_DeInit(), ILI9341_NOT_INITIALIZED);
	if ((ILI9341_GetFrameBuffer() != NULL) || (ILI9341_TP_GetState() != NULL))
	{
		Error_Handler();
	}
}

static void ILI9341_TestDeInit(void)
{
	/* -- DeInit libera los handles y apaga el FMC (HAL_SDRAM_DeInit): las llamadas posteriores
	 *    deben devolver NOT_INITIALIZED sin tocar el bus -- */
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_RED));
	ILI_CHECK(ILI9341_PrintfAligned(2, ILI9341_WIDTH - 3, 150, ILI9341_ALIGN_CENTER, &LCD_Font_16x26,
									ILI9341_COLOR_WHITE, ILI9341_COLOR_RED, "DeInit"));
	HAL_Delay(1500);

	ILI_CHECK(ILI9341_DeInit());
	ILI_EXPECT(ILI9341_DeInit(), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Fill(ILI9341_COLOR_BLACK), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_DrawPixel(0, 0, ILI9341_COLOR_BLACK), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Rotate(ILI9341_Orientation_Portrait_1), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_Flush(), ILI9341_NOT_INITIALIZED);
	ILI_EXPECT(ILI9341_TP_Config(), ILI9341_NOT_INITIALIZED);
	if ((ILI9341_GetFrameBuffer() != NULL) || (ILI9341_TP_GetState() != NULL))
	{
		Error_Handler();
	}
	HAL_Delay(1500);

	/* -- Re-Init: DeInit dejó el FMC apagado y el SPI en velocidad alta; se vuelven a configurar con
	 *    sus MX_*_Init() (SPI lento, como exige Init) antes de arrancar el driver otra vez -- */
	MX_FMC_Init();
	MX_SPI5_Init();
	ILI_CHECK(ILI9341_Init(&hspi5, &hi2c3, &hsdram2, &hdma2d));
	ILI_CHECK(ILI9341_Fill(ILI9341_COLOR_BLACK));
	ILI_CHECK(ILI9341_PrintfAligned(2, ILI9341_WIDTH - 3, 150, ILI9341_ALIGN_CENTER, &LCD_Font_16x26,
									ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK, "Re-Init OK"));
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
		HAL_GPIO_TogglePin(LED_Rojo_GPIO_Port, LED_Rojo_Pin);
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
