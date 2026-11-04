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
#include "MAX7219.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Parámetros de la batería de pruebas -- */
#define TEST_NUM_DEV     4U    /**< Módulos 8x8 en cascada (1 a MAX7219_MAX_DEV); los tests asumen 4 */
#define TEST_SCROLL_MS   60U   /**< Retardo entre pasos del scroll */
#define TEST_BRILLO      1U    /**< Nivel de brillo de trabajo (el mismo con el que arranca MAX7219_Init) */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_CHECK(x)  do { if ((x) != MAX7219_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define MAX_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
MAX7219_Status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
static void MAX7219_TestSuite(void);
static void MAX7219_TestPrint(void);
static void MAX7219_TestFonts(void);
static void MAX7219_TestPutc(void);
static void MAX7219_TestPixeles(void);
static void MAX7219_TestBarrido(void);
static void MAX7219_TestLineas(void);
static void MAX7219_TestRectangulos(void);
static void MAX7219_TestBitmap(void);
static void MAX7219_TestInversion(void);
static void MAX7219_TestBrillo(void);
static void MAX7219_TestScroll(void);
static void MAX7219_TestSleep(void);
static void MAX7219_TestErrores(void);
static void MAX7219_TestPreInit(void);
static void MAX7219_TestDeInit(void);
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
  MAX7219_TestPreInit();

  status = MAX7219_Init(&hspi2, GPIOB, GPIO_PIN_1, TEST_NUM_DEV);
  if (status != MAX7219_OK)
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
	  MAX7219_TestSuite();
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
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void MAX7219_TestSuite(void)
{
	MAX7219_TestPrint();
	HAL_Delay(2000);

	MAX7219_TestFonts();
	HAL_Delay(2000);

	MAX7219_TestPutc();
	HAL_Delay(2000);

	MAX7219_TestPixeles();
	HAL_Delay(2000);

	MAX7219_TestBarrido();
	HAL_Delay(2000);

	MAX7219_TestLineas();
	HAL_Delay(2000);

	MAX7219_TestRectangulos();
	HAL_Delay(2000);

	MAX7219_TestBitmap();
	HAL_Delay(2000);

	MAX7219_TestInversion();
	HAL_Delay(2000);

	MAX7219_TestBrillo();
	HAL_Delay(2000);

	MAX7219_TestScroll();
	HAL_Delay(2000);

	MAX7219_TestSleep();
	HAL_Delay(2000);

	MAX7219_TestErrores();
	HAL_Delay(2000);

	MAX7219_TestDeInit();
	HAL_Delay(2000);
}

static void MAX7219_TestPrint(void)
{
	/* -- PrintString: cada carácter ocupa 7 columnas, con 4 módulos caben 4 caracteres -- */
	MAX_CHECK(MAX7219_PrintString("1234"));
	HAL_Delay(1500);

	/* -- Un solo carácter: queda pegado al borde izquierdo y apaga los restos de "1234" -- */
	MAX_CHECK(MAX7219_PrintString("A"));
	HAL_Delay(1500);

	/* -- Texto más largo que la matriz: se recorta en el borde derecho sin error
	 *    (debe verse "ABCD" y las 4 primeras columnas de la "E") -- */
	MAX_CHECK(MAX7219_PrintString("ABCDEFGH"));
}

static void MAX7219_TestFonts(void)
{
	const uint16_t w = MAX7219_GetWidth();

	/* -- Texto centrado con GetStringWidth: "Hi" ocupa 14 columnas, sobran 9 a cada lado -- */
	MAX_CHECK(MAX7219_Fill(MAX7219_COLOR_OFF));
	MAX_CHECK(MAX7219_GotoXY((w - MAX7219_GetStringWidth("Hi")) / 2, 0));
	MAX_CHECK(MAX7219_Puts("Hi", MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_UpdateScreen());
	HAL_Delay(1500);

	/* -- Texto apagado sobre fondo encendido: Putc es opaco y pinta la celda con el color contrario -- */
	MAX_CHECK(MAX7219_Fill(MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_GotoXY(2, 0));
	MAX_CHECK(MAX7219_Puts("Inv", MAX7219_COLOR_OFF));
	MAX_CHECK(MAX7219_UpdateScreen());
}

static void MAX7219_TestPutc(void)
{
	const uint16_t w = MAX7219_GetWidth();

	/* -- Puts recorta el último carácter que empieza dentro de la pantalla y devuelve OK:
	 *    "12345" -> la "5" empieza en x = 28 y solo se ven sus 4 primeras columnas -- */
	MAX_CHECK(MAX7219_Fill(MAX7219_COLOR_OFF));
	MAX_CHECK(MAX7219_GotoXY(0, 0));
	MAX_CHECK(MAX7219_Puts("12345", MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_UpdateScreen());
	HAL_Delay(1500);

	/* -- Con el cursor ya fuera de la pantalla, el siguiente carácter se rechaza -- */
	MAX_EXPECT(MAX7219_Putc('6', MAX7219_COLOR_ON), MAX7219_INVALID_PARAM);
	MAX_CHECK(MAX7219_GotoXY(0, 0));
	MAX_EXPECT(MAX7219_Puts("123456", MAX7219_COLOR_ON), MAX7219_INVALID_PARAM);

	/* -- Cursor desplazado hacia abajo: la "A" baja 3 filas y pierde su parte inferior -- */
	MAX_CHECK(MAX7219_Fill(MAX7219_COLOR_OFF));
	MAX_CHECK(MAX7219_GotoXY(w / 2, 3));
	MAX_CHECK(MAX7219_Putc('A', MAX7219_COLOR_ON));

	/* -- Errores: cadena NULL -- */
	MAX_EXPECT(MAX7219_Puts(NULL, MAX7219_COLOR_ON), MAX7219_INVALID_PARAM);
	MAX_CHECK(MAX7219_UpdateScreen());
}

static void MAX7219_TestPixeles(void)
{
	const uint16_t w = MAX7219_GetWidth();
	const uint16_t h = MAX7219_GetHeight();

	if (w != TEST_NUM_DEV * 8U || h != 8U)
	{
		Error_Handler();
	}

	/* -- Un píxel en cada esquina: deben verse los cuatro, pegados al borde -- */
	MAX_CHECK(MAX7219_Fill(MAX7219_COLOR_OFF));
	MAX_CHECK(MAX7219_DrawPixel(0, 0, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_DrawPixel(w - 1, 0, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_DrawPixel(0, h - 1, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_DrawPixel(w - 1, h - 1, MAX7219_COLOR_ON));

	/* -- Orden de la cascada: el módulo n (de izquierda a derecha) muestra n + 1 puntos en la fila central -- */
	for (uint16_t dev = 0; dev < TEST_NUM_DEV; dev++)
	{
		for (uint16_t i = 0; i <= dev; i++)
		{
			MAX_CHECK(MAX7219_DrawPixel(dev * 8U + 1U + 2U * i, h / 2, MAX7219_COLOR_ON));
		}
	}

	/* -- Fuera de pantalla: DrawPixel no recorta y lo rechaza -- */
	MAX_EXPECT(MAX7219_DrawPixel(w, 0, MAX7219_COLOR_ON), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_DrawPixel(0, h, MAX7219_COLOR_ON), MAX7219_INVALID_PARAM);
	MAX_CHECK(MAX7219_UpdateScreen());
}

static void MAX7219_TestBarrido(void)
{
	const uint16_t w = MAX7219_GetWidth();
	const uint16_t h = MAX7219_GetHeight();

	/* -- Barrido con SetPixel (actualiza al instante): las columnas avanzan de izquierda a
	 *    derecha sin saltos entre módulos y luego se apagan en el mismo orden -- */
	MAX_CHECK(MAX7219_Clear());
	for (uint8_t pasada = 0; pasada < 2; pasada++)
	{
		const uint8_t state = (pasada == 0) ? 1 : 0;

		for (uint16_t x = 0; x < w; x++)
		{
			for (uint16_t y = 0; y < h; y++)
			{
				MAX_CHECK(MAX7219_SetPixel(x, y, state));
			}
			HAL_Delay(30);
		}
	}

	/* -- Fuera de pantalla -- */
	MAX_EXPECT(MAX7219_SetPixel(w, 0, 1), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_SetPixel(0, h, 1), MAX7219_INVALID_PARAM);
}

static void MAX7219_TestLineas(void)
{
	const int16_t w = (int16_t)MAX7219_GetWidth();
	const int16_t h = (int16_t)MAX7219_GetHeight();

	/* -- Lineas: marco y diagonales de esquina a esquina -- */
	MAX_CHECK(MAX7219_Fill(MAX7219_COLOR_OFF));
	MAX_CHECK(MAX7219_DrawRectangle(0, 0, w - 1, h - 1, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_DrawLine(0, 0, w - 1, h - 1, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_DrawLine(0, h - 1, w - 1, 0, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_UpdateScreen());
	HAL_Delay(1500);

	/* -- Recorte: las tres líneas deben cruzar la matriz completa (sin error ni huecos):
	 *    horizontal que sobresale 20 px por cada lado, horizontal con ancho negativo
	 *    y vertical con alto negativo -- */
	MAX_CHECK(MAX7219_Fill(MAX7219_COLOR_OFF));
	MAX_CHECK(MAX7219_DrawFastHLine(-20, 1, w + 40, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_DrawFastHLine(w - 1, h - 2, -w, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_DrawFastVLine(w / 2, h - 1, -h, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_UpdateScreen());
}

static void MAX7219_TestRectangulos(void)
{
	const uint16_t w = MAX7219_GetWidth();
	const uint16_t h = MAX7219_GetHeight();

	/* -- Rectangulos: contorno a la izquierda (esquinas pasadas en orden inverso) y relleno
	 *    a la derecha con un ancho mayor que el disponible: se recorta en el borde derecho -- */
	MAX_CHECK(MAX7219_Fill(MAX7219_COLOR_OFF));
	MAX_CHECK(MAX7219_DrawRectangle(w / 2 - 2, h - 1, 0, 0, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_DrawFilledRectangle(w / 2 + 1, 0, w, h, MAX7219_COLOR_ON));

	/* -- Hueco: rectángulo apagado dentro del relleno -- */
	MAX_CHECK(MAX7219_DrawFilledRectangle(w / 2 + 4, 2, 4, 4, MAX7219_COLOR_OFF));

	/* -- Tamaño 0: no dibuja nada y devuelve OK -- */
	MAX_CHECK(MAX7219_DrawFilledRectangle(2, 2, 0, 4, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_UpdateScreen());
}

static void MAX7219_TestBitmap(void)
{
	const int16_t w = (int16_t)MAX7219_GetWidth();

	/* "F" de 6x8: asimétrica para detectar espejos. Ancho no múltiplo de 8 -> los
	 * 2 bits de relleno (0x03) van en 1 a propósito: si aparecen columnas extra a la
	 * derecha de la F, el relleno no se ignora. */
	static const uint8_t bmpF[] = {
		0xFF,
		0xFF,
		0xC3,
		0xF3,
		0xF3,
		0xC3,
		0xC3,
		0xC3,
	};
	const int16_t bw = 6;
	const int16_t bh = 8;

	/* -- Bitmaps: F pegada al borde izquierdo y F pegada al borde derecho -- */
	MAX_CHECK(MAX7219_Fill(MAX7219_COLOR_OFF));
	MAX_CHECK(MAX7219_DrawBitmap(0, 0, bmpF, bw, bh, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_DrawBitmap(w - bw, 0, bmpF, bw, bh, MAX7219_COLOR_ON));

	/* -- Recorte: F a medio salir por arriba (solo se ve su mitad inferior, sin error) -- */
	MAX_CHECK(MAX7219_DrawBitmap(w / 2 - 8, -bh / 2, bmpF, bw, bh, MAX7219_COLOR_ON));

	/* -- Color: F apagada sobre un recuadro encendido (los bits en 0 no pintan) -- */
	MAX_CHECK(MAX7219_DrawFilledRectangle(w / 2, 0, bw + 2, bh, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_DrawBitmap(w / 2 + 1, 0, bmpF, bw, bh, MAX7219_COLOR_OFF));

	/* -- Parámetro inválido: NULL debe rechazarse -- */
	MAX_EXPECT(MAX7219_DrawBitmap(0, 0, NULL, bw, bh, MAX7219_COLOR_ON), MAX7219_INVALID_PARAM);
	MAX_CHECK(MAX7219_UpdateScreen());
}

static void MAX7219_TestInversion(void)
{
	const uint16_t w = MAX7219_GetWidth();
	const uint16_t h = MAX7219_GetHeight();

	/* -- Mitad izquierda con texto, mitad derecha encendida -- */
	MAX_CHECK(MAX7219_Fill(MAX7219_COLOR_OFF));
	MAX_CHECK(MAX7219_GotoXY(0, 0));
	MAX_CHECK(MAX7219_Puts("ab", MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_DrawFilledRectangle(w / 2, 0, w / 2, h, MAX7219_COLOR_ON));
	MAX_CHECK(MAX7219_UpdateScreen());
	HAL_Delay(1500);

	/* -- ToggleInvert: el MAX7219 no invierte por hardware, se invierte el buffer y hace falta UpdateScreen -- */
	MAX_CHECK(MAX7219_ToggleInvert());
	MAX_CHECK(MAX7219_UpdateScreen());
	HAL_Delay(1500);

	/* -- Mientras está activa, los colores se invierten: Clear deja todos los LEDs encendidos -- */
	MAX_CHECK(MAX7219_Clear());
	HAL_Delay(1500);

	/* -- Segunda llamada: vuelve al estado normal (buffer invertido de nuevo -> todo apagado) -- */
	MAX_CHECK(MAX7219_ToggleInvert());
	MAX_CHECK(MAX7219_UpdateScreen());
}

static void MAX7219_TestBrillo(void)
{
	/* -- Rampa de brillo 0 -> 15 -> 0 sobre texto (no con la matriz llena: a brillo máximo
	 *    los 4 módulos encendidos pueden pedir más de 1 A a la fuente de 5 V) -- */
	MAX_CHECK(MAX7219_PrintString("Luz"));
	for (uint8_t nivel = 0; nivel <= MAX7219_INTENSITY_MAX; nivel++)
	{
		MAX_CHECK(MAX7219_SetIntensity(nivel));
		HAL_Delay(150);
	}
	for (int8_t nivel = MAX7219_INTENSITY_MAX; nivel >= 0; nivel--)
	{
		MAX_CHECK(MAX7219_SetIntensity((uint8_t)nivel));
		HAL_Delay(150);
	}

	/* -- Fuera de rango -- */
	MAX_EXPECT(MAX7219_SetIntensity(MAX7219_INTENSITY_MAX + 1U), MAX7219_INVALID_PARAM);

	/* Restaura el brillo de trabajo para el resto de los tests */
	MAX_CHECK(MAX7219_SetIntensity(TEST_BRILLO));
}

static void MAX7219_TestScroll(void)
{
	/* -- Scroll: el texto entra por la derecha y se desplaza hacia la izquierda -- */
	MAX_CHECK(MAX7219_Clear());
	MAX_CHECK(MAX7219_ScrollString("Matriz de LEDs  ", TEST_SCROLL_MS));
}

static void MAX7219_TestSleep(void)
{
	/* -- SetSleep: la matriz se apaga 1.5 s y al despertar vuelve la misma imagen
	 *    (los registros de dígito se conservan en shutdown, no hace falta redibujar) -- */
	MAX_CHECK(MAX7219_PrintString("Zzz"));
	HAL_Delay(1000);

	MAX_CHECK(MAX7219_SetSleep(1));
	HAL_Delay(1500);
	MAX_CHECK(MAX7219_SetSleep(0));
}

static void MAX7219_TestErrores(void)
{
	const uint16_t w = MAX7219_GetWidth();
	const uint16_t h = MAX7219_GetHeight();

	/* -- Coordenadas fuera de pantalla: no recortan y se rechazan sin dibujar nada -- */
	MAX_EXPECT(MAX7219_GotoXY(w, 0), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_GotoXY(0, h), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_DrawLine(0, 0, w, 0, MAX7219_COLOR_ON), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_DrawRectangle(0, 0, 4, h, MAX7219_COLOR_ON), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_DrawFilledRectangle(w, 0, 4, 4, MAX7219_COLOR_ON), MAX7219_INVALID_PARAM);

	/* -- Cadenas NULL (GetStringWidth no falla: devuelve 0) -- */
	MAX_EXPECT(MAX7219_PrintString(NULL), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_ScrollString(NULL, TEST_SCROLL_MS), MAX7219_INVALID_PARAM);
	if (MAX7219_GetStringWidth(NULL) != 0U || MAX7219_GetStringWidth("") != 0U)
	{
		Error_Handler();
	}

	/* -- Init con parámetros inválidos: se rechaza antes de tocar el estado del driver -- */
	MAX_EXPECT(MAX7219_Init(NULL, GPIOB, GPIO_PIN_1, TEST_NUM_DEV), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_Init(&hspi2, NULL, GPIO_PIN_1, TEST_NUM_DEV), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_Init(&hspi2, GPIOB, 0, TEST_NUM_DEV), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_Init(&hspi2, GPIOB, GPIO_PIN_1, 0), MAX7219_INVALID_PARAM);
	MAX_EXPECT(MAX7219_Init(&hspi2, GPIOB, GPIO_PIN_1, MAX7219_MAX_DEV + 1U), MAX7219_INVALID_PARAM);

	/* El driver sigue operativo tras los rechazos */
	MAX_CHECK(MAX7219_PrintString("Err"));
}

static void MAX7219_TestPreInit(void)
{
	/* -- Antes de Init: todo debe devolver NOT_INITIALIZED sin tocar el bus -- */
	if (MAX7219_GetWidth() != 0U || MAX7219_GetHeight() != 0U)
	{
		Error_Handler();
	}
	MAX_EXPECT(MAX7219_Clear(), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_Fill(MAX7219_COLOR_ON), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_DrawPixel(0, 0, MAX7219_COLOR_ON), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_Putc('X', MAX7219_COLOR_ON), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_UpdateScreen(), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_PrintString("X"), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_ScrollString("X", TEST_SCROLL_MS), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_SetPixel(0, 0, 1), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_SetIntensity(TEST_BRILLO), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_SetSleep(1), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_ToggleInvert(), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_DeInit(), MAX7219_NOT_INITIALIZED);

	/* -- GetStringWidth no depende de Init: 7 columnas por carácter -- */
	if (MAX7219_GetStringWidth("Hola") != 28U)
	{
		Error_Handler();
	}
}

static void MAX7219_TestDeInit(void)
{
	/* -- Re-Init sobre un driver ya inicializado: reconfigura la matriz y la limpia -- */
	MAX_CHECK(MAX7219_PrintString("Re"));
	HAL_Delay(1000);
	MAX_CHECK(MAX7219_Init(&hspi2, GPIOB, GPIO_PIN_1, TEST_NUM_DEV));
	HAL_Delay(1000);

	/* -- DeInit apaga la matriz y libera la configuración: las llamadas posteriores
	 *    deben devolver NOT_INITIALIZED sin tocar el bus -- */
	MAX_CHECK(MAX7219_PrintString("Off"));
	HAL_Delay(1000);
	MAX_CHECK(MAX7219_DeInit());
	MAX_EXPECT(MAX7219_DeInit(), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_Clear(), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_PrintString("X"), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_SetPixel(0, 0, 1), MAX7219_NOT_INITIALIZED);
	MAX_EXPECT(MAX7219_SetSleep(0), MAX7219_NOT_INITIALIZED);
	if (MAX7219_GetWidth() != 0U)
	{
		Error_Handler();
	}
	HAL_Delay(1500);

	/* -- Re-Init: el driver debe poder arrancar de nuevo tras un DeInit -- */
	MAX_CHECK(MAX7219_Init(&hspi2, GPIOB, GPIO_PIN_1, TEST_NUM_DEV));
	MAX_CHECK(MAX7219_PrintString("OK"));
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
