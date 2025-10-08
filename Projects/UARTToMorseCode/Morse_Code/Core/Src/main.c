/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "ctype.h"
#include "string.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Tiempos Morse (en milisegundos)
#define DOT 200	              	// Punto: 200 ms
#define DASH (DOT * 3)        	// Raya: 600 ms
#define SYMBOL_SPACE DOT      	// Espacio entre símbolos de una letra: 200 ms
#define LETTER_SPACE (DOT * 3) 	// Espacio entre letras: 600 ms
#define WORD_SPACE (DOT * 7)   	// Espacio entre palabras: 1400 ms

#define BUFFER_SIZE 100         // Tamaño de buffer para recepción
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t RxData[BUFFER_SIZE];    // Buffer de recepción
uint8_t FinalData[BUFFER_SIZE]; // Buffer para cadena limpia
uint8_t RxByte;                 // Buffer para recibir 1 byte
volatile uint8_t indx = 0;      // Manejador del índice de la cadena recibida
volatile bool newMessageReady = false;  //Señal de listo

// Tabla de letras A-Z en Morse
char* letters[] = {
		".-",	// A
		"-...",	// B
		"-.-.",	// C
		"-..",	// D
		".",	// E
		"..-.",	// F
		"--.",	// G
		"....",	// H
		"..",	// I
		".---",	// J
		"-.-",	// K
		".-..",	// L
		"--",	// M
		"-.",	// N
		"---",	// O
		".--.",	// P
		"--.-",	// Q
		".-.",	// R
		"...",	// S
		"-",	// T
		"..-",	// U
		"...-",	// V
		".--",	// W
		"-..-",	// X
		"-.--",	// Y
		"--.."	// Z
};

// Tabla de números 0-9 en Morse
char* numbers[] = {
		"-----",	// 0
		".----", 	// 1
		"..---", 	// 2
		"...--", 	// 3
		"....-", 	// 4
		".....",	// 5
		"-....", 	// 6
		"--...", 	// 7
		"---..", 	// 8
		"----."		// 9
};

// Mensaje a transmitir
char message[] = "SOS";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/**
  * @brief  Transmite una cadena completa en código Morse
  * @param  str: Puntero a la cadena a transmitir
  * @retval None
  */
void transmitString(char* str);

/**
  * @brief  Transmite un carácter en código Morse
  * @param  c: Carácter a transmitir (letra, número o espacio)
  * @retval None
  */
void transmitChar(char c);

/**
  * @brief  Reproduce una secuencia Morse (puntos y rayas)
  * @param  sequence: Puntero a la secuencia (ej: ".-.")
  * @retval None
  */
void flashSequence(char* sequence);

/**
  * @brief  Reproduce un punto o raya en el LED
  * @param  dotOrDash: '.' para punto, '-' para raya
  * @retval None
  */
void flashDotOrDash(char dotOrDash);

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
  /* USER CODE BEGIN 2 */

  // Inicializar buffers
  memset(RxData, 0, BUFFER_SIZE);
  memset(FinalData, 0, BUFFER_SIZE);

  // Iniciar recepeción UART por interrupción
  HAL_UART_Receive_IT(&huart1, &RxByte, 1);

  // Mensaje de bienvenida
  char welcome[] = "Traductor Morse listo. Envie texto terminado en ENTER.\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)welcome, strlen(welcome), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Verificar si hay un nuevo mensaje completo
    if(newMessageReady)
    {
      // Copiar mensaje recibido a FinalData
      memcpy(FinalData, RxData, indx);
      FinalData[indx] = '\0';  // Asegurar terminación nula

      // Echo del mensaje recibido
      char echo[] = "\n\rTransmitiendo: ";
      HAL_UART_Transmit(&huart1, (uint8_t*)echo, strlen(echo), HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart1, FinalData, strlen((char*)FinalData), HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

      // Transmitir en morse
      transmitString((char*)FinalData);

      // Mensaje completado
      char done[] = "\r\nTransmisión completada. \r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)done, strlen(done), HAL_MAX_DELAY);

      // Limpiar buffers y bandera
      memset(RxData, 0, BUFFER_SIZE);
      memset(FinalData, 0, BUFFER_SIZE);
      indx = 0;
      newMessageReady = false; 
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB2 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Transmite una cadena completa en código Morse
  * @param  str: Puntero a la cadena a transmitir
  * @retval None
  */
void transmitString(char* str)
{
  int i = 0;
  while(str[i] != '\0')
  {
    transmitChar(str[i]);
    i++;
  }
}

/**
  * @brief  Transmite un carácter en código Morse
  * @param  c: Carácter a transmitir (letra, número o espacio)
  * @retval None
  */
void transmitChar(char c)
{
  // Convertir a mayúscula si es letra
  c = toupper(c);

  if(c >= 'A' && c <= 'Z')
  {
    // Es una letra
    flashSequence(letters[c - 'A']);
    HAL_Delay(LETTER_SPACE);
  }
  else if(c >= '0' && c <= '9')
  {
    // Es un número
    flashSequence(numbers[c - '0']);
    HAL_Delay(LETTER_SPACE);
  }
  else if(c == ' ')
  {
    // Espacio entre palabras
    // Ya hay LETTER_SPACE del carácter anterior
    // Añadimos el resto para completar WORD_SPACE
    HAL_Delay(WORD_SPACE - LETTER_SPACE);
  }
  // Otros caracteres se ignoran
}

/**
  * @brief  Reproduce una secuencia Morse (puntos y rayas)
  * @param  sequence: Puntero a la secuencia (ej: ".-.")
  * @retval None
  */
void flashSequence(char* sequence)
{
  int i = 0;
  while(sequence[i] != '\0')
  {
	flashDotOrDash(sequence[i]);
	i++;

	// Espacio entre símbolos (excepto después del último)
	if(sequence[i] != '\0')
	{
	  HAL_Delay(SYMBOL_SPACE);
	}
  }
}

/**
  * @brief  Reproduce un punto o raya en el LED
  * @param  dotOrDash: '.' para punto, '-' para raya
  * @retval None
  */
void flashDotOrDash(char dotOrDash)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

	if(dotOrDash == '.')
	{
		HAL_Delay(DOT);
	}
	else  // '-'
	{
		HAL_Delay(DASH);
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
}

/**
  * @brief  Callback de recepción UART
  * @param  huart: Handle del UART
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Verificar si es ENTER (carriage return o line feed)
	if(RxByte == '\r' || RxByte == '\n')
	{
		// Marcar que hay un mensaje completo
		if(indx > 0) // Solo si hay datos
		{
		  newMessageReady = true;
		}
	  }
	  else if(RxByte == 127 || RxByte == 8)	// Backspace o DEL
	  {
		  // Eliminar último carácter
		  if(indx > 0)
		  {
			indx--;
			RxData[indx] = 0;
			// Echo del backspace
			HAL_UART_Transmit(&huart1, (uint8_t*)"\b \b", 3, 10);
		  }
	  }
	  else if(indx < BUFFER_SIZE - 1)  // Protección contra desbordamiento
	  {
		// Almacenar el byte recibido
		RxData[indx] = RxByte;
		indx++;

		// Echo del carácter recibido
		HAL_UART_Transmit(&huart1, &RxByte, 1, 10);
	  }
	  // Reiniciar recepción para el siguiente byte
	  HAL_UART_Receive_IT(&huart1, &RxByte, 1);
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
  __disable_irq();
  while (1)
  {
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
