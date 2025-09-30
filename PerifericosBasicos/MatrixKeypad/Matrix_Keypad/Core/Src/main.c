/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Controlador de teclado matricial 4x4 con salida UART
  ******************************************************************************
  * @description
  * Este programa implementa un escáner de teclado matricial 4x4 para microcontroladores STM32.
  * Características:
  * - Antirrebote de hardware con retardo configurable
  * - Detección de pulsaciones de teclas con prevención de repetición
  * - Salida UART para la pantalla del terminal
  * - Borrado de la pantalla del terminal entre pulsaciones de teclas
  * 
  * Configuración de hardware:
  * - Filas (R1-R4): Pines de salida configurados como push-pull
  * - Columnas (C1-C4): Pines de entrada con resistencias pull-up
  * - UART1: 115200 baudios, configuración 8N1
  * 
  * Keypad Layout:
  *     C1   C2   C3   C4
  * R1  1    2    3    A
  * R2  4    5    6    B
  * R3  7    8    9    C
  * R4  *    0    #    D
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
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Configuración del tiempo de debounce (en milisegundos)
#define DEBOUNCE_TIME       50      // Tiempo mínimo entre lecturas de teclas
#define KEY_REPEAT_TIME     200     // Tiempo antes de permitir la repetición de la misma tecla
#define KEY_RELEASE_TIME    500     // Tiempo de espera antes de borrar la última tecla

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Variables de seguimiento del estado del teclado
static uint32_t lastKeyTime = 0;  // Marca de tiempo de la última pulsación de tecla válida
static char lastKey = 0;          // Última tecla presionada
static uint8_t keyReleased = 1;   // Bandera para rastrear si se soltó la tecla

// Variables de búfer para la transmisión UART
char pressedKey;      // Tecla actual
char keyPressed;      // Valor de tecla para ser transmitida
char buffer[10];      // Búfer para formateo de sprintf

// Secuencias de escape ANSI para control del terminal
//uint8_t cls[] = "\e[1;1H\e[2J";   // Mueve el cursos a (1,1) y limpia la pantalla
uint8_t clearLine[] = "\r\033[K";        // Retorno de carro + borrar hasta el final de la línea
uint8_t homeCursor[] = "\r";             // Solo retorno de carro (vuelve al inicio de línea)
uint8_t showPrompt[] = "KEY = ";         // Prompt fijo
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/**
 * @brief Escanea el teclado de matriz 4x4 y devuelve la tecla presionada
 * @retval char: carácter ASCII de la tecla presionada, o 0 si no se presionó ninguna tecla
 */
char Read_Keypad(void);
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

  // Enviar prompt inicial
  HAL_UART_Transmit(&huart1, (uint8_t*)showPrompt, strlen(showPrompt), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Escanea la matriz del teclado para detectar pulsaciones de teclas
	keyPressed = Read_Keypad();

    // Transmitir solo si se detectó una tecla válida (retorno distinto de cero)
    if(keyPressed != 0)
    {
    	// Borrar la línea actual y mostrar la nueva tecla
		HAL_UART_Transmit(&huart1, clearLine, strlen((char*)clearLine), 1000);

		// Mostrar el prompt con la tecla presionada
		sprintf(buffer, "KEY = %c", keyPressed);

		// Limpiar la pantalla del terminal usando la secuencia de escape ANSI
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
    }else
    {
    	// Cuando no hay tecla, mantener solo el prompt visible
		// Esto se hace automáticamente ya que no borramos la pantalla completa
		// Solo actualizamos si es necesario mantener consistencia
    	static uint32_t lastNoKeyTime = 0;
		uint32_t currentTime = HAL_GetTick();

		// Actualizar el prompt vacío periódicamente (cada 2 segundos) para mantenerlo visible
		if ((currentTime - lastNoKeyTime) > 2000)
		{
			HAL_UART_Transmit(&huart1, homeCursor, strlen((char*)homeCursor), 1000);
			HAL_UART_Transmit(&huart1, showPrompt, strlen((char*)showPrompt), 1000);
			lastNoKeyTime = currentTime;
		}
    }

    // Pequeña pausa para evitar uso excesivo de CPU
	HAL_Delay(10);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, R4_Pin|R3_Pin|R2_Pin|R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : C4_Pin C3_Pin C2_Pin C1_Pin */
  GPIO_InitStruct.Pin = C4_Pin|C3_Pin|C2_Pin|C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R4_Pin R3_Pin R2_Pin R1_Pin */
  GPIO_InitStruct.Pin = R4_Pin|R3_Pin|R2_Pin|R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Escanea el teclado matricial 4x4 para detectar pulsaciones de teclas
 * @details Algoritmo de escaneo
 *          1. Configura todas las filas en HIGH
 *          2. Para cada fila
 *            a) Configura la fila actual en LOW
 *            b) Lee tolas las columnas
 *            c) Si la columna es LOW, la tecla presionada es [fila][columna]
 *            d) Restaura las filas en HIGH
 *          3. Implementa la eliminación de rebotes y la prevención de repetición
 * 
 * @note El escaneo del teclado funciona así:
 *          - Poner una fila en LOW a la vez
 *          - Leer los pines de columna (con pull-ups)
 *          - Cuando se pulsa una tecla, conecta la fila con la columna
 *          - El LOW de la fila se propaga a la columna
 *          - Esto crea una lectura LOW en el pin de la columna     
 * 
 * @retval char: carácter ASCII de la tecla presionada, o 0 si no se presionó ninguna tecla
 */
char Read_Keypad(void)
{
  // Keypad character mapping
	char keyMap[4][4] =
	{
		{'1', '2', '3', 'A'}, // Fila 1
		{'4', '5', '6', 'B'}, // Fila 2
		{'7', '8', '9', 'C'}, // FIla 3
		{'*', '0', '#', 'D'}  // Fila 4
	};

  // Definiciones de puerto GPIO para cada fila
	GPIO_TypeDef* rowPorts[4] = {
    R1_GPIO_Port, 
    R2_GPIO_Port, 
    R3_GPIO_Port, 
    R4_GPIO_Port
  };

  // Definiciones de pines GPIO paracada fila
	uint16_t rowPins[4] = {
    R1_Pin, 
    R2_Pin, 
    R3_Pin, 
    R4_Pin
  };

  // Definiciones de puerto GPIO para cada columna
	GPIO_TypeDef* colPorts[4] ={
    C1_GPIO_Port, 
    C2_GPIO_Port, 
    C3_GPIO_Port, 
    C4_GPIO_Port
  };

  // Definiciones de pines GPIO paracada columna
	uint16_t colPins[4] = {
    C1_Pin, 
    C2_Pin, 
    C3_Pin, 
    C4_Pin
  };

	// Obtener la lectura del tiempo transcurrido
	uint32_t currentTime = HAL_GetTick();

  // COMPROBACIÓN DE REBOTES: Evitar lecturas demasiado frecuentes
  // Si no ha transcurrido suficiente tiempo desde la última lectura, volver inmediatamente
	if ((currentTime - lastKeyTime) < DEBOUNCE_TIME)
	{
		return 0;  // No leer si no ha pasado el tiempo de debounce
	}

  // INICIALIZACIÓN: Configura todos las filas en HIGH antes de escanear
  // Esto asegura empezar en un estadi conocido
	for(int i = 0; i < 4; i++)
	{
		HAL_GPIO_WritePin(rowPorts[i], rowPins[i], GPIO_PIN_SET);
	}

	// CICLO DE ESCANEO PRINCIPAL: Verifica cada fila secuencialmente
	for(int row = 0; row < 4; row++)
	{
		// PASO 1: Activar fila actual (LOW)
		HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_RESET);

		// PASO 2: Pequeño delay para permitir que la señal se estabilice
    // Esto evita lecturas falsas debido a transitorios de conmutación GPIO
		HAL_Delay(1);

    // PASO 3: Lee todas las columnas por fila
		for(int col = 0; col < 4; col++)
		{
      // Comprobar si la columna está en LOW (la tecla presionada conecta la fila a la columna)
			if(HAL_GPIO_ReadPin(colPorts[col], colPins[col]) == GPIO_PIN_RESET)
			{
        // Tecla detectada en la posición [fila][columna]
				pressedKey = keyMap[row][col];

        // LÓGICA DE PREVENCIÓN DE REPETICIÓN DE TECLAS:
        // Permitir la misma pulsación de tecla solo si:
        //    1. Es una tecla diferente a la anterior, O
        //    2. Ha transcurrido suficiente tiempo desde la última pulsación de la misma tecla
				if(pressedKey != lastKey || (currentTime - lastKey) > KEY_REPEAT_TIME)
				{
          // Actualizar el estado de las variables
					lastKey = pressedKey;
					lastKeyTime = currentTime;
          keyReleased = 0;  // Marcar la tecla como presionada actualmente

					// Restaurar todas las filas a HIGH
					for(int i = 0; i < 4; i++)
					{
						HAL_GPIO_WritePin(rowPorts[i], rowPins[i], GPIO_PIN_SET);
					}

					return pressedKey;  // Retorna la tecla presionada
				}
			}
		}

		// PASO 4: Restaurar fila actual a HIGH después de comprobar todas las columnas
		HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_SET);
	}

  // DETECCIÓN DE TECLA LIBERADA:
  // Si no se pulsa ninguna tecla y ha transcurrido suficiente tiempo, se borra la última tecla.
	// Esto permite volver a pulsar la misma tecla después de soltarla.
	if((currentTime - lastKeyTime) > KEY_RELEASE_TIME)
	{
		lastKey = 0;
    keyReleased = 1;
	}

	return 0;	// No hay tecla presionada
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
