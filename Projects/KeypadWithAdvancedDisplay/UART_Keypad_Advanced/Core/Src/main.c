/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Controlador de teclado de matriz 4x4 con interfaz de terminal mejorada
  * @author         : Daniel Ruiz
  * @date           : 01 Oct 2025
  ******************************************************************************
  * @description
  * Este programa implementa un escáner de teclado matricial 4x4 para microcontroladores STM32.
  * Características:
  * - Visualización de teclas en tiempo real con retroalimentación visual
  * - Salida codificada por colores mediante secuencias de escape ANSI
  * - Seguimiento del historial de teclas
  * - Display de estadísticas (total de teclas presionadas)
  * - Layout en terminal limpio y organizado
  * 
  * Configuración de hardware:
  * - Filas (R1-R4): Pines de salida configurados como push-pull
  * - Columnas (C1-C4): Pines de entrada con resistencias pull-up
  * - UART1: 115200 baudios, configuración 8N1
  * 
  * Requisitos de terminal
  *   - Debe de soportar secuencias de salida ANSI
  *   - Recomensadas: Putty, Tera Term, o similar
  * 
  * Keypad Layout:
  *     C1   C2   C3   C4
  * R1  1    2    3    A
  * R2  4    5    6    B
  * R3  7    8    9    C
  * R4  *    0    #    D
  * 
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

// TIempo de actualización de display
#define DISPLAY_UPDATE_MS   100     // Actualiza el display cada 100 ms

//Configuración de historial de teclas
#define HISTORY_SIZE        10      // Numero de teclas a mantener en el historial

// Códigos ANSI para salida a terminal
#define COLOR_RESET         "\033[0m"
#define COLOR_RED           "\033[31m"
#define COLOR_GREEN         "\033[32m"
#define COLOR_YELLOW        "\033[33m"
#define COLOR_BLUE          "\033[34m"
#define COLOR_MAGENTA       "\033[35m"
#define COLOR_CYAN          "\033[36m"
#define COLOR_WHITE         "\033[37m"
#define COLOR_BRIGHT_GREEN  "\033[92m"
#define COLOR_BRIGHT_CYAN   "\033[96m"
#define COLOR_BRIGHT_YELLOW "\033[93m"

// Formato de texto ANSI
#define TEXT_BOLD           "\033[1m"
#define TEXT_DIM            "\033[2m"
#define TEXT_UNDERLINE      "\033[4m"
#define TEXT_BLINK          "\033[5m"

// Control de cursor ANSI
#define CURSOR_HOME         "\033[H"
#define CLEAR_SCREEN        "\033[2J"
#define CLEAR_LINE          "\033[2K"
#define CURSOR_HIDE         "\033[?25l"
#define CURSOR_SHOW         "\033[?25h"


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

// Variables de estado del display
static uint32_t lastDisplayUpdate = 0;    // Última vez que el display fue actualizado
static uint32_t totalKeysPressed = 0;      // Cuenta total de teclas presionadas
static char     keyHistory[HISTORY_SIZE]; // Buffer circular para las teclas presionadas
static uint8_t  historyIndex = 0;         // Posición actual del buffer de historial

// Buffer para las cadenas formateadas
char displayBuffer[512];                  // Buffer formateado para salidas complejas

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

/**
 * @brief  Inicializa el display de la terminal
 * @retval None
 */
void Terminal_Init(void);

/**
 * @brief  Actualiza la pantalla del terminal con el estado actual del teclado
 * @param  currentKey: Tecla presionada actualmente (0 si no hay ninguna)
 * @retval None
 */
void Terminal_Update(char currentKey);

/**
 * @brief  Agrega una clave al buffer de historial
 * @param  key: Tecla a agregar
 * @retval None
 */
void AddToHistory(char key);

/**
 * @brief  Transmite una cadena a través de UART
 * @param  str: Cadena terminada en nulo para transmitir
 * @retval None
 */
void UART_Print(const char* str);

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
  char currentKey = 0;
  char previousKey = 0;
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

  // Pequeño delay para que termine de inicialziar la terminal
  HAL_Delay(500);

  // Inicializa el display de la terminal
  Terminal_Init();

  // Actualización inicial del display
  Terminal_Update(0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // Escanea el teclado matricial por teclas presionadas
    currentKey = Read_Keypad();

    // Revisa si hay nuevas teclas presionadas (diferente de la anterior)
    if(currentKey != 0 && currentKey != previousKey)
    {
      // Agrega la tecla al historial
      AddToHistory(currentKey);

      // Actualiza la tecla anterior
      previousKey = currentKey;

      //Actualiza el display inmediatamente cuando se presiona una tecla
      Terminal_Update(currentKey);
    }
    else if(currentKey == 0 && previousKey != 0)
    {
      // La tecla fue liberada
      previousKey = 0;
    }

    //Actualización periódica del display (cada DISPLAY_UPDATE_MS)
    uint32_t currentTime = HAL_GetTick();
    if((currentTime - lastDisplayUpdate) >= DISPLAY_UPDATE_MS)
    {
      lastDisplayUpdate = currentTime;
      Terminal_Update(currentKey);
    }

    // Pequeño delay para prevenir sobrecarga de CPU
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
 ******************************************************************************
 * @brief  Transmite una cadena a través de UART
 * @param  str: Cadena terminada en nulo para transmitir
 * @retval None
 ******************************************************************************
 */
void UART_Print(const char* str)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
}

/**
 ******************************************************************************
 * @brief  Inicializa el display de la terminal
 * @retval None
 ******************************************************************************
 */
void Terminal_Init(void)
{
  // Limpia la pantalla y esconde el cursor para un display más limpio
  UART_Print(CLEAR_SCREEN CURSOR_HOME CURSOR_HIDE);

  // Imprime el header con el título
  sprintf(displayBuffer, 
    "%s╔═══════════════════════════════════════════════╗%s\r\n"
    "%s║         %sMONITOR TECLADO MATRICIAL 4x4%s         %s║%s\r\n"
    "%s╠═══════════════════════════════════════════════╣%s\r\n\r\n",
    COLOR_CYAN, COLOR_RESET,
    COLOR_CYAN, TEXT_DIM, COLOR_RESET, COLOR_CYAN,COLOR_RESET,
    COLOR_CYAN, COLOR_RESET);
  UART_Print(displayBuffer);

  // Inicializa el buffer de historial con espacios
  for(int i =0 ; i < HISTORY_SIZE; i++)
  {
    keyHistory[i] = ' ';
  }
}

/**
 ******************************************************************************
 * @brief  Actualiza la pantalla del terminal con el estado actual del teclado
 * @param  currentKey: Tecla presionada actualmente (0 si no hay ninguna)
 * @retval None
 ******************************************************************************
 */
void Terminal_Update(char currentKey)
{
  // Mueve el cursos a la posición (5,1) para actualizar el área de estado
  UART_Print("\33[5;1H");

  // ═══════════════════════════════════════════════════════════
  //  Tecla actual centrada
  // ═══════════════════════════════════════════════════════════
  if(currentKey != 0)
  {
    sprintf(displayBuffer,
      "%s╔═══════════════════════════════════════════════╗%s\r\n"
      "%s║ TECLA ACTUAL: %s%s[%c]%s%s                             ║%s\r\n"
      "%s╚═══════════════════════════════════════════════╝%s\r\n\r\n",
      COLOR_GREEN, COLOR_RESET,
      COLOR_GREEN, TEXT_BOLD, COLOR_BRIGHT_YELLOW, currentKey, COLOR_RESET, COLOR_GREEN, COLOR_RESET,
      COLOR_GREEN, COLOR_RESET);
  }
  else
  {
    sprintf(displayBuffer,
      "%s╔═══════════════════════════════════════════════╗%s\r\n"
      "%s║ TECLA ACTUAL: %s[Esperando...]%s                  ║%s\r\n"
      "%s╚═══════════════════════════════════════════════╝%s\r\n\r\n",
      TEXT_DIM, COLOR_RESET,
      TEXT_DIM, COLOR_WHITE, TEXT_DIM, COLOR_RESET,
      TEXT_DIM, COLOR_RESET);
  }
  UART_Print(displayBuffer);

  // ═══════════════════════════════════════════════════════════
  // Historial de teclas
  // ═══════════════════════════════════════════════════════════
  sprintf(displayBuffer,
    "%s┌───────────────────────────────────────────────┐%s\r\n"
    "%s│ %sHISTORIAL DE TECLAS (Últimas %d teclas):%s      │%s\r\n"
    "%s├───────────────────────────────────────────────┤%s\r\n"
    "%s│ %s",
    COLOR_BLUE, COLOR_RESET,
    COLOR_BLUE, COLOR_BRIGHT_CYAN, HISTORY_SIZE, COLOR_BLUE, COLOR_RESET,
    COLOR_BLUE, COLOR_RESET,
    COLOR_BLUE, COLOR_YELLOW);
  UART_Print(displayBuffer);

  // Imprime el buffer del historial (Del más nuevo, al más antiguo)
  for(int i = 0; i < HISTORY_SIZE; i++)
  {
    int idx = (historyIndex - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
    if(keyHistory[idx] != ' ')
    {
      sprintf(displayBuffer, "[%c] ", keyHistory[idx]);
      UART_Print(displayBuffer);
    }else{
      UART_Print("[_] ");
    }
  }

  sprintf(displayBuffer, "%s      │%s\r\n"
	"%s└───────────────────────────────────────────────┘%s\r\n\r\n",
	COLOR_BLUE, COLOR_RESET,
	COLOR_BLUE, COLOR_RESET);
  UART_Print(displayBuffer);

  // ═══════════════════════════════════════════════════════════
  // Display de estadísticas
  // ═══════════════════════════════════════════════════════════

  // Primera parte
  sprintf(displayBuffer,
      "%s┌───────────────────────────────────────────────┐%s\r\n"
      "%s│ %sESTADÍSTICAS:%s                                 │%s\r\n"
      "%s├───────────────────────────────────────────────┤%s\r\n"
      "%s│ Total de Teclas Presionadas: ",
      COLOR_MAGENTA, COLOR_RESET,
      COLOR_MAGENTA, COLOR_BRIGHT_YELLOW, COLOR_MAGENTA, COLOR_RESET,
      COLOR_MAGENTA, COLOR_RESET,
      COLOR_MAGENTA);
  UART_Print(displayBuffer);

  // El número con su color
  snprintf(displayBuffer, sizeof(displayBuffer),
      " %s%-5lu%s",
      COLOR_BRIGHT_GREEN, totalKeysPressed, COLOR_MAGENTA);
  UART_Print(displayBuffer);

  // Parte final
  snprintf(displayBuffer, sizeof(displayBuffer),
      "           │%s\r\n"
      "%s└───────────────────────────────────────────────┘%s\r\n\r\n",
      COLOR_RESET,
      COLOR_MAGENTA, COLOR_RESET);
  UART_Print(displayBuffer);

  // ═══════════════════════════════════════════════════════════
  // Referencia de teclado matricial
  // ═══════════════════════════════════════════════════════════
  // Parte 1
  sprintf(displayBuffer,
	  "%s╔═══════════════════════════════════════════════╗%s\r\n"
	  "%s║            Distribución de teclado            ║%s\r\n"
	  "%s╠═══════════════════════════════════════════════╣%s\r\n"
	  "%s║             ┌────┬────┬────┬────┐             ║%s\r\n"
	  ,
	  COLOR_WHITE, COLOR_RESET,
	  COLOR_WHITE, COLOR_RESET,
	  COLOR_WHITE, COLOR_RESET,
	  COLOR_WHITE, COLOR_RESET
  );

  UART_Print(displayBuffer);

  // Parte2
  sprintf(displayBuffer,
	  "%s║             │ %s1%s  │ %s2%s  │ %s3%s  │ %sA%s  │             ║%s\r\n"
	  "%s║             ├────┼────┼────┼────┤             ║%s\r\n"
	  "%s║             │ %s4%s  │ %s5%s  │ %s6%s  │ %sB%s  │             ║%s\r\n"
	  "%s║             ├────┼────┼────┼────┤             ║%s\r\n"
	  ,
	  COLOR_WHITE, COLOR_BLUE, COLOR_WHITE, COLOR_BLUE, COLOR_WHITE, COLOR_BLUE, COLOR_WHITE, COLOR_RED, COLOR_WHITE, COLOR_RESET,
	  COLOR_WHITE, COLOR_RESET,
	  COLOR_WHITE, COLOR_BLUE, COLOR_WHITE, COLOR_BLUE, COLOR_WHITE, COLOR_BLUE, COLOR_WHITE, COLOR_RED, COLOR_WHITE, COLOR_RESET,
	  COLOR_WHITE, COLOR_RESET
  );

  UART_Print(displayBuffer);

  //Parte 3
  sprintf(displayBuffer,
	  "%s║             │ %s7%s  │ %s8%s  │ %s9%s  │ %sC%s  │             ║%s\r\n"
	  "%s║             ├────┼────┼────┼────┤             ║%s\r\n"
	  "%s║             │ %s*%s  │ %s0%s  │ %s#%s  │ %sD%s  │             ║%s\r\n"
	  "%s║             └────┴────┴────┴────┘             ║%s\r\n"
  ,
  	  COLOR_WHITE, COLOR_BLUE, COLOR_WHITE, COLOR_BLUE, COLOR_WHITE, COLOR_BLUE, COLOR_WHITE, COLOR_RED, COLOR_WHITE, COLOR_RESET,
  	  COLOR_WHITE, COLOR_RESET,
	  COLOR_WHITE, COLOR_RED, COLOR_WHITE, COLOR_BLUE, COLOR_WHITE, COLOR_RED, COLOR_WHITE, COLOR_RED, COLOR_WHITE, COLOR_RESET,
	  COLOR_WHITE, COLOR_RESET
  );
  UART_Print(displayBuffer);

  //Parte 4
  sprintf(displayBuffer,
	  "%s╚═══════════════════════════════════════════════╝%s\r\n"
  ,
  COLOR_WHITE, COLOR_RESET
  );
  UART_Print(displayBuffer);

  // Footer
  sprintf(displayBuffer,
    "\r\n%s%s  Estado: %sACTIVO%s | Presiona cualquier tecla para probar...%s\r\n",
    TEXT_DIM, COLOR_WHITE, COLOR_GREEN, COLOR_WHITE, COLOR_RESET);
  UART_Print(displayBuffer);
}

/**
 ******************************************************************************
 * @brief  Agrega una clave al buffer de historial
 * @param  key: Tecla a agregar
 * @retval None
 ******************************************************************************
 */
void AddToHistory(char key)
{
  keyHistory[historyIndex] = key;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  totalKeysPressed++;
}

/**
 ******************************************************************************
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
 ******************************************************************************
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
				char pressedKey = keyMap[row][col];

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
