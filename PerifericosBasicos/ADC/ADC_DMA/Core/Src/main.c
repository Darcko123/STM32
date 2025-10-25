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
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TIMEOUT       100   // Timeout en ms para transmisión UART
#define BUFFER_SIZE        20    // Tamaño del buffer de transmisión
#define ADC_BUFFER_SIZE    1     // Número de conversiones ADC a almacenar
#define VREF               3.3f  // Voltaje de referencia del ADC (V)
#define ADC_RESOLUTION     4095  // Resolución ADC de 12 bits (2^12 - 1)
#define DISPLAY_DELAY      100   // Delay entre transmisiones UART (ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/**
 * @brief Buffer para almacenar valores convertidos por el ADC
 * @note  - Tipo uint16_t porque el ADC de 12 bits cabe en 16 bits
 *        - Array de tamaño 1 para un solo canal
 *        - El DMA escribe directamente en esta ubicación
 *        - En modo circular, se actualiza continuamente
 *
 * IMPORTANTE: Se usa uint16_t en lugar de uint32_t porque:
 * - El ADC del STM32F103 es de 12 bits (0-4095)
 * - uint16_t optimiza el uso de memoria
 * - La función HAL_ADC_Start_DMA acepta uint32_t* pero el cast es seguro
 */
uint16_t ADC_VAL[ADC_BUFFER_SIZE];

/**
 * @brief Buffer para formatear la cadena a transmitir por UART
 * @note  Almacena el mensaje "ADC: XXXX (X.XXV)\r\n"
 */
char buffer[BUFFER_SIZE];

/**
 * @brief Bandera volátil que indica cuando hay datos ADC listos
 * @note  - volatile: Indica al compilador que esta variable puede cambiar
 *          inesperadamente (desde una ISR), evitando optimizaciones incorrectas
 *        - Se establece en 1 en el callback del DMA
 *        - Se limpia en 0 en el bucle principal después de procesar
 *
 * PATRÓN DE USO:
 * - ISR: adc_ready = 1 (señaliza datos listos)
 * - Main: if(adc_ready) { procesar(); adc_ready = 0; }
 */
volatile uint8_t adc_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /**
   * ========== INICIALIZACIÓN DEL ADC EN MODO DMA ==========
   *
   * HAL_ADC_Start_DMA():
   * - Inicia el ADC en modo DMA con conversión continua
   * - Configura el DMA para transferir datos automáticamente
   * - El DMA opera en modo circular (reinicio automático)
   *
   * PARÁMETROS:
   * @param &hadc1: Puntero al manejador del ADC
   * @param (uint32_t *)ADC_VAL: Dirección de memoria destino
   *        - Se hace cast a uint32_t* aunque ADC_VAL es uint16_t[]
   *        - Esto es seguro porque la HAL maneja correctamente el tamaño
   * @param 1: Número de conversiones a transferir antes de generar interrupción
   *
   */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_VAL, ADC_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	   * ========== PROCESAMIENTO DE DATOS ADC ==========
	   *
	   * Este bucle espera la bandera adc_ready que es establecida
	   * por el callback del DMA cuando se completa una transferencia.
	   */
	  if(adc_ready == 1)
	  {
		  /* ========== PASO 1: Limpiar la bandera ========== */
		  /**
		    * Limpiamos la bandera inmediatamente para:
			* - Evitar procesar el mismo dato múltiples veces
			* - Sincronizar correctamente el flujo de datos
			*/
		  adc_ready = 0;

		  /* ========== PASO 2: Calcular el voltaje ========== */
		  /**
		   * Conversión de valor digital a voltaje analógico:
		   *
		   * Fórmula: Voltaje = (Valor_ADC / Resolución_Máxima) × VREF
		   *
		   * Donde:
		   * - ADC_VAL[0]: Valor digital del ADC (0-4095)
		   * - ADC_RESOLUTION: 4095 (2^12 - 1 para 12 bits)
		   * - VREF: Voltaje de referencia (típicamente 3.3V)
		   *
		   * Ejemplo:
		   * - Si ADC_VAL[0] = 2048 (mitad de escala)
		   * - voltage = (2048 / 4095) × 3.3V = 1.65V
		   *
		   * NOTA: Se usa 3.3f con 'f' para forzar aritmética de punto flotante
		   */
		  float voltage = (ADC_VAL[0] * VREF) / ADC_RESOLUTION;

		  /* ========== PASO 3: Formatear y transmitir ========== */
		  sprintf(buffer, "ADC: %u (%.2fV)\r\n", ADC_VAL[0], voltage);


		  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
	  }

	  /* ========== PASO 4: Pequeño delay para control de flujo ========== */
	  HAL_Delay(DISPLAY_DELAY);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Callback ejecutado cuando el DMA completa la conversión de todos los canales.
  * @param  hadc: puntero al manejador del ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	/**
	 * Verificación de instancia:
	 * - Buena práctica para sistemas con múltiples ADCs
	 * - Evita conflictos si se usan ADC1, ADC2, ADC3 simultáneamente
	 * - En este proyecto simple podría omitirse, pero es buena práctica
	 */
	if(hadc->Instance == ADC1)
	{
		adc_ready = 1;
	}
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
