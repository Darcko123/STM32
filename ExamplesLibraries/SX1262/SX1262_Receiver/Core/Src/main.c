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
#include "ANSII_Codes.h"
#include "SX1262.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

SX1262_Status_t status;

// Configuración LoRa — debe ser IDÉNTICA a la del transmisor
lora_config_t mi_config = {
    .frequency        = 915000000,          // 915 MHz
    .spreading_factor = 11,                 // SF11
    .bandwidth        = BW_250_KHZ,         // 250 kHz
    .coding_rate      = CR_4_5,             // CR 4/5
    .tx_power         = 20,                 // +20 dBm (no aplica en RX, pero se inicializa)
    .preamble_len     = 8,                  // 8 símbolos
    .iq_inverted      = false,              // Modo estándar
    .network_mode     = LORA_NETWORK_PUBLIC,// Sync Word 0x34 (LoRaWAN / público)
    .lora_sync_word   = 0,                  // Sin sync word personalizado
    .config_pending   = false
};

// Buffer de recepción
#define RX_BUFFER_SIZE  255
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_length = 0;

// Contador de paquetes recibidos
uint32_t packet_counter = 0;

// Timeout de recepción en ms  (0 = bloqueante sin límite de tiempo)
#define RX_TIMEOUT_MS   10000   // 10 segundos

char buffer[150];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* --- Banner de inicio --- */
  sprintf(buffer, "\r\n%s==============================%s\r\n", COLOR_BOLD, COLOR_RESET);
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
  sprintf(buffer, "%s  SX1262 — Modo Receptor LoRa  %s\r\n", COLOR_BRIGHT_CYAN, COLOR_RESET);
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
  sprintf(buffer, "%s==============================%s\r\n\r\n", COLOR_BOLD, COLOR_RESET);
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

  /* --- Inicialización del módulo SX1262 --- */
  status = SX1262_Init(&hspi1,
                       NSS_GPIO_Port,  NSS_Pin,
                       BUSY_GPIO_Port, BUSY_Pin,
                       DIO_GPIO_Port,  DIO_Pin,
                       RST_GPIO_Port,  RST_Pin
                      );

  if (status != SX1262_OK)
  {
      sprintf(buffer, "%s[ERROR] SX1262_Init falló. Código: %d%s\r\n", COLOR_BRIGHT_RED, status, COLOR_RESET);
      HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
      Error_Handler();
  }

  sprintf(buffer, "%s[OK] SX1262 inicializado correctamente.%s\r\n", COLOR_BRIGHT_GREEN, COLOR_RESET);
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

  /* --- Aplicar configuración LoRa (idéntica al transmisor) --- */
  if (SX1262_ApplyConfig(&mi_config) != SX1262_OK)
  {
      sprintf(buffer, "%s[ERROR] SX1262_ApplyConfig falló.%s\r\n", COLOR_BRIGHT_RED, COLOR_RESET);
      HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
      Error_Handler();
  }

  sprintf(buffer, "%s[OK] Configuración LoRa aplicada. Esperando paquetes...%s\r\n\r\n",
          COLOR_BRIGHT_GREEN, COLOR_RESET);
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      sprintf(buffer, "%s[RX] Esperando paquete (timeout: %d ms)...%s\r\n",
              COLOR_YELLOW, RX_TIMEOUT_MS, COLOR_RESET);
      HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

      // Limpiar el buffer antes de cada recepción
      memset(rx_buffer, 0, sizeof(rx_buffer));
      rx_length = 0;

      // Intentar recibir un paquete
      status = SX1262_Receive(rx_buffer, &rx_length, RX_TIMEOUT_MS);

      if (status == SX1262_OK)
      {
          packet_counter++;

          // Paquete recibido: mostrar datos por UART
          sprintf(buffer, "%s[OK] Paquete #%lu recibido (%d bytes):%s\r\n",
                  COLOR_BRIGHT_GREEN, packet_counter, rx_length, COLOR_RESET);
          HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

          // Mostrar payload como string (asumiendo texto ASCII del transmisor)
          // Se añade terminador nulo por seguridad antes de imprimir
          if (rx_length < RX_BUFFER_SIZE)
          {
              rx_buffer[rx_length] = '\0';
          }
          sprintf(buffer, "  Payload : \"%s\"\r\n", (char*)rx_buffer);
          HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);

          HAL_Delay(100);
      }
      else if (status == SX1262_TIMEOUT)
      {
          sprintf(buffer, "%s[TIMEOUT] No se recibió paquete en %d ms.%s\r\n",
                  COLOR_BRIGHT_YELLOW, RX_TIMEOUT_MS, COLOR_RESET);
          HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
      }
      else
      {
          // Error de recepción (CRC, header inválido, error SPI, etc.)
          sprintf(buffer, "%s[ERROR] Fallo en recepción. Código: %d%s\r\n",
                  COLOR_BRIGHT_RED, status, COLOR_RESET);
          HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
      }

      sprintf(buffer, "\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO_Pin */
  GPIO_InitStruct.Pin = DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
