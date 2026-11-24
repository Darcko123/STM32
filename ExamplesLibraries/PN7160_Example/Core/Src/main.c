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
#include "PN7160.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -- Tiempos de la batería de pruebas (ajustables) -- */
#define TEST_TAG_MS         20000U  /**< Espera máxima a que se acerque o retire un tag/celular */
#define TEST_TIMEOUT_MS     500U    /**< WaitTag sin tag en el campo: debe tardar esto... */
#define TEST_MARGEN_MS      250U    /**< ...más como mucho esto (RF_DEACTIVATE + su NTF, hasta PN7160_TIMEOUT_MS) */
#define TEST_PRESENTE_MS    1500U   /**< WaitTagRemoval con el tag apoyado: debe agotar este tiempo */
#define TEST_REDETECTAR_MS  2000U   /**< Un tag que sigue apoyado se vuelve a detectar en menos de esto */

/* -- Tags para TestTransceive: 1 = hay que acercar uno de ese tipo, 0 = no se tiene -- */
#define TEST_CON_T2T        1       /**< NTAG / Ultralight */
#define TEST_CON_ISODEP     1       /**< Tarjeta ISO-DEP (bancaria, DESFire, ...) */
#define TEST_CON_MFC        1       /**< MIFARE Classic 1K */

/* -- Clave con formato válido para las pruebas de errores (no toca RF) -- */
#define MFC_CLAVE           0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU

/* -- MIFARE Classic: las claves conocidas están en mfcClaves (PV) -- */
#define MFC_SECTORES        16U     /**< ponytail: solo 1K; la 4K tiene 40 (32-39 de 16 bloques) */
#define TEST_MFC_BLOQUE     60U     /**< Bloque de datos que se escribe y se restaura */

/* -- UID que anuncia el PN7160 en modo tarjeta (4 bytes, sin 0x08 ni 0x88 al inicio) -- */
#define TEST_CARD_UID       0x12U, 0x34U, 0x56U, 0x78U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PN_CHECK(x)  do { if ((x) != PN7160_OK) { Error_Handler(); } } while (0)
/* Para los tests de error: la llamada debe devolver exactamente el estado esperado */
#define PN_EXPECT(x, st)  do { if ((x) != (st)) { Error_Handler(); } } while (0)
/* Para los tests de valores: la condición debe cumplirse */
#define PN_ASSERT(c)  do { if (!(c)) { Error_Handler(); } } while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
PN7160_Status_t status;
static PN7160_Info_t info;              /* identificación leída tras el primer Init */
static PN7160_Tag_t tag;                /* último tag de WaitTag */
static PN7160_Tag_t tagIT;              /* copia del tag que entrega el callback */
static volatile uint8_t tagITListo;     /* lo pone PN7160_TagDetectedCallback */

/* Claves conocidas: en cada sector se prueban, en este orden, como clave A y como clave B */
static const uint8_t mfcClaves[][6] = {
	{ 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU },   /* de fábrica */
	{ 0xA0U, 0xA1U, 0xA2U, 0xA3U, 0xA4U, 0xA5U },   /* MAD */
	{ 0xD3U, 0xF7U, 0xD3U, 0xF7U, 0xD3U, 0xF7U },   /* NDEF */
};

/* Ni el bloque 0 (fabricante) ni un trailer: escribir ahí puede inutilizar la tarjeta */
_Static_assert(TEST_MFC_BLOQUE != 0U && (TEST_MFC_BLOQUE % 4U) != 3U && TEST_MFC_BLOQUE < 4U * MFC_SECTORES,
               "TEST_MFC_BLOQUE debe ser un bloque de datos");
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static PN7160_Status_t PN7160_Iniciar(void);
static void PN7160_TestSuite(void);
static void PN7160_TestPreInit(void);
static void PN7160_TestInfo(void);
static void PN7160_TestErrores(void);
static void PN7160_TestTimeout(void);
static void PN7160_TestLectura(void);
static void PN7160_TestTransceive(void);
static void PN7160_TestIT(void);
static void PN7160_TestModoTarjeta(void);
static void PN7160_TestDeInit(void);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf(CLEAR_SCREEN CURSOR_HOME COLOR_BOLD COLOR_CYAN "=== Pruebas PN7160 ===" COLOR_RESET "\r\n");
  PN7160_TestPreInit();

  status = PN7160_Iniciar();
  if(status != PN7160_OK)
  {
	  Error_Handler();
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  PN_CHECK(PN7160_GetInfo(&info));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    PN7160_TestSuite();
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PN_VEN_GPIO_Port, PN_VEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PN_VEN_Pin */
  GPIO_InitStruct.Pin = PN_VEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PN_VEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PN_IRQ_Pin */
  GPIO_InitStruct.Pin = PN_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PN_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* printf por USART1 (PA9): syscalls.c redirige _write a esta función débil */
int __io_putchar(int ch)
{
	uint8_t c = (uint8_t)ch;
	HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
	return ch;
}

/* IRQ del PN7160 (PB10, flanco de subida): solo marca la bandera, el trabajo lo hace PN7160_Process */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == PN_IRQ_Pin)
	{
		PN7160_IRQ_Handler();
	}
}

/* Redefine el callback __weak de la librería: copia el tag y avisa al bucle de TestIT */
void PN7160_TagDetectedCallback(const PN7160_Tag_t* t)
{
	tagIT = *t;
	tagITListo = 1U;
}

static PN7160_Status_t PN7160_Iniciar(void)
{
	return PN7160_Init(&hi2c1, PN_VEN_GPIO_Port, PN_VEN_Pin, PN_IRQ_GPIO_Port, PN_IRQ_Pin);
}

static void PN7160_ImprimirHex(const char* nombre, const uint8_t* d, uint16_t n)
{
	printf("  %s", nombre);
	for (uint16_t i = 0; i < n; i++)
	{
		printf(" %02X", d[i]);
	}
	printf("\r\n");
}

static void PN7160_Imprimir(const PN7160_Tag_t* t)
{
	printf("  Tec 0x%02X  Proto 0x%02X  ATQA %02X %02X  SAK 0x%02X\r\n",
	       t->tecnologia, t->protocolo, t->atqa[0], t->atqa[1], t->sak);
	PN7160_ImprimirHex("UID:", t->uid, t->uidLen);
}

/* Coherencia de un tag leído en modo lector: longitud de UID por tecnología y SAK por protocolo */
static void PN7160_CheckTag(const PN7160_Tag_t* t)
{
	switch (t->tecnologia)
	{
	case PN7160_TEC_NFC_A:
		PN_ASSERT(t->uidLen == 4U || t->uidLen == 7U || t->uidLen == 10U);
		PN_ASSERT(t->atqa[0] != 0U || t->atqa[1] != 0U);
		if (t->protocolo == PN7160_PROT_T2T)    { PN_ASSERT(t->sak == 0x00U); }
		if (t->protocolo == PN7160_PROT_ISODEP) { PN_ASSERT((t->sak & 0x20U) != 0U); }   /* bit ISO 14443-4 */
		if (t->protocolo == PN7160_PROT_MIFARE) { PN_ASSERT((t->sak & 0x18U) != 0U); }   /* 0x08 1K, 0x18 4K, 0x09 Mini */
		break;
	case PN7160_TEC_NFC_B:
		PN_ASSERT(t->uidLen == 4U);
		break;
	case PN7160_TEC_NFC_F:
	case PN7160_TEC_NFC_V:
		PN_ASSERT(t->uidLen == 8U);
		break;
	default:
		PN_ASSERT(0);
	}

	/* ATQA y SAK solo existen en NFC-A */
	if (t->tecnologia != PN7160_TEC_NFC_A)
	{
		PN_ASSERT(t->atqa[0] == 0U && t->atqa[1] == 0U && t->sak == 0U);
	}
}

/* Mismo tag en dos lecturas. Un NFCID1 de 4 bytes que empieza por 0x08 (ISO 14443-3) y el
   PUPI de NFC-B pueden ser aleatorios: cambian en cada activación, así que no se comparan */
static uint8_t PN7160_MismoTag(const PN7160_Tag_t* a, const PN7160_Tag_t* b)
{
	uint8_t uidAleatorio = (a->tecnologia == PN7160_TEC_NFC_B) ||
	                       (a->tecnologia == PN7160_TEC_NFC_A && a->uidLen == 4U && a->uid[0] == 0x08U);

	return a->tecnologia == b->tecnologia && a->protocolo == b->protocolo &&
	       a->uidLen == b->uidLen && a->sak == b->sak && memcmp(a->atqa, b->atqa, 2) == 0 &&
	       (uidAleatorio || memcmp(a->uid, b->uid, a->uidLen) == 0);
}

/* Pide un tag y espera hasta TEST_TAG_MS */
static void PN7160_EsperarTag(const char* que, uint8_t keepActive)
{
	printf("  Acerca %s...\r\n", que);
	uint32_t t0 = HAL_GetTick();
	status = PN7160_WaitTag(&tag, TEST_TAG_MS, keepActive);
	if (status != PN7160_OK)
	{
		printf("  WaitTag devolvio %d (1 = ERROR, 2 = TIMEOUT) a los %lu ms\r\n",
		       (int)status, (unsigned long)(HAL_GetTick() - t0));
		Error_Handler();
	}
	PN7160_Imprimir(&tag);
}

/* Deja el campo vacío: si queda algo (tag o celular) lo activa y espera a que se retire */
static void PN7160_EsperarRetirada(void)
{
	status = PN7160_WaitTag(&tag, 1000U, 1U);
	if (status == PN7160_TIMEOUT)
	{
		return;   /* ya no había nada */
	}
	PN_CHECK(status);

	printf("  Retira el tag...\r\n");
	PN_CHECK(PN7160_WaitTagRemoval(TEST_TAG_MS));
}

/* Atiende el descubrimiento IT hasta que llega el callback o pasan ms; 1 = llegó */
static uint8_t PN7160_ProcesarHasta(uint32_t ms)
{
	uint32_t t0 = HAL_GetTick();
	while (!tagITListo && (HAL_GetTick() - t0) < ms)
	{
		PN_CHECK(PN7160_Process());
	}
	return tagITListo;
}

/* Parámetros inválidos: se validan antes del estado, así que dan INVALID_PARAM con o sin Init */
static void PN7160_CheckParametros(void)
{
	static const uint8_t uid[] = { TEST_CARD_UID };
	uint8_t rx[4];
	uint16_t n = sizeof(rx);

	PN_EXPECT(PN7160_Init(NULL, PN_VEN_GPIO_Port, PN_VEN_Pin, PN_IRQ_GPIO_Port, PN_IRQ_Pin), PN7160_INVALID_PARAM);
	PN_EXPECT(PN7160_Init(&hi2c1, NULL, PN_VEN_Pin, PN_IRQ_GPIO_Port, PN_IRQ_Pin), PN7160_INVALID_PARAM);
	PN_EXPECT(PN7160_Init(&hi2c1, PN_VEN_GPIO_Port, PN_VEN_Pin, NULL, PN_IRQ_Pin), PN7160_INVALID_PARAM);

	PN_EXPECT(PN7160_GetInfo(NULL), PN7160_INVALID_PARAM);
	PN_EXPECT(PN7160_SetMode((PN7160_Mode_t)2), PN7160_INVALID_PARAM);

	PN_EXPECT(PN7160_SetCardUid(uid, 3U), PN7160_INVALID_PARAM);
	PN_EXPECT(PN7160_SetCardUid(uid, 5U), PN7160_INVALID_PARAM);
	PN_EXPECT(PN7160_SetCardUid(uid, 11U), PN7160_INVALID_PARAM);
	PN_EXPECT(PN7160_SetCardUid(NULL, 4U), PN7160_INVALID_PARAM);

	PN_EXPECT(PN7160_WaitTag(NULL, 10U, 0U), PN7160_INVALID_PARAM);

	PN_EXPECT(PN7160_Transceive(NULL, 1U, rx, &n), PN7160_INVALID_PARAM);
	PN_EXPECT(PN7160_Transceive(uid, 1U, NULL, &n), PN7160_INVALID_PARAM);
	PN_EXPECT(PN7160_Transceive(uid, 1U, rx, NULL), PN7160_INVALID_PARAM);
}

/* Con el módulo sin inicializar, todo debe devolver NOT_INITIALIZED sin tocar el bus */
static void PN7160_CheckSinInit(void)
{
	static const uint8_t uid[] = { TEST_CARD_UID };
	static const uint8_t clave[6] = { MFC_CLAVE };
	PN7160_Info_t inf;
	uint8_t rx[4], rx16[16] = { 0 };
	uint16_t n = sizeof(rx);

	PN_EXPECT(PN7160_DeInit(), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_GetInfo(&inf), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_SetMode(PN7160_MODE_READER), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_SetCardUid(uid, sizeof(uid)), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_SetCardUid(NULL, 0U), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_WaitTag(&tag, 10U, 0U), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_Transceive(NULL, 0U, rx, &n), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_ReleaseTag(), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_WaitTagRemoval(10U), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_MfcAuth(0U, 0U, clave), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_MfcRead(0U, rx16), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_MfcWrite(4U, rx16), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_StartDiscoveryIT(0U), PN7160_NOT_INITIALIZED);
	PN_EXPECT(PN7160_Process(), PN7160_NOT_INITIALIZED);

	/* Solo pone una bandera: no debe colgarse ni hacer que Process trabaje sin Init */
	PN7160_IRQ_Handler();
	PN_EXPECT(PN7160_Process(), PN7160_NOT_INITIALIZED);
}

static void PN7160_TestSuite(void)
{
	PN7160_TestInfo();
	HAL_Delay(2000);

	PN7160_TestErrores();
	HAL_Delay(2000);

	PN7160_TestTimeout();
	HAL_Delay(2000);

	PN7160_TestLectura();
	HAL_Delay(2000);

	PN7160_TestTransceive();
	HAL_Delay(2000);

	PN7160_TestIT();
	HAL_Delay(2000);

	PN7160_TestModoTarjeta();
	HAL_Delay(2000);

	PN7160_TestDeInit();
	HAL_Delay(2000);
}

static void PN7160_TestPreInit(void)
{
	/* -- Antes de Init: parámetros inválidos y NOT_INITIALIZED en todo -- */
	printf(COLOR_YELLOW "[PreInit]" COLOR_RESET "\r\n");
	PN7160_CheckParametros();
	PN7160_CheckSinInit();

	printf(COLOR_GREEN "PreInit OK" COLOR_RESET "\r\n");
}

static void PN7160_TestInfo(void)
{
	PN7160_Info_t inf;

	/* -- Model ID y firmware del CORE_RESET_NTF; GetInfo no habla con el chip -- */
	printf(COLOR_YELLOW "[Info]" COLOR_RESET "\r\n");
	PN_CHECK(PN7160_GetInfo(&inf));
	printf("  Modelo 0x%02X (%s)  Firmware %02X.%02X.%02X\r\n", inf.modelo,
	       (inf.modelo == 0x61U) ? "PN7160" : (inf.modelo == 0x71U) ? "PN7161" : "?",
	       inf.firmware[0], inf.firmware[1], inf.firmware[2]);
	PN_ASSERT(inf.modelo == 0x61U || inf.modelo == 0x71U);

	/* Es la copia del último Init: igual a la que se guardó al arrancar */
	PN_ASSERT(memcmp(&inf, &info, sizeof(inf)) == 0);

	printf(COLOR_GREEN "Info OK" COLOR_RESET "\r\n");
}

static void PN7160_TestErrores(void)
{
	PN7160_Info_t inf;
	uint8_t rx[4];
	uint16_t n = sizeof(rx);

	/* -- Ya inicializado: parámetros inválidos siguen dando INVALID_PARAM -- */
	printf(COLOR_YELLOW "[Errores]" COLOR_RESET "\r\n");
	PN7160_CheckParametros();

	/* Un Init con parámetros inválidos no desinicializa el módulo */
	PN_CHECK(PN7160_GetInfo(&inf));

	/* -- Sin tag activado: Transceive y WaitTagRemoval dan ERROR, ReleaseTag no hace nada -- */
	PN_EXPECT(PN7160_Transceive(NULL, 0U, rx, &n), PN7160_ERROR);
	PN_EXPECT(PN7160_WaitTagRemoval(100U), PN7160_ERROR);
	PN_CHECK(PN7160_ReleaseTag());

	/* -- MIFARE: NULL da INVALID_PARAM; sin tag activado, ERROR -- */
	static const uint8_t clave[6] = { MFC_CLAVE };
	uint8_t bloque[16] = { 0 };
	PN_EXPECT(PN7160_MfcAuth(0U, 0U, NULL), PN7160_INVALID_PARAM);
	PN_EXPECT(PN7160_MfcAuth(40U, 0U, clave), PN7160_INVALID_PARAM);    /* 4K llega al sector 39 */
	PN_EXPECT(PN7160_MfcAuth(0U, 2U, clave), PN7160_INVALID_PARAM);     /* keyB es 0 o 1 */
	PN_EXPECT(PN7160_MfcRead(0U, NULL), PN7160_INVALID_PARAM);
	PN_EXPECT(PN7160_MfcWrite(4U, NULL), PN7160_INVALID_PARAM);
	PN_EXPECT(PN7160_MfcAuth(0U, 0U, clave), PN7160_ERROR);
	PN_EXPECT(PN7160_MfcRead(0U, bloque), PN7160_ERROR);
	PN_EXPECT(PN7160_MfcWrite(4U, bloque), PN7160_ERROR);

	/* -- Sin descubrimiento IT armado, Process regresa de inmediato -- */
	PN_CHECK(PN7160_Process());

	/* -- Modo lector sobre modo lector y UID dinámico en modo lector: solo configuran -- */
	PN_CHECK(PN7160_SetMode(PN7160_MODE_READER));
	PN_CHECK(PN7160_SetCardUid(NULL, 0U));

	printf(COLOR_GREEN "Errores OK" COLOR_RESET "\r\n");
}

static void PN7160_TestTimeout(void)
{
	/* -- Sin tag en el campo, WaitTag agota exactamente su tiempo -- */
	printf(COLOR_YELLOW "[Timeout]" COLOR_RESET "\r\n");
	PN7160_EsperarRetirada();

	uint32_t t0 = HAL_GetTick();
	PN_EXPECT(PN7160_WaitTag(&tag, TEST_TIMEOUT_MS, 0U), PN7160_TIMEOUT);
	uint32_t dt = HAL_GetTick() - t0;
	printf("  WaitTag(%u ms) tardo %lu ms\r\n", (unsigned)TEST_TIMEOUT_MS, (unsigned long)dt);
	PN_ASSERT(dt >= TEST_TIMEOUT_MS && dt < TEST_TIMEOUT_MS + TEST_MARGEN_MS);

	/* Con keepActive = 1 también vuelve a RFST_IDLE: no queda tag para Transceive */
	PN_EXPECT(PN7160_WaitTag(&tag, TEST_TIMEOUT_MS, 1U), PN7160_TIMEOUT);
	PN_EXPECT(PN7160_WaitTagRemoval(100U), PN7160_ERROR);

	/* timeoutMs = 0: arranca y cancela el descubrimiento sin esperar */
	PN_EXPECT(PN7160_WaitTag(&tag, 0U, 0U), PN7160_TIMEOUT);

	printf(COLOR_GREEN "Timeout OK" COLOR_RESET "\r\n");
}

static void PN7160_TestLectura(void)
{
	PN7160_Tag_t primero;
	uint8_t rx[4];
	uint16_t n = sizeof(rx);

	/* -- Lectura sin keepActive: datos coherentes y vuelve a RFST_IDLE -- */
	printf(COLOR_YELLOW "[Lectura]" COLOR_RESET "\r\n");
	PN7160_EsperarTag("un tag cualquiera y dejalo apoyado", 0U);
	PN7160_CheckTag(&tag);
	primero = tag;

	PN_EXPECT(PN7160_Transceive(NULL, 0U, rx, &n), PN7160_ERROR);
	PN_EXPECT(PN7160_WaitTagRemoval(100U), PN7160_ERROR);

	/* -- Mismo tag, ahora activado: mismos datos -- */
	PN_CHECK(PN7160_WaitTag(&tag, TEST_REDETECTAR_MS, 1U));
	PN_ASSERT(PN7160_MismoTag(&tag, &primero));

	/* -- Con el tag apoyado WaitTagRemoval agota su tiempo y deja RFST_IDLE (sin tag activado) -- */
	uint32_t t0 = HAL_GetTick();
	PN_EXPECT(PN7160_WaitTagRemoval(TEST_PRESENTE_MS), PN7160_TIMEOUT);
	PN_ASSERT((HAL_GetTick() - t0) >= TEST_PRESENTE_MS);
	PN_EXPECT(PN7160_WaitTagRemoval(100U), PN7160_ERROR);

	/* -- Activado otra vez: ahora sí se retira -- */
	PN_CHECK(PN7160_WaitTag(&tag, TEST_REDETECTAR_MS, 1U));
	printf("  Retira el tag...\r\n");
	PN_CHECK(PN7160_WaitTagRemoval(TEST_TAG_MS));

	/* Retirado y en RFST_IDLE: sin tag en el campo, el siguiente WaitTag no encuentra nada */
	PN_EXPECT(PN7160_WaitTag(&tag, TEST_TIMEOUT_MS, 0U), PN7160_TIMEOUT);

	printf(COLOR_GREEN "Lectura OK" COLOR_RESET "\r\n");
}

/* T2T (Frame RF): READ 0x30 devuelve 4 páginas (16 bytes) + byte de estado NCI */
static void PN7160_ProbarT2T(void)
{
	static const uint8_t leer0[] = { 0x30U, 0x00U };   /* READ desde la página 0 */
	uint8_t pag[32];
	uint8_t rx[32];
	uint16_t n = sizeof(pag);

	PN_CHECK(PN7160_Transceive(leer0, sizeof(leer0), pag, &n));
	PN7160_ImprimirHex("Paginas 0-3:", pag, n);
	PN_ASSERT(n == 17U && pag[16] == 0x00U);

	/* Página 0 = UID0 UID1 UID2 BCC0, página 1 = UID3..UID6; BCC0 = 0x88 ^ UID0 ^ UID1 ^ UID2 */
	if (tag.uidLen == 7U)
	{
		PN_ASSERT(memcmp(&pag[0], &tag.uid[0], 3) == 0 && memcmp(&pag[4], &tag.uid[3], 4) == 0);
		PN_ASSERT(pag[3] == (uint8_t)(0x88U ^ tag.uid[0] ^ tag.uid[1] ^ tag.uid[2]));
	}

	/* -- Buffer chico: devuelve ERROR con los bytes que sí cupieron, y el tag sigue respondiendo -- */
	n = 4U;
	PN_EXPECT(PN7160_Transceive(leer0, sizeof(leer0), rx, &n), PN7160_ERROR);
	PN_ASSERT(n == 4U && memcmp(rx, pag, 4) == 0);

	n = sizeof(rx);
	PN_CHECK(PN7160_Transceive(leer0, sizeof(leer0), rx, &n));
	PN_ASSERT(n == 17U && memcmp(rx, pag, 17) == 0);
}

/* ISO-DEP: APDU completos. SELECT PPSE responde 9000 en una bancaria, otro SW en el resto */
static void PN7160_ProbarIsoDep(void)
{
	static const uint8_t ppse[] = { 0x00U, 0xA4U, 0x04U, 0x00U, 0x0EU,
	                                '2', 'P', 'A', 'Y', '.', 'S', 'Y', 'S', '.', 'D', 'D', 'F', '0', '1',
	                                0x00U };
	uint8_t rx[258];
	uint16_t n = sizeof(rx);

	PN_CHECK(PN7160_Transceive(ppse, sizeof(ppse), rx, &n));
	PN7160_ImprimirHex("R-APDU:", rx, n);
	PN_ASSERT(n >= 2U);

	/* SW1 válido según ISO 7816-4: 0x6X o 0x9X */
	uint8_t sw1 = rx[n - 2U];
	printf("  SW %02X%02X (9000 = bancaria, 6A82 = sin PPSE)\r\n", sw1, rx[n - 1U]);
	PN_ASSERT((sw1 & 0xF0U) == 0x60U || (sw1 & 0xF0U) == 0x90U);
}

/* Una clave incorrecta deja la tarjeta en HALT: se apaga el campo y se vuelve a activar */
static void PN7160_ReactivarMfc(void)
{
	PN7160_Tag_t antes = tag;

	PN_CHECK(PN7160_ReleaseTag());
	PN_CHECK(PN7160_WaitTag(&tag, TEST_REDETECTAR_MS, 1U));
	PN_ASSERT(PN7160_MismoTag(&tag, &antes));
}

/* Prueba las claves de mfcClaves en el sector (keyB: 0 = A, 1 = B) y devuelve el índice de la que
   entra. Una clave incorrecta debe dar exactamente ERROR; si ninguna entra, la prueba falla */
static uint8_t PN7160_BuscarClaveMfc(uint8_t sector, uint8_t keyB)
{
	char nombre[24];

	for (uint8_t k = 0; k < sizeof(mfcClaves) / sizeof(mfcClaves[0]); k++)
	{
		status = PN7160_MfcAuth(sector, keyB, mfcClaves[k]);
		if (status == PN7160_OK)
		{
			snprintf(nombre, sizeof(nombre), "Sector %2u, clave %c:", (unsigned)sector, keyB ? 'B' : 'A');
			PN7160_ImprimirHex(nombre, mfcClaves[k], 6U);
			return k;
		}
		PN_ASSERT(status == PN7160_ERROR);
		PN7160_ReactivarMfc();
	}

	printf("  Sector %u: ninguna clave %c conocida\r\n", (unsigned)sector, keyB ? 'B' : 'A');
	Error_Handler();
	return 0U;
}

/* MIFARE Classic con los wrappers del driver (TAG-CMD de NXP): busca las claves A y B de cada
   sector, vuelca la tarjeta y escribe TEST_MFC_BLOQUE restaurándolo después */
static void PN7160_ProbarMfc(void)
{
	static const uint8_t ceros[6] = { 0 };
	uint8_t claveA[MFC_SECTORES];
	uint8_t rx[16], orig[16], patron[16];
	char nombre[24];

	/* -- Volcado: claves B y A de cada sector, y lectura de sus 4 bloques con la A -- */
	for (uint8_t s = 0; s < MFC_SECTORES; s++)
	{
		/* La B solo se busca: si los access bits dejan leerla, autentica pero no da acceso a los
		   bloques. La A va después porque es con la que se lee */
		(void)PN7160_BuscarClaveMfc(s, 1U);
		claveA[s] = PN7160_BuscarClaveMfc(s, 0U);

		for (uint8_t b = 4U * s; b < 4U * s + 4U; b++)
		{
			PN_CHECK(PN7160_MfcRead(b, rx));
			snprintf(nombre, sizeof(nombre), "  Bloque %2u:", (unsigned)b);
			PN7160_ImprimirHex(nombre, rx, sizeof(rx));
		}

		/* La clave A nunca se puede leer: el trailer (último bloque leído) la devuelve en ceros */
		PN_ASSERT(memcmp(rx, ceros, sizeof(ceros)) == 0);

		/* El bloque 0 (fabricante) empieza por el UID; con 4 bytes le sigue el BCC (XOR del UID) */
		if (s == 0U)
		{
			PN_CHECK(PN7160_MfcRead(0U, rx));
			PN_ASSERT(memcmp(rx, tag.uid, tag.uidLen) == 0);
			if (tag.uidLen == 4U)
			{
				PN_ASSERT(rx[4] == (uint8_t)(tag.uid[0] ^ tag.uid[1] ^ tag.uid[2] ^ tag.uid[3]));
			}
		}
	}

	/* -- Escritura: patrón invertido, relectura, y se restaura el original antes de comprobar -- */
	const uint8_t s = TEST_MFC_BLOQUE / 4U;
	PN_CHECK(PN7160_MfcAuth(s, 0U, mfcClaves[claveA[s]]));
	PN_CHECK(PN7160_MfcRead(TEST_MFC_BLOQUE, orig));
	for (uint8_t i = 0; i < sizeof(patron); i++)
	{
		patron[i] = (uint8_t)~orig[i];
	}

	PN_CHECK(PN7160_MfcWrite(TEST_MFC_BLOQUE, patron));
	PN_CHECK(PN7160_MfcRead(TEST_MFC_BLOQUE, rx));
	uint8_t escrito = (memcmp(rx, patron, sizeof(rx)) == 0);

	PN_CHECK(PN7160_MfcWrite(TEST_MFC_BLOQUE, orig));
	PN_CHECK(PN7160_MfcRead(TEST_MFC_BLOQUE, rx));
	printf("  Bloque %u: escrito %s, restaurado %s\r\n", (unsigned)TEST_MFC_BLOQUE,
	       escrito ? "OK" : "MAL", (memcmp(rx, orig, sizeof(rx)) == 0) ? "OK" : "MAL");
	PN_ASSERT(escrito && memcmp(rx, orig, sizeof(rx)) == 0);
}

static void PN7160_TestTransceive(void)
{
	uint8_t faltaT2T = TEST_CON_T2T, faltaIsoDep = TEST_CON_ISODEP, faltaMfc = TEST_CON_MFC;
	uint8_t rx[4], bloque[16];
	uint16_t n;

	/* -- Un tag de cada protocolo, en cualquier orden; repetir uno no cuenta dos veces -- */
	printf(COLOR_YELLOW "[Transceive]" COLOR_RESET "\r\n");
	while (faltaT2T || faltaIsoDep || faltaMfc)
	{
		printf("  Faltan:%s%s%s\r\n", faltaT2T ? " T2T" : "", faltaIsoDep ? " ISO-DEP" : "",
		       faltaMfc ? " MIFARE-Classic" : "");
		PN7160_EsperarTag("uno de ellos", 1U);
		PN7160_CheckTag(&tag);

		/* -- Los wrappers MIFARE rechazan otro protocolo sin mandar nada por RF -- */
		if (tag.protocolo != PN7160_PROT_MIFARE)
		{
			PN_EXPECT(PN7160_MfcRead(0U, bloque), PN7160_ERROR);
		}

		switch (tag.protocolo)
		{
		case PN7160_PROT_T2T:    PN7160_ProbarT2T();    faltaT2T = 0U;    break;
		case PN7160_PROT_ISODEP: PN7160_ProbarIsoDep(); faltaIsoDep = 0U; break;
		case PN7160_PROT_MIFARE: PN7160_ProbarMfc();    faltaMfc = 0U;    break;
		default:           printf("  Protocolo sin prueba de Transceive\r\n"); break;
		}

		/* -- ReleaseTag apaga el campo: ya no hay tag para Transceive; repetirlo no hace nada -- */
		PN_CHECK(PN7160_ReleaseTag());
		n = sizeof(rx);
		PN_EXPECT(PN7160_Transceive(NULL, 0U, rx, &n), PN7160_ERROR);
		PN_CHECK(PN7160_ReleaseTag());

		PN7160_EsperarRetirada();
	}

	printf(COLOR_GREEN "Transceive OK" COLOR_RESET "\r\n");
}

static void PN7160_TestIT(void)
{
	PN7160_Tag_t primero;
	uint8_t rx[4];
	uint16_t n = sizeof(rx);

	/* -- Detección por callback, dejando el tag activado -- */
	printf(COLOR_YELLOW "[Interrupcion]" COLOR_RESET "\r\n");
	tagITListo = 0U;
	PN_CHECK(PN7160_StartDiscoveryIT(1U));
	printf("  Acerca un tag y dejalo apoyado...\r\n");
	PN_ASSERT(PN7160_ProcesarHasta(TEST_TAG_MS));
	PN7160_Imprimir(&tagIT);
	PN7160_CheckTag(&tagIT);
	primero = tagIT;

	/* Es de un solo disparo: sin rearmar, Process no vuelve a llamar al callback */
	tagITListo = 0U;
	PN_ASSERT(!PN7160_ProcesarHasta(TEST_REDETECTAR_MS));
	PN_CHECK(PN7160_ReleaseTag());

	/* -- Cancelación: con el tag apoyado, ReleaseTag desarma el descubrimiento y no llega callback -- */
	PN_CHECK(PN7160_StartDiscoveryIT(0U));
	PN_CHECK(PN7160_ReleaseTag());
	PN_ASSERT(!PN7160_ProcesarHasta(TEST_REDETECTAR_MS));

	/* -- Rearmado sin keepActive: el mismo tag se detecta otra vez y queda en RFST_IDLE -- */
	PN_CHECK(PN7160_StartDiscoveryIT(0U));
	PN_ASSERT(PN7160_ProcesarHasta(TEST_REDETECTAR_MS));
	PN_ASSERT(PN7160_MismoTag(&tagIT, &primero));
	PN_EXPECT(PN7160_Transceive(NULL, 0U, rx, &n), PN7160_ERROR);

	PN7160_EsperarRetirada();

	printf(COLOR_GREEN "Interrupcion OK" COLOR_RESET "\r\n");
}

static void PN7160_TestModoTarjeta(void)
{
	static const uint8_t uid[] = { TEST_CARD_UID };
	static const uint8_t noEncontrado[] = { 0x6AU, 0x82U };   /* SW: aplicación no encontrada */
	uint8_t capdu[64];
	uint16_t n = sizeof(capdu);

	/* -- En modo lector SetCardUid solo guarda el UID; SetMode(CARD) lo envía -- */
	printf(COLOR_YELLOW "[ModoTarjeta]" COLOR_RESET "\r\n");
	PN_CHECK(PN7160_SetCardUid(uid, sizeof(uid)));
	PN_CHECK(PN7160_SetMode(PN7160_MODE_CARD));

	printf("  Acerca el celular (NFC Tools > Leer)\r\n");
	printf("  Debe verse: ISO 14443-4, ");
	PN7160_ImprimirHex("UID", uid, sizeof(uid));
	PN_CHECK(PN7160_WaitTag(&tag, TEST_TAG_MS, 1U));
	PN7160_Imprimir(&tag);
	PN_ASSERT(tag.tecnologia == PN7160_TEC_NFC_A_ESCUCHA && tag.protocolo == PN7160_PROT_ISODEP && tag.uidLen == 0U);
	PN_ASSERT(tag.atqa[0] == 0U && tag.atqa[1] == 0U && tag.sak == 0U);

	/* -- Orden inverso: con txLen = 0 se recibe el primer C-APDU del lector -- */
	PN_CHECK(PN7160_Transceive(NULL, 0U, capdu, &n));
	PN7160_ImprimirHex("C-APDU:", capdu, n);
	PN_ASSERT(n >= 4U);   /* CLA INS P1 P2 como mínimo */

	/* La R-APDU sale y llega el siguiente C-APDU; el lector también puede irse (ERROR o TIMEOUT) */
	n = sizeof(capdu);
	status = PN7160_Transceive(noEncontrado, sizeof(noEncontrado), capdu, &n);
	printf("  Tras responder 6A82: estado %d, %u bytes\r\n", (int)status, (unsigned)n);

	PN7160_EsperarRetirada();

	/* -- Ya en modo tarjeta, SetCardUid se aplica en el acto: vuelve al UID dinámico -- */
	PN_CHECK(PN7160_SetCardUid(NULL, 0U));
	PN_CHECK(PN7160_SetMode(PN7160_MODE_READER));

	printf(COLOR_GREEN "ModoTarjeta OK" COLOR_RESET "\r\n");
}

static void PN7160_TestDeInit(void)
{
	PN7160_Info_t inf;

	/* -- Re-Init sobre un módulo inicializado: desinicializa y vuelve a arrancar -- */
	printf(COLOR_YELLOW "[DeInit]" COLOR_RESET "\r\n");
	PN_CHECK(PN7160_Iniciar());
	PN_CHECK(PN7160_GetInfo(&inf));
	PN_ASSERT(memcmp(&inf, &info, sizeof(inf)) == 0);

	/* -- DeInit: VEN en bajo y todo vuelve a NOT_INITIALIZED -- */
	PN_CHECK(PN7160_DeInit());
	PN_ASSERT(HAL_GPIO_ReadPin(PN_VEN_GPIO_Port, PN_VEN_Pin) == GPIO_PIN_RESET);
	PN7160_CheckSinInit();

	/* -- Init de nuevo: mismo chip y lector funcional (sin tag, WaitTag agota su tiempo) -- */
	PN_CHECK(PN7160_Iniciar());
	PN_CHECK(PN7160_GetInfo(&inf));
	PN_ASSERT(memcmp(&inf, &info, sizeof(inf)) == 0);
	PN_EXPECT(PN7160_WaitTag(&tag, TEST_TIMEOUT_MS, 0U), PN7160_TIMEOUT);

	printf(COLOR_GREEN "DeInit OK" COLOR_RESET "\r\n");
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
  /* Si la UART aún no está inicializada, HAL_UART_Transmit devuelve BUSY sin bloquear */
  printf(COLOR_BOLD COLOR_RED "\r\nERROR: prueba fallida" COLOR_RESET "\r\n");
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
