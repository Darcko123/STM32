/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Ecualizador de 3 Bandas con Filtros FIR
  ******************************************************************************
  * @description
  * Este programa implementa un ecualizador de audio de 3 bandas (graves, medios, 
  * agudos) utilizando filtros FIR (Finite Impulse Response) en un microcontrolador
  * STM32. El sistema procesa señales de audio en tiempo real mediante:
  * 
  * - ADC1: Captura la señal de audio de entrada a través de DMA
  * - ADC2: Lee 3 potenciómetros para controlar las ganancias de cada banda
  * - DAC: Genera la señal de audio procesada
  * - TIM8: Sincroniza el muestreo del ADC y DAC
  * - ARM CMSIS-DSP: Biblioteca optimizada para procesamiento de señales
  * 
  * El procesamiento se realiza en dos mitades del buffer (ping-pong) para
  * minimizar la latencia y evitar pérdida de muestras.
  * 
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h" // Biblioteca CMSIS-DSP para procesamiento optimizado
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ============================================================================
// CONFIGURACIÓN DE BUFFERS Y FILTROS
// ============================================================================

#define N 32                    // Tamaño total del buffer (muestras)
#define halfN N/2               // Mitad del buffer para procesamiento ping-pong
#define NUM_TAPS 32             // Número de coeficientes de cada filtro FIR

// ============================================================================
// CONFIGURACIÓN DE CONVERSIÓN ADC/DAC
// ============================================================================

// El ADC de 12 bits genera valores de 0 a 4095
// Para procesamiento de señales AC, centramos en 2047.5 (punto medio)
#define ADC_OFFSET 2047.5f      // Offset para centrar señal de entrada en cero
#define DAC_OFFSET 2047         // Offset para centrar señal de salida

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

// ============================================================================
// BUFFERS DE ENTRADA/SALIDA
// ============================================================================

// Buffer circular para captura de audio (valores ADC de 12 bits)
uint32_t adc_buffer[N];         

// Buffer circular para salida de audio (valores DAC de 12 bits)
uint32_t dac_buffer[N]; 

// ============================================================================
// BUFFERS DE PROCESAMIENTO DE SEÑALES (punto flotante para precisión)
// ============================================================================

// Buffer de entrada para filtros (señal centrada en cero)
float32_t filt_in[N];           

// Buffer de salida combinada (suma de las 3 bandas)
float32_t filt_out[N];  

// Buffers de salida individuales para cada banda de frecuencia
float32_t filt_out_low[N];      // Salida filtro pasa-bajos (graves)
float32_t filt_out_mid[N];      // Salida filtro pasa-banda (medios)
float32_t filt_out_high[N];     // Salida filtro pasa-altos (agudos)

// Punteros para acceso directo a los buffers (optimización)
float32_t *filt_in_ptr = &filt_in[0];
float32_t *filt_out_ptr = &filt_out[0];
float32_t *filt_out_low_ptr = &filt_out_low[0];
float32_t *filt_out_mid_ptr = &filt_out_mid[0];
float32_t *filt_out_high_ptr = &filt_out_high[0];

// ============================================================================
// ESTRUCTURAS DE FILTROS FIR (ARM CMSIS-DSP)
// ============================================================================

// Instancias de filtros FIR para cada banda
arm_fir_instance_f32 filterLow;   // Filtro pasa-bajos: deja pasar graves
arm_fir_instance_f32 filterBand;  // Filtro pasa-banda: deja pasar medios
arm_fir_instance_f32 filterHigh;  // Filtro pasa-altos: deja pasar agudos

// Buffers de estado para cada filtro (necesarios para FIR con memoria)
// Tamaño: halfN + NUM_TAPS - 1 (requerido por CMSIS-DSP)
float32_t StateLow[halfN + NUM_TAPS - 1];
float32_t StateMid[halfN + NUM_TAPS - 1];
float32_t StateHigh[halfN + NUM_TAPS - 1];

// ============================================================================
// COEFICIENTES DE FILTROS FIR
// ============================================================================

/**
 * Coeficientes del filtro pasa-bajos (Lowpass)
 * Permite el paso de frecuencias bajas (graves)
 * Diseñado con respuesta simétrica (fase lineal)
 */
float32_t LowCoeffs[NUM_TAPS] = {
		0.028314228399935807006793098139496578369,
		0.028677143031090316510622884038639313076,
		0.02901982347161971911853051153684646124 ,
		0.029341653042583251687069889612757833675,
		0.029642051039593286104745217812705959659,
		0.029920474095872975212051514404265617486,
		0.030176417459572918433607924271200317889,
		0.030409416181928369815601342907029902562,
		0.030619046213074270490483996809416566975,
		0.030804925402580399934615584811581356917,
		0.030966714402022359198252132728157448582,
		0.031104117467165303539955445444320503157,
		0.031216883157605550963564766675517603289,
		0.031304804931989942129444415286343428306,
		0.031367721637212304941488838494478841312,
		0.031405517890271482384090262485187849961,
		0.031418124351763429547013117826281813905,
		0.031405517890271482384090262485187849961,
		0.031367721637212304941488838494478841312,
		0.031304804931989942129444415286343428306,
		0.031216883157605550963564766675517603289,
		0.031104117467165303539955445444320503157,
		0.030966714402022359198252132728157448582,
		0.030804925402580399934615584811581356917,
		0.030619046213074270490483996809416566975,
		0.030409416181928369815601342907029902562,
		0.030176417459572918433607924271200317889,
		0.029920474095872975212051514404265617486,
		0.029642051039593286104745217812705959659,
		0.029341653042583251687069889612757833675,
		0.02901982347161971911853051153684646124 ,
		0.028677143031090316510622884038639313076
};

/**
 * Coeficientes del filtro pasa-banda (Bandpass)
 * Permite el paso de frecuencias medias
 * Atenúa tanto graves como agudos
 */
float32_t BandCoeffs[NUM_TAPS] = {
		 0                                        ,
		-0.000053366635373974343945456533244708908,
		-0.000009197839776747510521105050118606528,
		 0.000353535997103919161742291432659612838,
		 0.000805629039306712704630875077072005297,
		 0.000328211799665740003306702021745877573,
		-0.002737517146227729653018556632559921127,
		-0.009669687214123487764561026835963275516,
		-0.019644837546352478946376507451532233972,
		-0.028312875036357761787675357822990918066,
		-0.028081891735227539758090742338936252054,
		-0.01091458299741123079418247954208709416 ,
		 0.027132120920916418665980174296237237286,
		 0.081977606714179376834472634527628542855,
		 0.140902208399525441517852186734671704471,
		 0.186476132730579891028099837058107368648,
		 0.203646383744244263702682928851572796702,
		 0.186476132730579891028099837058107368648,
		 0.140902208399525441517852186734671704471,
		 0.081977606714179376834472634527628542855,
		 0.027132120920916418665980174296237237286,
		-0.01091458299741123079418247954208709416 ,
		-0.028081891735227539758090742338936252054,
		-0.028312875036357761787675357822990918066,
		-0.019644837546352478946376507451532233972,
		-0.009669687214123487764561026835963275516,
		-0.002737517146227729653018556632559921127,
		 0.000328211799665740003306702021745877573,
		 0.000805629039306712704630875077072005297,
		 0.000353535997103919161742291432659612838,
		-0.000009197839776747510521105050118606528,
		-0.000053366635373974343945456533244708908
};

/**
 * Coeficientes del filtro pasa-altos (Highpass)
 * Permite el paso de frecuencias altas (agudos)
 * Atenúa frecuencias bajas
 */
float32_t HighCoeffs[NUM_TAPS] = {
		 0.001511545109837736334743030219840420614,
		 0.001882591379028836610712294152847334772,
		 0.002483585545241961348855719293737820408,
		 0.003116059534362420647968239251213162788,
		 0.003343310614823753919300219905608173576,
		 0.002539929188056473403217738749049203761,
		-0.000000000000000015585067345520673885766,
		-0.004914128527513412537930381773776389309,
		-0.012611514421013634429380445567403512541,
		-0.023134852490067567276632587436324683949,
		-0.036077275462792365856579124283598503098,
		-0.050576523872093027967000722355805919506,
		-0.065394665302038482157520604687306331471,
		-0.079074464358267398944590809151122812182,
		-0.090147533401616602999339988855354022235,
		-0.097357957089300478648041803353407885879,
		 0.898746402163526969530948917963542044163,
		-0.097357957089300478648041803353407885879,
		-0.090147533401616602999339988855354022235,
		-0.079074464358267398944590809151122812182,
		-0.065394665302038482157520604687306331471,
		-0.050576523872093027967000722355805919506,
		-0.036077275462792365856579124283598503098,
		-0.023134852490067567276632587436324683949,
		-0.012611514421013634429380445567403512541,
		-0.004914128527513412537930381773776389309,
		-0.000000000000000015585067345520673885766,
		 0.002539929188056473403217738749049203761,
		 0.003343310614823753919300219905608173576,
		 0.003116059534362420647968239251213162788,
		 0.002483585545241961348855719293737820408,
		 0.001882591379028836610712294152847334772
};

// ============================================================================
// VARIABLES DE CONTROL DE GANANCIA
// ============================================================================

// Ganancias lineales aplicadas a cada banda (multiplicadores)
float32_t gainLow;    // Ganancia para graves (0.1 a 10.0 aprox.)
float32_t gainMid;    // Ganancia para medios (0.1 a 10.0 aprox.)
float32_t gainHigh;   // Ganancia para agudos (0.1 a 10.0 aprox.)

// Valores de ganancia en decibelios leídos de los potenciómetros
float32_t db_low;     // Ganancia en dB para graves (-20 a +20 dB)
float32_t db_mid;     // Ganancia en dB para medios (-20 a +20 dB)
float32_t db_high;    // Ganancia en dB para agudos (-20 a +20 dB)

// ============================================================================
// BUFFER DE CONTROL (POTENCIÓMETROS)
// ============================================================================

// Buffer para los 3 canales del ADC2 (potenciómetros de control)
// [0]: Potenciómetro graves
// [1]: Potenciómetro medios
// [2]: Potenciómetro agudos
uint32_t ADC_VAL[3];

// Bandera para indicar que hay nuevos valores de los potenciómetros
volatile uint8_t adc2_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/**
 * @brief Mapea un valor de un rango a otro linealmente.
 * 
 * @param x Valor a mapear
 * @param in_min Valor mínimo del rango de entrada
 * @param in_max Valor máximo del rango de entrada
 * @param out_min Valor mínimo del rango de salida
 * @param out_max Valor máximo del rango de salida
 * @return float32_t Valor mapeado al rango de salida
 * 
 * @details
 * Esta función realiza una interpolación lineal para convertir un valor
 * de un rango a otro. Es útil para convertir lecturas del ADC (0-4095)
 * a valores de ganancia en decibelios (-20 a +20 dB).
 * 
 * Fórmula: y = m * x + b
 * donde: m = (out_max - out_min) / (in_max - in_min)
 *        b = out_min - m * in_min
 * 
 * @example
 * // Convertir lectura ADC a dB
 * float db = map_float(2048, 0, 4095, -20.0f, 20.0f);  // Resultado: ~0 dB
 */
float32_t map_float(uint32_t x, uint32_t in_min, uint32_t in_max, float32_t out_min, float32_t out_max);

/**
 * @brief Convierte una ganancia en decibelios a ganancia lineal.
 * 
 * @param db Ganancia en decibelios
 * @return float32_t Ganancia lineal (multiplicador)
 * 
 * @details
 * Convierte un valor de ganancia expresado en decibelios (dB) a su
 * equivalente lineal que puede ser aplicado como multiplicador.
 * 
 * Fórmula: ganancia_lineal = 10^(dB/20)
 * 
 * Ejemplos de conversión:
 * - 0 dB   → 1.0   (sin cambio)
 * - +6 dB  → 2.0   (doble amplitud)
 * - -6 dB  → 0.5   (mitad de amplitud)
 * - +20 dB → 10.0  (diez veces la amplitud)
 * - -20 dB → 0.1   (décima parte de la amplitud)
 */
float32_t gain_db_to_linear(float32_t db);

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
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  // ============================================================================
  // INICIO DE PERIFÉRICOS CON DMA
  // ============================================================================
  
  // Inicia ADC1 para capturar audio continuamente
  // - Modo circular: cuando llena el buffer, vuelve al inicio
  // - Genera interrupciones al completar la mitad y el buffer completo
  HAL_ADC_Start_DMA(&hadc1, adc_buffer, N);

  // Inicia ADC2 para leer los 3 potenciómetros
  // - Modo continuo: actualiza constantemente los valores
  HAL_ADC_Start_DMA(&hadc2, ADC_VAL, 3);

  // Inicia DAC para generar la señal de audio procesada
  // - Sincronizado con TIM8 para mantener la frecuencia de muestreo
  // - Alineación a la derecha de 12 bits
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, dac_buffer, N, DAC_ALIGN_12B_R);

  // Inicia el timer que sincroniza ADC y DAC
  HAL_TIM_Base_Start(&htim8);

  // ============================================================================
  // INICIALIZACIÓN DE FILTROS FIR
  // ============================================================================
  
  // Inicializa filtro pasa-bajos (graves)
  // Parámetros: instancia, num_taps, coeficientes, buffer_estado, tamaño_bloque
  arm_fir_init_f32(&filterLow, NUM_TAPS, &LowCoeffs[0], &StateLow[0], halfN);
  
  // Inicializa filtro pasa-banda (medios)
  arm_fir_init_f32(&filterBand, NUM_TAPS, &BandCoeffs[0], &StateMid[0], halfN);
  
  // Inicializa filtro pasa-altos (agudos)
  arm_fir_init_f32(&filterHigh, NUM_TAPS, &HighCoeffs[0], &StateHigh[0], halfN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // ========================================================================
	  // ACTUALIZACIÓN DE GANANCIAS
	  // ========================================================================

    // Verifica si hay nuevos valores de los potenciómetros
	  if(adc2_ready)
	  {
		  adc2_ready = 0;  // Limpia la bandera

		  // Convierte las lecturas ADC (0-4095) a ganancia en dB (-20 a +20)
		  // Rango conservador para evitar saturación excesiva
		  db_low = map_float(ADC_VAL[0], 0, 4095, -20.0f, 20.0f);
		  db_mid = map_float(ADC_VAL[1], 0, 4095, -20.0f, 20.0f);
		  db_high = map_float(ADC_VAL[2], 0, 4095, -20.0f, 20.0f);

		  // Convierte los valores en dB a ganancia lineal
		  // Estas variables serán leídas en las interrupciones del ADC
		  gainLow = gain_db_to_linear(db_low);
		  gainMid = gain_db_to_linear(db_mid);
		  gainHigh = gain_db_to_linear(db_high);
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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T8_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 450;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ============================================================================
// CALLBACKS DE INTERRUPCIONES DMA
// ============================================================================

/**
  * @brief Callback cuando se completa la primera mitad del buffer ADC
  * @details Esta función se ejecuta automáticamente cuando el DMA llena
  *          la primera mitad del buffer de captura. Implementa el esquema
  *          de "ping-pong" o doble buffer para procesamiento en tiempo real.
  * 
  * FLUJO DE PROCESAMIENTO:
  * 1. Conversión ADC → float y centrado en cero
  * 2. Aplicación de filtros FIR a cada banda
  * 3. Aplicación de ganancias individuales
  * 4. Suma de las 3 bandas
  * 5. Restauración de offset y limitación
  * 6. Conversión a valores DAC
  * 
  * @param hadc Puntero al handle del ADC que generó la interrupción
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{

    // ====================================================================
		// PASO 1: CONVERSIÓN Y CENTRADO DE LA SEÑAL
		// ====================================================================

    // Procesa la primera mitad del buffer (0 a halfN-1)
		// Convierte de uint32_t (0-4095) a float32_t centrado en cero
		// Ejemplo: 2047 → -0.5, 2048 → 0.5, 4095 → 2047.5
		for(int n = 0; n < halfN; n++)
		{
			filt_in[n] = (float32_t)adc_buffer[n] - ADC_OFFSET;
		}

    // ====================================================================


    // ====================================================================
		// PASO 2: APLICACIÓN DE FILTROS FIR
		// ====================================================================

		// Filtra graves (pasa-bajos): deja pasar frecuencias bajas
		arm_fir_f32(&filterLow, filt_in_ptr, filt_out_low_ptr, halfN);
		
		// Filtra medios (pasa-banda): deja pasar frecuencias medias
		arm_fir_f32(&filterBand, filt_in_ptr, filt_out_mid_ptr, halfN);
		
		// Filtra agudos (pasa-altos): deja pasar frecuencias altas
		arm_fir_f32(&filterHigh, filt_in_ptr, filt_out_high_ptr, halfN);

    // ====================================================================


    // ====================================================================
		// PASO 3: COPIA LOCAL DE GANANCIAS (evita race conditions)
		// ====================================================================
		
    // Las ganancias pueden ser modificadas por el main loop
		// Copiamos a variables locales para garantizar consistencia
		float32_t local_gainLow = gainLow;
		float32_t local_gainMid = gainMid;
		float32_t local_gainHigh = gainHigh;

    // ====================================================================


    // ====================================================================
		// PASO 4: APLICACIÓN DE GANANCIAS
		// ====================================================================

    // Multiplica cada banda por su ganancia correspondiente
		// arm_scale_f32 es una función optimizada de CMSIS-DSP
		arm_scale_f32(filt_out_low_ptr, local_gainLow, filt_out_low_ptr, halfN);
		arm_scale_f32(filt_out_mid_ptr, local_gainMid, filt_out_mid_ptr, halfN);
		arm_scale_f32(filt_out_high_ptr, local_gainHigh, filt_out_high_ptr, halfN);

    // ====================================================================


    // ====================================================================
		// PASO 5: SUMA DE BANDAS Y CONVERSIÓN A DAC
		// ====================================================================
		for(int n = 0; n<halfN; n++)
		{
      // Suma las 3 bandas filtradas y escaladas
			filt_out[n] = filt_out_low[n] + filt_out_mid[n] + filt_out_high[n];

			// Restaura el offset DC para el DAC (centrar en 2047.5)
			float32_t dac_val = filt_out[n] + ADC_OFFSET;

      // Limita el valor para prevenir overflow del DAC
			// El DAC solo acepta valores de 0 a 4095 (12 bits)
			if(dac_val > 4095.0f) dac_val = 4095.0f;
			if(dac_val < 0.0f) dac_val = 0.0f;

      // Convierte a entero para el DAC
			dac_buffer[n] = (uint32_t)dac_val;
		}
	}
	else if(hadc->Instance == ADC2)
	{
    // ADC2 completó lectura de potenciómetros
		// Señaliza al main loop que hay nuevos valores disponibles
		adc2_ready = 1;
	}
}

/**
  * @brief Callback cuando se completa el buffer completo del ADC
  * @details Similar a HAL_ADC_ConvHalfCpltCallback pero procesa la
  *          segunda mitad del buffer (halfN a N-1).
  * 
  * Este callback se ejecuta cuando el DMA termina de llenar todo el buffer
  * y vuelve al inicio (modo circular). Mientras se procesa esta mitad,
  * el DMA está llenando la primera mitad, implementando el esquema ping-pong.
  * 
  * @param hadc Puntero al handle del ADC que generó la interrupción
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{
		
    // ====================================================================
		// PASO 1: CONVERSIÓN Y CENTRADO DE LA SEÑAL
		// ====================================================================
		
		// Procesa la segunda mitad del buffer (halfN a N-1)
		for(int n = halfN; n<N; n++)
		{
			filt_in[n] = (float32_t)adc_buffer[n] - ADC_OFFSET;
		}

    // ====================================================================


    // ====================================================================
		// PASO 2: APLICACIÓN DE FILTROS FIR
		// ====================================================================
		
		// Nota: Se suman offsets a los punteros para procesar la segunda mitad
		arm_fir_f32(&filterLow, filt_in_ptr + halfN, filt_out_low_ptr + halfN, halfN);
		arm_fir_f32(&filterBand, filt_in_ptr + halfN, filt_out_mid_ptr + halfN, halfN);
		arm_fir_f32(&filterHigh, filt_in_ptr + halfN, filt_out_high_ptr + halfN, halfN);

    // ====================================================================


    // ====================================================================
		// PASO 3: COPIA LOCAL DE GANANCIAS
		// ====================================================================

		float32_t local_gainLow = gainLow;
		float32_t local_gainMid = gainMid;
		float32_t local_gainHigh = gainHigh;

    // ====================================================================


    // ====================================================================
		// PASO 4: APLICACIÓN DE GANANCIAS
		// ====================================================================

		arm_scale_f32(filt_out_low_ptr + halfN, local_gainLow, filt_out_low_ptr + halfN, halfN);
		arm_scale_f32(filt_out_mid_ptr + halfN, local_gainMid, filt_out_mid_ptr + halfN, halfN);
		arm_scale_f32(filt_out_high_ptr + halfN, local_gainHigh, filt_out_high_ptr + halfN, halfN);

    // ====================================================================


    // ====================================================================
		// PASO 5: SUMA DE BANDAS Y CONVERSIÓN A DAC
		// ====================================================================

		for(int n = halfN; n<N; n++)
		{
			filt_out[n] = filt_out_low[n] + filt_out_mid[n] + filt_out_high[n];

			// Restaurar offset y limitar
			float32_t dac_val = filt_out[n] + ADC_OFFSET;

			if(dac_val > 4095.0f) dac_val = 4095.0f;
			if(dac_val < 0.0f) dac_val = 0.0f;

			dac_buffer[n] = (uint32_t)dac_val;
		}
	}
	else if(hadc->Instance == ADC2)
	{
    // ADC2 completó lectura de potenciómetros
		adc2_ready = 1;
	}
}

// ============================================================================
// FUNCIONES AUXILIARES
// ============================================================================

/**
 * @brief Mapea un valor de un rango a otro linealmente.
 * 
 * @param x Valor a mapear
 * @param in_min Valor mínimo del rango de entrada
 * @param in_max Valor máximo del rango de entrada
 * @param out_min Valor mínimo del rango de salida
 * @param out_max Valor máximo del rango de salida
 * @return float32_t Valor mapeado al rango de salida
 * 
 * @details
 * Esta función implementa una transformación lineal (interpolación) que
 * convierte un valor de un rango de entrada a un rango de salida.
 * 
 * La transformación se calcula mediante la ecuación de una recta:
 *   y = m * x + b
 * 
 * donde:
 *   m = (out_max - out_min) / (in_max - in_min)  [pendiente]
 *   b = out_min - m * in_min                      [intersección]
 * 
 * Reorganizando para optimizar el cálculo:
 *   y = ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min
 * 
 * @example
 * // Convertir el centro del rango ADC (2048) a 0 dB
 * float db = map_float(2048, 0, 4095, -20.0f, 20.0f);  
 * // Resultado: db ≈ 0.0
 * 
 * // Convertir el máximo ADC (4095) a +20 dB
 * float db_max = map_float(4095, 0, 4095, -20.0f, 20.0f);  
 * // Resultado: db_max = 20.0
 * 
 * // Convertir el mínimo ADC (0) a -20 dB
 * float db_min = map_float(0, 0, 4095, -20.0f, 20.0f);  
 * // Resultado: db_min = -20.0
 */
float32_t map_float(uint32_t x, uint32_t in_min, uint32_t in_max, float32_t out_min, float32_t out_max)
{
	return ((float32_t)(x - in_min) * (out_max - out_min)) / ((float32_t)(in_max - in_min)) + out_min;
}

/**
 * @brief Convierte una ganancia en decibelios a ganancia lineal.
 * 
 * @param db Ganancia en decibelios (puede ser negativa, positiva o cero)
 * @return float32_t Ganancia lineal (siempre positiva)
 * 
 * @details
 * Los decibelios (dB) son una escala logarítmica para expresar ganancias
 * o atenuaciones. Para aplicar una ganancia a una señal, necesitamos
 * convertirla a escala lineal (multiplicador).
 * 
 * Fórmula de conversión:
 *   ganancia_lineal = 10^(dB/20)
 * 
 * La razón del divisor 20 (no 10) es porque trabajamos con amplitud,
 * no con potencia. La relación es:
 *   - Para potencia: dB = 10 * log10(P2/P1)
 *   - Para amplitud: dB = 20 * log10(A2/A1)
 * 
 * Tabla de conversión de referencia:
 * ┌──────────┬─────────────┬────────────────────────┐
 * │    dB    │   Lineal    │      Descripción       │
 * ├──────────┼─────────────┼────────────────────────┤
 * │  -20 dB  │    0.100    │ Reduce a 1/10          │
 * │  -12 dB  │    0.251    │ Reduce a ~1/4          │
 * │   -6 dB  │    0.501    │ Reduce a la mitad      │
 * │   -3 dB  │    0.708    │ Reduce ~29%            │
 * │    0 dB  │    1.000    │ Sin cambio (unidad)    │
 * │   +3 dB  │    1.413    │ Aumenta ~41%           │
 * │   +6 dB  │    1.995    │ Duplica (x2)           │
 * │  +12 dB  │    3.981    │ Cuadruplica (x4)       │
 * │  +20 dB  │   10.000    │ Multiplica por 10      │
 * └──────────┴─────────────┴────────────────────────┘
 * 
 * @note La función powf() puede ser costosa computacionalmente.
 *       Para optimización, considerar usar tablas de lookup (LUT)
 *       si se necesita procesar en tiempo crítico.
 * 
 * @example
 * float gain = gain_db_to_linear(6.0f);   // gain = 1.995 (casi x2)
 * float signal_out = signal_in * gain;     // Duplica la amplitud
 * 
 * float attenuation = gain_db_to_linear(-6.0f);  // attenuation = 0.501
 * float signal_out = signal_in * attenuation;     // Reduce a la mitad
 */
float32_t gain_db_to_linear(float32_t db) {
    return powf(10.0f, db / 20.0f);
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
