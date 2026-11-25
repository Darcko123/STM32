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
#include "SSD1306.h"
#include "MORSE.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { MENU, APRENDER, IDENTIFICAR, GRUPOS, LIBRE, ESTADISTICAS } Modo;
typedef enum { SONANDO, RESPUESTA, PAUSA } Paso;
typedef enum
{
  OPC_APRENDER, OPC_IDENTIFICAR, OPC_GRUPOS, OPC_REPASO, OPC_LIBRE, OPC_ESTADISTICAS, OPC_NIVEL, OPC_WPM, OPC_FARNSWORTH,
  OPC_TONO, OPC_PATRON, OPCIONES
} Opcion;

typedef struct
{
  uint8_t estable;   /* 1 = pulsado */
  uint8_t cuenta;    /* Muestras seguidas distintas de 'estable' */
} Entrada;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_PORT        GPIOB
#define KEY_PIN         GPIO_PIN_1      /* Manipulador a 3V3 con pull-down, activo en alto */
#define MENU_PORT       GPIOA
#define MENU_PIN        GPIO_PIN_0      /* Botón KEY de la Black Pill, a GND con pull-up, activo en bajo */
#define LED_PORT        GPIOC
#define LED_PIN         GPIO_PIN_13     /* LED de la Black Pill, activo en bajo */
#define ANTIRREBOTE_MS  5U              /* Muestras iguales seguidas para aceptar un cambio */
#define PRUEBA_TONO_MS  300U            /* Bip de arranque para comprobar la frecuencia */
/* Ajustables desde el menú; estos son los valores al encender. Cada letra manipulada
   sobre la opción avanza un paso y al pasar del máximo vuelve al mínimo. */
#define WPM_CARACTER    18U             /* Velocidad de los símbolos: 12..30, de 2 en 2 */
#define WPM_EFECTIVO    8U              /* Farnsworth en el nivel 2 (llega a la de carácter en
                                           el 36): 4..carácter, de 2 en 2 */
#define TONO_HZ         700U            /* 400..1000 Hz, de 100 en 100 */
#define BIP_MS          60U             /* Bip de acierto, al doble de la frecuencia del tono */
#define PAUSA_MS        800U            /* Silencio antes del siguiente carácter */
#define LATENCIA_MAX_MS 2000U           /* Más hasta la primera pulsación = no reconocido */
#define ESPERA_MAX_MS   6000U           /* Sin pulsar en este tiempo = sin respuesta */
#define SESION_MS       (15UL * 60UL * 1000UL)  /* Duración recomendada de la sesión */
#define AVISO_MS        500U            /* Bip largo de fin de sesión */
#define GRUPO_LARGO     5U              /* Caracteres por grupo (práctica clásica de Koch) */
#define FILAS_MENU      6U              /* Opciones visibles; el resto se desplaza */
#define LIBRE_COLUMNAS  18U             /* Font_7x10 en 128 px */
#define LIBRE_FILAS     4U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* -- Avanzados en la ISR de TIM2; el bucle principal solo los toca con la IRQ parada -- */
static MORSE_Reproductor_t   reproductor;
static MORSE_Decodificador_t decodificador;
static volatile char       recibido = 0;    /* Último carácter decodificado; 0 = ya leído */
static volatile uint8_t    menu_pulsado = 0;  /* Se pulsó el botón de menú; 0 = ya leído */
static volatile uint16_t   bip_ms = 0;      /* Lo que queda del bip de acierto */
static volatile uint32_t   pulsado_en = 0;  /* HAL_GetTick() de la primera pulsación; 0 = ninguna */
static volatile uint16_t   tono_arr = (uint16_t)(1000000UL / TONO_HZ - 1U);  /* ARR del tono */

/* -- Solo bucle principal -- */
static MORSE_Tiempos_t tiempos;
static MORSE_KochProgreso_t koch;
static uint8_t  wpm_caracter = WPM_CARACTER;
static uint8_t  wpm_efectivo = WPM_EFECTIVO;  /* Nunca mayor que wpm_caracter */
static uint16_t tono_hz      = TONO_HZ;
static Modo     modo = MENU;
static Opcion   opcion = OPC_APRENDER;      /* Resaltada en el menú */
static uint8_t  mostrar_patron = 1;         /* Patrón al presentar y al fallar */
static Paso     paso;
static Paso     tras_sonar;                 /* RESPUESTA o PAUSA al acabar de sonar */
static uint32_t pausa_desde;
static char     actual;                     /* Carácter que se espera como respuesta */
static uint8_t  siguiente;                  /* Aprender: índice en MORSE_KochOrden del próximo */
static uint8_t  presentar;                  /* Identificar: presentar el recién desbloqueado */
static uint8_t  repaso;                     /* Identificar solo entre fallados y confundidos */
static char     repasar[MORSE_SESION_REPASO_MAX + 1U];  /* Repaso: caracteres de la última elección */
static char     sonido[GRUPO_LARGO + 1U];   /* Lo que suena; debe vivir hasta que acabe.
                                               En Grupos es también el grupo que se espera */
static char     respuesta[GRUPO_LARGO + 1U];  /* Grupos: lo manipulado hasta ahora */
static uint8_t  n_respuesta;
static uint32_t ultima_en;                  /* Grupos: última letra (o fin de la reproducción) */
static char     libre[LIBRE_COLUMNAS * LIBRE_FILAS + 1U];  /* Libre: texto decodificado */
static char     tu[12];                     /* "Tu: " + patrón manipulado, o "N/5 bien" */
static uint32_t escucha_desde;              /* Fin de la reproducción: origen de la latencia */
static MORSE_Sesion_t   sesion;
static uint32_t sesion_desde;               /* Primera lección; 0 = sin empezar */
static uint8_t  sesion_avisada;
static uint8_t  fin_sesion;                 /* Estadísticas abiertas por el temporizador */
/* -- Pantalla -- */
static char        grande;                  /* Carácter en grande; 0 = ninguno */
static uint8_t     ver_patron;
static const char *estado = "";             /* Línea inferior */
static const char *aviso  = NULL;           /* Indicación de ritmo de la última respuesta */
static uint8_t     redibujar = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void Tono(uint16_t arr);
static uint8_t Antirrebote(Entrada *e, uint8_t crudo);
static void AjustarVelocidad(void);
static void Reiniciar(const char *texto);
static void Sonar(Paso despues);
static void Escuchar(void);
static void Pausa(void);
static void Preguntar(void);
static void Responder(char c);
static void Copiar(char c);
static void Anotar(char c);
static void ElegirOpcion(void);
static void DibujarEstadisticas(void);
static void Dibujar(void);

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  /* -- PWM siempre en marcha; el tono se abre y cierra con el CCR -- */
  Tono(0);
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (SSD1306_Init(&hi2c1, 128, 64) != SSD1306_OK)
  {
    Error_Handler();
  }

  /* -- Bip de arranque: medir aquí la frecuencia (700 Hz esperados) -- */
  Tono(tono_arr);
  HAL_Delay(PRUEBA_TONO_MS);
  Tono(0);

  /* -- Se inicializan antes del tick, así no hay carrera con la ISR --
     Se ignora el estado: solo falla con punteros NULL y estos son estáticos */
  (void)MORSE_KochInit(&koch, MORSE_KOCH_MINIMO);
  AjustarVelocidad();
  (void)MORSE_DecodInit(&decodificador, tiempos);
  (void)MORSE_Reproducir(&reproductor, "", tiempos);
  (void)MORSE_SesionInit(&sesion);

  /* -- Tick de 1 ms: antirrebote, tono, reproductor y decodificador -- */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* -- Entre dos caracteres hay ≥ 2 puntos de silencio: sobra tiempo para leer -- */
    char c = recibido;
    if (c != 0)
    {
      recibido = 0;
    }
    uint8_t boton = menu_pulsado;
    if (boton)
    {
      menu_pulsado = 0;
    }

    if (modo == MENU)
    {
      if (boton)                            /* Botón: siguiente opción */
      {
        opcion = (Opcion)((opcion + 1U) % OPCIONES);
        redibujar = 1;
      }
      else if (c != 0 && c != ' ')          /* Cualquier letra manipulada: elegir */
      {
        ElegirOpcion();
      }
    }
    else if (boton)                         /* Botón fuera del menú: volver */
    {
      modo       = MENU;
      fin_sesion = 0;
      Reiniciar("");                        /* Corta lo que esté sonando */
      redibujar = 1;
    }
    else if (modo == ESTADISTICAS)
    {
      /* Pantalla fija: solo se sale con el botón */
    }
    else if (modo == LIBRE)
    {
      if (c != 0)
      {
        Anotar(c);
      }
    }
    else if (paso == SONANDO)
    {
      if (!MORSE_Reproduciendo(&reproductor))
      {
        if (tras_sonar == PAUSA)
        {
          Pausa();
        }
        else
        {
          Escuchar();
        }
      }
    }
    else if (paso == RESPUESTA)
    {
      if (modo == GRUPOS)
      {
        Copiar(c);
      }
      else if (c != 0 && c != ' ')
      {
        Responder(c);
      }
      else if (modo == IDENTIFICAR && pulsado_en == 0U &&
               HAL_GetTick() - escucha_desde >= ESPERA_MAX_MS)
      {
        Responder(0);                       /* Sin respuesta */
      }
    }
    else if (HAL_GetTick() - pausa_desde >= PAUSA_MS)
    {
      Preguntar();
    }

    /* -- Solo se redibuja al cambiar: UpdateScreen bloquea ~25 ms -- */
    if (redibujar)
    {
      redibujar = 0;
      Dibujar();
    }
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1427;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 714;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Tono del buzzer: arr = 0 lo silencia; si no, suena a f = 1 MHz / (arr + 1)
 *        con ciclo del 50 %. ARR y CCR tienen precarga, así que el cambio entra al
 *        acabar el periodo en curso, sin cortes. Seguro desde la ISR.
 *        La frecuencia del tono es tono_arr (ajustable en el menú): ARR = 1 000 000 / f - 1.
 *        htim3.Init.Period (.ioc) es solo la configuración inicial del timer.
 */
static void Tono(uint16_t arr)
{
  if (arr != 0U)
  {
    TIM3->ARR = arr;
  }
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (arr + 1U) / 2U);
}

/**
 * @brief Antirrebote: un cambio se acepta tras ANTIRREBOTE_MS muestras seguidas
 *        distintas del estado estable. El retardo es igual al pulsar y al soltar,
 *        así que las duraciones que ve el decodificador no se ven afectadas.
 * @return El estado estable (1 = pulsado).
 */
static uint8_t Antirrebote(Entrada *e, uint8_t crudo)
{
  if (crudo == e->estable)
  {
    e->cuenta = 0;
  }
  else if (++e->cuenta >= ANTIRREBOTE_MS)
  {
    e->cuenta  = 0;
    e->estable = crudo;
  }
  return e->estable;
}

/**
 * @brief ISR de 1 ms (TIM2): antirrebote del manipulador y del botón de menú,
 *        reproductor, bip de acierto y decodificador.
 *        Suena el tono local del manipulador o el del reproductor (el LED lo sigue);
 *        el bip solo se oye si no suena ninguno de los dos.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static Entrada manipulador = {0};
  static Entrada boton       = {0};

  if (htim->Instance != TIM2)
  {
    return;
  }

  uint8_t antes   = manipulador.estable;
  uint8_t pulsado = Antirrebote(&manipulador, HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_SET);
  if (pulsado && !antes && pulsado_en == 0U)
  {
    pulsado_en = HAL_GetTick();             /* Retrasado ANTIRREBOTE_MS: despreciable */
  }
  antes = boton.estable;
  if (Antirrebote(&boton, HAL_GPIO_ReadPin(MENU_PORT, MENU_PIN) == GPIO_PIN_RESET) && !antes)
  {
    menu_pulsado = 1;
  }

  uint8_t morse = MORSE_ReprodTick(&reproductor) | pulsado;
  uint8_t bip   = 0;
  if (bip_ms != 0U)
  {
    bip_ms--;
    bip = 1;
  }
  uint16_t arr = tono_arr;
  Tono(morse ? arr : bip ? (uint16_t)(arr / 2U) : 0U);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, morse ? GPIO_PIN_RESET : GPIO_PIN_SET);

  char c = MORSE_DecodTick(&decodificador, pulsado);
  if (c != 0)
  {
    recibido = c;
  }
}

/* ============================ Lecciones ============================ */

/**
 * @brief Farnsworth decreciente: los silencios entre caracteres se acortan con el
 *        nivel hasta igualar la velocidad de carácter. Lo que esté sonando sigue
 *        con los tiempos anteriores (el reproductor guarda su copia).
 */
static void AjustarVelocidad(void)
{
  tiempos = MORSE_CalcularTiempos(wpm_caracter,
                                  MORSE_KochWpmEfectivo(koch.nivel, wpm_efectivo, wpm_caracter));
}

/**
 * @brief Empieza a reproducir un texto ("" = silencio) y vacía el decodificador:
 *        lo manipulado antes no cuenta como respuesta. El punto que el decodificador
 *        ha aprendido del usuario se conserva.
 */
static void Reiniciar(const char *texto)
{
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  (void)MORSE_Reproducir(&reproductor, texto, tiempos);   /* texto nunca es NULL aquí */
  (void)MORSE_DecodVaciar(&decodificador);
  recibido   = 0;
  pulsado_en = 0;
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/** @brief Reproduce 'sonido'; al acabar pasa a 'despues' (RESPUESTA o PAUSA). */
static void Sonar(Paso despues)
{
  Reiniciar(sonido);
  paso       = SONANDO;
  tras_sonar = despues;
  redibujar  = 1;
}

/** @brief Espera la respuesta. El patrón se oculta: se responde de oído. */
static void Escuchar(void)
{
  escucha_desde = HAL_GetTick();          /* Antes de Reiniciar: pulsado_en >= escucha_desde */
  ultima_en     = escucha_desde;
  Reiniciar("");
  paso       = RESPUESTA;
  ver_patron = 0;
  estado     = "Manipula";
  redibujar  = 1;
}

static void Pausa(void)
{
  paso        = PAUSA;
  pausa_desde = HAL_GetTick();
}

/**
 * @brief Presenta (×3, con patrón) o pregunta (×1, sin mostrarlo) el siguiente carácter.
 *        Cumplido el tiempo de sesión, avisa una vez y muestra las estadísticas.
 */
static void Preguntar(void)
{
  if (!sesion_avisada && HAL_GetTick() - sesion_desde >= SESION_MS)
  {
    sesion_avisada = 1;
    fin_sesion     = 1;
    bip_ms         = AVISO_MS;
    modo           = ESTADISTICAS;
    redibujar      = 1;
    return;
  }

  aviso = NULL;
  if (modo == APRENDER || presentar)
  {
    actual = MORSE_KochOrden[(modo == APRENDER) ? siguiente : koch.nivel - 1U];
    memset(sonido, actual, 3);
    sonido[3]  = '\0';
    grande     = actual;
    ver_patron = mostrar_patron;
    estado     = presentar ? "Nuevo caracter" : "Escucha";
    Sonar(presentar ? PAUSA : RESPUESTA);
    presentar = 0;
    return;
  }

  if (modo == GRUPOS)                     /* Letras seguidas, con el espaciado Farnsworth */
  {
    for (uint8_t i = 0; i < GRUPO_LARGO; i++)
    {
      sonido[i] = MORSE_SesionElegir(&sesion, koch.nivel, NULL, (uint32_t)rand());
    }
    sonido[GRUPO_LARGO] = '\0';
    memset(respuesta, 0, sizeof respuesta);
    n_respuesta = 0;
  }
  else
  {
    if (repaso)                           /* Se recalcula con cada respuesta */
    {
      MORSE_SesionRepaso(&sesion, koch.nivel, repasar);
    }
    actual    = MORSE_SesionElegir(&sesion, koch.nivel, repaso ? repasar : NULL, (uint32_t)rand());
    sonido[0] = actual;
    sonido[1] = '\0';
  }
  grande     = 0;
  ver_patron = 0;
  estado     = "Escucha";
  Sonar(RESPUESTA);
}

/**
 * @brief Evalúa la respuesta (c = 0: sin respuesta).
 *        Reconocido (correcto y, en Identificar, primera pulsación antes de
 *        LATENCIA_MAX_MS): bip y siguiente. Correcto pero lento: cuenta como fallo,
 *        sin bip. Fallo: se muestra el correcto y suenan el correcto y lo manipulado;
 *        en Aprender se repite, en Identificar se sigue.
 */
static void Responder(char c)
{
  uint32_t pulsado  = pulsado_en;
  uint32_t espera   = pulsado - escucha_desde;
  uint16_t latencia = (pulsado == 0U) ? MORSE_SESION_SIN_LATENCIA
                    : (espera < MORSE_SESION_SIN_LATENCIA) ? (uint16_t)espera : MORSE_SESION_SIN_LATENCIA - 1U;
  uint8_t  correcto   = (c == actual);
  uint8_t  reconocido = correcto && (modo == APRENDER || latencia <= LATENCIA_MAX_MS);
  grande = actual;

  /* Copia con la IRQ parada: si ya empezó otra letra, la ISR pisaría las duraciones */
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  MORSE_Decodificador_t manip = decodificador;
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  aviso = (c != 0) ? MORSE_EvaluarManip(&manip, actual, koch.nivel) : NULL;

  if (modo == IDENTIFICAR)
  {
    (void)MORSE_SesionRegistrar(&sesion, actual, c, reconocido, latencia);  /* actual sale de MORSE_KochOrden */
    /* El repaso no cuenta en la progresión: su muestra está sesgada hacia lo difícil */
    presentar = repaso ? 0U : MORSE_KochRegistrar(&koch, reconocido);
    if (presentar)
    {
      AjustarVelocidad();
    }
  }

  if (correcto)
  {
    if (reconocido)
    {
      bip_ms = BIP_MS;
      if (modo == APRENDER)
      {
        siguiente = (uint8_t)((siguiente + 1U) % koch.nivel);
      }
    }
    ver_patron = 0;
    estado     = reconocido ? "Bien" : "Bien, pero lento";
    redibujar  = 1;
    Pausa();
    return;
  }

  snprintf(tu, sizeof tu, "Tu: %s", manip.patron);
  ver_patron = mostrar_patron;
  estado     = (c == 0) ? "Sin respuesta" : mostrar_patron ? tu : "Fallo";
  /* "K M": el correcto, pausa de palabra y lo manipulado ('?' y 0 no suenan) */
  sonido[0] = actual;
  sonido[1] = ' ';
  sonido[2] = c;
  sonido[3] = '\0';
  Sonar((modo == APRENDER) ? RESPUESTA : PAUSA);
}

/**
 * @brief Grupos: acumula lo manipulado y corrige al completar el grupo o tras
 *        ESPERA_MAX_MS sin letras nuevas. Cada carácter cuenta en la progresión Koch
 *        y en las estadísticas (sin latencia). Si todo está bien, bip; si no, se ven
 *        el grupo oído y lo manipulado y el grupo vuelve a sonar.
 *        Un carácter no reconocido se marca con 6 o más puntos ('?'), así el resto
 *        no se desplaza.
 */
static void Copiar(char c)
{
  if (c != 0 && c != ' ')
  {
    respuesta[n_respuesta++] = c;
    ultima_en = HAL_GetTick();
    redibujar = 1;
    if (n_respuesta < GRUPO_LARGO)
    {
      return;
    }
  }
  else if (HAL_GetTick() - ultima_en < ESPERA_MAX_MS)
  {
    return;
  }

  /* ponytail: comparación posición a posición; una letra saltada desplaza el resto.
     Por eso las confusiones solo se anotan con el grupo completo. Alinear si molesta. */
  uint8_t bien = 0;
  for (uint8_t i = 0; i < GRUPO_LARGO; i++)
  {
    uint8_t ok = (respuesta[i] == sonido[i]);   /* Lo que falta es '\0': fallo */
    bien += ok;
    (void)MORSE_SesionRegistrar(&sesion, sonido[i], (n_respuesta == GRUPO_LARGO) ? respuesta[i] : 0,
                                ok, MORSE_SESION_SIN_LATENCIA);  /* sonido sale de MORSE_KochOrden */
    presentar |= MORSE_KochRegistrar(&koch, ok);
  }
  if (presentar)
  {
    AjustarVelocidad();
  }

  snprintf(tu, sizeof tu, "%u/%u bien", bien, GRUPO_LARGO);
  estado     = tu;
  ver_patron = 1;                           /* En Grupos: mostrar el grupo oído */
  redibujar  = 1;
  if (bien == GRUPO_LARGO)
  {
    bip_ms = BIP_MS;
    Pausa();
  }
  else
  {
    Sonar(PAUSA);                           /* El grupo otra vez, ya viéndolo */
  }
}

/**
 * @brief Libre: añade lo decodificado al texto; al llenarse las LIBRE_FILAS líneas,
 *        sube una. La línea inferior muestra el último carácter y lo manipulado.
 */
static void Anotar(char c)
{
  size_t n = strlen(libre);
  if (c == ' ' && (n == 0U || libre[n - 1U] == ' '))
  {
    return;                                 /* Fin de la palabra con la que se eligió */
  }
  if (n == LIBRE_COLUMNAS * LIBRE_FILAS)
  {
    n -= LIBRE_COLUMNAS;
    memmove(libre, libre + LIBRE_COLUMNAS, n);
  }
  libre[n]      = c;
  libre[n + 1U] = '\0';

  if (c != ' ')
  {
    char patron[sizeof decodificador.patron];
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    memcpy(patron, decodificador.patron, sizeof patron);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    snprintf(tu, sizeof tu, "%c %s", c, patron);
    estado = tu;
  }
  redibujar = 1;
}

static void ElegirOpcion(void)
{
  switch (opcion)
  {
    case OPC_APRENDER:
    case OPC_IDENTIFICAR:
    case OPC_GRUPOS:
    case OPC_REPASO:
      srand(HAL_GetTick());                 /* El momento de la pulsación es azar suficiente */
      if (sesion_desde == 0U)
      {
        sesion_desde = HAL_GetTick();       /* La sesión empieza con la primera lección */
      }
      modo      = (opcion == OPC_APRENDER) ? APRENDER : (opcion == OPC_GRUPOS) ? GRUPOS
                                                                  : IDENTIFICAR;
      repaso    = (opcion == OPC_REPASO);
      siguiente = (uint8_t)(koch.nivel - 1U);   /* Aprender empieza por el más nuevo */
      presentar = 0;
      Preguntar();
      break;
    case OPC_LIBRE:
      modo     = LIBRE;
      libre[0] = '\0';
      /* Fin de palabra a 5 puntos del usuario (entre 3 y 7) en vez del umbral Farnsworth
         de la reproducción. Solo Libre usa los espacios: no hace falta restaurarlo. */
      decodificador.palabra_ms = 0;
      estado   = "Manipula";
      break;
    case OPC_ESTADISTICAS:
      modo = ESTADISTICAS;
      break;
    case OPC_NIVEL:                         /* Sustituye a la persistencia: se elige al encender */
      (void)MORSE_KochInit(&koch, (koch.nivel >= MORSE_KOCH_TOTAL) ? MORSE_KOCH_MINIMO : (uint8_t)(koch.nivel + 1U));
      AjustarVelocidad();
      break;
    case OPC_WPM:
      wpm_caracter = (wpm_caracter >= 30U) ? 12U : (uint8_t)(wpm_caracter + 2U);
      if (wpm_efectivo > wpm_caracter)
      {
        wpm_efectivo = wpm_caracter;
      }
      AjustarVelocidad();
      break;
    case OPC_FARNSWORTH:                    /* Igual a la de carácter = sin Farnsworth */
      wpm_efectivo = (wpm_efectivo >= wpm_caracter) ? 4U
                   : (wpm_efectivo + 2U > wpm_caracter) ? wpm_caracter
                   : (uint8_t)(wpm_efectivo + 2U);
      AjustarVelocidad();
      break;
    case OPC_TONO:
      tono_hz  = (tono_hz >= 1000U) ? 400U : (uint16_t)(tono_hz + 100U);
      tono_arr = (uint16_t)(1000000UL / tono_hz - 1U);
      Reiniciar("T");                       /* Una raya para oír el tono nuevo */
      break;
    default:
      mostrar_patron = !mostrar_patron;
      break;
  }
  redibujar = 1;
}

/** @brief Resumen de la sesión (cuentan Identificar, Repaso y Grupos). */
static void DibujarEstadisticas(void)
{
  char linea[20];
  uint32_t intentos = 0;
  uint32_t aciertos = 0;
  for (uint8_t i = 0; i < MORSE_KOCH_TOTAL; i++)
  {
    intentos += sesion.intentos[i];
    aciertos += sesion.aciertos[i];
  }
  uint32_t minutos = (sesion_desde != 0U) ? (HAL_GetTick() - sesion_desde) / 60000U : 0U;

  snprintf(linea, sizeof linea, fin_sesion ? "Fin: %lu min" : "Sesion %lu min",
           (unsigned long)minutos);
  SSD1306_GotoXY(0, 0);
  SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);

  snprintf(linea, sizeof linea, "Bien %lu/%lu %lu%%", (unsigned long)aciertos,
           (unsigned long)intentos, (unsigned long)(intentos ? 100U * aciertos / intentos : 0U));
  SSD1306_GotoXY(0, 10);
  SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);

  if (sesion.latencias != 0U)
  {
    snprintf(linea, sizeof linea, "Latencia %lu ms",
             (unsigned long)(sesion.latencia_total_ms / sesion.latencias));
  }
  else
  {
    snprintf(linea, sizeof linea, "Latencia -");
  }
  SSD1306_GotoXY(0, 20);
  SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);

  char    fallados[4];
  uint8_t n     = MORSE_SesionMasFallados(&sesion, fallados, 3);
  int     largo = snprintf(linea, sizeof linea, "Fallos");
  for (uint8_t i = 0; i < n; i++)
  {
    largo += snprintf(linea + largo, sizeof linea - (size_t)largo, " %c", fallados[i]);
  }
  SSD1306_GotoXY(0, 30);
  SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);

  char pares[7];
  n     = MORSE_SesionConfusiones(&sesion, pares, 3);
  largo = snprintf(linea, sizeof linea, "Conf");
  for (uint8_t i = 0; i < n; i++)           /* oído>respondido */
  {
    largo += snprintf(linea + largo, sizeof linea - (size_t)largo, " %c>%c",
                      pares[2 * i], pares[2 * i + 1]);
  }
  SSD1306_GotoXY(0, 40);
  SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);

  /* Velocidad a la que manipula el usuario, según el decodificador adaptativo */
  snprintf(linea, sizeof linea, "Tu %u WPM Menu=sal",
           (uint8_t)(1200U / decodificador.punto_ms));   /* ≤ 60: punto ≥ 20 ms */
  SSD1306_GotoXY(0, 52);
  SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);
}

static void Dibujar(void)
{
  char linea[22];                          /* 18 visibles; holgura para -Wformat-truncation */
  SSD1306_Fill(SSD1306_COLOR_BLACK);

  if (modo == MENU)
  {
    uint8_t primera = (opcion >= FILAS_MENU) ? (uint8_t)(opcion - FILAS_MENU + 1U) : 0U;
    for (uint8_t i = primera; i < OPCIONES && i < primera + FILAS_MENU; i++)
    {
      char marca = (i == opcion) ? '>' : ' ';
      switch (i)
      {
        case OPC_APRENDER:    snprintf(linea, sizeof linea, "%c Aprender", marca); break;
        case OPC_IDENTIFICAR: snprintf(linea, sizeof linea, "%c Identificar", marca); break;
        case OPC_GRUPOS:      snprintf(linea, sizeof linea, "%c Grupos de %u", marca, GRUPO_LARGO); break;
        case OPC_REPASO:      snprintf(linea, sizeof linea, "%c Repaso", marca); break;
        case OPC_LIBRE:       snprintf(linea, sizeof linea, "%c Libre", marca); break;
        case OPC_ESTADISTICAS: snprintf(linea, sizeof linea, "%c Estadisticas", marca); break;
        case OPC_NIVEL:
          snprintf(linea, sizeof linea, "%c Nivel %u (..%c)", marca, koch.nivel,
                   MORSE_KochOrden[koch.nivel - 1U]);
          break;
        case OPC_WPM:
          snprintf(linea, sizeof linea, "%c Velocidad %u WPM", marca, wpm_caracter);
          break;
        case OPC_FARNSWORTH:
          if (wpm_efectivo < wpm_caracter)
          {
            snprintf(linea, sizeof linea, "%c Farnsworth %u", marca, wpm_efectivo);
          }
          else
          {
            snprintf(linea, sizeof linea, "%c Farnsworth no", marca);
          }
          break;
        case OPC_TONO:
          snprintf(linea, sizeof linea, "%c Tono %u Hz", marca, tono_hz);
          break;
        default:
          snprintf(linea, sizeof linea, "%c Patron: %s", marca, mostrar_patron ? "si" : "no");
          break;
      }
      SSD1306_GotoXY(0, (uint16_t)((i - primera) * 9U));  /* FILAS_MENU + ayuda: filas de 9 px */
      SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);
    }
    SSD1306_GotoXY(0, 54);
    SSD1306_Puts("Menu=sig Tecla=ok", &Font_7x10, SSD1306_COLOR_WHITE);
  }
  else if (modo == ESTADISTICAS)
  {
    DibujarEstadisticas();
  }
  else if (modo == LIBRE)
  {
    snprintf(linea, sizeof linea, "Libre  Tu %u WPM", (uint8_t)(1200U / decodificador.punto_ms));
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);
    for (uint8_t f = 0; f < LIBRE_FILAS; f++)
    {
      snprintf(linea, sizeof linea, "%.*s", (int)LIBRE_COLUMNAS,
               libre + (strnlen(libre, f * LIBRE_COLUMNAS)));
      SSD1306_GotoXY(0, (uint16_t)(12U + f * 10U));
      SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);
    }
    SSD1306_GotoXY(0, 52);
    SSD1306_Puts((char *)estado, &Font_7x10, SSD1306_COLOR_WHITE);
  }
  else
  {
    if (modo == APRENDER)
    {
      snprintf(linea, sizeof linea, "Aprender %u/%u WPM", wpm_caracter,
               MORSE_KochWpmEfectivo(koch.nivel, wpm_efectivo, wpm_caracter));
    }
    else if (repaso)                        /* Sin fallos aún: elige entre todos */
    {
      snprintf(linea, sizeof linea, "Repaso %s", repasar[0] ? repasar : "todos");
    }
    else
    {
      snprintf(linea, sizeof linea, "%s %u  %u/%u", (modo == GRUPOS) ? "Grupos" : "Nivel",
               koch.nivel, MORSE_KochAciertos(&koch), koch.intentos);
    }
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);
    if (modo == GRUPOS && grande == 0)      /* grande != 0: presentación de uno nuevo */
    {
      if (ver_patron)                       /* Tras corregir: el grupo oído */
      {
        SSD1306_GotoXY(0, 13);
        SSD1306_Puts(sonido, &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(66, 17);
        SSD1306_Puts("oido", &Font_7x10, SSD1306_COLOR_WHITE);
      }
      SSD1306_GotoXY(0, 33);
      SSD1306_Puts(respuesta, &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY(66, 37);
      SSD1306_Puts("tu", &Font_7x10, SSD1306_COLOR_WHITE);
    }
    else if (grande != 0)
    {
      SSD1306_GotoXY(8, 16);
      SSD1306_Putc(grande, &Font_16x26, SSD1306_COLOR_WHITE);
      const char *patron;
      if (ver_patron && MORSE_Patron(grande, &patron) == MORSE_OK)
      {
        SSD1306_GotoXY(40, 20);
        SSD1306_Puts((char *)patron, &Font_11x18, SSD1306_COLOR_WHITE);
      }
    }
    if (aviso != NULL)                      /* Línea libre bajo el carácter grande */
    {
      SSD1306_GotoXY(0, 42);
      SSD1306_Puts((char *)aviso, &Font_7x10, SSD1306_COLOR_WHITE);
    }
    SSD1306_GotoXY(0, 52);
    SSD1306_Puts((char *)estado, &Font_7x10, SSD1306_COLOR_WHITE);
  }

  if (SSD1306_UpdateScreen() != SSD1306_OK)
  {
    Error_Handler();
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
  TIM3->CCR1 = 0;                           /* Silenciar el buzzer */
  while (1)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);  /* Parpadeo rápido = error */
    for (volatile uint32_t i = 0; i < 2000000U; i++) {}
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
