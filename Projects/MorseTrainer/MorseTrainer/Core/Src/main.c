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
#include "morse.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { MENU, APRENDER, IDENTIFICAR, ESTADISTICAS } Modo;
typedef enum { SONANDO, RESPUESTA, PAUSA } Paso;
typedef enum
{
  OPC_APRENDER, OPC_IDENTIFICAR, OPC_ESTADISTICAS, OPC_NIVEL, OPC_PATRON, OPCIONES
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
#define WPM_CARACTER    18U             /* Velocidad de los símbolos */
#define WPM_EFECTIVO    8U              /* Farnsworth en el nivel 2; llega a WPM_CARACTER en el 36 */
#define BIP_MS          60U             /* Bip de acierto, al doble de la frecuencia del tono */
#define PAUSA_MS        800U            /* Silencio antes del siguiente carácter */
#define LATENCIA_MAX_MS 2000U           /* Más hasta la primera pulsación = no reconocido */
#define ESPERA_MAX_MS   6000U           /* Sin pulsar en este tiempo = sin respuesta */
#define SESION_MS       (15UL * 60UL * 1000UL)  /* Duración recomendada de la sesión */
#define AVISO_MS        500U            /* Bip largo de fin de sesión */

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
static Morse_Reproductor   reproductor;
static Morse_Decodificador decodificador;
static volatile char       recibido = 0;    /* Último carácter decodificado; 0 = ya leído */
static volatile uint8_t    menu_pulsado = 0;  /* Se pulsó el botón de menú; 0 = ya leído */
static volatile uint16_t   bip_ms = 0;      /* Lo que queda del bip de acierto */
static volatile uint32_t   pulsado_en = 0;  /* HAL_GetTick() de la primera pulsación; 0 = ninguna */

/* -- Solo bucle principal -- */
static Morse_Tiempos tiempos;
static Koch_Progreso koch;
static Modo     modo = MENU;
static Opcion   opcion = OPC_APRENDER;      /* Resaltada en el menú */
static uint8_t  mostrar_patron = 1;         /* Patrón al presentar y al fallar */
static Paso     paso;
static Paso     tras_sonar;                 /* RESPUESTA o PAUSA al acabar de sonar */
static uint32_t pausa_desde;
static char     actual;                     /* Carácter que se espera como respuesta */
static uint8_t  siguiente;                  /* Aprender: índice en KOCH_ORDEN del próximo */
static uint8_t  presentar;                  /* Identificar: presentar el recién desbloqueado */
static char     sonido[4];                  /* Lo que suena; debe vivir hasta que acabe */
static char     tu[10];                     /* "Tu: " + patrón manipulado */
static uint32_t escucha_desde;              /* Fin de la reproducción: origen de la latencia */
static Sesion   sesion;
static uint32_t sesion_desde;               /* Primera lección; 0 = sin empezar */
static uint8_t  sesion_avisada;
static uint8_t  fin_sesion;                 /* Estadísticas abiertas por el temporizador */
/* -- Pantalla -- */
static char        grande;                  /* Carácter en grande; 0 = ninguno */
static uint8_t     ver_patron;
static const char *estado = "";             /* Línea inferior */
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
  Tono((uint16_t)htim3.Init.Period);
  HAL_Delay(PRUEBA_TONO_MS);
  Tono(0);

  /* -- Se inicializan antes del tick, así no hay carrera con la ISR -- */
  Koch_Init(&koch, KOCH_MINIMO);
  AjustarVelocidad();
  Morse_DecodInit(&decodificador, tiempos);
  Morse_Reproducir(&reproductor, "", tiempos);
  Sesion_Init(&sesion);

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
    else if (paso == SONANDO)
    {
      if (!Morse_Reproduciendo(&reproductor))
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
      if (c != 0 && c != ' ')
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

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
 *        La frecuencia del tono es htim3.Init.Period (.ioc): ARR = 1 000 000 / f - 1.
 *        Se escribe TIM3->ARR directamente porque __HAL_TIM_SET_AUTORELOAD pisaría Init.Period.
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

  uint8_t morse = Morse_ReprodTick(&reproductor) | pulsado;
  uint8_t bip   = 0;
  if (bip_ms != 0U)
  {
    bip_ms--;
    bip = 1;
  }
  uint16_t arr = (uint16_t)htim3.Init.Period;
  Tono(morse ? arr : bip ? (uint16_t)(arr / 2U) : 0U);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, morse ? GPIO_PIN_RESET : GPIO_PIN_SET);

  char c = Morse_DecodTick(&decodificador, pulsado);
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
  tiempos = Morse_CalcularTiempos(WPM_CARACTER,
                                  Koch_WpmEfectivo(koch.nivel, WPM_EFECTIVO, WPM_CARACTER));
}

/**
 * @brief Empieza a reproducir un texto ("" = silencio) y vacía el decodificador:
 *        lo manipulado antes no cuenta como respuesta. El punto que el decodificador
 *        ha aprendido del usuario se conserva.
 */
static void Reiniciar(const char *texto)
{
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  Morse_Reproducir(&reproductor, texto, tiempos);
  Morse_DecodVaciar(&decodificador);
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

  if (modo == APRENDER || presentar)
  {
    actual = KOCH_ORDEN[(modo == APRENDER) ? siguiente : koch.nivel - 1U];
    memset(sonido, actual, 3);
    sonido[3]  = '\0';
    grande     = actual;
    ver_patron = mostrar_patron;
    estado     = presentar ? "Nuevo caracter" : "Escucha";
    Sonar(presentar ? PAUSA : RESPUESTA);
    presentar = 0;
    return;
  }

  actual     = Sesion_Elegir(&sesion, koch.nivel, (uint32_t)rand());
  sonido[0]  = actual;
  sonido[1]  = '\0';
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
  uint16_t latencia = (pulsado == 0U) ? SESION_SIN_LATENCIA
                    : (espera < SESION_SIN_LATENCIA) ? (uint16_t)espera : SESION_SIN_LATENCIA - 1U;
  uint8_t  correcto   = (c == actual);
  uint8_t  reconocido = correcto && (modo == APRENDER || latencia <= LATENCIA_MAX_MS);
  grande = actual;

  if (modo == IDENTIFICAR)
  {
    Sesion_Registrar(&sesion, actual, c, reconocido, latencia);
    presentar = Koch_Registrar(&koch, reconocido);
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

  /* Antes de Sonar(), que vacía el decodificador */
  snprintf(tu, sizeof tu, "Tu: %s", decodificador.patron);
  ver_patron = mostrar_patron;
  estado     = (c == 0) ? "Sin respuesta" : mostrar_patron ? tu : "Fallo";
  /* "K M": el correcto, pausa de palabra y lo manipulado ('?' y 0 no suenan) */
  sonido[0] = actual;
  sonido[1] = ' ';
  sonido[2] = c;
  sonido[3] = '\0';
  Sonar((modo == APRENDER) ? RESPUESTA : PAUSA);
}

static void ElegirOpcion(void)
{
  switch (opcion)
  {
    case OPC_APRENDER:
    case OPC_IDENTIFICAR:
      srand(HAL_GetTick());                 /* El momento de la pulsación es azar suficiente */
      if (sesion_desde == 0U)
      {
        sesion_desde = HAL_GetTick();       /* La sesión empieza con la primera lección */
      }
      modo      = (opcion == OPC_APRENDER) ? APRENDER : IDENTIFICAR;
      siguiente = (uint8_t)(koch.nivel - 1U);   /* Aprender empieza por el más nuevo */
      presentar = 0;
      Preguntar();
      break;
    case OPC_ESTADISTICAS:
      modo = ESTADISTICAS;
      break;
    case OPC_NIVEL:                         /* Sustituye a la persistencia: se elige al encender */
      Koch_Init(&koch, (koch.nivel >= KOCH_TOTAL) ? KOCH_MINIMO : (uint8_t)(koch.nivel + 1U));
      AjustarVelocidad();
      break;
    default:
      mostrar_patron = !mostrar_patron;
      break;
  }
  redibujar = 1;
}

/** @brief Resumen de la sesión (solo cuenta el modo Identificar). */
static void DibujarEstadisticas(void)
{
  char linea[20];
  uint32_t intentos = 0;
  uint32_t aciertos = 0;
  for (uint8_t i = 0; i < KOCH_TOTAL; i++)
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
  uint8_t n     = Sesion_MasFallados(&sesion, fallados, 3);
  int     largo = snprintf(linea, sizeof linea, "Fallos");
  for (uint8_t i = 0; i < n; i++)
  {
    largo += snprintf(linea + largo, sizeof linea - (size_t)largo, " %c", fallados[i]);
  }
  SSD1306_GotoXY(0, 30);
  SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);

  char pares[7];
  n     = Sesion_Confusiones(&sesion, pares, 3);
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
  char linea[20];
  SSD1306_Fill(SSD1306_COLOR_BLACK);

  if (modo == MENU)
  {
    for (uint8_t i = 0; i < OPCIONES; i++)
    {
      char marca = (i == opcion) ? '>' : ' ';
      switch (i)
      {
        case OPC_APRENDER:    snprintf(linea, sizeof linea, "%c Aprender", marca); break;
        case OPC_IDENTIFICAR: snprintf(linea, sizeof linea, "%c Identificar", marca); break;
        case OPC_ESTADISTICAS: snprintf(linea, sizeof linea, "%c Estadisticas", marca); break;
        case OPC_NIVEL:
          snprintf(linea, sizeof linea, "%c Nivel %u (..%c)", marca, koch.nivel,
                   KOCH_ORDEN[koch.nivel - 1U]);
          break;
        default:
          snprintf(linea, sizeof linea, "%c Patron: %s", marca, mostrar_patron ? "si" : "no");
          break;
      }
      SSD1306_GotoXY(0, (uint16_t)(i * 10U));
      SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);
    }
    SSD1306_GotoXY(0, 52);
    SSD1306_Puts("Menu=sig Tecla=ok", &Font_7x10, SSD1306_COLOR_WHITE);
  }
  else if (modo == ESTADISTICAS)
  {
    DibujarEstadisticas();
  }
  else
  {
    if (modo == APRENDER)
    {
      snprintf(linea, sizeof linea, "Aprender %u/%u WPM", WPM_CARACTER,
               Koch_WpmEfectivo(koch.nivel, WPM_EFECTIVO, WPM_CARACTER));
    }
    else
    {
      snprintf(linea, sizeof linea, "Nivel %u  %u/%u", koch.nivel, Koch_Aciertos(&koch),
               koch.intentos);
    }
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts(linea, &Font_7x10, SSD1306_COLOR_WHITE);
    if (grande != 0)
    {
      SSD1306_GotoXY(8, 16);
      SSD1306_Putc(grande, &Font_16x26, SSD1306_COLOR_WHITE);
      if (ver_patron)
      {
        SSD1306_GotoXY(40, 20);
        SSD1306_Puts((char *)Morse_Patron(grande), &Font_11x18, SSD1306_COLOR_WHITE);
      }
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
