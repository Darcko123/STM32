/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @author Daniel Ruiz
 * @date Dec 19, 2025
 * @version 1.1.0
 ******************************************************************************
 * @attention
 *
 * Este archivo demuestra todas las funcionalidades principales de la librería:
 * - Control directo de ángulo
 * - Movimiento suave bloqueante
 * - Movimiento suave no bloqueante con múltiples servos
 * - Control de PWM de bajo nivel
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PCA9685_PWMModule.h"
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
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* Estructuras para control suave de servomotores */
Servo_Smooth_t servo0;  // Servo en canal 0
Servo_Smooth_t servo1;  // Servo en canal 1
Servo_Smooth_t servo2;  // Servo en canal 2

/* Variables de control */
uint8_t example_mode = 0;
uint32_t last_mode_change = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* Prototipos de funciones de ejemplo */
void Example_DirectControl(void);
void Example_SmoothBlocking(void);
void Example_SmoothNonBlocking(void);
void Example_MultipleServos(void);

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
  /* USER CODE BEGIN 2 */
	HAL_Delay(100);  // Esperar inicialización del PCA9685

	/* Inicializar el módulo PCA9685 a 50 Hz (frecuencia estándar para servos) */
	PCA9685_Init(&hi2c1, 50);
	/* Pequeño delay para estabilización */
	HAL_Delay(100);

	/* Inicializar estructuras para movimiento suave no bloqueante */
	PCA9685_InitSmoothServo(&servo0, 0, 90, 20);  // Canal 0, inicio en 90°, actualización cada 20ms
	PCA9685_InitSmoothServo(&servo1, 1, 90, 20);  // Canal 1, inicio en 90°, actualización cada 20ms
	PCA9685_InitSmoothServo(&servo2, 2, 90, 20);  // Canal 2, inicio en 90°, actualización cada 20ms

	// Poner todos los servos en posición inicial
	PCA9685_SetServoAngle(0, 90);
	PCA9685_SetServoAngle(1, 90);
	PCA9685_SetServoAngle(2, 90);

	last_mode_change = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/* Cambiar de ejemplo cada 10 segundos */

		if ((HAL_GetTick() - last_mode_change) > 10000)
		{
			example_mode++;
			if (example_mode > 3) example_mode = 0;
			last_mode_change = HAL_GetTick();
		}

		/* Ejecutar el ejemplo correspondiente */
		switch(example_mode)
		{
			case 0:
				Example_DirectControl();
				break;

			case 1:
				Example_SmoothBlocking();
				break;

			case 2:
				Example_SmoothNonBlocking();
				break;

			case 3:
				Example_MultipleServos();
				break;
		}

		HAL_Delay(10);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Ejemplo 1: Control directo de ángulo
 * Demuestra el uso de PCA9685_SetServoAngle() para control inmediato
 */
void Example_DirectControl(void)
{
    static uint32_t last_update = 0;
    static uint8_t position = 0;

    if ((HAL_GetTick() - last_update) > 1000)
    {
        switch(position)
        {
            case 0:
                // Mover servo del canal 0 a 0°
                PCA9685_SetServoAngle(0, 0);
                position = 1;
                break;

            case 1:
                // Mover servo del canal 0 a 90°
                PCA9685_SetServoAngle(0, 90);
                position = 2;
                break;

            case 2:
                // Mover servo del canal 0 a 180°
                PCA9685_SetServoAngle(0, 180);
                position = 0;
                break;
        }

        last_update = HAL_GetTick();
    }
}

/**
 * @brief Ejemplo 2: Movimiento suave bloqueante
 * Demuestra el uso de PCA9685_SmoothMove() para transiciones suaves
 * NOTA: Esta función bloquea la ejecución durante el movimiento
 */
void Example_SmoothBlocking(void)
{
    static bool sequence_running = false;

    // Ejecutar la secuencia completa solo una vez
    if (!sequence_running)
    {
        sequence_running = true;

        // Mover suavemente de 0° a 180° en 2 segundos
        PCA9685_SmoothMove(0, 0, 180, 2000, 20);
        HAL_Delay(500);

        // Regresar suavemente a 90°
        PCA9685_SmoothMove(0, 180, 90, 1500, 20);
        HAL_Delay(500);

        // Bajar a 0°
        PCA9685_SmoothMove(0, 90, 0, 1500, 20);
        HAL_Delay(500);

        // Subir a 90° (posición central)
        PCA9685_SmoothMove(0, 0, 90, 1000, 20);
        HAL_Delay(1000);

        sequence_running = false;
    }
}

/**
 * @brief Ejemplo 3: Movimiento suave no bloqueante
 * Demuestra el control de un servo sin bloquear el programa
 * Permite realizar otras tareas mientras el servo se mueve
 */
void Example_SmoothNonBlocking(void)
{
    static uint8_t state = 0;
    static uint32_t state_start = 0;

    // Actualizar el servo (esto debe llamarse continuamente)
    bool movement_finished = PCA9685_UpdateSmoothServo(&servo0);

    // Máquina de estados para diferentes movimientos
    switch(state)
    {
        case 0:
            // Iniciar movimiento a 180° después de 500ms
            if ((HAL_GetTick() - state_start) >= 500)
            {
                PCA9685_SetSmoothAngle(&servo0, 180, 2000);
                state = 1;
                state_start = HAL_GetTick();
            }
            break;

        case 1:
            // Esperar a que termine y luego ir a 0°
            if (movement_finished && ((HAL_GetTick() - state_start) >= 2000))
            {
                PCA9685_SetSmoothAngle(&servo0, 0, 2000);
                state = 2;
                state_start = HAL_GetTick();
            }
            break;

        case 2:
            // Esperar y regresar a 90°
            if (movement_finished && ((HAL_GetTick() - state_start) >= 2000))
            {
                PCA9685_SetSmoothAngle(&servo0, 90, 1500);
                state = 3;
                state_start = HAL_GetTick();
            }
            break;

        case 3:
            // Reiniciar el ciclo
            if (movement_finished && ((HAL_GetTick() - state_start) >= 1500))
            {
                state = 0;
                state_start = HAL_GetTick();
            }
            break;
    }
}

/**
 * @brief Ejemplo 4: Control simultáneo de múltiples servos
 * Demuestra cómo controlar varios servos de forma independiente y simultánea
 */
void Example_MultipleServos(void)
{
    static uint8_t choreography_step = 0;
    static uint32_t step_start = 0;

    // Actualizar todos los servos
    bool servo0_done = PCA9685_UpdateSmoothServo(&servo0);
    bool servo1_done = PCA9685_UpdateSmoothServo(&servo1);
    bool servo2_done = PCA9685_UpdateSmoothServo(&servo2);

    // Verificar si todos terminaron su movimiento
    bool all_done = servo0_done && servo1_done && servo2_done;

    if (all_done && ((HAL_GetTick() - step_start) > 500))
    {
        switch(choreography_step)
        {
            case 0:
                // Movimiento en cascada: todos a 180°
                PCA9685_SetSmoothAngle(&servo0, 180, 1000);
                PCA9685_SetSmoothAngle(&servo1, 180, 1200);
                PCA9685_SetSmoothAngle(&servo2, 180, 1400);
                choreography_step = 1;
                step_start = HAL_GetTick();
                break;

            case 1:
                // Movimiento sincronizado: todos a 0°
                PCA9685_SetSmoothAngle(&servo0, 0, 1500);
                PCA9685_SetSmoothAngle(&servo1, 0, 1500);
                PCA9685_SetSmoothAngle(&servo2, 0, 1500);
                choreography_step = 2;
                step_start = HAL_GetTick();
                break;

            case 2:
                // Movimiento alternado
                PCA9685_SetSmoothAngle(&servo0, 180, 1000);
                PCA9685_SetSmoothAngle(&servo1, 90, 1000);
                PCA9685_SetSmoothAngle(&servo2, 0, 1000);
                choreography_step = 3;
                step_start = HAL_GetTick();
                break;

            case 3:
                // Regresar a posición central
                PCA9685_SetSmoothAngle(&servo0, 90, 1200);
                PCA9685_SetSmoothAngle(&servo1, 90, 1200);
                PCA9685_SetSmoothAngle(&servo2, 90, 1200);
                choreography_step = 0;
                step_start = HAL_GetTick();
                break;
        }
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
