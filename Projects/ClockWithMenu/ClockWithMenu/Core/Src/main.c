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
#include "RTC.h"
#include "SI7021.h"
#include "liquidcrystal_i2c.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TIM_FREQ 72000000 ///< Frecuencia base para el cálculo del prescaler del temporizador.
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

//------------------------------------------ MENUS ------------------------------------------//
/**
 * @brief Matrices que contienen las opciones de los menús.
 * Cada matriz representa un nivel diferente del menú.
 */
char *menu1[] = {"Configuracion", "Alarma", "Atras"}; ///< Menú principal.
int sizemenu1 = sizeof(menu1) / sizeof(menu1[0]); ///< Número de opciones en el menú principal.

char *menu2[] = {"Hora", "Fecha", "Atras"};
int sizemenu2 = sizeof(menu2) / sizeof(menu2[0]);

char *menu2_1[] = {"Encendido", "Configuracion", "SIG", "Atras"};
int sizemenu2_1 = sizeof(menu2_1) / sizeof(menu2_1[0]);

char *menu3_1[] = {"Hora", "Minutos", "Segundos", "Atras"}; ///< Opciones para la configuración de la hora.
int sizemenu3_1 = sizeof(menu3_1) / sizeof(menu3_1[0]);

char *menu3_2[] = {"Dia", "Mes", "Ano", "Atras"}; ///< Opciones para la configuración de la fecha.
int sizemenu3_2 = sizeof(menu3_2) / sizeof(menu3_2[0]);

char *menu3_3[] = {"ON", "OFF", "Atras"}; ///< Opciones para encender/apagar alarma o señal.
int sizemenu3_3 = sizeof(menu3_3) / sizeof(menu3_3[0]);

char *menu4[] = {"++", "--", "Atras"}; ///< Opciones para ajustar valores (incremento/decremento).
int sizemenu4 = sizeof(menu4) / sizeof(menu4[0]);

char *menu4_1[] = {"Reset", "Atras"}; ///< Opciones para resetear valores.
int sizemenu4_1 = sizeof(menu4_1) / sizeof(menu4_1[0]);

/**
 * @brief Buffers para las cadenas mostradas en la pantalla LCD.
 */
char linea1[20];
char linea2[20];
char linea3[20];
char linea4[20];

/**
 * @brief Variables para el control de los niveles del menú.
 */
int level_menu  = 0;
int level2_menu = 0;
int level3_menu = 0;
int level4_menu = 0;

int contador = 0; ///< Control de desplazamiento entre las opciones del menú.

bool btnpress = false; ///< Estado del botón OK.

//-------------------------------------------------------------------------------------------//

/**
 * @brief Buffers para datos adicionales como temperatura y humedad.
 */
char buffer[20];
char bufferSemana[10];
char *nombre;
int dia_semana_num;

/* Variables para la temperatura y humedad */
float temperaturaF, humedadF;
int temperaturaInt, humedadInt;

//------ Control de Alarma No Bloqueante ------//
typedef enum {
    ALARM_IDLE,
    ALARM_RINGING,
    ALARM_TONE1,
    ALARM_TONE2,
    ALARM_PAUSE
} AlarmState_t;

AlarmState_t alarmState = ALARM_IDLE;
uint32_t alarmStartTime = 0;
uint32_t alarmLastToneTime = 0;
uint32_t alarmTotalDuration = 60000;   // Duración total de la alarma en ms (60 segundos)
uint32_t alarmToneDuration = 300;      // Duración de cada tono en ms
uint32_t alarmPauseDuration = 150;     // Pausa entre tonos en ms
uint32_t alarmCycleDuration = 1200;    // Duración de cada ciclo completo (tono1-tono2-pausa)
bool alarmTriggered = false;
static int lastAlarmSecond = -1;       // Variable para el reset del trigger

uint32_t Tone1 = 500;
uint32_t Tone2 = 1000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

//-------------- Reloj --------------//
//Algoritmo de Zeller
/**
 * @brief Algoritmo de Zeller para determinar el día de la semana.
 * @param d Día del mes.
 * @param m Mes (ajustado para enero y febrero).
 * @param a Año.
 * @return Número del día de la semana (0=sábado, 1=domingo, ...).
 */
int dia_semana(int d, int m, int a);

/**
 * @brief Devuelve el nombre del día de la semana.
 * @param dia Número del día (0=sábado, 1=domingo, ...).
 * @return Cadena con el nombre del día.
 */
char* nombre_dia(int dia);
//-----------------------------------//

//-------------- MENU --------------//
/**
 * @brief Lógica principal para manejar las opciones del menú.
 */
void selectOption(void);

/**
 * @brief Cambia entre opciones del menú usando botones de incremento y decremento.
 * @param sizemenu Número de opciones en el menú actual.
 * @return true si hubo un cambio, false en caso contrario.
 */
bool fnSwitch(uint8_t sizemenu);

/**
 * @brief Muestra las opciones del menú actual en la pantalla LCD.
 * @param pos Posición actual dentro del menú.
 * @param menus Puntero al arreglo de opciones del menú.
 * @param sizemenu Tamaño del arreglo del menú.
 */
void fn_menu(int pos, char *menus[], uint8_t sizemenu);

/**
 * @brief Imprime el valor actual de una variable en la pantalla LCD.
 * @param label Etiqueta del valor (por ejemplo, "Hora").
 * @param value Valor a mostrar.
 */
void printCurrentValue(const char* label, int value);
//---------------------------------//

/**
 * @brief Calcula el prescaler necesario para una frecuencia deseada.
 * @param frequency Frecuencia deseada en Hz.
 * @return Valor del prescaler.
 */
int presForFrequency (int frequency);

/**
 * @brief Alterna el estado del retroiluminado de la pantalla LCD.
 */
void Toggle_Backlight(void);

/**
 * @brief Maneja el sistema de alarma de manera no bloqueante.
 */
void handleAlarmSystem(void);

/**
 * @brief Inicia la secuencia de alarma.
 */
void startAlarmSequence(void);

/**
 * @brief Detiene la alarma y resetea el estado.
 */
void stopAlarmSequence(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Variable para mantener el estado del retroiluminado
uint8_t backlightState = 0; // 0: Apagado, 1: Encendido
uint8_t previousButtonState = GPIO_PIN_RESET;
uint8_t currentButtonState;

//------ Control reloj ------//
TIME time;
ALARM1 alarma;

//Encendido de Alarma
bool Alarma = true;

//Encendido de SIG
bool SIG = true;
//---------------------------//

uint32_t lastUpdateTime = 0; // Variable para almacenar el último tiempo de actualización
const uint32_t updateInterval = 1000; // Intervalo de actualización en ms

float lastTemperaturaInt = -1.0; // Valor anterior de la temperatura para comparar
float lastHumedadInt = -1.0;     // Valor anterior de la humedad para comparar

int cadenaTemperatura = 0;		// Variable para controlar la cadena de actualización de temperatura
int lastCadenaTemperatura = -1; // Último valor de la cadena de temperatura

int anioBisiesto;

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
  /* USER CODE BEGIN 2 */

  //---------------- LCD ----------------//
  HD44780_Init(4);
  HD44780_Clear();
  HD44780_Backlight();
  //-------------------------------------//

  //---------------- Temperatura y humedad ----------------//
  // Inicialización de sensores.
  SI7021_Init();
  //-------------------------------------------------------//

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  //Si es la primera vez que se configura el módulo RTC, primero programar con la siguiente línea
  //RTC_SetTime(0, 0, 0, 1, 1, 25);
  //RTC_SetAlarm1(0, 0, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Actualizar la hora y la alarma desde el RTC.
      RTC_GetTime(&time);
      RTC_GetAlarm1(&alarma);

      // Determinar el día de la semana basado en la fecha actual.
      dia_semana_num = dia_semana(time.dayofmonth, time.month, time.year);
      nombre = nombre_dia(dia_semana_num);
	  /*---------------------------------------- */

      // Manejo de las opciones del menú.
      selectOption();

      // Actualizar datos de temperatura y humedad cada segundo.
      if (HAL_GetTick() - lastUpdateTime >= updateInterval)
      {
          Get_SI7021(&temperaturaF, &humedadF);

		  // Actualizar la temperatura solo si ha cambiado.
          if (lastHumedadInt != humedadF)
          {
              humedadInt = (int)humedadF;
          }

          lastUpdateTime = HAL_GetTick(); // Actualiza el límite de referencia.
      }

	  // PANTALLA PRINCIPAL
      // Lógica para la pantalla principal del sistema.
	  if(level_menu == 0)
	  {
		  // Mostrar hora.
		  sprintf(buffer, "%02d:%02d:%02d   ", time.hour, time.minutes, time.seconds);
		  HD44780_SetCursor(0, 0);
		  HD44780_PrintStr(buffer);

		  // Mostrar día de la semana.
		  HD44780_SetCursor(11, 0);
		  HD44780_PrintStr(nombre);

		  // Mostrar fecha.
		  sprintf(buffer, "%02d-%02d-20%02d     ", time.dayofmonth, time.month, time.year);
		  HD44780_SetCursor(0, 1);
		  HD44780_PrintStr(buffer);

          // Mostrar temperatura.
          sprintf(buffer, "T = %.1f", temperaturaF);
          HD44780_SetCursor(0, 2);
          HD44780_PrintStr(buffer);
          HD44780_PrintSpecialChar(6); // Símbolo de grados.
          HD44780_PrintStr("C");

          // Mostrar humedad.
          sprintf(buffer, "H = %02d %%", humedadInt);
          HD44780_SetCursor(0, 3);
          HD44780_PrintStr(buffer);

          // Manejo de señal SIG.
          if (SIG == true)
          {
              HD44780_SetCursor(15, 1);
              HD44780_PrintStr("SIG");

              if ((time.minutes == 0) && (time.seconds == 0))
              {
                  __HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(7100));
                  HAL_Delay(250);
                  __HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(0));
              }
          }

          // Manejar sistema de alarma no bloqueante
          handleAlarmSystem();

          // En la sección de PANTALLA PRINCIPAL (level_menu == 0), reemplazar:
          // Manejo de alarma.
          if (Alarma == true)
          {
              sprintf(buffer, "%02d:%02d:%02d", alarma.hour, alarma.minutes, alarma.seconds);
              HD44780_SetCursor(12, 2);
              HD44780_PrintStr(buffer);

              HD44780_SetCursor(19, 1);
              HD44780_PrintSpecialChar(5);

              // Mostrar indicador si la alarma está sonando
              if (alarmState != ALARM_IDLE)
              {
                  HD44780_SetCursor(15, 3);
                  HD44780_PrintStr("RING");
              }
              else
              {
                  HD44780_SetCursor(15, 3);
                  HD44780_PrintStr("    ");
              }
          }
          else if(Alarma == false)
          {
              sprintf(buffer, "        ");
              HD44780_SetCursor(12, 2);
              HD44780_PrintStr(buffer);
              HD44780_SetCursor(15, 3);
              HD44780_PrintStr("    ");
          }

          // Entrar al menú principal al presionar el botón.
          if (btnpress)
          {
              level_menu = 1;
              fn_menu(contador, menu1, sizemenu1);
              btnpress = false;
          }
	  }

	  // MENU 1 PRINCIPAL
      // Lógica para el menú principal.
      if (level_menu == 1)
      {
          // Navegar entre opciones del menú principal.
          if (fnSwitch(sizemenu1))
          {
              fn_menu(contador, menu1, sizemenu1);
          }

          // Selección de opciones en el menú principal.
          if (btnpress)
          {
              if (contador == 0) // Configuración.
              {
                  contador = 0;
                  fn_menu(contador, menu2, sizemenu2);
                  level_menu = 2;
              }
              else if (contador == 1) // Mostrar Alarma.
              {
                  contador = 0;
                  fn_menu(contador, menu2_1, sizemenu2_1);
                  level_menu = 3;
              }
              else if (contador == 2) // Atrás.
              {
                  contador = 0;
                  level_menu = 0;
              }
              btnpress = false;
          }
      }

	  // Detección del botón y alternar el estado del retroiluminado
	  currentButtonState = HAL_GPIO_ReadPin(LCDLed_GPIO_Port, LCDLed_Pin);
	  if (currentButtonState == GPIO_PIN_SET && previousButtonState == GPIO_PIN_RESET)
	  {
		  Toggle_Backlight();
	  }
	  previousButtonState = currentButtonState;

	  // MENU 2 {"HORA", "FECHA", "ATRAS"}
	  if (level_menu == 2)
	  {
		  if (level2_menu == 0)
		  {
			  if (fnSwitch(sizemenu2))
			  {
				  fn_menu(contador, menu2, sizemenu2);
			  }
			  if (btnpress)
			  {
				  if (contador == 0) //HORA
				  {
					  contador = 0;
					  fn_menu(contador, menu3_1, sizemenu3_1);
					  level2_menu = 1;
				  }
				  else if (contador == 1) //FECHA
				  {
					  contador = 0;
					  fn_menu(contador, menu3_2, sizemenu3_2);
					  level2_menu = 2;
				  }
				  else if (contador == 2) //ATRAS
				  {
					  contador = 0;
					  fn_menu(contador, menu1, sizemenu1);
					  level_menu = 1;
				  }
				  btnpress = false;
			  }
		  }

		  // MENU 3_1 : {"Hora", "Minutos", "Segundos", "Atras"}
		  if (level2_menu == 1)
		  {
			  if(level3_menu == 0)
			  {
				  if (fnSwitch(sizemenu3_1))
				  {
					  fn_menu(contador, menu3_1, sizemenu3_1);
				  }
				  if (btnpress)
				  {
					  if (contador == 0) //Hora
					  {
						  contador = 0;
						  fn_menu(contador, menu4, sizemenu4);
						  level3_menu = 1;
					  }
					  else if (contador == 1) //Minutos
					  {
						  contador = 0;
						  fn_menu(contador, menu4, sizemenu4);
						  level3_menu = 2;
					  }
					  else if (contador == 2) //Segundos
					  {
						  contador = 0;
						  fn_menu(contador, menu4_1, sizemenu4_1);
						  level3_menu = 3;
					  }
					  else if (contador == 3) //Atrás
					  {
						  contador = 0;
						  fn_menu(contador, menu2, sizemenu2);
						  level2_menu = 0;
					  }
					  btnpress = false;
				  }
			  }
		  }

		  // MENU 3_2: Configuración de Fecha
		  if (level2_menu == 2)
		  {
			  if(level3_menu == 0)
			  {
				  if (fnSwitch(sizemenu3_2))
				  {
					  fn_menu(contador, menu3_2, sizemenu3_2);
				  }
				  if (btnpress)
				  {
					  if (contador == 0) //Dia
					  {
						  contador = 0;
						  fn_menu(contador, menu4, sizemenu4);
						  level3_menu = 4;
					  }
					  else if (contador == 1) //Mes
					  {
						  contador = 0;
						  fn_menu(contador, menu4, sizemenu4);
						  level3_menu = 5;
					  }
					  else if (contador == 2) //Año
					  {
						  contador = 0;
						  fn_menu(contador, menu4, sizemenu4);
						  level3_menu = 6;
					  }
					  else if (contador == 3) //Atrás
					  {
						  contador = 0;
						  fn_menu(contador, menu2, sizemenu2);
						  level2_menu = 0;
					  }
					  btnpress = false;
				  }
			  }
		  }

		  // Configuración Horas
		  if (level3_menu == 1)
		  {
			  if (fnSwitch(sizemenu4))
			  {
				  fn_menu(contador, menu4, sizemenu4);
				  printCurrentValue("Hora", time.hour);
			  }
			  if (btnpress)
			  {
				  printCurrentValue("Hora", time.hour);
				  if (contador == 0) //++
				  {
					  time.hour++;
					  if(time.hour > 23)
					  {
						  time.hour = 0;
					  }
					  RTC_SetTime(time.hour, time.minutes, time.seconds, time.dayofmonth, time.month, time.year);
				  }
				  else if (contador == 1) //--
				  {
					  time.hour--;
					  if(time.hour < 0)
					  {
						  time.hour = 23;
					  }
					  RTC_SetTime(time.hour, time.minutes, time.seconds, time.dayofmonth, time.month, time.year);
				  }
				  else if (contador == 2) //Atras
				  {
					  contador = 0;
					  fn_menu(contador, menu3_1, sizemenu3_1);
					  level3_menu = 0;
				  }
				  printCurrentValue("Hora", time.hour);
				  btnpress = false;
			  }
		  }

		  // Configuración minutos
		  if (level3_menu == 2)
		  {
			  if (fnSwitch(sizemenu4))
			  {
				  fn_menu(contador, menu4, sizemenu4);
				  printCurrentValue("Minutos", time.minutes);
			  }
			  if (btnpress)
			  {
				  printCurrentValue("Minutos", time.minutes);
				  if (contador == 0) //++
				  {
					  time.minutes++;
					  if(time.minutes > 59)
					  {
						  time.minutes = 0;
					  }
					  RTC_SetTime(time.hour, time.minutes, time.seconds, time.dayofmonth, time.month, time.year);
				  }
				  else if (contador == 1) //--
				  {
					  time.minutes--;
					  if(time.minutes <= 0)
					  {
						  time.minutes = 0;
					  }
					  RTC_SetTime(time.hour, time.minutes, time.seconds, time.dayofmonth, time.month, time.year);
				  }
				  else if (contador == 2) //Atras
				  {
					  contador = 0;
					  fn_menu(contador, menu3_1, sizemenu3_1);
					  level3_menu = 0;
				  }
				  printCurrentValue("Minutos", time.minutes);
				  btnpress = false;
			  }
		  }

		  // Configuración segundos
		  if (level3_menu == 3)
		  {
			  if (fnSwitch(sizemenu4_1))
			  {
				  fn_menu(contador, menu4_1, sizemenu4_1);
				  printCurrentValue("Segundos", time.seconds);
			  }
			  if (btnpress)
			  {
				  printCurrentValue("Segundos", time.seconds);
				  if (contador == 0) //Reset
				  {
					  time.seconds = 0;
					  RTC_SetTime(time.hour, time.minutes, time.seconds, time.dayofmonth, time.month, time.year);
				  }
				  else if (contador == 1) //Atras
				  {
					  contador = 0;
					  fn_menu(contador, menu3_1, sizemenu3_1);
					  level3_menu = 0;
				  }
				  printCurrentValue("Segundos", time.seconds);
				  btnpress = false;
			  }
		  }

		  /**
		   * @brief Configuración del día en el sistema de menús.
		   * Este bloque de código permite ajustar el día del mes en función del mes actual
		   * y considera años bisiestos para el mes de febrero.
		   */

		  // Configuración del día
		  // Summary: Este bloque maneja el incremento y la disminución del día del mes con lógica envolvente,
		  // asegurando límites correctos para cada mes y años bisiestos (febrero).).
		  if (level3_menu == 4)
		  {
			  // Manejo de cambios en el menú (botones incrementales y decrementales)
			  if (fnSwitch(sizemenu4))
			  {
				  // Mostrar opciones de menú e imprimir el valor actual del día
				  fn_menu(contador, menu4, sizemenu4);
				  printCurrentValue("Dia", time.dayofmonth);
			  }

			  // Manejo de la selección con el botón OK
			  if (btnpress)
			  {
				  printCurrentValue("Dia", time.dayofmonth);

				  if (contador == 0) // Incrementar el día (++).
		          {
		              time.dayofmonth++;

		              // Verificar el límite superior del día dependiendo del mes
		              if ((time.month == 1 || time.month == 3 || time.month == 5 || time.month == 7 ||
		                   time.month == 8 || time.month == 10 || time.month == 12) && time.dayofmonth > 31)
		              {
		                  time.dayofmonth = 1;
		              }
		              else if ((time.month == 4 || time.month == 6 || time.month == 9 || time.month == 11) && time.dayofmonth > 30)
		              {
		                  time.dayofmonth = 1;
		              }
		              else if (time.month == 2)
		              {
		                  // Verificar si el año es bisiesto para determinar los días de febrero
						  anioBisiesto = ((time.year % 4 == 0 && time.year % 100 != 0) || (time.year % 400 == 0));
						  if ((anioBisiesto && time.dayofmonth > 29) || (!anioBisiesto && time.dayofmonth > 28))
						  {
							  time.dayofmonth = 1;
						  }
		              }

		              // Actualizar la hora en el RTC con el nuevo día
		              RTC_SetTime(time.hour, time.minutes, time.seconds, time.dayofmonth, time.month, time.year);
		          }
		          else if (contador == 1) // Decrementar el día (--).
		          {
		              time.dayofmonth--;

		              // Verificar el límite inferior del día
		              if (time.dayofmonth < 1)
		              {
		                  if (time.month == 1 || time.month == 3 || time.month == 5 || time.month == 7 ||
		                      time.month == 8 || time.month == 10 || time.month == 12)
		                  {
		                      time.dayofmonth = 31;
		                  }
		                  else if (time.month == 4 || time.month == 6 || time.month == 9 || time.month == 11)
		                  {
		                      time.dayofmonth = 30;
		                  }
		                  else if (time.month == 2)
		                  {
		                      // Verificar si el año es bisiesto para determinar los días de febrero
							  anioBisiesto = ((time.year % 4 == 0 && time.year % 100 != 0) || (time.year % 400 == 0));
							  time.dayofmonth = anioBisiesto ? 29 : 28;
		                  }
		              }

		              // Actualizar la hora en el RTC con el nuevo día
		              RTC_SetTime(time.hour, time.minutes, time.seconds, time.dayofmonth, time.month, time.year);
		          }
		          else if (contador == 2) // Regresar al menú anterior.
		          {
		              contador = 0;
		              fn_menu(contador, menu3_2, sizemenu3_2);
		              level3_menu = 0;
		          }

		          // Imprimir el valor actualizado del día
		          printCurrentValue("Dia", time.dayofmonth);
		          btnpress = false; // Restablecer el estado del botón
		      }
		  }

		  // Configuración mes
		  if (level3_menu == 5)
		  {
			  if (fnSwitch(sizemenu4))
			  {
				  fn_menu(contador, menu4, sizemenu4);
				  printCurrentValue("Mes", time.month);
			  }
			  if (btnpress)
			  {
				  printCurrentValue("Mes", time.month);
				  if (contador == 0) //++
				  {
					  time.month++;
					  if(time.month > 12)
					  {
						  time.month = 1;
					  }
					  RTC_SetTime(time.hour, time.minutes, time.seconds, time.dayofmonth, time.month, time.year);
				  }
				  else if (contador == 1) //--
				  {
					  time.month--;
					  if(time.month <= 0)
					  {
						  time.month = 12;
					  }
					  RTC_SetTime(time.hour, time.minutes, time.seconds, time.dayofmonth, time.month, time.year);
				  }
				  else if (contador == 2) //Atras
				  {
					  contador = 0;
					  fn_menu(contador, menu3_2, sizemenu3_2);
					  level3_menu = 0;
				  }
				  printCurrentValue("Mes", time.month);
				  btnpress = false;
			  }
		  }

		  // Configuración año
		  if (level3_menu == 6)
		  {
			  if (fnSwitch(sizemenu4))
			  {
				  fn_menu(contador, menu4, sizemenu4);
				  printCurrentValue("Anio", time.year);
			  }
			  if (btnpress)
			  {
				  printCurrentValue("Anio", time.year);
				  if (contador == 0) //++
				  {
					  time.year++;
					  RTC_SetTime(time.hour, time.minutes, time.seconds, time.dayofmonth, time.month, time.year);
				  }
				  else if (contador == 1) //--
				  {
					  time.year--;
					  RTC_SetTime(time.hour, time.minutes, time.seconds, time.dayofmonth, time.month, time.year);
				  }
				  else if (contador == 2) //Atras
				  {
					  contador = 0;
					  fn_menu(contador, menu3_2, sizemenu3_2);
					  level3_menu = 0;
				  }
				  printCurrentValue("Anio", time.year);
				  btnpress = false;
			  }
		  }
	  }

	  if(level_menu == 3)
	  {
		  if (level2_menu == 0)
		  {
			  if (fnSwitch(sizemenu2_1))
			  {
				  fn_menu(contador, menu2_1, sizemenu2_1);
			  }
			  if (btnpress)
			  {
				  if (contador == 0) //Encendido
				  {
					  contador = 0;
					  fn_menu(contador, menu3_3, sizemenu3_3);
					  level2_menu = 1;
				  }
				  else if (contador == 1) //Configuracion
				  {
					  contador = 0;
					  fn_menu(contador, menu3_1, sizemenu3_1);
					  level2_menu = 2;
				  }
				  else if(contador == 2)	//SIG
				  {
					  contador = 0;
					  fn_menu(contador, menu3_3, sizemenu3_3);
					  level2_menu = 3;
				  }
				  else if (contador == 3) //ATRAS
				  {
					  contador = 0;
					  fn_menu(contador, menu1, sizemenu1);
					  level_menu = 1;
				  }
				  btnpress = false;
			  }
		  }

		  if(level2_menu == 1)
		  {
			  if (fnSwitch(sizemenu3_3))
			  {
				  fn_menu(contador, menu3_3, sizemenu3_3);
			  }
			  if (btnpress)
			  {
				  if (contador == 0) 		//ON
				  {
					  Alarma = true;
				  }
				  else if (contador == 1) //OFF
				  {
					  Alarma = false;
				  }
				  else if (contador == 2) //ATRAS
				  {
					  contador = 0;
					  fn_menu(contador, menu2_1, sizemenu2_1);
					  level2_menu = 0;
				  }
				  btnpress = false;
			  }
		  }

		  // MENU 3_1 : {"Hora", "Minutos", "Segundos"}
		  if (level2_menu == 2)
		  {
			  if(level3_menu == 0)
			  {
				  if (fnSwitch(sizemenu3_1))
				  {
					  fn_menu(contador, menu3_1, sizemenu3_1);
				  }
				  if (btnpress)
				  {
					  if (contador == 0) //Hora
					  {
						  contador = 0;
						  fn_menu(contador, menu4, sizemenu4);
						  level3_menu = 1;
					  }
					  else if (contador == 1) //Minutos
					  {
						  contador = 0;
						  fn_menu(contador, menu4, sizemenu4);
						  level3_menu = 2;
					  }
					  else if (contador == 2) //Segundos
					  {
						  contador = 0;
						  fn_menu(contador, menu4, sizemenu4);
						  level3_menu = 3;
					  }
					  else if (contador == 3) //Atrás
					  {
						  contador = 0;
						  fn_menu(contador, menu2_1, sizemenu2_1);
						  level2_menu = 0;
					  }
					  btnpress = false;
				  }
			  }
		  }

		  if(level2_menu == 3)
		  {
			  if (fnSwitch(sizemenu3_3))
			  {
				  fn_menu(contador, menu3_3, sizemenu3_3);
			  }
			  if (btnpress)
			  {
				  if (contador == 0) 		//ON
				  {
					  SIG = true;
				  }
				  else if (contador == 1) //OFF
				  {
					  SIG = false;
				  }
				  else if (contador == 2) //ATRAS
				  {
					  contador = 0;
					  fn_menu(contador, menu2_1, sizemenu2_1);
					  level2_menu = 0;
				  }
				  btnpress = false;
			  }
		  }

		  // Configuración Horas
		  if (level3_menu == 1)
		  {
			  if (fnSwitch(sizemenu4))
			  {
				  fn_menu(contador, menu4, sizemenu4);
				  printCurrentValue("AlarmH", alarma.hour);
			  }
			  if (btnpress)
			  {
				  printCurrentValue("AlarmH", alarma.hour);
				  if (contador == 0) //++
				  {
					  alarma.hour = adjustHour(alarma.hour, 1);
					  RTC_SetAlarm1(alarma.hour, alarma.minutes, alarma.seconds);
				  }
				  else if (contador == 1) //--
				  {
					  alarma.hour = adjustHour(alarma.hour, -1);
					  RTC_SetAlarm1(alarma.hour, alarma.minutes, alarma.seconds);
				  }
				  else if (contador == 2) //Atras
				  {
					  contador = 0;
					  fn_menu(contador, menu3_1, sizemenu3_1);
					  level3_menu = 0;
				  }
				  printCurrentValue("AlarmH", alarma.hour);
				  btnpress = false;
			  }
		  }

		  // Configuración minutos
		  if (level3_menu == 2)
		  {
			  if (fnSwitch(sizemenu4))
			  {
				  fn_menu(contador, menu4, sizemenu4);
				  printCurrentValue("AlarmM", alarma.minutes);
			  }
			  if (btnpress)
			  {
				  printCurrentValue("AlarmM", alarma.minutes);
				  if (contador == 0) //++
				  {
					  alarma.minutes++;
					  if(alarma.minutes > 59)
					  {
						  alarma.minutes = 0;
					  }
					  RTC_SetAlarm1(alarma.hour, alarma.minutes, alarma.seconds);
				  }
				  else if (contador == 1) //--
				  {
					  alarma.minutes--;
					  // Si ocurre underflow (menor que 0), volver a 59
					  if(alarma.minutes < 0)
					  {
						  alarma.minutes = 59;
					  }
					  RTC_SetAlarm1(alarma.hour, alarma.minutes, alarma.seconds);
				  }
				  else if (contador == 2) //Atras
				  {
					  contador = 0;
					  fn_menu(contador, menu3_1, sizemenu3_1);
					  level3_menu = 0;
				  }
				  printCurrentValue("AlarmM", alarma.minutes);
				  btnpress = false;
			  }
		  }

		  // Configuración segundos
		  if (level3_menu == 3)
		  {
			  if (fnSwitch(sizemenu4))
			  {
				  fn_menu(contador, menu4, sizemenu4);
				  printCurrentValue("AlarmS", alarma.seconds);
			  }
			  if (btnpress)
			  {
				  printCurrentValue("AlarmS", alarma.seconds);
				  if (contador == 0) //++
				  {
					  alarma.seconds++;
					  if(alarma.seconds > 59)
					  {
						  alarma.seconds = 0;
					  }
					  RTC_SetAlarm1(alarma.hour, alarma.minutes, alarma.seconds);
				  }
				  else if (contador == 1) //--
				  {
					  alarma.seconds--;
					  if(alarma.seconds < 0)
					  {
						  alarma.seconds = 59;
					  }
					  RTC_SetAlarm1(alarma.hour, alarma.minutes, alarma.seconds);
				  }
				  else if (contador == 2) //Atras
				  {
					  contador = 0;
					  fn_menu(contador, menu3_1, sizemenu3_1);
					  level3_menu = 0;
				  }
				  printCurrentValue("AlarmS", alarma.seconds);
				  btnpress = false;
			  }
		  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : LCDLed_Pin */
  GPIO_InitStruct.Pin = LCDLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LCDLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ButtonMinus_Pin ButtonOk_Pin ButtonPlus_Pin */
  GPIO_InitStruct.Pin = ButtonMinus_Pin|ButtonOk_Pin|ButtonPlus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Algoritmo de Zeller para calcular el día de la semana basado en la fecha.
 * @param d Día del mes (1-31).
 * @param m Mes del año (1-12).
 * @param a Año completo (por ejemplo, 2025).
 * @return Número del día de la semana (0=Sábado, 1=Domingo, ..., 6=Viernes).
 */
int dia_semana(int d, int m, int a)
{
    if (m < 3)
    {
        m += 12;
        a--;
    }
    int y = a % 100;
    int c = a / 100;
    int w = (d + (13 * (m + 1)) / 5 + y + y / 4 + c / 4 - 2 * c) % 7;
    return w;
}

/**
 * @brief Devuelve el nombre del día de la semana.
 * @param dia Número del día de la semana (0=Sábado, ..., 6=Viernes).
 * @return Nombre del día de la semana como cadena.
 */
char* nombre_dia(int dia)
{
    char *dias[] = {"Sabado", "Domingo", "Lunes", "Martes", "Miercoles", "Jueves", "Viernes"};
    return dias[dia];
}

/**
 * @brief Detecta la pulsación del botón OK y actualiza la variable btnpress.
 */
void selectOption(void)
{
    // Detener alarma si está sonando y se presiona cualquier botón
    if (alarmState != ALARM_IDLE)
    {
        if (HAL_GPIO_ReadPin(ButtonOk_GPIO_Port, ButtonOk_Pin) == GPIO_PIN_SET ||
            HAL_GPIO_ReadPin(ButtonPlus_GPIO_Port, ButtonPlus_Pin) == GPIO_PIN_SET ||
            HAL_GPIO_ReadPin(ButtonMinus_GPIO_Port, ButtonMinus_Pin) == GPIO_PIN_SET)
        {
            stopAlarmSequence();
            HAL_Delay(300); // Anti-rebote para evitar entrar al menú accidentalmente
            return; // Salir sin procesar más botones
        }
    }

    // Solo procesar el botón OK si no hay alarma sonando
    if (HAL_GPIO_ReadPin(ButtonOk_GPIO_Port, ButtonOk_Pin) == GPIO_PIN_SET)
    {
        HAL_Delay(500); // Anti-rebote.
        btnpress = true;
    }
}

/**
 * @brief Cambia entre opciones de un menú usando botones de incremento y decremento.
 * @param sizemenu Cantidad de opciones en el menú actual.
 * @return true si se realizó un cambio, false en caso contrario.
 */
bool fnSwitch(uint8_t sizemenu)
{
	static uint32_t lastButtonTime = 0;
	const uint32_t debounceDelay = 250; // ms
	bool retorno = false;

	uint32_t now = HAL_GetTick();

	if ((HAL_GPIO_ReadPin(ButtonPlus_GPIO_Port, ButtonPlus_Pin) == GPIO_PIN_SET ||
		 HAL_GPIO_ReadPin(ButtonMinus_GPIO_Port, ButtonMinus_Pin) == GPIO_PIN_SET) &&
		(now - lastButtonTime > debounceDelay))
	{
		if (HAL_GPIO_ReadPin(ButtonPlus_GPIO_Port, ButtonPlus_Pin) == GPIO_PIN_SET)
		{
			contador++;
		}
		else if (HAL_GPIO_ReadPin(ButtonMinus_GPIO_Port, ButtonMinus_Pin) == GPIO_PIN_SET)
		{
			contador--;
		}

		// Ajustar el contador al rango permitido con wrap-around.
		if (contador < 0)
			contador = sizemenu - 1;
		if (contador >= sizemenu)
			contador = 0;

		lastButtonTime = now;
		retorno = true;
	}
	return retorno;
}

/**
 * @brief Muestra las opciones de un menú en la pantalla LCD, con la opción seleccionada indicada.
 * @param pos Posición actual en el menú.
 * @param menus Arreglo de cadenas con las opciones del menú.
 * @param sizemenu Número total de opciones en el menú.
 */
void fn_menu(int pos, char *menus[], uint8_t sizemenu)
{
    HD44780_Clear();

    // Resetear líneas del LCD.
    strcpy(linea1, "");
    strcpy(linea2, "");
    strcpy(linea3, "");
    strcpy(linea4, "");

    // Asignar opciones a las líneas según la posición actual.
    if ((pos % 4) == 0)
    {
        HD44780_SetCursor(0, 0);
        HD44780_PrintSpecialChar(4); // Indicar opción seleccionada.
        strcpy(linea1, menus[pos]);
        if (pos + 1 < sizemenu) strcpy(linea2, menus[pos + 1]);
        if (pos + 2 < sizemenu) strcpy(linea3, menus[pos + 2]);
        if (pos + 3 < sizemenu) strcpy(linea4, menus[pos + 3]);
    }
    else if ((pos % 4) == 1)
    {
        strcpy(linea1, menus[pos - 1]);
        HD44780_SetCursor(0, 1);
        HD44780_PrintSpecialChar(4);
        strcpy(linea2, menus[pos]);
        if (pos + 1 < sizemenu) strcpy(linea3, menus[pos + 1]);
        if (pos + 2 < sizemenu) strcpy(linea4, menus[pos + 2]);
    }
    else if ((pos % 4) == 2)
    {
        strcpy(linea1, menus[pos - 2]);
        strcpy(linea2, menus[pos - 1]);
        HD44780_SetCursor(0, 2);
        HD44780_PrintSpecialChar(4);
        strcpy(linea3, menus[pos]);
        if (pos + 1 < sizemenu) strcpy(linea4, menus[pos + 1]);
    }
    else
    {
        strcpy(linea1, menus[pos - 3]);
        strcpy(linea2, menus[pos - 2]);
        strcpy(linea3, menus[pos - 1]);
        HD44780_SetCursor(0, 3);
        HD44780_PrintSpecialChar(4);
        strcpy(linea4, menus[pos]);
    }

    // Imprimir las líneas en el LCD.
    HD44780_SetCursor(1, 0);
    HD44780_PrintStr(linea1);
    HD44780_SetCursor(1, 1);
    HD44780_PrintStr(linea2);
    HD44780_SetCursor(1, 2);
    HD44780_PrintStr(linea3);
    HD44780_SetCursor(1, 3);
    HD44780_PrintStr(linea4);
}

/**
 * @brief Imprime el valor actual de una variable en la pantalla LCD.
 * @param label Etiqueta descriptiva del valor.
 * @param value Valor numérico a mostrar.
 */
void printCurrentValue(const char* label, int value)
{
	char buffer[21];
	char truncatedLabel[11]; // Permitir hasta 10 caracteres para la etiqueta
	strncpy(truncatedLabel, label, 10);
	truncatedLabel[10] = '\0';
	sprintf(buffer, "%s: %02d", truncatedLabel, value);

	// Calcular la columna de inicio para que la cadena quepa en 20 caracteres
	int startCol = 20 - strlen(buffer);
	if (startCol < 0) startCol = 0;

	HD44780_SetCursor(startCol, 3);
	HD44780_PrintStr(buffer);
}

/**
 * @brief Calcula el valor del prescaler para obtener la frecuencia deseada.
 * @param frequency Frecuencia deseada en Hz.
 * @return Valor del prescaler calculado.
 */
int presForFrequency(int frequency)
{
    if (frequency == 0) return 0;
    return ((TIM_FREQ / (1000 * frequency)) - 1); // Fórmula para prescaler.
}

/**
 * @brief Alterna entre encender y apagar el retroiluminado del LCD.
 */
void Toggle_Backlight(void)
{
    if (backlightState)
    {
        HD44780_NoBacklight();
        backlightState = 0;
    }
    else
    {
        HD44780_Backlight();
        backlightState = 1;
    }
}
/**
 * @brief Ajusta la hora de la alarma con wrap-around.
 * @param hour Hora actual.
 * @param delta +1 para incrementar, -1 para decrementar.
 * @return Nueva hora ajustada.
 */
int adjustHour(int hour, int delta)
{
	hour += delta;
	if (hour > 23)
		hour = 0;
	else if (hour < 0)
		hour = 23;
	return hour;
}

void handleAlarmSystem(void)
{
    uint32_t currentTime = HAL_GetTick();

    // Verificar si debe activarse la alarma
    if (Alarma == true && !alarmTriggered)
    {
        if ((alarma.hour == time.hour) && (alarma.minutes == time.minutes) && (alarma.seconds == time.seconds))
        {
            startAlarmSequence();
        }
    }

    // Manejar los estados de la alarma
    switch (alarmState)
    {
        case ALARM_IDLE:
            // No hacer nada, esperar activación
            break;

        case ALARM_RINGING:
        case ALARM_TONE1:
        case ALARM_TONE2:
        case ALARM_PAUSE:
            // Verificar si ha pasado el tiempo total de la alarma
            if (currentTime - alarmStartTime >= alarmTotalDuration)
            {
                stopAlarmSequence();
            }
            else
            {
                // Calcular en qué parte del ciclo estamos
                uint32_t cycleTime = (currentTime - alarmStartTime) % alarmCycleDuration;

                if (cycleTime < alarmToneDuration)
                {
                    // Primer tono
                    if (alarmState != ALARM_TONE1)
                    {
                        __HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(Tone1));
                        alarmState = ALARM_TONE1;
                    }
                }
                else if (cycleTime < (alarmToneDuration * 2))
                {
                    // Segundo tono
                    if (alarmState != ALARM_TONE2)
                    {
                        __HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(Tone2));
                        alarmState = ALARM_TONE2;
                    }
                }
                else
                {
                    // Pausa (sin sonido)
                    if (alarmState != ALARM_PAUSE)
                    {
                        __HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(0));
                        alarmState = ALARM_PAUSE;
                    }
                }
            }
            break;
    }

    // Reset del trigger de alarma cuando cambie el segundo
    if (time.seconds != lastAlarmSecond)
    {
        if (alarmState == ALARM_IDLE)
        {
            alarmTriggered = false;
        }
        lastAlarmSecond = time.seconds;
    }
}

/**
 * @brief Inicia la secuencia de alarma.
 */
void startAlarmSequence(void)
{
    alarmState = ALARM_RINGING;
    alarmStartTime = HAL_GetTick();
    alarmTriggered = true;
}

/**
 * @brief Detiene la alarma y resetea el estado.
 */
void stopAlarmSequence(void)
{
    alarmState = ALARM_IDLE;
    alarmTriggered = false;
    __HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(0)); // Apagar sonido
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
