#  Blink (Hola Mundo)

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)

## Introducción
Este proyecto implementa un programa básico de blink (parpadeo de un LED), también conocido como el "Hola Mundo" de los microcontroladores, utilizando el LED integrado en la placa de desarrollo STM32F411CEU6 (Black Pill).

El objetivo de este tutorial es guiar paso a paso en la configuración del entorno de desarrollo, la generación del código base con STM32CubeIDE y la implementación del blink utilizando diferentes funciones de la capa de abstracción de hardware (HAL).

## Características Principales
- ✅ Configuración del reloj HSE para máxima velocidad.
- ✅ Habilitación de debug por Serial Wire.
- ✅ Configuración de GPIO para controlar el LED.
- ✅ Explicación de funciones HAL como `HAL_GPIO_WritePin()`, `HAL_GPIO_TogglePin()` y uso de `GPIO_PIN_SET`/`GPIO_PIN_RESET`.

## Componentes Requeridos
- STM32CubeIDE (descargable desde [STMicroelectronics](https://www.st.com/en/development-tools/stm32cubeide.html))
- Placa STM32F411CEU6 (Black Pill) o compatible.
- ST-Link V2 (programador/depurador).
- Fuente de alimentación de 5V.

# Configuración del Proyecto en STM32CubeIDE
## 1. Crear un Nuevo Proyecto
  ![Crear archivo](/PerifericosBasicos/HelloWorld/images/CrearArchivo.png)

  - Abre STM32CubeIDE y ve a `File` > `New` > `STM32 Project`.
  - En la ventana emergente, busca el microcontrolador por su parte comercial: __STM32F411CEU6__.
  - Selecciónalo y haz clic en `Next`.

  ![MPUSelector](/PerifericosBasicos/HelloWorld/images/MPUSelector.png)

## 2. Configurar el Proyecto
  - Asigna un nombre al proyecto (ej: `HolaMundo`). Evita usar espacios.
  - Selecciona `C` como lenguaje, `Executable` como tipo de binario y `STM32Cube` como tipo de proyecto.
  - Haz clic en `Next`.

  ![Seleccionar MPU](/PerifericosBasicos/HelloWorld/images/STM32Project.png)

  >[!Note]
  >⚠️ Si es la primera vez que usas este microcontrolador, se te pedirá aceptar el acuerdo de licencia.
  >![LicensingAgreement](/PerifericosBasicos/HelloWorld/images/LicensingAgreement.png)

## 3. Configurar el Reloj (HSE)
   
  Desplegar el menú `System Core` > `RCC` > `High Speed Clock (HSE)` pasar de `Disable` a `Crystal/Ceramic Resonator`, para poder usar la velocidad máxima disponible del reloj.


## 4. Habilitar Debug por Serial Wire
   
  Desplegar `System Core` > `SYS` > `Debug` y pasar de `Disable` a `Serial Wire`. Esto permite usar el ST-Link V2 para programar y depurar.

## 5. Configurar la Velocidad del Reloj
   
  - En el menú superior, seleccionar `Clock Configuration` y cambiar las velocidades del reloj.
  - Selecciona HSE en lugar de HSI como fuente del reloj.
  ![HSE](/PerifericosBasicos/HelloWorld/images/HSE.png)
  - Configura HCLK a la velocidad máxima permitida (100 MHz para el STM32F411).
  ![HCLK](/PerifericosBasicos/HelloWorld/images/HCLK.png)

## 6. Configurar el Pin del LED
  - Regresa a `Pinout & Configuration`.
  - Busca el pin `PC13` (conectado al LED integrado en la Black Pill).
  - Selecciona `GPIO_Output` en el menú desplegable.

  ![GPIO_Output](/PerifericosBasicos/HelloWorld/images/GPIO_Output.png)

## 7. Generar el Código
  - Haz clic en el icono de la llave de la tuerca amarilla (`Device Configuration Tool Code Generation`) para generar el código base.

  ![Code_Generation](/PerifericosBasicos/HelloWorld/images/CodeGeneration.png)

# Estructura del Código y Buenas Prácticas

STM32CubeIDE genera código modular con secciones delimitadas por comentarios como `USER CODE BEGIN` y `USER CODE END`. 

Las secciones más importantes son:

- `USER CODE BEGIN Includes` → Para incluir librerías.
- `USER CODE BEGIN PD` → Para definiciones.
- `USER CODE BEGIN PV` → Para variables privadas.
- `USER CODE BEGIN PFP` → Para prototipos de funciones.
- `USER CODE BEGIN WHILE` → Para el loop principal.

>[!Warning]
>Todo el código que se agregue debe estar dentro de estas secciones, ya que el código fuera de ellas puede ser eliminado si regeneras la configuración desde el archivo `.ioc` nuevamente.

# Implementación del Blink

El código para el parpadeo del LED debe ubicarse dentro del loop principal:

```C
/* USER CODE BEGIN WHILE */
while (1)
{
    // Aquí debe de ir el código a ciclar
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
```

## 1. Usando `HAL_GPIO_WritePin()`
Esta función permite escribir un estado específico en un pin:

Cuyos parametros son:
```C
HAL_GPIO_WritePin(
    GPIOx,       // Puerto GPIO (ej: GPIOC)
    GPIO_Pin,    // Número de pin (ej: GPIO_PIN_13)
    PinState     // Estado: 0 o 1
);
```
Ejemplo de código:
```C
while (1)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);  // Encender LED
    HAL_Delay(1000);                           // Esperar 1 segundo
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);  // Apagar LED
    HAL_Delay(1000);                           // Esperar 1 segundo
}
```

EL IDE permite autocompletado usando `CTRL + space`

>[!Note]
> 💡 Nota: El LED integrado en la Black Pill se enciende con 0 y se apaga con 1. Esto se debe a que está conectado en configuración de sumidero de corriente (sink).

## 2. Usando `GPIO_PIN_SET` y `GPIO_PIN_RESET`.

Para mejorar la legibilidad, se pueden usar las constantes `GPIO_PIN_SET` y `GPIO_PIN_RESET`:

```C
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  // Encender
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);    // Apagar
    HAL_Delay(1000); 
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
```

## 3. Usando `HAL_GPIO_Toggle()`.

La función `HAL_GPIO_Toggle()` describe un cambio de estado. Configura los pines en alto si estaban en bajo y viceversa.

Los parametros que reciben:

```C
HAL_GPIO_TogglePin(
    GPIOx,      // Puerto GPIO
    GPIO_Pin    // Número del pin
)
``` 

Ejemplo de código:

```C
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  // Cambiar estado
    HAL_Delay(1000);                         // Esperar 1 segundo 
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
```

## Cargar el Programa en la Placa
- Conectar la placa al computador usando el ST-Link V2.
- Hacer clic en el icono de play verde (Run) en la barra superior.

![Flechita](/PerifericosBasicos/HelloWorld/images/FlecaVerde.png)

- En la ventana emergente, seleccionar `Ok` para guardar el archivo.
  
![SaveAndLaunch](/PerifericosBasicos/HelloWorld/images/SaveLaunch.png)

- Posteriormente en la nueva ventana emergente, selecciona `Ok` para confirmar la configuración de debug.

![SaveAndLaunch](/PerifericosBasicos/HelloWorld/images/EditConfig.png)

El programa se compilará y cargará en la placa. ¡El LED debería empezar a parpadear!

>[!Note]
> ⚠️ Nota: Algunas placas clónicas pueden requerir tools externos como STM32CubeProgrammer para cargar el firmware.