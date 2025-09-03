[file name]: README.md
# Botón en Modo Pull-Down

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)

## Introducción
Este proyecto implementa el control de LEDs mediante un botón físico conectado en configuración Pull-Down utilizando la placa de desarrollo `STM32F411CEU6` (Black Pill). 

El objetivo es demostrar cómo leer el estado de un pin digital configurado como entrada y cómo responder a eventos de botón con anti-rebote software, incluyendo detección de pulsaciones cortas y largas.

## Características Principales
- ✅ Configuración de GPIO como entrada con resistor Pull-Down interno
- ✅ Lectura del estado de un pin digital usando HAL
- ✅ Implementación de anti-rebote (debounce) software
- ✅ Control de LED mediante pulsador
- ✅ Detección de pulsaciones cortas y largas
- ✅ Uso del systick para medición precisa de tiempo

## Componentes Requeridos
- STM32CubeIDE
- Placa STM32F411CEU6 (Black Pill)
- ST-Link V2 (programador/depurador)
- Botón/pulsador
- Resistores (opcional, si no se usa Pull-Down interno)
- Protoboard y cables
- Fuente de alimentación de 5V

# Configuración del Proyecto en STM32CubeIDE

## 1. Crear un Nuevo Proyecto
Sigue los mismos pasos del tutorial [Blink](/PerifericosBasicos/HelloWorld/README.md) para crear un nuevo proyecto para el `STM32F411CEU6`.

## 2. Configurar el Pin del Botón (Modo Pull-Down)
- En `Pinout & Configuration`, busca el pin que usarás para el botón (ej: `PB10`)
- Selecciona `GPIO_Input` en el menú desplegable
- En la configuración del pin en `Pinout & Configuration` > `System Core` > `GPIO`, establece:
  - `GPIO Pull-up/Pull-down`: Pull-Down

![GPIO_Input_PullDown](/PerifericosBasicos/ButtonToggleLed/images/GPIO_Input_PullDown.png)

## 3. Configuraciones adicionales
- Configura el pin `PC13` como `GPIO_Output` (LED integrado)
- Configura el pin `PB12` como `GPIO_Output` (LED externo adicional)

## 4. Configuración del Reloj
Configurar el reloj de igual manera que en [Blink](/PerifericosBasicos/HelloWorld/README.md))

## 5. Generar el Código
Haz clic en el icono de la llave de la tuerca amarilla para generar el código base.
![Generar_Codigo](/PerifericosBasicos/HelloWorld/images/CodeGeneration.png)

## 6. Conexiones Físicas

![Conexion_Fisica](/PerifericosBasicos/ButtonToggleLed/images/ConexioFisica.png)

# Implementación del Control por Botón

## 1. Lectura Básica del Botón (Sin Anti-Rebote)
```C
/* USER CODE BEGIN WHILE */
while (1)
{
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(250); // Pequeño delay para evitar múltiples toggles
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
```

## 2. Implementación con Anti-Rebote Mejorado
Para un mejor control del rebote del botón, se recomienda esta implementación:

```C
/* USER CODE BEGIN PV */
uint32_t lastButtonPress = 0;
const uint32_t debounceDelay = 50; // 50ms para anti-rebote
/* USER CODE END PV */

/* USER CODE BEGIN WHILE */
while (1)
{
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET)
    {
        // Verificar anti-rebote
        if(HAL_GetTick() - lastButtonPress > debounceDelay)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            lastButtonPress = HAL_GetTick();
        }
    }
    
    // Pequeño delay para no saturar la CPU
    HAL_Delay(10);
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
```

## 3. Implementación con Detección de Flanco de Subida
Para una detección más precisa del momento en que se presiona el botón:
```C
/* USER CODE BEGIN PV */
uint8_t buttonState = 0;
uint8_t lastButtonState = 0;
/* USER CODE END PV */

/* USER CODE BEGIN WHILE */
while (1)
{
    buttonState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);

    // Detectar flanco de subida (cuando el botón se presiona)
    if(buttonState == GPIO_PIN_SET && lastButtonState == GPIO_PIN_RESET)
    {
        // Anti-rebote: verificar que el botón sigue presionado después del delay
        HAL_Delay(debounceDelay);
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
    }
    lastButtonState = buttonState;

    // Pequeño delay para reducir carga de CPU
    HAL_Delay(10);
    // =====================================================
/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
```

## 4. Implementación con Detección de Pulsación Larga
Para detectar pulsaciones cortas vs largas:
```C
/* USER CODE BEGIN PV */
uint8_t buttonState = 0;
uint8_t lastButtonState = 0;
uint32_t lastButtonPress = 0;
uint32_t buttonPressDuration = 0;
const uint32_t debounceDelay = 50; // 50ms para anti-rebote
const uint32_t longPressThreshold = 1000; // 1000ms = 1s para pulsación larga
/* USER CODE END PV */

/* USER CODE BEGIN WHILE */
while (1)
{
    // Leer el estado actual del botón
    buttonState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);

    // Detectar flanco de subida (botón presionado)
    if(buttonState == GPIO_PIN_SET && lastButtonState == GPIO_PIN_RESET)
    {
        // Registrar el momento en que se presiona el botón
        lastButtonPress = HAL_GetTick();
    }

    // Detectar flanco de bajada (botón liberado)
    if(buttonState == GPIO_PIN_RESET && lastButtonState == GPIO_PIN_SET)
    {
        // Calcular la duración de la pulsación
        buttonPressDuration = HAL_GetTick() - lastButtonPress;

        // Determinar si fue pulsación corta o larga
        if(buttonPressDuration < longPressThreshold)
        {
            // PULSACIÓN CORTA: Cambiar estado del LED integrado
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
        else
        {
            // PULSACIÓN LARGA: Cambiar estado del LED externo
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
        }
    }

    // Actualizar el estado anterior del botón
    lastButtonState = buttonState;

    // Pequeño delay para reducir carga de CPU
    HAL_Delay(10);
/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
```

# Función `HAL_GetTick()` - Sistema de Ticks
## ¿Qué es `HAL_GetTick()`?
`HAL_GetTick()` es una función proporcionada por HAL (Hardware Abstraction Layer) de STM32 que devuelve el número de milisegundos transcurridos desde que se inició el sistema.

## Características principales
- Precisión: Resolución de 1 milisegundo
- Base de tiempo: Utiliza el systick del Cortex-M
- Monótono: Siempre incrementa, nunca disminuye
- Wrap-around: Se reinicia después de ~49.7 días (valor máximo de uint32_t)

## Uso en Anti-Rebote y Mediciones de tiempo
```C
// Ejemplo: Medir duración de un evento
uint32_t startTime = HAL_GetTick();
// ... realizar alguna operación ...
uint32_t duration = HAL_GetTick() - startTime;

// Ejemplo: Anti-rebote
if(HAL_GetTick() - lastButtonPress > debounceDelay) {
    // El botón ha estado estable por suficiente tiempo
}
```

## Configuración del Systick
El systick se configura automáticamente en `SystemClock_Config()` con:
- Frecuencia de 1kHz (1ms por tick)
- Prioridad configurada apropiadamente
- Habilitado durante la inicialización de HAL

## Explicación del Funcionamiento
### Modo Pull-Down
En la configuración Pull-Down:

- El pin está conectado a tierra (GND) internamente através de un resistor
- Cuando el botón no está presionado, el pin lee LOW (0V)
- Cuando el botón está presionado, el pin se conecta a VCC y lee HIGH (3.3V)

### Detección de Pulsación Larga
El algoritmo de pulsación larga funciona así:
- Detección de presión: Cuando se detecta que el botón se presiona (flanco de subida), se guarda el timestamp
- Detección de liberación: Cuando se detecta que el botón se suelta (flanco de bajada), se calcula la duración
- Clasificación: Si la duración es menor al umbral → pulsación corta, si es mayor → pulsación larga

## Anti-Rebote (Debounce)
Los botones mecánicos generan múltiples transiciones rápidas (rebotes) al ser presionados. El anti-rebote software:
- Ignora cambios de estado que ocurren en menos de 50ms
- Garantiza que solo se detecte una pulsación por presionado real
- Mejora la confiabilidad de la detección

## Consideraciones Importantes
1. __Anti-Rebote__: Siempre implementa anti-rebote para botones mecánicos
2. __Consumo de CPU__: Evita delays largos en el loop principal
3. __Interrupciones__: Para aplicaciones más avanzadas, considera usar interrupciones GPIO
4. __Resistores Pull__: El STM32 tiene resistores Pull-Up y Pull-Down internos
5. __Precisión temporal__: `HAL_GetTick()` proporciona una base de tiempo confiable para mediciones
6. __Umbrales ajustables__: Los tiempos de debounce y pulsación larga pueden ajustarse según necesidades