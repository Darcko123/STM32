# Teclado matricial en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)


## Introducción
Este proyecto presenta una guía completa para la implementación de un **teclado matricial 4x4** en microcontroladores STM32, utilizando **STM32CubeIDE**.  
El objetivo es mostrar cómo detectar y procesar la pulsación de cada tecla del teclado, incluyendo la gestión de **rebotes mecánicos (debounce)** y la transmisión de datos vía UART.

---

## Funcionamiento de un Teclado Matricial
Un teclado matricial está conformado por una **malla de filas y columnas**.  
Cada tecla se encuentra en la intersección de una fila y una columna:

- Al presionar una tecla, se cierra el circuito entre una fila y una columna.
- Para identificar la tecla, el microcontrolador **activa secuencialmente cada fila** y **lee el estado de las columnas**.  
- Si una columna detecta un nivel lógico bajo mientras su fila correspondiente está activada, se determina qué tecla ha sido presionada.

Este método permite escanear eficientemente **16 teclas** (4 filas × 4 columnas) utilizando solo 8 pines de E/S.

**Anti-rebote (Debounce):**  
Cuando una tecla es presionada, los contactos mecánicos generan múltiples transiciones eléctricas en pocos milisegundos.  
Para evitar falsas detecciones, se implementa un retardo de lectura (`DEBOUNCE_TIME`) antes de aceptar una nueva pulsación.

---

## Características principales
- ✅ Lectura de un teclado matricial 4x4.
- ✅ Configuración de **GPIOs** de entrada con *Pull-Up interno*.
- ✅ Escaneo por filas para detectar teclas.
- ✅ Anti-rebote por software con temporización basada en `HAL_GetTick()`.
- ✅ Transmisión del carácter detectado vía **UART** a 115200 baudios.

## Componentes Requeridos
- STM32CubeIDE
- Placa STM32F103C6T8 (Blue Pill)
- ST-Link V2 (programador/depurador)
- Teclado matricial
- Protoboard y cables
- Fuente de alimentación de 5V
- TTL a USB

# Configuración del Proyecto STM32CubeIDE
## 1. Creación del proyecto
Sigue los mismos pasos del tutorial [Blink](/PerifericosBasicos/HelloWorld/README.md) para crear un nuevo proyecto para el `STM32F411CEU6`.

## 2. Configuración de los pines
En la pestaña **Pinout & Configuration**, configure:
- `PA0` a `PA3` como **GPIO_Input** (Columnas C4–C1).
- `PA4` a `PA7` como **GPIO_Output** (Filas R4–R1).
- Dar click derecho sobre cada uno de los pines y seleccionar `Enter User Label`. Ajustar de la siguiente manera
  
| Pin  | Etiqueta |
|------|---------|
| PA0  | C4 |
| PA1  | C3 |
| PA2  | C2 |
| PA3  | C1 |
| PA4  | R4 |
| PA5  | R3 |
| PA6  | R2 |
| PA7  | R1 |

- Active el **USART1** para transmisión (TX/RX) como se describe en [UART](/PerifericosBasicos/UART/README.md). 

El pinout quedaría de la siguiente manera:

![Pinout](/PerifericosBasicos/MatrixKeypad/images/Pinout.png)

- Ir a `System core` > `GPIO` > y en la sección `GPIO Pull-up/Pull-Down` cambiar todas las entradas a `Pull-up` cómo se ve en la siguiente imagen

![Pullup_Pines](/PerifericosBasicos/MatrixKeypad/images/PullUpPines.png)

## 3. Configuración del Reloj
Configurar el reloj de igual manera que en [Blink](/PerifericosBasicos/HelloWorld/README.md)

## 4. Generar el Código
Haz clic en el icono de la llave de la tuerca amarilla para generar el código base.

![Generar_Codigo](/PerifericosBasicos/HelloWorld/images/CodeGeneration.png)

## 5. Conexiones Físicas

![Conexion_Fisica](/PerifericosBasicos/MatrixKeypad/images/Conections.png)

Adicionando los pines para el UART:

| STM32F103 | TTL |
| --------- | --- |
| `PA9` TX  | RX  |
| `PA10` RX | TX  |
| `GND`     | GND |

# Implementación de la detección de click
## Estructura del código principal

El programa principal implementa un bucle infinito que continuamente escanea el teclado matricial y transmite los resultados por UART:

```c
while (1)
{
  // Escanea la matriz del teclado para detectar pulsaciones de teclas
	keyPressed = Read_Keypad();

  // Transmitir solo si se detectó una tecla válida (retorno distinto de cero)
  if(keyPressed != 0)
  {
      // Borrar la línea actual y mostrar la nueva tecla
      HAL_UART_Transmit(&huart1, clearLine, strlen((char*)clearLine), 1000);

      // Mostrar el prompt con la tecla presionada
      sprintf(buffer, "KEY = %c", keyPressed);

      // Limpiar la pantalla del terminal usando la secuencia de escape ANSI
      HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
  }else
  {
      // Cuando no hay tecla, mantener solo el prompt visible
      // Esto se hace automáticamente ya que no borramos la pantalla completa
      // Solo actualizamos si es necesario mantener consistencia
      static uint32_t lastNoKeyTime = 0;
      uint32_t currentTime = HAL_GetTick();

      // Actualizar el prompt vacío periódicamente (cada 2 segundos) para mantenerlo visible
      if ((currentTime - lastNoKeyTime) > 2000)
      {
          HAL_UART_Transmit(&huart1, homeCursor, strlen((char*)homeCursor), 1000);
          HAL_UART_Transmit(&huart1, showPrompt, strlen((char*)showPrompt), 1000);
          lastNoKeyTime = currentTime;
      }
  }

  // Pequeña pausa para evitar uso excesivo de CPU
	HAL_Delay(10);
}
```

## Función `Read_Keypad()` - Explicación detallada
### Configuración del mapeo de teclas

```c
char keyMap[4][4] =
{
  {'1', '2', '3', 'A'}, // Fila 1
  {'4', '5', '6', 'B'}, // Fila 2
  {'7', '8', '9', 'C'}, // Fila 3
  {'*', '0', '#', 'D'}  // Fila 4
};
```

### Algoritmo de escaneo
1. **Inicialización**: Todas las filas se configuran en estado HIGH
2. **Escaneo por filas**: Cada fila se activa secuencialmente (LOW)
3. **Lectura de columnas**: Se verifica el estado de cada columna
4. **Detección de tecla**: Si una columna está en LOW, se identifica la tecla presionada
5. **Gestión de rebotes**: Se implementan retardos para evitar lecturas falsas

### Parametros de temporización

```c
#define DEBOUNCE_TIME       50      // Tiempo mínimo entre lecturas de teclas
#define KEY_REPEAT_TIME     200     // Tiempo antes de permitir la repetición de la misma tecla
#define KEY_RELEASE_TIME    500     // Tiempo de espera antes de borrar la última tecla
```

### Lógica de prevención de repetición
El algoritmo incluye mecanismos sofisticados para:
- Evitar lecturas múltiples de la misma tecla (anti-rebote)
- Permitir la repetición solo después de un tiempo configurado
- Detectar cuando una tecla ha sido liberada

## Flujo de operación del teclado
1. **Inicialización del sistema**
   1. Configuración de GPIO para filas (salida) y columnas (entrada con pull-up)
   2. Inicialización de UART para comunicación serial
   3. Establecimiento de variables de estado
2. **Escaneo continuo**
   1. Activación secuencial de cada fila
   2. Lectura simultánea de todas las columnas
   3. Detección de cambios de estado
3. **Procesamiento de teclas**
   1. Validación contra rebotes mecánicos
   2. Mapeo a caracteres ASCII
   3. Transmisión por UART
   4. Limpieza de pantalla del terminal

# Resultados esperados
Al presionar cualquier tecla del keypad, en la terminal serial se mostrará:
```text
KEY = [carácter presionado]
```

Seguido de un borrado de pantalla después de 2s, preparando la terminal para la siguiente pulsación.

![Resultado](/PerifericosBasicos/MatrixKeypad/images/Result.png)

# Consideraciones importantes
- **Pull-up** resistors: Las columnas utilizan resistencias pull-up internas para asegurar un estado HIGH cuando no hay teclas presionadas
- **Velocidad de escaneo**: El algoritmo escanea todo el teclado en aproximadamente 4ms (excluyendo retardos de debounce)
- **Consumo de recursos**: Utiliza solo 8 pines GPIO y una instancia UART
- **Flexibilidad**: El mapeo de teclas puede modificarse fácilmente cambiando la matriz `keyMap`

# Licencia
Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.