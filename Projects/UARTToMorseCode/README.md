# Traductor de UART a Código Morse

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)
[![UART](https://img.shields.io/badge/Interface-UART%20Terminal-green)](https://es.wikipedia.org/wiki/Universal_Asynchronous_Receiver-Transmitter)

## Introducción
Este proyecto implementa un **sistema completo de traducción y transmisión de código Morse** para microcontroladores STM32. Permite recibir texto a través de UART, convertirlo a código Morse y transmitirlo tanto **visualmente** mediante un LED como **auditivamente** mediante un buzzer activo. El sistema incluye recepción por interrupciones y manejo de buffers.

---

## Características Principales

### Funcionalidades de Traducción
- ✅ **Traducción completa** de letras (A-Z) y números (0-9) a Morse
- ✅ **Conversión automática** a mayúsculas
- ✅ **Manejo de espacios** entre palabras
- ✅ **Temporizaciones precisas** según estándar Morse internacional
- ✅ **Tablas de búsqueda optimizadas** para acceso rápido

### Sistema de Comunicación
- ✅ **Recepción UART por interrupciones** (no bloqueante)
- ✅ **Echo en tiempo real** de caracteres recibidos
- ✅ **Soporte para backspace** para corrección de texto
- ✅ **Buffer circular** para manejo eficiente de datos
- ✅ **Detección automática** de fin de mensaje (ENTER)

### Sistema de Salida
- ✅ **Salida visual** mediante LED (GPIO)
- ✅ **Salida auditiva** mediante buzzer activo
- ✅ **Sincronización precisa** de puntos y rayas

### Características Técnicas
- ✅ **Arquitectura no bloqueante** con interrupciones
- ✅ **Gestión eficiente de memoria** con memset
- ✅ **Protección contra desbordamiento** de buffers
- ✅ **Temporizaciones configurables** fácilmente

## Tabla de Contenidos

- [Traductor de UART a Código Morse](#traductor-de-uart-a-código-morse)
  - [Introducción](#introducción)
  - [Características Principales](#características-principales)
    - [Funcionalidades de Traducción](#funcionalidades-de-traducción)
    - [Sistema de Comunicación](#sistema-de-comunicación)
    - [Sistema de Salida](#sistema-de-salida)
    - [Características Técnicas](#características-técnicas)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Requisitos del Sistema](#requisitos-del-sistema)
    - [Hardware Requerido](#hardware-requerido)
    - [Software Requerido](#software-requerido)
  - [Configuración de Hardware](#configuración-de-hardware)
  - [Configuración del Proyecto STM32CubeIDE](#configuración-del-proyecto-stm32cubeide)
    - [1. Creación del proyecto](#1-creación-del-proyecto)
    - [2. Configuración de Pines GPIO](#2-configuración-de-pines-gpio)
    - [3. Configuración UART](#3-configuración-uart)
  - [Estructura del Código](#estructura-del-código)
    - [Arquitectura principal](#arquitectura-principal)
    - [Módulos principales](#módulos-principales)
    - [Concepto Fundamental](#concepto-fundamental)
    - [Análisis de la resta `c - 'A'` dentro la llamada a función `flashSequence`](#análisis-de-la-resta-c---a-dentro-la-llamada-a-función-flashsequence)
    - [Ventajas de este método](#ventajas-de-este-método)
      - [Alternativas menos eficientes](#alternativas-menos-eficientes)
  - [Tablas de Código Morse](#tablas-de-código-morse)
    - [Temporizaciones Morse](#temporizaciones-morse)
  - [Flujo de Operación](#flujo-de-operación)
    - [1. Inicialización](#1-inicialización)
    - [2. Recepción de Datos](#2-recepción-de-datos)
    - [3. Procesamiento](#3-procesamiento)
    - [4. Finalización](#4-finalización)
  - [Uso del Sistema](#uso-del-sistema)
    - [Secuencia de Operación Normal](#secuencia-de-operación-normal)
    - [Ejemplo de Sesión](#ejemplo-de-sesión)
    - [Caracteres Soportados](#caracteres-soportados)
  - [Licencia](#licencia)

---

## Requisitos del Sistema
### Hardware Requerido
- **STM32CubeIDE** (última versión recomendada)
- **Placa STM32F103C6T8** (Blue Pill) o compatible
- **ST-Link V2** (programador/depurador)
- **Convertidor TTL a USB** (CP2102, CH340, FT232, etc.)
- **Protoboard y cables dupont**
- **Fuente de alimentación 5V**
- **Buzzer Activo**
- **Resistencias** (100Ω para buzzer)

### Software Requerido
- **Terminal serial** (Putty, Tera Term, CoolTerm)
- **Driver del convertidor USB-Serial** instalado

---

## Configuración de Hardware

| Comunicación Serial | STM32F103 | TTL-USB |
|---------------------|-----------|---------|
| TX                  | PA9       | RX      |
| RX                  | PA10      | TX      |
| GND                 | GND       | GND     |

| Componente | STM32F103 | Conexión | Notas |
|------------|-----------|----------|-------|
| LED        | PB2       | Ánodo | Resistencia 220Ω en serie |
| Cátodo LED | GND       | - | Conexión a tierra |
| Buzzer (+) | PB7       | Positivo | Resistencia 100Ω en serie |
| Buzzer (-) | GND       | Negativo | Conexión a tierra |

## Configuración del Proyecto STM32CubeIDE

### 1. Creación del proyecto

Siga los pasos del tutorial [UART](/PerifericosBasicos/UART/README.md) para recepción por interrupción

### 2. Configuración de Pines GPIO

En `Pinout & Configuration` → **GPIO**:

| Pin | Configuración |
|-----|---------------|
| PB2 | GPIO_Output   |
| PB7 | GPIO_Output   |

### 3. Configuración UART
- **Periférico**: USART1
- **Modo**: Asynchronous
- **Baud Rate**: 115200
- **Word Length**: 8 Bits
- **Parity**: None
- **Stop Bits**: 1
- **USART1 global interrupt**: Enable

---

## Estructura del Código

### Arquitectura principal

```C
// Núcleo del sistema
int main(void)
{
  Inicialización();        // GPIO, UART, Buffers
  while(1)
  {
    if(newMessageReady)    // Verificar mensaje completo
    {
      EchoMensajeRecibido();     // Confirmar recepción
      TraduccionAMorse();        // Convertir y transmitir
      MensajeCompletado();       // Limpiar y preparar siguiente
    }
  }
}
```

### Módulos principales

1. Sistema de Recepción UART (`HAL_UART_RxCpltCallback`)
- Recepción por interrupciones no bloqueante
- Manejo de caracteres especiales (ENTER, backspace)
- Echo en tiempo real de tecleo
- Protección contra buffer overflow
  
2. Sistema de Traducción (`transmitString`, `transmitChar`)
- Conversión carácter por carácter
- Manejo de diferentes tipos (letras, números, espacios)
- Acceso a tablas de búsqueda optimizadas
  
3. Sistema de Transmisión Morse (`flashSequence`, `flashDotOrDash`)
- Generación de puntos y rayas
- Control preciso de temporizaciones
- Salida simultánea LED y buzzer

4. Gestión de buffers
    ```C
    // Inicialización segura de buffers
    memset(RxData, 0, BUFFER_SIZE);
    memset(FinalData, 0, BUFFER_SIZE);
    ```

La función `memset` (de `string.h`) se utiliza para limpieza segura de buffers:

Recibe:
```C
memset(
  void*  _Dst,   // Puntero al bloque de memoria
  int    _Val,   // Valor para llenar (0 para limpiar)
  size_t _Size   // Número de bytes a limpiar
);
```

### Concepto Fundamental
En C, los caracteres se almacenan como números según la tabla ASCII:
```text
Letras mayúsculas:
'A' = 65, 'B' = 66, 'C' = 67, ..., 'Z' = 90

Números:
'0' = 48, '1' = 49, '2' = 50, ..., '9' = 57
```

### Análisis de la resta `c - 'A'` dentro la llamada a función `flashSequence`
```C
flashSequence(letters[c - 'A']);
```
1. Supongamos que `c = 'C'`
   1. `'C'` en ASCII = 67
   2. `'A'` en ASCII = 65
   3. `c - 'A'` = 67 - 65 = 2

2. Acceso al array `letters`:
  ```C
  char* letters[] = {
    ".-",      // índice 0 → 'A'
    "-...",    // índice 1 → 'B'  
    "-.-.",    // índice 2 → 'C' ←
    "-..",     // índice 3 → 'D'
    // ...
  };
  ```
  1. `letters[2]` retorna `"-.-."` (código Morse de 'C')

De forma similar se realiza para:
```C
flashSequence(numbers[c - '0']);
```
1. Supongamos que `c = '5'`
   1. `'5'` en ASCII = 53
   2. `'0'` en ASCII = 48
   3. `c - '0'` = 53 - 48 = 5

2. Acceso al array `numbers`:
  ```C
  char* numbers[] = {
      "-----",   // índice 0 → '0'
      ".----",   // índice 1 → '1'
      "..---",   // índice 2 → '2'
      "...--",   // índice 3 → '3'
      "....-",   // índice 4 → '4'
      ".....",   // índice 5 → '5' ←
      // ...
  };
  ```
  1. `numbers[2]` retorna `"....."` (código Morse de '5')
  
### Ventajas de este método
- Eficiente: Conversión directa sin switch o múltiples if
- Rápido: Operación aritmética simple
- Compacto: Menos código que alternativas
- Mantenible: Fácil agregar nuevos caracteres
  
#### Alternativas menos eficientes
```C
// Menos eficiente con muchos if
if(c == 'A') flashSequence(".-");
else if(c == 'B') flashSequence("-...");
else if(c == 'C') flashSequence("-.-.");
// ... (26 líneas más)

// Switch también verboso
switch(c) {
    case 'A': flashSequence(".-"); break;
    case 'B': flashSequence("-..."); break;
    // ... (24 casos más!)
}
```

## Tablas de Código Morse

Letras (A-Z)
```C
char* letters[] = {
    ".-",   "-...", "-.-.", "-..",  ".",    // A-E
    "..-.", "--.",  "....", "..",   ".---", // F-J  
    "-.-",  ".-..", "--",   "-.",   "---",  // K-O
    ".--.", "--.-", ".-.",  "...",  "-",    // P-T
    "..-",  "...-", ".--",  "-..-", "-.--", // U-Y
    "--.."                                  // Z
};
```

Números(0-9)
```C
char* numbers[] = {
    "-----", ".----", "..---", "...--", "....-",  // 0-4
    ".....", "-....", "--...", "---..", "----."   // 5-9
};
```

### Temporizaciones Morse
El sistema utiliza temporizaciones estándar según las especificaciones internacionales:
| Elemento | Duración | Relación | Valor |
|----------|----------|----------|-------|
|**Punto** (dot) | 1 unidad | Base     | 200 ms |
|**Raya** (dash) | 3 unidades | 3 x punto     | 600 ms |
|**Espacio entre símbolos** | 1 unidad | 1 x punto     | 200 ms |
| **Espacio entre letras** | 3 unidades | 3 x punto     | 600 ms |
|**Espacio entre palabras** | 7 unidades | 7 x punto     | 1400 ms |

```C
#define DOT 200                   // Punto: 200 ms
#define DASH (DOT * 3)            // Raya: 600 ms  
#define SYMBOL_SPACE DOT          // Entre símbolos: 200 ms
#define LETTER_SPACE (DOT * 3)    // Entre letras: 600 ms
#define WORD_SPACE (DOT * 7)      // Entre palabras: 1400 ms
```

## Flujo de Operación

### 1. Inicialización
```text
Power On → Inicializar GPIO → Configurar UART → 
Limpiar buffers → Mensaje bienvenida → Esperar datos
```

### 2. Recepción de Datos
```text
Carácter recibido → Interrupción UART → 
Almacenar en buffer → Echo en terminal → 
¿ENTER? → Marcar mensaje listo
```

### 3. Procesamiento
```text
Mensaje listo → Copiar a buffer final → 
Echo confirmación → Traducir a Morse → 
Transmitir por LED/buzzer
```

### 4. Finalización
```text
Transmisión completa → Mensaje "hecho" → 
Limpiar buffers → Reiniciar banderas → 
Esperar siguiente mensaje
```

## Uso del Sistema
### Secuencia de Operación Normal
1. Conectar y Alimentar el sistema
2. Abrir terminal serial a 115200 baudios
3. Ver mensaje de bienvenida
   ```text
   "Traductor Morse listo. Envie texto terminado en ENTER."
   ```
4. Escribir texto en terminal
5. Usar backspace para correcciones (tecla Backspace o Delete)
6. Presionar ENTER para iniciar transmisión Morse
7. Observar LED y escuchar buzzer
8. Ver confirmación en terminal cuando termine

### Ejemplo de Sesión
```text
Traductor Morse listo. Envie texto terminado en ENTER.
HOLA MUNDO[BACKSPACE][BACKSPACE]DO⏎

Transmitiendo: HOLA MUNDO

Transmisión completada.
```

### Caracteres Soportados
- **Letras**: A-Z, a - z (convertidas a mayúsculas)
- **Números**: 0 - 9
- **Espacios**: Entre palabras
- **Backspace/Delete**: Para corrección

## Licencia
Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.