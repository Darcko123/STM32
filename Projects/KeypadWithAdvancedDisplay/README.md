# Teclado Matricial con Interfaz Avanzada en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)
[![UART](https://img.shields.io/badge/Interface-UART%20Terminal-green)](https://es.wikipedia.org/wiki/Universal_Asynchronous_Receiver-Transmitter)

## Introducción
Este proyecto implementa un **sistema avanzado de monitorización de teclado matricial 4x4** para microcontroladores STM32, que incluye una **interfaz de terminal enriquecida** con visualización en tiempo real, historial de teclas y estadísticas completas. Utiliza **secuencias de escape ANSI** para crear una experiencia de usuario profesional en terminales seriales.

---

## Características Principales

### 🎯 Funcionalidades del Sistema
- ✅ **Visualización en tiempo real** de teclas presionadas
- ✅ **Interfaz gráfica en terminal** con bordes y colores
- ✅ **Historial circular** de las últimas 10 teclas presionadas
- ✅ **Contador de estadísticas** (total de teclas presionadas)
- ✅ **Mapa de referencia** del teclado matricial
- ✅ **Gestión avanzada de rebotes** (debounce) y prevención de repetición
- ✅ **Actualización dinámica** del display cada 100ms
- 
### 🎨 Características de Visualización
- **Codificación por colores** para diferentes secciones
- **Bordes Unicode** para una interfaz atractiva
- **Texto en negrita y efectos** para énfasis visual
- **Cursor oculto** para una experiencia más limpia
- **Actualizaciones parciales** sin parpadeo excesivo

### ⚡ Rendimiento y Eficiencia
- **Escaneo optimizado** del teclado matricial
- **Gestión eficiente de memoria** con buffers circulares
- **Temporizaciones configurables** para diferentes necesidades
- **Bajo consumo de CPU** con delays estratégicos

---

## Tabla de Contenidos
- [Teclado Matricial con Interfaz Avanzada en STM32](#teclado-matricial-con-interfaz-avanzada-en-stm32)
  - [Introducción](#introducción)
  - [Características Principales](#características-principales)
    - [🎯 Funcionalidades del Sistema](#-funcionalidades-del-sistema)
    - [🎨 Características de Visualización](#-características-de-visualización)
    - [⚡ Rendimiento y Eficiencia](#-rendimiento-y-eficiencia)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Requisitos del Sistema](#requisitos-del-sistema)
    - [Hardware Requerido](#hardware-requerido)
    - [Software Requerido](#software-requerido)
  - [Configuración de Hardware](#configuración-de-hardware)
    - [Diagrama de Conexiones](#diagrama-de-conexiones)
    - [Layout del Teclado Matricial](#layout-del-teclado-matricial)
  - [Configuración del Proyecto STM32CubeIDE](#configuración-del-proyecto-stm32cubeide)
    - [1. Creación del Proyecto](#1-creación-del-proyecto)
    - [2. Configuración de Pines GPIO](#2-configuración-de-pines-gpio)
    - [3. Configuración UART](#3-configuración-uart)
  - [Estructura del Código](#estructura-del-código)
    - [Arquitectura Principal](#arquitectura-principal)
    - [Modulos principales](#modulos-principales)
  - [Funcionamiento del sistema](#funcionamiento-del-sistema)
    - [Flujo de Ejecución](#flujo-de-ejecución)
    - [Parametros de temporización configurables](#parametros-de-temporización-configurables)
  - [Salida en Terminal](#salida-en-terminal)
    - [Estructura de la terminal](#estructura-de-la-terminal)
    - [Colores y formato](#colores-y-formato)
    - [Resultado](#resultado)
  - [Personalización](#personalización)
    - [Modificación del Mapeo de Teclas](#modificación-del-mapeo-de-teclas)
    - [Personalización de colores](#personalización-de-colores)
  - [Ejemplos de uso](#ejemplos-de-uso)
    - [Caso 1: Entrada de pin](#caso-1-entrada-de-pin)
    - [Caso 2: Navegación de menú](#caso-2-navegación-de-menú)
  - [Licencia](#licencia)

---

## Requisitos del Sistema

### Hardware Requerido
- **STM32CubeIDE** (última versión recomendada)
- **Placa STM32F103C6T8** (Blue Pill) o compatible
- **Teclado matricial 4x4**
- **ST-Link V2** (programador/depurador)
- **Convertidor TTL a USB** (CP2102, CH340, FT232, etc.)
- **Protoboard y cables dupont**
- **Fuente de alimentación 5V**

### Software Requerido
- **Terminal serial** que soporte ANSI (Putty, Tera Term, CoolTerm)
- **Driver del convertidor USB-Serial** instalado

---

## Configuración de Hardware

### Diagrama de Conexiones

| Teclado Matricial | STM32F103 | Función |
|-------------------|-----------|---------|
| Fila 1 (R1)       | PA7       | Salida  |
| Fila 2 (R2)       | PA6       | Salida  |
| Fila 3 (R3)       | PA5       | Salida  |
| Fila 4 (R4)       | PA4       | Salida  |
| Columna 1 (C1)    | PA3       | Entrada |
| Columna 2 (C2)    | PA2       | Entrada |
| Columna 3 (C3)    | PA1       | Entrada |
| Columna 4 (C4)    | PA0       | Entrada |

| Comunicación Serial | STM32F103 | TTL-USB |
|---------------------|-----------|---------|
| TX                  | PA9       | RX      |
| RX                  | PA10      | TX      |
| GND                 | GND       | GND     |

### Layout del Teclado Matricial
|  * | C1 | C2 | C3 | C4 |
|----|----|----|----|----|
| R1 | 1  | 2  | 3  | A  |
| R2 | 4  | 5  | 6  | B  |
| R3 | 7  | 8  | 9  | C  |
| R4 | *  | 0  | #  | D  |

## Configuración del Proyecto STM32CubeIDE

### 1. Creación del Proyecto
Siga los pasos del tutorial [UART](/PerifericosBasicos/UART/README.md) y [Teclado Matricial](/PerifericosBasicos/MatrixKeypad/README.md)

### 2. Configuración de Pines GPIO

En **Pinout & Configuration** → **GPIO**:

**Filas (Salidas - Push Pull):**
| Pin  | Etiqueta | Configuración |
|------|----------|---------------|
| PA7  | R1       | GPIO_Output   |
| PA6  | R2       | GPIO_Output   |
| PA5  | R3       | GPIO_Output   |
| PA4  | R4       | GPIO_Output   |

**Columnas (Entradas - Pull Up):**
| Pin  | Etiqueta | Configuración |
|------|----------|---------------|
| PA3  | C1       | GPIO_Input    |
| PA2  | C2       | GPIO_Input    |
| PA1  | C3       | GPIO_Input    |
| PA0  | C4       | GPIO_Input    |

### 3. Configuración UART
- **Periférico**: USART1
- **Modo**: Asynchronous
- **Baud Rate**: 115200
- **Word Length**: 8 Bits
- **Parity**: None
- **Stop Bits**: 1

---

## Estructura del Código

### Arquitectura Principal

```c
// Núcleo del sistema
int main(void)
{
    Inicialización();
    while (1) {
        EscaneoTeclado();
        ActualizaciónDisplay();
        GestiónTemporizaciones();
    }
}
```

### Modulos principales
1. Sistema de Escaneo (`Read_Keypad`)
 - Algoritmo de barrido por filas
 - Gestión de rebotes mecánicos
 - Prevención de repetición de teclas
 - Mapeo a caracteres ASCII

2. Sistema de Visualización (`Terminal_Update`)
 - Generación de interfaz gráfica
 - Manejo de secuencias ANSI
 - Actualización de estadísticas
 - Gestión de historial circular

3. Gestión de Tiempo
 - Temporizaciones configurables
 - Control de actualizaciones
 - Gestión de delays no bloqueantes 

---

## Funcionamiento del sistema
### Flujo de Ejecución
1. Inicialización
 - Configuración de GPIO y UART
 - Limpieza de la terminal
 - Dibujado de la interfaz inicial

2. Bucle Principal
 - Escaneo del teclado cada 50ms (DEBOUNCE_TIME)
 - Actualización del display cada 100ms (DISPLAY_UPDATE_MS)
 - Gestión del historial de teclas
 - Cálculo de estadísticas

3. Detección de Teclas
 - Activación secuencial de filas
 - Lectura simultánea de columnas
 - Validación contra rebotes
 - Procesamiento de teclas válidas

### Parametros de temporización configurables
```C
#define DEBOUNCE_TIME       50      // Tiempo anti-rebote
#define KEY_REPEAT_TIME     200     // Prevención de repetición
#define KEY_RELEASE_TIME    500     // Liberación de tecla
#define DISPLAY_UPDATE_MS   100     // Actualización de pantalla
#define HISTORY_SIZE        10      // Tamaño del historial
```

---

## Salida en Terminal
### Estructura de la terminal
``` text
┌───────────────────────────────────────────────┐
│           MONITOR TECLADO MATRICIAL 4x4       │
└───────────────────────────────────────────────┘

┌───────────────────────────────────────────────┐
│ TECLA ACTUAL: [X]                             │
└───────────────────────────────────────────────┘

┌───────────────────────────────────────────────┐
│ HISTORIAL DE TECLAS (Últimas 10 teclas):      │
│ [A] [B] [C] [1] [2] [3] [4] [5] [6] [7]       │
└───────────────────────────────────────────────┘

┌───────────────────────────────────────────────┐
│ ESTADÍSTICAS:                                 │
│ Total de Teclas Presionadas: 150              │
└───────────────────────────────────────────────┘

┌───────────────────────────────────────────────┐
│            Distribución de teclado            │
│             ┌────┬────┬────┬────┐             │
│             │ 1  │ 2  │ 3  │ A  │             │
│             ├────┼────┼────┼────┤             │
│             │ 4  │ 5  │ 6  │ B  │             │
│             ├────┼────┼────┼────┤             │
│             │ 7  │ 8  │ 9  │ C  │             │
│             ├────┼────┼────┼────┤             │
│             │ *  │ 0  │ #  │ D  │             │
│             └────┴────┴────┴────┘             │
└───────────────────────────────────────────────┘

  Estado: ACTIVO | Presiona cualquier tecla para probar...
```

### Colores y formato
- **Verde**: Tecla actual presionada
- **Azul**: Números del teclado
- **Rojo**: Letras y caracteres especiales
- **Amarillo**: Encabezados y títulos
- **Cian**: Historial de teclas
- **Magenta**: Estadísticas
- **Negrita**: Elementos importantes

### Resultado
![Result](/Projects/KeypadWithAdvancedDisplay/images/Result.png)

---

## Personalización
### Modificación del Mapeo de Teclas
```C
char keyMap[4][4] = {
    {'1', '2', '3', 'A'},   // Fila 1
    {'4', '5', '6', 'B'},   // Fila 2  
    {'7', '8', '9', 'C'},   // Fila 3
    {'*', '0', '#', 'D'}    // Fila 4
};
```

### Personalización de colores
```C
// Ejemplo: Cambiar color de números
#define COLOR_NUMBERS    COLOR_BRIGHT_GREEN
#define COLOR_LETTERS    COLOR_BRIGHT_MAGENTA
```

---

## Ejemplos de uso
### Caso 1: Entrada de pin
```C
// Ejemplo de implementación para entrada de PIN
char pin[4] = {0};
uint8_t pinIndex = 0;

if (currentKey >= '0' && currentKey <= '9') {
    pin[pinIndex++] = currentKey;
    if (pinIndex >= 4) {
        // PIN completo, procesar...
        ProcessPIN(pin);
        pinIndex = 0;
    }
}
```

### Caso 2: Navegación de menú
```C
// Ejemplo para control de menú
switch(currentKey) {
    case 'A': Menu_Up(); break;
    case 'B': Menu_Down(); break;
    case 'C': Menu_Select(); break;
    case 'D': Menu_Back(); break;
}
```

---

## Licencia
Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.