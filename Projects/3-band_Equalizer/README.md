# Ecualizador de 3 bandas con STM32 y Filtros FIR

[![STM32](https://img.shields.io/badge/Platform-STM32F446-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![ADC](https://img.shields.io/badge/Protocol-ADC-yellow)](https://es.wikipedia.org/wiki/Convertidor_anal%C3%B3gico-digital)
[![DAC](https://img.shields.io/badge/Protocol-DAC-yellow)](https://es.wikipedia.org/wiki/Convertidor_digital-anal%C3%B3gico)
[![DMA](https://img.shields.io/badge/Protocol-DMA-yellow)](https://es.wikipedia.org/wiki/Acceso_directo_a_memoria)
[![TIM](https://img.shields.io/badge/Protocol-TIM-yellow)](https://es.wikipedia.org/wiki/Temporizador)
[![FIR](https://img.shields.io/badge/Filter-FIR-green)](https://en.wikipedia.org/wiki/Finite_impulse_response)


## Tabla de contenidos
- [Ecualizador de 3 bandas con STM32 y Filtros FIR](#ecualizador-de-3-bandas-con-stm32-y-filtros-fir)
  - [Tabla de contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Introducción Teórica](#introducción-teórica)
  - [Características Principales](#características-principales)
    - [Procesamiento de Señal](#procesamiento-de-señal)
    - [Arquitectura del Sistema](#arquitectura-del-sistema)
    - [Implementación y Rendimiento](#implementación-y-rendimiento)
  - [Requisitos del Sistema](#requisitos-del-sistema)
    - [Hardware Requerido](#hardware-requerido)
    - [Software Requerido](#software-requerido)
  - [Descripción Funcional del Sistema](#descripción-funcional-del-sistema)
    - [Diagrama de flujo del procesamiento general](#diagrama-de-flujo-del-procesamiento-general)
  - [Configuración del Hardware](#configuración-del-hardware)
    - [Configuración del ADC](#configuración-del-adc)
    - [Configuración del DAC](#configuración-del-dac)
    - [Configuración del Timer 8](#configuración-del-timer-8)
    - [Conexiones de Hardware](#conexiones-de-hardware)
  - [Incorporación de librerías DSP](#incorporación-de-librerías-dsp)
    - [Integración de Archivos en el Proyecto](#integración-de-archivos-en-el-proyecto)
    - [Configuración del Compilador (C/C++ Build)](#configuración-del-compilador-cc-build)
      - [1. Rutas de Inclusión (Include Paths)](#1-rutas-de-inclusión-include-paths)
      - [2. Configuración del Enlazador (Linker)](#2-configuración-del-enlazador-linker)
    - [Verificación de la Integración](#verificación-de-la-integración)
  - [Explicación de estructura básica para filtro FIR](#explicación-de-estructura-básica-para-filtro-fir)
    - [Definiciones](#definiciones)
    - [Variables globales privadas](#variables-globales-privadas)
      - [Buffers de almacenamiento de muestras](#buffers-de-almacenamiento-de-muestras)
      - [Buffers de entrada y salida del filtro FIR](#buffers-de-entrada-y-salida-del-filtro-fir)
      - [Instancia del filtro FIR](#instancia-del-filtro-fir)
      - [Buffer de estado del filtro](#buffer-de-estado-del-filtro)
    - [Inicialización del filtro FIR](#inicialización-del-filtro-fir)
    - [Callbacks](#callbacks)
      - [Callback de Mitad de Buffer](#callback-de-mitad-de-buffer)
      - [Callback de Buffer Completo](#callback-de-buffer-completo)
    - [Parámetros de la función `arm_fir_f32`](#parámetros-de-la-función-arm_fir_f32)
  - [Generación de Coeficientes para Filtros FIR](#generación-de-coeficientes-para-filtros-fir)
    - [Filtro Pasa-Bajos (Bajos)](#filtro-pasa-bajos-bajos)
    - [Filtro Pasa-Banda (Medios)](#filtro-pasa-banda-medios)
    - [Filtro Pasa-Altos (Agudos)](#filtro-pasa-altos-agudos)
    - [Notas de Exportación](#notas-de-exportación)
  - [Aplicación de Filtros en el Ecualizador](#aplicación-de-filtros-en-el-ecualizador)
    - [Integración de Coeficientes](#integración-de-coeficientes)
    - [Inicialización de Múltiples Filtros FIR](#inicialización-de-múltiples-filtros-fir)
    - [Procesamiento Paralelo y Mezcla de Señales](#procesamiento-paralelo-y-mezcla-de-señales)
    - [Implementación de Control de Ganancia](#implementación-de-control-de-ganancia)
      - [Conversión de Decibelios a Ganancia Lineal](#conversión-de-decibelios-a-ganancia-lineal)
      - [Aplicación de Ganancias con Librería CMSIS-DSP](#aplicación-de-ganancias-con-librería-cmsis-dsp)
      - [Parametros de la función `arm_scale_f32`](#parametros-de-la-función-arm_scale_f32)
  - [Protecciones implementadas](#protecciones-implementadas)
    - [Protección contra Saturación (Clipping)](#protección-contra-saturación-clipping)
    - [Mecanismo Anti-Race Conditions](#mecanismo-anti-race-conditions)
    - [Control de Rango de Ganancias](#control-de-rango-de-ganancias)
    - [Verificación de Instancias de Periféricos](#verificación-de-instancias-de-periféricos)
  - [Análisis de Carga Computacional](#análisis-de-carga-computacional)
    - [Estimación de Carga de CPU](#estimación-de-carga-de-cpu)
    - [Cálculo para Sistema Completo](#cálculo-para-sistema-completo)
    - [Análisis de Tiempos de Ejecución](#análisis-de-tiempos-de-ejecución)
    - [Métricas de rendimiento](#métricas-de-rendimiento)
  - [Capacidad de Expansión](#capacidad-de-expansión)
  - [Licencia](#licencia)

## Descripción
Este proyecto consiste en la implementación de un ecualizador de audio digital de 3 bandas (Bajos, Medios y Agudos) sobre una plataforma embebida STM32. El núcleo del procesamiento de señal se basa en filtros FIR (Finite Impulse Response), aplicados para modificar de forma independiente la ganancia de cada banda de frecuencia en tiempo real.

El flujo de señal sigue una arquitectura típica de procesamiento digital: la señal de audio analógica de entrada se captura mediante el ADC (Convertidor Analógico-Digital) del microcontrolador. Una vez convertida al dominio digital, la señal es procesada por la cascada de filtros FIR. El resultado final se reconstruye en una señal analógica a través del DAC (Convertidor Digital-Analógico).

Para garantizar un rendimiento eficiente y un procesamiento en tiempo real estricto, el sistema hace un uso extensivo del DMA (Acceso Directo a Memoria). Este mecanismo gestiona las transferencias de datos entre los periféricos (ADC/DAC) y la memoria, liberando al núcleo de la CPU de esta carga y minimizando la latencia. La sincronización precisa de todo el proceso (muestreo, procesamiento y reconstrucción) está coordinada por un temporizador (TIM), que actúa como reloj maestro del sistema.

---

## Introducción Teórica

Un **filtro FIR (Finite Impulse Response)** es un sistema digital cuya salida depende únicamente de los valores actuales y pasados de la señal de entrada, y no de valores previos de salida.  
Matemáticamente, su salida se define como:

$$
y[n] = \sum_{k=0}^{N-1} h[k] \, x[n-k]
$$

donde:  
- *h[k]* son los coeficientes del filtro (respuesta al impulso),  
- *x[n]* es la señal de entrada, y  
- *N* es el número de coeficientes o “taps” del filtro.

Los filtros FIR son ampliamente utilizados en ecualizadores digitales debido a su **estabilidad inherente** y **fase lineal**, características esenciales para preservar la fidelidad del audio.  
En este proyecto, se diseñan tres filtros FIR independientes para las bandas:

- **Bajos (Low-pass)**  
- **Medios (Band-pass)**  
- **Agudos (High-pass)**

Cada filtro atenúa o refuerza su rango de frecuencia correspondiente, y la suma ponderada de sus salidas conforma la señal ecualizada.

---

## Características Principales

### Procesamiento de Señal
- ✅ **Ecualización de 3 Bandas**: Control independiente sobre las bandas de Bajos, Medios y Agudos.
- ✅ **Filtros FIR Optimizados**: Implementación de filtros de 32 coeficientes, diseñados para un equilibrio óptimo entre rendimiento computacional y respuesta en frecuencia.
- ✅ **Frecuencia de Muestreo**: 40 kHz, garantizando el cumplimiento del teorema de Nyquist para señales de audio en el rango audible.
- ✅ **Procesamiento en Punto Flotante**: Utiliza la unidad de punto flotante (FPU) del Cortex-M4 para maximizar la precisión en los cálculos.
- ✅ **Fase Lineal**: Empleo de coeficientes simétricos en los filtros FIR para preservar la fase lineal, evitando distorsiones en la forma de onda.

### Arquitectura del Sistema
- ✅ **Adquisición No Bloqueante**: El ADC, activado por un Timer, se sirve del controlador DMA para transferir las muestras a la memoria. Esta implementación utiliza un buffer circular de tipo "Ping-Pong" con interrupciones en la mitad y al final de la transferencia, permitiendo el procesamiento simultáneo de un bloque de datos mientras se adquiere el siguiente.
- ✅ **Salida en Tiempo Real**: El DAC se actualiza mediante DMA, sincronizado por el mismo Timer, asegurando una conversión analógica continua y sin interrupciones. También utiliza callbacks para la gestión eficiente de los buffers de salida.
- ✅ **Sincronización de Precisión**: Un temporizador (TIM) central actúa como el reloj maestro del sistema, generando los eventos de disparo para el ADC y el DAC, lo que garantiza una frecuencia de muestreo constante y precisa.
- ✅ **Interfaz de Control Analógico**: La ganancia de cada banda se ajusta mediante potenciómetros externos conectados a entradas ADC adicionales.
- ✅ **Protección de Salida**: Implementación de un limitador (limiter) digital para prevenir la saturación y el recorte (clipping) de la señal.

### Implementación y Rendimiento
- ✅ Librerías CMSIS-DSP: Aprovecha las librerías de procesamiento de señal de ARM, altamente optimizadas para la arquitectura Cortex-M4.
- ✅ Manejo Eficiente de Memoria: Empleo de buffers circulares y alineación de datos para minimizar los ciclos de DMA y CPU.
- ✅ Procesamiento por Bloques: Los algoritmos se aplican sobre bloques de muestras, optimizando el uso de la cache y el pipeline del procesador.
- ✅ Diseño Modular: La arquitectura de software está organizada en módulos independientes (adquisición, filtros, salida), facilitando el mantenimiento y la extensibilidad del sistema.

## Requisitos del Sistema

### Hardware Requerido
- **STM32CubeIDE** (última versión recomendada)
- **Placa STM32F446RE** (Nucleo-F446RE o compatible)
- **ST-Link V2/V3** (programador/depurador)
- **Convertidor de Audio** (ADC/DAC externos o entrada de línea)
- **Potenciometros** (3 unidades para control de ganancia)
- **Amplificador de Audio** y altavoz
- **Protoboard y cables dupont**
- **Fuente de alimentación 3.3V/5V**

### Software Requerido
- **STM32CubeMX** para configuración de periféricos
- **ARM CMSIS-DSP** librerías para procesamiento de señal
- **Terminal serial** para monitorización (opcional)
- **MATLAB/Python** para diseño de filtros (opcional)

## Descripción Funcional del Sistema

El flujo general del sistema es el siguiente:

1. El **ADC** adquiere muestras de la señal de audio y las almacena en un **buffer circular** mediante **DMA**.  
2. Al completarse la mitad o totalidad del buffer, se activa una **callback** que indica al microcontrolador que el bloque de datos está listo para procesarse.  
3. El bloque de muestras se filtra mediante tres **filtros FIR** (bajos, medios y agudos).  
4. Las tres salidas filtradas se combinan ponderadamente según las ganancias definidas.  
5. La señal resultante se envía al periférico DAC o PWM (según implementación).

### Diagrama de flujo del procesamiento general

```text
        ┌─────────────────────┐
        │   Inicio del MCU    │
        └─────────┬───────────┘
                  │
           Configuración de:
       ┌─────────────────────┐
       │ ADC + DMA + TIM     │
       │ FIR (CMSIS-DSP)     │
       └──────────┬──────────┘
                  │
         ┌────────▼─────────┐
         │ DMA llena buffer │
         └────────┬─────────┘
                  │
        ┌─────────▼─────────┐
        │ Callback DMA      │
        │ (mitad/completo)  │
        └─────────┬─────────┘
                  │
        ┌─────────▼─────────┐
        │ Filtro FIR Bajo   │
        │ Filtro FIR Medio  │
        │ Filtro FIR Alto   │
        └─────────┬─────────┘
                  │
        ┌─────────▼─────────┐
        │ Suma ponderada    │
        │ (Ganancias)       │
        └─────────┬─────────┘
                  │
        ┌─────────▼─────────┐
        │ Salida procesada  │
        └───────────────────┘
```

---

## Configuración del Hardware
- Microcontrolador: STM32F446RE
### Configuración del ADC
|ADC Mode   | Configuration |
|-----------|----------------|
| ADC1_IN0 Enabled | Enabled | 

| DMA Settings | Configuration |
|--------------|----------------|
| ADD | ADC1 |
| Mode | Circular | 
| Memory Data Width | Word |


![DMA Configuration](/Projects/3-band_Equalizer/images/ADC_DMA_Configuration.png)

| ADC Parameter Settings | Configuration | 
|--------------|----------------|
| Continous Conversion Mode| Disabled |
| DMA Continous Request | Enabled |
| External Trigger Conversion Source | Timer 8 Trigger Out Event |


![ADC Configuration](/Projects/3-band_Equalizer/images/ADC_Configuration.png)

### Configuración del DAC

| DAC Mode   | Configuration |
|-----------|----------------|
| OUT1 | Enabled |

| DMA Settings | Configuration |
|--------------|----------------|
| ADD | DAC1 |
| Mode | Circular | 
| Memory Data Width | Word |

![DMA Configuration](/Projects/3-band_Equalizer/images/DAC_DMA_Configuration.png)

| DAC Parameter Settings | Configuration | 
|--------------|----------------|
| Output Buffer| Enable |
| DMA Continous Request | Enabled |
| External Trigger Conversion Source | Timer 8 Trigger Out Event |

![DAC Configuration](/Projects/3-band_Equalizer/images/DAC_Configuration.png)

### Configuración del Timer 8

La sincronización precisa del sistema se logra mediante la configuración del `Timer 8` (TIM8) como el reloj maestro. Este temporizador es responsable de generar la frecuencia de muestreo de 40 kHz, la cual gobierna tanto la adquisición de datos (ADC) como la conversión de salida (DAC).

A continuación, se detalla el procedimiento para el cálculo de los parámetros `Prescaler` y `Counter Period`.

1. La frecuencia de muestreo adecuada
> [!WARNING] 
> La selección de la frecuencia de muestreo está dictada por el Teorema de Nyquist. Este establece que la frecuencia de muestreo debe ser, como mínimo, el doble del ancho de banda de la señal a procesar. Dado que el sistema está diseñado para audio con componentes de hasta 20 kHz, la frecuencia de muestreo mínima requerida es de 40 kHz.

2. Reloj de Base de Tiempo (Clock Source)
   1. El Timer 8 está conectado al bus de alta velocidad APB2, el cual opera a una frecuencia de 180 MHz. Esta frecuencia $\(F_{clk})\$ sirve como la señal de reloj de entrada para el temporizador.

    ![Bus de datos](/Projects/3-band_Equalizer/images/BusDatos.png)

    ![Timer Clocks](/Projects/3-band_Equalizer/images/TimerClocks.png)

3. Cálculo de los Parámetros del Timer
   1. La frecuencia de salida del timer (la frecuencia de actualización) se calcula mediante la siguiente fórmula:
   
   $F_{out} = \frac{F_{clk}}{(Prescaler + 1) \times (Counter Period + 1)}$

      1. Donde:
         1. $F_{out}$ : Frecuencia de salida deseada (40 kHz).
         2. $F_{clk}$ : Frecuencia del reloj de entrada (180 MHz).
         3. `Prescaler` Valor del preescalador del timer (PSC).
         4. `Counter Period` Valor del registro de auto-reload (ARR)
   
      2. Procedimiento de Cálculo: podemos despejar el valor del `Prescaler`
         1. Se fija el valor de $F_{out}$ en 40 kHz y $F_{clk}$ en 180 MHz.
         2. Para simplificar el cálculo, se elige un valor arbitrario para `(Counter Period + 1) = 10`.
         3. Se despeja la ecuación para encontrar `(Prescaler + 1)`:
            $(Prescaler + 1) = \frac{F_{clk}}{F_{out} \times (Counter Period + 1)}$

         4. Sustituyendo los valores:       
            $\(Prescaler + 1) = \frac{180 MHz}{(40 kHz)(10)}\$

            `Prescaler + 1` = 450

4.  Configuración Final
   
   Los valores finales a cargar en los registros del Timer 8 son los siguientes:

   | Timer 8 Settings | Configuration |
   |------------------|---------------|
   | Prescaler | 450 - 1  |
   | Counter Period | 10 | 
   | Trigger Event Selection | Update Event |

Esta configuración genera una frecuencia de actualización exacta de 40 kHz, la cual se utiliza para disparar las conversiones del ADC y actualizar el DAC.

   ![Timer Configuration](/Projects/3-band_Equalizer/images/Timer_Configuration.png)

### Conexiones de Hardware

| Señal de Audio | STM32F446 | Conexión Externa |
|----------------|-----------|------------------|
| Entrada Audio  | PA0 (ADC1) | Fuente de audio |
| Salida Audio   | PA4 (DAC1) | Amplificador |

| Control de Ganancia | STM32F446 | Potenciómetro |
|---------------------|-----------|---------------|
| Bajos (Low)         | PA1 (ADC2) | Canal 1 |
| Medios (Mid)        | PA2 (ADC2) | Canal 2 |
| Agudos (High)       | PA3 (ADC2) | Canal 3 |

| Alimentación | STM32F446 | Fuente |
|--------------|-----------|--------|
| 3.3V         | 3V3       | 3.3V   |
| GND          | GND       | GND    |


## Incorporación de librerías DSP

Para la implementación de los filtros FIR, este proyecto utiliza las librerías de procesamiento de señal digital (DSP) optimizadas por ARM para microcontroladores Cortex-M. Los archivos necesarios ya se encuentran recopilados en el directorio [`DSP_Drivers`](/Projects/3-band_Equalizer/DSP_Drivers/) del repositorio.

### Integración de Archivos en el Proyecto
El primer paso consiste en integrar la carpeta `DSP_Drivers` en la estructura del proyecto de STM32CubeIDE. La ruta recomendada es la siguiente:

```text
ProjectName/
├── Includes/
│  
├── Core/
│
├── Drivers/        
│   ├── CMSIS/          <- Incorporar aquí
│   │   ├── Device
│   │   ├── DSP_Drivers <- Esta
│   │   └── Include
│   └── STM32Xxx_HAL_Driver/
...
```

### Configuración del Compilador (C/C++ Build)

Una vez copiados los archivos, es necesario configurar el entorno de compilación. En el menú superior, navegue a:
`Project` > `Properties` > `C/C++ Build` > `Settings`.

#### 1. Rutas de Inclusión (Include Paths)

En la pestaña `MCU GCC Compiler`, seleccione la opción `Include paths` y agregue una nueva ruta.

![IncludePath](/Projects/3-band_Equalizer/images/IncludePath.png)

Seleccione desde el workspace la carpeta `Include` ubicada dentro de `DSP_Drivers`.

![Workspace](/Projects/3-band_Equalizer/images/Workspace.png)

La ruta resultante debe ser similar a:
```text
${workspace_loc:/${ProjName}/Drivers/CMSIS/DSP_Drivers/Include}
```

#### 2. Configuración del Enlazador (Linker)

En la pestaña `MCU GCC Linker`:

- En la sección `Libraries`, agregue una nueva librería con el nombre:
  ```text
  arm_cortexM4lf_math
  ```
- En la sección `Library search path` (debajo de `Libraries`), agregue una nueva ruta. Navegue desde el workspace y seleccione la carpeta `GCC` dentro de `DSP_Drivers/Lib`.

La ruta resultante debe ser similar a:
  ```text
  ${workspace_loc:/${ProjName}/Drivers/CMSIS/DSP_Drivers/Lib/GCC}
  ```

La configuración final debe verse similar a la siguiente imagen:

![Libraries](/Projects/3-band_Equalizer/images/Libraries.png)

Finalice haciendo clic en `Apply and Close`.

### Verificación de la Integración

Para verificar que la integración se ha realizado correctamente, agregue la siguiente línea de inclusión en la sección de includes privados del archivo `main.c`:

```C
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */
```

Si al compilar y depurar (debug) el proyecto no se obtienen warnings o errores relacionados con la librería arm_math.h, la integración se ha realizado de manera exitosa.

## Explicación de estructura básica para filtro FIR

### Definiciones
```C
/* USER CODE BEGIN PD */
#define N 32
#define halfN N/2
#define NUM_TAPS 32
/* USER CODE END PD */
```

Estas definiciones establecen los parámetros fundamentales del filtro:

 - `N` y `NUM_TAPS`: Especifican el número de coeficientes (taps) del filtro FIR, que determina su selectividad en frecuencia y complejidad computacional. Para este proyecto se utilizan 32 taps.
 - `halfN`: Define la mitad del número de taps, útil para operaciones de procesamiento simétricas.
  
  >[!Note]
  >Se recomienda utilizar valores pares para el número de taps, ya que esto facilita la implementación de estructuras simétricas y optimizadas.

### Variables globales privadas
#### Buffers de almacenamiento de muestras
```C
uint32_t adc_buffer[N];
uint32_t dac_buffer[N];
```
Estos buffers gestionan el flujo de datos en el sistema:
- `adc_buffer`: Almacena las muestras adquiridas por el ADC antes de ser procesadas.
- `dac_buffer`: Contiene las muestras procesadas que serán enviadas al DAC para su conversión analógica.
  
>[!Important]
> Importante: El tamaño de ambos buffers debe coincidir con el número de taps del filtro (N).

#### Buffers de entrada y salida del filtro FIR
```C
float32_t filt_in[N];
float32_t filt_out[N];
float32_t *filt_in_ptr = &filt_in[0];
float32_t *filt_out_ptr = &filt_out[0];
```
Estos buffers facilitan el procesamiento en punto flotante:
- `filt_in`: Buffer de entrada que almacena las muestras convertidas a formato flotante para el procesamiento DSP.
- `filt_out`: Buffer de salida que recibe los resultados del filtrado.
- **Punteros asociados**: Proporcionan acceso eficiente a los buffers para las funciones de la librería DSP.

#### Instancia del filtro FIR
```C
arm_fir_instance_f32 filter1;
```
Esta variable representa la instancia del filtro FIR y contiene:
- La configuración del filtro (número de taps, coeficientes)
- Los estados internos necesarios para el procesamiento en bloques
- Sirve como handle para todas las operaciones de filtrado mediante las funciones CMSIS-DSP

#### Buffer de estado del filtro
```C
float32_t State[halfN + NUM_TAPS - 1];
```
Este buffer almacena el estado interno del filtro FIR, conservando las muestras anteriores necesarias para el cálculo de las salidas actuales. Su tamaño está determinado por la fórmula halfN + NUM_TAPS - 1, donde:

- `halfN` representa la mitad del número de taps
- `NUM_TAPS` es el número total de coeficientes del filtro

Esta dimensión específica permite el correcto funcionamiento de las funciones de procesamiento por bloques de la librería `CMSIS-DSP`, manteniendo la continuidad temporal entre bloques de muestras procesados secuencialmente.

Coefficients del filtro
```C
float32_t Coeffs[NUM_TAPS] = { . . .};
```
Este array contiene los coeficientes que definen completamente la respuesta en frecuencia del filtro FIR. Cada coeficiente determina cómo contribuyen las muestras pasadas y presentes a la salida actual.

Los coeficientes pueden generarse mediante diversas herramientas de diseño de filtros digitales:

- MATLAB con el Filter Design & Analysis Tool
- Python utilizando librerías como SciPy (scipy.signal.firwin)
- Herramientas online especializadas en diseño de filtros como [T-Filter](http://t-filter.engineerjs.com/)
- Software comercial como LabVIEW o Octave

>[!Note]
> En secciones posteriores de este documento se detallará el procedimiento específico para generar estos coeficientes utilizando diferentes metodologías.

### Inicialización del filtro FIR
```C
arm_fir_init_f32(&filter1, NUM_TAPS, &Coeffs[0], &State[0], halfN);
```
Esta función configura e inicializa la estructura del filtro FIR con los parámetros definidos. Los argumentos son:

- `&filter1`: Puntero a la instancia del filtro FIR
- `NUM_TAPS`: Número total de coeficientes del filtro
- `&Coeffs[0]`: Puntero al array que contiene los coeficientes del filtro
- `&State[0]`: Puntero al buffer de estado del filtro
- `halfN`: Tamaño del bloque de procesamiento (mitad del número de taps)

### Callbacks
#### Callback de Mitad de Buffer
```C
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	for(int n = 0; n<halfN; n++)
	{
		filt_in[n] = (float32_t)adc_buffer[n];
	}

	arm_fir_f32(&filter1, filt_in_ptr, filt_out_ptr, halfN);

	for(int n = 0; n<halfN; n++)
	{
		dac_buffer[n] = (uint32_t)filt_out[n];
	}
}
```

#### Callback de Buffer Completo
```C  
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	for(int n = halfN; n<N; n++)
	{
		filt_in[n] = (float32_t)adc_buffer[n];
	}

	arm_fir_f32(&filter1, filt_in_ptr + halfN, filt_out_ptr + halfN, halfN);

	for(int n = halfN; n<N; n++)
	{
		dac_buffer[n] = (uint32_t)filt_out[n];
	}
}
```
Estas funciones de callback se ejecutan automáticamente cuando el DMA completa la transferencia de la mitad del buffer (`ConvHalfCpltCallback`) y del buffer completo (`ConvCpltCallback`), respectivamente.

El flujo de procesamiento en ambos callbacks es idéntico:

1. **Conversión de formato**: Las muestras del ADC se convierten de `uint32_t` a `float32_t`
2. **Procesamiento FIR**: Se aplica el filtro al bloque de muestras
3. **Conversión de salida**: Las muestras procesadas se convierten de vuelta a `uint32_t` para el DAC
 
 ### Parámetros de la función `arm_fir_f32`

Los parametros de la función `arm_fir_f32` son:
```C
arm_fir_f32(
        const arm_fir_instance_f32 * S, 
        const float32_t * pSrc, 
        const float32_t * pDst, 
        uint32_t blockSize
        )
```
- `S`: Puntero a la instancia del filtro FIR previamente inicializada
- `pSrc`: Puntero al buffer de entrada que contiene las muestras a filtrar
- `pDst`: Puntero al buffer de salida donde se almacenarán las muestras filtradas
- `blockSize`: Número de muestras a procesar en esta ejecución (en este caso, `halfN`)

Esta implementación utiliza una arquitectura de **doble buffer (ping-pong)**, permitiendo el procesamiento simultáneo de un bloque de muestras mientras se adquiere el siguiente, garantizando así una operación en tiempo real sin pérdida de datos.

## Generación de Coeficientes para Filtros FIR

La implementación del ecualizador de 3 bandas requiere el diseño de tres filtros FIR distintos: un filtro **pasa-bajos**, un filtro **pasa-bandas** y un filtro **pasa-altos**. Existen diversas herramientas para generar estos coeficientes, como [T Filter](http://t-filter.engineerjs.com/) u Octave, pero en este proyecto se utiliza **MATLAB** con la herramienta **Filter Designer**.

### Filtro Pasa-Bajos (Bajos)

1. Abra **Filter Designer** en MATLAB
2. En `Response Type`, seleccione **`Lowpass`**
3. En `Design Method`, seleccione **`FIR`** y dentro de este, **`Window`**

> [!TIP]
> Basado en pruebas experimentales, el método de ventana con ventana **Kaiser** ofrece un equilibrio óptimo entre complejidad computacional y rendimiento en la respuesta frecuencial.

1. Establezca los siguientes parámetros:
   - **Specify Order**: `32`
   - **FS**: `40000` (Frecuencia de muestreo del sistema)
   - **Fc**: `200` (Frecuencia de corte en Hz)

**Respuesta en Frecuencia:**
![Lowpass](/Projects/3-band_Equalizer/images/ResponseLowPass.png)

Para exportar los coeficientes: `File` > `Export` > `Coefficients File ASCII`. Guarde el archivo como `CoeffsLowPass`.

### Filtro Pasa-Banda (Medios)

| Parámetro | Valor |
|-----------|-------|
| **Response Type** | Bandpass |
| **Design Method** | FIR - Window |
| **Window Type** | Blackman |
| **Order** | 32 |
| **FS** | 40000 Hz |
| **Fc1** | 300 Hz |
| **Fc2** | 4000 Hz |

**Respuesta en Frecuencia:**
![Bandpass](/Projects/3-band_Equalizer/images/ResponseBandPass.png)

### Filtro Pasa-Altos (Agudos)

| Parámetro | Valor |
|-----------|-------|
| **Response Type** | Highpass |
| **Design Method** | FIR - Window |
| **Window Type** | Hamming |
| **Order** | 32 |
| **FS** | 40000 Hz |
| **Fc** | 2000 Hz |

**Respuesta en Frecuencia:**
![Highpass](/Projects/3-band_Equalizer/images/ResponseHighPass.png)

### Notas de Exportación

Para cada filtro, repita el proceso de exportación mediante `File` > `Export` > `Coefficients File ASCII`, generando archivos separados (`CoeffsLowPass`, `CoeffsBandPass`, `CoeffsHighPass`). Estos archivos contienen los coeficientes en formato ASCII que posteriormente se integrarán en el código del microcontrolador.

## Aplicación de Filtros en el Ecualizador

### Integración de Coeficientes

Los coeficientes generados mediante MATLAB deben integrarse en el código como arreglos de constantes. Cada conjunto de coeficientes se define en un arreglo separado:

```C
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
		0.028677143031090316510622884038639313076,
		0.028314228399935807006793098139496578369
};
```

>[!Note]
>**Verificación de Diseño**: Se puede observar que los coeficientes exhiben simetría respecto al centro del arreglo. Esta propiedad es característica de los filtros FIR con fase lineal y sirve como validación del diseño correcto del filtro.

### Inicialización de Múltiples Filtros FIR
Para el ecualizador de 3 bandas, se requieren tres instancias de filtros independientes:
```C
arm_fir_init_f32(&filterLow, NUM_TAPS, &LowCoeffs[0], &StateLow[0], halfN);
arm_fir_init_f32(&filterBand, NUM_TAPS, &BandCoeffs[0], &StateMid[0], halfN);
arm_fir_init_f32(&filterHigh, NUM_TAPS, &HighCoeffs[0], &StateHigh[0], halfN);
```
Cada filtro requiere su propio buffer de estado (StateLow, StateMid, StateHigh) para mantener la continuidad temporal entre bloques de procesamiento.

### Procesamiento Paralelo y Mezcla de Señales

En los callbacks del DMA, los tres filtros se aplican en paralelo y sus salidas se combinan:
```C
// Aplicar los tres filtros al mismo bloque de entrada
arm_fir_f32(&filterLow, filt_in_ptr, filt_out_low_ptr, halfN);
arm_fir_f32(&filterBand, filt_in_ptr, filt_out_mid_ptr, halfN);
arm_fir_f32(&filterHigh, filt_in_ptr, filt_out_high_ptr, halfN);

// Combinar las salidas con ganancia unitaria inicial
for(int n = 0; n < halfN; n++)
{
    filt_out[n] = filt_out_low[n] + filt_out_mid[n] + filt_out_high[n];
    dac_buffer[n] = (uint32_t)filt_out[n];
}
```

### Implementación de Control de Ganancia
#### Conversión de Decibelios a Ganancia Lineal

Para convertir valores en decibelios a factores de ganancia lineal:

```C
/**
 * @brief Convierte una ganancia en decibelios a una ganancia lineal.
 * @param db Ganancia en decibelios (puede ser positiva o negativa).
 * @return Factor de ganancia lineal.
 */
float32_t gain_db_to_linear(float32_t db) {
    return powf(10.0f, db / 20.0f);
}
```
La relación matemáticas es:

$\text{dB} = 20 \times \log_{10}(\text{gain}) \rightarrow \text{gain} = 10^{\frac{\text{dB}}{20}}$

#### Aplicación de Ganancias con Librería CMSIS-DSP

La librería ARM CMSIS-DSP proporciona funciones optimizadas para el escalado de señales:

```C
// Factores de ganancia para cada banda (ejemplo con ganancia unitaria)
float32_t gainLow = 1.0f;   // Ganancia para bajos
float32_t gainMid = 1.0f;   // Ganancia para medios  
float32_t gainHigh = 1.0f;  // Ganancia para agudos

// Aplicar ganancias usando funciones optimizadas
arm_scale_f32(filt_out_low_ptr, gainLow, filt_out_low_ptr, halfN);
arm_scale_f32(filt_out_mid_ptr, gainMid, filt_out_mid_ptr, halfN);
arm_scale_f32(filt_out_high_ptr, gainHigh, filt_out_high_ptr, halfN);

// Combinar señales ecualizadas
for(int n = 0; n < halfN; n++)
{
    filt_out[n] = filt_out_low[n] + filt_out_mid[n] + filt_out_high[n];
    dac_buffer[n] = (uint32_t)filt_out[n];
}
```

#### Parametros de la función `arm_scale_f32`

```C
void arm_scale_f32(
   const float32_t * pSrc,
         float32_t scale,
         float32_t * pDst,
         uint32_t blockSize
);
```
- `pSrc`: Puntero al buffer de entrada
- `scale`: Factor de escalado (ganancia lineal)
- `pDst`: Puntero al buffer de salida
- `blockSize`: Número de muestras a procesar (`halfN`)

>[!Note]
>Los factores de ganancia (`gainLow`, `gainMid`, `gainHigh`) pueden obtenerse en tiempo real mediante lecturas ADC de potenciómetros externos, permitiendo el control interactivo del ecualizador.

## Protecciones implementadas
El sistema incopora múltiples mecanismos de protección para grantizar estabilidad y prevenir condiciones de error:

### Protección contra Saturación (Clipping)
Para evitar que la señal de salida exceda los límites del DAC y cause distorsión por saturación, se implementa un limitador digital. Este mecanismo asegura que la señal procesada se mantenga dentro del rango aceptable.

```C
// Limita el valor para prevenir overflow del DAC (0-4095)
if(dac_val > 4095.0f) dac_val = 4095.0f;
if(dac_val < 0.0f) dac_val = 0.0f;
dac_buffer[n] = (uint32_t)dac_val;
```

Esto evita distorsión por recorte cuando la señal excede el rango del DAC de 12 bits.

### Mecanismo Anti-Race Conditions
```C
// Copia local de ganancias para evitar condiciones de carrera
float32_t local_gainLow = gainLow;
float32_t local_gainMid = gainMid;
float32_t local_gainHigh = gainHigh;
```
Previene inconsistencias cuando el main loop actualiza ganancias durante el procesamiento de interrupciones.

### Control de Rango de Ganancias
```C
// Conversión segura de ADC a dB con rango limitado (-20 dB a +20 dB)
db_low = map_float(ADC_VAL[0], 0, 4095, -20.0f, 20.0f);
```
Limita las ganancias a un rango operativo seguro (±20 dB) para evitar sobre-amplificación.

### Verificación de Instancias de Periféricos
```C
if(hadc->Instance == ADC1) {
    // Procesamiento de audio
} else if(hadc->Instance == ADC2) {
    // Actualización de ganancias
}
```
Garantiza que los callbacks procesen solo los eventos de sus periféricos designados.

## Análisis de Carga Computacional
### Estimación de Carga de CPU
Operaciones por Muestra (por banda):
- **Filtrado FIR**: 32 multiplicaciones + 31 sumas = 63 operaciones
- **Escalado (ganancia)**: 1 multiplicación
- **Suma de Bandas**: 2 sumas
- **Conversión y Limitación**: 2 operaciones + 1 suma

**Total por muestra por banda**: 70 operaciones/muestra/banda

### Cálculo para Sistema Completo
```text
Frecuencia de muestreo: 40 kHz
Muestras/segundo: 40,000
Operaciones/segundo: 40,000 × 70 ops × 3 bandas ≈ 8.4 MOps
Capacidad STM32F446 (@180 MHz): ~210 MOps
```
**Utilización estimada**: 4% de la capacidad total del Cortex-M4

### Análisis de Tiempos de Ejecución
**Callbacks de DMA** (mitada de buffer - 16 muestras):
- Conversión ADC→Float: 16 operaciones
- 3x Filtrado FIR: 3 × 63 × 16 = 3024 operaciones
- Escalado: 3 × 16 = 48 operaciones
- Suma y Conversión: 16 x 3 = 48 operaciones
- **Total por callbacks**: ~3136 operaciones
  
**Tiempo estimado @180 MHz**: ~17.4 us

### Métricas de rendimiento
| **Métrica**               | **Valor Estimado** | Límite Seguro STM32F446 |  
|--------------------------|--------------------|-------------------------|
| **CPU Load (%)**            | 4%                 | < 70%                   |
| **Latencia por Callback (us)** | ~17.4 us           | < 25 us                |
| **Memoria RAM Utilizada (KB)** | ~2.1 KB             | < 128 KB                  |

## Capacidad de Expansión
El sistema tiene margen para:
- Aumentar a 5 bandas de ecualización
- Incrementar taps a 64 por filtro
- Incorporar efectos adicionales (reverberación, delay)

## Licencia

Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.
