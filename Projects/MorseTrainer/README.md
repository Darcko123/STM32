# Entrenador de Código Morse con STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F411-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![I2C](https://img.shields.io/badge/Protocol-I2C-yellow)](https://es.wikipedia.org/wiki/I2C)
[![PWM](https://img.shields.io/badge/Audio-PWM-yellow)](https://es.wikipedia.org/wiki/Modulaci%C3%B3n_por_ancho_de_pulsos)
[![SSD1306](https://img.shields.io/badge/Display-SSD1306-green)](/Librerias/SSD1306)
[![Version](https://img.shields.io/badge/MORSE-0.1.0-green.svg)](MorseTrainer/Core/Inc/MORSE.h)

## Tabla de Contenidos
- [Entrenador de Código Morse con STM32](#entrenador-de-código-morse-con-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características Principales](#características-principales)
    - [🎓 Pedagogía](#-pedagogía)
    - [🎧 Modos](#-modos)
    - [⚙️ Técnicas](#️-técnicas)
  - [Requisitos del Sistema](#requisitos-del-sistema)
    - [Hardware Requerido](#hardware-requerido)
    - [Software Requerido](#software-requerido)
  - [Configuración de Hardware](#configuración-de-hardware)
    - [Conexiones](#conexiones)
    - [Configuración del `.ioc`](#configuración-del-ioc)
    - [Librería SSD1306 enlazada](#librería-ssd1306-enlazada)
  - [Arquitectura del Software](#arquitectura-del-software)
    - [Diagrama de bloques](#diagrama-de-bloques)
    - [Módulo `MORSE.c/.h`](#módulo-morsech)
    - [Temporizaciones Morse y Farnsworth](#temporizaciones-morse-y-farnsworth)
    - [Decodificador adaptativo](#decodificador-adaptativo)
    - [Generación del tono](#generación-del-tono)
  - [Uso del Sistema](#uso-del-sistema)
    - [Controles](#controles)
    - [Menú principal](#menú-principal)
    - [Modos de entrenamiento](#modos-de-entrenamiento)
    - [Indicaciones de manipulación](#indicaciones-de-manipulación)
  - [Parámetros Configurables](#parámetros-configurables)
  - [Pruebas en PC](#pruebas-en-pc)
  - [Estructura de Archivos](#estructura-de-archivos)
  - [Solución de Problemas](#solución-de-problemas)
  - [Pendiente para Versiones Posteriores](#pendiente-para-versiones-posteriores)
  - [Licencia](#licencia)

## Descripción
Dispositivo autónomo para **aprender a recibir y transmitir código Morse** sin PC ni aplicación. El STM32 reproduce caracteres por un buzzer, el usuario responde **manipulando en Morse** con un pulsador o un manipulador real, y una pantalla OLED SSD1306 muestra el resultado, las estadísticas y los ajustes.

El entrenamiento sigue el **método Koch** con **espaciado Farnsworth**, el esquema que usan LCWO, la ARRL y la mayoría de cursos de radioaficionado: se aprende por sonido, a velocidad de carácter real desde el primer día, añadiendo caracteres uno a uno según el porcentaje de aciertos.

> [!TIP]
> La teoría pedagógica detrás de cada decisión del dispositivo está en [PEDAGOGIA.md](PEDAGOGIA.md).

## Características Principales

### 🎓 Pedagogía
- **Progresión Koch**: empieza con K y M; desbloquea el siguiente carácter al acertar 18 de los últimos 20 intentos (90 %), en el orden de LCWO.
- **Farnsworth progresivo**: caracteres a 18 WPM y silencios de 8 WPM efectivas en el nivel 2, que se acortan con el nivel hasta desaparecer en el 36.
- **Reconocimiento inmediato**: se mide la latencia hasta la primera pulsación; más de 2 s cuenta como no reconocido aunque sea correcto.
- **Retroalimentación auditiva**: al fallar suenan el carácter correcto y lo que manipuló el usuario, para comparar los dos ritmos de oído.
- **Selección ponderada**: los caracteres más fallados y el recién desbloqueado salen más a menudo.
- **Calidad de manipulación**: avisos concretos ("Rayas cortas", "Letra partida"…) con una exigencia que crece con el nivel.
- **Sesiones cortas**: aviso a los 15 minutos con resumen de la sesión.

### 🎧 Modos
- **Aprender**, **Identificar**, **Grupos de 5**, **Repaso** dirigido, **Libre** y **Estadísticas** (ver [Modos de entrenamiento](#modos-de-entrenamiento)).

### ⚙️ Técnicas
- **Arquitectura no bloqueante**: reproductor, decodificador y antirrebote avanzan en una ISR de 1 ms; el bucle principal solo dibuja y decide.
- **Decodificador adaptativo**: sigue la velocidad real del usuario (3–60 WPM) en lugar de exigirle la de reproducción.
- **Lógica Morse sin HAL**: `MORSE.c` compila igual en el STM32 y en el PC, con pruebas `assert` en [pruebas/test_morse.c](pruebas/test_morse.c).
- **Tono ajustable** de 400 a 1000 Hz por PWM con buzzer pasivo, sin librerías.

## Requisitos del Sistema

### Hardware Requerido
- **Microcontrolador**: Black Pill STM32F411CEU6 (100 MHz)
- **Pantalla**: OLED SSD1306 128×64 por I2C
- **Buzzer pasivo** (piezo o magnético)
- **Manipulador Morse** o pulsador
- **Transistor NPN** 2N2222 + resistencia de 1 kΩ (solo para buzzer magnético)
- **Programador**: ST-Link V2
- Protoboard y cables dupont

### Software Requerido
- **STM32CubeIDE** (incluye STM32CubeMX y el HAL de STM32F4xx)
- **gcc** en el PC, solo para las pruebas de [pruebas/](pruebas/)

## Configuración de Hardware

### Conexiones

| Función | Periférico | Pin Black Pill | Notas |
|---------|------------|----------------|-------|
| SSD1306 SCL | I2C1 | PB6 | Fast Mode 400 kHz |
| SSD1306 SDA | I2C1 | PB7 | |
| Buzzer pasivo | TIM3 CH1 (PWM) | PA6 | Piezo directo al pin; magnético a través del NPN |
| Manipulador | GPIO Input, pull-down | PB1 | Entre PB1 y 3V3, activo en alto |
| Botón de menú | GPIO Input, pull-up | PA0 | Botón **KEY** de la propia Black Pill, activo en bajo |
| LED de estado | GPIO Output | PC13 | LED de la placa, activo en bajo; sigue al manipulador |

```text
Manipulador:       PB1 ──[ manipulador ]── 3V3

Buzzer piezo:      PA6 ──[ buzzer ]── GND

Buzzer magnético:  3V3 ──[ buzzer ]── C
                                      2N2222
                   PA6 ──[ 1 kΩ ]──── B
                                      E ── GND
```

> [!NOTE]
> PB0 está configurado en el `.ioc` como entrada con pull-down, pero el firmware no lo usa.

### Configuración del `.ioc`

| Periférico | Parámetro | Valor | Resultado |
|------------|-----------|-------|-----------|
| RCC | PLL desde HSI (16 MHz) | SYSCLK 100 MHz | El HSE de 25 MHz queda libre si se necesitara más precisión |
| TIM2 | Prescaler / Period | 99 / 999 | Tick de 1 ms, con interrupción global en NVIC |
| TIM3 CH1 | Prescaler / Period / Pulse | 99 / 1427 / 714 | 700 Hz al 50 % |
| TIM3 | Auto-reload preload | Enable | Evita un corte de ~65 ms al bajar el ARR con el contador por encima |
| I2C1 | Speed Mode | Fast Mode 400 kHz | |

### Librería SSD1306 enlazada
El driver de la pantalla **no se copia**: `SSD1306.c/.h` y `FONTS.c/.h` están enlazados desde [Librerias/SSD1306](/Librerias/SSD1306) en el `.project` (`PARENT-3-PROJECT_LOC/Librerias/SSD1306/...`) y la ruta `../../../../Librerias/SSD1306` está añadida a los *include paths* del compilador C en Debug y Release.

> [!IMPORTANT]
> Por eso el proyecto solo compila dentro de este repositorio, con la estructura de carpetas intacta. Son cuatro niveles porque el proyecto de CubeIDE está anidado en `Projects/MorseTrainer/MorseTrainer/`.

## Arquitectura del Software

### Diagrama de bloques
```text
                ┌──────────────────── ISR TIM2 (1 ms) ────────────────────┐
 Manipulador ──►│ Antirrebote (5 muestras) ─► MORSE_DecodTick() ─► carácter│──┐
     PB1        │                         └─► tono local + LED PC13      │  │
                │ MORSE_ReprodTick() ─► abre/cierra el CCR de TIM3 ─► PA6│  │
                └──────────────────────────────────────────────────────────┘  │
                                                                              ▼
                ┌──────────────────────── Bucle principal ────────────────────┐
 Botón KEY ────►│ Menú y modos (Aprender, Identificar, Grupos, Repaso, Libre) │
     PA0        │ Progresión Koch y estadísticas (MORSE_Koch*, MORSE_Sesion*) │
                │ Pantalla SSD1306 (solo redibuja al cambiar algo)            │
                └─────────────────────────────────────────────────────────────┘
```

Todo lo que depende del tiempo vive en la ISR: `SSD1306_UpdateScreen()` bloquea ~25 ms, y si el reproductor o el decodificador estuvieran en el bucle principal, cada redibujado deformaría el ritmo.

### Módulo `MORSE.c/.h`
Lógica Morse pura, **sin HAL y sin estado global**: cada función trabaja sobre la estructura que recibe, así que no hay `MORSE_Init()`; cada estructura tiene su propia función de arranque. Sigue las [convenciones de las librerías](/Librerias) del repositorio (`MORSE_Status_t`, Doxygen en español).

| Grupo | Funciones | Qué hace |
|-------|-----------|----------|
| Codificación | `MORSE_Patron`, `MORSE_CalcularTiempos` | Patrón de A–Z / 0–9 y tiempos por WPM + Farnsworth |
| Reproductor | `MORSE_Reproducir`, `MORSE_ReprodTick`, `MORSE_Reproduciendo` | Reproducción no bloqueante por ticks de 1 ms |
| Decodificador | `MORSE_DecodInit`, `MORSE_DecodVaciar`, `MORSE_DecodTick`, `MORSE_EvaluarManip` | Pulsaciones → carácter, y calidad del ritmo |
| Progresión Koch | `MORSE_KochInit`, `MORSE_KochRegistrar`, `MORSE_KochAciertos`, `MORSE_KochWpmEfectivo` | Desbloqueo por ventana de aciertos y Farnsworth por nivel |
| Sesión | `MORSE_SesionInit`, `MORSE_SesionRegistrar`, `MORSE_SesionElegir`, `MORSE_SesionRepaso`, `MORSE_SesionMasFallados`, `MORSE_SesionConfusiones` | Estadísticas, selección ponderada y repaso |

Las funciones de arranque y registro validan sus punteros y devuelven `MORSE_Status_t`; las de ticks (llamadas a 1 kHz desde la ISR) y las consultas devuelven el valor directamente y esperan punteros válidos.

### Temporizaciones Morse y Farnsworth
La unidad es el **punto**: `punto_ms = 1200 / WPM` (palabra de referencia *PARIS*, 50 unidades).

| Elemento | Duración estándar | Con Farnsworth |
|----------|-------------------|----------------|
| Punto | 1 unidad | 1 unidad (velocidad de carácter) |
| Raya | 3 unidades | 3 unidades |
| Espacio entre símbolos | 1 unidad | 1 unidad |
| Espacio entre caracteres | 3 unidades | `3·tₐ / 19` |
| Espacio entre palabras | 7 unidades | `7·tₐ / 19` |

Con Farnsworth se usa la fórmula de la ARRL, donde `c` es la velocidad de carácter y `s` la efectiva (en WPM):

$$
t_a = \frac{60\,c - 37{,}2\,s}{s \cdot c}\ \text{s}
$$

`MORSE_CalcularTiempos()` la calcula en enteros. Ejemplo con los valores de arranque (18 / 8 WPM):

| Punto | Raya | Entre caracteres | Entre palabras |
|-------|------|------------------|----------------|
| 67 ms | 200 ms | 858 ms | 2002 ms |

La velocidad efectiva crece en línea recta con el nivel (`MORSE_KochWpmEfectivo`): 8 WPM en el nivel 2 → 18 WPM (sin Farnsworth) en el 36, aproximadamente 1 WPM cada 3,4 niveles.

### Decodificador adaptativo
`MORSE_DecodTick()` recibe cada milisegundo el estado del manipulador ya sin rebotes. Con `u` = punto estimado del usuario:

- **Fin de letra**: silencio > 2u.
- **Fin de palabra**: silencio > máx((letra + palabra) / 2, 5u). Con un umbral fijo de 5 unidades, el espaciado Farnsworth partía cada letra en una palabra.
- **Punto o raya** (al cerrar la letra): si la letra mezcla pulsaciones cortas y largas (la más larga ≥ 2× la más corta), el umbral es el punto medio entre ambas; si todas se parecen (E, T, M, S, O…), raya = pulsación ≥ 2u.
- **Adaptación**: cada letra válida ajusta `u` con una media móvil de 1/4, con la muestra limitada a [u/2, 2u] y `u` en [20, 400] ms (60–3 WPM). Por eso la primera letra a una velocidad muy distinta puede salir partida, pero desde la segunda o la tercera se lee bien.
- Más de 5 símbolos o un patrón inexistente devuelven `'?'`.

### Generación del tono
TIM3 CH1 corre siempre; el tono se abre y se cierra escribiendo el CCR (50 % del ARR actual o 0). La frecuencia se cambia con el ARR:

$$
ARR = \frac{1\,000\,000}{f} - 1
$$

El bip de acierto usa ARR / 2, es decir, el doble de la frecuencia del tono.

## Uso del Sistema

### Controles
| Control | En el menú | Dentro de un modo |
|---------|------------|-------------------|
| Botón **KEY** (PA0) | Pasa a la opción siguiente | Vuelve al menú |
| Manipulador (PB1) | Cualquier letra manipulada elige la opción | Responder |

Al encender suena un bip de 300 ms (sirve para comprobar la frecuencia) y aparece el menú.

### Menú principal
El menú tiene 11 opciones y muestra 6 a la vez; se desplaza con la opción resaltada.

| Opción | Función |
|--------|---------|
| Aprender | Presentación de caracteres con su patrón |
| Identificar | Práctica principal: oír y responder |
| Grupos de 5 | Grupos de 5 caracteres respondidos de memoria |
| Repaso | Identificar restringido a lo más fallado |
| Libre | Manipular y ver el texto decodificado |
| Estadísticas | Resumen de la sesión |
| Nivel | Caracteres desbloqueados (2–36); cada letra manipulada suma uno y tras 36 vuelve a 2. Cambiarlo vacía la ventana de aciertos |
| Velocidad | Velocidad de carácter: 12–30 WPM, de 2 en 2 |
| Farnsworth | Velocidad efectiva en el nivel 2: de 4 WPM a la de carácter ("no") |
| Tono | 400–1000 Hz, de 100 en 100; al cambiarlo suena una raya con el tono nuevo |
| Patrón | Muestra u oculta el patrón `.-` al fallar |

> [!NOTE]
> Los ajustes no se guardan: al encender vuelven a los valores de `main.c` (ver [Parámetros Configurables](#parámetros-configurables)). La sesión y el nivel también se reinician.

### Modos de entrenamiento

**Aprender.** Carácter grande en pantalla con su patrón, reproducido tres veces; el patrón se oculta y el usuario lo repite. Al acertar, bip y siguiente; al fallar, se muestra el patrón con lo manipulado (`Tu: -.-.`), suenan el correcto y el del usuario, y se repite. Recorre los desbloqueados empezando por el más nuevo. La cabecera muestra la velocidad (`Aprender 18/8 WPM`).

**Identificar.** Suena un carácter aleatorio de los desbloqueados (selección ponderada) y el usuario lo manipula:
- Correcto y en menos de 2 s → **"Bien"** + bip.
- Correcto pero tarde → **"Bien, pero lento"**, sin bip; cuenta como fallo.
- Incorrecto → se muestra el correcto y suenan el correcto y lo manipulado (si es un carácter válido).
- Sin pulsar en 6 s → **"Sin respuesta"**: se muestra y suena el correcto.

La cabecera muestra el nivel y los aciertos de la ventana. Al llegar a 18/20 se desbloquea el siguiente carácter: se presenta tres veces con su patrón ("Nuevo caracter") y la ventana vuelve a empezar.

**Grupos de 5.** La práctica clásica de Koch: suenan 5 caracteres seguidos con el espaciado del nivel y se manipulan de memoria. Se corrige al completar 5 letras o tras 6 s sin letras nuevas. Todo bien → bip; si no, `N/5 bien`, el grupo oído encima de lo manipulado y el grupo vuelve a sonar. Cada carácter cuenta en la progresión y en las estadísticas.
> [!TIP]
> Si no reconoces un carácter, manipula 6 o más puntos (`?`) para no desplazar el resto del grupo.

**Repaso.** Igual que Identificar, pero elige solo entre los 3 caracteres más fallados y las dos letras de cada uno de los 3 pares más confundidos. La cabecera muestra la lista (`Repaso EURSAK`); sin fallos todavía muestra "Repaso todos". Cuenta en las estadísticas pero **no** en la progresión Koch: la muestra está sesgada hacia lo difícil y frenaría el desbloqueo.

**Libre.** Se manipula y se ve el texto decodificado en cuatro líneas de 18 caracteres que suben al llenarse. La cabecera muestra la velocidad estimada (`Tu N WPM`) y la línea inferior el último carácter con lo manipulado (`K -.-`). El fin de palabra es a 5 puntos del usuario. No cuenta en estadísticas ni en la progresión.

**Estadísticas.** Minutos de sesión, reconocidos/intentos y porcentaje, latencia media, los 3 caracteres más fallados, los 3 pares más confundidos (`oído>respondido`) y la velocidad del usuario. Cuentan Identificar, Grupos y Repaso. A los 15 minutos, al pasar al siguiente carácter, suena un bip largo y se abre esta pantalla con "Fin: N min"; avisa una sola vez y se puede seguir desde el menú.

### Indicaciones de manipulación
En Aprender e Identificar, tanto al acertar como al fallar, `MORSE_EvaluarManip()` compara las pulsaciones con el carácter **esperado** y puede mostrar:

| Indicación | Causa | Tolerancia nivel 2 → 36 |
|------------|-------|-------------------------|
| Letra partida | Lo manipulado es el principio de lo esperado: una pausa cerró la letra antes | — |
| Rayas cortas / Rayas largas | Relación raya/punto fuera de 3 ± tolerancia | ± 1 → ± 0,5 |
| Pausas largas | Hueco máximo entre símbolos demasiado largo | 1,8 → 1,4 puntos |

No se muestra nada si se manipuló otro carácter (largo distinto, o una raya que no supera a un punto): eso es confusión de oído, no de ritmo.

## Parámetros Configurables
En el bloque `USER CODE BEGIN PD` de [main.c](MorseTrainer/Core/Src/main.c):

```C
#define ANTIRREBOTE_MS  5U              // Muestras iguales seguidas para aceptar un cambio
#define WPM_CARACTER    18U             // Velocidad de carácter al encender
#define WPM_EFECTIVO    8U              // Farnsworth en el nivel 2 al encender
#define TONO_HZ         700U            // Tono al encender
#define BIP_MS          60U             // Bip de acierto
#define PAUSA_MS        800U            // Silencio antes del siguiente carácter
#define LATENCIA_MAX_MS 2000U           // Más hasta la primera pulsación = no reconocido
#define ESPERA_MAX_MS   6000U           // Sin pulsar en este tiempo = sin respuesta
#define SESION_MS       (15UL * 60UL * 1000UL)  // Duración recomendada de la sesión
#define GRUPO_LARGO     5U              // Caracteres por grupo
```

En [MORSE.h](MorseTrainer/Core/Inc/MORSE.h) (redefinibles con `-D` al compilar):

| Macro | Valor | Significado |
|-------|-------|-------------|
| `MORSE_KOCH_VENTANA` | 20 | Intentos que se miran para desbloquear (≤ 32) |
| `MORSE_KOCH_ACIERTOS` | 18 | Aciertos necesarios en la ventana (90 %) |
| `MORSE_SESION_NUEVO_INTENTOS` | 10 | Intentos con peso extra del recién desbloqueado |
| `MORSE_PUNTO_MIN_MS` / `MORSE_PUNTO_MAX_MS` | 20 / 400 | Límites del punto estimado (60 / 3 WPM) |

## Pruebas en PC
`MORSE.c` no depende del HAL, así que se prueba en el PC con `gcc` y `assert`. Desde `Projects/MorseTrainer`:

```bash
gcc -std=c11 -Wall -Wextra -IMorseTrainer/Core/Inc pruebas/test_morse.c \
    MorseTrainer/Core/Src/MORSE.c -o /tmp/test_morse && /tmp/test_morse
```

Salida esperada: `test_morse: OK`. Cubre codificación, tiempos, tramos del reproductor, ida y vuelta reproductor → decodificador a 20/20, 18/8 y 30/30 WPM, manipulación imperfecta, usuarios a 8 y 30 WPM partiendo de 18, evaluación de la manipulación, progresión Koch, Farnsworth por nivel, selección ponderada, estadísticas y repaso.

## Estructura de Archivos
```plaintext
Projects/MorseTrainer/
├── README.md                       # Este archivo
├── PEDAGOGIA.md                    # Teoría del método de enseñanza
├── PLAN.md                         # Plan de trabajo y decisiones
├── pruebas/
│   └── test_morse.c                # Pruebas en PC de MORSE.c
└── MorseTrainer/                   # Proyecto de STM32CubeIDE
    ├── MorseTrainer.ioc            # Configuración de periféricos
    ├── Core/
    │   ├── Inc/
    │   │   ├── MORSE.h             # API de la lógica Morse
    │   │   └── main.h
    │   └── Src/
    │       ├── MORSE.c             # Lógica Morse pura (sin HAL)
    │       └── main.c              # ISR, menú y modos de entrenamiento
    ├── Drivers/                    # HAL y CMSIS de STM32F4xx
    └── (enlazados) SSD1306.c/.h, FONTS.c/.h  → /Librerias/SSD1306
```

## Solución de Problemas

1. **El LED PC13 parpadea rápido al encender**
   1. Es `Error_Handler()`: normalmente la pantalla no responde.
   2. Revisar SDA/SCL (PB7/PB6), alimentación y dirección I2C del SSD1306.

2. **No suena el buzzer o suena muy débil**
   1. Comprobar que es **pasivo**: uno activo no reproduce la frecuencia del PWM.
   2. Si es magnético, usar el NPN; el pin no da corriente suficiente.

3. **Una pulsación cuenta varias veces**
   1. Subir `ANTIRREBOTE_MS` (los manipuladores con contactos gastados rebotan más).

4. **Se parten las letras o se confunden puntos con rayas**
   1. El decodificador necesita una o dos letras para adaptarse a tu velocidad; probar en modo Libre y mirar `Tu N WPM`.
   2. Si persiste, bajar la velocidad o la Farnsworth desde el menú.

5. **No compila: no encuentra `SSD1306.h`**
   1. El proyecto debe estar dentro del repositorio, con [Librerias/SSD1306](/Librerias/SSD1306) cuatro niveles por encima del proyecto de CubeIDE.

## Pendiente para Versiones Posteriores
- Persistencia del progreso (último sector de la flash interna o una memoria W25Q).
- Puntuación y prosignos (AR, SK, BT).

## Licencia
Este proyecto se distribuye bajo la licencia MIT. Consulte el archivo [LICENSE](/LICENSE.md) para más detalles.
