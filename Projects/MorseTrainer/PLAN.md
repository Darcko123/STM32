# Entrenador de Código Morse — Plan de trabajo

Registro del alcance acordado y de lo que queda pendiente. Marcar `[x]` al completar.

## Decisiones tomadas (2026-11-23)

| Tema | Decisión |
|---|---|
| Placa | Black Pill STM32F411CEU6 @ 100 MHz |
| Pantalla | `Librerias/SSD1306` (enlazada, no copiada) |
| Sonido | Buzzer pasivo con PWM de timer (sin librería, HAL directa) |
| Respuesta del usuario | Manipulando en Morse con un pulsador/manipulador |
| Persistencia | No en v1 |
| Voz / audio | Descartado (la F411 no tiene DAC) |

### Supuestos por confirmar

- [ ] Juego de caracteres: A–Z y 0–9 (puntuación y prosignos AR/SK/BT después).
- [ ] Navegación: botón de menú dedicado en PB1 (alternativa: pulsación larga del manipulador).
- [ ] Velocidad inicial: caracteres a 18 WPM, espaciado efectivo 8 WPM (Farnsworth).
- [ ] Frecuencia del tono: 700 Hz.

## Hardware

| Función | Periférico | Pin | Configuración `.ioc` |
|---|---|---|---|
| Pantalla SSD1306 | I2C1 Fast Mode 400 kHz | PB6 SCL / PB7 SDA | — |
| Buzzer pasivo | TIM3 CH1 PWM | PA6 | PSC 99, ARR 1427, Pulse 714 (700 Hz, 50 %) |
| Tick 1 ms | TIM2 base de tiempo + NVIC | — | PSC 99, ARR 999 |
| Manipulador | GPIO Input pull-up | PB0 | — |
| Botón menú | GPIO Input pull-up | PB1 | — |
| LED estado | GPIO Output | PC13 | — |

- Buzzer piezo: directo al pin. Buzzer magnético: NPN (2N2222 + 1 kΩ en base).
- Tono ajustable cambiando ARR: `ARR = 1 000 000 / f − 1`.

## Arquitectura

- **ISR TIM2 (1 ms)**: antirrebote del manipulador (~5 ms), tono local al manipular, máquina de estados del reproductor (no bloqueante).
- **Bucle principal**: pantalla y lógica de lecciones. `SSD1306_UpdateScreen` bloquea ~25 ms, por eso nada que dependa del tiempo vive aquí.
- **`morse.c/.h`** (lógica pura, sin HAL, probable en PC):
  - Tablas A–Z / 0–9 (reutilizadas de `Projects/UARTToMorseCode`).
  - Tiempos: punto = 1200 / WPM ms; espaciado Farnsworth.
  - Decodificador: pulsación < 2 puntos → punto, si no raya; silencio > 2 unidades → fin de letra, > 5 → fin de palabra.
- **`main.c`**: ISR, modos y menú.

## Método pedagógico

Base: **método Koch** combinado con **espaciado Farnsworth**, el estándar usado por LCWO, ARRL y la mayoría de cursos de radioaficionado. Principios que el dispositivo debe respetar:

1. **Aprender por sonido, no por la vista.** Cada carácter se reconoce como un ritmo completo, no contando puntos y rayas. Mostrar el patrón `.-` como apoyo principal fomenta "traducir" con una tabla mental, que luego frena al subir la velocidad. El patrón se muestra solo al presentar un carácter nuevo y al fallar, y se puede ocultar desde el menú.
2. **Velocidad de carácter alta desde el primer día** (≥ 18 WPM). A esa velocidad no da tiempo a contar símbolos y el oído aprende el ritmo. Lo que se alarga son los silencios entre caracteres (Farnsworth, 8 WPM efectivas al inicio), que se van acortando con el progreso hasta igualar la velocidad de carácter.
3. **Introducción progresiva (Koch).** Se empieza con 2 caracteres y se añade uno nuevo solo al alcanzar ≥ 90 % de aciertos sobre los últimos 20 intentos. Orden Koch (LCWO) restringido a A–Z y 0–9:

   `K M U R E S N A P T L W I J Z F O Y V G 5 Q 9 2 H 3 8 B 4 7 C 1 D 6 0 X`

   El orden separa caracteres que se confunden fácilmente (E/T, A/N, U/V, D/B) y deja los de patrón regular, como los números, para cuando el oído ya está entrenado.
4. **Reconocimiento inmediato.** La respuesta debe empezar sin pensarla. Se mide la latencia desde el fin de la reproducción hasta la primera pulsación. Una respuesta que tarda más de ~2 s cuenta como no reconocida, aunque sea correcta, para no premiar la deducción.
5. **Retroalimentación inmediata.** Si se acierta, un bip corto y se sigue. Si se falla, se muestra el carácter correcto, se reproduce de nuevo y se reproduce también lo que manipuló el usuario, para comparar los dos ritmos de oído.
6. **Más práctica para lo nuevo y lo fallado.** La selección aleatoria se pondera: el carácter recién introducido sale más a menudo durante sus primeros intentos, y los caracteres con menor tasa de acierto se repiten más.
7. **Calidad de manipulación.** Como la respuesta es manipulada, también se evalúa el ritmo de emisión: relación raya/punto (ideal ≈ 3) y espacios entre símbolos (≈ 1 unidad). Se avisa con indicaciones concretas ("rayas cortas", "letra partida por una pausa larga"). Al principio el decodificador es tolerante y se va volviendo más exigente con el nivel.
8. **Sesiones cortas.** Se recomiendan 10–15 minutos. El dispositivo avisa al cumplirse el tiempo y muestra el resumen de la sesión.
9. **Seguimiento de confusiones.** Se registran los pares de caracteres confundidos (oído X, respondido Y) para mostrarlos en las estadísticas y reforzarlos en el repaso.

## Fases

### Fase 0 — Preparación
- [x] Crear rama `projects/MorseTrainer`.
- [x] Generar el proyecto en STM32CubeIDE con la configuración de la tabla de hardware (`MorseTrainer/`). Corregidos en `.ioc` y `main.c`: pull-up en PB0/PB1 y Pulse 714 en TIM3 CH1 (venían a NOPULL y 0).
- [x] Enlazar `SSD1306.c/.h` y `FONTS.c/.h` en `.project` (`PARENT-3-PROJECT_LOC`) + include path `../../../../Librerias/SSD1306` en Debug y Release (compilador C). Cuatro niveles porque el proyecto está anidado en `Projects/MorseTrainer/MorseTrainer/`.
- [x] Build headless Debug y Release: 0 errores, 0 avisos.

> Reloj: PLL desde HSI (16 MHz, ±1 %) → 100 MHz. Suficiente para tono y tiempos Morse; el HSE de 25 MHz de la Black Pill queda disponible si hiciera falta más precisión.

### Fase 1 — Puesta en marcha del hardware
- [x] Tono PWM: TIM3 CH1 siempre en marcha, `Tono()` abre/cierra con el CCR (50 % del ARR actual). Bip de 300 ms al arrancar para medir la frecuencia.
- [x] ISR de 1 ms en TIM2 (`HAL_TIM_PeriodElapsedCallback`).
- [x] Manipulador con antirrebote (5 muestras seguidas) y tono local; LED PC13 sigue al manipulador.
- [x] SSD1306 mostrando "Morse", número de pulsaciones y duración de la última (solo redibuja al cambiar).
- [ ] Prueba en hardware: bip de arranque a ~700 Hz; cada pulsación suma exactamente 1 (si suma más, subir `ANTIRREBOTE_MS`); duraciones coherentes; parpadeo rápido del LED = `Error_Handler` (p. ej. pantalla no detectada).

### Fase 2 — Módulo `morse.c`
- [ ] Tablas y codificación carácter → patrón.
- [ ] Cálculo de tiempos por WPM + Farnsworth.
- [ ] Reproductor no bloqueante (avanzado desde la ISR).
- [ ] Decodificador de pulsaciones → carácter.
- [ ] Prueba en PC (`gcc` + `assert`) de codificación, tiempos y decodificación.

### Fase 3 — Modos de aprendizaje y práctica
- [ ] Menú principal (botón PB1).
- [ ] Selección del nivel inicial al encender (número de caracteres desbloqueados), sustituto de la persistencia.
- [ ] Modo **Aprender**: carácter grande (patrón solo en la presentación, ocultable), reproducción ×3, el usuario lo repite, avanza al acertar.
- [ ] Modo **Identificar**: carácter aleatorio de los desbloqueados, respuesta manipulada, bip de acierto; al fallar, carácter correcto + reproducción del correcto y de lo manipulado.
- [ ] Progresión Koch en el orden LCWO: empieza con K y M; desbloquea el siguiente al llegar a 90 % en los últimos 20 intentos.
- [ ] Medición de latencia de respuesta (> ~2 s cuenta como no reconocido).
- [ ] Selección ponderada: más peso al carácter nuevo y a los de menor acierto.
- [ ] Temporizador de sesión (aviso a los 10–15 min).
- [ ] Pantalla de estadísticas de la sesión: aciertos, caracteres más fallados, pares confundidos, latencia media.
- [ ] Prueba en hardware.

### Fase 4 — Opcional
- [ ] Modo libre: manipular y ver el texto decodificado.
- [ ] Ajuste de WPM, Farnsworth y tono desde el menú.
- [ ] Umbral punto/raya adaptativo a la velocidad real del usuario.
- [ ] Evaluación de la calidad de manipulación (relación raya/punto, espacios) con indicaciones concretas; tolerancia decreciente con el nivel.
- [ ] Reducción automática del espaciado Farnsworth con el progreso.
- [ ] Repaso dirigido a los caracteres y pares más confundidos.
- [ ] Grupos de 5 caracteres (práctica clásica de Koch) respondidos manipulando.

## Pendiente para versiones posteriores

- [ ] Persistencia del progreso: último sector de la flash interna con `HAL_FLASH` (borrado por sector, ~10 000 ciclos → guardar al cerrar lección), o un driver W25Q (`Librerias/W25Q` está vacía).
- [ ] Puntuación y prosignos.
- [ ] README del proyecto.
