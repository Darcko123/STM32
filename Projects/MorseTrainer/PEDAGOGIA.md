# Pedagogía del Entrenador Morse

Este documento explica **por qué** el entrenador enseña como enseña: de dónde vienen el método Koch y el espaciado Farnsworth, qué principios del aprendizaje motor y de la memoria los respaldan, y cómo se traduce cada principio en una decisión concreta del firmware. El funcionamiento técnico está en el [README](README.md).

## Tabla de Contenidos
- [Pedagogía del Entrenador Morse](#pedagogía-del-entrenador-morse)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [El problema: por qué el Morse se aprende mal](#el-problema-por-qué-el-morse-se-aprende-mal)
  - [Fundamentos](#fundamentos)
    - [1. El Morse es una habilidad auditiva, no un código escrito](#1-el-morse-es-una-habilidad-auditiva-no-un-código-escrito)
    - [2. El método Koch](#2-el-método-koch)
    - [3. El espaciado Farnsworth](#3-el-espaciado-farnsworth)
    - [4. Automatización: las tres fases del aprendizaje motor](#4-automatización-las-tres-fases-del-aprendizaje-motor)
    - [5. Práctica de recuperación y retroalimentación inmediata](#5-práctica-de-recuperación-y-retroalimentación-inmediata)
    - [6. Práctica intercalada y ponderada](#6-práctica-intercalada-y-ponderada)
    - [7. Práctica distribuida: sesiones cortas](#7-práctica-distribuida-sesiones-cortas)
  - [Del principio al firmware](#del-principio-al-firmware)
  - [El orden Koch](#el-orden-koch)
  - [Recepción y transmisión: por qué se responde manipulando](#recepción-y-transmisión-por-qué-se-responde-manipulando)
  - [Cómo usar el entrenador](#cómo-usar-el-entrenador)
  - [Límites del método](#límites-del-método)
  - [Referencias](#referencias)

## El problema: por qué el Morse se aprende mal
La forma intuitiva de aprender Morse es memorizar una tabla visual (`A = .-`, `B = -...`) y, al oír un carácter, contar los puntos y rayas y buscarlo mentalmente. Funciona a velocidades muy bajas y luego **se estanca**: es conocida la "meseta de las 10 WPM", donde muchos alumnos se quedan atascados durante meses.

La causa es que ese proceso tiene tres pasos conscientes (oír → contar → buscar en la tabla) y a partir de cierta velocidad no hay tiempo para hacerlos. Para superar la meseta hay que **desaprender** la tabla y aprender a reconocer el carácter como un sonido completo, lo que cuesta más que haberlo aprendido así desde el principio.

Todos los principios siguientes buscan lo mismo: que el alumno nunca construya esa tabla mental.

## Fundamentos

### 1. El Morse es una habilidad auditiva, no un código escrito
Un operador experto no oye "punto-raya"; oye **"di-dah"** como una unidad, igual que un lector no deletrea las palabras. Cada carácter tiene un ritmo propio, una *gestalt* sonora, y es ese ritmo lo que se reconoce.

Consecuencias:
- **La representación visual estorba.** Ver `.-` invita a traducir. El patrón solo debe aparecer como apoyo puntual.
- **Lo que se practica es el reconocimiento**, no la memorización: oír y responder, no leer y repetir.

### 2. El método Koch
Propuesto por el psicólogo alemán **Ludwig Koch** en la década de 1930 a partir de sus estudios sobre cómo aprendían los operadores de radio. Sus dos ideas centrales:

1. **Velocidad de carácter final desde el primer día.** Koch observó que los ritmos aprendidos a baja velocidad no se parecen a los mismos caracteres a velocidad real: a 5 WPM una "A" son dos sonidos separados; a 18 WPM es un único sonido. Aprender a 5 WPM es aprender un código distinto.
2. **Empezar con solo dos caracteres** y añadir uno nuevo únicamente cuando se reconocen los anteriores con un **90 % de aciertos**. El alumno siempre trabaja a velocidad real, pero con un problema de tamaño manejable que crece a su ritmo.

El resultado es que el alumno practica desde el principio exactamente la tarea final, solo que con un alfabeto reducido. Koch es el método que usan hoy LCWO (*Learn CW Online*), muchos cursos de la ARRL y gran parte de los clubes de radioaficionados.

### 3. El espaciado Farnsworth
Un problema práctico de Koch es que a 18 WPM el principiante no tiene tiempo de **procesar** un carácter antes de que llegue el siguiente. El espaciado Farnsworth (por **Donald R. Farnsworth**, W6TTB) lo resuelve separando dos velocidades:

- **Velocidad de carácter**: la de los puntos, rayas y silencios dentro del carácter. Se mantiene alta, así el ritmo que se aprende es el real.
- **Velocidad efectiva**: la media resultante al alargar **solo** los silencios entre caracteres y entre palabras. Es baja al principio y da tiempo a pensar.

```text
18 WPM sin Farnsworth:   K ─ M ─ K          (silencio entre caracteres: 200 ms)
18/8 WPM Farnsworth:     K ──────── M ──────── K   (858 ms)
                         ↑ el sonido de cada carácter es idéntico en los dos casos
```

Según el alumno progresa, los silencios se acortan hasta llegar al espaciado estándar. La ARRL publicó una norma de temporización Farnsworth (Bloom, 1990) que define cuánto se alarga cada silencio; es la que implementa `MORSE_CalcularTiempos()`.

### 4. Automatización: las tres fases del aprendizaje motor
El modelo clásico de **Fitts y Posner** (1967) describe tres fases en el aprendizaje de cualquier habilidad:

| Fase | Cómo se ejecuta | En Morse |
|------|-----------------|----------|
| Cognitiva | Consciente, lenta, con errores | "Esto era di-dah… ¿A?" |
| Asociativa | Más fluida, menos errores | Se reconoce sin dudar, pero con esfuerzo |
| Autónoma | Automática, sin atención | El carácter "se oye" directamente |

El objetivo es la fase autónoma. Lo que distingue a las dos últimas fases es la **velocidad de respuesta**: un carácter reconocido de verdad se responde de inmediato; uno deducido, tras una pausa. Por eso la latencia es un indicador tan importante como el acierto.

### 5. Práctica de recuperación y retroalimentación inmediata
La psicología de la memoria muestra que **recuperar** activamente lo aprendido fortalece la memoria mucho más que volver a estudiarlo (*efecto test*, Roediger y Karpicke, 2006). Escuchar Morse pasivamente enseña poco; tener que responder a cada carácter enseña mucho.

Además, la **retroalimentación** es más útil cuanto antes llega y cuanto más concreta es. En una habilidad auditiva, la retroalimentación más concreta es también auditiva: oír uno junto a otro el ritmo correcto y el que uno ha producido.

### 6. Práctica intercalada y ponderada
Practicar los elementos mezclados al azar (*práctica intercalada*) produce un aprendizaje más duradero que practicarlos en bloques ("diez K, luego diez M"), porque obliga a **discriminar** en cada intento. En Morse, discriminar es justamente la tarea: distinguir la K de la R o la U de la V.

Dentro de esa mezcla tiene sentido dar más presencia a lo que más cuesta, de forma parecida a como lo hacen los sistemas de repetición espaciada: los caracteres que se fallan más, y el recién llegado, salen con más frecuencia, sin dejar de practicar el resto.

### 7. Práctica distribuida: sesiones cortas
Desde Ebbinghaus se sabe que repartir la práctica en varias sesiones (*práctica distribuida*) es más eficaz que concentrarla en una sola. En Morse, además, la atención auditiva se fatiga pronto y los errores por cansancio enseñan ritmos equivocados. La recomendación habitual son **10–15 minutos al día** mejor que una hora a la semana.

## Del principio al firmware
Cada principio se traduce en una o más decisiones del dispositivo:

| Principio | Implementación | Dónde |
|-----------|----------------|-------|
| Habilidad auditiva | El patrón `.-` solo se ve al presentar un carácter nuevo y al fallar, y se puede ocultar del todo (opción **Patrón**). La respuesta es sonora (manipulando), nunca eligiendo en una lista | `main.c` |
| Koch: velocidad real | Caracteres a 18 WPM desde el nivel 2 (ajustable 12–30) | `WPM_CARACTER` |
| Koch: introducción progresiva | Empieza con K y M; desbloquea el siguiente con 18/20 aciertos en los últimos 20 intentos (90 %); el nuevo se presenta 3 veces con su patrón | `MORSE_KochRegistrar` |
| Farnsworth | Silencios de 8 WPM efectivas en el nivel 2 que se acortan en línea recta hasta el espaciado estándar en el nivel 36 | `MORSE_KochWpmEfectivo` |
| Automatización | Se mide la latencia hasta la primera pulsación: más de 2 s es **"Bien, pero lento"** y cuenta como fallo, para no premiar la deducción | `LATENCIA_MAX_MS` |
| Recuperación activa | Todos los modos exigen responder; no hay modo de escucha pasiva | Identificar, Grupos, Repaso |
| Retroalimentación inmediata | Acierto: bip corto y se sigue. Fallo: se muestra el correcto y **suenan el correcto y lo manipulado**, uno tras otro | `main.c` |
| Práctica intercalada | Caracteres al azar entre los desbloqueados | `MORSE_SesionElegir` |
| Práctica ponderada | Peso = 1 + 4 · tasa de fallo suavizada (≈1 si siempre se acierta, ≈5 si siempre se falla, 3 sin intentos); ×3 para el recién desbloqueado en sus primeros 10 intentos | `MORSE_SesionElegir` |
| Discriminación de confusiones | Se registran los pares confundidos (oído X, respondido Y). El modo **Repaso** practica solo los 3 más fallados y los dos caracteres de los 3 pares más confundidos | `MORSE_SesionRepaso` |
| Práctica distribuida | Aviso y resumen de la sesión a los 15 minutos | `SESION_MS` |
| Fluidez | **Grupos de 5**: el ejercicio clásico de Koch, que obliga a retener y encadenar caracteres como en una comunicación real | Modo Grupos |

Dos decisiones merecen una explicación aparte:

- **El Repaso no cuenta para la progresión Koch.** La muestra está sesgada a propósito hacia lo difícil; si contara, el porcentaje de aciertos bajaría artificialmente y frenaría el desbloqueo de caracteres que el alumno ya domina en condiciones normales.
- **Las confusiones en Grupos solo se anotan con el grupo completo.** La comparación es posición a posición, y una letra saltada desplazaría todas las siguientes, generando confusiones falsas.

## El orden Koch
El entrenador usa el orden de LCWO restringido a letras y números:

```text
K M U R E S N A P T L W I J Z F O Y V G 5 Q 9 2 H 3 8 B 4 7 C 1 D 6 0 X
```

El orden no es alfabético ni por frecuencia de uso, y responde a varios criterios:

- **Empezar con caracteres contrastados.** K (`-.-`) y M (`--`) tienen ritmos claramente distintos y suficientes símbolos para que el oído trabaje desde el primer día.
- **Separar en el tiempo los pares que se confunden.** Caracteres opuestos o parecidos (E/T, A/N, U/V, D/B) no se introducen seguidos, para que cada uno se asiente antes de que llegue su "gemelo".
- **Retrasar los patrones regulares.** Los números siguen una estructura sistemática (`1 = .----`, `2 = ..---`…) que invita a contar; aparecen en la segunda mitad del orden, cuando el oído ya reconoce ritmos y no necesita hacerlo.
- **Los caracteres de un solo símbolo no van primero.** E (`.`) y T (`-`) son tan cortos que, solos, apenas entrenan el reconocimiento de ritmo.

## Recepción y transmisión: por qué se responde manipulando
La mayoría de entrenadores piden la respuesta con un teclado. Este responde **manipulando en Morse**, por tres razones:

1. **Mantiene todo el ejercicio en el canal auditivo-motor.** Pasar por un teclado añade un paso visual (buscar la letra) que no existe en la tarea real.
2. **Entrena las dos habilidades a la vez.** Recepción (oír) y transmisión (manipular) se refuerzan mutuamente: producir el ritmo ayuda a fijarlo, y oír lo que uno ha producido junto al ritmo correcto es una comparación muy directa.
3. **Permite evaluar la calidad de la manipulación.** Un operador debe transmitir con un ritmo que otros puedan leer. El dispositivo mide la relación raya/punto (ideal ≈ 3) y los huecos entre símbolos (≈ 1 punto) y da indicaciones concretas: "Rayas cortas", "Rayas largas", "Pausas largas", "Letra partida".

La exigencia crece con el nivel (± 1 en la relación raya/punto al principio, ± 0,5 al final), pero el **decodificador sigue siendo tolerante**: la exigencia está en las indicaciones, no en dar por mala la respuesta. Así, un principiante con un ritmo imperfecto no recibe fallos por un problema de transmisión cuando lo que se está evaluando es si reconoció el carácter. Por el mismo motivo, si se manipula otro carácter no se da indicación de ritmo: es una confusión de oído, no de manipulación.

El decodificador también **se adapta a la velocidad del alumno** en lugar de exigirle la de reproducción: es habitual recibir a 18 WPM y transmitir más despacio al principio.

## Cómo usar el entrenador
Una pauta de uso coherente con el método:

1. **Sesiones de 10–15 minutos**, a ser posible a diario. El aviso de fin de sesión llega a los 15.
2. **Aprender** solo para conocer un carácter nuevo; la mayor parte del tiempo, **Identificar**.
3. **No contar.** Si hay que contar puntos para responder, es mejor fallar o dejar pasar y escuchar la corrección. Responder tarde cuenta como fallo a propósito.
4. **Ocultar el patrón** (opción Patrón) en cuanto se pueda prescindir de él.
5. **Grupos de 5** cuando Identificar resulte cómodo: entrena retener y encadenar.
6. **Repaso** al final de la sesión, para insistir en las confusiones del día.
7. **No bajar la velocidad de carácter** para ir más cómodo: si cuesta demasiado, es mejor bajar la Farnsworth.
8. Usar **Libre** para practicar la transmisión y comprobar la propia velocidad (`Tu N WPM`).

## Límites del método
- Koch exige tolerar un porcentaje de fallos alto al principio; para algunos alumnos es frustrante y el progreso parece lento en las primeras sesiones.
- El umbral del 90 % y la ventana de 20 intentos son una convención, no un valor demostrado; se pueden cambiar con `MORSE_KOCH_ACIERTOS` y `MORSE_KOCH_VENTANA`.
- Reconocer caracteres sueltos no es copiar texto: la copia de palabras y frases reales, y la copia "con retraso" (*copy behind*), son etapas posteriores que este dispositivo solo cubre en parte con los grupos de 5.
- El progreso no se guarda al apagar (pendiente para una versión posterior), así que el nivel se debe ajustar a mano en cada sesión desde la opción **Nivel**.

## Referencias
- Koch, L. (1936). Estudio original (en alemán) sobre el aprendizaje de la recepción de Morse por operadores de radio. Origen del método Koch.
- Bloom, J. (1990). *A Standard for Morse Timing Using the Farnsworth Technique*. QEX, ARRL. Fórmula de los silencios Farnsworth.
- Fitts, P. M. y Posner, M. I. (1967). *Human Performance*. Brooks/Cole. Fases del aprendizaje motor.
- Roediger, H. L. y Karpicke, J. D. (2006). *Test-Enhanced Learning*. Psychological Science, 17(3). Efecto test.
- [LCWO — Learn CW Online](https://lcwo.net). Implementación de referencia de Koch + Farnsworth y origen del orden de caracteres.
- [ARRL — Learning Morse Code](https://www.arrl.org/learning-morse-code).
