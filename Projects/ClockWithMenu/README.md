# Proyecto: Reloj con Sistema de Menús con STM32

## Descripción
Este proyecto implementa un sistema de menús interactivo utilizando un microcontrolador STM32. El sistema permite la configuración de hora, fecha, alarmas y visualización de datos ambientales (temperatura y humedad). La interfaz con el usuario se realiza a través de una pantalla LCD y botones físicos.

## Características
- **Sistema de Menús**:
  - Navegación por menús jerárquicos.
  - Configuración de parámetros como hora, fecha y alarmas.
- **Medición Ambiental**:
  - Lectura de temperatura y humedad mediante un sensor SI7021.
- **Interfaz de Usuario**:
  - Pantalla LCD para mostrar información.
  - Control mediante botones físicos (incremento, decremento y selección).
- **Señal SIG y Alarmas**:
  - Generación de señales PWM para alarmas o notificaciones visuales.

## Estructura del Código
El código está dividido en varias secciones, cada una con su función específica:

1. **Configuración de Hardware**:
   - Inicialización de periféricos como GPIO, I2C y temporizadores.
2. **Menús**:
   - Definición de matrices de opciones y lógica de navegación.
3. **Lógica Principal**:
   - Bucle infinito para la actualización de pantalla, manejo de entradas y ejecución de acciones.
4. **Funciones Auxiliares**:
   - Cálculo del día de la semana (algoritmo de Zeller).
   - Control del estado del retroiluminado de la pantalla.
   - Generación de señales PWM según condiciones.

## Requisitos
- **Hardware**:
  - Microcontrolador STM32. 
  - Pantalla LCD de 4 filas controlado por módulo de I2C
  - Sensor SI7021 para medición de temperatura y humedad.
  - Módulo de RTC DS3231 para el control de la hora y las alarmas.
  - Botones físicos para la navegación.
- **Software**:
  - STM32CubeIDE para la compilación y carga del código.

## Cómo Usar
1. **Preparación del Hardware**:
   - Conecta la pantalla LCD al microcontrolador mediante el bus I2C.
   - Conecta el sensor SI7021 a los pines I2C correspondientes.
   - Conecta el módulo RTC DS3231 al mismo bus I2C.
   - Realiza la conexión de los botones en modo Pull-down. en los pines correspondientes.
2. **Compilación y Programación**:
   - Abre el proyecto en STM32CubeIDE.
   - Compila y carga el programa en el microcontrolador.
3. **Operación**:
   - Usa los botones para navegar por los menús y configurar los parámetros deseados.
   - Observa los datos actualizados en la pantalla LCD.

## Estructura de Archivos
- `main.c`: Código principal del sistema.
- `RTC.h`: Control del reloj en tiempo real.
- `SI7021.h`: Control del sensor de temperatura y humedad.
- `liquidcrystal_i2c.h`: Controlador de la pantalla LCD.

## Notas
- La configuración inicial del tiempo en el RTC está comentada en el código; descomentar si es necesario.
- El código está documentado para facilitar su comprensión y mantenimiento.
- Es necesario tener un microcontrolador con al menos 64 Kbytes de memoria flash ya que debido al uso de buffers, se llena la memoria. Es por eso que un microcontrolador `STM32F103C6` no será suficiente, en su lugar, se recomienda el uso uso de un microcontrolador `STM32F103C8`

## Licencia
Este proyecto está licenciado bajo los términos que se encuentran en el archivo `LICENSE.md` en el directorio raíz del proyecto.

---


