# Reloj con Sistema de Menús con STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F103-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)
[![I2C](https://img.shields.io/badge/Protocol-I2C-yellow)](https://es.wikipedia.org/wiki/I2C)

## Descripción
Sistema de reloj inteligente con interfaz de menús interactiva desarrollado para microcontrolador STM32. Incluye funcionalidades completas de reloj, alarma, monitorización ambiental y sistema de navegación mediante botones físicos.

## Características Principales

### ⏰ Funcionalidades de Reloj
- **Reloj en Tiempo Real**: Implementado con módulo RTC DS3231 de alta precisión
- **Visualización Completa**: Hora, fecha y día de la semana
- **Algoritmo de Zeller**: Cálculo automático del día de la semana
- **Manejo de Años Bisiestos**: Configuración automática de febrero
  
### 🌡️ Monitoreo Ambiental
- **Sensor SI7021**: Medición de temperatura y humedad ambiental
- **Actualización en Tiempo Real**: Datos refrescados cada segundo
- **Visualización Simultánea**: Temperatura en °C y humedad relativa en %

### 🎚️ Sistema de Menús
- **Navegación Jerárquica**: 4 niveles de menús organizados
- **Interfaz Intuitiva**: Indicador visual de selección (▶)
- **Control por Botones**: 
  - `+` Incrementar/Navegar arriba
  - `-` Decrementar/Navegar abajo
  - `OK` Seleccionar/Confirmar

### ⚙️ Funcionalidades Avanzadas
- **Sistema de Alarmas**: Configuración múltiple con señalización audible
- **Señal SIG**: Notificación horaria mediante PWM
- **Retroiluminación**: Control ON/OFF del backlight LCD
- **Anti-rebote**: Software debounce para botones

## Hardware Requerido
- **Microcontrolador**: STM32F103C8T6 (Blue Pill)
- **Pantalla**: LCD 20x4 con interfaz I2C
- **RTC**: Módulo DS3231 (Reloj en Tiempo Real)
- **Sensor**: SI7021 (Temperatura y Humedad)
- **Botones**: 3 pulsadores (Incremento, Decremento, OK)
- **Programador**: ST-Link V2

### Esquema de Conexiones
I2C Bus (SDA/SCL):
- LCD I2C(0x27)
- DS32311 RTC (0xD0)
- SI7021 Sensor (0x80)

Botones:
- ButtonPlus: PA10
- ButtonMinus: PA9
- ButtonOk: PA8
- LCDLed: PB10(Retroilumincación)

PWM Output:
- TIM2_CH1: PA0 (Señal de alarma)

## Estructura del Software

### Arquitectura del Código
```plaintext
main.c
├── Sistema de Menús (4 niveles)
├── Gestión de Periféricos
├── Lógica de Reloj y Alarmas
└── Monitorización Ambiental
```

Librerías:
- RTC.h: Control del reloj DS3231
- liquidcrystal_i2c.h: Driver LCD I2C
- SI7021.h: Sensor temperatura/humedad

## Sistema de menús
```C
// Estructura jerárquica de menús
Nivel 1: {"Configuracion", "Alarma", "Atras"}
Nivel 2: {"Hora", "Fecha", "Atras"} / {"Encendido", "Configuracion", "SIG", "Atras"}
Nivel 3: Opciones específicas de configuración
Nivel 4: Ajustes finos (++, --, Reset)
```

## Configuración del proyecto
### Requisitos de software
- STM32CubeIDE v1.8.0 o superior
- STM32CubeMX para configuración de periféricos
- Drivers: STM32F4xx HAL Library

### Pasos de Configuración
1. Importar proyecto
   ```bash
   File > Import > Existing Projects into Workspace
   ``` 

2. Generar Código
   1. Usar Device Configuration Tool
   2. Mantener código entre bloques USER CODE BEGIN/END
  
## Compilación y Programación
### Compilar proyecto
```bash
Project > Build All (Ctrl+B)
```
### Programar microcontrolador
1. Conectar ST-Link V2
2. Run > Debug (F11)
3. Run > Resume (F8) para ejecutar

## Uso del Sistema
### Navegación Básica
1. Pantalla Principal: Muestra hora, fecha, temperatura y humedad
2. Presionar OK: Acceder al menú principal
3. Botones +/-: Navegar entre opciones
4. OK: Seleccionar opción
5. Atras: Volver al menú anterior

### Configuración de Hora/Fecha
1. Menú → Configuración → Hora/Fecha
2. Seleccionar parámetro a ajustar
3. Usar ++/-- para modificar valores
4. OK para confirmar cambios

### Gestión de Alarmas
1. Menú → Alarma → Configuración
2. Establecer hora de alarma
3. Activar/desactivar con Encendido
4. Configurar señal SIG para notificaciones

## Estructura de Archivos
```plaintext
Project/
├── Core/
│   ├── Src/
│   │   ├── main.c              # Código principal
│   │   ├── RTC.c               # Control reloj tiempo real
│   │   ├── SI7021.c            # Driver sensor ambiental
│   │   └── liquidcrystal_i2c.c # Controlador LCD
│   └── Inc/                    # Headers correspondientes
├── Drivers/                    # Librerías HAL STM32
└── STM32CubeMX/                # Configuración de periféricos
```

## Optimizaciones Implementadas
### Gestión de Memoria
- Buffers Optimizados: Uso eficiente de arrays estáticos
- Variables Volátiles: Para datos compartidos con ISRs
- Manejo de Cadenas: sprintf optimizado para LCD
- 
### Rendimiento
- Actualización Selectiva: Solo redibuja LCD cuando hay cambios
- Debounce Software: 250ms para botones
- Polling Eficiente: Lectura no bloqueante de sensores

### Consumo Energético
- Control Backlight: Encendido/apagado manual

## Solución de Problemas
### Errores Comunes
1. LCD no muestra nada
   1. Verificar conexiones I2C
   2. Comprobar dirección I2C (0x27)

2. Hora no se mantiene
   1. Revisar batería del RTC DS3231
   2. Verificar configuración RTC

3. Botones no responden
   1. Comprobar configuración GPIO pull-down
   2. Verificar debounce software

4. Sensor no lee valores
   1. Confirmar conexión I2C del SI7021
   2. Verificar dirección (0x80)

## Personalización
### Añadir Nuevas Funcionalidades
1. Extender Menús:
   ```C
   char *menu_nuevo[] = {"Opcion1", "Opcion2", "Atras"};
   ```

2. Nuevos Sensores:
   1. Implementar drivers en carpeta Drivers/
   2. Añadir lectura en loop principal

3. Modificar Interfaz:
   1. Editar funciones en fn_menu()
   2. Ajustar printCurrentValue()

### Optimizaciones Adicionales
- Interrupciones: Migrar botones a EXTI
- DMA: Para transferencias I2C
- FreeRTOS: Para multitarea

## Consideraciones Técnicas
### Requisitos de Memoria
- Flash: ≥64KB (Recomendado STM32F411CEU6)
  >[!Note]
  > - Se requiere un microcontrolador con una memoria Flash de al menos 64 Kbytes, ya que el uso de buffers consume una cantidad significativa de espacio de almacenamiento. Por esta razón, el microcontrolador `STM32F103C6` resulta insuficiente para esta aplicación. En su lugar, se recomienda utilizar el modelo `STM32F103C8`.
- RAM: ≥16KB para buffers y variables
- EEPROM: Para configuración persistente (opcional)

### Timing Crítico
- RTC: Actualización cada 1 segundo
- Sensor: Lectura cada 1 segundo
- Botones: Debounce 250ms
- PWM: Frecuencias 7.1KHz y 8.7KHz para alarmas

## Licencia
Este proyecto está licenciado bajo los términos que se encuentran en el archivo `LICENSE.md` en el directorio raíz del proyecto.

---