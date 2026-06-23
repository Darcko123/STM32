# STM32 Embedded Libraries and Examples
[![STM32](https://img.shields.io/badge/Platform-STM32-blue)](https://www.st.com/content/st_com/en.html)

¡Bienvenido al repositorio de **STM32 Embedded Libraries and Examples**! Aquí encontrarás una colección de librerías y ejemplos prácticos para facilitar el desarrollo de aplicaciones embebidas basadas en microcontroladores STM32.

## **Descripción General**

Este repositorio está dividido en cuatro secciones principales:

- **Librerias/**: Contiene librerías personalizadas para diferentes periféricos y módulos externos, como sensores, RTC, pantallas, módulos de radiofrecuencia y más.
- **ExamplesLibraries/**: Incluye ejemplos prácticos sencillos que demuestran cómo utilizar cada una de las librerías.
- **Projects/**: Incluye proyectos realizados con diversos microcontroladores STM32 con aplicaciones en sistemas embebidos.
- **PerifericosBasicos/**: Incluye ejemplos y explicaciones de cómo usar periféricos como Timers, Interrupciones, ADCs, etc.

---

## **Librerías Disponibles**

| Librería | Versión | Descripción |
|----------|---------|-------------|
| [AHT10](./Librerias/AHT10/) | v1.0.0 | Sensor de temperatura y humedad (I2C) |
| [ANSII_Codes](./Librerias/ANSII_Codes/) | v1.0.0 | Códigos de escape ANSI para formato de salida por terminal/UART |
| [BMP280](./Librerias/BMP280/) | v1.0.0 | Sensor de presión barométrica y temperatura (I2C) |
| [CME6005](./Librerias/CME6005/) | v0.1.0 | Receptor de señal horaria WWVB / 60 kHz |
| [DHT11](./Librerias/DHT11/) | v1.0.0 | Sensor de temperatura y humedad |
| [DS3231 (RTC_Module)](./Librerias/DS3231\(RTC_Module\)/) | v2.1.0 | Módulo de reloj de tiempo real (RTC) |
| [HC05](./Librerias/HC05/) | v0.1.0 | Módulo Bluetooth |
| [ILI9341_Disc1](./Librerias/ILI9341_Disc1/) | v1.2.0 | Pantalla TFT LCD ILI9341 para STM32F429-Discovery |
| [MAX7219](./Librerias/MAX7219/) | v2.1.0 | Control de matrices de LEDs y displays de 7 segmentos |
| [MPU6050](./Librerias/MPU6050/) | v2.1.0 | Acelerómetro y giroscopio de 6 ejes |
| [NEO_6M](./Librerias/NEO_6M/) | v1.1.0 | Módulo GPS NEO-6M |
| [PWM_Module (PCA9685)](./Librerias/PCA9685/) | v2.1.0 | Módulo PWM de 16 canales por I2C |
| [Si7021](./Librerias/Si7021/) | v2.1.0 | Sensor de temperatura y humedad (I2C) |
| [SX1262](./Librerias/SX1262/) | v1.6.0 | Módulo de radiofrecuencia LoRa |

---

## **Estructura del Repositorio**

```
STM32/
├── Librerias/               # Librerías para periféricos y módulos externos
│   ├── AHT10/               # Sensor de temperatura y humedad
│   ├── ANSII_Codes/         # Códigos de escape ANSI
│   ├── BMP280/              # Sensor de presión y temperatura
│   ├── CME6005/             # Receptor de señal horaria WWVB/60 kHz
│   ├── DHT11/               # Sensor de temperatura y humedad
│   ├── DS3231(RTC_Module)/  # Módulo de reloj de tiempo real
│   ├── HC05/                # Módulo bluetooth
│   ├── ILI9341_Disc1/       # Pantalla TFT LCD para STM32F429-Discovery
│   ├── MAX7219/             # Control de matrices de LEDs / 7 segmentos
│   ├── MPU6050/             # Acelerómetro y giroscopio
│   ├── NEO_6M/              # Módulo GPS
│   ├── PWM_Module/          # Módulo PWM PCA9685 de 16 canales por I2C
│   ├── Si7021/              # Sensor de temperatura y humedad
│   └── SX1262/              # Módulo de radiofrecuencia LoRa
│
├── ExamplesLibraries/       # Ejemplos prácticos de uso de las librerías
│   ├── MAX7219_Example/
│   ├── MPU6050_Example/
│   ├── PCA9685_Example/
│   ├── RTC_Example/
│   ├── SI7021_Example/
│   └── SX1262/
│
├── Projects/                # Proyectos varios
│   ├── 3-band_Equalizer/
│   ├── 4DOF_RobotArm/
│   ├── ClockWithMenu/
│   ├── KeypadWithAdvancedDisplay/
│   └── UARTToMorseCode/
│
├── PerifericosBasicos/      # Uso de periféricos básicos
│   ├── ADC/
│   │   ├── ADC_Polling/
│   │   ├── ADC_Interrupt/
│   │   ├── ADC_DMA/
│   │   └── ADC_DMA_Multichannel/
│   ├── ButtonToggleLed/
│   ├── HelloWorld/
│   ├── MatrixKeypad/
│   ├── ServomotorPotentiometer/
│   ├── ServomotorWithPWM/
│   ├── StepperWithTimer/
│   ├── TimerInterrupt/
│   └── UART/
│
├── LICENSE.md               # Licencia del proyecto
├── README.md                # Este archivo
└── .gitignore
```

---

## **Requisitos Previos**

- STM32CubeIDE o cualquier entorno compatible con STM32.
- Microcontrolador STM32 (dependiendo del ejemplo o librería).
- Conocimiento básico de C y sistemas embebidos.

---

## **Cómo Empezar**

1. **Clona el repositorio:**
   ```bash
   git clone https://github.com/Darcko123/STM32.git
   ```
2. **Navega a la carpeta deseada:**
   ```bash
   cd STM32/.../...
   ```
3. **Abre el ejemplo correspondiente en tu entorno de desarrollo.**
4. **Compila y sube el código a tu microcontrolador STM32.**

---

## **Licencia**

Este proyecto está bajo la licencia **MIT**. Consulta el archivo [LICENSE](./LICENSE.md) para más detalles.

---

## **Contacto**

Si tienes alguna duda o sugerencia, no dudes en ponerte en contacto:
- **Autor:** Daniel Ruiz
- **Correo:** daniel18052002@yahoo.com
