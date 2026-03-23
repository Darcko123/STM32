# STM32 Embedded Libraries and Examples
[![STM32](https://img.shields.io/badge/Platform-STM32-blue)](https://www.st.com/content/st_com/en.html)

¡Bienvenido al repositorio de **STM32 Embedded Libraries and Examples**! Aquí encontrarás una colección de librerías y ejemplos prácticos para facilitar el desarrollo de aplicaciones embebidas basadas en microcontroladores STM32.

## 📚 **Descripción General**

Este repositorio está dividido en cuatro secciones principales:

- **Libraries/**: Contiene librerías personalizadas para diferentes periféricos y módulos externos, como sensores, RTC, y más.
- **LibrariesExamples/**: Incluye ejemplos prácticos sencillos que demuestran cómo utilizar cada una de las librerías.
- **Projects/**: Incluye proyectos realizados con diversos microcontroladores STM32 con aplicaciones en sistemas embebidos.
- **PerifericosBasicos/**: Incluye ejemplos y explicaciones de cómo usar perifericos como Timers, Interrupciones, ADCs, etc.

---

## 📁 **Estructura del Repositorio**

```
STM32/
├── Libraries/       # Librerías para periféricos y módulos externos
│   ├── DS3231/      # Librería para módulo de tiempo real
│   ├── HC05/        # Librería para módulo bluetooth
│   ├── MAX7219/     # Librería para el control de una matriz de leds
│   ├── MPU950/      # Librería para acelerómetro
│   ├── PCA9685/     # Librería para módulo PWM de 16 canales por I2C
│   ├── SI7021/      # Librería para sensor de temperatura y humedad
│   └── SX1262/      # Librería para módulo de radiofrecuencia LoRa
│
├── LibrariesExamples/        # Ejemplos prácticos de uso de las librerías
│   ├── MAX7219_Example/
│   ├── MPU6050_Example/
│   ├── PCA9685_Example/
│   ├── RTC_Example/
│   └── SI7021_Example/
│
├── Projects/        #Proyectos varios
│   ├── 3-band_Equalizer/
│   ├── 4DOF_RobotArm
│   ├── ClockWithMenu/
│   ├── KeypadWithAdvancedDisplay/
│   └── UARTToMorseCode/
│
├── PerifericosBasicos/        #Uso de périfericos básicos
│   ├── ADC/
│   │   ├── ADC_Polling
│   │   ├── ADC_Interrupt
│   │   ├── ADC_DMA
│   │   └── ADC_Multichannel
│   ├── ButtonToggleLed/
│   ├── HelloWorld/
│   ├── MatrixKeypad
│   ├── ServomotorPotentiometer/
│   ├── ServomotorWithPWM/
│   ├── StepperWithTimer/
│   ├── TimerInterrupt/
│   └── UART/
│
├── LICENSE          # Licencia del proyecto
├── README.md        # Este archivo
└── .gitignore

```

---

## ⚙️ **Requisitos Previos**

- STM32CubeIDE o cualquier entorno compatible con STM32.
- Microcontrolador STM32 (dependiendo del ejemplo o librería).
- Conocimiento básico de C y sistemas embebidos.

---

## 🚀 **Cómo Empezar**

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

---

## 📄 **Licencia**

Este proyecto está bajo la licencia **MIT**. Consulta el archivo [LICENSE](./LICENSE.md) para más detalles.

---

## 📞 **Contacto**

Si tienes alguna duda o sugerencia, no dudes en ponerte en contacto:
- **Autor:** Daniel Ruiz
- **Correo:** daniel18052002@yahoo.com


