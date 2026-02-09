# DAC en STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32F446-black)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![ADC](https://img.shields.io/badge/Protocol-ADC-yellow)](https://es.wikipedia.org/wiki/Convertidor_anal%C3%B3gico-digital)

Este documento presenta una guía completa para la implementación del **DAC (Digital-to-Analog Converter)** en microcontroladores STM32, utilizando **STM32CubeIDE**.
Incluye explicaciones detalladas sobre la configuración del periférico y ejemplos prácticos que facilitan la comprensión y aplicación del DAC en proyectos embebidos. Además, se realiza una comparación entre los modos de operación bloqueante, no bloqueante e interrupción.

---

## Tabla de Contenidos

- [DAC en STM32](#dac-en-stm32)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción General](#descripción-general)
  - [Características Principales](#características-principales)
  - [Componentes Requeridos](#componentes-requeridos)
  - [Polling](#polling)

---

## Descripción General
El objetivo de este tutorial es guiar paso a paso en la configuración del entorno de desarrollo, la generación del código base mediante **STM32CubeIDE**, y la implementación de diferentes métodos de uso del DAC utilizando las funciones de la capa de abstracción de hardware (HAL).

---

## Características Principales

- ✅ Configuración completa del protocolo DAC.
- ✅ Ejemplos funcionales de transmisión.
- ✅ Comparación entre modos bloqueantes, no bloqueantes e interrupciones.
- ✅ Compatible con cualquier microcontrolador STM32 que disponga de DAC.

---

## Componentes Requeridos

- **STM32CubeIDE**: [Descargar desde STMicroelectronics](https://www.st.com/en/development-tools/stm32cubeide.html)
- **Placa de desarrollo**: STM32F446 o compatible.  
- **Programador/depurador**: ST-Link V2.  
- **Convertidor TTL a USB** (cualquier modelo es válido).

--- 

## Polling