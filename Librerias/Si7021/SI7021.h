/*
 * si7021.h
 *
 * @brief Librería para el sensor de temperatura y humedad SI7021 utilizando I2C en STM32.
 *
 * @author Daniel Ruiz
 * @date Dec 28, 2024
 * @version 1.0
 */

#ifndef __SI7021_H
#define __SI7021_H

#include "stm32f1xx_hal.h"

/**
 * @brief Inicializa el sensor SI7021.
 *
 * Envía un comando de reset al sensor a través de I2C para asegurar que está en un estado conocido.
 *
 * @param hi2c Puntero al manejador de la interfaz I2C previamente configurado.
 * 
 * @note Esta función debe ser llamada antes de realizar lecturas de temperatura y humedad.
 */
void SI7021_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief Lee los valores de temperatura y humedad del sensor SI7021.
 *
 * @param[out] temp  Puntero a una variable donde se almacenará la temperatura en grados Celsius (°C).
 * @param[out] humid Puntero a una variable donde se almacenará la humedad relativa (%HR).
 *
 * @note Esta función realiza la lectura secuencial de los datos del sensor.
 *
 * @attention Asegúrate de llamar a `si7021_init` antes de usar esta función.
 */
void SI7021_Get(float *temp, float *humid);

#endif /* __SI7021_H */
