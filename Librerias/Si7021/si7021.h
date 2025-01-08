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
 * @brief Handler externo para la comunicación I2C.
 *
 * Asegúrate de que el handler I2C (hi2c1) está correctamente configurado
 * en tu archivo principal (main.c) antes de llamar a las funciones de esta librería.
 */
extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Inicializa el sensor SI7021.
 *
 * Envía un comando de reinicio al sensor SI7021 para asegurar su correcto funcionamiento.
 * Esta función debe ser llamada antes de leer datos del sensor.
 *
 * @note La comunicación se realiza a través de I2C.
 */
void si7021_init(void);

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
void get_si7021(float *temp, float *humid);

#endif /* __SI7021_H */
