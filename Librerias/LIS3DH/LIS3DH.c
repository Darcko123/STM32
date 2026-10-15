/**
 * @file LIS3DH.c
 * @brief Implementación de funciones para el manejo del acelerómetro LIS3DH
 * 
 * @details Este archivo contiene las definiciones de las funciones para inicializar
 *          y leer datos del acelerómetro LIS3DH mediante la interfaz 
 *          I2C en un microcontrolador STM32.
 * 
 * @author   Daniel Ruiz
 * @date     Octubre 15, 2026
 * @version  1.0.0
 */

#include "LIS3DH.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static I2C_HandleTypeDef*   LIS3DH_hi2c        = NULL; /**< Manejador de la interfaz I2C utilizado para comunicarse con el LIS3DH */
static uint8_t              LIS3DH_Initialized = 0U;   /**< Bandera para verificar si el sensor está inicializado */

/** @brief Almacenan valores RAW del acelerómetro. */
static int16_t Accel_X_RAW = 0, Accel_Y_RAW = 0, Accel_Z_RAW = 0;

/** @brief Almacenan valores RAW del giroscopio. */
static int16_t Gyro_X_RAW = 0, Gyro_Y_RAW = 0, Gyro_Z_RAW = 0;