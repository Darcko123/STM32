/**
 * @file PN7160.h
 *
 * @brief Librería para el control del CI NFC PN7160 mediante SPI.
 *
 * @author Daniel Ruiz
 * @date Noviembre 19, 2026
 * @version 0.1.0
 */

#ifndef PN7160_H
#define PN7160_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdint.h>

// ============================================================================
// MACROS Y CONSTANTES PN7160
// ============================================================================

#ifndef PN7160_TIMEOUT_MS
#define PN7160_TIMEOUT_MS   100U    /**< Timeout de SPI y de espera de IRQ en ms (redefinible antes de incluir) */
#endif

#ifndef PN7160_BOOT_MS
#define PN7160_BOOT_MS      3U      /**< Espera tras subir VEN en ms; tboot = 2.5 ms máx. (redefinible antes de incluir) */
#endif

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno del PN7160.
 */
typedef enum {
    PN7160_OK               = 0,    /**< Operación exitosa */
    PN7160_ERROR            = 1,    /**< Error en la operación */
    PN7160_TIMEOUT          = 2,    /**< Timeout en la operación */
    PN7160_NOT_INITIALIZED  = 3,    /**< Módulo no inicializado */
    PN7160_INVALID_PARAM    = 4     /**< Parámetro inválido */
} PN7160_Status_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el controlador NFC PN7160 y deja la pila NCI lista para usarse.
 *
 * @details Secuencia:
 *          1. Reset por hardware: VEN en bajo al menos tWL(VEN) = 10 µs (estado
 *             Hard Power Down) y después en alto.
 *          2. Espera del arranque del firmware: PN7160_BOOT_MS (tboot = 2.5 ms
 *             máximo). Si IRQ ya está en alto, se descarta el paquete pendiente.
 *          3. CORE_RESET_CMD (20 00 01 00) -> CORE_RESET_RSP (40 00 01 00) ->
 *             CORE_RESET_NTF. Del NTF se valida el model ID (byte 9: 0x61 = PN7160,
 *             0x71 = PN7161); los bytes 10 a 12 traen la versión de firmware.
 *          4. CORE_INIT_CMD (20 01 02 00 00) -> CORE_INIT_RSP (NCI 2.0).
 *
 *          Trama SPI (UM11495 / UM11577, no viene en el datasheet): 0x7F antes de
 *          escribir un paquete NCI y 0xFF antes de leerlo, ambos con NSS en bajo.
 *
 *          Cada respuesta se espera a que IRQ pase a alto (activo en alto por
 *          defecto) antes de leerla. Si ya estaba inicializado, desinicializa y
 *          vuelve a inicializar.
 *
 *          Configuración previa (p. ej. en STM32CubeMX):
 *          - SPI en modo maestro, modo 0 (CPOL = 0, CPHA = 0), MSB primero, 8 bits
 *            y SCK <= 7 MHz (periodo mínimo de 142 ns).
 *          - NSS y VEN como salidas push-pull; IRQ como entrada.
 *          - VBAT y VDD(PAD) deben estar presentes antes de subir VEN.
 *          - DWL_REQ en bajo; en alto el chip arranca en modo descarga de firmware.
 *
 * @param hspi    Handle del SPI ya inicializado por CubeMX.
 * @param nssPort Puerto GPIO del pin NSS (HIF1).
 * @param nssPin  Pin GPIO del pin NSS.
 * @param venPort Puerto GPIO del pin VEN (reset / Hard Power Down).
 * @param venPin  Pin GPIO del pin VEN.
 * @param irqPort Puerto GPIO del pin IRQ.
 * @param irqPin  Pin GPIO del pin IRQ.
 *
 * @return PN7160_OK si la inicialización fue exitosa.
 * @return PN7160_INVALID_PARAM si hspi o algún puerto GPIO es NULL.
 * @return PN7160_TIMEOUT si IRQ no se activó a tiempo o el bus SPI no respondió.
 * @return PN7160_ERROR si falla el bus, una respuesta NCI no es la esperada o el
 *         model ID no corresponde a un PN7160/PN7161.
 */
PN7160_Status_t PN7160_Init(SPI_HandleTypeDef* hspi,
                            GPIO_TypeDef* nssPort, uint16_t nssPin,
                            GPIO_TypeDef* venPort, uint16_t venPin,
                            GPIO_TypeDef* irqPort, uint16_t irqPin);

PN7160_Status_t PN7160_DeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* PN7160_H */
