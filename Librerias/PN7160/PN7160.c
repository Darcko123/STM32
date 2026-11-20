/**
 * @file PN7160.c
 *
 * @brief Librería para el control del CI NFC PN7160 mediante SPI.
 *
 * @author Daniel Ruiz
 * @date Noviembre 19, 2026
 * @version 0.1.0
 */

#include "PN7160.h"

// ============================================================================
// MACROS PRIVADAS
// ============================================================================

/* -- Trama SPI (UM11495 / UM11577): byte de dirección antes del paquete NCI -- */
#define PN7160_SPI_WRITE        0x7FU   /**< Byte de dirección para escribir un paquete NCI */
#define PN7160_SPI_READ         0xFFU   /**< Byte de dirección para leer un paquete NCI */

/* -- Paquetes NCI -- */
#define PN7160_NCI_HEADER_LEN   3U      /**< Cabecera NCI: MT/GID, OID, longitud del payload */
#define PN7160_NCI_MAX_LEN      (PN7160_NCI_HEADER_LEN + 255U)  /**< Paquete NCI más largo posible */
#define PN7160_NCI_STATUS_OK    0x00U   /**< STATUS_OK en las respuestas NCI */

#define PN7160_MODEL_PN7160     0x61U   /**< Model ID del PN7160 en CORE_RESET_NTF */
#define PN7160_MODEL_PN7161     0x71U   /**< Model ID del PN7161 en CORE_RESET_NTF */

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static SPI_HandleTypeDef*   PN7160_hspi             = NULL; /**< Handle de SPI utilizado para comunicarse con el PN7160 */
static GPIO_TypeDef*        PN7160_NSS_Port         = NULL; /**< Puerto del pin NSS */
static uint16_t             PN7160_NSS_Pin          = 0U;   /**< Pin NSS */
static GPIO_TypeDef*        PN7160_VEN_Port         = NULL; /**< Puerto del pin VEN */
static uint16_t             PN7160_VEN_Pin          = 0U;   /**< Pin VEN */
static GPIO_TypeDef*        PN7160_IRQ_Port         = NULL; /**< Puerto del pin IRQ */
static uint16_t             PN7160_IRQ_Pin          = 0U;   /**< Pin IRQ */
static uint8_t              PN7160_Initialized      = 0U;   /**< Bandera para verificar si el módulo está inicializado */

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Traduce un estado del HAL al estado de la librería.
 *
 * @param hal Estado devuelto por el HAL.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
static PN7160_Status_t pn7160_FromHal(HAL_StatusTypeDef hal)
{
    if (hal == HAL_TIMEOUT) { return PN7160_TIMEOUT; }
    if (hal != HAL_OK)      { return PN7160_ERROR;   }

    return PN7160_OK;
}

/**
 * @brief Espera a que el PN7160 active IRQ (dato listo para leer).
 *
 * @return PN7160_Status_t PN7160_OK, o PN7160_TIMEOUT si IRQ no subió a tiempo.
 */
static PN7160_Status_t pn7160_WaitIrq(void)
{
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(PN7160_IRQ_Port, PN7160_IRQ_Pin) != GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start) >= PN7160_TIMEOUT_MS) { return PN7160_TIMEOUT; }
    }

    return PN7160_OK;
}

/**
 * @brief Envía un paquete NCI al PN7160.
 *
 * @param cmd Paquete NCI completo (cabecera + payload).
 * @param len Longitud del paquete en bytes.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
static PN7160_Status_t pn7160_Write(const uint8_t* cmd, uint16_t len)
{
    uint8_t dir = PN7160_SPI_WRITE;

    HAL_GPIO_WritePin(PN7160_NSS_Port, PN7160_NSS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef hal = HAL_SPI_Transmit(PN7160_hspi, &dir, 1U, PN7160_TIMEOUT_MS);
    if (hal == HAL_OK)
    {
        hal = HAL_SPI_Transmit(PN7160_hspi, (uint8_t*)cmd, len, PN7160_TIMEOUT_MS);
    }

    HAL_GPIO_WritePin(PN7160_NSS_Port, PN7160_NSS_Pin, GPIO_PIN_SET);

    return pn7160_FromHal(hal);
}

/**
 * @brief Espera a IRQ y lee un paquete NCI del PN7160.
 *
 * @param[out] buf Buffer de destino (al menos PN7160_NCI_MAX_LEN bytes).
 *
 * @return PN7160_Status_t Estado de la operación. El paquete leído mide
 *         PN7160_NCI_HEADER_LEN + buf[2] bytes.
 */
static PN7160_Status_t pn7160_Read(uint8_t* buf)
{
    PN7160_Status_t status = pn7160_WaitIrq();
    if (status != PN7160_OK) { return status; }

    uint8_t dir = PN7160_SPI_READ;

    HAL_GPIO_WritePin(PN7160_NSS_Port, PN7160_NSS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef hal = HAL_SPI_Transmit(PN7160_hspi, &dir, 1U, PN7160_TIMEOUT_MS);
    if (hal == HAL_OK)
    {
        hal = HAL_SPI_Receive(PN7160_hspi, buf, PN7160_NCI_HEADER_LEN, PN7160_TIMEOUT_MS);
    }
    if (hal == HAL_OK && buf[2] > 0U)
    {
        hal = HAL_SPI_Receive(PN7160_hspi, &buf[PN7160_NCI_HEADER_LEN], buf[2], PN7160_TIMEOUT_MS);
    }

    HAL_GPIO_WritePin(PN7160_NSS_Port, PN7160_NSS_Pin, GPIO_PIN_SET);

    return pn7160_FromHal(hal);
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el controlador NFC PN7160.
 *
 * @param hspi    Handle del SPI ya inicializado por CubeMX.
 * @param nssPort Puerto GPIO del pin NSS.
 * @param nssPin  Pin GPIO del pin NSS.
 * @param venPort Puerto GPIO del pin VEN.
 * @param venPin  Pin GPIO del pin VEN.
 * @param irqPort Puerto GPIO del pin IRQ.
 * @param irqPin  Pin GPIO del pin IRQ.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_Init(SPI_HandleTypeDef* hspi,
                            GPIO_TypeDef* nssPort, uint16_t nssPin,
                            GPIO_TypeDef* venPort, uint16_t venPin,
                            GPIO_TypeDef* irqPort, uint16_t irqPin)
{
    static const uint8_t coreReset[] = { 0x20U, 0x00U, 0x01U, 0x00U };        /* CORE_RESET_CMD, conserva la configuración */
    static const uint8_t coreInit[]  = { 0x20U, 0x01U, 0x02U, 0x00U, 0x00U }; /* CORE_INIT_CMD (NCI 2.0) */
    uint8_t rsp[PN7160_NCI_MAX_LEN];
    PN7160_Status_t status;

    // 1. Validar parámetros
    if(hspi == NULL || nssPort == NULL || venPort == NULL || irqPort == NULL)
    {
        return PN7160_INVALID_PARAM;
    }

    // 2. Re-Init: limpiar estado anterior. Se ignora a propósito: solo puede
    //    devolver NOT_INITIALIZED, y aquí sabemos que sí lo está.
    if (PN7160_Initialized)
    {
        (void)PN7160_DeInit();
    }

    PN7160_hspi     = hspi;
    PN7160_NSS_Port = nssPort;  PN7160_NSS_Pin = nssPin;
    PN7160_VEN_Port = venPort;  PN7160_VEN_Pin = venPin;
    PN7160_IRQ_Port = irqPort;  PN7160_IRQ_Pin = irqPin;

    // 3. Reset por hardware: VEN en bajo >= tWL(VEN) = 10 µs, luego en alto
    //    y esperar tboot = 2.5 ms máx. (HAL_Delay garantiza al menos el valor pedido)
    HAL_GPIO_WritePin(PN7160_NSS_Port, PN7160_NSS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PN7160_VEN_Port, PN7160_VEN_Pin, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(PN7160_VEN_Port, PN7160_VEN_Pin, GPIO_PIN_SET);
    HAL_Delay(PN7160_BOOT_MS);

    // 4. Descartar cualquier notificación pendiente tras el arranque
    if (HAL_GPIO_ReadPin(PN7160_IRQ_Port, PN7160_IRQ_Pin) == GPIO_PIN_SET)
    {
        status = pn7160_Read(rsp);
        if (status != PN7160_OK) { return status; }
    }

    // 5. CORE_RESET_CMD -> CORE_RESET_RSP (40 00 01 00)
    status = pn7160_Write(coreReset, sizeof(coreReset));
    if (status != PN7160_OK) { return status; }

    status = pn7160_Read(rsp);
    if (status != PN7160_OK) { return status; }
    if (rsp[0] != 0x40U || rsp[1] != 0x00U || rsp[2] < 1U || rsp[3] != PN7160_NCI_STATUS_OK)
    {
        return PN7160_ERROR;
    }

    // 6. CORE_RESET_NTF (60 00 ...): byte 9 = model ID, bytes 10-12 = versión de FW
    status = pn7160_Read(rsp);
    if (status != PN7160_OK) { return status; }
    if (rsp[0] != 0x60U || rsp[1] != 0x00U || rsp[2] < 9U ||
        (rsp[8] != PN7160_MODEL_PN7160 && rsp[8] != PN7160_MODEL_PN7161))
    {
        return PN7160_ERROR;
    }

    // 7. CORE_INIT_CMD -> CORE_INIT_RSP (40 01 ... con STATUS_OK)
    status = pn7160_Write(coreInit, sizeof(coreInit));
    if (status != PN7160_OK) { return status; }

    status = pn7160_Read(rsp);
    if (status != PN7160_OK) { return status; }
    if (rsp[0] != 0x40U || rsp[1] != 0x01U || rsp[2] < 1U || rsp[3] != PN7160_NCI_STATUS_OK)
    {
        return PN7160_ERROR;
    }

    // 8. Marcar como inicializado solo al final, tras un Init exitoso
    PN7160_Initialized = 1U;

    return PN7160_OK;
}

/**
 * @brief Desinicializa el controlador NFC PN7160 dejándolo en Hard Power Down.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_DeInit(void)
{
    if(!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }

    // VEN en bajo = Hard Power Down (10.5 µA típ.)
    HAL_GPIO_WritePin(PN7160_VEN_Port, PN7160_VEN_Pin, GPIO_PIN_RESET);

    PN7160_hspi         =   NULL;
    PN7160_Initialized  =   0U;

    return PN7160_OK;
}
