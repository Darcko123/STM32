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
 * @param timeoutMs Tiempo máximo de espera en ms.
 *
 * @return PN7160_Status_t PN7160_OK, o PN7160_TIMEOUT si IRQ no subió a tiempo.
 */
static PN7160_Status_t pn7160_WaitIrq(uint32_t timeoutMs)
{
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(PN7160_IRQ_Port, PN7    160_IRQ_Pin) != GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start) >= timeoutMs) { return PN7160_TIMEOUT; }
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
 * @param[out] buf       Buffer de destino (al menos PN7160_NCI_MAX_LEN bytes).
 * @param      timeoutMs Tiempo máximo de espera de IRQ en ms.
 *
 * @return PN7160_Status_t Estado de la operación. El paquete leído mide
 *         PN7160_NCI_HEADER_LEN + buf[2] bytes.
 */
static PN7160_Status_t pn7160_Read(uint8_t* buf, uint32_t timeoutMs)
{
    PN7160_Status_t status = pn7160_WaitIrq(timeoutMs);
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

/**
 * @brief Envía un comando NCI y valida su respuesta: mismo GID/OID con MT = RSP
 *        (0x2X -> 0x4X) y STATUS_OK en el primer byte del payload.
 *
 * @param      cmd Comando NCI completo.
 * @param      len Longitud del comando.
 * @param[out] rsp Buffer de respuesta (al menos PN7160_NCI_MAX_LEN bytes).
 *
 * @return PN7160_Status_t Estado de la operación.
 */
static PN7160_Status_t pn7160_Transceive(const uint8_t* cmd, uint16_t len, uint8_t* rsp)
{
    PN7160_Status_t status = pn7160_Write(cmd, len);
    if (status != PN7160_OK) { return status; }

    status = pn7160_Read(rsp, PN7160_TIMEOUT_MS);
    if (status != PN7160_OK) { return status; }

    if (rsp[0] != (cmd[0] ^ 0x60U) || rsp[1] != cmd[1] || rsp[2] < 1U || rsp[3] != PN7160_NCI_STATUS_OK)
    {
        return PN7160_ERROR;
    }

    return PN7160_OK;
}

/**
 * @brief Reinicia la pila NCI: CORE_RESET (con NTF y model ID), CORE_INIT y
 *        NCI_PROPRIETARY_ACT. Se usa en Init y tras cambiar configuración en EEPROM.
 *
 * @param[out] rsp Buffer de trabajo (al menos PN7160_NCI_MAX_LEN bytes).
 *
 * @return PN7160_Status_t Estado de la operación.
 */
static PN7160_Status_t pn7160_CoreReset(uint8_t* rsp)
{
    static const uint8_t coreReset[] = { 0x20U, 0x00U, 0x01U, 0x00U };        /* CORE_RESET_CMD, conserva la configuración */
    static const uint8_t coreInit[]  = { 0x20U, 0x01U, 0x02U, 0x00U, 0x00U }; /* CORE_INIT_CMD (NCI 2.0) */
    static const uint8_t propAct[]   = { 0x2FU, 0x02U, 0x00U };               /* NCI_PROPRIETARY_ACT_CMD (NXP) */

    // CORE_RESET_CMD -> CORE_RESET_RSP (40 00 01 00)
    PN7160_Status_t status = pn7160_Transceive(coreReset, sizeof(coreReset), rsp);
    if (status != PN7160_OK) { return status; }

    // CORE_RESET_NTF (60 00 ...): byte 9 = model ID, bytes 10-12 = versión de FW
    status = pn7160_Read(rsp, PN7160_TIMEOUT_MS);
    if (status != PN7160_OK) { return status; }
    if (rsp[0] != 0x60U || rsp[1] != 0x00U || rsp[2] < 9U ||
        (rsp[8] != PN7160_MODEL_PN7160 && rsp[8] != PN7160_MODEL_PN7161))
    {
        return PN7160_ERROR;
    }

    // CORE_INIT_CMD -> CORE_INIT_RSP (40 01 ... con STATUS_OK)
    status = pn7160_Transceive(coreInit, sizeof(coreInit), rsp);
    if (status != PN7160_OK) { return status; }

    // NCI_PROPRIETARY_ACT_CMD -> RSP (4F 02 ...): habilita las extensiones NXP
    // (protocolo MIFARE 0x80 e interfaz TAG-CMD)
    return pn7160_Transceive(propAct, sizeof(propAct), rsp);
}

/**
 * @brief Extrae el identificador del tag de los "RF Technology Specific
 *        Parameters" (modo sondeo, NCI 2.0).
 *
 *        - NFC-A: SENS_RES(2), longitud NFCID1, NFCID1(4/7/10), ...
 *        - NFC-B: longitud SENSB_RES, SENSB_RES sin 0x50 -> NFCID0 = bytes 1-4
 *        - NFC-F: bit rate, longitud SENSF_RES, SENSF_RES sin 0x01 -> NFCID2 = bytes 2-9
 *        - NFC-V: RES_FLAG, DSFID, UID(8)
 *
 * @param      tecnologia RF Technology and Mode.
 * @param      protocolo  RF Protocol.
 * @param      par        Inicio de los parámetros específicos.
 * @param      parLen     Longitud de los parámetros específicos.
 * @param[out] tag        Tag a rellenar.
 *
 * @return PN7160_Status_t PN7160_OK, o PN7160_ERROR si los parámetros no cuadran.
 */
static PN7160_Status_t pn7160_ParseTag(uint8_t tecnologia, uint8_t protocolo,
                                       const uint8_t* par, uint8_t parLen,
                                       PN7160_Tag_t* tag)
{
    uint8_t off = 0U;
    uint8_t len = 0U;

    switch (tecnologia)
    {
        case 0x00U: off = 3U; len = (parLen > 2U) ? par[2] : 0xFFU; break;  /* NFC-A */
        case 0x01U: off = 1U; len = 4U; break;                              /* NFC-B */
        case 0x02U:                                                         /* NFC-F */
        case 0x06U: off = 2U; len = 8U; break;                              /* NFC-V */
        default:    return PN7160_ERROR;
    }

    if (len > sizeof(tag->uid) || (uint16_t)off + len > parLen) { return PN7160_ERROR; }

    tag->tecnologia = tecnologia;
    tag->protocolo  = protocolo;
    tag->uidLen     = len;
    for (uint8_t i = 0U; i < len; i++) { tag->uid[i] = par[off + i]; }

    return PN7160_OK;
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
    static const uint8_t getClock[]  = { 0x20U, 0x03U, 0x03U, 0x01U, 0xA0U, 0x03U };   /* CORE_GET_CONFIG_CMD: CLOCK_SEL_CFG */
    static const uint8_t setClock[]  = { 0x20U, 0x02U, 0x05U, 0x01U, 0xA0U, 0x03U,     /* CORE_SET_CONFIG_CMD: CLOCK_SEL_CFG */
                                         0x01U, PN7160_CLOCK_SEL_CFG };
    static const uint8_t discMap[]   = { 0x21U, 0x00U, 0x07U, 0x02U,          /* RF_DISCOVER_MAP_CMD, 2 mapeos */
                                         0x04U, 0x01U, 0x02U,                 /* ISO-DEP, sondeo -> interfaz ISO-DEP */
                                         0x80U, 0x01U, 0x80U };               /* MIFARE, sondeo -> interfaz TAG-CMD */
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
        status = pn7160_Read(rsp, PN7160_TIMEOUT_MS);
        if (status != PN7160_OK) { return status; }
    }

    // 5. CORE_RESET + CORE_INIT + activación propietaria NXP
    status = pn7160_CoreReset(rsp);
    if (status != PN7160_OK) { return status; }

    // 6. CLOCK_SEL_CFG (A0 03): se guarda en la EEPROM del PN7160, así que solo se
    //    escribe si difiere (evita desgaste) y después se repite el paso 5 para aplicarlo.
    //    CORE_GET_CONFIG_RSP: 40 03 len | status, nParams, A0, 03, 01, valor
    status = pn7160_Transceive(getClock, sizeof(getClock), rsp);
    if (status != PN7160_OK) { return status; }
    if (rsp[2] < 6U || rsp[5] != 0xA0U || rsp[6] != 0x03U || rsp[7] != 1U)
    {
        return PN7160_ERROR;
    }

    if (rsp[8] != PN7160_CLOCK_SEL_CFG)
    {
        status = pn7160_Transceive(setClock, sizeof(setClock), rsp);
        if (status != PN7160_OK) { return status; }

        status = pn7160_CoreReset(rsp);
        if (status != PN7160_OK) { return status; }
    }

    // 7. RF_DISCOVER_MAP_CMD -> RSP (41 00 01 00). Los protocolos sin mapear
    //    (T1T, T2T, T3T, T5T) usan la interfaz Frame RF por defecto
    status = pn7160_Transceive(discMap, sizeof(discMap), rsp);
    if (status != PN7160_OK) { return status; }

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

/**
 * @brief Sondea NFC-A/B/F/V hasta detectar un tag o agotar timeoutMs.
 *
 * @param[out] tag       Datos del tag detectado.
 * @param      timeoutMs Tiempo máximo de espera de un tag, en ms.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_WaitTag(PN7160_Tag_t* tag, uint32_t timeoutMs)
{
    static const uint8_t rfDiscover[] = { 0x21U, 0x03U, 0x09U, 0x04U,   /* RF_DISCOVER_CMD, 4 configuraciones */
                                          0x00U, 0x01U,                 /* NFC-A sondeo pasivo, cada periodo */
                                          0x01U, 0x01U,                 /* NFC-B sondeo pasivo */
                                          0x02U, 0x01U,                 /* NFC-F sondeo pasivo */
                                          0x06U, 0x01U };               /* NFC-V sondeo pasivo */
    static const uint8_t rfDeactivate[] = { 0x21U, 0x06U, 0x01U, 0x00U };  /* RF_DEACTIVATE_CMD -> Idle */
    uint8_t rsp[PN7160_NCI_MAX_LEN];
    PN7160_Status_t status;

    // 1. Validar estado y parámetros
    if (!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }
    if (tag == NULL)
    {
        return PN7160_INVALID_PARAM;
    }

    // 2. RF_DISCOVER_CMD -> RF_DISCOVER_RSP (41 03 01 00)
    status = pn7160_Transceive(rfDiscover, sizeof(rfDiscover), rsp);
    if (status != PN7160_OK) { return status; }

    // 3. Esperar RF_INTF_ACTIVATED_NTF (61 05) o RF_DISCOVER_NTF (61 03); el resto se ignora
    //    61 05: [3] ID, [4] interfaz, [5] protocolo, [6] tecnología, [7] max payload, [8] créditos, [9] long., [10..] parámetros
    //    61 03: [3] ID, [4] protocolo, [5] tecnología, [6] long., [7..] parámetros
    uint32_t start = HAL_GetTick();
    for (;;)
    {
        uint32_t elapsed = HAL_GetTick() - start;
        if (elapsed >= timeoutMs) { status = PN7160_TIMEOUT; break; }

        status = pn7160_Read(rsp, timeoutMs - elapsed);
        if (status != PN7160_OK) { break; }

        if (rsp[0] == 0x61U && rsp[1] == 0x05U && rsp[2] >= 7U && rsp[2] >= 7U + rsp[9])
        {
            status = pn7160_ParseTag(rsp[6], rsp[5], &rsp[10], rsp[9], tag);
            break;
        }
        if (rsp[0] == 0x61U && rsp[1] == 0x03U && rsp[2] >= 4U && rsp[2] >= 4U + rsp[6])
        {
            status = pn7160_ParseTag(rsp[5], rsp[4], &rsp[7], rsp[6], tag);
            break;
        }
    }

    // 4. Volver siempre a RFST_IDLE: RF_DEACTIVATE_CMD -> RSP (41 06), ignorando
    //    las NTF que se crucen (p. ej. un tag activado justo al vencer el timeout)
    PN7160_Status_t stop = pn7160_Write(rfDeactivate, sizeof(rfDeactivate));
    while (stop == PN7160_OK)
    {
        stop = pn7160_Read(rsp, PN7160_TIMEOUT_MS);
        if (rsp[0] == 0x41U && rsp[1] == 0x06U) { break; }
    }
    if (stop == PN7160_OK && (rsp[2] < 1U || rsp[3] != PN7160_NCI_STATUS_OK))
    {
        stop = PN7160_ERROR;
    }

    // 5. Descartar RF_DEACTIVATE_NTF (61 06): llega si había un tag activado; desde
    //    RFST_DISCOVERY puede no llegar, así que su timeout no es un error
    if (stop == PN7160_OK)
    {
        (void)pn7160_Read(rsp, PN7160_TIMEOUT_MS);
    }

    return (status != PN7160_OK) ? status : stop;
}
