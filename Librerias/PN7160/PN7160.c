/**
 * @file PN7160.c
 *
 * @brief Librería para el control del CI NFC PN7160 mediante SPI o I2C.
 *
 * @author Daniel Ruiz
 * @date Noviembre 19, 2026
 * @version 0.1.0
 */

#include "PN7160.h"
#include <string.h>

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
#define PN7160_NCI_PBF          0x10U   /**< Packet Boundary Flag: 1 = siguen más segmentos */
#define PN7160_NCI_RF_TIMEOUT   0xB2U   /**< STATUS_RF_TIMEOUT_EXCEPTION en CORE_INTERFACE_ERROR_NTF */
#define PN7160_CREDITS_OFF      0xFFU   /**< Créditos = 0xFF: control de flujo deshabilitado */

#define PN7160_MODEL_PN7160     0x61U   /**< Model ID del PN7160 en CORE_RESET_NTF */
#define PN7160_MODEL_PN7161     0x71U   /**< Model ID del PN7161 en CORE_RESET_NTF */
#define PN7160_CARD_UID_UNSET   0xFFU   /**< PN7160_CardUidLen: nunca se llamó a SetCardUid, no se envía LA_NFCID1 */

/* -- TAG-CMD de MIFARE Classic (propietario NXP, UM11495) -- */
#define PN7160_MFC_XCHG         0x10U   /**< Intercambio de datos crudo (el PN7160 añade/quita el CRC y el cifrado Crypto1) */
#define PN7160_MFC_AUTH         0x40U   /**< Autenticación de un sector */
#define PN7160_MFC_KEY_EMBEDDED 0x10U   /**< Key selector: la clave va en el comando (bit 7 = 1 -> clave B) */
#define PN7160_MFC_KEY_B        0x80U   /**< Key selector: clave B en lugar de A */
#define PN7160_MFC_READ         0x30U   /**< Comando MIFARE READ (16 bytes) */
#define PN7160_MFC_WRITE        0xA0U   /**< Comando MIFARE WRITE (dos fases: dirección y 16 bytes) */
#define PN7160_MFC_MAX_SECTOR   39U     /**< Último sector de una Classic 4K (1K: 0-15) */

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

/* -- Transporte: solo existen las variables del bus seleccionado -- */
#if defined(PN7160_TRANSPORT_SPI)
static SPI_HandleTypeDef*   PN7160_hspi             = NULL; /**< Handle de SPI utilizado para comunicarse con el PN7160 */
static GPIO_TypeDef*        PN7160_NSS_Port         = NULL; /**< Puerto del pin NSS */
static uint16_t             PN7160_NSS_Pin          = 0U;   /**< Pin NSS */
#elif defined(PN7160_TRANSPORT_I2C)
static I2C_HandleTypeDef*   PN7160_hi2c             = NULL; /**< Handle de I2C utilizado para comunicarse con el PN7160 */
#endif

/* -- Comunes a ambos transportes -- */
static GPIO_TypeDef*      PN7160_VEN_Port         = NULL; /**< Puerto del pin VEN */
static uint16_t             PN7160_VEN_Pin          = 0U;   /**< Pin VEN */
static GPIO_TypeDef*        PN7160_IRQ_Port         = NULL; /**< Puerto del pin IRQ */
static uint16_t             PN7160_IRQ_Pin          = 0U;   /**< Pin IRQ */
static uint8_t              PN7160_Initialized      = 0U;   /**< Bandera para verificar si el módulo está inicializado */
static PN7160_Mode_t        PN7160_Mode             = PN7160_MODE_READER;   /**< Modo RF configurado; decide qué sondea/escucha WaitTag */
static uint8_t              PN7160_RfIdle           = 1U;   /**< 1 = RFST_IDLE; 0 = hay que mandar RF_DEACTIVATE antes de otro comando RF */
static uint8_t              PN7160_TagActive        = 0U;   /**< 1 = tag/lector activado, se permite PN7160_Transceive */
static uint8_t              PN7160_ActiveProt       = 0U;   /**< RF Protocol del último RF_INTF_ACTIVATED_NTF; vale solo con PN7160_TagActive */
static uint8_t              PN7160_MaxPayload       = 0U;   /**< Max Data Packet Payload Size del RF_INTF_ACTIVATED_NTF */
static uint8_t              PN7160_Credits          = 0U;   /**< Créditos de la conexión RF estática (0xFF = sin control de flujo) */

/* -- Descubrimiento por interrupción (PN7160_StartDiscoveryIT / PN7160_Process) -- */
static volatile uint8_t     PN7160_IrqFlag          = 0U;   /**< 1 = IRQ subió (lo pone PN7160_IRQ_Handler desde la ISR) */
static uint8_t              PN7160_ItArmed          = 0U;   /**< 1 = descubrimiento IT en curso; PN7160_Process lo atiende */
static uint8_t              PN7160_ItKeepActive     = 0U;   /**< keepActive pedido en PN7160_StartDiscoveryIT */
static uint8_t              PN7160_ItSelId          = 0U;   /**< RF Discovery ID elegido entre varios tags (0 = ninguno) */
static PN7160_Tag_t         PN7160_ItTag;                   /**< Tag que se entrega a PN7160_TagDetectedCallback */
static PN7160_Info_t        PN7160_Info;                    /**< Model ID y versión de FW del último CORE_RESET_NTF */
static uint8_t              PN7160_CardUid[10];             /**< NFCID1 a anunciar en modo tarjeta (LA_NFCID1) */
static uint8_t              PN7160_CardUidLen       = PN7160_CARD_UID_UNSET;    /**< 0/4/7/10, o PN7160_CARD_UID_UNSET */

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

    while (HAL_GPIO_ReadPin(PN7160_IRQ_Port, PN7160_IRQ_Pin) != GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start) >= timeoutMs) { return PN7160_TIMEOUT; }
    }

    return PN7160_OK;
}

// ----------------------------------------------------------------------------
// Transporte. Cada bus implementa la misma interfaz y el resto del archivo solo
// usa estas funciones:
//   - pn7160_Write(cmd, len): escribe un paquete NCI completo.
//   - pn7160_ReadPacket(buf): lee un paquete NCI (cabecera + payload); se llama
//     con IRQ ya en alto (pn7160_Read espera IRQ antes).
//   - pn7160_ReleaseBus():    olvida el handle del bus en DeInit.
// VEN e IRQ son GPIO comunes a ambos buses y se manejan fuera de esta capa.
// ----------------------------------------------------------------------------

#if defined(PN7160_TRANSPORT_SPI)

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
 * @brief Lee un paquete NCI del PN7160 (IRQ ya en alto).
 *
 * @param[out] buf Buffer de destino (al menos PN7160_NCI_MAX_LEN bytes).
 *
 * @return PN7160_Status_t Estado de la operación.
 */
static PN7160_Status_t pn7160_ReadPacket(uint8_t* buf)
{
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
 * @brief Olvida el handle SPI almacenado (DeInit).
 */
static void pn7160_ReleaseBus(void)
{
    PN7160_hspi = NULL;
}

#elif defined(PN7160_TRANSPORT_I2C)

/**
 * @brief Envía un paquete NCI al PN7160 por I2C. No hay byte de dirección como en
 *        SPI: la dirección de esclavo con R/W = 0 ya indica escritura.
 *
 *        En Standby el PN7160 se despierta con su propia dirección I2C y puede no
 *        reconocer (NACK) esa primera trama; se reintenta una vez tras
 *        PN7160_I2C_RETRY_MS, igual que la librería de referencia de NXP.
 *
 * @param cmd Paquete NCI completo (cabecera + payload).
 * @param len Longitud del paquete en bytes.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
static PN7160_Status_t pn7160_Write(const uint8_t* cmd, uint16_t len)
{
    HAL_StatusTypeDef hal = HAL_I2C_Master_Transmit(PN7160_hi2c, PN7160_I2C_ADDRESS,
                                                    (uint8_t*)cmd, len, PN7160_TIMEOUT_MS);
    if (hal != HAL_OK)
    {
        HAL_Delay(PN7160_I2C_RETRY_MS);
        hal = HAL_I2C_Master_Transmit(PN7160_hi2c, PN7160_I2C_ADDRESS,
                                      (uint8_t*)cmd, len, PN7160_TIMEOUT_MS);
    }

    return pn7160_FromHal(hal);
}

/**
 * @brief Lee un paquete NCI del PN7160 (IRQ ya en alto) en dos lecturas I2C:
 *        la cabecera (3 bytes) y después buf[2] bytes de payload. IRQ baja al
 *        terminar de leer el paquete completo.
 *
 * @param[out] buf Buffer de destino (al menos PN7160_NCI_MAX_LEN bytes).
 *
 * @return PN7160_Status_t Estado de la operación.
 */
static PN7160_Status_t pn7160_ReadPacket(uint8_t* buf)
{
    HAL_StatusTypeDef hal = HAL_I2C_Master_Receive(PN7160_hi2c, PN7160_I2C_ADDRESS,
                                                   buf, PN7160_NCI_HEADER_LEN, PN7160_TIMEOUT_MS);

    // Tras leer un paquete IRQ sigue en alto un momento aunque el siguiente aún no esté
    // listo, y el PN7160 no reconoce su dirección (NACK): "todavía no hay paquete", no error
    if (hal == HAL_ERROR && (HAL_I2C_GetError(PN7160_hi2c) & HAL_I2C_ERROR_AF) != 0U)
    {
        return PN7160_TIMEOUT;
    }
    if (hal == HAL_OK && buf[2] > 0U)
    {
        hal = HAL_I2C_Master_Receive(PN7160_hi2c, PN7160_I2C_ADDRESS,
                                     &buf[PN7160_NCI_HEADER_LEN], buf[2], PN7160_TIMEOUT_MS);
    }

    return pn7160_FromHal(hal);
}

/**
 * @brief Olvida el handle I2C almacenado (DeInit).
 */
static void pn7160_ReleaseBus(void)
{
    PN7160_hi2c = NULL;
}

#endif /* PN7160_TRANSPORT_SPI / PN7160_TRANSPORT_I2C */

// ----------------------------------------------------------------------------
// Protocolo NCI (común a ambos transportes)
// ----------------------------------------------------------------------------

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
    uint32_t start = HAL_GetTick();

    // pn7160_ReadPacket devuelve TIMEOUT si el paquete aún no está listo (I2C: NACK con
    // IRQ todavía en alto del paquete anterior): se vuelve a esperar dentro del mismo plazo
    for (;;)
    {
        uint32_t elapsed = HAL_GetTick() - start;
        PN7160_Status_t status = pn7160_WaitIrq((elapsed < timeoutMs) ? (timeoutMs - elapsed) : 0U);
        if (status != PN7160_OK) { return status; }

        status = pn7160_ReadPacket(buf);
        if (status != PN7160_TIMEOUT || (HAL_GetTick() - start) >= timeoutMs) { return status; }
    }
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
    PN7160_Info.modelo = rsp[8];
    memcpy(PN7160_Info.firmware, &rsp[9], sizeof(PN7160_Info.firmware));

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
 *        - NFC-A: SENS_RES(2), longitud NFCID1, NFCID1(4/7/10), longitud SEL_RES(0/1), SEL_RES
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
        case PN7160_TEC_NFC_A: off = 3U; len = (parLen > 2U) ? par[2] : 0xFFU; break;
        case PN7160_TEC_NFC_B: off = 1U; len = 4U; break;
        case PN7160_TEC_NFC_F:
        case PN7160_TEC_NFC_V: off = 2U; len = 8U; break;
        case PN7160_TEC_NFC_A_ESCUCHA: break;   /* el lector remoto no tiene identificador */
        default:    return PN7160_ERROR;
    }

    if (len > sizeof(tag->uid) || (uint16_t)off + len > parLen) { return PN7160_ERROR; }

    tag->tecnologia = tecnologia;
    tag->protocolo  = protocolo;
    tag->uidLen     = len;
    for (uint8_t i = 0U; i < len; i++) { tag->uid[i] = par[off + i]; }

    // ATQA de NFC-A: los 2 primeros bytes (parLen > 2 ya está garantizado arriba)
    tag->atqa[0] = (tecnologia == PN7160_TEC_NFC_A) ? par[0] : 0U;
    tag->atqa[1] = (tecnologia == PN7160_TEC_NFC_A) ? par[1] : 0U;

    // SAK de NFC-A: tras el NFCID1 viene su longitud (0 en T1T, 1 en el resto) y el byte
    off += len;
    tag->sak = (tecnologia == PN7160_TEC_NFC_A && (uint16_t)off + 1U < parLen && par[off] == 1U) ? par[off + 1U] : 0U;

    return PN7160_OK;
}

/**
 * @brief Configura el modo RF. Solo es válido en RFST_IDLE, que es donde
 *        Init y WaitTag dejan siempre al PN7160.
 *
 *        - Lector: RF_DISCOVER_MAP ISO-DEP -> interfaz ISO-DEP y MIFARE -> TAG-CMD,
 *          ambos en sondeo. El resto de protocolos usa Frame RF.
 *        - Tarjeta: RF_DISCOVER_MAP ISO-DEP en escucha -> interfaz ISO-DEP,
 *          RF_SET_LISTEN_MODE_ROUTING ISO-DEP -> DH (el host responde los APDU) y
 *          LA_SEL_INFO = 0x20 (el SEL_RES anuncia ISO-DEP) y, si se llamó a
 *          PN7160_SetCardUid(), LA_NFCID1 en el mismo CORE_SET_CONFIG.
 *
 * @param      mode Modo a configurar.
 * @param[out] rsp  Buffer de trabajo (al menos PN7160_NCI_MAX_LEN bytes).
 *
 * @return PN7160_Status_t Estado de la operación.
 */
static PN7160_Status_t pn7160_ConfigMode(PN7160_Mode_t mode, uint8_t* rsp)
{
    static const uint8_t mapReader[] = { 0x21U, 0x00U, 0x07U, 0x02U,      /* RF_DISCOVER_MAP_CMD, 2 mapeos */
                                         0x04U, 0x01U, 0x02U,             /* ISO-DEP, sondeo -> interfaz ISO-DEP */
                                         0x80U, 0x01U, 0x80U };           /* MIFARE, sondeo -> interfaz TAG-CMD */
    static const uint8_t mapCard[]   = { 0x21U, 0x00U, 0x04U, 0x01U,      /* RF_DISCOVER_MAP_CMD, 1 mapeo */
                                         0x04U, 0x02U, 0x02U };           /* ISO-DEP, escucha -> interfaz ISO-DEP */
    static const uint8_t routing[]   = { 0x21U, 0x01U, 0x07U, 0x00U, 0x01U, /* RF_SET_LISTEN_MODE_ROUTING_CMD: último mensaje, 1 entrada */
                                         0x01U, 0x03U,                    /* ruta por protocolo, 3 bytes */
                                         0x00U, 0x01U, 0x04U };           /* DH, encendido, ISO-DEP */
    uint8_t setCfg[9U + sizeof(PN7160_CardUid)] = { 0x20U, 0x02U, 0x04U, 0x01U,  /* CORE_SET_CONFIG_CMD, 1 parámetro */
                                                    0x32U, 0x01U, 0x20U };       /* LA_SEL_INFO = ISO-DEP */
    uint16_t setCfgLen = 7U;
    PN7160_Status_t status;

    // Con PN7160_SetCardUid() se agrega LA_NFCID1 (33 len NFCID1) en el mismo comando
    if (PN7160_CardUidLen != PN7160_CARD_UID_UNSET)
    {
        setCfg[2] = 6U + PN7160_CardUidLen;     /* payload: nParams + 3 de SEL_INFO + 2 + NFCID1 */
        setCfg[3] = 2U;
        setCfg[7] = 0x33U;
        setCfg[8] = PN7160_CardUidLen;
        memcpy(&setCfg[9], PN7160_CardUid, PN7160_CardUidLen);
        setCfgLen = 9U + PN7160_CardUidLen;
    }

    if (mode == PN7160_MODE_READER)
    {
        // RF_DISCOVER_MAP_RSP (41 00 01 00); el nuevo mapa reemplaza al anterior
        status = pn7160_Transceive(mapReader, sizeof(mapReader), rsp);
    }
    else
    {
        status = pn7160_Transceive(mapCard, sizeof(mapCard), rsp);              /* -> 41 00 01 00 */
        if (status == PN7160_OK) { status = pn7160_Transceive(routing, sizeof(routing), rsp); } /* -> 41 01 01 00 */
        if (status == PN7160_OK) { status = pn7160_Transceive(setCfg, setCfgLen, rsp); }       /* -> 40 02 02 00 00 */
    }

    if (status == PN7160_OK) { PN7160_Mode = mode; }

    return status;
}

/**
 * @brief Vuelve a RFST_IDLE: RF_DEACTIVATE_CMD -> RSP (41 06) [-> NTF (61 06)],
 *        ignorando las NTF que se crucen (p. ej. un tag activado justo al vencer
 *        el timeout o créditos pendientes).
 *
 * @param[out] rsp Buffer de trabajo (al menos PN7160_NCI_MAX_LEN bytes).
 *
 * @return PN7160_Status_t Estado de la operación.
 */
static PN7160_Status_t pn7160_Deactivate(uint8_t* rsp)
{
    static const uint8_t rfDeactivate[] = { 0x21U, 0x06U, 0x01U, 0x00U };  /* RF_DEACTIVATE_CMD -> Idle */

    PN7160_TagActive = 0U;
    PN7160_ItArmed   = 0U;  /* cualquier comando que vuelva a Idle cancela el descubrimiento IT */

    PN7160_Status_t status = pn7160_Write(rfDeactivate, sizeof(rfDeactivate));
    while (status == PN7160_OK)
    {
        status = pn7160_Read(rsp, PN7160_TIMEOUT_MS);
        if (rsp[0] == 0x41U && rsp[1] == 0x06U) { break; }
    }
    if (status == PN7160_OK && (rsp[2] < 1U || rsp[3] != PN7160_NCI_STATUS_OK))
    {
        status = PN7160_ERROR;
    }

    // RF_DEACTIVATE_NTF (61 06): llega si había un tag activado; desde
    // RFST_DISCOVERY puede no llegar, así que su timeout no es un error
    if (status == PN7160_OK)
    {
        (void)pn7160_Read(rsp, PN7160_TIMEOUT_MS);
        PN7160_RfIdle = 1U;
    }

    return status;
}

/**
 * @brief Lee un paquete durante el intercambio de datos y procesa los de control:
 *        - CORE_CONN_CREDITS_NTF (60 06): suma los créditos devueltos.
 *        - CORE_INTERFACE_ERROR_NTF (60 08): error RF del intercambio en curso.
 *        - RF_DEACTIVATE_NTF (61 06): el tag/lector se fue.
 *
 * @param[out] buf       Buffer de destino (al menos PN7160_NCI_MAX_LEN bytes).
 * @param      timeoutMs Tiempo máximo de espera de IRQ en ms.
 *
 * @return PN7160_Status_t PN7160_OK si se leyó un paquete (de datos o de créditos).
 */
static PN7160_Status_t pn7160_ReadData(uint8_t* buf, uint32_t timeoutMs)
{
    PN7160_Status_t status = pn7160_Read(buf, timeoutMs);
    if (status != PN7160_OK) { return status; }

    if (buf[0] == 0x60U && buf[1] == 0x06U && buf[2] >= 3U && PN7160_Credits != PN7160_CREDITS_OFF)
    {
        // 60 06 len | nEntradas, ConnID, créditos (solo existe la conexión 0)
        PN7160_Credits += buf[5];
    }
    else if (buf[0] == 0x60U && buf[1] == 0x08U && buf[2] >= 1U)
    {
        return (buf[3] == PN7160_NCI_RF_TIMEOUT) ? PN7160_TIMEOUT : PN7160_ERROR;
    }
    else if (buf[0] == 0x61U && buf[1] == 0x06U)
    {
        // Sigue fuera de RFST_IDLE (vuelve a descubrimiento): el próximo
        // WaitTag/SetMode manda RF_DEACTIVATE
        PN7160_TagActive = 0U;
        return PN7160_ERROR;
    }

    return PN7160_OK;
}

/**
 * @brief Arranca el descubrimiento RF según el modo: deja RFST_IDLE si hacía
 *        falta y manda RF_DISCOVER_CMD -> RF_DISCOVER_RSP (41 03 01 00).
 *
 * @param[out] rsp Buffer de trabajo (al menos PN7160_NCI_MAX_LEN bytes).
 *
 * @return PN7160_Status_t Estado de la operación.
 */
static PN7160_Status_t pn7160_StartDiscovery(uint8_t* rsp)
{
    static const uint8_t rfDiscover[] = { 0x21U, 0x03U, 0x09U, 0x04U,   /* RF_DISCOVER_CMD, 4 configuraciones */
                                          0x00U, 0x01U,                 /* NFC-A sondeo pasivo, cada periodo */
                                          0x01U, 0x01U,                 /* NFC-B sondeo pasivo */
                                          0x02U, 0x01U,                 /* NFC-F sondeo pasivo */
                                          0x06U, 0x01U };               /* NFC-V sondeo pasivo */
    static const uint8_t rfListen[]   = { 0x21U, 0x03U, 0x03U, 0x01U,   /* RF_DISCOVER_CMD, 1 configuración */
                                          0x80U, 0x01U };               /* NFC-A escucha pasiva */
    PN7160_Status_t status;

    // Si quedó un tag activado (keepActive) o un descubrimiento a medias, volver a RFST_IDLE
    if (!PN7160_RfIdle)
    {
        status = pn7160_Deactivate(rsp);
        if (status != PN7160_OK) { return status; }
    }

    if (PN7160_Mode == PN7160_MODE_CARD)
    {
        status = pn7160_Transceive(rfListen, sizeof(rfListen), rsp);
    }
    else
    {
        status = pn7160_Transceive(rfDiscover, sizeof(rfDiscover), rsp);
    }
    if (status == PN7160_OK) { PN7160_RfIdle = 0U; }

    return status;
}

/**
 * @brief Procesa un paquete leído durante el descubrimiento. Atiende
 *        RF_INTF_ACTIVATED_NTF (61 05) y RF_DISCOVER_NTF (61 03); el resto se ignora.
 *
 *        - 61 05: [3] ID, [4] interfaz, [5] protocolo, [6] tecnología, [7] max payload,
 *          [8] créditos, [9] long., [10..] parámetros
 *        - 61 03: [3] ID, [4] protocolo, [5] tecnología, [6] long., [7..] parámetros,
 *          [7 + long.] tipo (0x02 = vienen más)
 *
 * @param         rsp        Paquete leído; se reutiliza como buffer de trabajo.
 * @param[out]    tag        Tag a rellenar.
 * @param         keepActive 1 = hay que activar el tag elegido entre varios.
 * @param[in,out] selId      RF Discovery ID elegido (0 = ninguno todavía).
 * @param[out]    done       1 = terminó el descubrimiento (tag en *tag si devuelve OK).
 *
 * @return PN7160_Status_t Estado de la operación; distinto de OK también termina.
 */
static PN7160_Status_t pn7160_DiscoveryStep(uint8_t* rsp, PN7160_Tag_t* tag, uint8_t keepActive,
                                            uint8_t* selId, uint8_t* done)
{
    PN7160_Status_t status = PN7160_OK;

    *done = 0U;

    if (rsp[0] == 0x61U && rsp[1] == 0x05U && rsp[2] >= 7U && rsp[2] >= 7U + rsp[9])
    {
        status = pn7160_ParseTag(rsp[6], rsp[5], &rsp[10], rsp[9], tag);
        PN7160_MaxPayload = (rsp[7] != 0U) ? rsp[7] : 1U;
        PN7160_Credits    = rsp[8];
        PN7160_ActiveProt = rsp[5];
        *done = 1U;
    }
    else if (rsp[0] == 0x61U && rsp[1] == 0x03U && rsp[2] >= 4U && rsp[2] >= 4U + rsp[6])
    {
        // Varios tags o protocolos: se toma el primero. Sin keepActive basta con sus datos
        if (*selId == 0U)
        {
            status = pn7160_ParseTag(rsp[5], rsp[4], &rsp[7], rsp[6], tag);
            if (status != PN7160_OK || !keepActive) { *done = 1U; return status; }
            *selId = rsp[3];
        }

        // Con keepActive hay que activarlo, pero solo tras la última NTF (RFST_W4_HOST_SELECT):
        // RF_DISCOVER_SELECT_CMD (21 04) -> RSP (41 04 01 00) -> después llega 61 05
        if (rsp[2] < 5U + rsp[6] || rsp[7U + rsp[6]] != 0x02U)
        {
            uint8_t intf = (tag->protocolo == PN7160_PROT_ISODEP) ? 0x02U :     /* ISO-DEP -> interfaz ISO-DEP */
                           (tag->protocolo == PN7160_PROT_MIFARE) ? 0x80U :     /* MIFARE  -> interfaz TAG-CMD */
                                                                    0x01U;      /* resto   -> Frame RF */
            uint8_t select[] = { 0x21U, 0x04U, 0x03U, *selId, tag->protocolo, intf };

            status = pn7160_Transceive(select, sizeof(select), rsp);
        }
    }

    return status;
}

/**
 * @brief Cierra el descubrimiento: con keepActive y éxito deja el tag activado
 *        (RFST_POLL_ACTIVE / RFST_LISTEN_ACTIVE); si no, vuelve a RFST_IDLE.
 *
 * @param      status     Resultado del descubrimiento.
 * @param      keepActive 1 = dejar el tag activado si status es OK.
 * @param[out] rsp        Buffer de trabajo (al menos PN7160_NCI_MAX_LEN bytes).
 *
 * @return PN7160_Status_t status, o el error de RF_DEACTIVATE si status era OK.
 */
static PN7160_Status_t pn7160_DiscoveryFinish(PN7160_Status_t status, uint8_t keepActive, uint8_t* rsp)
{
    if (keepActive && status == PN7160_OK)
    {
        PN7160_TagActive = 1U;
        return PN7160_OK;
    }

    PN7160_Status_t stop = pn7160_Deactivate(rsp);

    return (status != PN7160_OK) ? status : stop;
}

/**
 * @brief Envía un TAG-CMD de MIFARE Classic y valida la respuesta: repite el byte
 *        de comando y termina en un estado 0x00.
 *
 * @param         tx    Comando TAG-CMD (tx[0] = PN7160_MFC_AUTH o PN7160_MFC_XCHG).
 * @param         txLen Bytes de tx.
 * @param[out]    rx    Respuesta completa (eco + datos + estado).
 * @param[in,out] rxLen Entrada: capacidad de rx. Salida: bytes recibidos.
 *
 * @return PN7160_Status_t Estado de la operación; PN7160_ERROR si el tag activado
 *         no es MIFARE Classic o el PN7160/tag rechazó el comando.
 */
static PN7160_Status_t pn7160_MfcCmd(const uint8_t* tx, uint16_t txLen, uint8_t* rx, uint16_t* rxLen)
{
    if (!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }
    if (!PN7160_TagActive || PN7160_ActiveProt != PN7160_PROT_MIFARE)
    {
        return PN7160_ERROR;
    }

    PN7160_Status_t status = PN7160_Transceive(tx, txLen, rx, rxLen);
    if (status != PN7160_OK) { return status; }

    return (*rxLen >= 2U && rx[0] == tx[0] && rx[*rxLen - 1U] == PN7160_NCI_STATUS_OK)
           ? PN7160_OK : PN7160_ERROR;
}

/**
 * @brief Parte de PN7160_Init común a ambos transportes (pasos 3 a 8): reset por
 *        VEN, pila NCI, CLOCK_SEL_CFG y modo lector. El PN7160_Init del bus ya
 *        validó los parámetros, desinicializó y guardó su handle.
 *
 * @param venPort Puerto GPIO del pin VEN.
 * @param venPin  Pin GPIO del pin VEN.
 * @param irqPort Puerto GPIO del pin IRQ.
 * @param irqPin  Pin GPIO del pin IRQ.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
static PN7160_Status_t pn7160_Start(GPIO_TypeDef* venPort, uint16_t venPin,
                                    GPIO_TypeDef* irqPort, uint16_t irqPin)
{
    static const uint8_t getClock[]  = { 0x20U, 0x03U, 0x03U, 0x01U, 0xA0U, 0x03U };   /* CORE_GET_CONFIG_CMD: CLOCK_SEL_CFG */
    static const uint8_t setClock[]  = { 0x20U, 0x02U, 0x05U, 0x01U, 0xA0U, 0x03U,     /* CORE_SET_CONFIG_CMD: CLOCK_SEL_CFG */
                                         0x01U, PN7160_CLOCK_SEL_CFG };
    uint8_t rsp[PN7160_NCI_MAX_LEN];
    PN7160_Status_t status;

    PN7160_VEN_Port = venPort;  PN7160_VEN_Pin = venPin;
    PN7160_IRQ_Port = irqPort;  PN7160_IRQ_Pin = irqPin;
    PN7160_RfIdle   = 1U;       PN7160_TagActive = 0U;      /* el reset por VEN deja RFST_IDLE */
    PN7160_ItArmed  = 0U;       PN7160_IrqFlag   = 0U;

    // 3. Reset por hardware: VEN en bajo >= tWL(VEN) = 10 µs, luego en alto
    //    y esperar tboot = 2.5 ms máx. (HAL_Delay garantiza al menos el valor pedido)
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

    // 7. Modo lector por defecto (RF_DISCOVER_MAP)
    status = pn7160_ConfigMode(PN7160_MODE_READER, rsp);
    if (status != PN7160_OK) { return status; }

    // 8. Marcar como inicializado solo al final, tras un Init exitoso
    PN7160_Initialized = 1U;

    return PN7160_OK;
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

#if defined(PN7160_TRANSPORT_SPI)

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
    // 1. Validar parámetros
    if (hspi == NULL || nssPort == NULL || venPort == NULL || irqPort == NULL)
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
    HAL_GPIO_WritePin(PN7160_NSS_Port, PN7160_NSS_Pin, GPIO_PIN_SET);

    // 3 a 8. Comunes a ambos transportes
    return pn7160_Start(venPort, venPin, irqPort, irqPin);
}

#elif defined(PN7160_TRANSPORT_I2C)

/**
 * @brief Inicializa el controlador NFC PN7160 por I2C.
 *
 * @param hi2c    Handle del I2C ya inicializado por CubeMX.
 * @param venPort Puerto GPIO del pin VEN.
 * @param venPin  Pin GPIO del pin VEN.
 * @param irqPort Puerto GPIO del pin IRQ.
 * @param irqPin  Pin GPIO del pin IRQ.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_Init(I2C_HandleTypeDef* hi2c,
                            GPIO_TypeDef* venPort, uint16_t venPin,
                            GPIO_TypeDef* irqPort, uint16_t irqPin)
{
    // 1. Validar parámetros
    if (hi2c == NULL || venPort == NULL || irqPort == NULL)
    {
        return PN7160_INVALID_PARAM;
    }

    // 2. Re-Init: limpiar estado anterior (ver la versión SPI)
    if (PN7160_Initialized)
    {
        (void)PN7160_DeInit();
    }

    PN7160_hi2c = hi2c;

    // Bus trabado antes de empezar: con VEN en bajo desde el reset del MCU se observaron
    // SCL/SDA retenidas en bajo y el periférico con BUSY sin transferencia en curso, así que
    // rechaza todo con HAL_BUSY. Con VEN en alto el PN7160 suelta las líneas; se reinicia el
    // periférico (DeInit/Init hace SWRST) hasta que BUSY se libere.
    // ponytail: asume un solo maestro en el bus; con varios, BUSY puede ser legítimo
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET)
    {
        uint32_t start = HAL_GetTick();

        HAL_GPIO_WritePin(venPort, venPin, GPIO_PIN_SET);
        do
        {
            HAL_Delay(1U);
            if (HAL_I2C_DeInit(hi2c) != HAL_OK || HAL_I2C_Init(hi2c) != HAL_OK)
            {
                return PN7160_ERROR;
            }
        } while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET &&
                 (HAL_GetTick() - start) < PN7160_TIMEOUT_MS);

        if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET)
        {
            return PN7160_ERROR;
        }
    }

    // 3 a 8. Comunes a ambos transportes
    return pn7160_Start(venPort, venPin, irqPort, irqPin);
}

#endif /* PN7160_TRANSPORT_SPI / PN7160_TRANSPORT_I2C */

/**
 * @brief Desinicializa el controlador NFC PN7160 dejándolo en Hard Power Down.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_DeInit(void)
{
    if (!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }

    // VEN en bajo = Hard Power Down (10.5 µA típ.)
    HAL_GPIO_WritePin(PN7160_VEN_Port, PN7160_VEN_Pin, GPIO_PIN_RESET);

    pn7160_ReleaseBus();
    PN7160_Initialized  =   0U;
    PN7160_ItArmed      =   0U;

    return PN7160_OK;
}

/**
 * @brief Devuelve el model ID y la versión de firmware leídos en PN7160_Init().
 *
 * @param[out] info Destino de la identificación.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_GetInfo(PN7160_Info_t* info)
{
    if (info == NULL)
    {
        return PN7160_INVALID_PARAM;
    }
    if (!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }

    *info = PN7160_Info;
    return PN7160_OK;
}

/**
 * @brief Cambia el modo RF entre lector de tags y emulación de tarjeta.
 *
 * @param mode PN7160_MODE_READER o PN7160_MODE_CARD.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_SetMode(PN7160_Mode_t mode)
{
    uint8_t rsp[PN7160_NCI_MAX_LEN];

    if (mode != PN7160_MODE_READER && mode != PN7160_MODE_CARD)
    {
        return PN7160_INVALID_PARAM;
    }
    if (!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }

    // Los comandos de configuración RF solo valen en RFST_IDLE
    if (!PN7160_RfIdle)
    {
        PN7160_Status_t status = pn7160_Deactivate(rsp);
        if (status != PN7160_OK) { return status; }
    }

    return pn7160_ConfigMode(mode, rsp);
}

/**
 * @brief Fija el NFCID1 (UID) que anuncia el PN7160 en modo tarjeta.
 *
 * @param uid NFCID1 a anunciar; puede ser NULL si len = 0.
 * @param len 4, 7 o 10 = UID fijo; 0 = UID dinámico generado por el PN7160.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_SetCardUid(const uint8_t* uid, uint8_t len)
{
    // 1. Validar parámetros y estado
    if ((len != 0U && len != 4U && len != 7U && len != 10U) || (uid == NULL && len != 0U))
    {
        return PN7160_INVALID_PARAM;
    }
    if (!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }

    // 2. Guardar; pn7160_ConfigMode lo envía en cada SetMode(PN7160_MODE_CARD)
    if (len != 0U) { memcpy(PN7160_CardUid, uid, len); }
    PN7160_CardUidLen = len;

    // 3. Si ya está en modo tarjeta, aplicarlo ahora
    return (PN7160_Mode == PN7160_MODE_CARD) ? PN7160_SetMode(PN7160_MODE_CARD) : PN7160_OK;
}


/**
 * @brief Sondea NFC-A/B/F/V (lector) o escucha como NFC-A (tarjeta) hasta que
 *        se active un tag/lector o se agote timeoutMs.
 *
 * @param[out] tag        Datos del tag detectado.
 * @param      timeoutMs  Tiempo máximo de espera de un tag, en ms.
 * @param      keepActive 0 = volver a RFST_IDLE al terminar; 1 = dejar el tag
 *                        activado para PN7160_Transceive().
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_WaitTag(PN7160_Tag_t* tag, uint32_t timeoutMs, uint8_t keepActive)
{
    uint8_t rsp[PN7160_NCI_MAX_LEN];
    uint8_t selId = 0U;     /* RF Discovery ID elegido entre varios tags (0 = ninguno) */
    uint8_t done  = 0U;
    PN7160_Status_t status;

    // 1. Validar parámetros y estado
    if (tag == NULL)
    {
        return PN7160_INVALID_PARAM;
    }
    if (!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }

    // 2. [RF_DEACTIVATE] + RF_DISCOVER_CMD -> RF_DISCOVER_RSP, según el modo
    status = pn7160_StartDiscovery(rsp);
    if (status != PN7160_OK) { return status; }

    // 3. Esperar RF_INTF_ACTIVATED_NTF (61 05) o RF_DISCOVER_NTF (61 03) hasta timeoutMs
    uint32_t start = HAL_GetTick();
    while (status == PN7160_OK && !done)
    {
        uint32_t elapsed = HAL_GetTick() - start;
        if (elapsed >= timeoutMs) { status = PN7160_TIMEOUT; break; }

        status = pn7160_Read(rsp, timeoutMs - elapsed);
        if (status == PN7160_OK) { status = pn7160_DiscoveryStep(rsp, tag, keepActive, &selId, &done); }
    }

    // 4. Dejar el tag activado (keepActive) o volver a RFST_IDLE
    return pn7160_DiscoveryFinish(status, keepActive, rsp);
}

/**
 * @brief Arranca el descubrimiento RF sin bloquear; PN7160_Process() lo atiende.
 *
 * @param keepActive 0 = volver a RFST_IDLE tras el callback; 1 = dejar el tag
 *                   activado para PN7160_Transceive().
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_StartDiscoveryIT(uint8_t keepActive)
{
    uint8_t rsp[PN7160_NCI_MAX_LEN];

    if (!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }

    PN7160_Status_t status = pn7160_StartDiscovery(rsp);
    if (status != PN7160_OK) { return status; }

    PN7160_ItKeepActive = keepActive;
    PN7160_ItSelId      = 0U;
    PN7160_ItArmed      = 1U;

    return PN7160_OK;
}

/**
 * @brief Marca que IRQ subió. Llamar desde HAL_GPIO_EXTI_Callback(); no usa el bus (SPI/I2C).
 */
void PN7160_IRQ_Handler(void)
{
    PN7160_IrqFlag = 1U;
}

/**
 * @brief Atiende el descubrimiento IT desde el bucle principal: lee el paquete
 *        pendiente y, al detectar un tag, llama a PN7160_TagDetectedCallback().
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_Process(void)
{
    uint8_t rsp[PN7160_NCI_MAX_LEN];
    uint8_t done = 0U;

    if (!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }

    if (!PN7160_ItArmed || !PN7160_IrqFlag)
    {
        return PN7160_OK;
    }

    // La bandera solo avisa: se lee mientras IRQ siga en alto. Así no se espera
    // por flancos de respuestas ya leídas (p. ej. el RF_DISCOVER_RSP), y un flanco
    // que llegue tras limpiar la bandera la vuelve a poner para la siguiente llamada
    PN7160_IrqFlag = 0U;

    PN7160_Status_t status = PN7160_OK;
    while (status == PN7160_OK && !done &&
           HAL_GPIO_ReadPin(PN7160_IRQ_Port, PN7160_IRQ_Pin) == GPIO_PIN_SET)
    {
        status = pn7160_Read(rsp, PN7160_TIMEOUT_MS);
        if (status == PN7160_OK)
        {
            status = pn7160_DiscoveryStep(rsp, &PN7160_ItTag, PN7160_ItKeepActive, &PN7160_ItSelId, &done);
        }
    }
    if (status == PN7160_OK && !done) { return PN7160_OK; }

    // Terminó: se desarma antes del callback para que este pueda volver a armar
    PN7160_ItArmed = 0U;
    status = pn7160_DiscoveryFinish(status, PN7160_ItKeepActive, rsp);
    if (status == PN7160_OK) { PN7160_TagDetectedCallback(&PN7160_ItTag); }

    return status;
}

/**
 * @brief Callback de tag detectado. Implementación vacía; redefinirla en la aplicación.
 *
 * @param tag Datos del tag detectado.
 */
__weak void PN7160_TagDetectedCallback(const PN7160_Tag_t* tag)
{
    (void)tag;
}

/**
 * @brief Intercambia datos con el tag/lector activado por PN7160_WaitTag(..., 1).
 *
 * @param      tx    Datos a enviar (puede ser NULL si txLen = 0).
 * @param      txLen Bytes a enviar; 0 = solo recibir (modo tarjeta: primer C-APDU).
 * @param[out] rx    Buffer de recepción.
 * @param[in,out] rxLen Entrada: capacidad de rx. Salida: bytes recibidos.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_Transceive(const uint8_t* tx, uint16_t txLen, uint8_t* rx, uint16_t* rxLen)
{
    uint8_t buf[PN7160_NCI_MAX_LEN];
    PN7160_Status_t status = PN7160_OK;
    uint16_t sent     = 0U;
    uint16_t got      = 0U;
    uint8_t  overflow = 0U;

    // 1. Validar parámetros y estado
    if ((tx == NULL && txLen > 0U) || rx == NULL || rxLen == NULL)
    {
        return PN7160_INVALID_PARAM;
    }
    if (!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }
    if (!PN7160_TagActive)
    {
        return PN7160_ERROR;
    }

    uint16_t cap = *rxLen;
    *rxLen = 0U;

    // 2. Enviar tx por la conexión RF estática (ConnID 0) en segmentos de MaxPayload
    //    bytes: 0x10 00 len (PBF = 1) ... 0x00 00 len (último). Cada uno gasta un crédito
    while (sent < txLen)
    {
        uint16_t seg = txLen - sent;
        if (seg > PN7160_MaxPayload) { seg = PN7160_MaxPayload; }

        while (PN7160_Credits == 0U && status == PN7160_OK)
        {
            status = pn7160_ReadData(buf, PN7160_RF_TIMEOUT_MS);
        }
        if (status != PN7160_OK) { return status; }

        buf[0] = (sent + seg < txLen) ? PN7160_NCI_PBF : 0x00U;
        buf[1] = 0x00U;
        buf[2] = (uint8_t)seg;
        memcpy(&buf[PN7160_NCI_HEADER_LEN], &tx[sent], seg);

        status = pn7160_Write(buf, PN7160_NCI_HEADER_LEN + seg);
        if (status != PN7160_OK) { return status; }

        if (PN7160_Credits != PN7160_CREDITS_OFF) { PN7160_Credits--; }
        sent += seg;
    }

    // 3. Recibir segmentos de datos hasta el que trae PBF = 0. Los paquetes de
    //    control (créditos) se procesan en pn7160_ReadData y aquí se saltan
    for (;;)
    {
        status = pn7160_ReadData(buf, PN7160_RF_TIMEOUT_MS);
        if (status != PN7160_OK) { return status; }

        if ((buf[0] & (uint8_t)~PN7160_NCI_PBF) != 0x00U) { continue; }  /* no es dato de ConnID 0 */

        uint16_t n = buf[2];
        if (n > cap - got) { n = cap - got; overflow = 1U; }
        memcpy(&rx[got], &buf[PN7160_NCI_HEADER_LEN], n);
        got += n;

        if ((buf[0] & PN7160_NCI_PBF) == 0U) { break; }
    }

    *rxLen = got;

    return overflow ? PN7160_ERROR : PN7160_OK;
}

/**
 * @brief Desactiva el tag/lector y apaga el campo RF (vuelve a RFST_IDLE).
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_ReleaseTag(void)
{
    uint8_t rsp[PN7160_NCI_MAX_LEN];

    if (!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }

    // Ya en RFST_IDLE: nada que hacer
    if (PN7160_RfIdle)
    {
        return PN7160_OK;
    }

    return pn7160_Deactivate(rsp);
}

/**
 * @brief Espera a que se retire el tag/lector activado: repite WaitTag sin
 *        keepActive hasta que nadie se activa en PN7160_REMOVAL_MS.
 *
 * @param timeoutMs Tiempo máximo de espera de la retirada, en ms.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_WaitTagRemoval(uint32_t timeoutMs)
{
    PN7160_Tag_t tag;
    PN7160_Status_t status;

    // 1. Validar estado
    if (!PN7160_Initialized)
    {
        return PN7160_NOT_INITIALIZED;
    }
    if (!PN7160_TagActive)
    {
        return PN7160_ERROR;
    }

    // 2. Redescubrir mientras algo se active; cada WaitTag termina en RFST_IDLE
    // ponytail: no distingue el tag original de otro; presence check por protocolo
    //           (2F 11 para ISO-DEP, READ para T2T, ...) si hace falta, ver UM11495
    uint32_t start = HAL_GetTick();
    do
    {
        status = PN7160_WaitTag(&tag, PN7160_REMOVAL_MS, 0U);
    } while (status == PN7160_OK && (HAL_GetTick() - start) < timeoutMs);

    // 3. Nadie respondió en la ventana: se retiró. Seguía ahí al vencer timeoutMs: TIMEOUT
    if (status == PN7160_TIMEOUT) { return PN7160_OK; }

    return (status == PN7160_OK) ? PN7160_TIMEOUT : status;
}

/**
 * @brief Autentica un sector de la MIFARE Classic activada (TAG-CMD 0x40).
 *
 * @param sector Sector a autenticar (0-15 en 1K, 0-39 en 4K).
 * @param keyB   0 = clave A; 1 = clave B.
 * @param key    Clave de 6 bytes.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_MfcAuth(uint8_t sector, uint8_t keyB, const uint8_t key[6])
{
    uint8_t rx[4];
    uint16_t n = sizeof(rx);

    if (key == NULL || sector > PN7160_MFC_MAX_SECTOR || keyB > 1U)
    {
        return PN7160_INVALID_PARAM;
    }

    // 40 sector selector clave[6]: la clave va embebida en el comando
    uint8_t tx[3U + 6U] = { PN7160_MFC_AUTH, sector,
                            (uint8_t)(PN7160_MFC_KEY_EMBEDDED | (keyB ? PN7160_MFC_KEY_B : 0U)) };
    memcpy(&tx[3], key, 6U);

    return pn7160_MfcCmd(tx, sizeof(tx), rx, &n);
}

/**
 * @brief Lee un bloque de 16 bytes de la MIFARE Classic (MIFARE READ por TAG-CMD 0x10).
 *
 * @param      block Bloque absoluto (0-63 en 1K, 0-255 en 4K).
 * @param[out] out   Destino de los 16 bytes.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_MfcRead(uint8_t block, uint8_t out[16])
{
    const uint8_t tx[] = { PN7160_MFC_XCHG, PN7160_MFC_READ, block };
    uint8_t rx[1U + 16U + 1U];      /* eco 0x10 + datos + estado */
    uint16_t n = sizeof(rx);

    if (out == NULL)
    {
        return PN7160_INVALID_PARAM;
    }

    PN7160_Status_t status = pn7160_MfcCmd(tx, sizeof(tx), rx, &n);
    if (status != PN7160_OK) { return status; }
    if (n != sizeof(rx)) { return PN7160_ERROR; }

    memcpy(out, &rx[1], 16U);

    return PN7160_OK;
}

/**
 * @brief Escribe un bloque de 16 bytes en la MIFARE Classic (MIFARE WRITE en dos
 *        fases por TAG-CMD 0x10: dirección y después los datos).
 *
 * @param block Bloque absoluto (0-63 en 1K, 0-255 en 4K).
 * @param data  16 bytes a escribir.
 *
 * @return PN7160_Status_t Estado de la operación.
 */
PN7160_Status_t PN7160_MfcWrite(uint8_t block, const uint8_t data[16])
{
    const uint8_t cmd[] = { PN7160_MFC_XCHG, PN7160_MFC_WRITE, block };
    uint8_t tx[1U + 16U];
    uint8_t rx[4];
    uint16_t n = sizeof(rx);

    if (data == NULL)
    {
        return PN7160_INVALID_PARAM;
    }

    // 1. WRITE + dirección; el tag responde ACK
    PN7160_Status_t status = pn7160_MfcCmd(cmd, sizeof(cmd), rx, &n);
    if (status != PN7160_OK) { return status; }

    // 2. Los 16 bytes; el tag responde ACK al grabarlos
    tx[0] = PN7160_MFC_XCHG;
    memcpy(&tx[1], data, 16U);
    n = sizeof(rx);

    return pn7160_MfcCmd(tx, sizeof(tx), rx, &n);
}
