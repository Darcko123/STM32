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

#ifndef PN7160_RF_TIMEOUT_MS
#define PN7160_RF_TIMEOUT_MS 1000U  /**< Espera de cada paquete de datos/créditos en PN7160_Transceive, en ms (redefinible antes de incluir) */
#endif

#ifndef PN7160_BOOT_MS
#define PN7160_BOOT_MS      3U      /**< Espera tras subir VEN en ms; tboot = 2.5 ms máx. (redefinible antes de incluir) */
#endif

#ifndef PN7160_CLOCK_SEL_CFG
/**
 * @brief Valor de CLOCK_SEL_CFG (A0 03): fuente de reloj del frontal RF (redefinible antes de incluir).
 *
 * 0x08 = cristal de 27.12 MHz en NFC_CLK_XTAL1/XTAL2 (valor de la librería de referencia de NXP).
 * Para usar el PLL con reloj externo (19.2, 26, 32, 38.4 o 48 MHz en NFC_CLK_XTAL1)
 * toma la codificación de la tabla CLOCK_SEL_CFG del UM11495; el datasheet no la incluye.
 */
#define PN7160_CLOCK_SEL_CFG    0x08U
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

/**
 * @brief Modo RF del PN7160, configurado con PN7160_SetMode().
 */
typedef enum {
    PN7160_MODE_READER  = 0,    /**< Lector/escritor: sondea NFC-A/B/F/V (por defecto tras Init) */
    PN7160_MODE_CARD    = 1     /**< Emulación de tarjeta T4T (ISO-DEP sobre NFC-A); el host responde los APDU */
} PN7160_Mode_t;

/**
 * @brief Datos de un tag detectado por PN7160_WaitTag().
 */
typedef struct {
    uint8_t tecnologia;   /**< RF Tech & Mode: 0x00 NFC-A, 0x01 NFC-B, 0x02 NFC-F, 0x06 NFC-V; 0x80 NFC-A escucha (modo tarjeta) */
    uint8_t protocolo;    /**< RF Protocol: 0x01 T1T, 0x02 T2T, 0x03 T3T, 0x04 ISO-DEP, 0x05 NFC-DEP, 0x06 T5T, 0x80 MIFARE Classic */
    uint8_t uid[10];      /**< NFCID1 / NFCID0 / NFCID2 / UID, según la tecnología (en el orden en que lo entrega NCI; NFC-V llega LSB primero) */
    uint8_t uidLen;       /**< Bytes válidos en uid: 4/7/10 (A), 4 (B), 8 (F), 8 (V); 0 en modo tarjeta */
} PN7160_Tag_t;

/**
 * @brief Identificación del chip, leída del CORE_RESET_NTF durante PN7160_Init().
 */
typedef struct {
    uint8_t modelo;       /**< Model ID: 0x61 = PN7160, 0x71 = PN7161 */
    uint8_t firmware[3];  /**< Versión de firmware, en el orden del CORE_RESET_NTF (bytes 10 a 12) */
} PN7160_Info_t;

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
 *          5. NCI_PROPRIETARY_ACT_CMD (2F 02 00) -> RSP (4F 02 ...): habilita las
 *             extensiones NXP (MIFARE Classic como protocolo 0x80).
 *          6. CORE_GET_CONFIG_CMD (20 03 03 01 A0 03) lee CLOCK_SEL_CFG. Si difiere de
 *             PN7160_CLOCK_SEL_CFG, CORE_SET_CONFIG_CMD (20 02 05 01 A0 03 01 xx) lo
 *             escribe en EEPROM y se repiten los pasos 3 a 5 para aplicarlo.
 *          7. RF_DISCOVER_MAP_CMD (21 00): ISO-DEP -> interfaz ISO-DEP y MIFARE ->
 *             interfaz TAG-CMD, ambos en sondeo. El resto usa Frame RF.
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

/**
 * @brief Devuelve el model ID y la versión de firmware leídos en PN7160_Init().
 *
 * @details No habla con el chip: copia lo guardado del último CORE_RESET_NTF.
 *
 * @param[out] info Destino de la identificación.
 *
 * @return PN7160_OK si se copió la información.
 * @return PN7160_INVALID_PARAM si info es NULL.
 * @return PN7160_NOT_INITIALIZED si no se llamó antes a PN7160_Init().
 */
PN7160_Status_t PN7160_GetInfo(PN7160_Info_t* info);

/**
 * @brief Cambia el modo RF entre lector de tags y emulación de tarjeta.
 *
 * @details El PN7160 debe estar en RFST_IDLE, que es donde lo dejan siempre
 *          PN7160_Init() y PN7160_WaitTag(). El modo se aplica en el siguiente
 *          PN7160_WaitTag(). Comandos NCI 2.0 (UM11495, no vienen en el datasheet):
 *          - PN7160_MODE_READER: RF_DISCOVER_MAP_CMD (21 00) ISO-DEP -> interfaz
 *            ISO-DEP y MIFARE -> interfaz TAG-CMD en sondeo. Es lo que deja Init.
 *          - PN7160_MODE_CARD:
 *            1. RF_DISCOVER_MAP_CMD (21 00 04 01 04 02 02): ISO-DEP en escucha ->
 *               interfaz ISO-DEP.
 *            2. RF_SET_LISTEN_MODE_ROUTING_CMD (21 01 07 00 01 01 03 00 01 04):
 *               ISO-DEP se rutea al DH (el host responde los APDU) en estado encendido.
 *            3. CORE_SET_CONFIG_CMD (20 02 04 01 32 01 20): LA_SEL_INFO = 0x20, el
 *               SEL_RES anuncia ISO-DEP.
 *
 *          En modo tarjeta solo se emula NFC-A; Card Emulation requiere
 *          VBAT > 2.8 V (datasheet, Listener mode).
 *
 * @param mode PN7160_MODE_READER o PN7160_MODE_CARD.
 *
 * @return PN7160_OK si el modo quedó configurado.
 * @return PN7160_NOT_INITIALIZED si no se llamó antes a PN7160_Init().
 * @return PN7160_INVALID_PARAM si mode no es un PN7160_Mode_t válido.
 * @return PN7160_TIMEOUT si IRQ no se activó a tiempo o el bus SPI no respondió.
 * @return PN7160_ERROR si falla el bus o una respuesta NCI no es la esperada. El
 *         modo anterior se conserva, aunque la configuración puede quedar a medias.
 */
PN7160_Status_t PN7160_SetMode(PN7160_Mode_t mode);

/**
 * @brief Espera un tag (modo lector) o un lector externo (modo tarjeta) hasta
 *        que se active o se agote el tiempo.
 *
 * @details Secuencia NCI 2.0:
 *          1. Si una llamada anterior dejó un tag activado, RF_DEACTIVATE_CMD
 *             primero (ver paso 4).
 *          2. RF_DISCOVER_CMD (21 03) -> RF_DISCOVER_RSP (41 03 01 00). En modo
 *             lector, NFC-A/B/F/V en sondeo; en modo tarjeta, NFC-A en escucha
 *             (21 03 03 01 80 01). En modo tarjeta, tag->uidLen = 0.
 *          3. Espera RF_INTF_ACTIVATED_NTF (61 05), o RF_DISCOVER_NTF (61 03) si
 *             hay varios tags o protocolos; se toma el primero. Del bloque "RF
 *             Technology Specific Parameters" se extrae el identificador. Con
 *             keepActive y varios tags, tras la última 61 03 se activa el primero
 *             con RF_DISCOVER_SELECT_CMD (21 04) y se espera su 61 05.
 *          4. Sin keepActive, o si algo falló: RF_DEACTIVATE_CMD (21 06 01 00) ->
 *             RSP (41 06) [-> NTF (61 06)] y el PN7160 vuelve a RFST_IDLE. Con
 *             keepActive y éxito, el tag queda activado (campo RF encendido) para
 *             PN7160_Transceive() hasta el siguiente WaitTag/SetMode.
 *
 * @param[out] tag        Datos del tag detectado.
 * @param      timeoutMs  Tiempo máximo de espera de un tag, en ms.
 * @param      keepActive 0 = volver a RFST_IDLE al terminar; 1 = dejar el tag
 *                        activado para PN7160_Transceive().
 *
 * @return PN7160_OK si se detectó un tag y *tag es válido.
 * @return PN7160_NOT_INITIALIZED si no se llamó antes a PN7160_Init().
 * @return PN7160_INVALID_PARAM si tag es NULL.
 * @return PN7160_TIMEOUT si no apareció ningún tag en timeoutMs.
 * @return PN7160_ERROR si falla el bus o una respuesta NCI no es la esperada.
 */
PN7160_Status_t PN7160_WaitTag(PN7160_Tag_t* tag, uint32_t timeoutMs, uint8_t keepActive);

/**
 * @brief Intercambia datos con el tag (modo lector) o el lector externo (modo
 *        tarjeta) activado por PN7160_WaitTag(..., keepActive = 1).
 *
 * @details Usa paquetes de datos NCI por la conexión RF estática (ConnID 0):
 *          1. tx se parte en segmentos de "Max Data Packet Payload Size" (del
 *             RF_INTF_ACTIVATED_NTF) con PBF = 1 salvo el último. Cada segmento
 *             gasta un crédito; si no quedan, se espera CORE_CONN_CREDITS_NTF (60 06).
 *          2. Se reciben segmentos hasta el que trae PBF = 0 y se concatenan en rx.
 *
 *          Formato de los datos, según la interfaz que eligió el discover map:
 *          - ISO-DEP (protocolo 0x04): APDU completos, sin bytes extra.
 *          - Frame RF (T2T, T5T, ...): rx termina con un byte de estado NCI
 *            (0x00 = OK) que no viene del tag.
 *          - TAG-CMD (MIFARE Classic, 0x80): tx y rx llevan el formato propietario
 *            NXP (0x10 = intercambio de datos, 0x40 = autenticación; UM11495).
 *
 *          En modo tarjeta el orden es inverso: con txLen = 0 se recibe el primer
 *          C-APDU, y en las siguientes llamadas tx es la R-APDU y rx el siguiente
 *          C-APDU.
 *
 * @param         tx    Datos a enviar; puede ser NULL si txLen = 0.
 * @param         txLen Bytes a enviar; 0 = solo recibir.
 * @param[out]    rx    Buffer de recepción.
 * @param[in,out] rxLen Entrada: capacidad de rx. Salida: bytes guardados en rx.
 *
 * @return PN7160_OK si se envió tx y se recibió una respuesta completa.
 * @return PN7160_NOT_INITIALIZED si no se llamó antes a PN7160_Init().
 * @return PN7160_INVALID_PARAM si rx o rxLen son NULL, o tx es NULL con txLen > 0.
 * @return PN7160_TIMEOUT si pasaron PN7160_RF_TIMEOUT_MS sin un paquete, o el
 *         PN7160 reportó timeout RF (CORE_INTERFACE_ERROR_NTF 0xB2).
 * @return PN7160_ERROR si no hay tag activado, el tag/lector se fue
 *         (RF_DEACTIVATE_NTF), hubo otro error RF, o la respuesta no cupo en rx
 *         (*rxLen = bytes que sí cupieron).
 */
PN7160_Status_t PN7160_Transceive(const uint8_t* tx, uint16_t txLen, uint8_t* rx, uint16_t* rxLen);

/**
 * @brief Desactiva el tag/lector que dejó PN7160_WaitTag(..., keepActive = 1) y
 *        apaga el campo RF.
 *
 * @details RF_DEACTIVATE_CMD (21 06 01 00) -> RSP (41 06) [-> NTF (61 06)]: el
 *          PN7160 vuelve a RFST_IDLE. Llamarla tras el último PN7160_Transceive()
 *          ahorra energía; si ya está en RFST_IDLE no hace nada. También sirve tras
 *          un PN7160_Transceive() que devolvió error porque el tag/lector se fue.
 *
 * @return PN7160_OK si el PN7160 quedó en RFST_IDLE.
 * @return PN7160_NOT_INITIALIZED si no se llamó antes a PN7160_Init().
 * @return PN7160_TIMEOUT si IRQ no se activó a tiempo o el bus SPI no respondió.
 * @return PN7160_ERROR si falla el bus o la respuesta NCI no es la esperada.
 */
PN7160_Status_t PN7160_ReleaseTag(void);

// ----------------------------------------------------------------------------
// Descubrimiento por interrupción (EXTI + callback), sin timeout
// ----------------------------------------------------------------------------

/**
 * @brief Arranca el descubrimiento RF sin bloquear. Es la versión por
 *        interrupción de PN7160_WaitTag(): no tiene timeout y el tag se entrega
 *        en PN7160_TagDetectedCallback(), llamado desde PN7160_Process().
 *
 * @details Hace los pasos 1 y 2 de PN7160_WaitTag() y regresa; los pasos 3 y 4
 *          los hace PN7160_Process() conforme llegan las NTF. El descubrimiento
 *          sigue hasta que aparece un tag o se cancela con PN7160_ReleaseTag(),
 *          PN7160_SetMode() o PN7160_WaitTag(). Es de un solo disparo: para seguir
 *          leyendo tags hay que volver a llamarla (se puede desde el callback).
 *
 *          Configuración previa (p. ej. en STM32CubeMX):
 *          - IRQ como GPIO_EXTI con flanco de subida (External Interrupt Mode with
 *            Rising edge trigger detection) y su línea EXTI habilitada en el NVIC.
 *          - HAL_GPIO_EXTI_Callback() debe llamar a PN7160_IRQ_Handler().
 *
 * @param keepActive 0 = volver a RFST_IDLE antes del callback; 1 = dejar el tag
 *                   activado para PN7160_Transceive() (se puede usar dentro del
 *                   callback) hasta PN7160_ReleaseTag() o el siguiente descubrimiento.
 *
 * @return PN7160_OK si el PN7160 quedó descubriendo.
 * @return PN7160_NOT_INITIALIZED si no se llamó antes a PN7160_Init().
 * @return PN7160_TIMEOUT si IRQ no se activó a tiempo o el bus SPI no respondió.
 * @return PN7160_ERROR si falla el bus o la respuesta NCI no es la esperada.
 */
PN7160_Status_t PN7160_StartDiscoveryIT(uint8_t keepActive);

/**
 * @brief Manejador de IRQ del PN7160. Llamar desde HAL_GPIO_EXTI_Callback().
 *
 *        Se ejecuta en contexto de interrupción: solo activa una bandera, NO usa
 *        SPI. El trabajo lo hace PN7160_Process() en el bucle principal.
 *
 * Ejemplo de integración en main.c:
 * @code
 *   void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 *       if (GPIO_Pin == NFC_IRQ_Pin) {
 *           PN7160_IRQ_Handler();
 *       }
 *   }
 * @endcode
 */
void PN7160_IRQ_Handler(void);

/**
 * @brief Atiende el descubrimiento arrancado con PN7160_StartDiscoveryIT().
 *        Llamar en el bucle principal, nunca desde una ISR.
 *
 * @details Si PN7160_IRQ_Handler() no marcó nada, regresa de inmediato. Si no,
 *          lee los paquetes pendientes (mientras IRQ siga en alto) y, cuando se
 *          activa un tag, cierra el descubrimiento como PN7160_WaitTag() (paso 4)
 *          y llama a PN7160_TagDetectedCallback(). Solo bloquea lo que dura leer un
 *          paquete o un intercambio comando/respuesta (PN7160_TIMEOUT_MS como máximo).
 *
 * @return PN7160_OK si no había nada que hacer, el descubrimiento sigue o se
 *         entregó un tag al callback.
 * @return PN7160_NOT_INITIALIZED si no se llamó antes a PN7160_Init().
 * @return PN7160_TIMEOUT si el bus SPI no respondió a tiempo.
 * @return PN7160_ERROR si falla el bus o una NTF no es válida. En ambos errores el
 *         descubrimiento termina sin callback; volver a llamar a
 *         PN7160_StartDiscoveryIT().
 */
PN7160_Status_t PN7160_Process(void);

/**
 * @brief Callback de tag detectado (modo lector) o lector externo activado
 *        (modo tarjeta). Lo llama PN7160_Process().
 *
 *        Definida como __weak en la librería; redefinirla en la aplicación.
 *        Se ejecuta en el contexto de PN7160_Process() (no en la ISR), así que
 *        puede llamar a PN7160_Transceive(), PN7160_ReleaseTag() o
 *        PN7160_StartDiscoveryIT().
 *
 * @param tag Datos del tag; el puntero sigue siendo válido hasta el siguiente
 *            PN7160_StartDiscoveryIT().
 */
void PN7160_TagDetectedCallback(const PN7160_Tag_t* tag);

#ifdef __cplusplus
}
#endif

#endif /* PN7160_H */
