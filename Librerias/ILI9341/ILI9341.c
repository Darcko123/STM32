/**
 * @file ILI9341.c
 * @brief Implementación del driver genérico del controlador LCD ILI9341.
 *
 * @origin El código de este driver se basa en la librería Petr Machala, Tilen Majerle, 2014.
 * @author Dr. Luis Antonio Raygoza Pérez & Ing. Daniel Ruiz
 * @date July 11, 2026
 * @version 2.0.0
 */

#include "ILI9341.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

/** @brief Orientación física de la pantalla — uso exclusivo interno del driver. */
typedef enum {
    ILI9341_Landscape, /**< Modo horizontal (width > height) */
    ILI9341_Portrait   /**< Modo vertical   (height > width) */
} ILI9341_Orientation;

/** @brief Geometría de pantalla activa; actualizada por ILI9341_Rotate(). */
typedef struct {
    uint16_t width;              /**< Ancho activo en píxeles  */
    uint16_t height;             /**< Alto  activo en píxeles  */
    ILI9341_Orientation orientation; /**< Orientación actual    */
} ILI9341_Options_t;

/* -- Pines de control (parametrizados en ILI9341_Init) -- */
static GPIO_TypeDef* ILI9341_CS_Port  = NULL; /**< Puerto GPIO del pin CS  */
static uint16_t      ILI9341_CS_Pin   = 0U;   /**< Pin GPIO del pin CS     */
static GPIO_TypeDef* ILI9341_RST_Port = NULL; /**< Puerto GPIO del pin RST */
static uint16_t      ILI9341_RST_Pin  = 0U;   /**< Pin GPIO del pin RST    */
static GPIO_TypeDef* ILI9341_DC_Port  = NULL; /**< Puerto GPIO del pin D/C */
static uint16_t      ILI9341_DC_Pin   = 0U;   /**< Pin GPIO del pin D/C    */

#define ILI9341_RST_SET     HAL_GPIO_WritePin(ILI9341_RST_Port, ILI9341_RST_Pin, GPIO_PIN_SET)   /**< Desactiva el reset por hardware del LCD.  */
#define ILI9341_RST_RESET   HAL_GPIO_WritePin(ILI9341_RST_Port, ILI9341_RST_Pin, GPIO_PIN_RESET) /**< Activa el reset por hardware del LCD.     */
#define ILI9341_CS_SET      HAL_GPIO_WritePin(ILI9341_CS_Port,  ILI9341_CS_Pin,  GPIO_PIN_SET)   /**< Deselecciona el chip SPI del LCD.         */
#define ILI9341_CS_RESET    HAL_GPIO_WritePin(ILI9341_CS_Port,  ILI9341_CS_Pin,  GPIO_PIN_RESET) /**< Selecciona el chip SPI del LCD.           */
#define ILI9341_WRX_SET     HAL_GPIO_WritePin(ILI9341_DC_Port,  ILI9341_DC_Pin,  GPIO_PIN_SET)   /**< Indica modo datos en la línea D/C.        */
#define ILI9341_WRX_RESET   HAL_GPIO_WritePin(ILI9341_DC_Port,  ILI9341_DC_Pin,  GPIO_PIN_RESET) /**< Indica modo comando en la línea D/C.      */

static SPI_HandleTypeDef*    ILI9341_hspi        = NULL; /**< Handle SPI usado para la comunicación con el ILI9341      */
#ifdef HAL_I2C_MODULE_ENABLED
static I2C_HandleTypeDef*    ILI9341_hi2c        = NULL; /**< Handle I2C usado para la comunicación con el STMPE811     */
#endif
static uint8_t               ILI9341_Initialized = 0U;   /**< Bandera de inicialización: 1 = driver listo, 0 = no listo */

static uint16_t              ILI9341_x           = 0U;   /**< Columna del cursor de texto activo                        */
static uint16_t              ILI9341_y           = 0U;   /**< Fila    del cursor de texto activo                        */
static ILI9341_Options_t ILI9341_Opts;                   /**< Geometría y orientación actuales de la pantalla           */

static TP_STATE TP_State; /**< Estado interno del panel táctil (actualizado en ILI9341_TP_GetState()) */
static ILI9341_TouchDriver_t ILI9341_TouchDriver = ILI9341_TOUCH_NONE; /**< Controlador de touch activo */

/* -- Pines del XPT2046 (parametrizados en ILI9341_TP_ConfigXPT2046) -- */
static GPIO_TypeDef* XPT2046_CS_Port  = NULL; /**< Puerto GPIO del pin CS (T_CS) del XPT2046   */
static uint16_t      XPT2046_CS_Pin   = 0U;   /**< Pin GPIO del pin CS del XPT2046             */
static GPIO_TypeDef* XPT2046_IRQ_Port = NULL; /**< Puerto GPIO del pin PENIRQ, o NULL si no usado */
static uint16_t      XPT2046_IRQ_Pin  = 0U;   /**< Pin GPIO del pin PENIRQ                     */

static uint32_t* ILI9341_framebuffer      = NULL; /**< Front buffer: transmitido a la LCD por DMA en modo doble buffer     */
static uint32_t* ILI9341_back_framebuffer = NULL; /**< Back buffer: la CPU dibuja aquí mientras el DMA envía el front      */

#ifdef HAL_DMA2D_MODULE_ENABLED
static DMA2D_HandleTypeDef* ILI9341_hdma2d = NULL; /**< Handle DMA2D inyectado en Init; NULL si no se habilitó o se pasó NULL */
#endif

static volatile uint8_t ILI9341_spi_dma_done = 0U; /**< Bandera puesta por HAL_SPI_TxCpltCallback al completar toda la transferencia. */

/* Estado del DMA async (doble buffer). El callback usa estas variables para
 * encadenar el segundo tramo automáticamente en el modo pipelining. */
static volatile uint8_t ILI9341_dma_state  = 0U;    /**< 0=idle, 1=primer tramo en vuelo, 2=segundo tramo en vuelo */
static volatile uint8_t ILI9341_dma_cs_held = 0U;   /**< 1 mientras CS está bajo por un FlushAsync activo          */
static volatile uint16_t*  ILI9341_dma_px2    = NULL;   /**< Puntero al inicio del segundo tramo (píxeles 38 400–76 799) */
static volatile uint16_t   ILI9341_dma_half   = 0U;    /**< Tamaño de cada tramo en items de 16 bits                   */

// ============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
// ============================================================================

static ILI9341_Status_t SPI_ILI9341_Send(uint8_t* data, uint16_t size);
static ILI9341_Status_t SPI_ILI9341_BaudRateUp(void);
static ILI9341_Status_t SPI_ILI9341_WaitTXE(void);
static ILI9341_Status_t ILI9341_SendCommand(uint8_t data);
static ILI9341_Status_t ILI9341_SendData(uint8_t data);
static void ILI9341_Delay(volatile unsigned int delay);
static ILI9341_Status_t ILI9341_SetCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
#ifdef HAL_I2C_MODULE_ENABLED
static uint8_t ILI9341_TP_ReadDeviceRegister(uint8_t RegisterAddr);
static ILI9341_Status_t ILI9341_TP_WriteDeviceRegister(uint8_t RegisterAddr, uint8_t RegisterValue);
static uint16_t ILI9341_TP_ReadDataBuffer(uint32_t RegisterAddr);
static ILI9341_Status_t ILI9341_TP_IOAFConfig(uint8_t IO_Pin, FunctionalState NewState);
static ILI9341_Status_t ILI9341_TP_FnctCmd(uint8_t Fct, FunctionalState NewState);
static ILI9341_Status_t ILI9341_TP_Reset(void);
static uint16_t ILI9341_TP_Read_X(void);
static uint16_t ILI9341_TP_Read_Y(void);
static uint16_t ILI9341_TP_Read_Z(void);
static TP_STATE* STMPE811_TP_GetState(void);
#endif /* HAL_I2C_MODULE_ENABLED */
static void SPI_ILI9341_SetPrescaler(uint32_t prescaler);
static ILI9341_Status_t XPT2046_ReadChannel(uint8_t cmd, uint16_t* result);
static ILI9341_Status_t XPT2046_ReadRaw(uint16_t* x, uint16_t* y, uint16_t* z);
static TP_STATE* XPT2046_TP_GetState(void);
static ILI9341_Status_t DrawPixelClipped(int16_t x, int16_t y, uint16_t color);
static ILI9341_Status_t DrawHSpanClipped(int16_t xL, int16_t xR, int16_t y, uint16_t color);
static ILI9341_Status_t DrawHSpanClipped_ImageBuffer(int16_t xL, int16_t xR, int16_t y, uint16_t color, uint32_t* image);
static ILI9341_Status_t DrawPixelClipped_ImageBuffer(int16_t x, int16_t y, uint16_t color, uint32_t* image);
#ifdef HAL_DMA2D_MODULE_ENABLED
static void ILI9341_DMA2D_SetMode(uint32_t mode, uint32_t output_offset);
#endif
static ILI9341_Status_t ILI9341_SPI_SetDataSize(uint32_t datasize);
static ILI9341_Status_t ILI9341_SPI_WaitDMAdone(uint32_t timeout_ms);
static ILI9341_Status_t ILI9341_FlushAsync(void);
static ILI9341_Status_t ILI9341_PresentFrame(void);

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Transmite un bloque de bytes al ILI9341 por el periférico SPI configurado.
 *
 * @param[in] data  Puntero al buffer de bytes a transmitir.
 * @param[in] size  Número de bytes a transmitir.
 * @return ILI9341_OK si HAL_SPI_Transmit devuelve HAL_OK; ILI9341_ERROR en caso contrario.
 */
static ILI9341_Status_t SPI_ILI9341_Send(uint8_t* data, uint16_t size)
{
    return (HAL_SPI_Transmit(ILI9341_hspi, data, size, 5000U) == HAL_OK)
           ? ILI9341_OK : ILI9341_ERROR;
}

/**
 * @brief Re-inicializa el periférico SPI con el preescalador mínimo (máxima velocidad).
 *
 * @details Tras la secuencia de inicialización del ILI9341 — que requiere una velocidad
 *          conservadora — eleva la tasa de bits a ~45 Mbit/s estableciendo
 *          @c SPI_BAUDRATEPRESCALER_2 en el campo @c Init.BaudRatePrescaler del handle SPI.
 */
static ILI9341_Status_t SPI_ILI9341_BaudRateUp(void)
{
    if (HAL_SPI_DeInit(ILI9341_hspi) != HAL_OK) { return ILI9341_ERROR; }
    ILI9341_hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    return (HAL_SPI_Init(ILI9341_hspi) == HAL_OK) ? ILI9341_OK : ILI9341_ERROR;
}

/** @brief Iteraciones máximas de sondeo de TXE antes de declarar el bus colgado.
 *  Un byte a ~45 Mbit/s tarda ~32 ciclos de CPU; este límite (decenas de ms)
 *  solo se agota ante una falla real del bus. */
#define ILI9341_SPI_TXE_SPIN_MAX 1000000U

/**
 * @brief Cambia el preescalador de baud rate del SPI sin reinicializar el handle completo.
 *
 * @details Usado para bajar la velocidad del bus SPI compartido antes de dialogar con
 *          el XPT2046 (máx. ~2 MHz) y restaurarla después, sin el costo de un ciclo
 *          completo HAL_SPI_DeInit/Init como hace SPI_ILI9341_BaudRateUp(). Espera a
 *          que el bus esté libre antes de tocar el registro, igual que
 *          ILI9341_SPI_SetDataSize().
 *
 * @param[in] prescaler Uno de los valores SPI_BAUDRATEPRESCALER_x del HAL.
 */
static void SPI_ILI9341_SetPrescaler(uint32_t prescaler)
{
    uint32_t spin = ILI9341_SPI_TXE_SPIN_MAX;
    while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_BSY))
    {
        if (--spin == 0U) { break; }
    }
    __HAL_SPI_DISABLE(ILI9341_hspi);
    MODIFY_REG(ILI9341_hspi->Instance->CR1, SPI_CR1_BR, prescaler);
    ILI9341_hspi->Init.BaudRatePrescaler = prescaler;
    __HAL_SPI_ENABLE(ILI9341_hspi);
}

/**
 * @brief Espera a que TXE se active tras escribir en el DR (ruta crítica de envío de píxeles).
 *
 * @details Sondea TXE con un límite de iteraciones en lugar de HAL_GetTick(),
 *          eliminando una llamada a función y una lectura de tick por byte en
 *          los bucles de volcado. Si el límite se agota, restaura el periférico
 *          SPI y los pines CS/WRX igual que la ruta de timeout basada en ticks.
 *
 * @return ILI9341_OK si TXE se activó; ILI9341_TIMEOUT si el bus quedó colgado.
 */
static ILI9341_Status_t SPI_ILI9341_WaitTXE(void)
{
    uint32_t spin = ILI9341_SPI_TXE_SPIN_MAX;
    while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_TXE) == RESET)
    {
        if (--spin == 0U)
        {
            __HAL_SPI_DISABLE_IT(ILI9341_hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
            __HAL_SPI_DISABLE(ILI9341_hspi);
            if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) { SPI_RESET_CRC(ILI9341_hspi); }
            ILI9341_hspi->State = HAL_SPI_STATE_READY;
            __HAL_UNLOCK(ILI9341_hspi);
            ILI9341_CS_SET;
            ILI9341_WRX_RESET;
            return ILI9341_TIMEOUT;
        }
    }
    return ILI9341_OK;
}

/**
 * @brief Rellena un segmento angular (arco) entre dos ángulos y dos radios.
 *
 * @details Recorre el cuadro delimitador del arco fila por fila y dibuja,
 *          en cada una, los segmentos horizontales que caen dentro de la
 *          corona circular [iradius, oradius] y del sector angular
 *          [start, end]. Usado por ILI9341_DrawArc tanto para los bordes
 *          rectos del arco como para sus bordes curvos.
 *
 * @param cx      Coordenada X del centro.
 * @param cy      Coordenada Y del centro.
 * @param oradius Radio exterior del arco.
 * @param iradius Radio interior del arco.
 * @param start   Ángulo inicial en grados (0 a 360).
 * @param end     Ángulo final en grados (0 a 360).
 * @param color   Color a utilizar.
 *
 * @return ILI9341_Status_t Estado de la operación.
 */
static ILI9341_Status_t ILI9341_FillArcHelper(int16_t cx, int16_t cy, int16_t oradius, int16_t iradius, float start, float end, uint16_t color, uint32_t* image)
{
    ILI9341_Status_t st;
    float s_cos, e_cos, sslope, eslope, swidth, ewidth;
    int32_t ir2, or2, xs, y, ye, xe;
    bool start180, end180, reversed;

    if ((start == 0.0f) || (start == 90.0f) || (start == 180.0f) || (start == 270.0f) || (start == 360.0f))
    {
        start += 0.1f;
    }
    if ((end == 0.0f) || (end == 90.0f) || (end == 180.0f) || (end == 270.0f) || (end == 360.0f))
    {
        end += 0.1f;
    }

    s_cos  = cosf(start * 0.0174532925f);
    e_cos  = cosf(end * 0.0174532925f);
    sslope = s_cos / sinf(start * 0.0174532925f);
    eslope = e_cos / sinf(end * 0.0174532925f);
    swidth = 0.5f / s_cos;
    ewidth = -0.5f / e_cos;
    --iradius;
    ir2 = (int32_t)iradius * iradius + iradius;
    or2 = (int32_t)oradius * oradius + oradius;

    start180 = !(start < 180.0f);
    end180   = end < 180.0f;
    reversed = (start + 180.0f < end) || (end < start && start < end + 180.0f);

    xs = -oradius;
    y  = -oradius;
    ye = oradius;
    xe = oradius + 1;
    if (!reversed)
    {
        if ((end >= 270.0f || end < 90.0f) && (start >= 270.0f || start < 90.0f))
        {
            xs = 0;
        }
        else if (end < 270.0f && end >= 90.0f && start < 270.0f && start >= 90.0f)
        {
            xe = 1;
        }
        if (end >= 180.0f && start >= 180.0f)
        {
            ye = 0;
        }
        else if (end < 180.0f && start < 180.0f)
        {
            y = 0;
        }
    }

    do
    {
        int32_t y2 = y * y;
        int32_t x  = xs;
        int32_t len = 0;
        float ysslope, yeslope;

        if (x < 0)
        {
            while (x * x + y2 >= or2)
            {
                ++x;
            }
            if (xe != 1)
            {
                xe = 1 - x;
            }
        }

        ysslope = ((float)y + swidth) * sslope;
        yeslope = ((float)y + ewidth) * eslope;

        do
        {
            bool flg1 = start180 != (x <= ysslope);
            bool flg2 = end180 != (x <= yeslope);
            int32_t distance = x * x + y2;

            if (distance >= ir2 && ((flg1 && flg2) || (reversed && (flg1 || flg2))) && x != xe && distance < or2)
            {
                ++len;
            }
            else
            {
                if (len)
                {
                    if (image != NULL)
                    {
                        st = DrawHSpanClipped_ImageBuffer(cx + x - len, cx + x - 1, cy + y, color, image);
                    }
                    else
                    {
                        st = DrawHSpanClipped(cx + x - len, cx + x - 1, cy + y, color);
                    }
                    if (st != ILI9341_OK) { return st; }
                    len = 0;
                }
                if (distance >= or2)
                {
                    break;
                }
                if (x < 0 && distance < ir2)
                {
                    x = -x;
                }
            }
        } while (++x <= xe);
    } while (++y <= ye);

    return ILI9341_OK;
}

/**
 * @brief Envía un byte de comando a la pantalla LCD por SPI.
 *
 * @param[in] data Byte de comando.
 * @return ILI9341_Status_t
 *         - ILI9341_OK            en caso de éxito.
 *         - ILI9341_INVALID_PARAM si el handle SPI es NULL.
 *         - ILI9341_ERROR         si falla la transmisión SPI.
 */
static ILI9341_Status_t ILI9341_SendCommand(uint8_t data)
{
    ILI9341_Status_t st;
    if (ILI9341_hspi == NULL) { return ILI9341_INVALID_PARAM; }
    ILI9341_WRX_RESET;
    ILI9341_CS_RESET;
    st = SPI_ILI9341_Send(&data, 1U);
    ILI9341_CS_SET;
    return st;
}

/**
 * @brief Envía un byte de datos a la pantalla LCD por SPI.
 *
 * @param[in] data Byte de datos.
 * @return ILI9341_Status_t
 *         - ILI9341_OK            en caso de éxito.
 *         - ILI9341_INVALID_PARAM si el handle SPI es NULL.
 *         - ILI9341_ERROR         si falla la transmisión SPI.
 */
static ILI9341_Status_t ILI9341_SendData(uint8_t data)
{
    ILI9341_Status_t st;
    if (ILI9341_hspi == NULL) { return ILI9341_INVALID_PARAM; }
    ILI9341_WRX_SET;
    ILI9341_CS_RESET;
    st = SPI_ILI9341_Send(&data, 1U);
    ILI9341_CS_SET;
    return st;
}

/**
 * @brief Bucle de retardo por software (espera activa, no milisegundos).
 *
 * @param[in] delay Número de iteraciones del bucle.
 */
static void ILI9341_Delay(volatile unsigned int delay)
{
    for (; delay != 0U; delay--);
}

/**
 * @brief Establece la ventana de dibujo activa en la pantalla LCD (COLUMN_ADDR + PAGE_ADDR).
 *
 * @param[in] x1 Columna izquierda de la ventana.
 * @param[in] y1 Fila superior de la ventana.
 * @param[in] x2 Columna derecha de la ventana.
 * @param[in] y2 Fila inferior de la ventana.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
static ILI9341_Status_t ILI9341_SetCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    ILI9341_Status_t st;
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    st  = ILI9341_SendCommand(ILI9341_COLUMN_ADDR);
    st  = st ? st : ILI9341_SendData((uint8_t)(x1 >> 8));
    st  = st ? st : ILI9341_SendData((uint8_t)(x1 & 0xFFU));
    st  = st ? st : ILI9341_SendData((uint8_t)(x2 >> 8));
    st  = st ? st : ILI9341_SendData((uint8_t)(x2 & 0xFFU));
    st  = st ? st : ILI9341_SendCommand(ILI9341_PAGE_ADDR);
    st  = st ? st : ILI9341_SendData((uint8_t)(y1 >> 8));
    st  = st ? st : ILI9341_SendData((uint8_t)(y1 & 0xFFU));
    st  = st ? st : ILI9341_SendData((uint8_t)(y2 >> 8));
    st  = st ? st : ILI9341_SendData((uint8_t)(y2 & 0xFFU));
    return st;
}

#ifdef HAL_I2C_MODULE_ENABLED
/**
 * @brief Lee un byte de un registro del STMPE811 por I2C.
 *
 * @param[in] RegisterAddr Dirección del registro (0x00–0x59).
 * @return Valor del registro, o 0xAA si hay error I2C o no está inicializado.
 */
static uint8_t ILI9341_TP_ReadDeviceRegister(uint8_t RegisterAddr)
{
    uint8_t tmp = 0U;
    uint8_t Address;
    uint8_t buf[2] = { RegisterAddr, RegisterAddr };

    if (!ILI9341_Initialized) { return 0xAAU; }

    Address  = TP_ADDR;
    Address &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
    if (HAL_I2C_Master_Transmit(ILI9341_hi2c, Address, buf, 1U, 1000U) != HAL_OK)
    {
        return 0xAAU;
    }
    Address  = TP_ADDR;
    Address |= I2C_OAR1_ADD0;
    if (HAL_I2C_Master_Receive(ILI9341_hi2c, Address, &buf[0], 1U, 1000U) != HAL_OK)
    {
        return 0xAAU;
    }
    tmp = buf[0];

    return tmp;
}

/**
 * @brief Escribe un byte en un registro del STMPE811 por I2C.
 *
 * @param[in] RegisterAddr  Dirección del registro.
 * @param[in] RegisterValue Byte a escribir.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la comunicación I2C.
 */
static ILI9341_Status_t ILI9341_TP_WriteDeviceRegister(uint8_t RegisterAddr, uint8_t RegisterValue)
{
    uint8_t Address;
    uint8_t buf[2] = { RegisterAddr, RegisterValue };

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    Address  = TP_ADDR;
    Address &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
    if (HAL_I2C_Master_Transmit(ILI9341_hi2c, Address, buf, 2U, 1000U) != HAL_OK)
    {
        return ILI9341_ERROR;
    }

    return ILI9341_OK;
}

/**
 * @brief Lee dos bytes de un registro del STMPE811 (usado para datos ADC de X/Y/Z).
 *
 * @param[in] RegisterAddr Dirección del registro.
 * @return Valor reconstruido de 16 bits, o 0xAA si hay error I2C o no está inicializado.
 */
static uint16_t ILI9341_TP_ReadDataBuffer(uint32_t RegisterAddr)
{
    uint8_t Address;
    uint8_t TP_BufferRX[2] = { 0U, 0U };
    uint8_t buf             = (uint8_t)RegisterAddr;

    if (!ILI9341_Initialized) { return 0xAAU; }

    Address  = TP_ADDR;
    Address &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
    if (HAL_I2C_Master_Transmit(ILI9341_hi2c, Address, &buf, 1U, 1000U) != HAL_OK)
    {
        return 0xAAU;
    }
    Address  = TP_ADDR;
    Address |= I2C_OAR1_ADD0;
    if (HAL_I2C_Master_Receive(ILI9341_hi2c, Address, TP_BufferRX, 2U, 1000U) != HAL_OK)
    {
        return 0xAAU;
    }

    return (uint16_t)TP_BufferRX[1] | ((uint16_t)TP_BufferRX[0] << 8);
}

/**
 * @brief Configura el modo de función alternativa para los pines GPIO del STMPE811.
 *
 * @param[in] IO_Pin   Máscara de pin (valores IO_Pin_x).
 * @param[in] NewState ENABLE o DISABLE.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              siempre, una vez inicializado.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 */
static ILI9341_Status_t ILI9341_TP_IOAFConfig(uint8_t IO_Pin, FunctionalState NewState)
{
    uint8_t tmp;
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    tmp = ILI9341_TP_ReadDeviceRegister(TP_REG_GPIO_AF);
    if (NewState != DISABLE) { tmp |=  (uint8_t)IO_Pin; }
    else                     { tmp &= ~(uint8_t)IO_Pin; }
    ILI9341_TP_WriteDeviceRegister(TP_REG_GPIO_AF, tmp);
    return ILI9341_OK;
}

/**
 * @brief Habilita o deshabilita bloques funcionales del STMPE811 (ADC, panel táctil, GPIO).
 *
 * @param[in] Fct      Máscara de función: TP_ADC_FCT, TP_TP_FCT, o TP_IO_FCT.
 * @param[in] NewState ENABLE o DISABLE.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              siempre, una vez inicializado.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 */
static ILI9341_Status_t ILI9341_TP_FnctCmd(uint8_t Fct, FunctionalState NewState)
{
    uint8_t tmp;
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    tmp = ILI9341_TP_ReadDeviceRegister(TP_REG_SYS_CTRL2);
    if (NewState != DISABLE) { tmp &= ~(uint8_t)Fct; }
    else                     { tmp |=  (uint8_t)Fct; }
    ILI9341_TP_WriteDeviceRegister(TP_REG_SYS_CTRL2, tmp);
    return ILI9341_OK;
}

/**
 * @brief Reinicia el STMPE811 mediante el bit de reset por software SYS_CTRL1.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              siempre, una vez inicializado.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 */
static ILI9341_Status_t ILI9341_TP_Reset(void)
{
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    ILI9341_TP_WriteDeviceRegister(TP_REG_SYS_CTRL1, 0x02U);
    ILI9341_Delay(2U);
    ILI9341_TP_WriteDeviceRegister(TP_REG_SYS_CTRL1, 0x00U);
    return ILI9341_OK;
}

/**
 * @brief Lee la coordenada X calibrada del punto de toque activo.
 *
 * @return Coordenada X en el rango [0, 239], o 0 si no está inicializado.
 */
static uint16_t ILI9341_TP_Read_X(void)
{
    int32_t x, xr;

    if (!ILI9341_Initialized) { return 0U; }

    x  = (int32_t)ILI9341_TP_ReadDataBuffer(TP_REG_TP_DATA_X);
    x  = (x <= 3000) ? (3870 - x) : (3800 - x);
    xr = x / 15;
    if      (xr <= 0)  { xr = 0;   }
    else if (xr > 240) { xr = 239; }

    xr = 239 - xr;

    return (uint16_t)xr;
}

/**
 * @brief Lee la coordenada Y calibrada del punto de toque activo.
 *
 * @return Coordenada Y en el rango [0, 319], o 0 si no está inicializado.
 */
static uint16_t ILI9341_TP_Read_Y(void)
{
    int32_t y, yr;

    if (!ILI9341_Initialized) { return 0U; }

    y  = (int32_t)ILI9341_TP_ReadDataBuffer(TP_REG_TP_DATA_Y);
    y -= 360;
    yr = y / 11;
    if      (yr <= 0)  { yr = 0;   }
    else if (yr > 320) { yr = 319; }

    return (uint16_t)yr;
}

/**
 * @brief Lee el valor Z (presión) del punto de toque activo.
 *
 * @return Índice de presión (valor ADC crudo), o 0 si no está inicializado.
 */
static uint16_t ILI9341_TP_Read_Z(void)
{
    if (!ILI9341_Initialized) { return 0U; }
    return (uint16_t)ILI9341_TP_ReadDataBuffer(TP_REG_TP_DATA_Z);
}
#endif /* HAL_I2C_MODULE_ENABLED */

/**
 * @brief Dibuja un píxel en (x, y) con el color dado, recortando a los límites de la pantalla.
 *
 * @param x Coordenada X del píxel (puede ser negativa o exceder el ancho de la pantalla).
 * @param y Coordenada Y del píxel (puede ser negativa o exceder el alto de la pantalla).
 * @param color Color del píxel en formato RGB565.
 * @return ILI9341_Status_t
 */
static ILI9341_Status_t DrawPixelClipped(int16_t x, int16_t y, uint16_t color)
{
    if (x < 0 || y < 0 || (uint16_t)x >= ILI9341_Opts.width || (uint16_t)y >= ILI9341_Opts.height)
        return ILI9341_OK;
    return ILI9341_DrawPixel((uint16_t)x, (uint16_t)y, color);
}

/**
 * @brief Dibuja un tramo horizontal entre xL y xR en la fila y, recortando a los límites de la pantalla.
 *
 * @param xL    Extremo izquierdo del tramo (puede ser negativo o mayor que xR).
 * @param xR    Extremo derecho del tramo.
 * @param y     Fila del tramo (puede ser negativa o exceder el alto de la pantalla).
 * @param color Color del tramo en formato RGB565.
 * @return ILI9341_Status_t
 */
static ILI9341_Status_t DrawHSpanClipped(int16_t xL, int16_t xR, int16_t y, uint16_t color)
{
    int16_t tmp;
    if (y < 0 || (uint16_t)y >= ILI9341_Opts.height) return ILI9341_OK;
    if (xL > xR) { tmp = xL; xL = xR; xR = tmp; }
    if (xL < 0) xL = 0;
    if ((uint16_t)xR >= ILI9341_Opts.width) xR = (int16_t)(ILI9341_Opts.width - 1U);
    if (xL > xR) return ILI9341_OK;
    return ILI9341_DrawFilledRectangle((uint16_t)xL, (uint16_t)y, (uint16_t)xR, (uint16_t)y, color);
}

/**
 * @brief Dibuja un tramo horizontal entre xL y xR en la fila y del frame buffer, recortando a los límites de pantalla.
 *
 * @param xL    Extremo izquierdo (puede ser negativo o mayor que xR).
 * @param xR    Extremo derecho.
 * @param y     Fila (puede ser negativa o exceder el alto de pantalla).
 * @param color Color del tramo en formato RGB565.
 * @param image Frame buffer destino.
 */
static ILI9341_Status_t DrawHSpanClipped_ImageBuffer(int16_t xL, int16_t xR, int16_t y, uint16_t color, uint32_t* image)
{
    int16_t tmp;
    if (y < 0 || (uint16_t)y >= ILI9341_HEIGHT) { return ILI9341_OK; }
    if (xL > xR) { tmp = xL; xL = xR; xR = tmp; }
    if (xL < 0) { xL = 0; }
    if ((uint16_t)xR >= ILI9341_WIDTH) { xR = (int16_t)(ILI9341_WIDTH - 1U); }
    if (xL > xR) { return ILI9341_OK; }
    return ILI9341_DrawFilledRectangle_ImageBuffer((uint16_t)xL, (uint16_t)y, (uint16_t)xR, (uint16_t)y, color, image);
}

/**
 * @brief Dibuja un píxel en (x, y) del frame buffer con recorte a los límites fijos del panel.
 */
static ILI9341_Status_t DrawPixelClipped_ImageBuffer(int16_t x, int16_t y, uint16_t color, uint32_t* image)
{
    if (x < 0 || y < 0 || (uint16_t)x >= ILI9341_WIDTH || (uint16_t)y >= ILI9341_HEIGHT)
        return ILI9341_OK;
    return ILI9341_DrawPixel_ImageBuffer((uint16_t)x, (uint16_t)y, color, image);
}

#ifdef HAL_DMA2D_MODULE_ENABLED
/**
 * @brief Reprograma modo y offset de salida del DMA2D sin reinicializar el periférico.
 *
 * @details La configuración constante (reloj, formato de salida RGB565 y la capa de
 *          entrada para M2M) se realiza una sola vez en ILI9341_Init(). Esta función
 *          solo actualiza los dos parámetros que cambian por operación, evitando llamar
 *          a HAL_DMA2D_Init()/HAL_DMA2D_ConfigLayer() en cada dibujo. El periférico está
 *          inactivo entre transferencias (tras HAL_DMA2D_PollForTransfer), por lo que es
 *          seguro escribir CR/OOR directamente antes del siguiente HAL_DMA2D_Start().
 *
 * @param[in] mode          Modo de operación (DMA2D_R2M para relleno, DMA2D_M2M para copia).
 * @param[in] output_offset Píxeles a saltar al final de cada línea del destino.
 */
static void ILI9341_DMA2D_SetMode(uint32_t mode, uint32_t output_offset)
{
    ILI9341_hdma2d->Init.Mode         = mode;          /* DMA2D_SetConfig() lo lee para elegir R2M/M2M */
    ILI9341_hdma2d->Init.OutputOffset = output_offset;
    MODIFY_REG(ILI9341_hdma2d->Instance->CR,  DMA2D_CR_MODE, mode);
    MODIFY_REG(ILI9341_hdma2d->Instance->OOR, DMA2D_OOR_LO,  output_offset);
}
#endif /* HAL_DMA2D_MODULE_ENABLED */

/**
 * @brief Cambia el tamaño de dato del periférico SPI sin reinicializar el handle completo.
 *
 * @details Espera a que el bus SPI esté libre, deshabilita el periférico,
 *          modifica el bit DFF en CR1 y actualiza Init.DataSize para que
 *          HAL_SPI_Transmit_DMA configure el DMA correctamente.
 *
 * @param[in] datasize  SPI_DATASIZE_8BIT o SPI_DATASIZE_16BIT.
 * @return ILI9341_OK o ILI9341_TIMEOUT si el bus quedó colgado.
 */
static ILI9341_Status_t ILI9341_SPI_SetDataSize(uint32_t datasize)
{
    uint32_t spin = ILI9341_SPI_TXE_SPIN_MAX;
    while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_BSY))
    {
        if (--spin == 0U) { return ILI9341_TIMEOUT; }
    }
    __HAL_SPI_DISABLE(ILI9341_hspi);
    MODIFY_REG(ILI9341_hspi->Instance->CR1, SPI_CR1_DFF, datasize);
    ILI9341_hspi->Init.DataSize = datasize;
    __HAL_SPI_ENABLE(ILI9341_hspi);
    return ILI9341_OK;
}

/**
 * @brief Bloquea hasta que ILI9341_spi_dma_done sea distinto de cero o expire el timeout.
 *
 * @details Limpia la bandera antes de retornar en caso de éxito.
 *          Si expira el timeout llama a HAL_SPI_DMAStop para abortar la transferencia.
 *
 * @param[in] timeout_ms Tiempo máximo de espera en milisegundos.
 * @return ILI9341_OK o ILI9341_TIMEOUT.
 */
static ILI9341_Status_t ILI9341_SPI_WaitDMAdone(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (!ILI9341_spi_dma_done)
    {
        if ((HAL_GetTick() - t0) > timeout_ms)
        {
            HAL_SPI_DMAStop(ILI9341_hspi);
            ILI9341_dma_state = 0U;
            return ILI9341_TIMEOUT;
        }
    }
    ILI9341_spi_dma_done = 0U;
    return ILI9341_OK;
}

/**
 * @brief Callback de fin de transferencia SPI por DMA (override del símbolo __weak del HAL).
 *
 * @details Llamado desde el ISR de DMA2_Stream6 cuando SPI5 termina un tramo DMA.
 *          En modo pipelining (ILI9341_FlushAsync): si acaba el primer tramo encadena
 *          automáticamente el segundo; al terminar el segundo señaliza con spi_dma_done.
 *          En modo bloqueante (ILI9341_DisplayImage): dma_state es 0 en ambos tramos,
 *          por lo que el comportamiento es idéntico al original (señaliza spi_dma_done).
 *
 * @param[in] hspi Handle SPI que completó la transferencia.
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != ILI9341_hspi) { return; }

    if (ILI9341_dma_state == 1U)
    {
        /* Primer tramo del modo async completado: encadenar el segundo automáticamente. */
        ILI9341_dma_state = 2U;
        if (HAL_SPI_Transmit_DMA(ILI9341_hspi, (uint8_t*)ILI9341_dma_px2, ILI9341_dma_half) != HAL_OK)
        {
            /* Si el segundo tramo no pudo iniciarse, desbloquear el waiter limpiamente. */
            ILI9341_spi_dma_done = 1U;
            ILI9341_dma_state    = 0U;
        }
    }
    else
    {
        /* Segundo tramo (o tramo único en modo bloqueante): señalizar fin de frame. */
        ILI9341_spi_dma_done = 1U;
        ILI9341_dma_state    = 0U;
    }
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

#if defined(HAL_I2C_MODULE_ENABLED) && defined(HAL_DMA2D_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi,
                               GPIO_TypeDef* csPort, uint16_t csPin,
                               GPIO_TypeDef* rstPort, uint16_t rstPin,
                               GPIO_TypeDef* dcPort, uint16_t dcPin,
                               I2C_HandleTypeDef* hi2c, DMA2D_HandleTypeDef* hdma2d)
#elif defined(HAL_I2C_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi,
                               GPIO_TypeDef* csPort, uint16_t csPin,
                               GPIO_TypeDef* rstPort, uint16_t rstPin,
                               GPIO_TypeDef* dcPort, uint16_t dcPin,
                               I2C_HandleTypeDef* hi2c)
#elif defined(HAL_DMA2D_MODULE_ENABLED)
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi,
                               GPIO_TypeDef* csPort, uint16_t csPin,
                               GPIO_TypeDef* rstPort, uint16_t rstPin,
                               GPIO_TypeDef* dcPort, uint16_t dcPin,
                               DMA2D_HandleTypeDef* hdma2d)
#else
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi,
                               GPIO_TypeDef* csPort, uint16_t csPin,
                               GPIO_TypeDef* rstPort, uint16_t rstPin,
                               GPIO_TypeDef* dcPort, uint16_t dcPin)
#endif
{
    ILI9341_Status_t st;

    if (hspi == NULL || csPort == NULL || rstPort == NULL || dcPort == NULL) { return ILI9341_INVALID_PARAM; }
#ifdef HAL_I2C_MODULE_ENABLED
    if (hi2c == NULL) { return ILI9341_INVALID_PARAM; }
#endif

    ILI9341_hspi = hspi;

    ILI9341_CS_Port  = csPort;  ILI9341_CS_Pin  = csPin;
    ILI9341_RST_Port = rstPort; ILI9341_RST_Pin = rstPin;
    ILI9341_DC_Port  = dcPort;  ILI9341_DC_Pin  = dcPin;

#ifdef HAL_I2C_MODULE_ENABLED
    ILI9341_hi2c        = hi2c;
#endif
    ILI9341_Initialized = 0U;

    ILI9341_framebuffer      = NULL;
    ILI9341_back_framebuffer = NULL;

    ILI9341_CS_SET;
    ILI9341_RST_SET;

    st = ILI9341_SendCommand(ILI9341_RESET);
    ILI9341_Delay(200000);

    st = st ? st : ILI9341_SendCommand(ILI9341_POWERA);
    st = st ? st : ILI9341_SendData(0x39);
    st = st ? st : ILI9341_SendData(0x2C);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0x34);
    st = st ? st : ILI9341_SendData(0x02);
    st = st ? st : ILI9341_SendCommand(ILI9341_POWERB);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0xC1);
    st = st ? st : ILI9341_SendData(0x30);
    st = st ? st : ILI9341_SendCommand(ILI9341_DTCA);
    st = st ? st : ILI9341_SendData(0x85);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0x78);
    st = st ? st : ILI9341_SendCommand(ILI9341_DTCB);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendCommand(ILI9341_POWER_SEQ);
    st = st ? st : ILI9341_SendData(0x64);
    st = st ? st : ILI9341_SendData(0x03);
    st = st ? st : ILI9341_SendData(0x12);
    st = st ? st : ILI9341_SendData(0x81);
    st = st ? st : ILI9341_SendCommand(ILI9341_PRC);
    st = st ? st : ILI9341_SendData(0x20);
    st = st ? st : ILI9341_SendCommand(ILI9341_POWER1);
    st = st ? st : ILI9341_SendData(0x23);
    st = st ? st : ILI9341_SendCommand(ILI9341_POWER2);
    st = st ? st : ILI9341_SendData(0x10);
    st = st ? st : ILI9341_SendCommand(ILI9341_VCOM1);
    st = st ? st : ILI9341_SendData(0x3E);
    st = st ? st : ILI9341_SendData(0x28);
    st = st ? st : ILI9341_SendCommand(ILI9341_VCOM2);
    st = st ? st : ILI9341_SendData(0x86);
    st = st ? st : ILI9341_SendCommand(ILI9341_MAC);
    st = st ? st : ILI9341_SendData(0x48);
    st = st ? st : ILI9341_SendCommand(ILI9341_PIXEL_FORMAT);
    st = st ? st : ILI9341_SendData(0x55);
    st = st ? st : ILI9341_SendCommand(ILI9341_FRC);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0x18);
    st = st ? st : ILI9341_SendCommand(ILI9341_DFC);
    st = st ? st : ILI9341_SendData(0x08);
    st = st ? st : ILI9341_SendData(0x82);
    st = st ? st : ILI9341_SendData(0x27);
    st = st ? st : ILI9341_SendCommand(ILI9341_3GAMMA_EN);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendCommand(ILI9341_COLUMN_ADDR);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0xEF);
    st = st ? st : ILI9341_SendCommand(ILI9341_PAGE_ADDR);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0x01);
    st = st ? st : ILI9341_SendData(0x3F);
    st = st ? st : ILI9341_SendCommand(ILI9341_GAMMA);
    st = st ? st : ILI9341_SendData(0x01);
    st = st ? st : ILI9341_SendCommand(ILI9341_PGAMMA);
    st = st ? st : ILI9341_SendData(0x0F);
    st = st ? st : ILI9341_SendData(0x31);
    st = st ? st : ILI9341_SendData(0x2B);
    st = st ? st : ILI9341_SendData(0x0C);
    st = st ? st : ILI9341_SendData(0x0E);
    st = st ? st : ILI9341_SendData(0x08);
    st = st ? st : ILI9341_SendData(0x4E);
    st = st ? st : ILI9341_SendData(0xF1);
    st = st ? st : ILI9341_SendData(0x37);
    st = st ? st : ILI9341_SendData(0x07);
    st = st ? st : ILI9341_SendData(0x10);
    st = st ? st : ILI9341_SendData(0x03);
    st = st ? st : ILI9341_SendData(0x0E);
    st = st ? st : ILI9341_SendData(0x09);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendCommand(ILI9341_NGAMMA);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0x0E);
    st = st ? st : ILI9341_SendData(0x14);
    st = st ? st : ILI9341_SendData(0x03);
    st = st ? st : ILI9341_SendData(0x11);
    st = st ? st : ILI9341_SendData(0x07);
    st = st ? st : ILI9341_SendData(0x31);
    st = st ? st : ILI9341_SendData(0xC1);
    st = st ? st : ILI9341_SendData(0x48);
    st = st ? st : ILI9341_SendData(0x08);
    st = st ? st : ILI9341_SendData(0x0F);
    st = st ? st : ILI9341_SendData(0x0C);
    st = st ? st : ILI9341_SendData(0x31);
    st = st ? st : ILI9341_SendData(0x36);
    st = st ? st : ILI9341_SendData(0x0F);
    st = st ? st : ILI9341_SendCommand(ILI9341_INTERFACE);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendData(0x00);
    st = st ? st : ILI9341_SendCommand(ILI9341_SLEEP_OUT);
    if (st != ILI9341_OK) { return st; }

    ILI9341_Delay(1000000);

    st = ILI9341_SendCommand(ILI9341_DISPLAY_ON);
    st = st ? st : ILI9341_SendCommand(ILI9341_GRAM);
    if (st != ILI9341_OK) { return st; }

    ILI9341_x                = 0U;
    ILI9341_y                = 0U;
    ILI9341_Opts.width       = ILI9341_WIDTH;
    ILI9341_Opts.height      = ILI9341_HEIGHT;
    ILI9341_Opts.orientation = ILI9341_Portrait;

    st = SPI_ILI9341_BaudRateUp();
    if (st != ILI9341_OK) { return st; }

#ifdef HAL_DMA2D_MODULE_ENABLED
    ILI9341_hdma2d = hdma2d;
    if (hdma2d != NULL)
    {
        /* Configuración única: el reloj (vía MspInit), el formato de salida y la
         * capa de entrada quedan fijos aquí; las funciones de dibujo solo cambian
         * modo y offset mediante ILI9341_DMA2D_SetMode(). */
        hdma2d->Instance          = DMA2D;
        hdma2d->Init.Mode         = DMA2D_M2M;          /* base; se ajusta por operación */
        hdma2d->Init.ColorMode    = DMA2D_OUTPUT_RGB565;
        hdma2d->Init.OutputOffset = 0U;
        if (HAL_DMA2D_Init(hdma2d) != HAL_OK) { return ILI9341_ERROR; }

        /* Capa de entrada (solo la usa el modo M2M de ILI9341_BlitImage); constante. */
        hdma2d->LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
        hdma2d->LayerCfg[1].InputOffset    = 0U;
        hdma2d->LayerCfg[1].AlphaMode      = DMA2D_NO_MODIF_ALPHA;
        hdma2d->LayerCfg[1].InputAlpha     = 0xFFU;
        if (HAL_DMA2D_ConfigLayer(hdma2d, 1U) != HAL_OK) { return ILI9341_ERROR; }
    }
#endif

    ILI9341_Initialized = 1U;

    return ILI9341_OK;
}

/**
 * @brief Convierte una componente de color RGB888 (8 bits por canal) a RGB565.
 *
 * @param[in] r Componente roja (0-255).
 * @param[in] g Componente verde (0-255).
 * @param[in] b Componente azul (0-255).
 * @return uint16_t Color empaquetado en formato RGB565.
 */
uint16_t ILI9341_Color565(uint8_t r, uint8_t g, uint8_t b)
{
    return RGB565(r, g, b);
}

/**
 * @brief Rellena toda la pantalla LCD con un color sólido.
 *
 * @param[in] color Color de relleno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_Fill(uint16_t color)
{
    const uint32_t Timeout = 5000U;
    uint32_t       tickstart;
    uint32_t       n;
    uint8_t        hi = (uint8_t)(color >> 8);
    uint8_t        lo = (uint8_t)(color & 0xFFU);

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    ILI9341_SetCursorPosition(0U, 0U, ILI9341_Opts.width - 1U, ILI9341_Opts.height - 1U);
    ILI9341_SendCommand(ILI9341_GRAM);

    ILI9341_WRX_SET;
    ILI9341_CS_RESET;

    if (ILI9341_hspi->State != HAL_SPI_STATE_READY)
    {
        ILI9341_WRX_RESET;
        ILI9341_CS_SET;
        return ILI9341_ERROR;
    }

    assert_param(IS_SPI_DIRECTION_2LINES_OR_1LINE(ILI9341_hspi->Init.Direction));
    __HAL_LOCK(ILI9341_hspi);
    ILI9341_hspi->State      = HAL_SPI_STATE_BUSY_TX;
    ILI9341_hspi->ErrorCode  = HAL_SPI_ERROR_NONE;
    ILI9341_hspi->TxISR      = 0;
    ILI9341_hspi->RxISR      = 0;
    ILI9341_hspi->RxXferSize  = 0U;
    ILI9341_hspi->RxXferCount = 0U;

    if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        SPI_RESET_CRC(ILI9341_hspi);
    }
    if (ILI9341_hspi->Init.Direction == SPI_DIRECTION_1LINE)
    {
        SPI_1LINE_TX(ILI9341_hspi);
    }
    if ((ILI9341_hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
    {
        __HAL_SPI_ENABLE(ILI9341_hspi);
    }

    for (n = 0U; n < (uint32_t)ILI9341_PIXEL; n++)
    {
        ILI9341_hspi->Instance->DR = hi;
        if (SPI_ILI9341_WaitTXE() != ILI9341_OK) { return ILI9341_TIMEOUT; }

        ILI9341_hspi->Instance->DR = lo;
        if (SPI_ILI9341_WaitTXE() != ILI9341_OK) { return ILI9341_TIMEOUT; }
    }

    if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        ILI9341_hspi->Instance->CR1 |= SPI_CR1_CRCNEXT;
    }

    tickstart = HAL_GetTick();
    while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_TXE) == RESET)
    {
        if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
        {
            __HAL_SPI_DISABLE_IT(ILI9341_hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
            __HAL_SPI_DISABLE(ILI9341_hspi);
            if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) { SPI_RESET_CRC(ILI9341_hspi); }
            ILI9341_hspi->State      = HAL_SPI_STATE_READY;
            ILI9341_hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
            __HAL_UNLOCK(ILI9341_hspi);
            ILI9341_WRX_RESET;
            ILI9341_CS_SET;
            return ILI9341_TIMEOUT;
        }
    }

    tickstart = HAL_GetTick();
    while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_BSY) != RESET)
    {
        if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
        {
            __HAL_SPI_DISABLE_IT(ILI9341_hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
            __HAL_SPI_DISABLE(ILI9341_hspi);
            if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) { SPI_RESET_CRC(ILI9341_hspi); }
            ILI9341_hspi->State      = HAL_SPI_STATE_READY;
            ILI9341_hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
            __HAL_UNLOCK(ILI9341_hspi);
            ILI9341_WRX_RESET;
            ILI9341_CS_SET;
            return ILI9341_TIMEOUT;
        }
    }

    if (ILI9341_hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
        __HAL_SPI_CLEAR_OVRFLAG(ILI9341_hspi);
    }

    ILI9341_hspi->State = HAL_SPI_STATE_READY;
    __HAL_UNLOCK(ILI9341_hspi);
    ILI9341_WRX_RESET;
    ILI9341_CS_SET;

    return ILI9341_OK;
}

/**
 * @brief Rota la pantalla y actualiza el ancho/alto internos.
 *
 * @note La geometría interna solo se actualiza si el comando SPI es exitoso.
 *
 * @param[in] orientation Orientación deseada (ILI9341_Orientation_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_Rotate(ILI9341_Orientation_t orientation)
{
    ILI9341_Status_t st;
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    st = ILI9341_SendCommand(ILI9341_MAC);
    if (orientation == ILI9341_Orientation_Portrait_1)
        st = st ? st : ILI9341_SendData(0x58U);
    else if (orientation == ILI9341_Orientation_Portrait_2)
        st = st ? st : ILI9341_SendData(0x88U);
    else if (orientation == ILI9341_Orientation_Landscape_1)
        st = st ? st : ILI9341_SendData(0x28U);
    else
        st = st ? st : ILI9341_SendData(0xE8U);

    if (st == ILI9341_OK)
    {
        if (orientation == ILI9341_Orientation_Portrait_1 ||
            orientation == ILI9341_Orientation_Portrait_2)
        {
            ILI9341_Opts.width       = ILI9341_WIDTH;
            ILI9341_Opts.height      = ILI9341_HEIGHT;
            ILI9341_Opts.orientation = ILI9341_Portrait;
        }
        else
        {
            ILI9341_Opts.width       = ILI9341_HEIGHT;
            ILI9341_Opts.height      = ILI9341_WIDTH;
            ILI9341_Opts.orientation = ILI9341_Landscape;
        }
    }
    return st;
}

/**
 * @brief Dibuja un píxel en la pantalla LCD.
 *
 * @param[in] x     Coordenada X del píxel.
 * @param[in] y     Coordenada Y del píxel.
 * @param[in] color Color del píxel en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    ILI9341_Status_t st;
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    st  = ILI9341_SetCursorPosition(x, y, x, y);
    st  = st ? st : ILI9341_SendCommand(ILI9341_GRAM);
    st  = st ? st : ILI9341_SendData((uint8_t)(color >> 8));
    st  = st ? st : ILI9341_SendData((uint8_t)(color & 0xFFU));
    return st;
}

/**
 * @brief Dibuja una línea en la pantalla LCD usando el algoritmo de Bresenham.
 *
 * @param[in] x0    Coordenada X de inicio.
 * @param[in] y0    Coordenada Y de inicio.
 * @param[in] x1    Coordenada X de fin.
 * @param[in] y1    Coordenada Y de fin.
 * @param[in] color Color de la línea en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    ILI9341_Status_t st;
    int16_t dx, dy, sx, sy, err, e2;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    if (x0 >= ILI9341_Opts.width)  { x0 = ILI9341_Opts.width  - 1U; }
    if (x1 >= ILI9341_Opts.width)  { x1 = ILI9341_Opts.width  - 1U; }
    if (y0 >= ILI9341_Opts.height) { y0 = ILI9341_Opts.height - 1U; }
    if (y1 >= ILI9341_Opts.height) { y1 = ILI9341_Opts.height - 1U; }

    /* Líneas horizontales y verticales: una sola ventana + burst directo al DR. */
    if (y0 == y1 || x0 == x1)
    {
        uint16_t xa = (x0 < x1) ? x0 : x1;
        uint16_t xb = (x0 < x1) ? x1 : x0;
        uint16_t ya = (y0 < y1) ? y0 : y1;
        uint16_t yb = (y0 < y1) ? y1 : y0;
        return ILI9341_DrawFilledRectangle(xa, ya, xb, yb, color);
    }

    /* Línea diagonal: Bresenham pixel a pixel (SetCursorPosition por pixel inevitable). */
    dx  = (x0 < x1) ? (int16_t)(x1 - x0) : (int16_t)(x0 - x1);
    dy  = (y0 < y1) ? (int16_t)(y1 - y0) : (int16_t)(y0 - y1);
    sx  = (x0 < x1) ?  1 : -1;
    sy  = (y0 < y1) ?  1 : -1;
    err = ((dx > dy) ? dx : -dy) / 2;

    while (1)
    {
        st = ILI9341_DrawPixel(x0, y0, color);
        if (st != ILI9341_OK) { return st; }
        if (x0 == x1 && y0 == y1) { break; }
        e2 = err;
        if (e2 > -dx) { err -= dy; x0 += (uint16_t)sx; }
        if (e2 <  dy) { err += dx; y0 += (uint16_t)sy; }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja una línea con grosor (ancho de trazo) en la pantalla LCD.
 *
 * @details Las líneas horizontales y verticales se rellenan con un único rectángulo
 *          (recortado a los límites de pantalla). Las líneas diagonales se aproximan
 *          trazando @p thickness líneas de Bresenham paralelas, desplazadas sobre la
 *          normal del segmento y centradas en la línea original; en ángulos muy
 *          pronunciados puede quedar un ligero aliasing entre trazos adyacentes.
 *
 * @param[in] x0        Coordenada X de inicio.
 * @param[in] y0        Coordenada Y de inicio.
 * @param[in] x1        Coordenada X de fin.
 * @param[in] y1        Coordenada Y de fin.
 * @param[in] thickness Grosor de la línea en píxeles (0 y 1 equivalen a ILI9341_DrawLine()).
 * @param[in] color     Color de la línea en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawThickLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t thickness, uint16_t color)
{
    int32_t maxX, maxY, dx, dy;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    if (thickness <= 1U)
    {
        return ILI9341_DrawLine(x0, y0, x1, y1, color);
    }

    maxX = (int32_t)ILI9341_Opts.width  - 1;
    maxY = (int32_t)ILI9341_Opts.height - 1;

    /* Línea horizontal: un único rectángulo relleno recortado a pantalla. */
    if (y0 == y1)
    {
        int32_t xL   = (x0 < x1) ? x0 : x1;
        int32_t xR   = (x0 < x1) ? x1 : x0;
        int32_t half = (int32_t)thickness / 2;
        int32_t yT   = (int32_t)y0 - half;
        int32_t yB   = yT + (int32_t)thickness - 1;

        if (xL < 0)    { xL = 0;    }
        if (xR > maxX) { xR = maxX; }
        if (yT < 0)    { yT = 0;    }
        if (yB > maxY) { yB = maxY; }
        if (xL > xR || yT > yB) { return ILI9341_OK; }

        return ILI9341_DrawFilledRectangle((uint16_t)xL, (uint16_t)yT, (uint16_t)xR, (uint16_t)yB, color);
    }

    /* Línea vertical: un único rectángulo relleno recortado a pantalla. */
    if (x0 == x1)
    {
        int32_t yT   = (y0 < y1) ? y0 : y1;
        int32_t yB   = (y0 < y1) ? y1 : y0;
        int32_t half = (int32_t)thickness / 2;
        int32_t xL   = (int32_t)x0 - half;
        int32_t xR   = xL + (int32_t)thickness - 1;

        if (yT < 0)    { yT = 0;    }
        if (yB > maxY) { yB = maxY; }
        if (xL < 0)    { xL = 0;    }
        if (xR > maxX) { xR = maxX; }
        if (xL > xR || yT > yB) { return ILI9341_OK; }

        return ILI9341_DrawFilledRectangle((uint16_t)xL, (uint16_t)yT, (uint16_t)xR, (uint16_t)yB, color);
    }

    /* Línea diagonal: se aproxima con trazos de Bresenham paralelos, desplazados
     * sobre la normal del segmento y centrados en la línea original. */
    dx = (int32_t)x1 - (int32_t)x0;
    dy = (int32_t)y1 - (int32_t)y0;
    {
        float   len = sqrtf((float)(dx * dx + dy * dy));
        float   ux  = -(float)dy / len; /* Normal unitaria (componente X) */
        float   uy  =  (float)dx / len; /* Normal unitaria (componente Y) */
        float   mid = ((float)thickness - 1.0f) / 2.0f;
        int32_t i;

        for (i = 0; i < (int32_t)thickness; i++)
        {
            float             off = (float)i - mid;
            int32_t           ox  = (int32_t)lroundf(ux * off);
            int32_t           oy  = (int32_t)lroundf(uy * off);
            int32_t           nx0 = (int32_t)x0 + ox;
            int32_t           ny0 = (int32_t)y0 + oy;
            int32_t           nx1 = (int32_t)x1 + ox;
            int32_t           ny1 = (int32_t)y1 + oy;
            ILI9341_Status_t  st;

            /* Descartar el trazo si queda completamente fuera de pantalla; recortar
             * cada coordenada por separado en ese caso distorsionaría la pendiente. */
            if ((nx0 < 0 && nx1 < 0) || (nx0 > maxX && nx1 > maxX) ||
                (ny0 < 0 && ny1 < 0) || (ny0 > maxY && ny1 > maxY))
            {
                continue;
            }

            if      (nx0 < 0) { nx0 = 0; } else if (nx0 > maxX) { nx0 = maxX; }
            if      (nx1 < 0) { nx1 = 0; } else if (nx1 > maxX) { nx1 = maxX; }
            if      (ny0 < 0) { ny0 = 0; } else if (ny0 > maxY) { ny0 = maxY; }
            if      (ny1 < 0) { ny1 = 0; } else if (ny1 > maxY) { ny1 = maxY; }

            st = ILI9341_DrawLine((uint16_t)nx0, (uint16_t)ny0, (uint16_t)nx1, (uint16_t)ny1, color);
            if (st != ILI9341_OK) { return st; }
        }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja una línea vertical de forma optimizada (sin Bresenham).
 *
 * @param[in] x     Coordenada X de la línea.
 * @param[in] y     Coordenada Y inicial.
 * @param[in] h     Alto de la línea (puede ser negativo, se normaliza).
 * @param[in] color Color de la línea.
 * @return ILI9341_Status_t
 */
ILI9341_Status_t ILI9341_DrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	int16_t y1;

	if (ILI9341_Initialized != 1U) { return ILI9341_NOT_INITIALIZED; }

	if (h == 0)
	{
		return ILI9341_OK;
	}

	if (h < 0) { y += h + 1; h = -h; }

	if (x < 0 || (uint16_t)x >= ILI9341_Opts.width) { return ILI9341_OK; }

	y1 = (int16_t)(y + h - 1);
	if (y  < 0)                          { y  = 0; }
	if ((uint16_t)y1 >= ILI9341_Opts.height) { y1 = (int16_t)(ILI9341_Opts.height - 1U); }
	if (y > y1) { return ILI9341_OK; }

	return ILI9341_DrawFilledRectangle((uint16_t)x, (uint16_t)y, (uint16_t)x, (uint16_t)y1, color);
}

/**
 * @brief Dibuja una línea horizontal de forma optimizada (sin Bresenham).
 *
 * @param[in] x     Coordenada X inicial.
 * @param[in] y     Coordenada Y de la línea.
 * @param[in] w     Ancho de la línea (puede ser negativo, se normaliza).
 * @param[in] color Color de la línea.
 * @return ILI9341_Status_t
 */
ILI9341_Status_t ILI9341_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	int16_t x1;

	if (ILI9341_Initialized != 1U) { return ILI9341_NOT_INITIALIZED; }

	if (w == 0)
	{
		return ILI9341_OK;
	}

	if (w < 0) { x += w + 1; w = -w; }

	if (y < 0 || (uint16_t)y >= ILI9341_Opts.height) { return ILI9341_OK; }

	x1 = (int16_t)(x + w - 1);
	if (x  < 0)                         { x  = 0; }
	if ((uint16_t)x1 >= ILI9341_Opts.width) { x1 = (int16_t)(ILI9341_Opts.width - 1U); }
	if (x > x1) { return ILI9341_OK; }

	return ILI9341_DrawFilledRectangle((uint16_t)x, (uint16_t)y, (uint16_t)x1, (uint16_t)y, color);
}

/**
 * @brief Dibuja el contorno de un rectángulo en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X superior izquierda.
 * @param[in] y0    Coordenada Y superior izquierda.
 * @param[in] x1    Coordenada X inferior derecha.
 * @param[in] y1    Coordenada Y inferior derecha.
 * @param[in] color Color de la línea en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    ILI9341_Status_t st;
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    st  = ILI9341_DrawLine(x0, y0, x1, y0, color); /* Superior  */
    st  = st ? st : ILI9341_DrawLine(x0, y0, x0, y1, color); /* Izquierda */
    st  = st ? st : ILI9341_DrawLine(x1, y0, x1, y1, color); /* Derecha   */
    st  = st ? st : ILI9341_DrawLine(x0, y1, x1, y1, color); /* Inferior  */
    return st;
}

/**
 * @brief Dibuja el contorno de un rectángulo con esquinas redondeadas en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X superior izquierda.
 * @param[in] y0    Coordenada Y superior izquierda.
 * @param[in] x1    Coordenada X inferior derecha.
 * @param[in] y1    Coordenada Y inferior derecha.
 * @param[in] r     Radio de las esquinas en píxeles (se recorta a min(ancho,alto)/2).
 * @param[in] color Color del contorno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawRoundRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t r, uint16_t color)
{
    ILI9341_Status_t st;
    int16_t f, ddF_x, ddF_y, x, y;
    int16_t cx0, cx1, cy0, cy1;
    uint16_t rmax;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    if (x0 > x1) { uint16_t _t = x0; x0 = x1; x1 = _t; }
    if (y0 > y1) { uint16_t _t = y0; y0 = y1; y1 = _t; }

    rmax = ((x1 - x0) < (y1 - y0)) ? (x1 - x0) / 2U : (y1 - y0) / 2U;
    if (r > rmax) { r = rmax; }
    if (r == 0U)  { return ILI9341_DrawRectangle(x0, y0, x1, y1, color); }

    cx0 = (int16_t)x0 + (int16_t)r;
    cx1 = (int16_t)x1 - (int16_t)r;
    cy0 = (int16_t)y0 + (int16_t)r;
    cy1 = (int16_t)y1 - (int16_t)r;

    /* Segmentos rectos */
    st  = ILI9341_DrawLine((uint16_t)cx0, y0, (uint16_t)cx1, y0, color);
    st  = st ? st : ILI9341_DrawLine((uint16_t)cx0, y1, (uint16_t)cx1, y1, color);
    st  = st ? st : ILI9341_DrawLine(x0, (uint16_t)cy0, x0, (uint16_t)cy1, color);
    st  = st ? st : ILI9341_DrawLine(x1, (uint16_t)cy0, x1, (uint16_t)cy1, color);
    if (st != ILI9341_OK) { return st; }

    /* Bresenham — estado inicial (x=0, y=r): píxeles cardinales de cada esquina */
    f     =  1 - (int16_t)r;
    ddF_x =  1;
    ddF_y = -2 * (int16_t)r;
    x     =  0;
    y     =  (int16_t)r;

    st  = DrawPixelClipped(cx0,     cy0 - y, color); /* TL arriba   */
    st  = st ? st : DrawPixelClipped(cx1,     cy0 - y, color); /* TR arriba   */
    st  = st ? st : DrawPixelClipped(cx0,     cy1 + y, color); /* BL abajo    */
    st  = st ? st : DrawPixelClipped(cx1,     cy1 + y, color); /* BR abajo    */
    st  = st ? st : DrawPixelClipped(cx0 - y, cy0,     color); /* TL izquierda */
    st  = st ? st : DrawPixelClipped(cx1 + y, cy0,     color); /* TR derecha  */
    st  = st ? st : DrawPixelClipped(cx0 - y, cy1,     color); /* BL izquierda */
    st  = st ? st : DrawPixelClipped(cx1 + y, cy1,     color); /* BR derecha  */
    if (st != ILI9341_OK) { return st; }

    while (x < y)
    {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++;
        ddF_x += 2;
        f += ddF_x;

        /* TL: cuadrante (-x,-y) y (-y,-x) */
        st  = DrawPixelClipped(cx0 - x, cy0 - y, color);
        st  = st ? st : DrawPixelClipped(cx0 - y, cy0 - x, color);
        /* TR: cuadrante (+x,-y) y (+y,-x) */
        st  = st ? st : DrawPixelClipped(cx1 + x, cy0 - y, color);
        st  = st ? st : DrawPixelClipped(cx1 + y, cy0 - x, color);
        /* BL: cuadrante (-x,+y) y (-y,+x) */
        st  = st ? st : DrawPixelClipped(cx0 - x, cy1 + y, color);
        st  = st ? st : DrawPixelClipped(cx0 - y, cy1 + x, color);
        /* BR: cuadrante (+x,+y) y (+y,+x) */
        st  = st ? st : DrawPixelClipped(cx1 + x, cy1 + y, color);
        st  = st ? st : DrawPixelClipped(cx1 + y, cy1 + x, color);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja un rectángulo relleno con esquinas redondeadas en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X superior izquierda.
 * @param[in] y0    Coordenada Y superior izquierda.
 * @param[in] x1    Coordenada X inferior derecha.
 * @param[in] y1    Coordenada Y inferior derecha.
 * @param[in] r     Radio de las esquinas en píxeles (se recorta a min(ancho,alto)/2).
 * @param[in] color Color de relleno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledRoundRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t r, uint16_t color)
{
    ILI9341_Status_t st;
    int16_t f, ddF_x, ddF_y, x, y;
    int16_t cx0, cx1, cy0, cy1;
    uint16_t rmax;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    if (x0 > x1) { uint16_t _t = x0; x0 = x1; x1 = _t; }
    if (y0 > y1) { uint16_t _t = y0; y0 = y1; y1 = _t; }

    rmax = ((x1 - x0) < (y1 - y0)) ? (x1 - x0) / 2U : (y1 - y0) / 2U;
    if (r > rmax) { r = rmax; }
    if (r == 0U)  { return ILI9341_DrawFilledRectangle(x0, y0, x1, y1, color); }

    cx0 = (int16_t)x0 + (int16_t)r;
    cx1 = (int16_t)x1 - (int16_t)r;
    cy0 = (int16_t)y0 + (int16_t)r;
    cy1 = (int16_t)y1 - (int16_t)r;

    /* Franja central de ancho completo */
    st = ILI9341_DrawFilledRectangle(x0, (uint16_t)cy0, x1, (uint16_t)cy1, color);
    if (st != ILI9341_OK) { return st; }

    f     =  1 - (int16_t)r;
    ddF_x =  1;
    ddF_y = -2 * (int16_t)r;
    x     =  0;
    y     =  (int16_t)r;

    /* Tramos cardinales (estado inicial Bresenham: x=0, y=r) */
    st  = DrawHSpanClipped(cx0, cx1, cy0 - y, color); /* tope superior */
    st  = st ? st : DrawHSpanClipped(cx0, cx1, cy1 + y, color); /* tope inferior */
    if (st != ILI9341_OK) { return st; }

    while (x < y)
    {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++;
        ddF_x += 2;
        f += ddF_x;

        /* Filas del arco de esquina (cy0-y y cy1+y): ancho ±x desde los centros */
        st  = DrawHSpanClipped(cx0 - x, cx1 + x, cy0 - y, color);
        st  = st ? st : DrawHSpanClipped(cx0 - x, cx1 + x, cy1 + y, color);
        /* Filas del arco lateral (cy0-x y cy1+x): ancho ±y desde los centros */
        st  = st ? st : DrawHSpanClipped(cx0 - y, cx1 + y, cy0 - x, color);
        st  = st ? st : DrawHSpanClipped(cx0 - y, cx1 + y, cy1 + x, color);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja un rectángulo relleno en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X superior izquierda.
 * @param[in] y0    Coordenada Y superior izquierda.
 * @param[in] x1    Coordenada X inferior derecha.
 * @param[in] y1    Coordenada Y inferior derecha.
 * @param[in] color Color de relleno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    const uint32_t Timeout = 5000U;
    uint32_t       tickstart;
    uint32_t       n;
    uint32_t       pixels = (uint32_t)(x1 - x0 + 1U) * (uint32_t)(y1 - y0 + 1U);
    uint8_t        hi = (uint8_t)(color >> 8);
    uint8_t        lo = (uint8_t)(color & 0xFFU);

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    ILI9341_SetCursorPosition(x0, y0, x1, y1);
    ILI9341_SendCommand(ILI9341_GRAM);

    ILI9341_WRX_SET;
    ILI9341_CS_RESET;

    if (ILI9341_hspi->State != HAL_SPI_STATE_READY)
    {
        ILI9341_WRX_RESET;
        ILI9341_CS_SET;
        return ILI9341_ERROR;
    }

    assert_param(IS_SPI_DIRECTION_2LINES_OR_1LINE(ILI9341_hspi->Init.Direction));
    __HAL_LOCK(ILI9341_hspi);
    ILI9341_hspi->State      = HAL_SPI_STATE_BUSY_TX;
    ILI9341_hspi->ErrorCode  = HAL_SPI_ERROR_NONE;
    ILI9341_hspi->TxISR      = 0;
    ILI9341_hspi->RxISR      = 0;
    ILI9341_hspi->RxXferSize  = 0U;
    ILI9341_hspi->RxXferCount = 0U;

    if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        SPI_RESET_CRC(ILI9341_hspi);
    }
    if (ILI9341_hspi->Init.Direction == SPI_DIRECTION_1LINE)
    {
        SPI_1LINE_TX(ILI9341_hspi);
    }
    if ((ILI9341_hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
    {
        __HAL_SPI_ENABLE(ILI9341_hspi);
    }

    for (n = 0U; n < pixels; n++)
    {
        ILI9341_hspi->Instance->DR = hi;
        if (SPI_ILI9341_WaitTXE() != ILI9341_OK) { return ILI9341_TIMEOUT; }

        ILI9341_hspi->Instance->DR = lo;
        if (SPI_ILI9341_WaitTXE() != ILI9341_OK) { return ILI9341_TIMEOUT; }
    }

    if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        ILI9341_hspi->Instance->CR1 |= SPI_CR1_CRCNEXT;
    }

    tickstart = HAL_GetTick();
    while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_TXE) == RESET)
    {
        if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
        {
            __HAL_SPI_DISABLE_IT(ILI9341_hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
            __HAL_SPI_DISABLE(ILI9341_hspi);
            if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) { SPI_RESET_CRC(ILI9341_hspi); }
            ILI9341_hspi->State      = HAL_SPI_STATE_READY;
            ILI9341_hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
            __HAL_UNLOCK(ILI9341_hspi);
            ILI9341_WRX_RESET;
            ILI9341_CS_SET;
            return ILI9341_TIMEOUT;
        }
    }

    tickstart = HAL_GetTick();
    while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_BSY) != RESET)
    {
        if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
        {
            __HAL_SPI_DISABLE_IT(ILI9341_hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
            __HAL_SPI_DISABLE(ILI9341_hspi);
            if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) { SPI_RESET_CRC(ILI9341_hspi); }
            ILI9341_hspi->State      = HAL_SPI_STATE_READY;
            ILI9341_hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
            __HAL_UNLOCK(ILI9341_hspi);
            ILI9341_WRX_RESET;
            ILI9341_CS_SET;
            return ILI9341_TIMEOUT;
        }
    }

    if (ILI9341_hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
        __HAL_SPI_CLEAR_OVRFLAG(ILI9341_hspi);
    }

    ILI9341_hspi->State = HAL_SPI_STATE_READY;
    __HAL_UNLOCK(ILI9341_hspi);
    ILI9341_WRX_RESET;
    ILI9341_CS_SET;

    return ILI9341_OK;
}

/**
 * @brief Dibuja el contorno de un círculo en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] r     Radio en píxeles.
 * @param[in] color Color de la línea en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    ILI9341_Status_t st;
    int16_t f     =  1 - r;
    int16_t ddF_x =  1;
    int16_t ddF_y = -2 * r;
    int16_t x     =  0;
    int16_t y     =  r;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    st  = DrawPixelClipped(x0,     y0 + r, color);
    st  = st ? st : DrawPixelClipped(x0,     y0 - r, color);
    st  = st ? st : DrawPixelClipped(x0 + r, y0,     color);
    st  = st ? st : DrawPixelClipped(x0 - r, y0,     color);
    if (st != ILI9341_OK) { return st; }

    while (x < y)
    {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++;
        ddF_x += 2;
        f += ddF_x;

        st  = DrawPixelClipped(x0 + x, y0 + y, color);
        st  = st ? st : DrawPixelClipped(x0 - x, y0 + y, color);
        st  = st ? st : DrawPixelClipped(x0 + x, y0 - y, color);
        st  = st ? st : DrawPixelClipped(x0 - x, y0 - y, color);
        st  = st ? st : DrawPixelClipped(x0 + y, y0 + x, color);
        st  = st ? st : DrawPixelClipped(x0 - y, y0 + x, color);
        st  = st ? st : DrawPixelClipped(x0 + y, y0 - x, color);
        st  = st ? st : DrawPixelClipped(x0 - y, y0 - x, color);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja un círculo relleno en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] r     Radio en píxeles.
 * @param[in] color Color de relleno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    ILI9341_Status_t st;
    int16_t f     =  1 - r;
    int16_t ddF_x =  1;
    int16_t ddF_y = -2 * r;
    int16_t x     =  0;
    int16_t y     =  r;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    st  = DrawPixelClipped(x0,     y0 + r, color);
    st  = st ? st : DrawPixelClipped(x0,     y0 - r, color);
    st  = st ? st : DrawHSpanClipped(x0 - r, x0 + r, y0, color);
    if (st != ILI9341_OK) { return st; }

    while (x < y)
    {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++;
        ddF_x += 2;
        f += ddF_x;

        st  = DrawHSpanClipped(x0 - x, x0 + x, y0 + y, color);
        st  = st ? st : DrawHSpanClipped(x0 - x, x0 + x, y0 - y, color);
        st  = st ? st : DrawHSpanClipped(x0 - y, x0 + y, y0 + x, color);
        st  = st ? st : DrawHSpanClipped(x0 - y, x0 + y, y0 - x, color);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja el contorno de un triángulo en la pantalla LCD.
 *
 * @param[in] x0    Coordenada X del primer vértice.
 * @param[in] y0    Coordenada Y del primer vértice.
 * @param[in] x1    Coordenada X del segundo vértice.
 * @param[in] y1    Coordenada Y del segundo vértice.
 * @param[in] x2    Coordenada X del tercer vértice.
 * @param[in] y2    Coordenada Y del tercer vértice.
 * @param[in] color Color de la línea en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawTriangle(uint16_t x0, uint16_t y0,
                                       uint16_t x1, uint16_t y1,
                                       uint16_t x2, uint16_t y2,
                                       uint16_t color)
{
    ILI9341_Status_t st;
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    st  = ILI9341_DrawLine(x0, y0, x1, y1, color);
    st  = st ? st : ILI9341_DrawLine(x1, y1, x2, y2, color);
    st  = st ? st : ILI9341_DrawLine(x2, y2, x0, y0, color);
    return st;
}

/**
 * @brief Dibuja un triángulo relleno en la pantalla LCD.
 *
 * @details Ordena los vértices por coordenada Y y rellena con tramos horizontales
 *          usando interpolación entera. Los tramos se recortan al borde de pantalla.
 *
 * @param[in] x0    Coordenada X del primer vértice.
 * @param[in] y0    Coordenada Y del primer vértice.
 * @param[in] x1    Coordenada X del segundo vértice.
 * @param[in] y1    Coordenada Y del segundo vértice.
 * @param[in] x2    Coordenada X del tercer vértice.
 * @param[in] y2    Coordenada Y del tercer vértice.
 * @param[in] color Color de relleno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledTriangle(uint16_t x0, uint16_t y0,
                                             uint16_t x1, uint16_t y1,
                                             uint16_t x2, uint16_t y2,
                                             uint16_t color)
{
    ILI9341_Status_t st;
    int32_t ax, ay, bx, by, cx, cy, tmp;
    int32_t y, xa, xb;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    ax = (int32_t)x0; ay = (int32_t)y0;
    bx = (int32_t)x1; by = (int32_t)y1;
    cx = (int32_t)x2; cy = (int32_t)y2;

    /* Ordenar vértices por Y: ay <= by <= cy */
    if (ay > by) { tmp = ax; ax = bx; bx = tmp; tmp = ay; ay = by; by = tmp; }
    if (by > cy) { tmp = bx; bx = cx; cx = tmp; tmp = by; by = cy; cy = tmp; }
    if (ay > by) { tmp = ax; ax = bx; bx = tmp; tmp = ay; ay = by; by = tmp; }

    /* Triángulo degenerado: todos los vértices en la misma fila */
    if (ay == cy)
    {
        return DrawHSpanClipped((int16_t)ax, (int16_t)cx, (int16_t)ay, color);
    }

    for (y = ay; y <= cy; y++)
    {
        /* Interpolación sobre el lado largo (a→c, siempre presente) */
        xa = ax + (cx - ax) * (y - ay) / (cy - ay);
        /* Interpolación sobre el lado corto activo según la mitad del triángulo */
        if (y <= by)
            xb = (ay == by) ? bx : ax + (bx - ax) * (y - ay) / (by - ay);
        else
            xb = bx + (cx - bx) * (y - by) / (cy - by);
        st = DrawHSpanClipped((int16_t)xa, (int16_t)xb, (int16_t)y, color);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja el contorno de una elipse en la pantalla LCD.
 *
 * @details Implementación entera del algoritmo de punto medio para elipses
 *          (variante de Zingl), recortando cada píxel a los límites de la
 *          pantalla mediante DrawPixelClipped(). Los casos degenerados
 *          (rx == 0 o ry == 0) se delegan en ILI9341_DrawFastVLine() /
 *          ILI9341_DrawFastHLine().
 *
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] rx    Radio horizontal en píxeles.
 * @param[in] ry    Radio vertical en píxeles.
 * @param[in] color Color del contorno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p rx o @p ry son negativos.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color)
{
    ILI9341_Status_t st;
    int32_t xa, xb, ya, yb;
    int32_t a, b, b1;
    int32_t dx, dy, err, e2;

    if (ILI9341_Initialized != 1U) { return ILI9341_NOT_INITIALIZED; }
    if (rx < 0 || ry < 0)          { return ILI9341_INVALID_PARAM;   }

    if (rx == 0)
    {
        return ILI9341_DrawFastVLine(x0, (int16_t)(y0 - ry), (int16_t)(2 * ry + 1), color);
    }
    if (ry == 0)
    {
        return ILI9341_DrawFastHLine((int16_t)(x0 - rx), y0, (int16_t)(2 * rx + 1), color);
    }

    xa = (int32_t)x0 - rx;
    xb = (int32_t)x0 + rx;
    ya = (int32_t)y0 - ry;
    yb = (int32_t)y0 + ry;

    a  = xb - xa;
    b  = yb - ya;
    b1 = b & 1;

    dx  = 4 * (1 - a) * b * b;
    dy  = 4 * (b1 + 1) * a * a;
    err = dx + dy + b1 * a * a;

    ya += (b + 1) / 2;
    yb  = ya - b1;
    a  *= 8 * a;
    b1  = 8 * b * b;

    do
    {
        st  = DrawPixelClipped((int16_t)xb, (int16_t)ya, color);
        st  = st ? st : DrawPixelClipped((int16_t)xa, (int16_t)ya, color);
        st  = st ? st : DrawPixelClipped((int16_t)xa, (int16_t)yb, color);
        st  = st ? st : DrawPixelClipped((int16_t)xb, (int16_t)yb, color);
        if (st != ILI9341_OK) { return st; }

        e2 = 2 * err;
        if (e2 <= dy) { ya++; yb--; dy += a; err += dy; }
        if (e2 >= dx || 2 * err > dy) { xa++; xb--; dx += b1; err += dx; }
    } while (xa <= xb);

    /* Remate de puntas para elipses muy achatadas (a == 0 en algún eje). */
    while (ya - yb < b)
    {
        st  = DrawPixelClipped((int16_t)(xa - 1), (int16_t)ya, color);
        st  = st ? st : DrawPixelClipped((int16_t)(xb + 1), (int16_t)ya, color);
        ya++;
        st  = st ? st : DrawPixelClipped((int16_t)(xa - 1), (int16_t)yb, color);
        st  = st ? st : DrawPixelClipped((int16_t)(xb + 1), (int16_t)yb, color);
        yb--;
        if (st != ILI9341_OK) { return st; }
    }

    return ILI9341_OK;
}

/**
 * @brief Dibuja una elipse rellena en la pantalla LCD.
 * 
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] rx    Radio horizontal en píxeles.
 * @param[in] ry    Radio vertical en píxeles.
 * @param[in] color Color del contorno en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p rx o @p ry son negativos.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color)
{
    ILI9341_Status_t st;
    int32_t xa, xb, ya, yb;
    int32_t a, b, b1;
    int32_t dx, dy, err, e2;

    if (ILI9341_Initialized != 1U) { return ILI9341_NOT_INITIALIZED; }
    if (rx < 0 || ry < 0)          { return ILI9341_INVALID_PARAM;   }

    if (rx == 0)
    {
        return ILI9341_DrawFastVLine(x0, (int16_t)(y0 - ry), (int16_t)(2 * ry + 1), color);
    }
    if (ry == 0)
    {
        return ILI9341_DrawFastHLine((int16_t)(x0 - rx), y0, (int16_t)(2 * rx + 1), color);
    }

    /* Mismo trazado de contorno que ILI9341_DrawEllipse (Zingl), pero rellenando
     * cada fila visitada con un tramo horizontal en vez de graficar 4 píxeles. */
    xa = (int32_t)x0 - rx;
    xb = (int32_t)x0 + rx;
    ya = (int32_t)y0 - ry;
    yb = (int32_t)y0 + ry;

    a  = xb - xa;
    b  = yb - ya;
    b1 = b & 1;

    dx  = 4 * (1 - a) * b * b;
    dy  = 4 * (b1 + 1) * a * a;
    err = dx + dy + b1 * a * a;

    ya += (b + 1) / 2;
    yb  = ya - b1;
    a  *= 8 * a;
    b1  = 8 * b * b;

    do
    {
        st  = DrawHSpanClipped((int16_t)xa, (int16_t)xb, (int16_t)ya, color);
        st  = st ? st : DrawHSpanClipped((int16_t)xa, (int16_t)xb, (int16_t)yb, color);
        if (st != ILI9341_OK) { return st; }

        e2 = 2 * err;
        if (e2 <= dy) { ya++; yb--; dy += a; err += dy; }
        if (e2 >= dx || 2 * err > dy) { xa++; xb--; dx += b1; err += dx; }
    } while (xa <= xb);

    /* Remate de puntas para elipses muy achatadas (a == 0 en algún eje). */
    while (ya - yb < b)
    {
        ya++;
        yb--;
        st  = DrawHSpanClipped((int16_t)(xa - 1), (int16_t)(xb + 1), (int16_t)ya, color);
        st  = st ? st : DrawHSpanClipped((int16_t)(xa - 1), (int16_t)(xb + 1), (int16_t)yb, color);
        if (st != ILI9341_OK) { return st; }
    }

    return ILI9341_OK;
}

/**
 * @brief Dibuja el contorno de un arco (sector de anillo) entre dos ángulos.
 *
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] r1    Radio exterior del arco.
 * @param[in] r2    Radio interior del arco.
 * @param[in] start Ángulo inicial en grados (0° = derecha, sentido horario).
 * @param[in] end   Ángulo final en grados.
 * @param[in] color Color del contorno.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p r1 o @p r2 son negativos.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawArc(int16_t x0, int16_t y0, int16_t r1, int16_t r2, float start, float end, uint16_t color)
{
    ILI9341_Status_t st;
    bool equal;
    int16_t tmp;

    if (ILI9341_Initialized != 1U) { return ILI9341_NOT_INITIALIZED; }
    if (r1 < 0 || r2 < 0)          { return ILI9341_INVALID_PARAM;   }

    if (r1 < r2) { tmp = r1; r1 = r2; r2 = tmp; }
    if (r1 < 1)  { r1 = 1; }
    if (r2 < 1)  { r2 = 1; }

    equal = fabsf(start - end) < FLT_EPSILON;
    start = fmodf(start, 360.0f);
    end   = fmodf(end, 360.0f);
    if (start < 0) { start += 360.0f; }
    if (end < 0)   { end   += 360.0f; }

    /* Bordes rectos del arco (en start y en end) */
    st  = ILI9341_FillArcHelper(x0, y0, r1, r2, start, start, color, NULL);
    st  = st ? st : ILI9341_FillArcHelper(x0, y0, r1, r2, end, end, color, NULL);
    if (st != ILI9341_OK) { return st; }

    if (!equal && (fabsf(start - end) <= 0.0001f))
    {
        start = 0.0f;
        end   = 360.0f;
    }

    /* Bordes curvos del arco (radio exterior e interior) */
    st  = ILI9341_FillArcHelper(x0, y0, r1, r1, start, end, color, NULL);
    st  = st ? st : ILI9341_FillArcHelper(x0, y0, r2, r2, start, end, color, NULL);

    return st;
}

/**
 * @brief Dibuja un arco relleno (sector de anillo) entre dos ángulos.
 *
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] r1    Radio exterior del arco.
 * @param[in] r2    Radio interior del arco.
 * @param[in] start Ángulo inicial en grados (0° = derecha, sentido horario).
 * @param[in] end   Ángulo final en grados.
 * @param[in] color Color del contorno.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p r1 o @p r2 son negativos.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledArc(int16_t x0, int16_t y0, int16_t r1, int16_t r2, float start, float end, uint16_t color)
{
    bool equal;
    int16_t tmp;

    if (ILI9341_Initialized != 1U) { return ILI9341_NOT_INITIALIZED; }
    if (r1 < 0 || r2 < 0)          { return ILI9341_INVALID_PARAM;   }

    if (r1 < r2) { tmp = r1; r1 = r2; r2 = tmp; }
    if (r1 < 1)  { r1 = 1; }
    if (r2 < 1)  { r2 = 1; }

    equal = fabsf(start - end) < FLT_EPSILON;
    start = fmodf(start, 360.0f);
    end   = fmodf(end, 360.0f);
    if (start < 0) { start += 360.0f; }
    if (end < 0)   { end   += 360.0f; }

    if (!equal && (fabsf(start - end) <= 0.0001f))
    {
        start = 0.0f;
        end   = 360.0f;
    }

    return ILI9341_FillArcHelper(x0, y0, r1, r2, start, end, color, NULL);
}

/**
 * @brief Renderiza un carácter en la pantalla LCD.
 *
 * @param[in] x          Coordenada X superior izquierda de la celda del carácter.
 * @param[in] y          Coordenada Y superior izquierda de la celda del carácter.
 * @param[in] c          Carácter a mostrar.
 * @param[in] font       Puntero a la definición de la fuente.
 * @param[in] foreground Color de primer plano en formato RGB565.
 * @param[in] background Color de fondo en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si el periférico SPI estaba ocupado al iniciar.
 *         - ILI9341_TIMEOUT         si el bus SPI se bloqueó durante la transmisión.
 */
ILI9341_Status_t ILI9341_Putc(uint16_t x, uint16_t y, char c, LCD_FontDef_t* font, uint16_t foreground, uint16_t background)
{
    const uint32_t Timeout = 5000U;
    uint32_t       tickstart;
    uint32_t       i, b, j;
    uint16_t       color;
    uint8_t        hi, lo;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    if (font == NULL)         { return ILI9341_INVALID_PARAM;   }
    if ((uint8_t)c < 32U || (uint8_t)c > 126U) { return ILI9341_OK; }

    ILI9341_x = x;
    ILI9341_y = y;
    if ((ILI9341_x + font->FontWidth) > ILI9341_Opts.width)
    {
        ILI9341_y += font->FontHeight;
        ILI9341_x  = 0U;
    }

    ILI9341_SetCursorPosition(ILI9341_x, ILI9341_y,
                              ILI9341_x + font->FontWidth  - 1U,
                              ILI9341_y + font->FontHeight - 1U);
    ILI9341_SendCommand(ILI9341_GRAM);

    ILI9341_WRX_SET;
    ILI9341_CS_RESET;

    if (ILI9341_hspi->State != HAL_SPI_STATE_READY)
    {
        ILI9341_WRX_RESET;
        ILI9341_CS_SET;
        return ILI9341_ERROR;
    }

    assert_param(IS_SPI_DIRECTION_2LINES_OR_1LINE(ILI9341_hspi->Init.Direction));
    __HAL_LOCK(ILI9341_hspi);
    ILI9341_hspi->State      = HAL_SPI_STATE_BUSY_TX;
    ILI9341_hspi->ErrorCode  = HAL_SPI_ERROR_NONE;
    ILI9341_hspi->TxISR      = 0;
    ILI9341_hspi->RxISR      = 0;
    ILI9341_hspi->RxXferSize  = 0U;
    ILI9341_hspi->RxXferCount = 0U;

    if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        SPI_RESET_CRC(ILI9341_hspi);
    }
    if (ILI9341_hspi->Init.Direction == SPI_DIRECTION_1LINE)
    {
        SPI_1LINE_TX(ILI9341_hspi);
    }
    if ((ILI9341_hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
    {
        __HAL_SPI_ENABLE(ILI9341_hspi);
    }

    for (i = 0U; i < font->FontHeight; i++)
    {
        b = font->data[(c - 32) * font->FontHeight + i];
        for (j = 0U; j < font->FontWidth; j++)
        {
            color = ((b << j) & 0x8000U) ? foreground : background;
            hi    = (uint8_t)(color >> 8);
            lo    = (uint8_t)(color & 0xFFU);

            ILI9341_hspi->Instance->DR = hi;
            if (SPI_ILI9341_WaitTXE() != ILI9341_OK) { return ILI9341_TIMEOUT; }

            ILI9341_hspi->Instance->DR = lo;
            if (SPI_ILI9341_WaitTXE() != ILI9341_OK) { return ILI9341_TIMEOUT; }
        }
    }

    if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        ILI9341_hspi->Instance->CR1 |= SPI_CR1_CRCNEXT;
    }

    tickstart = HAL_GetTick();
    while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_TXE) == RESET)
    {
        if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
        {
            __HAL_SPI_DISABLE_IT(ILI9341_hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
            __HAL_SPI_DISABLE(ILI9341_hspi);
            if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) { SPI_RESET_CRC(ILI9341_hspi); }
            ILI9341_hspi->State      = HAL_SPI_STATE_READY;
            ILI9341_hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
            __HAL_UNLOCK(ILI9341_hspi);
            ILI9341_WRX_RESET;
            ILI9341_CS_SET;
            return ILI9341_TIMEOUT;
        }
    }

    tickstart = HAL_GetTick();
    while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_BSY) != RESET)
    {
        if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
        {
            __HAL_SPI_DISABLE_IT(ILI9341_hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
            __HAL_SPI_DISABLE(ILI9341_hspi);
            if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) { SPI_RESET_CRC(ILI9341_hspi); }
            ILI9341_hspi->State      = HAL_SPI_STATE_READY;
            ILI9341_hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
            __HAL_UNLOCK(ILI9341_hspi);
            ILI9341_WRX_RESET;
            ILI9341_CS_SET;
            return ILI9341_TIMEOUT;
        }
    }

    if (ILI9341_hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
        __HAL_SPI_CLEAR_OVRFLAG(ILI9341_hspi);
    }

    ILI9341_hspi->State = HAL_SPI_STATE_READY;
    __HAL_UNLOCK(ILI9341_hspi);
    ILI9341_WRX_RESET;
    ILI9341_CS_SET;

    ILI9341_x += font->FontWidth;
    return ILI9341_OK;
}

/**
 * @brief Renderiza una cadena terminada en nulo en la pantalla LCD.
 *
 * @param[in] x          Coordenada X superior izquierda del primer carácter.
 * @param[in] y          Coordenada Y superior izquierda del primer carácter.
 * @param[in] str        Puntero a la cadena terminada en nulo.
 * @param[in] font       Puntero a la definición de la fuente.
 * @param[in] foreground Color de primer plano en formato RGB565.
 * @param[in] background Color de fondo en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_Puts(uint16_t x, uint16_t y, char* str, LCD_FontDef_t* font, uint16_t foreground, uint16_t background)
{
    ILI9341_Status_t st;
    uint16_t startX = x;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    if (str == NULL || font == NULL) { return ILI9341_INVALID_PARAM; }

    ILI9341_x = x;
    ILI9341_y = y;

    while (*str)
    {
        if (*str == '\n')
        {
            ILI9341_y += font->FontHeight + 1U;
            if (*(str + 1) == '\r')
            {
                ILI9341_x = 0U;
                str++;
            }
            else
            {
                ILI9341_x = startX;
            }
            str++;
            continue;
        }
        else if (*str == '\r')
        {
            str++;
            continue;
        }
        st = ILI9341_Putc(ILI9341_x, ILI9341_y, *str++, font, foreground, background);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
}

/**
 * @brief Renderiza una cadena con formato (estilo printf) en la pantalla LCD.
 *
 * @details Formatea los argumentos variádicos con vsnprintf() en un buffer
 *          interno en pila de ILI9341_PRINTF_BUF_SIZE bytes y delega el dibujo
 *          en ILI9341_Puts(). El uso de vsnprintf con el tamaño del buffer
 *          garantiza que la cadena siempre quede terminada en nulo y sin
 *          desbordamiento aunque el resultado se trunque.
 *
 * @param[in] x          Coordenada X superior izquierda del primer carácter.
 * @param[in] y          Coordenada Y superior izquierda del primer carácter.
 * @param[in] font       Puntero a la definición de la fuente.
 * @param[in] foreground Color de primer plano en formato RGB565.
 * @param[in] background Color de fondo en formato RGB565.
 * @param[in] fmt        Cadena de formato estilo printf (terminada en nulo).
 * @param[in] ...        Argumentos variádicos correspondientes a fmt.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si fmt o font son NULL, o si vsnprintf falla.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_Printf(uint16_t x, uint16_t y, LCD_FontDef_t* font, uint16_t foreground, uint16_t background, const char* fmt, ...)
{
    char buf[ILI9341_PRINTF_BUF_SIZE];
    va_list args;
    int len;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    if (fmt == NULL || font == NULL) { return ILI9341_INVALID_PARAM; }

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0) { return ILI9341_INVALID_PARAM; }

    return ILI9341_Puts(x, y, buf, font, foreground, background);
}

/**
 * @brief Calcula el bounding-box en píxeles de una cadena para una fuente dada.
 *
 * @param[in]  str    Puntero a la cadena terminada en nulo.
 * @param[in]  font   Puntero a la definición de la fuente.
 * @param[out] width  Ancho total en píxeles.
 * @param[out] height Alto total en píxeles.
 */
void ILI9341_GetStringSize(char* str, LCD_FontDef_t* font, uint16_t* width, uint16_t* height)
{
    uint16_t w = 0U;
    if (str == NULL || font == NULL || width == NULL || height == NULL) { return; }
    *height = font->FontHeight;
    while (*str++) { w += font->FontWidth; }
    *width = w;
}

/**
 * @brief Deriva la coordenada X de arranque de una cadena dentro de una región horizontal.
 *
 * @param[in] x0        Borde izquierdo de la región.
 * @param[in] x1        Borde derecho de la región.
 * @param[in] strWidth  Ancho en píxeles de la cadena (de ILI9341_GetStringSize()).
 * @param[in] align     Alineación deseada.
 * @return Coordenada X donde debe comenzar el texto.
 */
static uint16_t ILI9341_AlignedX(uint16_t x0, uint16_t x1, uint16_t strWidth, ILI9341_TextAlign_t align)
{
    uint16_t regionWidth = (uint16_t)(x1 - x0 + 1U);

    if (strWidth >= regionWidth) { return x0; }

    switch (align)
    {
        case ILI9341_ALIGN_CENTER: return (uint16_t)(x0 + (regionWidth - strWidth) / 2U);
        case ILI9341_ALIGN_RIGHT:  return (uint16_t)(x1 - strWidth + 1U);
        case ILI9341_ALIGN_LEFT:
        default:                   return x0;
    }
}

/**
 * @brief Renderiza una cadena terminada en nulo alineada dentro de una región horizontal.
 *
 * @param[in] x0         Borde izquierdo de la región de alineación.
 * @param[in] x1         Borde derecho de la región de alineación (x1 >= x0).
 * @param[in] y          Coordenada Y superior izquierda del texto.
 * @param[in] align      Alineación deseada (izquierda, centro, derecha).
 * @param[in] str        Puntero a la cadena terminada en nulo.
 * @param[in] font       Puntero a la definición de la fuente.
 * @param[in] foreground Color de primer plano en formato RGB565.
 * @param[in] background Color de fondo en formato RGB565.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si str o font son NULL, o si x1 < x0.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_PutsAligned(uint16_t x0, uint16_t x1, uint16_t y, ILI9341_TextAlign_t align, char* str, LCD_FontDef_t* font, uint16_t foreground, uint16_t background)
{
    uint16_t strWidth, strHeight, x;

    if (!ILI9341_Initialized)        { return ILI9341_NOT_INITIALIZED; }
    if (str == NULL || font == NULL) { return ILI9341_INVALID_PARAM;   }
    if (x1 < x0)                     { return ILI9341_INVALID_PARAM;   }

    ILI9341_GetStringSize(str, font, &strWidth, &strHeight);
    x = ILI9341_AlignedX(x0, x1, strWidth, align);

    return ILI9341_Puts(x, y, str, font, foreground, background);
}

/**
 * @brief Renderiza una cadena con formato (estilo printf) alineada dentro de una región horizontal.
 *
 * @param[in] x0         Borde izquierdo de la región de alineación.
 * @param[in] x1         Borde derecho de la región de alineación (x1 >= x0).
 * @param[in] y          Coordenada Y superior izquierda del texto.
 * @param[in] align      Alineación deseada (izquierda, centro, derecha).
 * @param[in] font       Puntero a la definición de la fuente.
 * @param[in] foreground Color de primer plano en formato RGB565.
 * @param[in] background Color de fondo en formato RGB565.
 * @param[in] fmt        Cadena de formato estilo printf (terminada en nulo).
 * @param[in] ...        Argumentos variádicos correspondientes a fmt.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si fmt o font son NULL, si x1 < x0, o si vsnprintf falla.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_PrintfAligned(uint16_t x0, uint16_t x1, uint16_t y, ILI9341_TextAlign_t align, LCD_FontDef_t* font, uint16_t foreground, uint16_t background, const char* fmt, ...)
{
    char buf[ILI9341_PRINTF_BUF_SIZE];
    va_list args;
    int len;

    if (!ILI9341_Initialized)        { return ILI9341_NOT_INITIALIZED; }
    if (fmt == NULL || font == NULL) { return ILI9341_INVALID_PARAM;   }
    if (x1 < x0)                     { return ILI9341_INVALID_PARAM;   }

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0) { return ILI9341_INVALID_PARAM; }

    return ILI9341_PutsAligned(x0, x1, y, align, buf, font, foreground, background);
}

/**
 * @brief Transfiere un frame buffer RGB565 de pantalla completa a la LCD mediante SPI optimizado.
 *
 * @param[in] image Arreglo de IMG_TOTAL_BUF32 palabras uint32_t (dos píxeles RGB565 por palabra).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si todos los píxeles fueron enviados.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_TIMEOUT         si el bus SPI se bloqueó.
 *         - ILI9341_ERROR           si el periférico SPI estaba ocupado.
 */
ILI9341_Status_t ILI9341_DisplayImage(uint32_t image[IMG_TOTAL_BUF32])
{
    /* El frame buffer almacena píxeles como uint16_t en little-endian ([lo, hi] en bytes).
     * Con SPI en modo 16 bits el periférico lee cada halfword y lo envía MSB-first,
     * produciendo automáticamente el orden big-endian ([hi, lo]) que espera el ILI9341.
     * El transfer se divide en dos tramos de 38 400 píxeles para respetar el límite de
     * 65 535 items del registro NDTR del DMA. */
    ILI9341_Status_t  st;
    uint16_t* const   px   = (uint16_t*)image;
    const uint16_t    half = ILI9341_PIXEL / 2U;   /* 38 400 px — cabe en uint16_t */

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    /* Ventana + comando GRAM en modo 8 bits (normal) */
    ILI9341_SetCursorPosition(0U, 0U, ILI9341_Opts.width - 1U, ILI9341_Opts.height - 1U);
    ILI9341_SendCommand(ILI9341_GRAM);

    ILI9341_WRX_SET;
    ILI9341_CS_RESET;

    /* Cambiar SPI a 16 bits antes del DMA */
    st = ILI9341_SPI_SetDataSize(SPI_DATASIZE_16BIT);
    if (st != ILI9341_OK)
    {
        ILI9341_CS_SET; ILI9341_WRX_RESET;
        return st;
    }

    /* — Primer tramo: píxeles [0 … 38 399] — */
    ILI9341_spi_dma_done = 0U;
    if (HAL_SPI_Transmit_DMA(ILI9341_hspi, (uint8_t*)px, half) != HAL_OK)
    {
        ILI9341_SPI_SetDataSize(SPI_DATASIZE_8BIT);
        ILI9341_CS_SET; ILI9341_WRX_RESET;
        return ILI9341_ERROR;
    }
    st = ILI9341_SPI_WaitDMAdone(5000U);
    if (st != ILI9341_OK)
    {
        ILI9341_SPI_SetDataSize(SPI_DATASIZE_8BIT);
        ILI9341_CS_SET; ILI9341_WRX_RESET;
        return st;
    }

    /* — Segundo tramo: píxeles [38 400 … 76 799] — */
    ILI9341_spi_dma_done = 0U;
    if (HAL_SPI_Transmit_DMA(ILI9341_hspi, (uint8_t*)(px + half), half) != HAL_OK)
    {
        ILI9341_SPI_SetDataSize(SPI_DATASIZE_8BIT);
        ILI9341_CS_SET; ILI9341_WRX_RESET;
        return ILI9341_ERROR;
    }
    st = ILI9341_SPI_WaitDMAdone(5000U);
    if (st != ILI9341_OK)
    {
        ILI9341_SPI_SetDataSize(SPI_DATASIZE_8BIT);
        ILI9341_CS_SET; ILI9341_WRX_RESET;
        return st;
    }

    /* Restaurar SPI a 8 bits para el resto de las operaciones del driver */
    ILI9341_SPI_SetDataSize(SPI_DATASIZE_8BIT);
    ILI9341_CS_SET;
    ILI9341_WRX_RESET;
    return ILI9341_OK;
}

/**
 * @brief Rellena un frame buffer fuera de pantalla completo con un color sólido.
 *
 * @details Delega en ILI9341_DrawFilledRectangle_ImageBuffer() sobre el área
 *          completa del panel: usa DMA2D en modo R2M cuando el handle fue
 *          inyectado en ILI9341_Init() y el camino CPU optimizado en caso
 *          contrario.
 *
 * @param[in]     color  Color de relleno en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
ILI9341_Status_t ILI9341_Fill_ImageBuffer(uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    return ILI9341_DrawFilledRectangle_ImageBuffer(0U, 0U,
                                                   ILI9341_WIDTH  - 1U,
                                                   ILI9341_HEIGHT - 1U,
                                                   color, image);
}

/**
 * @brief Escribe un píxel en un frame buffer fuera de pantalla.
 *
 * @param[in]     x      Coordenada X del píxel.
 * @param[in]     y      Coordenada Y del píxel.
 * @param[in]     color  Color del píxel en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
ILI9341_Status_t ILI9341_DrawPixel_ImageBuffer(uint16_t x, uint16_t y, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    if (image == NULL)                              { return ILI9341_INVALID_PARAM; }
    if (x >= ILI9341_WIDTH || y >= ILI9341_HEIGHT)  { return ILI9341_OK; }

    ((uint16_t*)image)[(uint32_t)y * ILI9341_WIDTH + x] = color;
    return ILI9341_OK;
}

/**
 * @brief Renderiza un carácter en un frame buffer fuera de pantalla.
 *
 * @param[in]     x          Coordenada X superior izquierda de la celda del carácter.
 * @param[in]     y          Coordenada Y superior izquierda de la celda del carácter.
 * @param[in]     c          Carácter a mostrar.
 * @param[in]     font       Puntero a la definición de la fuente.
 * @param[in]     foreground Color de primer plano en formato RGB565.
 * @param[in,out] image      Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
ILI9341_Status_t ILI9341_Putc_ImageBuffer(uint16_t x, uint16_t y, char c, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32])
{
    uint32_t i, b, j;

    if (font == NULL || image == NULL)          { return ILI9341_INVALID_PARAM; }
    if ((uint8_t)c < 32U || (uint8_t)c > 126U) { return ILI9341_OK; }

    ILI9341_x = x;
    ILI9341_y = y;
    if ((ILI9341_x + font->FontWidth) > ILI9341_WIDTH)
    {
        ILI9341_y += font->FontHeight;
        ILI9341_x  = 0U;
    }

    for (i = 0U; i < font->FontHeight; i++)
    {
        b = font->data[(c - 32U) * font->FontHeight + i];
        uint16_t* row = (uint16_t*)image + (uint32_t)(ILI9341_y + i) * ILI9341_WIDTH + ILI9341_x;
        for (j = 0U; j < font->FontWidth; j++)
        {
            if ((b << j) & 0x8000U) { row[j] = foreground; }
        }
    }
    ILI9341_x += font->FontWidth;
    return ILI9341_OK;
}

/**
 * @brief Renderiza una cadena terminada en nulo en un frame buffer fuera de pantalla.
 *
 * @param[in]     x          Coordenada X superior izquierda del primer carácter.
 * @param[in]     y          Coordenada Y superior izquierda del primer carácter.
 * @param[in]     str        Puntero a la cadena terminada en nulo.
 * @param[in]     font       Puntero a la definición de la fuente.
 * @param[in]     foreground Color de primer plano en formato RGB565.
 * @param[in,out] image      Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
ILI9341_Status_t ILI9341_Puts_ImageBuffer(uint16_t x, uint16_t y, char* str, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_Status_t st;
    uint16_t startX = x;

    if (str == NULL || font == NULL || image == NULL) { return ILI9341_INVALID_PARAM; }
    ILI9341_x = x;
    ILI9341_y = y;

    while (*str)
    {
        if (*str == '\n')
        {
            ILI9341_y += font->FontHeight + 1U;
            if (*(str + 1) == '\r')
            {
                ILI9341_x = 0U;
                str++;
            }
            else
            {
                ILI9341_x = startX;
            }
            str++;
            continue;
        }
        else if (*str == '\r')
        {
            str++;
            continue;
        }
        st = ILI9341_Putc_ImageBuffer(ILI9341_x, ILI9341_y, *str++, font, foreground, image);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
}

/**
 * @brief Renderiza una cadena con formato (estilo printf) en un frame buffer fuera de pantalla.
 *
 * @details Formatea los argumentos variádicos con vsnprintf() en un buffer
 *          interno en pila de ILI9341_PRINTF_BUF_SIZE bytes y delega el dibujo
 *          en ILI9341_Puts_ImageBuffer(). El uso de vsnprintf con el tamaño del
 *          buffer garantiza que la cadena siempre quede terminada en nulo y sin
 *          desbordamiento aunque el resultado se trunque.
 *
 * @param[in]     x          Coordenada X superior izquierda del primer carácter.
 * @param[in]     y          Coordenada Y superior izquierda del primer carácter.
 * @param[in]     font       Puntero a la definición de la fuente.
 * @param[in]     foreground Color de primer plano en formato RGB565.
 * @param[in,out] image      Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @param[in]     fmt        Cadena de formato estilo printf (terminada en nulo).
 * @param[in]     ...        Argumentos variádicos correspondientes a fmt.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_INVALID_PARAM   si fmt, font o image son NULL, o si vsnprintf falla.
 */
ILI9341_Status_t ILI9341_Printf_ImageBuffer(uint16_t x, uint16_t y, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32], const char* fmt, ...)
{
    char buf[ILI9341_PRINTF_BUF_SIZE];
    va_list args;
    int len;

    if (fmt == NULL || font == NULL || image == NULL) { return ILI9341_INVALID_PARAM; }

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0) { return ILI9341_INVALID_PARAM; }

    return ILI9341_Puts_ImageBuffer(x, y, buf, font, foreground, image);
}

/**
 * @brief Renderiza una cadena terminada en nulo alineada dentro de una región horizontal, en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0         Borde izquierdo de la región de alineación.
 * @param[in]     x1         Borde derecho de la región de alineación (x1 >= x0).
 * @param[in]     y          Coordenada Y superior izquierda del texto.
 * @param[in]     align      Alineación deseada (izquierda, centro, derecha).
 * @param[in]     str        Puntero a la cadena terminada en nulo.
 * @param[in]     font       Puntero a la definición de la fuente.
 * @param[in]     foreground Color de primer plano en formato RGB565.
 * @param[in,out] image      Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_INVALID_PARAM   si str, font o image son NULL, o si x1 < x0.
 */
ILI9341_Status_t ILI9341_PutsAligned_ImageBuffer(uint16_t x0, uint16_t x1, uint16_t y, ILI9341_TextAlign_t align, char* str, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32])
{
    uint16_t strWidth, strHeight, x;

    if (str == NULL || font == NULL || image == NULL) { return ILI9341_INVALID_PARAM; }
    if (x1 < x0)                                       { return ILI9341_INVALID_PARAM; }

    ILI9341_GetStringSize(str, font, &strWidth, &strHeight);
    x = ILI9341_AlignedX(x0, x1, strWidth, align);

    return ILI9341_Puts_ImageBuffer(x, y, str, font, foreground, image);
}

/**
 * @brief Renderiza una cadena con formato (estilo printf) alineada dentro de una región horizontal, en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0         Borde izquierdo de la región de alineación.
 * @param[in]     x1         Borde derecho de la región de alineación (x1 >= x0).
 * @param[in]     y          Coordenada Y superior izquierda del texto.
 * @param[in]     align      Alineación deseada (izquierda, centro, derecha).
 * @param[in]     font       Puntero a la definición de la fuente.
 * @param[in]     foreground Color de primer plano en formato RGB565.
 * @param[in,out] image      Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @param[in]     fmt        Cadena de formato estilo printf (terminada en nulo).
 * @param[in]     ...        Argumentos variádicos correspondientes a fmt.
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_INVALID_PARAM   si fmt, font o image son NULL, si x1 < x0, o si vsnprintf falla.
 */
ILI9341_Status_t ILI9341_PrintfAligned_ImageBuffer(uint16_t x0, uint16_t x1, uint16_t y, ILI9341_TextAlign_t align, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32], const char* fmt, ...)
{
    char buf[ILI9341_PRINTF_BUF_SIZE];
    va_list args;
    int len;

    if (fmt == NULL || font == NULL || image == NULL) { return ILI9341_INVALID_PARAM; }
    if (x1 < x0)                                      { return ILI9341_INVALID_PARAM; }

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0) { return ILI9341_INVALID_PARAM; }

    return ILI9341_PutsAligned_ImageBuffer(x0, x1, y, align, buf, font, foreground, image);
}

/**
 * @brief Dibuja una línea en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X de inicio.
 * @param[in]     y0     Coordenada Y de inicio.
 * @param[in]     x1     Coordenada X de fin.
 * @param[in]     y1     Coordenada Y de fin.
 * @param[in]     color  Color de la línea en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
ILI9341_Status_t ILI9341_DrawLine_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_Status_t st;
    int16_t dx, dy, sx, sy, err, e2;

    if (image == NULL) { return ILI9341_INVALID_PARAM; }

    if (x0 >= ILI9341_WIDTH)  { x0 = ILI9341_WIDTH  - 1U; }
    if (x1 >= ILI9341_WIDTH)  { x1 = ILI9341_WIDTH  - 1U; }
    if (y0 >= ILI9341_HEIGHT) { y0 = ILI9341_HEIGHT - 1U; }
    if (y1 >= ILI9341_HEIGHT) { y1 = ILI9341_HEIGHT - 1U; }

    /* Líneas horizontales y verticales: delegar a DrawFilledRectangle_ImageBuffer
     * que usa escrituras de 32 bits (2 píxeles/word) sin lecturas en la región central. */
    if (y0 == y1 || x0 == x1)
    {
        uint16_t xa = (x0 < x1) ? x0 : x1;
        uint16_t xb = (x0 < x1) ? x1 : x0;
        uint16_t ya = (y0 < y1) ? y0 : y1;
        uint16_t yb = (y0 < y1) ? y1 : y0;
        return ILI9341_DrawFilledRectangle_ImageBuffer(xa, ya, xb, yb, color, image);
    }

    dx  = (x0 < x1) ? (x1 - x0) : (x0 - x1);
    dy  = (y0 < y1) ? (y1 - y0) : (y0 - y1);
    sx  = (x0 < x1) ?  1 : -1;
    sy  = (y0 < y1) ?  1 : -1;
    err = ((dx > dy) ? dx : -dy) / 2;

    while (1)
    {
        st = ILI9341_DrawPixel_ImageBuffer(x0, y0, color, image);
        if (st != ILI9341_OK) { return st; }
        if (x0 == x1 && y0 == y1) { break; }
        e2 = err;
        if (e2 > -dx) { err -= dy; x0 += sx; }
        if (e2 <  dy) { err += dx; y0 += sy; }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja una línea con grosor (ancho de trazo) en la pantalla LCD.
 *
 * @details Las líneas horizontales y verticales se rellenan con un único rectángulo
 *          (recortado a los límites de pantalla). Las líneas diagonales se aproximan
 *          trazando @p thickness líneas de Bresenham paralelas, desplazadas sobre la
 *          normal del segmento y centradas en la línea original; en ángulos muy
 *          pronunciados puede quedar un ligero aliasing entre trazos adyacentes.
 *
 * @param[in] x0        Coordenada X de inicio.
 * @param[in] y0        Coordenada Y de inicio.
 * @param[in] x1        Coordenada X de fin.
 * @param[in] y1        Coordenada Y de fin.
 * @param[in] thickness Grosor de la línea en píxeles (0 y 1 equivalen a ILI9341_DrawLine()).
 * @param[in] color     Color de la línea en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawThickLine_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t thickness, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    int32_t maxX, maxY, dx, dy;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    if (image == NULL) { return ILI9341_INVALID_PARAM; }

    if (thickness <= 1U)
    {
        return ILI9341_DrawLine_ImageBuffer(x0, y0, x1, y1, color, image);
    }

    maxX = (int32_t)ILI9341_Opts.width  - 1;
    maxY = (int32_t)ILI9341_Opts.height - 1;

    /* Línea horizontal: un único rectángulo relleno recortado a pantalla. */
    if (y0 == y1)
    {
        int32_t xL   = (x0 < x1) ? x0 : x1;
        int32_t xR   = (x0 < x1) ? x1 : x0;
        int32_t half = (int32_t)thickness / 2;
        int32_t yT   = (int32_t)y0 - half;
        int32_t yB   = yT + (int32_t)thickness - 1;

        if (xL < 0)    { xL = 0;    }
        if (xR > maxX) { xR = maxX; }
        if (yT < 0)    { yT = 0;    }
        if (yB > maxY) { yB = maxY; }
        if (xL > xR || yT > yB) { return ILI9341_OK; }

        return ILI9341_DrawFilledRectangle_ImageBuffer((uint16_t)xL, (uint16_t)yT, (uint16_t)xR, (uint16_t)yB, color, image);
    }

    /* Línea vertical: un único rectángulo relleno recortado a pantalla. */
    if (x0 == x1)
    {
        int32_t yT   = (y0 < y1) ? y0 : y1;
        int32_t yB   = (y0 < y1) ? y1 : y0;
        int32_t half = (int32_t)thickness / 2;
        int32_t xL   = (int32_t)x0 - half;
        int32_t xR   = xL + (int32_t)thickness - 1;

        if (yT < 0)    { yT = 0;    }
        if (yB > maxY) { yB = maxY; }
        if (xL < 0)    { xL = 0;    }
        if (xR > maxX) { xR = maxX; }
        if (xL > xR || yT > yB) { return ILI9341_OK; }

        return ILI9341_DrawFilledRectangle_ImageBuffer((uint16_t)xL, (uint16_t)yT, (uint16_t)xR, (uint16_t)yB, color, image);
    }

    /* Línea diagonal: se aproxima con trazos de Bresenham paralelos, desplazados
     * sobre la normal del segmento y centrados en la línea original. */
    dx = (int32_t)x1 - (int32_t)x0;
    dy = (int32_t)y1 - (int32_t)y0;
    {
        float   len = sqrtf((float)(dx * dx + dy * dy));
        float   ux  = -(float)dy / len; /* Normal unitaria (componente X) */
        float   uy  =  (float)dx / len; /* Normal unitaria (componente Y) */
        float   mid = ((float)thickness - 1.0f) / 2.0f;
        int32_t i;

        for (i = 0; i < (int32_t)thickness; i++)
        {
            float             off = (float)i - mid;
            int32_t           ox  = (int32_t)lroundf(ux * off);
            int32_t           oy  = (int32_t)lroundf(uy * off);
            int32_t           nx0 = (int32_t)x0 + ox;
            int32_t           ny0 = (int32_t)y0 + oy;
            int32_t           nx1 = (int32_t)x1 + ox;
            int32_t           ny1 = (int32_t)y1 + oy;
            ILI9341_Status_t  st;

            /* Descartar el trazo si queda completamente fuera de pantalla; recortar
             * cada coordenada por separado en ese caso distorsionaría la pendiente. */
            if ((nx0 < 0 && nx1 < 0) || (nx0 > maxX && nx1 > maxX) ||
                (ny0 < 0 && ny1 < 0) || (ny0 > maxY && ny1 > maxY))
            {
                continue;
            }

            if      (nx0 < 0) { nx0 = 0; } else if (nx0 > maxX) { nx0 = maxX; }
            if      (nx1 < 0) { nx1 = 0; } else if (nx1 > maxX) { nx1 = maxX; }
            if      (ny0 < 0) { ny0 = 0; } else if (ny0 > maxY) { ny0 = maxY; }
            if      (ny1 < 0) { ny1 = 0; } else if (ny1 > maxY) { ny1 = maxY; }

            st = ILI9341_DrawLine_ImageBuffer((uint16_t)nx0, (uint16_t)ny0, (uint16_t)nx1, (uint16_t)ny1, color, image);
            if (st != ILI9341_OK) { return st; }
        }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja el contorno de un rectángulo en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X superior izquierda.
 * @param[in]     y0     Coordenada Y superior izquierda.
 * @param[in]     x1     Coordenada X inferior derecha.
 * @param[in]     y1     Coordenada Y inferior derecha.
 * @param[in]     color  Color de la línea en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
ILI9341_Status_t ILI9341_DrawRectangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_Status_t st;
    st  = ILI9341_DrawLine_ImageBuffer(x0, y0, x1, y0, color, image); /* Superior  */
    st  = st ? st : ILI9341_DrawLine_ImageBuffer(x0, y0, x0, y1, color, image); /* Izquierda */
    st  = st ? st : ILI9341_DrawLine_ImageBuffer(x1, y0, x1, y1, color, image); /* Derecha   */
    st  = st ? st : ILI9341_DrawLine_ImageBuffer(x0, y1, x1, y1, color, image); /* Inferior  */
    return st;
}

/**
 * @brief Dibuja el contorno de un rectángulo con esquinas redondeadas en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X superior izquierda.
 * @param[in]     y0     Coordenada Y superior izquierda.
 * @param[in]     x1     Coordenada X inferior derecha.
 * @param[in]     y1     Coordenada Y inferior derecha.
 * @param[in]     r      Radio de las esquinas en píxeles (se recorta a min(ancho,alto)/2).
 * @param[in]     color  Color del contorno en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK            en caso de éxito.
 *         - ILI9341_INVALID_PARAM si @p image es NULL.
 */
ILI9341_Status_t ILI9341_DrawRoundRect_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t r, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_Status_t st;
    int16_t f, ddF_x, ddF_y, x, y;
    int16_t cx0, cx1, cy0, cy1;
    uint16_t rmax;

    if (image == NULL) { return ILI9341_INVALID_PARAM; }

    if (x0 > x1) { uint16_t _t = x0; x0 = x1; x1 = _t; }
    if (y0 > y1) { uint16_t _t = y0; y0 = y1; y1 = _t; }

    rmax = ((x1 - x0) < (y1 - y0)) ? (x1 - x0) / 2U : (y1 - y0) / 2U;
    if (r > rmax) { r = rmax; }
    if (r == 0U)  { return ILI9341_DrawRectangle_ImageBuffer(x0, y0, x1, y1, color, image); }

    cx0 = (int16_t)x0 + (int16_t)r;
    cx1 = (int16_t)x1 - (int16_t)r;
    cy0 = (int16_t)y0 + (int16_t)r;
    cy1 = (int16_t)y1 - (int16_t)r;

    /* Segmentos rectos */
    st  = ILI9341_DrawLine_ImageBuffer((uint16_t)cx0, y0, (uint16_t)cx1, y0, color, image);
    st  = st ? st : ILI9341_DrawLine_ImageBuffer((uint16_t)cx0, y1, (uint16_t)cx1, y1, color, image);
    st  = st ? st : ILI9341_DrawLine_ImageBuffer(x0, (uint16_t)cy0, x0, (uint16_t)cy1, color, image);
    st  = st ? st : ILI9341_DrawLine_ImageBuffer(x1, (uint16_t)cy0, x1, (uint16_t)cy1, color, image);
    if (st != ILI9341_OK) { return st; }

    f     =  1 - (int16_t)r;
    ddF_x =  1;
    ddF_y = -2 * (int16_t)r;
    x     =  0;
    y     =  (int16_t)r;

    st  = DrawPixelClipped_ImageBuffer(cx0,     cy0 - y, color, image);
    st  = st ? st : DrawPixelClipped_ImageBuffer(cx1,     cy0 - y, color, image);
    st  = st ? st : DrawPixelClipped_ImageBuffer(cx0,     cy1 + y, color, image);
    st  = st ? st : DrawPixelClipped_ImageBuffer(cx1,     cy1 + y, color, image);
    st  = st ? st : DrawPixelClipped_ImageBuffer(cx0 - y, cy0,     color, image);
    st  = st ? st : DrawPixelClipped_ImageBuffer(cx1 + y, cy0,     color, image);
    st  = st ? st : DrawPixelClipped_ImageBuffer(cx0 - y, cy1,     color, image);
    st  = st ? st : DrawPixelClipped_ImageBuffer(cx1 + y, cy1,     color, image);
    if (st != ILI9341_OK) { return st; }

    while (x < y)
    {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++;
        ddF_x += 2;
        f += ddF_x;

        st  = DrawPixelClipped_ImageBuffer(cx0 - x, cy0 - y, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(cx0 - y, cy0 - x, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(cx1 + x, cy0 - y, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(cx1 + y, cy0 - x, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(cx0 - x, cy1 + y, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(cx0 - y, cy1 + x, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(cx1 + x, cy1 + y, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(cx1 + y, cy1 + x, color, image);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja un rectángulo relleno con esquinas redondeadas en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X superior izquierda.
 * @param[in]     y0     Coordenada Y superior izquierda.
 * @param[in]     x1     Coordenada X inferior derecha.
 * @param[in]     y1     Coordenada Y inferior derecha.
 * @param[in]     r      Radio de las esquinas en píxeles (se recorta a min(ancho,alto)/2).
 * @param[in]     color  Color de relleno en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK            en caso de éxito.
 *         - ILI9341_INVALID_PARAM si @p image es NULL.
 *         - ILI9341_ERROR         si falla una transferencia DMA2D interna (solo con HAL_DMA2D_MODULE_ENABLED).
 */
ILI9341_Status_t ILI9341_DrawFilledRoundRect_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t r, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_Status_t st;
    int16_t f, ddF_x, ddF_y, x, y;
    int16_t cx0, cx1, cy0, cy1;
    uint16_t rmax;

    if (image == NULL) { return ILI9341_INVALID_PARAM; }

    if (x0 > x1) { uint16_t _t = x0; x0 = x1; x1 = _t; }
    if (y0 > y1) { uint16_t _t = y0; y0 = y1; y1 = _t; }

    rmax = ((x1 - x0) < (y1 - y0)) ? (x1 - x0) / 2U : (y1 - y0) / 2U;
    if (r > rmax) { r = rmax; }
    if (r == 0U)  { return ILI9341_DrawFilledRectangle_ImageBuffer(x0, y0, x1, y1, color, image); }

    cx0 = (int16_t)x0 + (int16_t)r;
    cx1 = (int16_t)x1 - (int16_t)r;
    cy0 = (int16_t)y0 + (int16_t)r;
    cy1 = (int16_t)y1 - (int16_t)r;

    /* Franja central de ancho completo */
    st = ILI9341_DrawFilledRectangle_ImageBuffer(x0, (uint16_t)cy0, x1, (uint16_t)cy1, color, image);
    if (st != ILI9341_OK) { return st; }

    f     =  1 - (int16_t)r;
    ddF_x =  1;
    ddF_y = -2 * (int16_t)r;
    x     =  0;
    y     =  (int16_t)r;

    st  = DrawHSpanClipped_ImageBuffer(cx0, cx1, cy0 - y, color, image);
    st  = st ? st : DrawHSpanClipped_ImageBuffer(cx0, cx1, cy1 + y, color, image);
    if (st != ILI9341_OK) { return st; }

    while (x < y)
    {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++;
        ddF_x += 2;
        f += ddF_x;

        st  = DrawHSpanClipped_ImageBuffer(cx0 - x, cx1 + x, cy0 - y, color, image);
        st  = st ? st : DrawHSpanClipped_ImageBuffer(cx0 - x, cx1 + x, cy1 + y, color, image);
        st  = st ? st : DrawHSpanClipped_ImageBuffer(cx0 - y, cx1 + y, cy0 - x, color, image);
        st  = st ? st : DrawHSpanClipped_ImageBuffer(cx0 - y, cx1 + y, cy1 + x, color, image);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja un rectángulo relleno en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X superior izquierda.
 * @param[in]     y0     Coordenada Y superior izquierda.
 * @param[in]     x1     Coordenada X inferior derecha.
 * @param[in]     y1     Coordenada Y inferior derecha.
 * @param[in]     color  Color de relleno en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
ILI9341_Status_t ILI9341_DrawFilledRectangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    if (image == NULL) { return ILI9341_INVALID_PARAM; }
    if (x0 >= ILI9341_WIDTH)  { x0 = ILI9341_WIDTH  - 1U; }
    if (x1 >= ILI9341_WIDTH)  { x1 = ILI9341_WIDTH  - 1U; }
    if (y0 >= ILI9341_HEIGHT) { y0 = ILI9341_HEIGHT - 1U; }
    if (y1 >= ILI9341_HEIGHT) { y1 = ILI9341_HEIGHT - 1U; }
    if (x0 > x1) { uint16_t _t = x0; x0 = x1; x1 = _t; }
    if (y0 > y1) { uint16_t _t = y0; y0 = y1; y1 = _t; }

#ifdef HAL_DMA2D_MODULE_ENABLED
    if (ILI9341_hdma2d != NULL)
    {
        uint16_t w   = x1 - x0 + 1U;
        uint16_t h   = y1 - y0 + 1U;
        uint32_t dst = (uint32_t)((uint16_t*)image + (uint32_t)y0 * ILI9341_WIDTH + x0);
        /* En R2M el HAL interpreta el color como ARGB8888 y lo reduce al formato de
         * salida; hay que expandir el RGB565 a RGB888 para recuperar el mismo color.
         * Los corrimientos dejan cada componente en los bits altos de su byte, que es
         * justo lo que el HAL vuelve a truncar (R:5, G:6, B:5). */
        uint32_t argb = ((uint32_t)(color & 0xF800U) << 8) |   /* R5 → bits[23:19] */
                        ((uint32_t)(color & 0x07E0U) << 5) |   /* G6 → bits[15:10] */
                        ((uint32_t)(color & 0x001FU) << 3);    /* B5 → bits[7:3]   */

        /* DMA2D ya inicializado en ILI9341_Init(); aquí solo cambian modo y offset. */
        ILI9341_DMA2D_SetMode(DMA2D_R2M, ILI9341_WIDTH - w);
        if (HAL_DMA2D_Start(ILI9341_hdma2d, argb, dst, w, h) != HAL_OK)         { return ILI9341_ERROR; }
        if (HAL_DMA2D_PollForTransfer(ILI9341_hdma2d, HAL_MAX_DELAY) != HAL_OK) { return ILI9341_ERROR; }
        return ILI9341_OK;
    }
#endif

    /* Camino CPU: respaldo cuando DMA2D no está habilitado o no se inyectó el handle. */
    {
        uint32_t packed = ((uint32_t)color << 16) | (uint32_t)color;
        uint32_t base32;
        uint16_t y, p, inner, pairs;
        uint8_t  head, tail;

        /* ILI9341_WIDTH es par → y * ILI9341_WIDTH siempre es par.
         * La alineación del primer píxel de cada fila depende solo de x0. */
        head  = (uint8_t)(x0 & 1U);
        inner = (x1 - x0 + 1U) - (uint16_t)head;
        pairs = inner >> 1U;
        tail  = (uint8_t)(inner & 1U);

        for (y = y0; y <= y1; y++)
        {
            base32 = (uint32_t)y * (ILI9341_WIDTH / 2U) + (x0 >> 1U);

            /* Píxel cabecera: índice impar → halfword alto del word. */
            if (head)
            {
                ((uint16_t*)image)[(base32 << 1U) + 1U] = color;
                base32++;
            }

            /* Ráfaga central: 2 píxeles por write de 32 bits */
            for (p = 0U; p < pairs; p++)
                image[base32++] = packed;

            /* Píxel cola: índice par → halfword bajo del word. */
            if (tail)
                ((uint16_t*)image)[base32 << 1U] = color;
        }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja el contorno de un círculo en un frame buffer fuera de pantalla.
 *
 * @details Misma lógica que ILI9341_DrawCircle() (Bresenham de punto medio con
 *          simetría de octantes) pero escribe directamente en el frame buffer.
 *          Los píxeles se recortan a los límites fijos del panel.
 *
 * @param[in]     x0     Coordenada X del centro.
 * @param[in]     y0     Coordenada Y del centro.
 * @param[in]     r      Radio en píxeles.
 * @param[in]     color  Color de la línea en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
ILI9341_Status_t ILI9341_DrawCircle_ImageBuffer(int16_t x0, int16_t y0, int16_t r, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_Status_t st;
    int16_t f     =  1 - r;
    int16_t ddF_x =  1;
    int16_t ddF_y = -2 * r;
    int16_t x     =  0;
    int16_t y     =  r;

    if (image == NULL) { return ILI9341_INVALID_PARAM; }

    st  = DrawPixelClipped_ImageBuffer(x0,     y0 + r, color, image);
    st  = st ? st : DrawPixelClipped_ImageBuffer(x0,     y0 - r, color, image);
    st  = st ? st : DrawPixelClipped_ImageBuffer(x0 + r, y0,     color, image);
    st  = st ? st : DrawPixelClipped_ImageBuffer(x0 - r, y0,     color, image);
    if (st != ILI9341_OK) { return st; }

    while (x < y)
    {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++;
        ddF_x += 2;
        f += ddF_x;

        st  = DrawPixelClipped_ImageBuffer(x0 + x, y0 + y, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(x0 - x, y0 + y, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(x0 + x, y0 - y, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(x0 - x, y0 - y, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(x0 + y, y0 + x, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(x0 - y, y0 + x, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(x0 + y, y0 - x, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer(x0 - y, y0 - x, color, image);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja un círculo relleno en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X del centro.
 * @param[in]     y0     Coordenada Y del centro.
 * @param[in]     r      Radio en píxeles.
 * @param[in]     color  Color de relleno en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
ILI9341_Status_t ILI9341_DrawFilledCircle_ImageBuffer(int16_t x0, int16_t y0, int16_t r, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_Status_t st;
    int16_t f     =  1 - r;
    int16_t ddF_x =  1;
    int16_t ddF_y = -2 * r;
    int16_t x     =  0;
    int16_t y     =  r;

    if (image == NULL) { return ILI9341_INVALID_PARAM; }

    /* Píxeles extremos: tope superior e inferior */
    st  = ILI9341_DrawPixel_ImageBuffer((uint16_t)x0, (uint16_t)(y0 + r), color, image);
    st  = st ? st : ILI9341_DrawPixel_ImageBuffer((uint16_t)x0, (uint16_t)(y0 - r), color, image);
    /* Tramo horizontal central */
    st  = st ? st : DrawHSpanClipped_ImageBuffer(x0 - r, x0 + r, y0, color, image);
    if (st != ILI9341_OK) { return st; }

    while (x < y)
    {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++;
        ddF_x += 2;
        f += ddF_x;

        st  = DrawHSpanClipped_ImageBuffer(x0 - x, x0 + x, y0 + y, color, image);
        st  = st ? st : DrawHSpanClipped_ImageBuffer(x0 - x, x0 + x, y0 - y, color, image);
        st  = st ? st : DrawHSpanClipped_ImageBuffer(x0 - y, x0 + y, y0 + x, color, image);
        st  = st ? st : DrawHSpanClipped_ImageBuffer(x0 - y, x0 + y, y0 - x, color, image);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja el contorno de un triángulo en un frame buffer fuera de pantalla.
 *
 * @param[in]     x0     Coordenada X del primer vértice.
 * @param[in]     y0     Coordenada Y del primer vértice.
 * @param[in]     x1     Coordenada X del segundo vértice.
 * @param[in]     y1     Coordenada Y del segundo vértice.
 * @param[in]     x2     Coordenada X del tercer vértice.
 * @param[in]     y2     Coordenada Y del tercer vértice.
 * @param[in]     color  Color de la línea en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
ILI9341_Status_t ILI9341_DrawTriangle_ImageBuffer(uint16_t x0, uint16_t y0,
                                                   uint16_t x1, uint16_t y1,
                                                   uint16_t x2, uint16_t y2,
                                                   uint16_t color,
                                                   uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_Status_t st;
    if (image == NULL) { return ILI9341_INVALID_PARAM; }
    st  = ILI9341_DrawLine_ImageBuffer(x0, y0, x1, y1, color, image);
    st  = st ? st : ILI9341_DrawLine_ImageBuffer(x1, y1, x2, y2, color, image);
    st  = st ? st : ILI9341_DrawLine_ImageBuffer(x2, y2, x0, y0, color, image);
    return st;
}

/**
 * @brief Dibuja un triángulo relleno en un frame buffer fuera de pantalla.
 *
 * @details Misma lógica que ILI9341_DrawFilledTriangle() pero escribe directamente
 *          en el frame buffer. Los tramos se recortan a los límites fijos del panel.
 *
 * @param[in]     x0     Coordenada X del primer vértice.
 * @param[in]     y0     Coordenada Y del primer vértice.
 * @param[in]     x1     Coordenada X del segundo vértice.
 * @param[in]     y1     Coordenada Y del segundo vértice.
 * @param[in]     x2     Coordenada X del tercer vértice.
 * @param[in]     y2     Coordenada Y del tercer vértice.
 * @param[in]     color  Color de relleno en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
ILI9341_Status_t ILI9341_DrawFilledTriangle_ImageBuffer(uint16_t x0, uint16_t y0,
                                                         uint16_t x1, uint16_t y1,
                                                         uint16_t x2, uint16_t y2,
                                                         uint16_t color,
                                                         uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_Status_t st;
    int32_t ax, ay, bx, by, cx, cy, tmp;
    int32_t y, xa, xb;

    if (image == NULL) { return ILI9341_INVALID_PARAM; }

    ax = (int32_t)x0; ay = (int32_t)y0;
    bx = (int32_t)x1; by = (int32_t)y1;
    cx = (int32_t)x2; cy = (int32_t)y2;

    if (ay > by) { tmp = ax; ax = bx; bx = tmp; tmp = ay; ay = by; by = tmp; }
    if (by > cy) { tmp = bx; bx = cx; cx = tmp; tmp = by; by = cy; cy = tmp; }
    if (ay > by) { tmp = ax; ax = bx; bx = tmp; tmp = ay; ay = by; by = tmp; }

    if (ay == cy)
    {
        return DrawHSpanClipped_ImageBuffer((int16_t)ax, (int16_t)cx, (int16_t)ay, color, image);
    }

    for (y = ay; y <= cy; y++)
    {
        xa = ax + (cx - ax) * (y - ay) / (cy - ay);
        if (y <= by)
            xb = (ay == by) ? bx : ax + (bx - ax) * (y - ay) / (by - ay);
        else
            xb = bx + (cx - bx) * (y - by) / (cy - by);
        st = DrawHSpanClipped_ImageBuffer((int16_t)xa, (int16_t)xb, (int16_t)y, color, image);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
}

/**
 * @brief Dibuja el contorno de una elipse en la pantalla LCD.
 *
 * @param[in]     x0    Coordenada X del centro.
 * @param[in]     y0    Coordenada Y del centro.
 * @param[in]     rx    Radio horizontal en píxeles.
 * @param[in]     ry    Radio vertical en píxeles.
 * @param[in]     color Color del contorno en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p rx o @p ry son negativos.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawEllipse_ImageBuffer(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_Status_t st;
    int32_t xa, xb, ya, yb;
    int32_t a, b, b1;
    int32_t dx, dy, err, e2;

    if (image == NULL)    { return ILI9341_INVALID_PARAM; }
    if (rx < 0 || ry < 0) { return ILI9341_INVALID_PARAM; }

    if (rx == 0)
    {
        return ILI9341_DrawLine_ImageBuffer(x0, (int16_t)(y0 - ry), x0, (int16_t)(y0 + ry), color, image);
    }
    if (ry == 0)
    {
        return ILI9341_DrawLine_ImageBuffer((int16_t)(x0 - rx), y0, (int16_t)(x0 + rx), y0, color, image);
    }

    xa = (int32_t)x0 - rx;
    xb = (int32_t)x0 + rx;
    ya = (int32_t)y0 - ry;
    yb = (int32_t)y0 + ry;

    a  = xb - xa;
    b  = yb - ya;
    b1 = b & 1;

    dx  = 4 * (1 - a) * b * b;
    dy  = 4 * (b1 + 1) * a * a;
    err = dx + dy + b1 * a * a;

    ya += (b + 1) / 2;
    yb  = ya - b1;
    a  *= 8 * a;
    b1  = 8 * b * b;

    do
    {
        st  = DrawPixelClipped_ImageBuffer((int16_t)xb, (int16_t)ya, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer((int16_t)xa, (int16_t)ya, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer((int16_t)xa, (int16_t)yb, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer((int16_t)xb, (int16_t)yb, color, image);
        if (st != ILI9341_OK) { return st; }

        e2 = 2 * err;
        if (e2 <= dy) { ya++; yb--; dy += a; err += dy; }
        if (e2 >= dx || 2 * err > dy) { xa++; xb--; dx += b1; err += dx; }
    } while (xa <= xb);

    /* Remate de puntas para elipses muy achatadas (a == 0 en algún eje). */
    while (ya - yb < b)
    {
        st  = DrawPixelClipped_ImageBuffer((int16_t)(xa - 1), (int16_t)ya, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer((int16_t)(xb + 1), (int16_t)ya, color, image);
        ya++;
        st  = st ? st : DrawPixelClipped_ImageBuffer((int16_t)(xa - 1), (int16_t)yb, color, image);
        st  = st ? st : DrawPixelClipped_ImageBuffer((int16_t)(xb + 1), (int16_t)yb, color, image);
        yb--;
        if (st != ILI9341_OK) { return st; }
    }

    return ILI9341_OK;
}

/**
 * @brief Dibuja el contorno de una elipse en la pantalla LCD.
 *
 * @param[in]     x0    Coordenada X del centro.
 * @param[in]     y0    Coordenada Y del centro.
 * @param[in]     rx    Radio horizontal en píxeles.
 * @param[in]     ry    Radio vertical en píxeles.
 * @param[in]     color Color del contorno en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p rx o @p ry son negativos.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledEllipse_ImageBuffer(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_Status_t st;
    int32_t xa, xb, ya, yb;
    int32_t a, b, b1;
    int32_t dx, dy, err, e2;

    if (image == NULL)    { return ILI9341_INVALID_PARAM; }
    if (rx < 0 || ry < 0) { return ILI9341_INVALID_PARAM; }

    if (rx == 0)
    {
        return ILI9341_DrawLine_ImageBuffer(x0, (int16_t)(y0 - ry), x0, (int16_t)(y0 + ry), color, image);
    }
    if (ry == 0)
    {
        return ILI9341_DrawLine_ImageBuffer((int16_t)(x0 - rx), y0, (int16_t)(x0 + rx), y0, color, image);
    }

    /* Mismo trazado de contorno que ILI9341_DrawEllipse_ImageBuffer (Zingl), pero rellenando
     * cada fila visitada con un tramo horizontal en vez de graficar 4 píxeles. */
    xa = (int32_t)x0 - rx;
    xb = (int32_t)x0 + rx;
    ya = (int32_t)y0 - ry;
    yb = (int32_t)y0 + ry;

    a  = xb - xa;
    b  = yb - ya;
    b1 = b & 1;

    dx  = 4 * (1 - a) * b * b;
    dy  = 4 * (b1 + 1) * a * a;
    err = dx + dy + b1 * a * a;

    ya += (b + 1) / 2;
    yb  = ya - b1;
    a  *= 8 * a;
    b1  = 8 * b * b;

    do
    {
        st  = DrawHSpanClipped_ImageBuffer((int16_t)xa, (int16_t)xb, (int16_t)ya, color, image);
        st  = st ? st : DrawHSpanClipped_ImageBuffer((int16_t)xa, (int16_t)xb, (int16_t)yb, color, image);
        if (st != ILI9341_OK) { return st; }

        e2 = 2 * err;
        if (e2 <= dy) { ya++; yb--; dy += a; err += dy; }
        if (e2 >= dx || 2 * err > dy) { xa++; xb--; dx += b1; err += dx; }
    } while (xa <= xb);

    /* Remate de puntas para elipses muy achatadas (a == 0 en algún eje). */
    while (ya - yb < b)
    {
        ya++;
        yb--;
        st  = DrawHSpanClipped_ImageBuffer((int16_t)(xa - 1), (int16_t)(xb + 1), (int16_t)ya, color, image);
        st  = st ? st : DrawHSpanClipped_ImageBuffer((int16_t)(xa - 1), (int16_t)(xb + 1), (int16_t)yb, color, image);
        if (st != ILI9341_OK) { return st; }
    }

    return ILI9341_OK;
}

/**
 * @brief Dibuja el contorno de un arco (sector de anillo) entre dos ángulos.
 *
 * @param[in] x     Coordenada X del centro.
 * @param[in] y     Coordenada Y del centro.
 * @param[in] r1    Radio exterior del arco.
 * @param[in] r2    Radio interior del arco.
 * @param[in] start Ángulo inicial en grados (0° = derecha, sentido horario).
 * @param[in] end   Ángulo final en grados.
 * @param[in] color Color del contorno.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p rx o @p ry son negativos.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawArc_ImageBuffer(int16_t x, int16_t y, int16_t r1, int16_t r2, float start, float end, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_Status_t st;
    bool equal;
    int16_t tmp;

    if (ILI9341_Initialized != 1U) { return ILI9341_NOT_INITIALIZED; }
    if (image == NULL)             { return ILI9341_INVALID_PARAM;   }
    if (r1 < 0 || r2 < 0)          { return ILI9341_INVALID_PARAM;   }

    if (r1 < r2) { tmp = r1; r1 = r2; r2 = tmp; }
    if (r1 < 1)  { r1 = 1; }
    if (r2 < 1)  { r2 = 1; }

    equal = fabsf(start - end) < FLT_EPSILON;
    start = fmodf(start, 360.0f);
    end   = fmodf(end, 360.0f);
    if (start < 0) { start += 360.0f; }
    if (end < 0)   { end   += 360.0f; }

    /* Bordes rectos del arco (en start y en end) */
    st  = ILI9341_FillArcHelper(x, y, r1, r2, start, start, color, image);
    st  = st ? st : ILI9341_FillArcHelper(x, y, r1, r2, end, end, color, image);
    if (st != ILI9341_OK) { return st; }

    if (!equal && (fabsf(start - end) <= 0.0001f))
    {
        start = 0.0f;
        end   = 360.0f;
    }

    /* Bordes curvos del arco (radio exterior e interior) */
    st  = ILI9341_FillArcHelper(x, y, r1, r1, start, end, color, image);
    st  = st ? st : ILI9341_FillArcHelper(x, y, r2, r2, start, end, color, image);

    return st;
}

/**
 * @brief Dibuja un arco relleno (sector de anillo) entre dos ángulos.
 *
 * @param[in] x0    Coordenada X del centro.
 * @param[in] y0    Coordenada Y del centro.
 * @param[in] r1    Radio exterior del arco.
 * @param[in] r2    Radio interior del arco.
 * @param[in] start Ángulo inicial en grados (0° = derecha, sentido horario).
 * @param[in] end   Ángulo final en grados.
 * @param[in] color Color del contorno.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si @p image es NULL, o si @p r1 o @p r2 son negativos.
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_DrawFilledArc_ImageBuffer(int16_t x0, int16_t y0, int16_t r1, int16_t r2, float start, float end, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    bool equal;
    int16_t tmp;

    if (ILI9341_Initialized != 1U) { return ILI9341_NOT_INITIALIZED; }
    if (image == NULL)             { return ILI9341_INVALID_PARAM;   }
    if (r1 < 0 || r2 < 0)          { return ILI9341_INVALID_PARAM;   }

    if (r1 < r2) { tmp = r1; r1 = r2; r2 = tmp; }
    if (r1 < 1)  { r1 = 1; }
    if (r2 < 1)  { r2 = 1; }

    equal = fabsf(start - end) < FLT_EPSILON;
    start = fmodf(start, 360.0f);
    end   = fmodf(end, 360.0f);
    if (start < 0) { start += 360.0f; }
    if (end < 0)   { end   += 360.0f; }

    if (!equal && (fabsf(start - end) <= 0.0001f))
    {
        start = 0.0f;
        end   = 360.0f;
    }

    return ILI9341_FillArcHelper(x0, y0, r1, r2, start, end, color, image);
}

#ifdef HAL_DMA2D_MODULE_ENABLED
/**
 * @brief Copia una imagen RGB565 al frame buffer usando DMA2D (memoria a memoria).
 *
 * @param[in]     src         Puntero a la imagen fuente en formato RGB565.
 * @param[in]     x0          Coordenada X de la esquina superior izquierda en el frame buffer.
 * @param[in]     y0          Coordenada Y de la esquina superior izquierda en el frame buffer.
 * @param[in]     img_w       Ancho de la imagen fuente en píxeles (se recorta al borde).
 * @param[in]     img_h       Alto de la imagen fuente en píxeles (se recorta al borde).
 * @param[in,out] framebuffer Frame buffer destino (IMG_TOTAL_BUF32 palabras uint32_t).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito (incluye x0/y0 fuera de pantalla, que se ignora).
 *         - ILI9341_INVALID_PARAM   si @p src, @p framebuffer son NULL, o el handle DMA2D no fue inyectado.
 *         - ILI9341_ERROR           si falla la transferencia DMA2D.
 */
ILI9341_Status_t ILI9341_BlitImage(const uint16_t* src, uint16_t x0, uint16_t y0,
                                    uint16_t img_w, uint16_t img_h,
                                    uint32_t* framebuffer)
{
    uint32_t dst;

    if (src == NULL || framebuffer == NULL)           { return ILI9341_INVALID_PARAM; }
    if (ILI9341_hdma2d == NULL)                       { return ILI9341_INVALID_PARAM; }
    if (x0 >= ILI9341_WIDTH || y0 >= ILI9341_HEIGHT)  { return ILI9341_OK; }
    if ((uint32_t)x0 + img_w > ILI9341_WIDTH)  { img_w = ILI9341_WIDTH  - x0; }
    if ((uint32_t)y0 + img_h > ILI9341_HEIGHT) { img_h = ILI9341_HEIGHT - y0; }

    /* Modo + formato + capa ya configurados en ILI9341_Init();
       aquí solo se actualizan modo (M2M) y offset de salida. */
    ILI9341_DMA2D_SetMode(DMA2D_M2M, ILI9341_WIDTH - img_w);

    dst = (uint32_t)((uint16_t*)framebuffer + (uint32_t)y0 * ILI9341_WIDTH + x0);
    if (HAL_DMA2D_Start(ILI9341_hdma2d, (uint32_t)src, dst, img_w, img_h) != HAL_OK) { return ILI9341_ERROR; }
    if (HAL_DMA2D_PollForTransfer(ILI9341_hdma2d, HAL_MAX_DELAY) != HAL_OK)          { return ILI9341_ERROR; }
    return ILI9341_OK;
}
#endif /* HAL_DMA2D_MODULE_ENABLED */

// ============================================================================
// FUNCIONES PÚBLICAS — Frame buffer (doble buffer con pipelining DMA)
// ============================================================================

/**
 * @brief Registra los buffers usados por el modo de doble buffer con pipelining DMA.
 *
 * @details El usuario reserva ambos buffers (RAM interna, RAM externa, etc.), cada uno
 *          de IMG_TOTAL_BUF32 palabras uint32_t; se limpian a cero al registrarse.
 *          Pasar NULL en ambos deshabilita el modo doble buffer (ILI9341_Flush(),
 *          ILI9341_Sync() y ILI9341_GetFrameBuffer() retornarán ILI9341_INVALID_PARAM/NULL).
 *
 * @param[in] front Buffer que se transmite a la LCD (usado por ILI9341_Flush()/ILI9341_Sync()).
 * @param[in] back  Buffer sobre el que dibuja la CPU (ver ILI9341_GetFrameBuffer()).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si solo uno de los dos punteros es NULL.
 */
ILI9341_Status_t ILI9341_SetFrameBuffers(uint32_t* front, uint32_t* back)
{
    if (!ILI9341_Initialized)               { return ILI9341_NOT_INITIALIZED; }
    if ((front == NULL) != (back == NULL))  { return ILI9341_INVALID_PARAM;   }

    ILI9341_framebuffer      = front;
    ILI9341_back_framebuffer = back;

    if (front != NULL)
    {
        memset(ILI9341_framebuffer,      0, IMG_TOTAL_BUF32 * sizeof(uint32_t));
        memset(ILI9341_back_framebuffer, 0, IMG_TOTAL_BUF32 * sizeof(uint32_t));
    }

    return ILI9341_OK;
}

/**
 * @brief Presenta el frame dibujado en pantalla usando doble buffer con pipelining DMA.
 *
 * @details Delega en ILI9341_PresentFrame() (privada): espera el DMA anterior, intercambia
 *          front/back e inicia el DMA del frame recién dibujado sin bloquear.
 *          Ver la documentación del header para el patrón de uso completo.
 *
 * @return ILI9341_Status_t
 */
ILI9341_Status_t ILI9341_Flush(void)
{
    if (!ILI9341_Initialized)                                            { return ILI9341_NOT_INITIALIZED; }
    if (ILI9341_framebuffer == NULL || ILI9341_back_framebuffer == NULL) { return ILI9341_INVALID_PARAM;   }
    return ILI9341_PresentFrame();
}

/**
 * @brief Espera a que concluya el DMA en curso y restaura el bus SPI al modo 8 bits.
 *
 * @details Debe llamarse al salir del modo de doble buffer antes de usar funciones de
 *          dibujo directo en pantalla (ILI9341_Fill, ILI9341_DrawPixel, etc.).
 *          Si no hay DMA activo retorna inmediatamente sin efecto.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si el bus quedó libre correctamente.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_TIMEOUT         si el DMA no terminó en 5 000 ms.
 */
ILI9341_Status_t ILI9341_Sync(void)
{
    ILI9341_Status_t st = ILI9341_OK;
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    if (!ILI9341_dma_cs_held) { return ILI9341_OK; }

    if (ILI9341_dma_state != 0U)
    {
        st = ILI9341_SPI_WaitDMAdone(5000U);
    }
    ILI9341_SPI_SetDataSize(SPI_DATASIZE_8BIT);
    ILI9341_CS_SET;
    ILI9341_WRX_RESET;
    ILI9341_dma_cs_held = 0U;
    return st;
}

/**
 * @brief Inicia la transferencia del front buffer a la LCD por SPI DMA sin bloquear.
 *
 * @details El callback HAL_SPI_TxCpltCallback encadena automáticamente el segundo tramo
 *          (píxeles 38 400–76 799) para respetar el límite de 65 535 items del NDTR de DMA.
 *          La función retorna en cuanto el primer tramo está encolado en el DMA.
 *
 * @return ILI9341_Status_t
 */
static ILI9341_Status_t ILI9341_FlushAsync(void)
{
    ILI9341_Status_t st;
    uint16_t* const  px   = (uint16_t*)ILI9341_framebuffer;
    const uint16_t   half = ILI9341_PIXEL / 2U;

    if (!ILI9341_Initialized)                                            { return ILI9341_NOT_INITIALIZED; }
    if (ILI9341_framebuffer == NULL || ILI9341_back_framebuffer == NULL) { return ILI9341_INVALID_PARAM;   }
    if (ILI9341_dma_state != 0U)                                         { return ILI9341_ERROR;            }

    ILI9341_SetCursorPosition(0U, 0U, ILI9341_Opts.width - 1U, ILI9341_Opts.height - 1U);
    ILI9341_SendCommand(ILI9341_GRAM);
    ILI9341_WRX_SET;
    ILI9341_CS_RESET;

    st = ILI9341_SPI_SetDataSize(SPI_DATASIZE_16BIT);
    if (st != ILI9341_OK)
    {
        ILI9341_CS_SET; ILI9341_WRX_RESET;
        return st;
    }

    /* Prepara segundo tramo antes de armar el estado — el callback puede disparar
     * muy rápido y debe encontrar px2/half ya escritos. */
    ILI9341_dma_px2      = px + half;
    ILI9341_dma_half     = half;
    ILI9341_spi_dma_done = 0U;
    ILI9341_dma_cs_held  = 1U;
    ILI9341_dma_state    = 1U; /* arma la máquina de estados ANTES del HAL call */

    if (HAL_SPI_Transmit_DMA(ILI9341_hspi, (uint8_t*)px, half) != HAL_OK)
    {
        ILI9341_dma_state   = 0U;
        ILI9341_dma_cs_held = 0U;
        ILI9341_SPI_SetDataSize(SPI_DATASIZE_8BIT);
        ILI9341_CS_SET; ILI9341_WRX_RESET;
        return ILI9341_ERROR;
    }

    return ILI9341_OK; /* DMA corriendo; caller puede dibujar en el back buffer */
}

/**
 * @brief Espera el DMA en curso, hace swap de buffers e inicia la transferencia del nuevo frame.
 *
 * @details Implementa el paso de sincronización del patrón de doble buffer con pipelining.
 *          Ver ILI9341_FlushAsync() y la documentación del header para el patrón de uso.
 *
 * @return ILI9341_Status_t
 */
static ILI9341_Status_t ILI9341_PresentFrame(void)
{
    ILI9341_Status_t st   = ILI9341_OK;
    uint32_t*        tmp;

    if (!ILI9341_Initialized)                                            { return ILI9341_NOT_INITIALIZED; }
    if (ILI9341_framebuffer == NULL || ILI9341_back_framebuffer == NULL) { return ILI9341_INVALID_PARAM;   }

    /* Sincronizar con el DMA anterior si sigue activo o aún no se limpió CS. */
    if (ILI9341_dma_cs_held)
    {
        if (ILI9341_dma_state != 0U)
        {
            st = ILI9341_SPI_WaitDMAdone(5000U);
        }
        ILI9341_SPI_SetDataSize(SPI_DATASIZE_8BIT);
        ILI9341_CS_SET;
        ILI9341_WRX_RESET;
        ILI9341_dma_cs_held = 0U;
        if (st != ILI9341_OK) { return st; }
    }

    /* Swap: el back buffer recién dibujado pasa a ser el front. */
    tmp                      = ILI9341_framebuffer;
    ILI9341_framebuffer      = ILI9341_back_framebuffer;
    ILI9341_back_framebuffer = tmp;

    /* Arrancar DMA sobre el nuevo front buffer inmediatamente. */
    return ILI9341_FlushAsync();
}

/**
 * @brief Retorna el puntero al back buffer activo registrado con ILI9341_SetFrameBuffers().
 *
 * @details Devuelve siempre el back buffer (el buffer sobre el que debe dibujar la CPU).
 *          Tiene formato IMG_TOTAL_BUF32 palabras uint32_t, compatible con todas las
 *          funciones ILI9341_*_ImageBuffer(). El puntero cambia tras cada llamada a
 *          ILI9341_Flush(), por lo que hay que invocarlo de nuevo en cada frame.
 *          Retorna NULL si no se registraron buffers o el driver no está inicializado.
 *
 * @return Puntero al back buffer activo, o NULL si no está disponible.
 */
uint32_t* ILI9341_GetFrameBuffer(void)
{
    if (!ILI9341_Initialized) { return NULL; }
    return ILI9341_back_framebuffer;
}

/**
 * @brief Desinicializa el driver LCD y libera los recursos periféricos.
 *
 * @details Marca el driver como no inicializado, pone a NULL los handles
 *          internos de SPI, I2C y DMA2D, y limpia los punteros de frame buffer
 *          registrados con ILI9341_SetFrameBuffers() (la memoria en sí sigue siendo
 *          responsabilidad del usuario). Tras esta llamada es necesario invocar
 *          ILI9341_Init() antes de usar cualquier otra función.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si la desinicialización fue exitosa.
 *         - ILI9341_NOT_INITIALIZED si el driver no estaba inicializado.
 */
ILI9341_Status_t ILI9341_DeInit(void)
{
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    ILI9341_Initialized = 0U;

    ILI9341_framebuffer      = NULL;
    ILI9341_back_framebuffer = NULL;
    ILI9341_dma_state        = 0U;
    ILI9341_dma_cs_held      = 0U;
    ILI9341_TouchDriver      = ILI9341_TOUCH_NONE;

    ILI9341_hspi = NULL;
#ifdef HAL_I2C_MODULE_ENABLED
    ILI9341_hi2c = NULL;
#endif
#ifdef HAL_DMA2D_MODULE_ENABLED
    ILI9341_hdma2d = NULL;
#endif

    return ILI9341_OK;
}

// ============================================================================
// FUNCIONES PÚBLICAS — Touch panel
// ============================================================================

#ifdef HAL_I2C_MODULE_ENABLED
/**
 * @brief Configura el controlador del panel táctil STMPE811.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si el dispositivo fue detectado y configurado.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si el ID del chip no coincidió con STMPE811_ID
 *                                   o si cualquier operación I2C de configuración falló.
 */
ILI9341_Status_t ILI9341_TP_Config(void)
{
    uint16_t         tmp    = 0U;
    ILI9341_Status_t status = ILI9341_OK;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    tmp  = (uint16_t)ILI9341_TP_ReadDeviceRegister(0U) << 8;
    tmp |= (uint16_t)ILI9341_TP_ReadDeviceRegister(1U);
    if (tmp != (uint16_t)STMPE811_ID) { return ILI9341_ERROR; }

    if ((status = ILI9341_TP_Reset())                                              != ILI9341_OK) { return status; }
    if ((status = ILI9341_TP_FnctCmd(TP_ADC_FCT, ENABLE))                         != ILI9341_OK) { return status; }
    if ((status = ILI9341_TP_FnctCmd(TP_TP_FCT,  ENABLE))                         != ILI9341_OK) { return status; }
    if ((status = ILI9341_TP_WriteDeviceRegister(TP_REG_ADC_CTRL1, 0x49U))        != ILI9341_OK) { return status; }
    ILI9341_Delay(200U);
    if ((status = ILI9341_TP_WriteDeviceRegister(TP_REG_ADC_CTRL2,    0x01U))     != ILI9341_OK) { return status; }
    if ((status = ILI9341_TP_IOAFConfig((uint8_t)TOUCH_IO_ALL, DISABLE))          != ILI9341_OK) { return status; }
    if ((status = ILI9341_TP_WriteDeviceRegister(TP_REG_TP_CFG,       0x9AU))     != ILI9341_OK) { return status; }
    if ((status = ILI9341_TP_WriteDeviceRegister(TP_REG_FIFO_TH,      0x01U))     != ILI9341_OK) { return status; }
    if ((status = ILI9341_TP_WriteDeviceRegister(TP_REG_FIFO_STA,     0x01U))     != ILI9341_OK) { return status; }
    if ((status = ILI9341_TP_WriteDeviceRegister(TP_REG_FIFO_STA,     0x00U))     != ILI9341_OK) { return status; }
    if ((status = ILI9341_TP_WriteDeviceRegister(TP_REG_TP_FRACT_XYZ, 0x01U))     != ILI9341_OK) { return status; }
    if ((status = ILI9341_TP_WriteDeviceRegister(TP_REG_TP_I_DRIVE,   0x01U))     != ILI9341_OK) { return status; }
    if ((status = ILI9341_TP_WriteDeviceRegister(TP_REG_TP_CTRL,      0x03U))     != ILI9341_OK) { return status; }
    if ((status = ILI9341_TP_WriteDeviceRegister(TP_REG_INT_STA,      0xFFU))     != ILI9341_OK) { return status; }

    TP_State.TouchDetected = TP_State.X = TP_State.Y = TP_State.Z = 0U;
    ILI9341_TouchDriver = ILI9341_TOUCH_STMPE811;

    return ILI9341_OK;
}

/**
 * @brief Implementación STMPE811 de la lectura de estado táctil.
 *
 * @return Puntero a la estructura TP_STATE interna con valores actualizados.
 */
static TP_STATE* STMPE811_TP_GetState(void)
{
    uint32_t xDiff, yDiff, x, y;
    static uint32_t _x = 0U, _y = 0U;

    TP_State.TouchDetected = (ILI9341_TP_ReadDeviceRegister(TP_REG_TP_CTRL) & 0x80U);

    if (TP_State.TouchDetected)
    {
        x     = ILI9341_TP_Read_X();
        y     = ILI9341_TP_Read_Y();
        xDiff = (x > _x) ? (x - _x) : (_x - x);
        yDiff = (y > _y) ? (y - _y) : (_y - y);
        if ((xDiff + yDiff) > 5U) { _x = x; _y = y; }
    }

    TP_State.X = (uint16_t)_x;
    TP_State.Y = (uint16_t)_y;
    TP_State.Z = ILI9341_TP_Read_Z();

    ILI9341_TP_WriteDeviceRegister(TP_REG_FIFO_STA, 0x01U);
    ILI9341_TP_WriteDeviceRegister(TP_REG_FIFO_STA, 0x00U);

    return &TP_State;
}
#endif /* HAL_I2C_MODULE_ENABLED */

/**
 * @brief Configura el controlador del panel táctil XPT2046 (SPI).
 *
 * @param[in] csPort  Puerto GPIO del pin CS (T_CS) del XPT2046.
 * @param[in] csPin   Pin GPIO del pin CS del XPT2046.
 * @param[in] irqPort Puerto GPIO del pin PENIRQ (T_IRQ), o NULL si no está conectado.
 * @param[in] irqPin  Pin GPIO del pin PENIRQ (ignorado si irqPort es NULL).
 * @return ILI9341_Status_t
 *         - ILI9341_OK              en caso de éxito.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si csPort es NULL.
 */
ILI9341_Status_t ILI9341_TP_ConfigXPT2046(GPIO_TypeDef* csPort, uint16_t csPin,
                                           GPIO_TypeDef* irqPort, uint16_t irqPin)
{
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    if (csPort == NULL) { return ILI9341_INVALID_PARAM; }

    XPT2046_CS_Port  = csPort;  XPT2046_CS_Pin  = csPin;
    XPT2046_IRQ_Port = irqPort; XPT2046_IRQ_Pin = irqPin;

    HAL_GPIO_WritePin(XPT2046_CS_Port, XPT2046_CS_Pin, GPIO_PIN_SET);

    TP_State.TouchDetected = TP_State.X = TP_State.Y = TP_State.Z = 0U;
    ILI9341_TouchDriver = ILI9341_TOUCH_XPT2046;

    return ILI9341_OK;
}

/**
 * @brief Lee un canal ADC del XPT2046 (control byte + 2 bytes de resultado de 12 bits).
 *
 * @param[in]  cmd    Byte de control (XPT2046_CMD_X, _Y, _Z1 o _Z2).
 * @param[out] result Valor de 12 bits leído (0-4095).
 * @return ILI9341_OK si la transacción SPI fue exitosa; ILI9341_ERROR en caso contrario.
 */
static ILI9341_Status_t XPT2046_ReadChannel(uint8_t cmd, uint16_t* result)
{
    uint8_t tx[3] = { cmd, 0x00U, 0x00U };
    uint8_t rx[3] = { 0U, 0U, 0U };

    if (HAL_SPI_TransmitReceive(ILI9341_hspi, tx, rx, 3U, 100U) != HAL_OK) { return ILI9341_ERROR; }

    *result = (((uint16_t)rx[1] << 8) | (uint16_t)rx[2]) >> 3;
    return ILI9341_OK;
}

/**
 * @brief Lee las coordenadas X/Y crudas y la presión (Z1) del XPT2046.
 *
 * @details Baja el preescalador SPI a XPT2046_SPI_PRESCALER, selecciona el XPT2046
 *          con su CS dedicado, descarta la primera muestra de X/Y (asentamiento del
 *          multiplexor analógico) y restaura el preescalador original al terminar.
 *
 * @param[out] x Coordenada X cruda (0-4095).
 * @param[out] y Coordenada Y cruda (0-4095).
 * @param[out] z Presión cruda (canal Z1, mayor valor = mayor presión).
 * @return ILI9341_OK en caso de éxito; ILI9341_ERROR si falla la transacción SPI o si
 *         el bus SPI está ocupado con un volcado de frame buffer asíncrono en curso
 *         (ILI9341_FlushAsync); reintentar en el siguiente ciclo de sondeo.
 */
static ILI9341_Status_t XPT2046_ReadRaw(uint16_t* x, uint16_t* y, uint16_t* z)
{
    ILI9341_Status_t st;
    uint16_t discard, rawX = 0U, rawY = 0U, rawZ = 0U;
    uint32_t savedPrescaler = ILI9341_hspi->Init.BaudRatePrescaler;

    if (ILI9341_dma_state != 0U) { return ILI9341_ERROR; }

    SPI_ILI9341_SetPrescaler(XPT2046_SPI_PRESCALER);
    HAL_GPIO_WritePin(XPT2046_CS_Port, XPT2046_CS_Pin, GPIO_PIN_RESET);

    st  = XPT2046_ReadChannel(XPT2046_CMD_X, &discard);
    st  = (st == ILI9341_OK) ? XPT2046_ReadChannel(XPT2046_CMD_X,  &rawX) : st;
    st  = (st == ILI9341_OK) ? XPT2046_ReadChannel(XPT2046_CMD_Y,  &discard) : st;
    st  = (st == ILI9341_OK) ? XPT2046_ReadChannel(XPT2046_CMD_Y,  &rawY) : st;
    st  = (st == ILI9341_OK) ? XPT2046_ReadChannel(XPT2046_CMD_Z1, &rawZ) : st;

    HAL_GPIO_WritePin(XPT2046_CS_Port, XPT2046_CS_Pin, GPIO_PIN_SET);
    SPI_ILI9341_SetPrescaler(savedPrescaler);

    if (st != ILI9341_OK) { return st; }

    *x = rawX;
    *y = rawY;
    *z = rawZ;
    return ILI9341_OK;
}

/**
 * @brief Implementación XPT2046 de la lectura de estado táctil.
 *
 * @details Si se configuró un pin PENIRQ, este determina la detección de toque
 *          (activo en bajo) y solo entonces se leen las coordenadas por SPI. Sin
 *          PENIRQ, se lee siempre y la detección se basa en XPT2046_PRESSURE_THRESHOLD.
 *
 * @return Puntero a la estructura TP_STATE interna con valores actualizados.
 */
static TP_STATE* XPT2046_TP_GetState(void)
{
    uint16_t rawX, rawY, rawZ;
    int32_t mapX, mapY;
    static uint32_t _x = 0U, _y = 0U;

    if (XPT2046_IRQ_Port != NULL && HAL_GPIO_ReadPin(XPT2046_IRQ_Port, XPT2046_IRQ_Pin) == GPIO_PIN_SET)
    {
        TP_State.TouchDetected = 0U;
        TP_State.Z             = 0U;
        return &TP_State;
    }

    if (XPT2046_ReadRaw(&rawX, &rawY, &rawZ) != ILI9341_OK)
    {
        TP_State.TouchDetected = 0U;
        return &TP_State;
    }

    TP_State.TouchDetected = (XPT2046_IRQ_Port != NULL) ? 1U : (rawZ >= XPT2046_PRESSURE_THRESHOLD);
    TP_State.Z              = rawZ;

    if (TP_State.TouchDetected)
    {
#if XPT2046_SWAP_XY
        { uint16_t tmp = rawX; rawX = rawY; rawY = tmp; }
#endif
        mapX = ((int32_t)rawX - (int32_t)XPT2046_X_MIN) * (int32_t)ILI9341_WIDTH
               / ((int32_t)XPT2046_X_MAX - (int32_t)XPT2046_X_MIN);
        mapY = ((int32_t)rawY - (int32_t)XPT2046_Y_MIN) * (int32_t)ILI9341_HEIGHT
               / ((int32_t)XPT2046_Y_MAX - (int32_t)XPT2046_Y_MIN);

        if (mapX < 0) { mapX = 0; } else if (mapX >= (int32_t)ILI9341_WIDTH)  { mapX = (int32_t)ILI9341_WIDTH  - 1; }
        if (mapY < 0) { mapY = 0; } else if (mapY >= (int32_t)ILI9341_HEIGHT) { mapY = (int32_t)ILI9341_HEIGHT - 1; }

#if XPT2046_INVERT_X
        mapX = (int32_t)ILI9341_WIDTH  - 1 - mapX;
#endif
#if XPT2046_INVERT_Y
        mapY = (int32_t)ILI9341_HEIGHT - 1 - mapY;
#endif

        _x = (uint32_t)mapX;
        _y = (uint32_t)mapY;
    }

    TP_State.X = (uint16_t)_x;
    TP_State.Y = (uint16_t)_y;

    return &TP_State;
}

/**
 * @brief Lee el estado actual del panel táctil (coordenadas y detección de toque).
 *
 * @details Despacha al controlador configurado con ILI9341_TP_Config() (STMPE811)
 *          o ILI9341_TP_ConfigXPT2046() (XPT2046).
 *
 * @return Puntero a la estructura TP_STATE interna con valores actualizados,
 *         o NULL si el driver no ha sido inicializado o no hay touch configurado.
 */
TP_STATE* ILI9341_TP_GetState(void)
{
    if (!ILI9341_Initialized) { return NULL; }

    switch (ILI9341_TouchDriver)
    {
#ifdef HAL_I2C_MODULE_ENABLED
        case ILI9341_TOUCH_STMPE811: return STMPE811_TP_GetState();
#endif
        case ILI9341_TOUCH_XPT2046:  return XPT2046_TP_GetState();
        default:                     return NULL;
    }
}
