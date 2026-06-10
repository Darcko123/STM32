/**
 * @file ILI9341_Disc1.c
 * @brief Implementación del driver del controlador LCD ILI9341 para la placa
 * STM32F429-Discovery.
 *
 * @author Dr. Luis Antonio Raygoza Pérez & Ing. Daniel Ruiz
 * @date June 08, 2026
 * @version 0.1.0
 */

#include "ILI9341_Disc1.h"

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

static SPI_HandleTypeDef*    ILI9341_hspi        = NULL; /**< Handle SPI usado para la comunicación con el ILI9341      */
#ifdef HAL_I2C_MODULE_ENABLED
static I2C_HandleTypeDef*    ILI9341_hi2c        = NULL; /**< Handle I2C usado para la comunicación con el STMPE811     */
#endif
static uint8_t               ILI9341_Initialized = 0U;   /**< Bandera de inicialización: 1 = driver listo, 0 = no listo */

static uint16_t              ILI9341_x           = 0U;   /**< Columna del cursor de texto activo                        */
static uint16_t              ILI9341_y           = 0U;   /**< Fila    del cursor de texto activo                        */
static ILI9341_Options_t ILI9341_Opts;                   /**< Geometría y orientación actuales de la pantalla           */

#ifdef HAL_I2C_MODULE_ENABLED
static TP_STATE TP_State; /**< Estado interno del panel táctil (actualizado en ILI9341_TP_GetState()) */
#endif

#ifdef HAL_SDRAM_MODULE_ENABLED
static SDRAM_HandleTypeDef* ILI9341_hsdram       = NULL; /**< Handle SDRAM; NULL si la SDRAM no fue habilitada en Init  */
static uint32_t*            ILI9341_framebuffer  = NULL; /**< Puntero al frame buffer en SDRAM; NULL si no disponible   */

/* Verificación en tiempo de compilación: el frame buffer debe caber en el chip SDRAM.
 * Genera un error "array size is negative" si la condición no se cumple. */
typedef char ILI9341_sdram_fb_size_check[(ILI9341_SDRAM_FB_SIZE <= IS42S16400J_SIZE) ? 1 : -1];
#endif

// ============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
// ============================================================================

static ILI9341_Status_t SPI_ILI9341_Send(uint8_t* data, uint16_t size);
static void SPI_ILI9341_BaudRateUp(void);
#ifdef HAL_SDRAM_MODULE_ENABLED
static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef* hsdram, FMC_SDRAM_CommandTypeDef* Command);
#endif
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
#endif /* HAL_I2C_MODULE_ENABLED */

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
static void SPI_ILI9341_BaudRateUp(void)
{
    HAL_SPI_DeInit(ILI9341_hspi);
    ILI9341_hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    HAL_SPI_Init(ILI9341_hspi);
}

#ifdef HAL_SDRAM_MODULE_ENABLED
/**
 * @brief Ejecuta la secuencia de inicialización requerida por el IS42S16400J.
 *
 * @details Sigue los pasos del §3.4 del datasheet del IS42S16400J:
 *          -# Habilitar el reloj SDRAM (FMC_SDRAM_CMD_CLK_ENABLE).
 *          -# Esperar al menos 100 µs (se usa HAL_Delay de 1 ms para mayor margen).
 *          -# Precargar todos los bancos (FMC_SDRAM_CMD_PALL).
 *          -# Cuatro ciclos de auto-refresco (mínimo 2 según datasheet).
 *          -# Programar el registro de modo: burst de 2, tipo secuencial,
 *             latencia CAS 3, modo estándar, escritura en ráfaga simple.
 *          -# Programar la tasa de refresco (REFRESH_COUNT).
 *
 * @param[in,out] hsdram  Handle SDRAM de HAL.
 * @param[out]    Command Estructura de comando FMC reutilizada en cada paso de la secuencia.
 */
static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef* hsdram, FMC_SDRAM_CommandTypeDef* Command)
{
    __IO uint32_t tmpmrd =0;

    /* Paso 1: habilitar reloj */
    Command->CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
    Command->CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
    Command->AutoRefreshNumber      = 1;
    Command->ModeRegisterDefinition = 0;
    HAL_SDRAM_SendCommand(hsdram, Command, 0x1000U);

    /* Paso 2: esperar al menos 100 µs */
    HAL_Delay(1U);

    /* Paso 3: precargar todos los bancos (PALL) */
    Command->CommandMode            = FMC_SDRAM_CMD_PALL;
    Command->CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
    Command->AutoRefreshNumber      = 1;
    Command->ModeRegisterDefinition = 0;
    HAL_SDRAM_SendCommand(hsdram, Command, 0x1000U);


    /* Paso 4: Configurar el comando de auto-refresco.
     * El datasheet del IS42S16400J exige un mínimo de 2 ciclos de auto-refresh
     * durante la inicialización (§3.4). Se usan 4 para mayor margen de seguridad. */
    Command->CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    Command->CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
    Command->AutoRefreshNumber      = 4;
    Command->ModeRegisterDefinition = 0;
    HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

    /** Paso 5: Programar la memoria externa en modo registro */
    tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_2             |
                       SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      |
                       SDRAM_MODEREG_CAS_LATENCY_3              |
                       SDRAM_MODEREG_OPERATING_MODE_STANDARD    |
                       SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

    Command->CommandMode   = FMC_SDRAM_CMD_LOAD_MODE;
    Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
    Command->AutoRefreshNumber = 1;
    Command->ModeRegisterDefinition = tmpmrd;
    HAL_SDRAM_SendCommand(hsdram, Command, 0x1000U);
    
    /* Paso 6: configurar tasa de refresco */
    HAL_SDRAM_ProgramRefreshRate(hsdram, REFRESH_COUNT);
}
#endif /* HAL_SDRAM_MODULE_ENABLED */

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

    __disable_irq();
    Address  = TP_ADDR;
    Address &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
    if (HAL_I2C_Master_Transmit(ILI9341_hi2c, Address, buf, 1U, 1000U) != HAL_OK)
    {
        __enable_irq();
        return 0xAAU;
    }
    Address  = TP_ADDR;
    Address |= I2C_OAR1_ADD0;
    if (HAL_I2C_Master_Receive(ILI9341_hi2c, Address, &buf[0], 1U, 1000U) != HAL_OK)
    {
        __enable_irq();
        return 0xAAU;
    }
    tmp = buf[0];
    __enable_irq();

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

    __disable_irq();
    Address  = TP_ADDR;
    Address &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
    if (HAL_I2C_Master_Transmit(ILI9341_hi2c, Address, buf, 2U, 1000U) != HAL_OK)
    {
        __enable_irq();
        return ILI9341_ERROR;
    }
    __enable_irq();

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
    uint8_t TP_BufferRX[2] = { (uint8_t)RegisterAddr, (uint8_t)RegisterAddr };
    uint8_t buf[2]         = { (uint8_t)RegisterAddr, (uint8_t)RegisterAddr };

    if (!ILI9341_Initialized) { return 0xAAU; }

    __disable_irq();
    Address  = TP_ADDR;
    Address &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
    if (HAL_I2C_Master_Transmit(ILI9341_hi2c, Address, buf, 1U, 1000U) != HAL_OK)
    {
        __enable_irq();
        return 0xAAU;
    }
    Address  = TP_ADDR;
    Address |= I2C_OAR1_ADD0;
    if (HAL_I2C_Master_Receive(ILI9341_hi2c, Address, &TP_BufferRX[1], 1U, 1000U) != HAL_OK)
    {
        __enable_irq();
        return 0xAAU;
    }
    if (HAL_I2C_Master_Receive(ILI9341_hi2c, Address, &TP_BufferRX[0], 1U, 1000U) != HAL_OK)
    {
        __enable_irq();
        return 0xAAU;
    }
    __enable_irq();

    return (uint16_t)TP_BufferRX[0] | ((uint16_t)TP_BufferRX[1] << 8);
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

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

#if defined(HAL_SDRAM_MODULE_ENABLED) && defined(HAL_I2C_MODULE_ENABLED)
/**
 * @brief Inicializa la pantalla LCD ILI9341, la memoria SDRAM IS42S16400 y el controlador táctil STMPE811.
 *
 * @param[in] hspi   Puntero al handle SPI de HAL.
 * @param[in] hi2c   Puntero al handle I2C de HAL.
 * @param[in] hsdram (Solo con HAL_SDRAM_MODULE_ENABLED) Puntero al handle SDRAM de HAL
 *                   generado por STM32CubeMX. Pasar NULL deshabilita el frame buffer en SDRAM.
 *                   Cuando no es NULL, la librería reserva los primeros ILI9341_SDRAM_FB_SIZE
 *                   bytes de ILI9341_SDRAM_BASE como frame buffer interno (153 600 B).
 * @return ILI9341_Status_t
 *         - ILI9341_OK            si la inicialización fue exitosa.
 *         - ILI9341_INVALID_PARAM si @p hspi o @p hi2c es NULL.
 *         - ILI9341_ERROR         si una transmisión SPI falló durante la inicialización.
 */
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, SDRAM_HandleTypeDef* hsdram)
#elif defined(HAL_SDRAM_MODULE_ENABLED)
/**
 * @brief Inicializa la pantalla LCD ILI9341, la memoria SDRAM IS42S16400.
 *
 * @param[in] hspi   Puntero al handle SPI de HAL.
 * @param[in] hsdram (Solo con HAL_SDRAM_MODULE_ENABLED) Puntero al handle SDRAM de HAL
 *                   generado por STM32CubeMX. Pasar NULL deshabilita el frame buffer en SDRAM.
 *                   Cuando no es NULL, la librería reserva los primeros ILI9341_SDRAM_FB_SIZE
 *                   bytes de ILI9341_SDRAM_BASE como frame buffer interno (153 600 B).
 * @return ILI9341_Status_t
 *         - ILI9341_OK            si la inicialización fue exitosa.
 *         - ILI9341_INVALID_PARAM si @p hspi o @p hi2c es NULL.
 *         - ILI9341_ERROR         si una transmisión SPI falló durante la inicialización.
 */
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi, SDRAM_HandleTypeDef* hsdram)
#elif defined(HAL_I2C_MODULE_ENABLED)
/**
 * @brief Inicializa la pantalla LCD ILI9341 y el controlador táctil STMPE811.
 *
 * @param[in] hspi   Puntero al handle SPI de HAL.
 * @param[in] hi2c   Puntero al handle I2C de HAL.
 * @return ILI9341_Status_t
 *         - ILI9341_OK            si la inicialización fue exitosa.
 *         - ILI9341_INVALID_PARAM si @p hspi o @p hi2c es NULL.
 *         - ILI9341_ERROR         si una transmisión SPI falló durante la inicialización.
 */
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c)
#else
/**
 * @brief Inicializa la pantalla LCD ILI9341.
 *
 * @param[in] hspi   Puntero al handle SPI de HAL.
 * 
 * @return ILI9341_Status_t
 *         - ILI9341_OK            si la inicialización fue exitosa.
 *         - ILI9341_INVALID_PARAM si @p hspi o @p hi2c es NULL.
 *         - ILI9341_ERROR         si una transmisión SPI falló durante la inicialización.
 */
ILI9341_Status_t ILI9341_Init(SPI_HandleTypeDef* hspi)
#endif
{
    ILI9341_Status_t st;

    if (hspi == NULL) { return ILI9341_INVALID_PARAM; }
#ifdef HAL_I2C_MODULE_ENABLED
    if (hi2c == NULL) { return ILI9341_INVALID_PARAM; }
#endif

    ILI9341_hspi        = hspi;
#ifdef HAL_I2C_MODULE_ENABLED
    ILI9341_hi2c        = hi2c;
#endif
    ILI9341_Initialized = 0U;

#ifdef HAL_SDRAM_MODULE_ENABLED
    ILI9341_hsdram     = NULL;
    ILI9341_framebuffer = NULL;
#endif

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

    SPI_ILI9341_BaudRateUp();

#ifdef HAL_SDRAM_MODULE_ENABLED
    ILI9341_hsdram = hsdram;
    if (hsdram != NULL)
    {
        FMC_SDRAM_CommandTypeDef sdramCmd = {0};
        SDRAM_Initialization_Sequence(hsdram, &sdramCmd);
        ILI9341_framebuffer = (uint32_t*)ILI9341_SDRAM_BASE;
        memset(ILI9341_framebuffer, 0, ILI9341_SDRAM_FB_SIZE);
    }
    else
    {
        ILI9341_framebuffer = NULL;
    }
#endif

    ILI9341_Initialized = 1U;

    return ILI9341_OK;
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
    ILI9341_Status_t st;
    unsigned int n;
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFFU);

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    st = ILI9341_SetCursorPosition(0U, 0U, ILI9341_Opts.width - 1U, ILI9341_Opts.height - 1U);
    st = st ? st : ILI9341_SendCommand(ILI9341_GRAM);
    if (st != ILI9341_OK) { return st; }

    for (n = 0U; n < ILI9341_PIXEL; n++)
    {
        st = ILI9341_SendData(hi);
        if (st != ILI9341_OK) { return st; }
        st = ILI9341_SendData(lo);
        if (st != ILI9341_OK) { return st; }
    }
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

    dx  = (x0 < x1) ? (x1 - x0) : (x0 - x1);
    dy  = (y0 < y1) ? (y1 - y0) : (y0 - y1);
    sx  = (x0 < x1) ?  1 : -1;
    sy  = (y0 < y1) ?  1 : -1;
    err = ((dx > dy) ? dx : -dy) / 2;

    while (1)
    {
        st = ILI9341_DrawPixel(x0, y0, color);
        if (st != ILI9341_OK) { return st; }
        if (x0 == x1 && y0 == y1) { break; }
        e2 = err;
        if (e2 > -dx) { err -= dy; x0 += sx; }
        if (e2 <  dy) { err += dx; y0 += sy; }
    }
    return ILI9341_OK;
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
    ILI9341_Status_t st;
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }
    for (; x0 < x1; x0++)
    {
        st = ILI9341_DrawLine(x0, y0, x0, y1, color);
        if (st != ILI9341_OK) { return st; }
    }
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

    st  = ILI9341_DrawPixel(x0,     y0 + r, color);
    st  = st ? st : ILI9341_DrawPixel(x0,     y0 - r, color);
    st  = st ? st : ILI9341_DrawPixel(x0 + r, y0,     color);
    st  = st ? st : ILI9341_DrawPixel(x0 - r, y0,     color);
    if (st != ILI9341_OK) { return st; }

    while (x < y)
    {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++;
        ddF_x += 2;
        f += ddF_x;

        st  = ILI9341_DrawPixel(x0 + x, y0 + y, color);
        st  = st ? st : ILI9341_DrawPixel(x0 - x, y0 + y, color);
        st  = st ? st : ILI9341_DrawPixel(x0 + x, y0 - y, color);
        st  = st ? st : ILI9341_DrawPixel(x0 - x, y0 - y, color);
        st  = st ? st : ILI9341_DrawPixel(x0 + y, y0 + x, color);
        st  = st ? st : ILI9341_DrawPixel(x0 - y, y0 + x, color);
        st  = st ? st : ILI9341_DrawPixel(x0 + y, y0 - x, color);
        st  = st ? st : ILI9341_DrawPixel(x0 - y, y0 - x, color);
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

    st  = ILI9341_DrawPixel(x0,     y0 + r, color);
    st  = st ? st : ILI9341_DrawPixel(x0,     y0 - r, color);
    st  = st ? st : ILI9341_DrawPixel(x0 + r, y0,     color);
    st  = st ? st : ILI9341_DrawPixel(x0 - r, y0,     color);
    st  = st ? st : ILI9341_DrawLine(x0 - r, y0, x0 + r, y0, color);
    if (st != ILI9341_OK) { return st; }

    while (x < y)
    {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++;
        ddF_x += 2;
        f += ddF_x;

        st  = ILI9341_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
        st  = st ? st : ILI9341_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);
        st  = st ? st : ILI9341_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
        st  = st ? st : ILI9341_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
        if (st != ILI9341_OK) { return st; }
    }
    return ILI9341_OK;
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
 *         - ILI9341_ERROR           si falla la transmisión SPI.
 */
ILI9341_Status_t ILI9341_Putc(uint16_t x, uint16_t y, char c, LCD_FontDef_t* font, uint16_t foreground, uint16_t background)
{
    ILI9341_Status_t st;
    uint32_t i, b, j;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    ILI9341_x = x;
    ILI9341_y = y;
    if ((ILI9341_x + font->FontWidth) > ILI9341_Opts.width)
    {
        ILI9341_y += font->FontHeight;
        ILI9341_x  = 0U;
    }

    for (i = 0U; i < font->FontHeight; i++)
    {
        b = font->data[(c - 32) * font->FontHeight + i];
        for (j = 0U; j < font->FontWidth; j++)
        {
            if ((b << j) & 0x8000U)
                st = ILI9341_DrawPixel(ILI9341_x + j, ILI9341_y + i, foreground);
            else
                st = ILI9341_DrawPixel(ILI9341_x + j, ILI9341_y + i, background);
            if (st != ILI9341_OK) { return st; }
        }
    }
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
    *height = font->FontHeight;
    while (*str++) { w += font->FontWidth; }
    *width = w;
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
    const uint32_t Timeout   = 5000U;
    uint32_t       pix;
    uint8_t        aux8;
    uint32_t       tickstart;

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

    for (uint32_t k = 0U; k < IMG_TOTAL_BUF32; k++)
    {
        pix = image[k];

        /* Primer píxel (word bajo) — byte alto */
        aux8 = (uint8_t)(pix >> 8);
        ILI9341_hspi->Instance->DR = aux8;
        tickstart = HAL_GetTick();
        while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_TXE) == RESET)
        {
            if (Timeout != HAL_MAX_DELAY)
            {
                if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
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
        }

        /* Primer píxel (word bajo) — byte bajo */
        aux8 = (uint8_t)(pix & 0x000000FFU);
        ILI9341_hspi->Instance->DR = aux8;
        tickstart = HAL_GetTick();
        while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_TXE) == RESET)
        {
            if (Timeout != HAL_MAX_DELAY)
            {
                if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
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
        }

        /* Segundo píxel (word alto) — byte alto */
        aux8 = (uint8_t)(pix >> 24);
        ILI9341_hspi->Instance->DR = aux8;
        tickstart = HAL_GetTick();
        while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_TXE) == RESET)
        {
            if (Timeout != HAL_MAX_DELAY)
            {
                if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
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
        }

        /* Segundo píxel (word alto) — byte bajo */
        aux8 = (uint8_t)(pix >> 16);
        ILI9341_hspi->Instance->DR = aux8;
        tickstart = HAL_GetTick();
        while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_TXE) == RESET)
        {
            if (Timeout != HAL_MAX_DELAY)
            {
                if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
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
        }
    }

    ILI9341_WRX_RESET;
    ILI9341_CS_SET;

    if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        ILI9341_hspi->Instance->CR1 |= SPI_CR1_CRCNEXT;
    }

    tickstart = HAL_GetTick();
    while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_TXE) == RESET)
    {
        if (Timeout != HAL_MAX_DELAY)
        {
            if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
            {
                __HAL_SPI_DISABLE_IT(ILI9341_hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
                __HAL_SPI_DISABLE(ILI9341_hspi);
                if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) { SPI_RESET_CRC(ILI9341_hspi); }
                ILI9341_hspi->State       = HAL_SPI_STATE_READY;
                ILI9341_hspi->ErrorCode  |= HAL_SPI_ERROR_FLAG;
                __HAL_UNLOCK(ILI9341_hspi);
                ILI9341_WRX_RESET;
                ILI9341_CS_SET;
                return ILI9341_TIMEOUT;
            }
        }
    }

    tickstart = HAL_GetTick();
    while (__HAL_SPI_GET_FLAG(ILI9341_hspi, SPI_FLAG_BSY) != RESET)
    {
        if (Timeout != HAL_MAX_DELAY)
        {
            if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
            {
                __HAL_SPI_DISABLE_IT(ILI9341_hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
                __HAL_SPI_DISABLE(ILI9341_hspi);
                if (ILI9341_hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) { SPI_RESET_CRC(ILI9341_hspi); }
                ILI9341_hspi->State       = HAL_SPI_STATE_READY;
                ILI9341_hspi->ErrorCode  |= HAL_SPI_ERROR_FLAG;
                __HAL_UNLOCK(ILI9341_hspi);
                ILI9341_WRX_RESET;
                ILI9341_CS_SET;
                return ILI9341_TIMEOUT;
            }
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
 * @brief Escribe un píxel en un frame buffer fuera de pantalla.
 *
 * @param[in]     x      Coordenada X del píxel.
 * @param[in]     y      Coordenada Y del píxel.
 * @param[in]     color  Color del píxel en formato RGB565.
 * @param[in,out] image  Frame buffer (IMG_TOTAL_BUF32 palabras uint32_t).
 */
void ILI9341_DrawPixel_ImageBuffer(uint16_t x, uint16_t y, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    uint32_t buf, dir16, dir32, aux, aux2;

    if (x >= ILI9341_WIDTH || y >= ILI9341_HEIGHT) { return; }

    dir16 = (uint32_t)y;
    aux   = (uint32_t)x;
    dir16 = ILI9341_WIDTH * dir16 + aux;
    dir32 = dir16 / 2U;

    if (dir32 >= IMG_TOTAL_BUF32) { return; }

    aux2  = dir16 - dir32 * 2U;
    buf   = image[dir32];
    aux   = (uint32_t)color;

    if (aux2 != 0U) { buf = (buf & 0x0000FFFFU) | (aux << 16); }
    else            { buf = (buf & 0xFFFF0000U) |  aux;         }

    image[dir32] = buf;
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
void ILI9341_Putc_ImageBuffer(uint16_t x, uint16_t y, char c, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32])
{
    uint32_t i, b, j;

    ILI9341_x = x;
    ILI9341_y = y;
    if ((ILI9341_x + font->FontWidth) > ILI9341_WIDTH)
    {
        ILI9341_y += font->FontHeight;
        ILI9341_x  = 0U;
    }

    for (i = 0U; i < font->FontHeight; i++)
    {
        b = font->data[(c - 32) * font->FontHeight + i];
        for (j = 0U; j < font->FontWidth; j++)
        {
            if ((b << j) & 0x8000U)
            {
                ILI9341_DrawPixel_ImageBuffer(ILI9341_x + j, ILI9341_y + i, foreground, image);
            }
        }
    }
    ILI9341_x += font->FontWidth;
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
void ILI9341_Puts_ImageBuffer(uint16_t x, uint16_t y, char* str, LCD_FontDef_t* font, uint16_t foreground, uint32_t image[IMG_TOTAL_BUF32])
{
    uint16_t startX = x;

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
        ILI9341_Putc_ImageBuffer(ILI9341_x, ILI9341_y, *str++, font, foreground, image);
    }
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
void ILI9341_DrawLine_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    int16_t dx, dy, sx, sy, err, e2;

    if (x0 >= ILI9341_WIDTH)  { x0 = ILI9341_WIDTH  - 1U; }
    if (x1 >= ILI9341_WIDTH)  { x1 = ILI9341_WIDTH  - 1U; }
    if (y0 >= ILI9341_HEIGHT) { y0 = ILI9341_HEIGHT - 1U; }
    if (y1 >= ILI9341_HEIGHT) { y1 = ILI9341_HEIGHT - 1U; }

    dx  = (x0 < x1) ? (x1 - x0) : (x0 - x1);
    dy  = (y0 < y1) ? (y1 - y0) : (y0 - y1);
    sx  = (x0 < x1) ?  1 : -1;
    sy  = (y0 < y1) ?  1 : -1;
    err = ((dx > dy) ? dx : -dy) / 2;

    while (1)
    {
        ILI9341_DrawPixel_ImageBuffer(x0, y0, color, image);
        if (x0 == x1 && y0 == y1) { break; }
        e2 = err;
        if (e2 > -dx) { err -= dy; x0 += sx; }
        if (e2 <  dy) { err += dx; y0 += sy; }
    }
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
void ILI9341_DrawRectangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    ILI9341_DrawLine_ImageBuffer(x0, y0, x1, y0, color, image); /* Superior  */
    ILI9341_DrawLine_ImageBuffer(x0, y0, x0, y1, color, image); /* Izquierda */
    ILI9341_DrawLine_ImageBuffer(x1, y0, x1, y1, color, image); /* Derecha   */
    ILI9341_DrawLine_ImageBuffer(x0, y1, x1, y1, color, image); /* Inferior  */
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
void ILI9341_DrawFilledRectangle_ImageBuffer(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint32_t image[IMG_TOTAL_BUF32])
{
    for (; x0 < x1; x0++)
    {
        ILI9341_DrawLine_ImageBuffer(x0, y0, x0, y1, color, image);
    }
}

// ============================================================================
// FUNCIONES PÚBLICAS — Frame buffer SDRAM
// ============================================================================

#ifdef HAL_SDRAM_MODULE_ENABLED

/**
 * @brief Transfiere el frame buffer interno (SDRAM) a la pantalla LCD mediante SPI.
 *
 * @details Equivalente a llamar ILI9341_DisplayImage() con el puntero interno
 *          al frame buffer en SDRAM. Requiere que ILI9341_Init() haya sido
 *          invocado con un handle SDRAM válido.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si la transferencia fue exitosa.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_INVALID_PARAM   si el frame buffer SDRAM no está habilitado (hsdram era NULL).
 *         - ILI9341_TIMEOUT         si el bus SPI se bloqueó.
 *         - ILI9341_ERROR           si el periférico SPI estaba ocupado.
 */
ILI9341_Status_t ILI9341_Flush(void)
{
    if (!ILI9341_Initialized)   { return ILI9341_NOT_INITIALIZED; }
    if (ILI9341_hsdram == NULL) { return ILI9341_INVALID_PARAM;   }
    return ILI9341_DisplayImage(ILI9341_framebuffer);
}

/**
 * @brief Retorna el puntero al frame buffer interno ubicado en SDRAM.
 *
 * @details El buffer tiene formato IMG_TOTAL_BUF32 palabras uint32_t (dos píxeles
 *          RGB565 empaquetados por palabra), compatible con todas las funciones
 *          ILI9341_*_ImageBuffer(). Retorna NULL si SDRAM no fue habilitada
 *          o si el driver no ha sido inicializado.
 *
 * @return Puntero al frame buffer en SDRAM, o NULL si no está disponible.
 */
uint32_t* ILI9341_GetFrameBuffer(void)
{
    if (!ILI9341_Initialized) { return NULL; }
    return ILI9341_framebuffer;
}

#endif /* HAL_SDRAM_MODULE_ENABLED */

/**
 * @brief Desinicializa el driver LCD y libera los recursos periféricos.
 *
 * @details Marca el driver como no inicializado y pone a NULL los handles
 *          internos de SPI e I2C. Si HAL_SDRAM_MODULE_ENABLED está definido y
 *          la SDRAM fue habilitada en Init(), también llama a HAL_SDRAM_DeInit()
 *          y limpia el puntero al frame buffer. Tras esta llamada es necesario
 *          invocar ILI9341_Init() antes de usar cualquier otra función.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si la desinicialización fue exitosa.
 *         - ILI9341_NOT_INITIALIZED si el driver no estaba inicializado.
 */
ILI9341_Status_t ILI9341_DeInit(void)
{
    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    ILI9341_Initialized = 0U;

#ifdef HAL_SDRAM_MODULE_ENABLED
    if (ILI9341_hsdram != NULL)
    {
        HAL_SDRAM_DeInit(ILI9341_hsdram);
        ILI9341_hsdram      = NULL;
        ILI9341_framebuffer = NULL;
    }
#endif

    ILI9341_hspi = NULL;
#ifdef HAL_I2C_MODULE_ENABLED
    ILI9341_hi2c = NULL;
#endif

    return ILI9341_OK;
}

// ============================================================================
// FUNCIONES PÚBLICAS — Touch panel (STMPE811)
// ============================================================================

#ifdef HAL_I2C_MODULE_ENABLED
/**
 * @brief Configura el controlador del panel táctil STMPE811.
 *
 * @return ILI9341_Status_t
 *         - ILI9341_OK              si el dispositivo fue detectado y configurado.
 *         - ILI9341_NOT_INITIALIZED si el driver no ha sido inicializado.
 *         - ILI9341_ERROR           si el ID del chip no coincidió con STMPE811_ID.
 */
ILI9341_Status_t ILI9341_TP_Config(void)
{
    uint16_t tmp = 0U;

    if (!ILI9341_Initialized) { return ILI9341_NOT_INITIALIZED; }

    tmp  = (uint16_t)ILI9341_TP_ReadDeviceRegister(0U) << 8;
    tmp |= (uint16_t)ILI9341_TP_ReadDeviceRegister(1U);
    if (tmp != (uint16_t)STMPE811_ID) { return ILI9341_ERROR; }

    ILI9341_TP_Reset();
    ILI9341_TP_FnctCmd(TP_ADC_FCT, ENABLE);
    ILI9341_TP_FnctCmd(TP_TP_FCT,  ENABLE);
    ILI9341_TP_WriteDeviceRegister(TP_REG_ADC_CTRL1, 0x49U);
    ILI9341_Delay(200U);
    ILI9341_TP_WriteDeviceRegister(TP_REG_ADC_CTRL2,    0x01U);
    ILI9341_TP_IOAFConfig((uint8_t)TOUCH_IO_ALL, DISABLE);
    ILI9341_TP_WriteDeviceRegister(TP_REG_TP_CFG,       0x9AU);
    ILI9341_TP_WriteDeviceRegister(TP_REG_FIFO_TH,      0x01U);
    ILI9341_TP_WriteDeviceRegister(TP_REG_FIFO_STA,     0x01U);
    ILI9341_TP_WriteDeviceRegister(TP_REG_FIFO_STA,     0x00U);
    ILI9341_TP_WriteDeviceRegister(TP_REG_TP_FRACT_XYZ, 0x01U);
    ILI9341_TP_WriteDeviceRegister(TP_REG_TP_I_DRIVE,   0x01U);
    ILI9341_TP_WriteDeviceRegister(TP_REG_TP_CTRL,      0x03U);
    ILI9341_TP_WriteDeviceRegister(TP_REG_INT_STA,      0xFFU);

    TP_State.TouchDetected = TP_State.X = TP_State.Y = TP_State.Z = 0U;

    return ILI9341_OK;
}

/**
 * @brief Lee el estado actual del panel táctil (coordenadas y detección de toque).
 *
 * @return Puntero a la estructura TP_STATE interna con valores actualizados,
 *         o NULL si el driver no ha sido inicializado.
 */
TP_STATE* ILI9341_TP_GetState(void)
{
    uint32_t xDiff, yDiff, x, y;
    static uint32_t _x = 0U, _y = 0U;

    if (!ILI9341_Initialized) { return NULL; }

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