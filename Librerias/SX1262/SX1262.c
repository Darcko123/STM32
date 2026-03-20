/**
 * @file SX1262.h
 * @brief Librería para la gestión de un módulo LoRa
 * 
 * Esta librería permite inicializar y controlar
 * 
 * @author Daniel Ruiz
 * @date March 16, 2026
 * @version 1.0
 */

#include "SX1262.h"

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static SPI_HandleTypeDef*   SX1262_hspi         = NULL;         /**< Manejador de la interfaz SPI utilizada para cominucarse con el módulo*/
static GPIO_TypeDef*        NSS_GPIO_Port       = NULL;         /**< Puerto GPIO del pin de datos del SX1262 */
static uint16_t             NSS_GPIO_Pin        = 0;            /**< Pin GPIO del pin de datos del SX1262 */
static GPIO_TypeDef*        BUSY_GPIO_Port      = NULL;
static uint16_t             BUSY_GPIO_Pin       = 0;
static GPIO_TypeDef*        DIO1_GPIO_Port      = NULL;
static uint16_t             DIO1_GPIO_Pin       = 0;
static GPIO_TypeDef*        RST_GPIO_Port       = NULL;
static uint16_t             RST_GPIO_Pin        = 0;
static uint8_t              SX1262_Initialized  = 0;     /**< Bandera para verificar si el módulo está inicializado */

// ============================================================================
// FUNCIONES PRIVADAS - ABSTRACCIÓN SPI
// ============================================================================
static SX1262_Status_t SX1262_WaitBusy(void)
{
    uint32_t tickstart = HAL_GetTick();
    while (HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_GPIO_Pin) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - tickstart) > SX1262_MAX_BUSY_TIMEOUT)
        {
            return SX1262_TIMEOUT;
        }
    }
    return SX1262_OK;
}

static SX1262_Status_t SX1262_WriteCommand(uint8_t cmd, uint8_t* buffer, uint16_t size)
{
    if (SX1262_WaitBusy() != SX1262_OK) return SX1262_TIMEOUT;

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SX1262_hspi, &cmd, 1, HAL_MAX_DELAY);
    if (size > 0 && buffer != NULL)
    {
        if(HAL_SPI_Transmit(SX1262_hspi, buffer, size, HAL_MAX_DELAY) != HAL_OK)
        {
            return SX1262_ERROR;
        }
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    return SX1262_OK;
}

static SX1262_Status_t SX1262_ReadCommand(uint8_t cmd, uint8_t* buffer, uint16_t size)
{
    if (SX1262_WaitBusy() != SX1262_OK) return SX1262_TIMEOUT;

    uint8_t nop = 0x00;
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SX1262_hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(SX1262_hspi, &nop, 1, HAL_MAX_DELAY); // El SX1262 retorna un STATUS primero
    if (size > 0 && buffer != NULL)
    {
        if(HAL_SPI_Receive(SX1262_hspi, buffer, size, HAL_MAX_DELAY) != HAL_OK)
        {
            return SX1262_ERROR;
        }
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    return SX1262_OK;
}

static SX1262_Status_t SX1262_WriteBuffer(uint8_t offset, uint8_t* data, uint8_t length)
{
    if (SX1262_WaitBusy() != SX1262_OK) return SX1262_TIMEOUT;

    uint8_t cmd[2] = {SX126X_CMD_WRITE_BUFFER, offset};
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(SX1262_hspi, cmd, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        return SX1262_ERROR;
    }
    if(HAL_SPI_Transmit(SX1262_hspi, data, length, HAL_MAX_DELAY) != HAL_OK)
    {
        return SX1262_OK;
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);
    
    return SX1262_OK;
}

static SX1262_Status_t SX1262_ReadBuffer(uint8_t offset, uint8_t* data, uint8_t length)
{
    if (SX1262_WaitBusy() != SX1262_OK) return SX1262_TIMEOUT;

    uint8_t cmd[3] = {SX126X_CMD_READ_BUFFER, offset, 0x00}; // NOP byte automatically handled here
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(SX1262_hspi, cmd, 3, HAL_MAX_DELAY) != HAL_OK)
    {
        return SX1262_ERROR;
    }
    if(HAL_SPI_Receive(SX1262_hspi, data, length, HAL_MAX_DELAY) != HAL_OK)
    {
        return SX1262_ERROR;
    }
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_GPIO_Pin, GPIO_PIN_SET);

    return SX1262_OK;
}

static void SX1262_Reset()
{
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_GPIO_Pin, GPIO_PIN_SET);
    HAL_Delay(10); // Permitir inicialización
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el módulo LoRa.
 * 
 * @param hspi Puntero al manejador de la interfaz SPI utilizada para comunicarse con el módulo.
 * @param GPIOx Puerto GPIO del pin CS del SX1262.
 * @param GPIO_PIN Pin GPIO del pin CS del SX1262.
 * 
 * @return SX1262_Status_t Estado de la operación
 */
SX1262_Status_t SX1262_Init(
	SPI_HandleTypeDef* hspi,
    GPIO_TypeDef*      NSS_Port,
    uint16_t           NSS_Pin,
    GPIO_TypeDef*      BUSY_Port,
    uint16_t           BUSY_Pin,
    GPIO_TypeDef*      DIO1_Port,
    uint16_t           DIO1_Pin,
    GPIO_TypeDef*      RST_Port,
    uint16_t           RST_Pin
)
{
    SX1262_Status_t status;
    
    // Validar parámetros de entrada
    if(hspi == NULL || NSS_Port  == NULL || NSS_Pin == 0
                    || BUSY_Port == NULL || BUSY_Pin == 0
                    || DIO1_Port == NULL || DIO1_Pin == 0
                    || RST_Port  == NULL || RST_Pin == 0 
    )
    {
        return SX1262_ERROR;
    }
    
    // Almacenar configuración para uso en funciones posteriores
    SX1262_hspi = hspi;
    NSS_GPIO_Port = NSS_Port;
    NSS_GPIO_Pin = NSS_Pin;
    BUSY_GPIO_Port = BUSY_Port;
    BUSY_GPIO_Pin = BUSY_Pin;
    DIO1_GPIO_Port = DIO1_Port;
    DIO1_GPIO_Pin = DIO1_Pin;
    RST_GPIO_Port = RST_Port;
    RST_GPIO_Pin = RST_Pin;
    SX1262_Initialized = 0;    // Marcar como no inicializada hasta que se termine el proceso

    SX1262_Reset();
    uint8_t buf[8];

    // Marcar como inicializada
    SX1262_Initialized = 1;

    return SX1262_OK;
}