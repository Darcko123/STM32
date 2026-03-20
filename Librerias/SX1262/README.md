# SX1262

## Pinout Configuration

| Parametro | Valor Recomendado | Notas |
|-----------|-------------------|-------|
| **Mode** | Full-Duplex Master | El STM32 es el que controla el bus |
| **Hardarware** | Disable | Para controlar el CS por software con un GPIO |
| **Data Size** | 8 Bit | Comunicación por bytes con el SX1262 |
| **First Byte** | MSB First | Estándar |
| **Prescaler** | 16 o 32 | Empieza lentro para grantizar la comunicación | 
| **CPOL** | Low | Modo 0 |
| **CPHA** | 1 Edge | Modo 0 |
| **NSS** | Software | Para manejar el pin CS manualmente |

```C
void sx1262_write_command(uint8_t op_code, uint8_t* data, uint16 len)
{
    // 1. Activar el chip seleccionando el CS (GPIO a LOW)
    HAL_GPIO_WritePin(CS_PIN_PORT, CS_Pin, GPIO_PIN_RESET);
    
    // 2. Enviar el byte de comando (operación)
    HAL_SPI_Transmit(&hspi1, &op_code, 1, HAL_MAX_DELAY);

    // 3. Si hay datos, enviarlos
    if(len > 0)
    {
        HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY);
    }

    // 4. Desactivar el chip (CS a HIGH)
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

```