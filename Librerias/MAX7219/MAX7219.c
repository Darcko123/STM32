/*
 * MAX7219.c
 *
 * @brief Implementación de la librería para el control de una matriz de LEDs usando MAX7219 en STM32 mediante SPI.
 *
 * @author Daniel Ruiz
 * @date Jan 4, 2025
 * @version 1.0
 */

#include "MAX7219.h"

extern SPI_HandleTypeDef hspi1;

uint8_t bufferCol[NUM_DEV*8] = {0};  // Ahora solo se define aquí

/**
 * @brief Inicializa la matriz de LEDs.
 * 
 * Configura los registros del MAX7219 para preparar la matriz de LEDs para su uso.
 */
void MAX7219_Init(void)
{
    max7219_cmd(0x09, 0);    // Sin decodificación (modo gráfico)
    max7219_cmd(0x0A, 0x01); // Intensidad de brillo en 3/32
    max7219_cmd(0x0B, 0x07); // Escaneo de las 8 columnas
    max7219_cmd(0x0C, 0x01); // Modo operación normal
    max7219_cmd(0x0F, 0);    // Deshabilitar test de display
}

/**
 * @brief Escribe un byte de datos en una fila específica de la matriz de LEDs.
 * 
 * @param row La fila en la que se escribirá el dato (1-8).
 * @param data El byte de datos que se escribirá en la fila.
 */
void max7219_write(int row, uint8_t data)
{
    int devTarget = (row - 1) / 8;  // Determina en qué MAX7219 se escribirá el dato
    int offset = devTarget * 8;     // Calcula el offset de la fila en el buffer
    uint16_t writeData = 0;

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);  // Habilitar el esclavo
    for (int dev = 0; dev < NUM_DEV; dev++)
    {
        if (dev == devTarget)
        {
            writeData = ((row - offset) << 8) | data;  // Enviar número de fila y datos
            HAL_SPI_Transmit(&hspi1, (uint8_t *)&writeData, 1, 1000);
        }
        else
        {
            writeData = 0;  // No operación en los otros dispositivos
            HAL_SPI_Transmit(&hspi1, (uint8_t *)&writeData, 1, 1000);
        }
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);  // Deshabilitar el esclavo
}

/**
 * @brief Envía un comando al MAX7219.
 * 
 * @param Addr Dirección del registro del MAX7219.
 * @param data Dato que se escribirá en el registro.
 */
void max7219_cmd(uint8_t Addr, uint8_t data)
{
    uint16_t writeData = (Addr << 8) | data;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
    for (int i = 0; i < NUM_DEV; i++)
    {
        HAL_SPI_Transmit(&hspi1, (uint8_t *)&writeData, 1, 100);
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
}

/**
 * @brief Limpia el buffer de la matriz de LEDs y actualiza la pantalla.
 */
void flushBuffer(void)
{
    uint8_t bufferRow[NUM_DEV * 8] = {0};

    // Convertir columnas en filas
    for (int i = 0; i < NUM_DEV * 8; i++)
    {
        int dev = i / 8;
        for (int j = 0; j < 8; j++)
        {
            if (bufferCol[i] & (1 << j))
            {
                bufferRow[j + (8 * dev)] |= (1 << (7 - (i - (8 * dev))));
            }
        }
    }

    for (int row = 1; row <= (NUM_DEV * 8); row++)
    {
        max7219_write(row, bufferRow[row - 1]);
    }
}

/**
 * @brief Apaga todos los LEDs de la matriz.
 */
void MAX7219_clearDisplay(void)
{
    for (int i = 0; i < NUM_DEV * 8; i++)
    {
        bufferCol[i] = 0;
    }
    flushBuffer();
}

/**
 * @brief Desplaza el contenido de la matriz hacia la izquierda.
 */
void shiftLeft(void)
{
    for (int cnt = NUM_DEV * 8 - 2; cnt >= 0; cnt--)
    {
        bufferCol[cnt + 1] = bufferCol[cnt];
    }
    bufferCol[0] = 0;
    flushBuffer();
}

/**
 * @brief Desplaza el contenido de la matriz hacia la derecha.
 */
void shiftRight(void)
{
    for (int cnt = 0; cnt < NUM_DEV * 8 - 1; cnt++)
    {
        bufferCol[cnt] = bufferCol[cnt + 1];
    }
    bufferCol[NUM_DEV * 8 - 1] = 0;
    flushBuffer();
}

/**
 * @brief Desplaza un carácter a través de la matriz de LEDs con un retardo específico.
 * 
 * Convierte un carácter en su representación de bits y lo desplaza a través de la matriz
 * de LEDs, mostrando el efecto de desplazamiento con un retardo configurable.
 * 
 * @param ch Carácter que se desplazará.
 * @param delay Retardo entre cada desplazamiento en milisegundos.
 */
void shiftchar(uint8_t ch, int delay)
{
    int indx = 0;
    for (int i = 0; i < FONT_WIDTH - 1; i++)  // Iterar sobre los bytes de la fuente de caracteres
    {
        uint8_t data = 0;
        
        // Invertir el orden de los bits
        for (int j = 7; j >= 0; j--)  // Extraer bits de cada byte
        {
            if ((MAX7219_Dot_Matrix_font[ch][indx]) & (1 << j))  // Si el bit es 1
            {
                data |= (1 << (7 - j));  // Escribirlo desde el bit 0 de 'data'
            }
        }
        
        bufferCol[0] = data;  // Almacenar el byte modificado en la primera posición del buffer
        flushBuffer();  // Actualizar la pantalla con el nuevo dato
        shiftLeft();  // Desplazar la imagen a la izquierda
        indx++;
        HAL_Delay(delay);  // Esperar el tiempo especificado antes del siguiente desplazamiento
    }
}

/**
 * @brief Muestra una cadena de caracteres en la matriz de LEDs con desplazamiento.
 * 
 * @param str Cadena de caracteres a mostrar.
 * @param delay Tiempo de retardo entre cada desplazamiento.
 */
void MAX7219_scrollString(char *str, int delay)
{
    while (*str)
    {
        shiftchar(*str, delay);
        str++;
    }
}

/**
 * @brief Muestra una cadena de caracteres en la matriz de LEDs.
 * 
 * Esta función toma una cadena de texto y la convierte en una representación de bytes
 * para visualizarla en la matriz de LEDs. Cada carácter se almacena en el buffer de
 * columnas y se muestra en la pantalla.
 * 
 * @param str Puntero a la cadena de caracteres que se mostrará.
 */
void MAX7219_printString(const char *str)
{
    int strindx = 0; // Índice de caracteres de la cadena de entrada
    for (int k = NUM_DEV * 8 - 1; k >= 0; )
    {
        int indx = 0; // Índice dentro de la fuente de caracteres
        for (int i = 0; i < FONT_WIDTH - 1; i++)  // Iterar sobre los bytes de la fuente
        {
            uint8_t data = 0;
            
            // Invertir el orden de los bits
            for (int j = 7; j >= 0; j--)  // Extraer bits de cada byte
            {
                if ((MAX7219_Dot_Matrix_font[str[strindx]][indx]) & (1 << j))  // Si el bit es 1
                {
                    data |= (1 << (7 - j));  // Escribirlo desde el bit 0 de 'data'
                }
            }
            
            bufferCol[k--] = data;  // Almacenar el byte modificado en el buffer de columnas
            indx++;
        }
        strindx++;  // Moverse al siguiente carácter en la cadena
        if (str[strindx] == '\0') break; // Importante: detenerse al final de la cadena
    }
    flushBuffer(); // Actualizar la pantalla con el nuevo contenido
}

/**
 * @brief Conjunto de fuentes para la pantalla de matriz de puntos MAX7219.
 *
 * Este array contiene el conjunto de fuentes de 8x8 para la pantalla de matriz de puntos MAX7219.
 * Cada carácter está representado por un array de 8 bytes, donde cada byte representa
 * una fila del carácter.
 */
const uint8_t MAX7219_Dot_Matrix_font [256] [8] = {
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x00
  { 0x7E, 0x81, 0x95, 0xB1, 0xB1, 0x95, 0x81, 0x7E }, // 0x01
  { 0x7E, 0xFF, 0xEB, 0xCF, 0xCF, 0xEB, 0xFF, 0x7E }, // 0x02
  { 0x0E, 0x1F, 0x3F, 0x7E, 0x3F, 0x1F, 0x0E, 0x00 }, // 0x03
  { 0x08, 0x1C, 0x3E, 0x7F, 0x3E, 0x1C, 0x08, 0x00 }, // 0x04
  { 0x18, 0xBA, 0xFF, 0xFF, 0xFF, 0xBA, 0x18, 0x00 }, // 0x05
  { 0x10, 0xB8, 0xFC, 0xFF, 0xFC, 0xB8, 0x10, 0x00 }, // 0x06
  { 0x00, 0x00, 0x18, 0x3C, 0x3C, 0x18, 0x00, 0x00 }, // 0x07
  { 0xFF, 0xFF, 0xE7, 0xC3, 0xC3, 0xE7, 0xFF, 0xFF }, // 0x08
  { 0x00, 0x3C, 0x66, 0x42, 0x42, 0x66, 0x3C, 0x00 }, // 0x09
  { 0xFF, 0xC3, 0x99, 0xBD, 0xBD, 0x99, 0xC3, 0xFF }, // 0x0A
  { 0x70, 0xF8, 0x88, 0x88, 0xFD, 0x7F, 0x07, 0x0F }, // 0x0B
  { 0x00, 0x4E, 0x5F, 0xF1, 0xF1, 0x5F, 0x4E, 0x00 }, // 0x0C
  { 0xC0, 0xE0, 0xFF, 0x7F, 0x05, 0x05, 0x07, 0x07 }, // 0x0D
  { 0xC0, 0xFF, 0x7F, 0x05, 0x05, 0x65, 0x7F, 0x3F }, // 0x0E
  { 0x99, 0x5A, 0x3C, 0xE7, 0xE7, 0x3C, 0x5A, 0x99 }, // 0x0F
  { 0x7F, 0x3E, 0x3E, 0x1C, 0x1C, 0x08, 0x08, 0x00 }, // 0x10
  { 0x08, 0x08, 0x1C, 0x1C, 0x3E, 0x3E, 0x7F, 0x00 }, // 0x11
  { 0x00, 0x24, 0x66, 0xFF, 0xFF, 0x66, 0x24, 0x00 }, // 0x12
  { 0x00, 0x5F, 0x5F, 0x00, 0x00, 0x5F, 0x5F, 0x00 }, // 0x13
  { 0x06, 0x0F, 0x09, 0x7F, 0x7F, 0x01, 0x7F, 0x7F }, // 0x14
  { 0x40, 0xDA, 0xBF, 0xA5, 0xFD, 0x59, 0x03, 0x02 }, // 0x15
  { 0x00, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x00 }, // 0x16
  { 0x80, 0x94, 0xB6, 0xFF, 0xFF, 0xB6, 0x94, 0x80 }, // 0x17
  { 0x00, 0x04, 0x06, 0x7F, 0x7F, 0x06, 0x04, 0x00 }, // 0x18
  { 0x00, 0x10, 0x30, 0x7F, 0x7F, 0x30, 0x10, 0x00 }, // 0x19
  { 0x08, 0x08, 0x08, 0x2A, 0x3E, 0x1C, 0x08, 0x00 }, // 0x1A
  { 0x08, 0x1C, 0x3E, 0x2A, 0x08, 0x08, 0x08, 0x00 }, // 0x1B
  { 0x3C, 0x3C, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00 }, // 0x1C
  { 0x08, 0x1C, 0x3E, 0x08, 0x08, 0x3E, 0x1C, 0x08 }, // 0x1D
  { 0x30, 0x38, 0x3C, 0x3E, 0x3E, 0x3C, 0x38, 0x30 }, // 0x1E
  { 0x06, 0x0E, 0x1E, 0x3E, 0x3E, 0x1E, 0x0E, 0x06 }, // 0x1F
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // ' '
  { 0x00, 0x06, 0x5F, 0x5F, 0x06, 0x00, 0x00, 0x00 }, // '!'
  { 0x00, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x00 }, // '"'
  { 0x14, 0x7F, 0x7F, 0x14, 0x7F, 0x7F, 0x14, 0x00 }, // '#'
  { 0x24, 0x2E, 0x6B, 0x6B, 0x3A, 0x12, 0x00, 0x00 }, // '$'
  { 0x46, 0x66, 0x30, 0x18, 0x0C, 0x66, 0x62, 0x00 }, // '%'
  { 0x30, 0x7A, 0x4F, 0x5D, 0x37, 0x7A, 0x48, 0x00 }, // '&'
  { 0x04, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 }, // '''
  { 0x00, 0x1C, 0x3E, 0x63, 0x41, 0x00, 0x00, 0x00 }, // '('
  { 0x00, 0x41, 0x63, 0x3E, 0x1C, 0x00, 0x00, 0x00 }, // ')'
  { 0x08, 0x2A, 0x3E, 0x1C, 0x1C, 0x3E, 0x2A, 0x08 }, // '*'
  { 0x08, 0x08, 0x3E, 0x3E, 0x08, 0x08, 0x00, 0x00 }, // '+'
  { 0x00, 0x80, 0xE0, 0x60, 0x00, 0x00, 0x00, 0x00 }, // ','
  { 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00 }, // '-'
  { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00 }, // '.'
  { 0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00 }, // '/'
  { 0x3E, 0x7F, 0x71, 0x59, 0x4D, 0x7F, 0x3E, 0x00 }, // '0'
  { 0x40, 0x42, 0x7F, 0x7F, 0x40, 0x40, 0x00, 0x00 }, // '1''
  { 0x62, 0x73, 0x59, 0x49, 0x6F, 0x66, 0x00, 0x00 }, // '2'
  { 0x22, 0x63, 0x49, 0x49, 0x7F, 0x36, 0x00, 0x00 }, // '3'
  { 0x18, 0x1C, 0x16, 0x53, 0x7F, 0x7F, 0x50, 0x00 }, // '4'
  { 0x27, 0x67, 0x45, 0x45, 0x7D, 0x39, 0x00, 0x00 }, // '5'
  { 0x3C, 0x7E, 0x4B, 0x49, 0x79, 0x30, 0x00, 0x00 }, // '6'
  { 0x03, 0x03, 0x71, 0x79, 0x0F, 0x07, 0x00, 0x00 }, // '7'
  { 0x36, 0x7F, 0x49, 0x49, 0x7F, 0x36, 0x00, 0x00 }, // '8'
  { 0x06, 0x4F, 0x49, 0x69, 0x3F, 0x1E, 0x00, 0x00 }, // '9'
  { 0x00, 0x00, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00 }, // ':'
  { 0x00, 0x80, 0xE6, 0x66, 0x00, 0x00, 0x00, 0x00 }, // ';'
  { 0x08, 0x1C, 0x36, 0x63, 0x41, 0x00, 0x00, 0x00 }, // '<'
  { 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x00, 0x00 }, // '='
  { 0x00, 0x41, 0x63, 0x36, 0x1C, 0x08, 0x00, 0x00 }, // '>'
  { 0x02, 0x03, 0x51, 0x59, 0x0F, 0x06, 0x00, 0x00 }, // '?'
  { 0x3E, 0x7F, 0x41, 0x5D, 0x5D, 0x1F, 0x1E, 0x00 }, // '@'
  { 0x7C, 0x7E, 0x13, 0x13, 0x7E, 0x7C, 0x00, 0x00 }, // 'A'
  { 0x41, 0x7F, 0x7F, 0x49, 0x49, 0x7F, 0x36, 0x00 }, // 'B'
  { 0x1C, 0x3E, 0x63, 0x41, 0x41, 0x63, 0x22, 0x00 }, // 'C'
  { 0x41, 0x7F, 0x7F, 0x41, 0x63, 0x3E, 0x1C, 0x00 }, // 'D'
  { 0x41, 0x7F, 0x7F, 0x49, 0x5D, 0x41, 0x63, 0x00 }, // 'E'
  { 0x41, 0x7F, 0x7F, 0x49, 0x1D, 0x01, 0x03, 0x00 }, // 'F'
  { 0x1C, 0x3E, 0x63, 0x41, 0x51, 0x73, 0x72, 0x00 }, // 'G'
  { 0x7F, 0x7F, 0x08, 0x08, 0x7F, 0x7F, 0x00, 0x00 }, // 'H'
  { 0x00, 0x41, 0x7F, 0x7F, 0x41, 0x00, 0x00, 0x00 }, // 'I'
  { 0x30, 0x70, 0x40, 0x41, 0x7F, 0x3F, 0x01, 0x00 }, // 'J'
  { 0x41, 0x7F, 0x7F, 0x08, 0x1C, 0x77, 0x63, 0x00 }, // 'K'
  { 0x41, 0x7F, 0x7F, 0x41, 0x40, 0x60, 0x70, 0x00 }, // 'L'
  { 0x00, 0x7F, 0x0E, 0x1C, 0x0E, 0x7F, 0x00, 0x00 }, // 'M'
  { 0x7F, 0x7F, 0x06, 0x0C, 0x18, 0x7F, 0x7F, 0x00 }, // 'N'
  { 0x1C, 0x3E, 0x63, 0x41, 0x63, 0x3E, 0x1C, 0x00 }, // 'O'
  { 0x41, 0x7F, 0x7F, 0x49, 0x09, 0x0F, 0x06, 0x00 }, // 'P'
  { 0x1E, 0x3F, 0x21, 0x71, 0x7F, 0x5E, 0x00, 0x00 }, // 'Q'
  { 0x41, 0x7F, 0x7F, 0x09, 0x19, 0x7F, 0x66, 0x00 }, // 'R'
  { 0x26, 0x6F, 0x4D, 0x59, 0x73, 0x32, 0x00, 0x00 }, // 'S'
  { 0x03, 0x41, 0x7F, 0x7F, 0x41, 0x03, 0x00, 0x00 }, // 'T'
  { 0x7F, 0x7F, 0x40, 0x40, 0x7F, 0x7F, 0x00, 0x00 }, // 'U'
  { 0x1F, 0x3F, 0x60, 0x60, 0x3F, 0x1F, 0x00, 0x00 }, // 'V'
  { 0x7F, 0x7F, 0x30, 0x18, 0x30, 0x7F, 0x7F, 0x00 }, // 'W'
  { 0x43, 0x67, 0x3C, 0x18, 0x3C, 0x67, 0x43, 0x00 }, // 'X'
  { 0x07, 0x4F, 0x78, 0x78, 0x4F, 0x07, 0x00, 0x00 }, // 'Y'
  { 0x47, 0x63, 0x71, 0x59, 0x4D, 0x67, 0x73, 0x00 }, // 'Z'
  { 0x00, 0x7F, 0x7F, 0x41, 0x41, 0x00, 0x00, 0x00 }, // '['
  { 0x01, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x00 }, // backslash
  { 0x00, 0x41, 0x41, 0x7F, 0x7F, 0x00, 0x00, 0x00 }, // ']'
  { 0x08, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x08, 0x00 }, // '^'
  { 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80 }, // '_'
  { 0x00, 0x00, 0x03, 0x07, 0x04, 0x00, 0x00, 0x00 }, // '`'
  { 0x20, 0x74, 0x54, 0x54, 0x3C, 0x78, 0x40, 0x00 }, // 'a'
  { 0x41, 0x7F, 0x3F, 0x48, 0x48, 0x78, 0x30, 0x00 }, // 'b'
  { 0x38, 0x7C, 0x44, 0x44, 0x6C, 0x28, 0x00, 0x00 }, // 'c'
  { 0x30, 0x78, 0x48, 0x49, 0x3F, 0x7F, 0x40, 0x00 }, // 'd'
  { 0x38, 0x7C, 0x54, 0x54, 0x5C, 0x18, 0x00, 0x00 }, // 'e'
  { 0x48, 0x7E, 0x7F, 0x49, 0x03, 0x02, 0x00, 0x00 }, // 'f'
  { 0x98, 0xBC, 0xA4, 0xA4, 0xF8, 0x7C, 0x04, 0x00 }, // 'g'
  { 0x41, 0x7F, 0x7F, 0x08, 0x04, 0x7C, 0x78, 0x00 }, // 'h'
  { 0x00, 0x44, 0x7D, 0x7D, 0x40, 0x00, 0x00, 0x00 }, // 'i'
  { 0x60, 0xE0, 0x80, 0x80, 0xFD, 0x7D, 0x00, 0x00 }, // 'j'
  { 0x41, 0x7F, 0x7F, 0x10, 0x38, 0x6C, 0x44, 0x00 }, // 'k'
  { 0x00, 0x41, 0x7F, 0x7F, 0x40, 0x00, 0x00, 0x00 }, // 'l'
  { 0x7C, 0x7C, 0x18, 0x38, 0x1C, 0x7C, 0x78, 0x00 }, // 'm'
  { 0x7C, 0x7C, 0x04, 0x04, 0x7C, 0x78, 0x00, 0x00 }, // 'n'
  { 0x38, 0x7C, 0x44, 0x44, 0x7C, 0x38, 0x00, 0x00 }, // 'o'
  { 0x84, 0xFC, 0xF8, 0xA4, 0x24, 0x3C, 0x18, 0x00 }, // 'p'
  { 0x18, 0x3C, 0x24, 0xA4, 0xF8, 0xFC, 0x84, 0x00 }, // 'q'
  { 0x44, 0x7C, 0x78, 0x4C, 0x04, 0x1C, 0x18, 0x00 }, // 'r'
  { 0x48, 0x5C, 0x54, 0x54, 0x74, 0x24, 0x00, 0x00 }, // 's'
  { 0x00, 0x04, 0x3E, 0x7F, 0x44, 0x24, 0x00, 0x00 }, // 't'
  { 0x3C, 0x7C, 0x40, 0x40, 0x3C, 0x7C, 0x40, 0x00 }, // 'u'
  { 0x1C, 0x3C, 0x60, 0x60, 0x3C, 0x1C, 0x00, 0x00 }, // 'v'
  { 0x3C, 0x7C, 0x70, 0x38, 0x70, 0x7C, 0x3C, 0x00 }, // 'w'
  { 0x44, 0x6C, 0x38, 0x10, 0x38, 0x6C, 0x44, 0x00 }, // 'x'
  { 0x9C, 0xBC, 0xA0, 0xA0, 0xFC, 0x7C, 0x00, 0x00 }, // 'y'
  { 0x4C, 0x64, 0x74, 0x5C, 0x4C, 0x64, 0x00, 0x00 }, // 'z'
  { 0x08, 0x08, 0x3E, 0x77, 0x41, 0x41, 0x00, 0x00 }, // '{'
  { 0x00, 0x00, 0x00, 0x77, 0x77, 0x00, 0x00, 0x00 }, // '|'
  { 0x41, 0x41, 0x77, 0x3E, 0x08, 0x08, 0x00, 0x00 }, // '}'
  { 0x02, 0x03, 0x01, 0x03, 0x02, 0x03, 0x01, 0x00 }, // '~'
  { 0x70, 0x78, 0x4C, 0x46, 0x4C, 0x78, 0x70, 0x00 }, // 0x7F
  { 0x0E, 0x9F, 0x91, 0xB1, 0xFB, 0x4A, 0x00, 0x00 }, // 0x80
  { 0x3A, 0x7A, 0x40, 0x40, 0x7A, 0x7A, 0x40, 0x00 }, // 0x81
  { 0x38, 0x7C, 0x54, 0x55, 0x5D, 0x19, 0x00, 0x00 }, // 0x82
  { 0x02, 0x23, 0x75, 0x55, 0x55, 0x7D, 0x7B, 0x42 }, // 0x83
  { 0x21, 0x75, 0x54, 0x54, 0x7D, 0x79, 0x40, 0x00 }, // 0x84
  { 0x21, 0x75, 0x55, 0x54, 0x7C, 0x78, 0x40, 0x00 }, // 0x85
  { 0x20, 0x74, 0x57, 0x57, 0x7C, 0x78, 0x40, 0x00 }, // 0x86
  { 0x18, 0x3C, 0xA4, 0xA4, 0xE4, 0x40, 0x00, 0x00 }, // 0x87
  { 0x02, 0x3B, 0x7D, 0x55, 0x55, 0x5D, 0x1B, 0x02 }, // 0x88
  { 0x39, 0x7D, 0x54, 0x54, 0x5D, 0x19, 0x00, 0x00 }, // 0x89
  { 0x39, 0x7D, 0x55, 0x54, 0x5C, 0x18, 0x00, 0x00 }, // 0x8A
  { 0x01, 0x45, 0x7C, 0x7C, 0x41, 0x01, 0x00, 0x00 }, // 0x8B
  { 0x02, 0x03, 0x45, 0x7D, 0x7D, 0x43, 0x02, 0x00 }, // 0x8C
  { 0x01, 0x45, 0x7D, 0x7C, 0x40, 0x00, 0x00, 0x00 }, // 0x8D
  { 0x79, 0x7D, 0x16, 0x12, 0x16, 0x7D, 0x79, 0x00 }, // 0x8E
  { 0x70, 0x78, 0x2B, 0x2B, 0x78, 0x70, 0x00, 0x00 }, // 0x8F
  { 0x44, 0x7C, 0x7C, 0x55, 0x55, 0x45, 0x00, 0x00 }, // 0x90
  { 0x20, 0x74, 0x54, 0x54, 0x7C, 0x7C, 0x54, 0x54 }, // 0x91
  { 0x7C, 0x7E, 0x0B, 0x09, 0x7F, 0x7F, 0x49, 0x00 }, // 0x92
  { 0x32, 0x7B, 0x49, 0x49, 0x7B, 0x32, 0x00, 0x00 }, // 0x93
  { 0x32, 0x7A, 0x48, 0x48, 0x7A, 0x32, 0x00, 0x00 }, // 0x94
  { 0x32, 0x7A, 0x4A, 0x48, 0x78, 0x30, 0x00, 0x00 }, // 0x95
  { 0x3A, 0x7B, 0x41, 0x41, 0x7B, 0x7A, 0x40, 0x00 }, // 0x96
  { 0x3A, 0x7A, 0x42, 0x40, 0x78, 0x78, 0x40, 0x00 }, // 0x97
  { 0x9A, 0xBA, 0xA0, 0xA0, 0xFA, 0x7A, 0x00, 0x00 }, // 0x98
  { 0x01, 0x19, 0x3C, 0x66, 0x66, 0x3C, 0x19, 0x01 }, // 0x99
  { 0x3D, 0x7D, 0x40, 0x40, 0x7D, 0x3D, 0x00, 0x00 }, // 0x9A
  { 0x18, 0x3C, 0x24, 0xE7, 0xE7, 0x24, 0x24, 0x00 }, // 0x9B
  { 0x68, 0x7E, 0x7F, 0x49, 0x43, 0x66, 0x20, 0x00 }, // 0x9C
  { 0x2B, 0x2F, 0xFC, 0xFC, 0x2F, 0x2B, 0x00, 0x00 }, // 0x9D
  { 0xFF, 0xFF, 0x09, 0x09, 0x2F, 0xF6, 0xF8, 0xA0 }, // 0x9E
  { 0x40, 0xC0, 0x88, 0xFE, 0x7F, 0x09, 0x03, 0x02 }, // 0x9F
  { 0x20, 0x74, 0x54, 0x55, 0x7D, 0x79, 0x40, 0x00 }, // 0xA0
  { 0x00, 0x44, 0x7D, 0x7D, 0x41, 0x00, 0x00, 0x00 }, // 0xA1
  { 0x30, 0x78, 0x48, 0x4A, 0x7A, 0x32, 0x00, 0x00 }, // 0xA2
  { 0x38, 0x78, 0x40, 0x42, 0x7A, 0x7A, 0x40, 0x00 }, // 0xA3
  { 0x7A, 0x7A, 0x0A, 0x0A, 0x7A, 0x70, 0x00, 0x00 }, // 0xA4
  { 0x7D, 0x7D, 0x19, 0x31, 0x7D, 0x7D, 0x00, 0x00 }, // 0xA5
  { 0x00, 0x26, 0x2F, 0x29, 0x2F, 0x2F, 0x28, 0x00 }, // 0xA6
  { 0x00, 0x26, 0x2F, 0x29, 0x2F, 0x26, 0x00, 0x00 }, // 0xA7
  { 0x30, 0x78, 0x4D, 0x45, 0x60, 0x20, 0x00, 0x00 }, // 0xA8
  { 0x38, 0x38, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00 }, // 0xA9
  { 0x08, 0x08, 0x08, 0x08, 0x38, 0x38, 0x00, 0x00 }, // 0xAA
  { 0x4F, 0x6F, 0x30, 0x18, 0xCC, 0xEE, 0xBB, 0x91 }, // 0xAB
  { 0x4F, 0x6F, 0x30, 0x18, 0x6C, 0x76, 0xFB, 0xF9 }, // 0xAC
  { 0x00, 0x00, 0x00, 0x7B, 0x7B, 0x00, 0x00, 0x00 }, // 0xAD
  { 0x08, 0x1C, 0x36, 0x22, 0x08, 0x1C, 0x36, 0x22 }, // 0xAE
  { 0x22, 0x36, 0x1C, 0x08, 0x22, 0x36, 0x1C, 0x08 }, // 0xAF
  { 0xAA, 0x00, 0x55, 0x00, 0xAA, 0x00, 0x55, 0x00 }, // 0xB0
  { 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55 }, // 0xB1
  { 0xDD, 0xFF, 0xAA, 0x77, 0xDD, 0xAA, 0xFF, 0x77 }, // 0xB2
  { 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00 }, // 0xB3
  { 0x10, 0x10, 0x10, 0xFF, 0xFF, 0x00, 0x00, 0x00 }, // 0xB4
  { 0x14, 0x14, 0x14, 0xFF, 0xFF, 0x00, 0x00, 0x00 }, // 0xB5
  { 0x10, 0x10, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x00 }, // 0xB6
  { 0x10, 0x10, 0xF0, 0xF0, 0x10, 0xF0, 0xF0, 0x00 }, // 0xB7
  { 0x14, 0x14, 0x14, 0xFC, 0xFC, 0x00, 0x00, 0x00 }, // 0xB8
  { 0x14, 0x14, 0xF7, 0xF7, 0x00, 0xFF, 0xFF, 0x00 }, // 0xB9
  { 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x00 }, // 0xBA
  { 0x14, 0x14, 0xF4, 0xF4, 0x04, 0xFC, 0xFC, 0x00 }, // 0xBB
  { 0x14, 0x14, 0x17, 0x17, 0x10, 0x1F, 0x1F, 0x00 }, // 0xBC
  { 0x10, 0x10, 0x1F, 0x1F, 0x10, 0x1F, 0x1F, 0x00 }, // 0xBD
  { 0x14, 0x14, 0x14, 0x1F, 0x1F, 0x00, 0x00, 0x00 }, // 0xBE
  { 0x10, 0x10, 0x10, 0xF0, 0xF0, 0x00, 0x00, 0x00 }, // 0xBF
  { 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x10, 0x10, 0x10 }, // 0xC0
  { 0x10, 0x10, 0x10, 0x1F, 0x1F, 0x10, 0x10, 0x10 }, // 0xC1
  { 0x10, 0x10, 0x10, 0xF0, 0xF0, 0x10, 0x10, 0x10 }, // 0xC2
  { 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x10, 0x10, 0x10 }, // 0xC3
  { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10 }, // 0xC4
  { 0x10, 0x10, 0x10, 0xFF, 0xFF, 0x10, 0x10, 0x10 }, // 0xC5
  { 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x14, 0x14, 0x14 }, // 0xC6
  { 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x10 }, // 0xC7
  { 0x00, 0x00, 0x1F, 0x1F, 0x10, 0x17, 0x17, 0x14 }, // 0xC8
  { 0x00, 0x00, 0xFC, 0xFC, 0x04, 0xF4, 0xF4, 0x14 }, // 0xC9
  { 0x14, 0x14, 0x17, 0x17, 0x10, 0x17, 0x17, 0x14 }, // 0xCA
  { 0x14, 0x14, 0xF4, 0xF4, 0x04, 0xF4, 0xF4, 0x14 }, // 0xCB
  { 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xF7, 0xF7, 0x14 }, // 0xCC
  { 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 }, // 0xCD
  { 0x14, 0x14, 0xF7, 0xF7, 0x00, 0xF7, 0xF7, 0x14 }, // 0xCE
  { 0x14, 0x14, 0x14, 0x17, 0x17, 0x14, 0x14, 0x14 }, // 0xCF
  { 0x10, 0x10, 0x1F, 0x1F, 0x10, 0x1F, 0x1F, 0x10 }, // 0xD0
  { 0x14, 0x14, 0x14, 0xF4, 0xF4, 0x14, 0x14, 0x14 }, // 0xD1
  { 0x10, 0x10, 0xF0, 0xF0, 0x10, 0xF0, 0xF0, 0x10 }, // 0xD2
  { 0x00, 0x00, 0x1F, 0x1F, 0x10, 0x1F, 0x1F, 0x10 }, // 0xD3
  { 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x14, 0x14, 0x14 }, // 0xD4
  { 0x00, 0x00, 0x00, 0xFC, 0xFC, 0x14, 0x14, 0x14 }, // 0xD5
  { 0x00, 0x00, 0xF0, 0xF0, 0x10, 0xF0, 0xF0, 0x10 }, // 0xD6
  { 0x10, 0x10, 0xFF, 0xFF, 0x10, 0xFF, 0xFF, 0x10 }, // 0xD7
  { 0x14, 0x14, 0x14, 0xFF, 0xFF, 0x14, 0x14, 0x14 }, // 0xD8
  { 0x10, 0x10, 0x10, 0x1F, 0x1F, 0x00, 0x00, 0x00 }, // 0xD9
  { 0x00, 0x00, 0x00, 0xF0, 0xF0, 0x10, 0x10, 0x10 }, // 0xDA
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }, // 0xDB
  { 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0 }, // 0xDC
  { 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00 }, // 0xDD
  { 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF }, // 0xDE
  { 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F }, // 0xDF
  { 0x38, 0x7C, 0x44, 0x6C, 0x38, 0x6C, 0x44, 0x00 }, // 0xE0
  { 0xFC, 0xFE, 0x2A, 0x2A, 0x3E, 0x14, 0x00, 0x00 }, // 0xE1
  { 0x7E, 0x7E, 0x02, 0x02, 0x06, 0x06, 0x00, 0x00 }, // 0xE2
  { 0x02, 0x7E, 0x7E, 0x02, 0x7E, 0x7E, 0x02, 0x00 }, // 0xE3
  { 0x63, 0x77, 0x5D, 0x49, 0x63, 0x63, 0x00, 0x00 }, // 0xE4
  { 0x38, 0x7C, 0x44, 0x7C, 0x3C, 0x04, 0x04, 0x00 }, // 0xE5
  { 0x80, 0xFE, 0x7E, 0x20, 0x20, 0x3E, 0x1E, 0x00 }, // 0xE6
  { 0x04, 0x06, 0x02, 0x7E, 0x7C, 0x06, 0x02, 0x00 }, // 0xE7
  { 0x99, 0xBD, 0xE7, 0xE7, 0xBD, 0x99, 0x00, 0x00 }, // 0xE8
  { 0x1C, 0x3E, 0x6B, 0x49, 0x6B, 0x3E, 0x1C, 0x00 }, // 0xE9
  { 0x4C, 0x7E, 0x73, 0x01, 0x73, 0x7E, 0x4C, 0x00 }, // 0xEA
  { 0x30, 0x78, 0x4A, 0x4F, 0x7D, 0x39, 0x00, 0x00 }, // 0xEB
  { 0x18, 0x3C, 0x24, 0x3C, 0x3C, 0x24, 0x3C, 0x18 }, // 0xEC
  { 0x98, 0xFC, 0x64, 0x3C, 0x3E, 0x27, 0x3D, 0x18 }, // 0xED
  { 0x1C, 0x3E, 0x6B, 0x49, 0x49, 0x00, 0x00, 0x00 }, // 0xEE
  { 0x7E, 0x7F, 0x01, 0x01, 0x7F, 0x7E, 0x00, 0x00 }, // 0xEF
  { 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x00, 0x00 }, // 0xF0
  { 0x44, 0x44, 0x5F, 0x5F, 0x44, 0x44, 0x00, 0x00 }, // 0xF1
  { 0x40, 0x51, 0x5B, 0x4E, 0x44, 0x40, 0x00, 0x00 }, // 0xF2
  { 0x40, 0x44, 0x4E, 0x5B, 0x51, 0x40, 0x00, 0x00 }, // 0xF3
  { 0x00, 0x00, 0x00, 0xFE, 0xFF, 0x01, 0x07, 0x06 }, // 0xF4
  { 0x60, 0xE0, 0x80, 0xFF, 0x7F, 0x00, 0x00, 0x00 }, // 0xF5
  { 0x08, 0x08, 0x6B, 0x6B, 0x08, 0x08, 0x00, 0x00 }, // 0xF6
  { 0x24, 0x36, 0x12, 0x36, 0x24, 0x36, 0x12, 0x00 }, // 0xF7
  { 0x00, 0x06, 0x0F, 0x09, 0x0F, 0x06, 0x00, 0x00 }, // 0xF8
  { 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00 }, // 0xF9
  { 0x00, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00 }, // 0xFA
  { 0x10, 0x30, 0x70, 0xC0, 0xFF, 0xFF, 0x01, 0x01 }, // 0xFB
  { 0x00, 0x1F, 0x1F, 0x01, 0x1F, 0x1E, 0x00, 0x00 }, // 0xFC
  { 0x00, 0x19, 0x1D, 0x17, 0x12, 0x00, 0x00, 0x00 }, // 0xFD
  { 0x00, 0x00, 0x3C, 0x3C, 0x3C, 0x3C, 0x00, 0x00 }, // 0xFE
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0xFF
};  //  end of MAX7219_Dot_Matrix_font

