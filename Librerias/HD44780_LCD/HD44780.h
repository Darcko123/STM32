/**
 * @file HD44780.h
 * @brief Librería para la gestión del módulo LCD HD44780 mediante comunicación I2C.
 *
 * @author Eziya - Daniel Ruiz
 * @date Mayo 10, 2026
 * @version 2.1.0
 */

#ifndef HD44780_H
#define HD44780_H

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdio.h>

// ============================================================================
// MACROS Y CONSTANTES DE COMANDOS HD44780
// ============================================================================

/** @brief Dirección I2C del módulo LCD HD44780. */
#define HD44780_ADDRESS     (0x27 << 1)     /**< I2C device address (0x4E) */

/** @brief Timeout por defecto para transmisiones HAL (ms). */
#define HD44780_TIMEOUT_MS  10U

#define LCD_CLEARDISPLAY    0x01
#define LCD_RETURNHOME      0x02
#define LCD_ENTRYMODESET    0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_CURSORSHIFT     0x10
#define LCD_FUNCTIONSET     0x20
#define LCD_SETCGRAMADDR    0x40
#define LCD_SETDDRAMADDR    0x80

/* Entry Mode */
#define LCD_ENTRYRIGHT              0x00
#define LCD_ENTRYLEFT               0x02
#define LCD_ENTRYSHIFTINCREMENT     0x01
#define LCD_ENTRYSHIFTDECREMENT     0x00

/* Display On/Off */
#define LCD_DISPLAYON   0x04
#define LCD_DISPLAYOFF  0x00
#define LCD_CURSORON    0x02
#define LCD_CURSOROFF   0x00
#define LCD_BLINKON     0x01
#define LCD_BLINKOFF    0x00

/* Cursor Shift */
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE  0x00
#define LCD_MOVERIGHT   0x04
#define LCD_MOVELEFT    0x00

/* Function Set */
#define LCD_8BITMODE    0x10
#define LCD_4BITMODE    0x00
#define LCD_2LINE       0x08
#define LCD_1LINE       0x00
#define LCD_5x10DOTS    0x04
#define LCD_5x8DOTS     0x00

/* Backlight */
#define LCD_BACKLIGHT   0x08
#define LCD_NOBACKLIGHT 0x00

/* Enable Bit */
#define ENABLE  0x04

/* Read/Write Bit */
#define RW  0x02

/* Register Select Bit */
#define RS  0x01

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Códigos de estado retornados por todas las funciones públicas del HD44780.
 */
typedef enum {
    HD44780_OK              = 0,    /**< Operación exitosa */
    HD44780_ERROR           = 1,    /**< Error en la operación */
    HD44780_TIMEOUT         = 2,    /**< Timeout en la operación */
    HD44780_NOT_INITIALIZED = 3,    /**< Módulo no inicializado */
    HD44780_INVALID_PARAM   = 4,    /**< Parámetro inválido */
} HD44780_Status_t;

// ============================================================================
// VARIABLES PÚBLICAS
// ============================================================================

extern const uint8_t special1[8];
extern const uint8_t special2[8];
extern const uint8_t heart[8];
extern const uint8_t Cyrillic[8];
extern const uint8_t Flecha[8];
extern const uint8_t Campana[8];
extern const uint8_t degrees[8];

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el módulo HD44780 con el handle I2C.
 *
 * @param[in] hi2c Puntero al handle de I2C.
 * @param[in] rows Número de filas del display.
 * @param[in] cols Número de columnas del display.
 * @return HD44780_Status_t
 *         - HD44780_OK            si la inicialización fue exitosa.
 *         - HD44780_ERROR         si ocurrió un error de comunicación.
 *         - HD44780_INVALID_PARAM si @p hi2c es NULL, @p rows es 0 o @p cols es 0.
 */
HD44780_Status_t HD44780_Init(I2C_HandleTypeDef* hi2c, uint8_t rows, uint8_t cols);

/**
 * @brief Borra el display y retorna el cursor a la posición (0,0).
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_Clear(void);

/**
 * @brief Retorna el cursor a la posición (0,0) sin borrar el contenido del display.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_Home(void);

/**
 * @brief Apaga el display conservando el contenido en memoria.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_NoDisplay(void);

/**
 * @brief Enciende el display para mostrar el contenido de la memoria.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_Display(void);

/**
 * @brief Desactiva el parpadeo del cursor.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_NoBlink(void);

/**
 * @brief Activa el parpadeo del cursor (bloque parpadeante en la posición actual).
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_Blink(void);

/**
 * @brief Oculta el cursor de subrayado del display.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_NoCursor(void);

/**
 * @brief Muestra el cursor de subrayado en la posición actual del display.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_Cursor(void);

/**
 * @brief Desplaza el contenido del display una posición hacia la izquierda.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_ScrollDisplayLeft(void);

/**
 * @brief Desplaza el contenido del display una posición hacia la derecha.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_ScrollDisplayRight(void);

/**
 * @brief Configura la dirección de escritura de izquierda a derecha (modo por defecto).
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_LeftToRight(void);

/**
 * @brief Configura la dirección de escritura de derecha a izquierda.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_RightToLeft(void);

/**
 * @brief Apaga la retroiluminación del display.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_NoBacklight(void);

/**
 * @brief Enciende la retroiluminación del display.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_Backlight(void);

/**
 * @brief Activa el desplazamiento automático del display al escribir un carácter.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_AutoScroll(void);

/**
 * @brief Desactiva el desplazamiento automático del display.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_NoAutoScroll(void);

/**
 * @brief Crea un carácter personalizado en la memoria CGRAM del LCD.
 *
 * @param[in] location Posición en CGRAM donde almacenar el carácter (0–7).
 * @param[in] charmap  Array de 8 bytes con el patrón del carácter (5 bits útiles por fila).
 * @return HD44780_Status_t
 *         - HD44780_OK              si el carácter fue creado correctamente.
 *         - HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 *         - HD44780_INVALID_PARAM   si @p charmap es NULL.
 */
HD44780_Status_t HD44780_CreateSpecialChar(uint8_t location, const uint8_t charmap[]);

/**
 * @brief Imprime un carácter especial guardado en CGRAM en la posición actual del cursor.
 *
 * @param[in] index Índice del carácter en CGRAM (0–7), correspondiente al usado en
 *                  HD44780_CreateSpecialChar().
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_PrintSpecialChar(uint8_t index);

/**
 * @brief Posiciona el cursor en la columna y fila indicadas.
 *
 * @param[in] col Columna (posición horizontal), comienza en 0.
 * @param[in] row Fila (posición vertical), comienza en 0. Si supera el número de filas
 *                configuradas, se ajusta a la última fila.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_SetCursor(uint8_t col, uint8_t row);

/**
 * @brief Establece el estado de la retroiluminación del display.
 *
 * @param[in] new_val Valor de retroiluminación: distinto de 0 para encender, 0 para apagar.
 * @return HD44780_Status_t HD44780_OK si la operación fue exitosa,
 *         HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 */
HD44780_Status_t HD44780_SetBacklight(uint8_t new_val);

/**
 * @brief Imprime una cadena de caracteres terminada en null en el display.
 *
 * @param[in] c Puntero a la cadena de caracteres a imprimir.
 * @return HD44780_Status_t
 *         - HD44780_OK              si todos los caracteres fueron enviados correctamente.
 *         - HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 *         - HD44780_INVALID_PARAM   si @p c es NULL.
 */
HD44780_Status_t HD44780_PrintStr(const char c[]);

/**
 * @brief Imprime un número entero en el display, convirtiéndolo a su representación decimal.
 *
 * @param[in] num Número entero a imprimir.
 * @return HD44780_Status_t
 *         - HD44780_OK              si todos los caracteres fueron enviados correctamente.
 *         - HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 *         - HD44780_ERROR           si ocurrió un error interno al convertir el número.
 */
HD44780_Status_t HD44780_PrintInt(int num);

/**
 * @brief Borra una fila específica del display escribiendo espacios, sin afectar las demás filas.
 *        Deja el cursor al inicio de la fila borrada.
 *
 * @param[in] row Fila a borrar, comienza en 0.
 * @return HD44780_Status_t
 *         - HD44780_OK              si la operación fue exitosa.
 *         - HD44780_NOT_INITIALIZED si el módulo no fue inicializado.
 *         - HD44780_INVALID_PARAM   si @p row es mayor o igual al número de filas configuradas.
 */
HD44780_Status_t HD44780_ClearLine(uint8_t row);

#ifdef __cplusplus
}
#endif

#endif /* HD44780_H */
