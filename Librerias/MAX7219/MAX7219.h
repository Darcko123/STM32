/**
 * @file MAX7219.h
 *
 * @brief Librería para el control de matrices de LEDs MAX7219 mediante SPI.
 *
 * @author Daniel Ruiz
 * @date Noviembre 03, 2026
 * @version 3.0.0
 */


#ifndef MAX7219_H_
#define MAX7219_H_

// ============================================================================
// INCLUDES
// ============================================================================

#include "main.h"
#include <stdint.h>

// ============================================================================
// DEFINICIONES DE CONSTANTES
// ============================================================================

#ifndef MAX7219_MAX_DEV
#define MAX7219_MAX_DEV        4U   /**< Número máximo de MAX7219 en cascada (dimensiona el buffer estático) */
#endif
#ifndef MAX7219_SPI_TIMEOUT_MS
#define MAX7219_SPI_TIMEOUT_MS 100U /**< Timeout de cada transmisión SPI [ms] */
#endif
#define MAX7219_INTENSITY_MAX  15U  /**< Nivel máximo de MAX7219_SetIntensity (registro 0x0A: 0x0 a 0xF) */

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================
/**
 * @brief Enumeración para estados de retorno del MAX7219.
 */
typedef enum {
	MAX7219_OK = 0,					/**< Operación exitosa */
	MAX7219_ERROR = 1,				/**< Error en la operación */
	MAX7219_TIMEOUT = 2,			/**< Timeout en la operación */
	MAX7219_NOT_INITIALIZED = 3,	/**< Módulo no inicializado */
	MAX7219_INVALID_PARAM = 4		/**< Parámetro inválido */
}MAX7219_Status_t;

/**
 * @brief Estado de un LED de la matriz.
 */
typedef enum {
	MAX7219_COLOR_OFF = 0x00,	/**< LED apagado */
	MAX7219_COLOR_ON  = 0x01	/**< LED encendido */
} MAX7219_COLOR_t;

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa la matriz de LEDs.
 *
 * @param hspi Puntero al manejador de la interfaz SPI utilizada para comunicarse con el módulo.
 * @param GPIOx Puerto GPIO del pin CS del MAX7219.
 * @param GPIO_PIN Pin GPIO del pin CS del MAX7219.
 * @param numDevices Número de dispositivos MAX7219 conectados en cascada (1 a MAX7219_MAX_DEV).
 *
 * @return MAX7219_Status_t Estado de la operación.
 */
MAX7219_Status_t MAX7219_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN, uint8_t numDevices);

/**
 * @brief Desinicializa el módulo MAX7219, apagando la pantalla y liberando la configuración almacenada.
 *
 * @return MAX7219_Status_t Estado de la operación.
 */
MAX7219_Status_t MAX7219_DeInit(void);

/**
 * @brief Activa o desactiva el modo de bajo consumo (shutdown) del MAX7219 sin perder la configuración almacenada.
 *
 * A diferencia de MAX7219_DeInit, esta función no libera el manejador SPI ni el pin CS, por lo que
 * el módulo puede reactivarse posteriormente sin necesidad de volver a llamar a MAX7219_Init.
 *
 * @param enable Distinto de 0 para entrar en modo de bajo consumo, 0 para volver al modo de operación normal.
 *
 * @return MAX7219_Status_t Estado de la operación.
 */
MAX7219_Status_t MAX7219_SetSleep(uint8_t enable);

/**
 * @brief Ajusta el brillo de la matriz de LEDs.
 *
 * Escribe directamente en el registro de intensidad (0x0A) de todos los MAX7219 de la cascada;
 * no usa el buffer ni requiere MAX7219_UpdateScreen(). El nivel 0 equivale a 1/32 de la corriente
 * máxima y el 15 a 31/32 (datasheet del MAX7219). MAX7219_Init arranca en el nivel 1.
 *
 * @param level Nivel de brillo, de 0 a MAX7219_INTENSITY_MAX (15).
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si el brillo se actualizó.
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 *         - MAX7219_INVALID_PARAM   si @p level es mayor que MAX7219_INTENSITY_MAX.
 *         - MAX7219_ERROR / MAX7219_TIMEOUT si falló la transmisión SPI.
 */
MAX7219_Status_t MAX7219_SetIntensity(uint8_t level);

/**
 * @brief Apaga todos los LEDs de la matriz.
 *
 * Equivale a MAX7219_Fill(MAX7219_COLOR_OFF) seguido de MAX7219_UpdateScreen():
 * vacía el buffer interno y actualiza la pantalla de inmediato.
 *
 * @return MAX7219_Status_t Estado de la operación.
 */
MAX7219_Status_t MAX7219_Clear(void);

/**
 * @brief Muestra una cadena de caracteres en la matriz de LEDs con desplazamiento.
 *
 * @param str Cadena de caracteres a mostrar.
 * @param delay Tiempo de retardo entre cada desplazamiento [ms].
 *
 * @return MAX7219_Status_t Estado de la operación.
 */
MAX7219_Status_t MAX7219_ScrollString(const char *str, uint32_t delay);

/**
 * @brief Muestra una cadena de caracteres en la matriz de LEDs.
 * 
 * Esta función toma una cadena de texto y la convierte en una representación de bytes
 * para visualizarla en la matriz de LEDs. Cada carácter se almacena en el buffer de
 * columnas y se muestra en la pantalla. Las columnas no ocupadas por el texto se apagan.
 *
 * @param str Puntero a la cadena de caracteres que se mostrará.
 *
 * @return MAX7219_Status_t Estado de la operación.
 */
MAX7219_Status_t MAX7219_PrintString(const char *str);

/**
 * @brief Enciende o apaga un píxel individual de la matriz de LEDs.
 *
 * Equivale a MAX7219_DrawPixel() seguido de MAX7219_UpdateScreen(): la actualización
 * de la pantalla es inmediata. Para dibujar varios píxeles, es más eficiente usar
 * MAX7219_DrawPixel() y llamar una sola vez a MAX7219_UpdateScreen().
 *
 * @note Desde la versión 3.0.0 el origen (0,0) es la esquina superior izquierda, igual que en
 *       el resto de pantallas del repositorio. En la versión 2.x era la esquina inferior derecha:
 *       para conservar un dibujo antiguo, usar SetPixel(MAX7219_GetWidth() - 1 - x, 7 - y, state).
 *
 * @param x Columna del píxel: 0 = izquierda, hasta MAX7219_GetWidth() - 1.
 * @param y Fila del píxel: 0 = arriba, hasta 7.
 * @param state Distinto de 0 para encender el píxel, 0 para apagarlo.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si el píxel se escribió y la pantalla se actualizó.
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 *         - MAX7219_INVALID_PARAM   si @p x o @p y están fuera de la pantalla.
 *         - MAX7219_ERROR / MAX7219_TIMEOUT si falló la transmisión SPI.
 */
MAX7219_Status_t MAX7219_SetPixel(uint16_t x, uint16_t y, uint8_t state);

/**
 * @brief Obtiene el ancho de la matriz configurado en MAX7219_Init.
 *
 * @return uint16_t Ancho en píxeles (número de dispositivos × 8), o 0 si el módulo no ha sido inicializado.
 */
uint16_t MAX7219_GetWidth(void);

/**
 * @brief Obtiene el alto de la matriz.
 *
 * @return uint16_t Alto en píxeles (8), o 0 si el módulo no ha sido inicializado.
 */
uint16_t MAX7219_GetHeight(void);

/**
 * @brief Envía el contenido del buffer interno a la matriz de LEDs.
 *
 * Las funciones de dibujo (MAX7219_DrawPixel, MAX7219_Fill) solo escriben en el buffer;
 * los cambios se ven al llamar a esta función.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si la pantalla se actualizó.
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 *         - MAX7219_ERROR / MAX7219_TIMEOUT si falló la transmisión SPI.
 */
MAX7219_Status_t MAX7219_UpdateScreen(void);

/**
 * @brief Rellena todo el buffer interno con un color.
 *
 * Solo escribe en el buffer: llamar a MAX7219_UpdateScreen() para mostrar el resultado.
 *
 * @param color MAX7219_COLOR_ON para encender todos los LEDs, MAX7219_COLOR_OFF para apagarlos.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si el buffer se rellenó.
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 */
MAX7219_Status_t MAX7219_Fill(MAX7219_COLOR_t color);

/**
 * @brief Invierte por software el estado de todos los LEDs del buffer interno.
 *
 * El MAX7219 no tiene inversión por hardware (como SSD1681_ToggleInvert, se hace sobre el buffer).
 * Cada llamada invierte el contenido actual del buffer y alterna una bandera: mientras esté activa,
 * MAX7219_Fill, MAX7219_DrawPixel y el resto de primitivas pintan con el color contrario (COLOR_ON apaga
 * el LED y COLOR_OFF lo enciende), de modo que el dibujo conserva su aspecto invertido. La bandera
 * se desactiva en MAX7219_Init.
 * Solo escribe en el buffer: llamar a MAX7219_UpdateScreen() para mostrar el resultado.
 *
 * @note MAX7219_Clear y MAX7219_SetPixel también siguen la bandera (con la inversión activa, Clear deja
 *       todos los LEDs encendidos). MAX7219_PrintString y MAX7219_ScrollString no la respetan:
 *       escriben el buffer sin invertir.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si el buffer se invirtió.
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 */
MAX7219_Status_t MAX7219_ToggleInvert(void);

/**
 * @brief Enciende o apaga un píxel en el buffer interno.
 *
 * Solo escribe en el buffer: llamar a MAX7219_UpdateScreen() para mostrar el resultado.
 * El origen (0,0) es la esquina superior izquierda.
 *
 * @param x     Columna del píxel: 0 = izquierda, hasta MAX7219_GetWidth() - 1.
 * @param y     Fila del píxel: 0 = arriba, hasta 7.
 * @param color Color del píxel.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si el píxel se escribió en el buffer.
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 *         - MAX7219_INVALID_PARAM   si @p x o @p y están fuera de la pantalla.
 */
MAX7219_Status_t MAX7219_DrawPixel(uint16_t x, uint16_t y, MAX7219_COLOR_t color);

/**
 * @brief Dibuja una línea horizontal en el buffer interno.
 *
 * Solo escribe en el buffer: llamar a MAX7219_UpdateScreen() para mostrar el resultado.
 * Los píxeles fuera de la pantalla se recortan sin generar error.
 *
 * @param x     Columna inicial: 0 = izquierda.
 * @param y     Fila de la línea: 0 = arriba.
 * @param w     Ancho de la línea (puede ser negativo: se dibuja hacia la izquierda de @p x).
 * @param color Color de la línea.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si la línea se escribió en el buffer (también si @p w es 0).
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 */
MAX7219_Status_t MAX7219_DrawFastHLine(int16_t x, int16_t y, int16_t w, MAX7219_COLOR_t color);

/**
 * @brief Dibuja una línea vertical en el buffer interno.
 *
 * Solo escribe en el buffer: llamar a MAX7219_UpdateScreen() para mostrar el resultado.
 * Los píxeles fuera de la pantalla se recortan sin generar error.
 *
 * @param x     Columna de la línea: 0 = izquierda.
 * @param y     Fila inicial: 0 = arriba.
 * @param h     Alto de la línea (puede ser negativo: se dibuja hacia arriba de @p y).
 * @param color Color de la línea.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si la línea se escribió en el buffer (también si @p h es 0).
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 */
MAX7219_Status_t MAX7219_DrawFastVLine(int16_t x, int16_t y, int16_t h, MAX7219_COLOR_t color);

/**
 * @brief Dibuja una línea entre dos puntos en el buffer interno (algoritmo de Bresenham).
 *
 * Solo escribe en el buffer: llamar a MAX7219_UpdateScreen() para mostrar el resultado.
 *
 * @param x0 Columna del primer extremo.
 * @param y0 Fila del primer extremo.
 * @param x1 Columna del segundo extremo.
 * @param y1 Fila del segundo extremo.
 * @param c  Color de la línea.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si la línea se escribió en el buffer.
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 *         - MAX7219_INVALID_PARAM   si algún extremo está fuera de la pantalla (no se dibuja nada).
 */
MAX7219_Status_t MAX7219_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, MAX7219_COLOR_t c);

/**
 * @brief Dibuja el borde de un rectángulo en el buffer interno.
 *
 * Solo escribe en el buffer: llamar a MAX7219_UpdateScreen() para mostrar el resultado.
 * Recibe dos esquinas opuestas, en cualquier orden (como SSD1681_DrawRectangle).
 *
 * @param x0 Columna de la primera esquina.
 * @param y0 Fila de la primera esquina.
 * @param x1 Columna de la esquina opuesta.
 * @param y1 Fila de la esquina opuesta.
 * @param c  Color del borde.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si el rectángulo se escribió en el buffer.
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 *         - MAX7219_INVALID_PARAM   si alguna esquina está fuera de la pantalla (no se dibuja nada).
 */
MAX7219_Status_t MAX7219_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, MAX7219_COLOR_t c);

/**
 * @brief Dibuja un rectángulo relleno en el buffer interno.
 *
 * Solo escribe en el buffer: llamar a MAX7219_UpdateScreen() para mostrar el resultado.
 * Recibe esquina superior izquierda, ancho y alto (como SSD1681_DrawFilledRectangle, a diferencia
 * de MAX7219_DrawRectangle, que recibe dos esquinas). Lo que sobresale de la pantalla se recorta.
 *
 * @param x Columna de la esquina superior izquierda.
 * @param y Fila de la esquina superior izquierda.
 * @param w Ancho en píxeles.
 * @param h Alto en píxeles.
 * @param c Color del relleno.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si el rectángulo se escribió en el buffer (también si @p w o @p h son 0).
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 *         - MAX7219_INVALID_PARAM   si la esquina superior izquierda está fuera de la pantalla.
 */
MAX7219_Status_t MAX7219_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, MAX7219_COLOR_t c);

/**
 * @brief Dibuja un bitmap monocromo en el buffer interno.
 *
 * Solo escribe en el buffer: llamar a MAX7219_UpdateScreen() para mostrar el resultado.
 * Mismo formato que SSD1681_DrawBitmap: por filas, bit 7 = píxel de la izquierda, cada fila ocupa
 * (w + 7) / 8 bytes (compatible con image2cpp, "Horizontal - 1 bit per pixel"). Es transparente:
 * solo se pintan los bits a 1. Lo que sobresale de la pantalla se recorta sin generar error.
 *
 * @param x      Columna de la esquina superior izquierda (puede ser negativa).
 * @param y      Fila de la esquina superior izquierda (puede ser negativa).
 * @param bitmap Datos del bitmap.
 * @param w      Ancho del bitmap en píxeles.
 * @param h      Alto del bitmap en píxeles.
 * @param color  Color de los píxeles activos.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si el bitmap se escribió en el buffer.
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 *         - MAX7219_INVALID_PARAM   si @p bitmap es NULL.
 */
MAX7219_Status_t MAX7219_DrawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, MAX7219_COLOR_t color);

/**
 * @brief Coloca el cursor de escritura usado por MAX7219_Putc y MAX7219_Puts.
 *
 * No dibuja ni actualiza la pantalla. El cursor es la esquina superior izquierda del siguiente carácter.
 *
 * @param x Columna del cursor: 0 = izquierda.
 * @param y Fila del cursor: 0 = arriba.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si el cursor se movió.
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 *         - MAX7219_INVALID_PARAM   si (@p x, @p y) está fuera de la pantalla.
 */
MAX7219_Status_t MAX7219_GotoXY(uint16_t x, uint16_t y);

/**
 * @brief Escribe un carácter en el buffer interno en la posición del cursor.
 *
 * Solo escribe en el buffer: llamar a MAX7219_UpdateScreen() para mostrar el resultado.
 * Es opaco (pinta el fondo del glifo con el color contrario). Si el glifo no cabe entero se recorta
 * en el borde y el cursor avanza igualmente. Cada carácter avanza el cursor 7 columnas, como MAX7219_PrintString.
 * A diferencia de MAX7219_PrintString, no borra el resto de la pantalla.
 *
 * @param ch    Carácter a escribir (los 256 códigos de la fuente).
 * @param color Color del carácter.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si el carácter se escribió (aunque quedara recortado).
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 *         - MAX7219_INVALID_PARAM   si el cursor ya está fuera de la pantalla (no se dibuja nada).
 */
MAX7219_Status_t MAX7219_Putc(char ch, MAX7219_COLOR_t color);

/**
 * @brief Escribe una cadena en el buffer interno desde la posición del cursor.
 *
 * Solo escribe en el buffer: llamar a MAX7219_UpdateScreen() para mostrar el resultado.
 * El texto que sobrepasa el borde derecho se recorta, pero la función devuelve MAX7219_INVALID_PARAM.
 *
 * @param str   Cadena terminada en '\0'.
 * @param color Color de los caracteres.
 *
 * @return MAX7219_Status_t
 *         - MAX7219_OK              si toda la cadena se escribió.
 *         - MAX7219_NOT_INITIALIZED si no se ha llamado a MAX7219_Init.
 *         - MAX7219_INVALID_PARAM   si @p str es NULL o el texto llegó al borde antes de terminar.
 */
MAX7219_Status_t MAX7219_Puts(const char* str, MAX7219_COLOR_t color);

/**
 * @brief Calcula el ancho en columnas que ocupa una cadena escrita con MAX7219_Puts.
 *
 * No toca el buffer ni la pantalla, ni requiere MAX7219_Init. Cuenta 7 columnas por carácter (el avance
 * del cursor), sin recortar al ancho de la pantalla, así que puede ser mayor que MAX7219_GetWidth().
 * Para centrar: MAX7219_GotoXY((MAX7219_GetWidth() - MAX7219_GetStringWidth(s)) / 2, 0), comprobando antes
 * que la cadena cabe (la resta es sin signo).
 *
 * @param str Cadena terminada en '\0'.
 *
 * @return uint16_t Ancho en columnas; 0 si @p str es NULL o está vacía.
 */
uint16_t MAX7219_GetStringWidth(const char* str);

/**
 * @brief Conjunto de fuentes para la pantalla de matriz de puntos MAX7219.
 *
 * Este array contiene el conjunto de fuentes de 8x8 para la pantalla de matriz de puntos MAX7219.
 * Cada carácter está representado por un array de 8 bytes, donde cada byte representa
 * una fila del carácter.
 */
extern const uint8_t MAX7219_Dot_Matrix_font[256][8];

#ifdef __cplusplus
}
#endif

#endif /* MAX7219_H_ */
