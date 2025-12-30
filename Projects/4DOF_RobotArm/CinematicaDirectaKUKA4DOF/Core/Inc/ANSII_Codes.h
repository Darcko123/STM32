/*
 * ANSII_Codes.h
 *
 *  Created on: Dec 29, 2025
 *      Author: ruizd
 */

#ifndef INC_ANSII_CODES_H_
#define INC_ANSII_CODES_H_

// ============================================================================
// CÓDIGOS ANSII
// ============================================================================

// Colores de texto
#define COLOR_RESET         "\033[0m"
#define COLOR_BOLD          "\033[1m"
#define COLOR_DIM           "\033[2m"
#define COLOR_UNDERLINE     "\033[4m"

// Colores básicos
#define COLOR_BLACK         "\033[30m"
#define COLOR_RED           "\033[31m"
#define COLOR_GREEN         "\033[32m"
#define COLOR_YELLOW        "\033[33m"
#define COLOR_BLUE          "\033[34m"
#define COLOR_MAGENTA       "\033[35m"
#define COLOR_CYAN          "\033[36m"
#define COLOR_WHITE         "\033[37m"

// Colores brillantes
#define COLOR_BRIGHT_BLACK  "\033[90m"
#define COLOR_BRIGHT_RED    "\033[91m"
#define COLOR_BRIGHT_GREEN  "\033[92m"
#define COLOR_BRIGHT_YELLOW "\033[93m"
#define COLOR_BRIGHT_BLUE   "\033[94m"
#define COLOR_BRIGHT_MAGENTA "\033[95m"
#define COLOR_BRIGHT_CYAN   "\033[96m"
#define COLOR_BRIGHT_WHITE  "\033[97m"

// Colores de fondo
#define BG_BLACK           "\033[40m"
#define BG_RED             "\033[41m"
#define BG_GREEN           "\033[42m"
#define BG_YELLOW          "\033[43m"
#define BG_BLUE            "\033[44m"
#define BG_MAGENTA         "\033[45m"
#define BG_CYAN            "\033[46m"
#define BG_WHITE           "\033[47m"

// Colores de fondo brillantes
#define BG_BRIGHT_BLACK    "\033[100m"
#define BG_BRIGHT_RED      "\033[101m"
#define BG_BRIGHT_GREEN    "\033[102m"
#define BG_BRIGHT_YELLOW   "\033[103m"
#define BG_BRIGHT_BLUE     "\033[104m"
#define BG_BRIGHT_MAGENTA  "\033[105m"
#define BG_BRIGHT_CYAN     "\033[106m"
#define BG_BRIGHT_WHITE    "\033[107m"

// Control de cursor y pantalla
#define CLEAR_SCREEN        "\033[2J"
#define CLEAR_LINE          "\033[2K"
#define CLEAR_TO_END        "\033[0J"
#define CLEAR_TO_LINE_END   "\033[0K"
#define CURSOR_HOME         "\033[H"
#define CURSOR_SAVE         "\033[s"
#define CURSOR_RESTORE      "\033[u"
#define CURSOR_HIDE         "\033[?25l"
#define CURSOR_SHOW         "\033[?25h"

// Posicionamiento del cursor
#define CURSOR_UP(n)        "\033[" #n "A"
#define CURSOR_DOWN(n)      "\033[" #n "B"
#define CURSOR_RIGHT(n)     "\033[" #n "C"
#define CURSOR_LEFT(n)      "\033[" #n "D"
#define CURSOR_POS(row,col) "\033[" #row ";" #col "H"

#endif /* INC_ANSII_CODES_H_ */
