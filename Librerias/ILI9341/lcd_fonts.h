// 2014, Petr Machala

#ifndef __LCD_FONTS_H
#define __LCD_FONTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FONT1    LCD_Font_7x10
#define FONT2    LCD_Font_11x18
#define FONT3    LCD_Font_16x26

// Declaraciones de los arrays de fuentes
extern const uint16_t LCD_Font7x10[];
extern const uint16_t LCD_Font11x18[];
extern const uint16_t LCD_Font16x26[];

// Font struct
typedef struct {
    uint8_t FontWidth;		// Ancho del caracter en pixeles
    uint8_t FontHeight;		// Alto del caracter en pixeles
    const uint16_t *data;	// Puntero a los datos de la fuente
} LCD_FontDef_t;

// Variables globales de fuentes
extern LCD_FontDef_t LCD_Font_7x10;
extern LCD_FontDef_t LCD_Font_11x18;
extern LCD_FontDef_t LCD_Font_16x26;

#ifdef __cplusplus
}
#endif

#endif /* __LCD_FONTS_H */
