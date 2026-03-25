# ANSII Codes Library 

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/Platform-STM32-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)](https://github.com/Darcko123/STM32/tree/main/Librerias/ANSII_Codes)

---
## Tabla de Contenidos
- [ANSII Codes Library](#ansii-codes-library)
  - [Tabla de Contenidos](#tabla-de-contenidos)
  - [Descripción](#descripción)
  - [Características](#características)
  - [Instalación](#instalación)
  - [Uso](#uso)
    - [Ejemplo Básico - Colores](#ejemplo-básico---colores)
    - [Tabla de Colores (Foreground)](#tabla-de-colores-foreground)
    - [Estilos y Control](#estilos-y-control)
  - [Compatibilidad](#compatibilidad)
  - [Ejemplos en Proyectos](#ejemplos-en-proyectos)
  - [Licencia](#licencia)
  - [Changelog](#changelog)
    - [Versión 1.0.0](#versión-100)

---

## Descripción

La librería **ANSII_Codes** proporciona macros para códigos de escape ANSI que permiten formatear la salida de texto en terminales compatibles con ANSI. Ideal para depuración avanzada y telemetría en proyectos STM32 mediante UART/printf.

Esta librería enriquece la salida de `printf()` con colores, estilos y control de cursor/pantalla, haciendo más legible la información en consolas como PuTTY, Linux Terminal o VSCode Serial Monitor (con extensiones).

---

## Características

- **Colores de texto**: 16 colores (8 básicos + 8 brillantes) para primer plano.
- **Colores de fondo**: 16 colores para fondo.
- **Estilos**: Negrita, subrayado, tenue.
- **Control de pantalla**: Limpiar pantalla/línea, posición cursor.
- **Control de cursor**: Mover arriba/abajo/izq/der, guardar/restaurar posición, ocultar/mostrar.

## Instalación

1. Copia `ANSII_Codes.h` a `Core/Inc/` en tu proyecto STM32CubeIDE.
2. Incluye en tu código: `#include "ANSII_Codes.h"`

## Uso

Los macros generan strings de escape ANSI. Concaténalos con `sprintf`.

### Ejemplo Básico - Colores
```c
#include "ANSII_Codes.h"
#include "stdio.h"
#include "stdlib.h"

char buffer[20];

int main() {
    sprintf(buffer, "%s¡Éxito!%s\n\r", COLOR_GREEN, COLOR_RESET);
    sprintf(buffer, "%s%sERROR crítico%s\n\r", COLOR_BOLD, COLOR_RED);
    return 0;
}
```

**Salida:**
```
¡Éxito! (en verde)
ERROR crítico (en rojo negrita)
```

### Tabla de Colores (Foreground)
| Macro              | Código | Ejemplo                  |
|--------------------|--------|--------------------------|
| `COLOR_BLACK`      | 30     | Negro                    |
| `COLOR_RED`        | 31     | Rojo                     |
| `COLOR_GREEN`      | 32     | Verde                   |
| `COLOR_YELLOW`     | 33     | Amarillo                |
| `COLOR_BLUE`       | 34     | Azul                    |
| `COLOR_MAGENTA`    | 35     | Magenta                 |
| `COLOR_CYAN`       | 36     | Cian                    |
| `COLOR_WHITE`      | 37     | Blanco                  |
| `COLOR_BRIGHT_RED` | 91     | Rojo brillante          |

*(Ver header completo para backgrounds `BG_*` y brillantes `BG_BRIGHT_*`)*

### Estilos y Control
| Macro              | Descripción              |
|--------------------|--------------------------|
| `COLOR_RESET`      | Restablece todo         |
| `COLOR_BOLD`       | Negrita                  |
| `COLOR_UNDERLINE`  | Subrayado                |
| `CLEAR_SCREEN`     | Limpia pantalla completa |
| `CURSOR_HIDE`      | Oculta cursor            |
| `CURSOR_SHOW`      | Muestra cursor           |

**Macros de cursor dinámicos:** `CURSOR_UP(n)` → `\033[nA`

## Compatibilidad

- PuTTY, Linux Console, Git Bash
- VSCode Serial Monitor (con ANSI support)
- Windows CMD: Limitado (usa Windows Terminal o WSL)
- STM32: Todos los MCUs con UART.

## Ejemplos en Proyectos

- [4DOF Robot Arm](https://github.com/Darcko123/STM32/tree/main/Projects/4DOF_RobotArm): Debug cinemática con colores.
- [KeypadWithAdvancedDisplay](https://github.com/Darcko123/STM32/tree/main/Projects/KeypadWithAdvancedDisplay): Telemetría con estilos.

---

## Licencia
Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE](/LICENSE.md) para más detalles.

---

## Changelog

### Versión 1.0.0
- Versión inicial de la librería ANSSI_Codes para STM32.
- Documentación básica con ejemplos de uso.