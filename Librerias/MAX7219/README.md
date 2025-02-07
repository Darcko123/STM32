# Control de Matriz de LEDs con MAX7219 en STM32

## Descripción
Esta librería permite controlar una matriz de LEDs mediante el controlador **MAX7219** utilizando comunicación **SPI** en un microcontrolador STM32. La implementación permite visualizar caracteres, desplazarlos y manipular la matriz de LEDs de manera eficiente.

Esta librería está basada en el código de **Controllerstech**:
[Cómo desplazar cadenas en una pantalla de matriz de puntos](https://controllerstech.com/how-to-scroll-string-on-dot-matrix-display/#info_box)

## Archivos
- **MAX7219.h**: Contiene las declaraciones de funciones y macros necesarias para la gestión de la matriz de LEDs.
- **MAX7219.c**: Implementación de las funciones de control del MAX7219, incluyendo inicialización, escritura de datos y manipulación de la matriz.

## Configuración del SPI en STM32
Para utilizar esta librería, se debe configurar el **SPI1** en modo **Half-Duplex Master** con los siguientes parámetros:

### **Parámetros Básicos**
- **Frame format:** Motorola
- **Data Size:** 16 Bits
- **First Bit:** MSB First

### **Parámetros del Reloj**
- **Prescaler (para Baud Rate):** 32
- **Baud Rate:** 2.25 MBits/s
- **Clock Polarity (CPOL):** High
- **Clock Phase (CPHA):** 1 Edge

### **Parámetros Avanzados**
- **CRC Calculation:** Disabled
- **NSS Signal Type:** Software

## Configuración de GPIO
El pin de **Chip Select (CS)** debe configurarse como una salida GPIO con las siguientes características:
- **Maximum output speed:** Low
- **GPIO output level:** High
- **User Label:** CS

## Uso de la Librería
### **Inicialización**
Antes de usar la matriz de LEDs, es necesario inicializar el MAX7219:
```c
MAX7219_Init();
```

### **Escribir en la Matriz**
Para escribir un byte de datos en una fila específica:
```c
max7219_write(3, 0xFF); // Escribe 0xFF en la fila 3
```

### **Escribir un texto fijo**
Para escribir un texto fijo en la matriz de leds:
```c
MAX7219_printString("Hola");
```

### **Mostrar un Texto Desplazado**
```c
MAX7219_scrollString("Hola mundo", 100);
```

### **Limpiar la Matriz**
```c
MAX7219_clearDisplay();
```

## Notas
- El número de dispositivos MAX7219 en cascada se define con `NUM_DEV` en **MAX7219.h**.
- La librería usa una tabla de fuentes de 8x8 almacenada en `MAX7219_Dot_Matrix_font`.

## Licencia
Este proyecto está licenciado bajo los términos que se encuentran en el archivo `LICENSE.md` en el directorio raíz del proyecto.

