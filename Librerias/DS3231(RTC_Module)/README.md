# Librería para el módulo RTC DS3231 en STM32

## Descripción
Esta librería proporciona funciones para interactuar con el módulo RTC DS3231 mediante comunicación I2C en microcontroladores STM32. Permite configurar y leer la hora, fecha y alarmas, así como ajustar sus modos de funcionamiento.

## Características
- Configuración y lectura de hora y fecha.
- Configuración y lectura de Alarmas 1 y 2.
- Conversión automática entre formatos BCD y decimal.

## Estructuras

### `TIME`
```c
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
    uint8_t dayofmonth;
    uint8_t month;
    uint8_t year;
} TIME;
```
**Descripción:** Almacena la hora y fecha obtenida del módulo.

### `ALARM1`
```c
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
} ALARM1;
```
**Descripción:** Almacena los datos de configuración de la Alarma 1.

### `ALARM2`
```c
typedef struct {
    uint8_t minutes;
    uint8_t hour;
} ALARM2;
```
**Descripción:** Almacena los datos de configuración de la Alarma 2.

## Funciones

### Configuración y lectura de hora
```c
//En SetTime se puede configurar el día de la semana, pero por el Algoritmo de Zeller es más fácil y preciso obtener el día de la semana, de manera que se omite en la función
void RTC_SetTime(uint8_t hour, uint8_t min, uint8_t sec, uint8_t dom, uint8_t month, uint8_t year);
void RTC_GetTime(TIME *time);
```

### Configuración y lectura de Alarma 1
```c
void RTC_SetAlarm1(uint8_t hourAlarm, uint8_t minAlarm, uint8_t secAlarm);
void RTC_GetAlarm1(ALARM1 *alarma1);
```

### Configuración y lectura de Alarma 2
```c
void RTC_SetAlarm2(uint8_t hourAlarm, uint8_t minAlarm);
void RTC_GetAlarm2(ALARM2 *alarma2);
```

## Ejemplo de uso

### Configurar y leer la hora
```c
TIME currentTime;
RTC_SetTime(12, 45, 30, 28, 12, 24); // 12:45:30, 28 de diciembre de 2024
RTC_GetTime(&currentTime);
```

### Configurar Alarma 1
```c
// Configurar Alarma 1 para activarse a las 08:30:15
RTC_SetAlarm1(8, 30, 15);
ALARM1 alarma1;
RTC_GetAlarm1(&alarma1);
```

### Configurar Alarma 2
```c
// Configurar Alarma 2 para activarse a las 09:45
RTC_SetAlarm2(9, 45);
ALARM2 alarma2;
RTC_GetAlarm2(&alarma2);
```

## Requisitos
- STM32 HAL Library
- Comunicación I2C habilitada

---

## Changelog

Todos los cambios notables de esta librería se documentan en esta sección.
El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).

---

### [1.1.0] - 22-09-2025

#### Added
- Manejador de la interfaz I2C como parámetro en las funciones de inicialización y lectura.
- Soporte para múltiples instancias del RTC mediante el paso del handle I2C.

#### Changed
- Modificación de las funciones públicas para recibir el handle I2C como argumento.

---

### [1.0.0] - 07-01-2025

#### Added
- Versión inicial con funciones básicas de inicialización, lectura y escritura de datos.
- Soporte para configuración de hora y fecha mediante `RTC_SetTime()`.
- Soporte para lectura de hora y fecha mediante `RTC_GetTime()`.
- Configuración y lectura de Alarma 1 (`RTC_SetAlarm1()`, `RTC_GetAlarm1()`).
- Configuración y lectura de Alarma 2 (`RTC_SetAlarm2()`, `RTC_GetAlarm2()`).
- Estructuras `TIME`, `ALARM1` y `ALARM2` para almacenamiento de datos.
- Conversión automática entre formato BCD y decimal.