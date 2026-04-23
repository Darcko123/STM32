/**
 * @file CME6005.c
 *
 * @brief Implementación del driver para decodificación WWVB con el CME6005.
 *
 * @author Daniel Ruiz
 * @date   Abril 20, 2026
 * @version 1.0.0
 */

#include "CME6005.h"
#include <string.h>

// ============================================================================
// DEFINICIONES INTERNAS
// ============================================================================

/** @brief Valor que indica un tipo de bit "0" (pulso ~200 ms). */
#define BIT_CERO 0U

/** @brief Valor que indica un tipo de bit "1" (pulso ~500 ms). */
#define BIT_UNO 1U

/** @brief Valor que indica un marcador de posición (pulso ~800 ms). */
#define BIT_MARCADOR 2U

/**
 * @brief Posiciones de los marcadores de posición dentro de la trama WWVB.
 *        P0=0, P1=9, P2=19, P3=29, P4=39, P5=49.
 *        El segundo 59 es el marcador de referencia de trama (no lleva datos).
 */
static const uint8_t POSICIONES_MARCADOR[] = {0U, 9U, 19U, 29U, 39U, 49U, 59U};
#define NUM_MARCADORES                                                         \
		(sizeof(POSICIONES_MARCADOR) / sizeof(POSICIONES_MARCADOR[0]))

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static GPIO_TypeDef *PON_GPIO_Port = NULL;
static uint16_t PON_GPIO_Pin = 0;
static GPIO_TypeDef *NTCO_GPIO_Port = NULL;
static uint16_t NTCO_GPIO_Pin = 0;

static WWVB_State_t CME6005_estado = WWVB_STATE_POWER_DOWN;
static uint8_t CME6005_trama[WWVB_FRAME_BITS];
static uint8_t CME6005_bit_actual = 0;
static uint32_t CME6005_inicio_pulso = 0;
static uint8_t CME6005_marcadores_consecutivos = 0;
static WWVB_Tiempo_t CME6005_tiempo;
static uint8_t CME6005_Initialized = 0;

// ============================================================================
// FUNCIONES PRIVADAS (IMPLEMENTACIÓN)
// ============================================================================

/**
 * @brief Clasifica un pulso de NTCO según su duración.
 *
 * @param duracion_ms Duración medida en milisegundos.
 * @return BIT_CERO, BIT_UNO o BIT_MARCADOR.
 */
static uint8_t clasificar_pulso(uint32_t duracion_ms)
{
	if (duracion_ms < WWVB_BIT0_THRESHOLD_MS)
	{
		return BIT_CERO;
	}
	else if (duracion_ms < WWVB_BIT1_THRESHOLD_MS)
	{
		return BIT_UNO;
	}
	else
	{
		return BIT_MARCADOR;
	}
}

/**
 * @brief Indica si la posición dada debe contener un marcador de posición.
 *
 * @param pos Índice del bit (0 - 59).
 * @return 1 si es posición de marcador, 0 si no.
 */
static uint8_t es_posicion_marcador(uint8_t pos)
{
	for (uint8_t i = 0U; i < NUM_MARCADORES; i++)
	{
		if (pos == POSICIONES_MARCADOR[i])
		{
			return 1U;
		}
	}
	return 0U;
}

/**
 * @brief Decodifica la trama WWVB almacenada en CME6005_trama[].
 *
 *        Formato de trama WWVB (NIST):
 *
 *        Bit  Peso      Campo
 *        ---  --------  -----
 *          0    -       Marcador P0
 *        1-3  40,20,10  Minutos (decenas BCD)
 *          4    -       Sin uso
 *        5-8  8,4,2,1   Minutos (unidades BCD)
 *          9    -       Marcador P1
 *       10-11   -       Sin uso
 *       12-13 20,10     Horas (decenas BCD)
 *         14    -       Sin uso
 *       15-18  8,4,2,1  Horas (unidades BCD)
 *         19    -       Marcador P2
 *       20-21   -       Sin uso
 *       22-23 200,100   Día del año (centenas BCD)
 *         24    -       Sin uso
 *       25-28 80,40,20,10  Día del año (decenas BCD)
 *         29    -       Marcador P3
 *       30-33  8,4,2,1  Día del año (unidades BCD)
 *       34-35   -       Sin uso
 *       36-38  +,-,-    Signo UTI (bit36='+', bit37='-')
 *         39    -       Marcador P4
 *       40-43 0.8,0.4,0.2,0.1  Corrección UTI (en décimas de segundo)
 *         44    -       Sin uso
 *       45-48 80,40,20,10  Año (decenas BCD)
 *         49    -       Marcador P5
 *       50-53  8,4,2,1  Año (unidades BCD)
 *         54    -       Indicador año bisiesto (LYI)
 *         55    -       Advertencia segundo intercalar (LSW)
 *       56-57   -       Bits DST (00=estándar, 10=verano, 01=termina hoy,
 *                                 11=comienza hoy)
 *         58    -       Sin uso
 *         59    -       Marcador de referencia de trama (FRM)
 */
static void decodificar_trama(void)
{
	const uint8_t *b = CME6005_trama;
	WWVB_Tiempo_t *t = &CME6005_tiempo;

	/* ---- Minutos (bits 1-8) ---- */
	t->minutos = (uint8_t)(b[1] * 40U + b[2] * 20U + b[3] * 10U + b[5] * 8U +
			b[6] * 4U + b[7] * 2U + b[8]);

	/* ---- Horas (bits 12-18) ---- */
	t->horas = (uint8_t)(b[12] * 20U + b[13] * 10U + b[15] * 8U + b[16] * 4U +
			b[17] * 2U + b[18]);

	/* ---- Día del año (bits 22-33) ---- */
	t->dia_del_anio = (uint16_t)(b[22] * 200U + b[23] * 100U + b[25] * 80U +
			b[26] * 40U + b[27] * 20U + b[28] * 10U +
			b[30] * 8U + b[31] * 4U + b[32] * 2U + b[33]);

	/* ---- Año (bits 45-53) ---- */
	t->anio = (uint8_t)(b[45] * 80U + b[46] * 40U + b[47] * 20U + b[48] * 10U +
			b[50] * 8U + b[51] * 4U + b[52] * 2U + b[53]);

	/* ---- Corrección UTI (bits 36-43) ---- */
	int8_t signo_uti = (b[36] == BIT_UNO) ? 1 : -1;
	uint8_t valor_uti = (uint8_t)(b[40] * 8U + b[41] * 4U + b[42] * 2U + b[43]);
	t->correccion_uti = signo_uti * (int8_t)valor_uti;

	/* ---- Indicadores (bits 54-57) ---- */
	t->anio_bisiesto = b[54];
	t->segundo_intercalar = b[55];
	t->dst = (uint8_t)((b[56] << 1U) | b[57]);
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

CME6005_Status_t CME6005_Init(GPIO_TypeDef* pon_port,  uint16_t pon_pin,
		                          GPIO_TypeDef* ntco_port, uint16_t ntco_pin)
{
	if (pon_port == NULL || ntco_port == NULL)
	{
		return CME6005_ERROR;
	}

	PON_GPIO_Port = pon_port;
	PON_GPIO_Pin = pon_pin;
	NTCO_GPIO_Port = ntco_port;
	NTCO_GPIO_Pin = ntco_pin;

	CME6005_estado = WWVB_STATE_POWER_DOWN;

	/* Asegurar que el módulo inicie apagado */
	HAL_GPIO_WritePin(PON_GPIO_Port, PON_GPIO_Pin, GPIO_PIN_SET);

  CME6005_Initialized = 1;
	return CME6005_OK;
}

CME6005_Status_t CME6005_PowerOn(void)
{
	if (CME6005_Initialized != 1)
	{
		return CME6005_NOT_INITIALIZED;
	}

	/* PON activo en LOW → enciende el receptor */
	HAL_GPIO_WritePin(PON_GPIO_Port, PON_GPIO_Pin, GPIO_PIN_RESET);

	/* Esperar estabilización interna del CME6005 (típico 0.5 s, máx 2 s) */
	HAL_Delay(CME6005_POWER_ON_DELAY_MS);

	CME6005_estado = WWVB_STATE_SYNCING;
	CME6005_bit_actual = 0U;
	CME6005_marcadores_consecutivos = 0U;

	return CME6005_OK;
}

CME6005_Status_t CME6005_PowerDown(void)
{
	if (CME6005_Initialized != 1)
	{
		return CME6005_NOT_INITIALIZED;
	}

	/* PON en HIGH → modo Power Down (consumo <0.05 µA) */
	HAL_GPIO_WritePin(PON_GPIO_Port, PON_GPIO_Pin, GPIO_PIN_SET);
	CME6005_estado = WWVB_STATE_POWER_DOWN;

	return CME6005_OK;
}

CME6005_Status_t CME6005_Reset(void)
{
	if (CME6005_Initialized != 1)
	{
		return CME6005_NOT_INITIALIZED;
	}

	CME6005_estado = WWVB_STATE_SYNCING;
	CME6005_bit_actual = 0U;
	CME6005_marcadores_consecutivos = 0U;
	memset(CME6005_trama, 0, sizeof(CME6005_trama));

	return CME6005_OK;
}

void CME6005_NTCO_RisingEdgeCallback(void)
{
	if ((CME6005_Initialized != 1) || (CME6005_estado == WWVB_STATE_POWER_DOWN))
	{
		return;
	}

	/*
	 * Flanco de subida de NTCO = inicio del segundo WWVB.
	 * La portadora del transmisor cae → el IC eleva NTCO.
	 * Registrar el timestamp para medir la duración del pulso.
	 */
	CME6005_inicio_pulso = HAL_GetTick();
}

void CME6005_NTCO_FallingEdgeCallback(void)
{
	if (!CME6005_Initialized || CME6005_estado == WWVB_STATE_POWER_DOWN)
	{
		return;
	}

	uint32_t duracion_ms = HAL_GetTick() - CME6005_inicio_pulso;
	uint8_t tipo_bit = clasificar_pulso(duracion_ms);

	switch (CME6005_estado)
	{
		case WWVB_STATE_SYNCING:
			/* ---------------------------------------------------------- */
			if (tipo_bit == BIT_MARCADOR)
			{
				CME6005_marcadores_consecutivos++;

				/*
				 * Se requieren dos marcadores 800 ms seguidos:
				 *   - Segundo 59: marcador de referencia de trama (FRM)
				 *   - Segundo  0: marcador P0
				 * Tras eso, el siguiente pulso corresponde al bit 1.
				 */
				if (CME6005_marcadores_consecutivos >= 2U)
				{
					CME6005_trama[0] = BIT_MARCADOR;
					CME6005_bit_actual = 1U;
					CME6005_marcadores_consecutivos = 0U;
					CME6005_estado = WWVB_STATE_RECEIVING;
				}
			}
			else
			{
				CME6005_marcadores_consecutivos = 0U;
			}
			break;

			/* ---------------------------------------------------------- */
		case WWVB_STATE_RECEIVING:
			/* ---------------------------------------------------------- */
			if (CME6005_bit_actual >= WWVB_FRAME_BITS)
			{
				/* No debería ocurrir; reiniciar sincronización */
				CME6005_Reset();
				break;
			}

			CME6005_trama[CME6005_bit_actual] = tipo_bit;

			/* Validar marcadores en sus posiciones esperadas (P1-P5) */
			if (es_posicion_marcador(CME6005_bit_actual))
			{
				if (tipo_bit != BIT_MARCADOR)
				{
					/* Marcador esperado pero llegó un bit de datos → error */
					CME6005_Reset();
					break;
				}
			}

			CME6005_bit_actual++;

			if (CME6005_bit_actual >= WWVB_FRAME_BITS)
			{
				decodificar_trama();
				CME6005_estado = WWVB_STATE_FRAME_READY;
			}
			break;

			/* ---------------------------------------------------------- */
		default:
			break;
			/* ---------------------------------------------------------- */
	}
}

uint8_t CME6005_IsFrameReady(void)
{
	if (CME6005_Initialized != 1)
	{
		return 0U;
	}
	return (CME6005_estado == WWVB_STATE_FRAME_READY) ? 1U : 0U;
}

CME6005_Status_t CME6005_GetTime(WWVB_Tiempo_t* tiempo_out)
{
	if (CME6005_Initialized != 1)
	{
		return CME6005_NOT_INITIALIZED;
	}

	if (CME6005_estado != WWVB_STATE_FRAME_READY)
	{
		return CME6005_NOT_READY;
	}

	if (tiempo_out != NULL)
	{
		memcpy(tiempo_out, &CME6005_tiempo, sizeof(WWVB_Tiempo_t));
	}

	/* Preparar para la siguiente trama sin volver a buscar sincronía */
	CME6005_bit_actual = 0U;
	CME6005_estado = WWVB_STATE_SYNCING;

	return CME6005_OK;
}
