/*
 * sp5KV3_tkDigitalIn.c
 *
 *  Created on: 13/4/2015
 *      Author: pablo
 *
 *  La nueva modalidad es por poleo.
 *  Configuro el MCP para que no interrumpa
 *  C/100ms leo el registro GPIO del MCP.
 *  En reposo la salida de los latch es 1 por lo que debo detectar cuando se hizo 0.
 *  Para evitar poder quedar colgado, c/ciclo borro el latch.
 *  Esto implica que no importa la duracion del pulso ya que lo capturo con un flip-flop, pero
 *  no pueden venir mas rapido que 10/s.
 *
 *	Esta version solo puede usarse con placas SP5K_3CH que tengan latch para los pulsos, o sea
 *	version >= R003.
 *
 */


#include "sp5KV5_3CH.h"

#if defined(OSE_3CH) || defined(UTE_8CH)

static void pv_clearQ(void);
static void pv_pollQ(void);

static char dIn_printfBuff[CHAR64];	// Buffer de impresion
static dinData_t digIn;				// Estructura local donde cuento los pulsos.
static float ticksUp0,ticksUp1;

#define MAX_PULSESXSEC	4
#define DEBOUNCE_TIME_MS 30
#define LOOPTIME (( 1000 / MAX_PULSESXSEC ) - DEBOUNCE_TIME_MS )

/*------------------------------------------------------------------------------------*/
void tkDigitalIn(void * pvParameters)
{

( void ) pvParameters;
u08 i = 0;
TickType_t xLastWakeTime;
const TickType_t xFrequency = 25;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("starting tkDigitalIn..\r\n\0"));
	FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );

	// Inicializo los latches borrandolos
	pv_clearQ();
	for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++) {
		digIn.level[i] = 0;
		digIn.pulses[i] = 0;
		digIn.secsUp[i] = 0;
	}

	ticksUp0 = 0;
	ticksUp1 = 0;

	//xLastWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		u_clearWdg(WDG_DIN);

		// Espero 250ms ( hasta 4 pulsos por seg.)
		vTaskDelay( ( TickType_t)( LOOPTIME / portTICK_RATE_MS ) );
		//vTaskDelayUntil( &xLastWakeTime, xFrequency );
		//xLastWakeTime = xTaskGetTickCount();

		// Solo poleo las entradas en modo normal. En modo service no para
		// poder manejarlas por los comandos de servicio.
		if ( ( systemVars.wrkMode == WK_NORMAL) || ( systemVars.wrkMode == WK_MONITOR_FRAME ) ) {
			pv_pollQ();
		}
	}

}
/*------------------------------------------------------------------------------------*/
void u_readDigitalCounters( dinData_t *dIn , s08 resetCounters )
{
	// copio los valores de los contadores en la estructura dIn.
	// Si se solicita, luego se ponen a 0.

u08 i = 0;

	// Si estoy en UTE_8CH, leo el nivel de las entradas digitales en
	// este momento

#if defined (OSE_3CH) || defined (OSE_POZOS)
	// Levels: El firmware de OSE_3CH no puede leer niveles.
	digIn.level[0] = 0;
	digIn.level[1] = 0;
#endif

#ifdef UTE_8CH
	digIn.level[0] = ( D0_LV_PIN & _BV(D0_LV) ) >> D0_LV;
	digIn.level[1] = ( D2_LV_PIN & _BV(D2_LV) ) >> D2_LV;
#endif

	digIn.secsUp[0] = (u16)(ticksUp0);
	if ( digIn.secsUp[0] > systemVars.timerPoll ) {
		digIn.secsUp[0] = systemVars.timerPoll;
	}
	digIn.secsUp[1] = (u16)(ticksUp1);
	if ( digIn.secsUp[1] > systemVars.timerPoll ) {
		digIn.secsUp[1] = systemVars.timerPoll;
	}

	memcpy( dIn, &digIn, sizeof(dinData_t)) ;
	if ( resetCounters == TRUE ) {
		for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++) {
			digIn.level[i] = 0;
			digIn.pulses[i] = 0;
			digIn.secsUp[i] = 0;
		}
	}

	ticksUp0 = 0;
	ticksUp1 = 0;
}
/*------------------------------------------------------------------------------------*/
static void pv_pollQ(void)
{

s08 retS = FALSE;
u08 din0 = 0;
u08 din1 = 0;
u08 din2 = 0;
u08 din3 = 0;
s08 debugQ = FALSE;
u32 tickCount;
u16 level0,level1;

#if defined(OSE_3CH) || defined(OSE_POZOS)
	// Leo el GPIO.
	retS = MCP_query2Din( &din0, &din1 );
#endif

#ifdef UTE_8CH
	// Leo los pulsos.
	retS = TRUE;
	din0 = ( D0_IN_PIN & _BV(D0_IN) ) >> D0_IN;
//	din1 = ( D1_IN_PIN & _BV(D1_IN) ) >> D1_IN;
	din1 = ( D2_IN_PIN & _BV(D2_IN) ) >> D2_IN;
//	din3 = ( D3_IN_PIN & _BV(D3_IN) ) >> D3_IN;

	// Leo los niveles y si son 0, sumo un intervalo de 100ms al contador
	level0 = ( D0_LV_PIN & _BV(D0_LV) ) >> D0_LV;
	level1 = ( D2_LV_PIN & _BV(D2_LV) ) >> D2_LV;

	// c/intervalo es 1/4 secs.
	if ( level0 == 0 ) { ticksUp0 += 0.25; }
	if ( level1 == 0 ) { ticksUp0 += 0.25; }

#endif

	if ( retS ) {
		// Counts
		debugQ = FALSE;
		if (din0 == 0 ) { digIn.pulses[0]++ ; debugQ = TRUE;}
		if (din1 == 0 ) { digIn.pulses[1]++ ; debugQ = TRUE;}
	} else {
		snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("tkDigitalIn: READ DIN ERROR !!\r\n\0"));
		u_debugPrint(D_DIGITAL, dIn_printfBuff, sizeof(dIn_printfBuff) );
		goto quit;
	}

	if ( ((systemVars.debugLevel & D_DIGITAL) != 0) && debugQ ) {
		tickCount = xTaskGetTickCount();
		snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR(".[%06lu] tkDigitalIn: din0=%.0f(%d)[.0f],din1=%.0f(%d)[.0f]\r\n\0"), tickCount, digIn.pulses[0],din0,ticksUp0, digIn.pulses[1],din1,ticksUp1 );
		u_debugPrint(( D_BASIC + D_DIGITAL ), dIn_printfBuff, sizeof(dIn_printfBuff) );
	}

quit:
	// Siempre borro los latches para evitar la posibilidad de quedar colgado.
	pv_clearQ();
	return;

}
/*------------------------------------------------------------------------------------*/
static void pv_clearQ(void)
{
	// Pongo un pulso 1->0->1 en Q0/Q1 pin para resetear el latch
	// En reposo debe quedar en H.

#if defined(OSE_3CH) || defined (OSE_POZOS)
	cbi(Q_PORT, Q0_CTL_PIN);
	cbi(Q_PORT, Q1_CTL_PIN);
	vTaskDelay( ( TickType_t)( DEBOUNCE_TIME_MS / portTICK_RATE_MS ) );
	//taskYIELD();
	//_delay_us(5);
	//asm("nop");
	sbi(Q_PORT, Q0_CTL_PIN);
	sbi(Q_PORT, Q1_CTL_PIN);
#endif

#ifdef UTE_8CH
	cbi(D0_CLR_PORT, D0_CLR);
	cbi(D1_CLR_PORT, D1_CLR);
	cbi(D2_CLR_PORT, D2_CLR);
	cbi(D3_CLR_PORT, D3_CLR);

	vTaskDelay( ( TickType_t)( DEBOUNCE_TIME_MS / portTICK_RATE_MS ) );
	//taskYIELD();
	//_delay_us(5);
	//asm("nop");
	sbi(D0_CLR_PORT, D0_CLR);
	sbi(D1_CLR_PORT, D1_CLR);
	sbi(D2_CLR_PORT, D2_CLR);
	sbi(D3_CLR_PORT, D3_CLR);
#endif

}
/*------------------------------------------------------------------------------------*/
#endif
