/*
 * sp5KV5_3CH_tkGprs_6data.c
 *
 *  Created on: 24 de may. de 2016
 *      Author: pablo
 */


#include "../sp5KV5_3CH.h"
#include "sp5KV5_3CH_tkGprs.h"

static int gTR_G00(void);

// Eventos locales
typedef enum {
	g_ev_CTIMER_NOT_0 = 0,

} t_eventos_ssdata;

#define sm_DATA_EVENT_COUNT 1

static u08 cTimer;

//------------------------------------------------------------------------------------
void sm_DATAFRAME(void)
{
s08 g_eventos[sm_DATA_EVENT_COUNT];
u08 i;

	// Inicializo la lista local de eventos.
	for ( i=0; i < sm_DATA_EVENT_COUNT; i++ ) {
		g_eventos[i] = FALSE;
	}

	// Evaluo solo los eventos del estado STANDBY.

	// Corro la FSM
	switch ( GPRS_stateVars.state.subState ) {
	case gSST_DATAFRAME_00:
		GPRS_stateVars.state.subState = gTR_G00();
		break;

	default:
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\ntkGprs::ERROR sst_gprsData: subState  (%d) NOT DEFINED\r\n\0"),GPRS_stateVars.state.subState );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		pv_cambiarEstado(gST_DATAFRAME,gST_MODEMAPAGADO);
		break;
	}
}
//------------------------------------------------------------------------------------
static int gTR_G00(void)
{

	// Evento inicial. Solo salta al primer estado operativo.

	g_printExitMsg("G00\0");
	return(gSST_DATAFRAME_01);
}
//------------------------------------------------------------------------------------
