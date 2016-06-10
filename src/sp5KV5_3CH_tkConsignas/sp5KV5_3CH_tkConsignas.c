/*
 * sp5KV5_3CH_tkConsignas.c
 *
 *  Created on: 1 de jun. de 2016
 *      Author: pablo
 */


#include "../sp5KV5_3CH.h"
#include "sp5KV5_3CH_tkConsignas.h"

//------------------------------------------------------------------------------------
void tkConsignas(void * pvParameters)
{

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("starting tkConsignas..\r\n\0"));
	FreeRTOS_write( &pdUART1, cons_printfBuff, sizeof(cons_printfBuff) );

	initDobleConsigna();
	//
	for( ;; )
	{

		u_clearWdg(WDG_CSG);

		// Espero hasta 100ms por un mensaje.
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 100 / portTICK_RATE_MS ) );
		// Si llego un mensaje, prendo la flag correspondiente.
		if ( xResult == pdTRUE ) {
			if ( ( ulNotifiedValue & TK_PARAM_RELOAD ) != 0 ) {
				// Inicializo los subisitemas
				initDobleConsigna();
				initConsignaContinua();
			}
		}

		switch ( systemVars.consigna.type ) {
		case CONSIGNA_OFF:
			sm_CONSIGNAOFF();
			break;
		case CONSIGNA_DOBLE:
			sm_DOBLECONSIGNA();
			break;
		case CONSIGNA_CONTINUA:
			sm_CONSIGNACONTINUA();
			break;
		default:
			snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("tkConsignas::ERROR type(%d) NOT DEFINED..\r\n\0"),systemVars.consigna.type);
	 		FreeRTOS_write( &pdUART1, cons_printfBuff,sizeof(cons_printfBuff) );
	 		systemVars.consigna.type  = CONSIGNA_OFF;
	 		break;
		}
	}
}
//------------------------------------------------------------------------------------
void pv_consignaPrintExitMsg(u08 code)
{

u32 tickCount;

	tickCount = xTaskGetTickCount();
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(".[%06lu] tkConsigna::exit %02d\r\n\0"), tickCount,code);
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );
}
//------------------------------------------------------------------------------------
