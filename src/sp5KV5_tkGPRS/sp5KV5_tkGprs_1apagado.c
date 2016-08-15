/*
 * sp5KV4_8CH_tkGprs_ssOFF.c
 *
 *  Created on: 22 de abr. de 2016
 *      Author: pablo
 *
 *  En este estado el modem queda apagado y en espera que transcurra el tiempo
 *  para prenderlo.
 *  Si estamos en modo pwrMode continuo, esperamos 1 minuto.
 *  Si estamos en modo pwrMode discreto, el tiempo lo marca systemVars.timerDial.
 *  Para contar los segundos con exactitud, usamos un timer general que expira c/1s.
 *
 */

#include "../sp5KV5_3CH.h"
#include "sp5KV5_tkGprs.h"

static int gTR_A00(void);
static int gTR_A01(void);
static int gTR_A02(void);
static int gTR_A03(void);
static int gTR_A04(void);
static int gTR_A05(void);
static int gTR_A06(void);
static int gTR_A07(void);

// Eventos locales
typedef enum {
	a_ev_AWAIT_NOT_0 = 0,
	a_ev_MSGRELOAD,
	a_ev_FLOODING,
	a_ev_INSIDE_PWRSAVE,

} t_eventos_ssApagado;

#define sm_APAGADO_EVENT_COUNT 4

typedef enum { TINIT_INIT = 0, TINIT_MSGRELOAD, TINIT_NORMAL, TINIT_PWRSAVE } t_timerInit;

static void pv_configCTimer(t_timerInit);
static s08 pwrSave_f = FALSE;

//------------------------------------------------------------------------------------
void sm_APAGADO(void)
{
	// Maquina de estados del estado apagado del modem.
	// Se queda esperando con el modem apagado.

s08 a_eventos[sm_APAGADO_EVENT_COUNT];
u08 i;

	// Inicializo la lista local de eventos.
	for ( i=0; i < sm_APAGADO_EVENT_COUNT; i++ ) {
		a_eventos[i] = FALSE;
	}

	// Evaluo solo los eventos del estado APAGADO.
	if ( GPRS_stateVars.counters.awaitSecs > 0 ) { a_eventos[a_ev_AWAIT_NOT_0] = TRUE; }
	if ( GPRS_stateVars.flags.msgReload == TRUE ) { a_eventos[a_ev_MSGRELOAD] = TRUE; }
	if ( GPRS_stateVars.flags.msgFlooding == TRUE ) { a_eventos[a_ev_FLOODING] = TRUE; }
	if ( pwrSave_f == TRUE ) { a_eventos[a_ev_INSIDE_PWRSAVE] = TRUE; }

	// Corro la FSM
	switch ( GPRS_stateVars.state.subState ) {
	case gSST_MODEMAPAGADO_00:
		GPRS_stateVars.state.subState = gTR_A00();
		break;
	case gSST_MODEMAPAGADO_01:
		// MSG RELOAD
		if ( a_eventos[a_ev_MSGRELOAD] ) {
			GPRS_stateVars.state.subState = gTR_A01();
			break;
		}
		// FLOODING
		if ( a_eventos[a_ev_FLOODING] )  {
			GPRS_stateVars.state.subState = gTR_A03();
			break;
		}
		// CTIMER
		if ( a_eventos[a_ev_AWAIT_NOT_0] )  {
			GPRS_stateVars.state.subState = gTR_A02();
		} else {
			GPRS_stateVars.state.subState = gTR_A04();
		}
		break;
	case gSST_MODEMAPAGADO_02:
		if ( a_eventos[a_ev_INSIDE_PWRSAVE]) {
			GPRS_stateVars.state.subState = gTR_A05();
		} else {
			GPRS_stateVars.state.subState = gTR_A06();
		}
		break;
	case gSST_MODEMAPAGADO_03:
		GPRS_stateVars.state.subState = gTR_A07();
		break;
	default:
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\ntkGprs::ERROR sst_gprsApagado: subState  (%d) NOT DEFINED\r\n\0"),GPRS_stateVars.state.subState );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		pv_cambiarEstado(gST_MODEMAPAGADO,gST_MODEMAPAGADO);
		break;
	}
}
/*------------------------------------------------------------------------------------*/
static int gTR_A00(void)
{

static s08 inicio = TRUE;

	// Evento inicial. Solo salta al primer estado operativo.
	// Inicializo el sistema aqui

	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );
	strncpy_P(systemVars.dlgIp, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	//
	// Arranque del equipo
	if ( inicio ) {
		// Cuando recien estoy arrancando espero solo 15s para prender.
		// No importa el pwrSave.
		inicio = FALSE;
		pv_configCTimer(TINIT_INIT);
	} else {
		pv_configCTimer(TINIT_NORMAL);
	}

	// Apago el modem
	MODEM_HWpwrOff();

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Modem Apagado\r\n\0"));
	u_logPrint(gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("A00\0");
	return(gSST_MODEMAPAGADO_01);
}
//------------------------------------------------------------------------------------
static int gTR_A01(void)
{
	// Llego un mensaje de reconfiguracion.

	GPRS_stateVars.flags.msgReload = FALSE;

	// Re-calculo el tiempo que debo mantenerme apagado ( 15s ).
	pv_configCTimer(TINIT_MSGRELOAD);

	g_printExitMsg("A01\0");
	return(gSST_MODEMAPAGADO_01);
}
//------------------------------------------------------------------------------------
static int gTR_A02(void)
{
	// Me quedo con el modem apagado esperando.
	// Espero 1 segundo

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	//g_printExitMsg("A02\0");
	return(gSST_MODEMAPAGADO_01);
}
//------------------------------------------------------------------------------------
static int gTR_A03(void)
{
	// Tengo una alarma de flooding: debo discar enseguida.
	// En el caso de FW de pozos nunca se prende esta flag.

	GPRS_stateVars.flags.msgFlooding = FALSE;

	g_printExitMsg("A03\0");
	return(gSST_MODEMAPAGADO_03);
}
//------------------------------------------------------------------------------------
static int gTR_A04(void)
{
	// Evaluo si estoy dentro de un intervalo de pwrSave

RtcTimeType_t rtcDateTime;
u16 now;

	pwrSave_f = FALSE;

	// Si estoy en modo discreto con pwrSave habilitado
	if ( ( systemVars.pwrMode == PWR_DISCRETO ) && ( systemVars.pwrSave == modoPWRSAVE_ON ) ) {

		RTC_read(&rtcDateTime);
		now = rtcDateTime.hour * 60 + rtcDateTime.min;	// Hora actual en minutos desde las 00:00

		// Caso 1:
		if ( systemVars.pwrSaveStartTime < systemVars.pwrSaveEndTime ) {
			if ( ( now > systemVars.pwrSaveStartTime) && ( now < systemVars.pwrSaveEndTime) ) {
				pwrSave_f = TRUE;
				goto quit;
			}
		}

		// Caso 2:
		if ( systemVars.pwrSaveStartTime >= systemVars.pwrSaveEndTime ) {
			if ( ( now < systemVars.pwrSaveEndTime) || ( now > systemVars.pwrSaveStartTime ) ) {
				pwrSave_f = TRUE;
				goto quit;
			}
		}

	}

quit:

	//g_printExitMsg("A04\0");
	return(gSST_MODEMAPAGADO_02);

}
//------------------------------------------------------------------------------------
static int gTR_A05(void)
{
	// Estoy dentro de un intervalo de PWR SAVE. Espero 10m para volver
	// a chequear
	pwrSave_f = FALSE;
	pv_configCTimer(TINIT_PWRSAVE);

	g_printExitMsg("A05\0");
	return(gSST_MODEMAPAGADO_01);
}
//------------------------------------------------------------------------------------
static int gTR_A06(void)
{
	//g_printExitMsg("A06\0");
	return(gSST_MODEMAPAGADO_03);
}
//------------------------------------------------------------------------------------
static int gTR_A07(void)
{
	// Expiro el tiempo de espera. Salgo del estado apagado

	g_printExitMsg("A07\0");
	return( pv_cambiarEstado(gST_MODEMAPAGADO,gST_MODEMPRENDIENDO) );
}
//------------------------------------------------------------------------------------
static void pv_configCTimer(u08 modo)
{
	switch(modo) {
	case TINIT_INIT: // INIT
		GPRS_stateVars.counters.awaitSecs = 15;
		break;
	case TINIT_MSGRELOAD:	// MSG RELOAD
		GPRS_stateVars.counters.awaitSecs = 15;
		break;
	case TINIT_NORMAL:		// NORMAL( continuo o discreto )
		switch (systemVars.wrkMode ) {
		case WK_SERVICE:
			// En modo service me quedo en forma indefinida
			GPRS_stateVars.counters.awaitSecs = 0xFFFF;
			break;
		case PWR_CONTINUO:
			GPRS_stateVars.counters.awaitSecs = 60;
			break;
		case  PWR_DISCRETO:
			GPRS_stateVars.counters.awaitSecs = systemVars.timerDial;
			break;
		}
		break;
	case TINIT_PWRSAVE:		// PWR SAVE
		GPRS_stateVars.counters.awaitSecs = 600;
		break;
	}

	tickCount = xTaskGetTickCount();
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] Modem off: Await %lu secs.\r\n\0"),tickCount,GPRS_stateVars.counters.awaitSecs);
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

}
//------------------------------------------------------------------------------------
