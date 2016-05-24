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
#include "sp5KV5_3CH_tkGprs.h"

static int gTR_A00(void);
static int gTR_A01(void);
static int gTR_A02(void);
static int gTR_A03(void);

// Eventos locales
typedef enum {
	a_ev_CTIMER_NOT_0 = 0,
	a_ev_MSGRELOAD
} t_eventos_ssApagado;

#define sm_APAGADO_EVENT_COUNT 2

static void pv_configCTimer(void);

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
	if ( GPRS_stateVars.counters.cTimer > 0 ) { a_eventos[a_ev_CTIMER_NOT_0] = TRUE; }
	if ( GPRS_stateVars.flags.msgReload == TRUE ) { a_eventos[a_ev_MSGRELOAD] = TRUE; }

	// Corro la FSM
	switch ( GPRS_stateVars.state.subState ) {
	case gSST_MODEMAPAGADO_00:
		GPRS_stateVars.state.subState = gTR_A00();
		break;
	case gSST_MODEMAPAGADO_01:
		if ( a_eventos[a_ev_MSGRELOAD] ) {
			GPRS_stateVars.state.subState = gTR_A01();
		} else if ( a_eventos[a_ev_CTIMER_NOT_0] )  {
			GPRS_stateVars.state.subState = gTR_A02();
		} else {
			GPRS_stateVars.state.subState = gTR_A03();
		}
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

	// Evento inicial. Solo salta al primer estado operativo.
	// Inicializo el sistema aqui

	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );
	strncpy_P(systemVars.dlgIp, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	//
	// Calculo el tiempo que debo mantenerme apagado.
	pv_configCTimer();

	// Apago el modem
	MODEM_HWpwrOff();

	g_printExitMsg("A00\0");
	return(gSST_MODEMAPAGADO_01);
}
//------------------------------------------------------------------------------------
static int gTR_A01(void)
{
	// Llego un mensaje de reconfiguracion.

	GPRS_stateVars.flags.msgReload = FALSE;

	// Re-calculo el tiempo que debo mantenerme apagado.
	pv_configCTimer();

	g_printExitMsg("A01\0");
	return(gSST_MODEMAPAGADO_01);
}
//------------------------------------------------------------------------------------
static int gTR_A02(void)
{
	// Me quedo con el modem apagado esperando.
	// Espero 1 segundo

	if ( GPRS_stateVars.counters.cTimer > 0 ) {
		--GPRS_stateVars.counters.cTimer;
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	}

	// En caso de alarma de flooding  (tilt ) debo discar inmediatamente ( cTimer = 0 para
	// salir del estado
	if ( u_checkAlarmFloding() == TRUE ) {
		GPRS_stateVars.counters.cTimer = 0;
	}

	//g_printExitMsg("A01\0");
	return(gSST_MODEMAPAGADO_01);
}
//------------------------------------------------------------------------------------
static int gTR_A03(void)
{
	// Expiro el tiempo de espera. Salgo del estado apagado

	g_printExitMsg("A03\0");
	return( pv_cambiarEstado(gST_MODEMAPAGADO,gST_MODEMPRENDIENDO) );
}
//------------------------------------------------------------------------------------
static void pv_configCTimer(void)
{
static s08 inicio = FALSE;
RtcTimeType_t rtcDateTime;
u16 now;

	// En modo service me quedo en forma indefinida
	if ( systemVars.wrkMode == WK_SERVICE ) {
		GPRS_stateVars.counters.cTimer = 0xFFFFFF;
		return;
	}

	// Cuando recien estoy arrancando espero solo 15s para prender
	if ( ! inicio ) {
		inicio = TRUE;
		GPRS_stateVars.counters.cTimer = 15;
		return;
	}

	// En modo continuo, espero 60s.
	if ( systemVars.pwrMode == PWR_CONTINUO ) {
		GPRS_stateVars.counters.cTimer = 60;
		return;
	}

	// En modo discreto, el tiempo lo marca el timerDial / pwrSave
	if ( systemVars.pwrMode == PWR_DISCRETO ) {

		if (systemVars.pwrSave) {
			// Pwr.save on.
			// Evaluo el pwrSave para ver si puedo discar o no.
			RTC_read(&rtcDateTime);
			now = rtcDateTime.hour * 60 + rtcDateTime.min;	// Hora actual en minutos desde las 00:00

			// Caso 1:
			if ( systemVars.pwrSaveStartTime < systemVars.pwrSaveEndTime ) {
				if ( ( now > systemVars.pwrSaveStartTime) && ( now < systemVars.pwrSaveEndTime) ) {
					// Estoy en intervalo de pwrSave. Vuelvo a chequear en 10 mins.
					GPRS_stateVars.counters.cTimer = 600;
				}
			}
			// Caso 2:
			if ( systemVars.pwrSaveStartTime >= systemVars.pwrSaveEndTime ) {
				if ( ( now < systemVars.pwrSaveEndTime) || ( now > systemVars.pwrSaveStartTime ) ) {
					// Estoy en intervalo de pwrSave. Vuelvo a chequear en 10 mins.
					GPRS_stateVars.counters.cTimer = 600;
				}
			}

		} else {
			// Pwr.save off.
			GPRS_stateVars.counters.cTimer = systemVars.timerDial;
			return;
		}
	}
}
//------------------------------------------------------------------------------------
