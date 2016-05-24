/*
 * sp5KV5_3CH_tkGprs_initframe.c
 *
 *  Created on: 23 de may. de 2016
 *      Author: pablo
 */

#include "../sp5KV5_3CH.h"
#include "sp5KV5_3CH_tkGprs.h"

static int gTR_F00(void);
static int gTR_F01(void);
static int gTR_F02(void);
static int gTR_F03(void);
static int gTR_F04(void);
static int gTR_F05(void);
static int gTR_F06(void);
static int gTR_F07(void);
static int gTR_F08(void);
static int gTR_F09(void);
static int gTR_F10(void);
static int gTR_F11(void);

// Eventos locales
typedef enum {
	f_ev_NROINITS_NOT_1 = 0,
	f_ev_CTIMER_NOT_0,
	f_ev_P_TRYES_NOT_0,
	f_ev_RSP_INIT_OK,		// La respuesta del SERVER es INIT OK.
	f_ev_SOCK_IS_CLOSED,
	f_ev_RSP_ERROR,			// La respuesta del modem es ERROR

} t_eventos_ssInitFrame;

#define sm_INITFRAME_EVENT_COUNT 6

static u08 pTryes;
static u08 cTimer;		// Contador de segundos

//------------------------------------------------------------------------------------
void sm_INITFRAME(void)
{
s08 f_eventos[sm_INITFRAME_EVENT_COUNT];
u08 i;

	// Inicializo la lista local de eventos.
	for ( i=0; i < sm_INITFRAME_EVENT_COUNT; i++ ) {
		f_eventos[i] = FALSE;
	}

	// Evaluo solo los eventos del estado INITFRAME.
	if ( cTimer > 0 ) { f_eventos[f_ev_CTIMER_NOT_0] = TRUE; }
	if ( GPRS_stateVars.counters.nroINITS != 1 ) { f_eventos[f_ev_NROINITS_NOT_1] = TRUE; }
	if ( GPRS_stateVars.flags.modemResponse ==  MRSP_INIT_OK ) { f_eventos[f_ev_RSP_INIT_OK] = TRUE; }
	if ( GPRS_stateVars.flags.modemResponse ==  MRSP_ERROR ) { f_eventos[f_ev_RSP_ERROR] = TRUE; }
	if ( GPRS_stateVars.flags.socketStatus == SOCKET_CLOSED ) { f_eventos[f_ev_SOCK_IS_CLOSED] = TRUE; }

	// Corro la FSM
	switch ( GPRS_stateVars.state.subState ) {
	case gSST_INITFRAME_00:
		GPRS_stateVars.state.subState = gTR_F00();
		break;
	case gSST_INITFRAME_01:
		GPRS_stateVars.state.subState = gTR_F01();
		break;
	case gSST_INITFRAME_02:
		if ( f_eventos[f_ev_NROINITS_NOT_1] ) {
			GPRS_stateVars.state.subState = gTR_F02();
		} else {
			GPRS_stateVars.state.subState = gTR_F03();
		}
		break;
	case gSST_INITFRAME_03:
		if ( f_eventos[f_ev_CTIMER_NOT_0] ) {
			GPRS_stateVars.state.subState = gTR_F04();
		} else {
			GPRS_stateVars.state.subState = gTR_F05();
		}
		break;
	case gSST_INITFRAME_04:
		if ( f_eventos[f_ev_RSP_INIT_OK] ) {
			GPRS_stateVars.state.subState = gTR_F06();
		} else if ( f_eventos[f_ev_SOCK_IS_CLOSED] ) {
			GPRS_stateVars.state.subState = gTR_F07();
		} else if ( f_eventos[f_ev_RSP_ERROR] ) {
			GPRS_stateVars.state.subState = gTR_F08();
		} else {
			GPRS_stateVars.state.subState = gTR_F09();
		}
		break;
	case gSST_INITFRAME_05:
		if ( f_eventos[f_ev_P_TRYES_NOT_0] ) {
			GPRS_stateVars.state.subState = gTR_F11();
		} else {
			GPRS_stateVars.state.subState = gTR_F10();
		}
		break;
	default:
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\ntkGprs::ERROR sst_gprsInitframe: subState  (%d) NOT DEFINED\r\n\0"),GPRS_stateVars.state.subState );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		pv_cambiarEstado(gST_INITFRAME,gST_MODEMAPAGADO);
		break;
	}

}
//------------------------------------------------------------------------------------
static int gTR_F00(void)
{

	g_printExitMsg("F00\0");
	return(gSST_INITFRAME_01);
}
//------------------------------------------------------------------------------------
static int gTR_F01(void)
{

	// Actualizo el contador de INITS lo que me va a permitir controlar si me excedi
	// y debo apagar el modem o no.
	if ( GPRS_stateVars.counters.nroINITS > 0 ) {
		GPRS_stateVars.counters.nroINITS--;
	}

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n%s: GPRS init FRAME(%d):\r\n\0"), u_now(), GPRS_stateVars.counters.nroINITS );
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("F01\0");
	return(gSST_INITFRAME_02);
}
//------------------------------------------------------------------------------------
static int gTR_F02(void)
{
	// Send Init Frame
	// GET /cgi-bin/sp5K/sp5K.pl?DLGID=SPY001&PASSWD=spymovil123&&INIT&ALARM&PWRM=CONT&TPOLL=23&TDIAL=234&PWRS=1,1230,2045&A0=pZ,1,20,3,10&D0=qE,3.24&CONS=1,1234,927,1,3 HTTP/1.1
	// Host: www.spymovil.com
	// Connection: close\r\r ( no mando el close )

u16 pos = 0;
u08 i;

	pTryes = 6;	// Pregunto hasta 6 veces
	cTimer = 3;	// Pregunto c/3secs

	// Trasmision: 1r.Parte.
	// HEADER:
	// Envio parcial ( no CR )
	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	pos = snprintf_P( gprs_printfBuff,CHAR256,PSTR("GET " ));
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("%s"), systemVars.serverScript );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("?DLGID=%s"), systemVars.dlgId );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&PASSWD=%s"), systemVars.passwd );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&VER=%s\0"), SP5K_REV );
	// GPRS sent
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	// LOG
	if ( (systemVars.debugLevel & D_GPRS) != 0) {
		snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0" ));
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// BODY ( 1a parte) :
	memset(gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	if ( u_checkAlarmFloding() ) {
		pos = snprintf_P( gprs_printfBuff ,CHAR256,PSTR("&INIT&ALARM"));
	} else {
		pos = snprintf_P( gprs_printfBuff ,CHAR256,PSTR("&INIT"));
	}

	// timerpoll
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&TPOLL=%d"), systemVars.timerPoll);
	// timerdial
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&TDIAL=%d"), systemVars.timerDial);
	// tilt
	if ( systemVars.tiltEnabled == TRUE) {
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&TILT=ON"));
	} else {
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&TILT=OFF"));
	}
	// pwrMode
	if ( systemVars.pwrMode == PWR_CONTINUO) { pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&PWRM=CONT")); }
	if ( systemVars.pwrMode == PWR_DISCRETO) { pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&PWRM=DISC")); }
	// pwrSave
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&PWRS=%d,%d,%d"),systemVars.pwrSave, u_convertMINS2hhmm( systemVars.pwrSaveStartTime ),u_convertMINS2hhmm( systemVars.pwrSaveEndTime) );
	// csq
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&CSQ=%d\0"), systemVars.csq);
	// GPRS sent
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	// LOG
	if ( (systemVars.debugLevel & D_GPRS) != 0) {
		snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0" ));
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// BODY ( 2a parte) :
	memset(gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	pos = 0;
	// Configuracion de canales analogicos
	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++) {
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&A%d=%s,%d,%d,%d,%.02f"), i,systemVars.aChName[i],systemVars.Imin[i], systemVars.Imax[i], systemVars.Mmin[i], systemVars.Mmax[i]);
	}
	// Configuracion de canales digitales
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&D0=%s,%.02f"),systemVars.dChName[0],systemVars.magPP[0]);
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&D1=%s,%.02f"),systemVars.dChName[1],systemVars.magPP[1]);
	// Reset status
	pos += snprintf_P( &gprs_printfBuff[pos],( CHAR256 - pos ),PSTR("&WDG=%d\0"),wdgStatus.resetCause );
	// GPRS sent
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	// LOG
	snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0" ));
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// TAIL ( No mando el close):
	memset(gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	pos = snprintf_P( gprs_printfBuff, ( sizeof(gprs_printfBuff) - pos ),PSTR(" HTTP/1.1\n") );
	pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ),PSTR("Host: www.spymovil.com\n" ));
	pos += snprintf_P( &gprs_printfBuff[pos], sizeof(gprs_printfBuff),PSTR("\n\n\0" ));
	// GPRS sent
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	// LOG
	snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0" ));
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("F02\0");
	return(gSST_INITFRAME_03);
}
//------------------------------------------------------------------------------------
static int gTR_F03(void)
{
	// Apago.

	g_printExitMsg("F03\0");
	return( pv_cambiarEstado(gST_INITFRAME,gST_MODEMAPAGADO) );
}
//------------------------------------------------------------------------------------
static int gTR_F04(void)
{
	// Espera 1s

	if ( cTimer > 0 ) {
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		--cTimer;
	}

	//FreeRTOS_write( &pdUART1, ".\0", sizeof(".\0") );
	//pv_GPRSprintExitMsg("F04\0");
	return(gSST_INITFRAME_03);
}
//------------------------------------------------------------------------------------
static int gTR_F05(void)
{

size_t pos;

	// Leo y Evaluo la respuesta al comando AT*E2IPO ( open socket )
	// La respuesta correcta debe ser CONNECT

	GPRS_stateVars.flags.modemResponse =  MRSP_NONE;

	if ( g_strstr("INIT_OK\0", &pos ) == TRUE ) {
		GPRS_stateVars.flags.modemResponse = MRSP_INIT_OK;
	} else 	if ( g_strstr("ERROR\0", &pos ) == TRUE ) {
		GPRS_stateVars.flags.modemResponse = MRSP_ERROR;
	}

	g_printRxBuffer();

	g_printExitMsg("F05\0");
	return( gSST_INITFRAME_04);
}
//------------------------------------------------------------------------------------
static int gTR_F06(void)
{
	// Recibi la respuesta del INIT.
	// La parseo y me reconfiguro
	// Cierro el socket

u08 saveFlag = 0;

	// Proceso la respuesta del INIT para reconfigurar los parametros
	g_GPRSprocessServerClock();
	saveFlag = g_GPRSprocessPwrMode();
	saveFlag += g_GPRSprocessTimerPoll();
	saveFlag += g_GPRSprocessTimerDial();
	saveFlag += g_GPRSprocessPwrSave();
	saveFlag += g_GPRSprocessTilt();
	// Canales analogicos.
	saveFlag += g_GPRSprocessAch(0);
	saveFlag += g_GPRSprocessAch(1);
	saveFlag += g_GPRSprocessAch(2);
	// Canales digitales
	saveFlag += g_GPRSprocessDch(0);
	saveFlag += g_GPRSprocessDch(1);

	if ( saveFlag > 0 ) {
		if ( u_saveSystemParams() ) {

			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\nGPRS: save parameters OK\r\n\0"));
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

			// Le aviso a la tkAnalog que se reconfigure.
			while ( xTaskNotify(xHandle_tkAIn, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
				vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
			}
			//
			// No le aviso a la tkGprs porque estoy en ella, solo prendo la flag.
			GPRS_stateVars.flags.msgReload = TRUE;
		}
	}

	// Reseteo la variable INITS para no enviar mas un INIT.
	GPRS_stateVars.counters.nroINITS = 0;

	// Trasmiti el ultimo error de WDG: lo borro
	wdgStatus.resetCause = 0;

	// Si el server me manda que me resetee para borrar las flags de alarma
	g_GPRSprocessReset();

	// Cambio de estado
	// El proximo frame debe ser de datos
	GPRS_stateVars.state.nextFrame = DATA_FRAME;
	g_printExitMsg("F06\0");
	return( pv_cambiarEstado(gST_INITFRAME,gST_STANDBY) );

}
//------------------------------------------------------------------------------------
static int gTR_F07(void)
{
	// Cambio de estado
	g_printExitMsg("F07\0");
	return( pv_cambiarEstado(gST_INITFRAME,gST_STANDBY) );

}
//------------------------------------------------------------------------------------
static int gTR_F08(void)
{
	// Cambio de estado
	g_printExitMsg("F08\0");
	return( pv_cambiarEstado(gST_INITFRAME,gST_STANDBY) );

}
//------------------------------------------------------------------------------------
static int gTR_F09(void)
{

	if ( pTryes > 0 ) {
		pTryes--;
	}

	g_printExitMsg("F09\0");
	return( gSST_INITFRAME_05);
}
//------------------------------------------------------------------------------------
static int gTR_F10(void)
{
	// Expiro el tiempo de espera de respuesta. Cierro el socket

	// Cambio de estado
	g_printExitMsg("F10\0");
	return( pv_cambiarEstado(gST_INITFRAME,gST_STANDBY) );
}
//------------------------------------------------------------------------------------
static int gTR_F11(void)
{
	g_printExitMsg("F11\0");
	return(gSST_INITFRAME_03);
}
//------------------------------------------------------------------------------------

