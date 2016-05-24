/*
 * sp5KV5_3CH_tkGprs_socket.c
 *
 *  Created on: 23 de may. de 2016
 *      Author: pablo
 */



#include "../sp5KV5_3CH.h"
#include "sp5KV5_3CH_tkGprs.h"

static int gTR_E00(void);
static int gTR_E01(void);
static int gTR_E02(void);
static int gTR_E03(void);
static int gTR_E04(void);
static int gTR_E05(void);
static int gTR_E06(void);
static int gTR_E07(void);
static int gTR_E08(void);
static int gTR_E09(void);
static int gTR_E10(void);
static int gTR_E11(void);
static int gTR_E12(void);
static int gTR_E13(void);
static int gTR_E14(void);

// Eventos locales
typedef enum {
	e_ev_CTIMER_NOT_0 = 0,
	e_ev_MSGRELOAD,
	e_ev_SOCK_IS_OPEN,
	e_ev_P_TRYES_NOT_0,
	e_ev_Q_TRYES_NOT_0,
	e_ev_M_RSP_ERROR

} t_eventos_ssSocket;

#define sm_SOCKET_EVENT_COUNT 6

static u08 cTimer;
static u08 pTryes;
static u08 qTryes;

//------------------------------------------------------------------------------------
void sm_SOCKET(void)
{
s08 e_eventos[sm_SOCKET_EVENT_COUNT];
u08 i;

	// Inicializo la lista local de eventos.
	for ( i=0; i < sm_SOCKET_EVENT_COUNT; i++ ) {
		e_eventos[i] = FALSE;
	}

	// Evaluo solo los eventos del estado STANDBY.
	if ( GPRS_stateVars.flags.msgReload == TRUE ) { e_eventos[e_ev_MSGRELOAD] = TRUE; }
	if ( GPRS_stateVars.flags.socketStatus ==SOCKET_OPEN ) { e_eventos[e_ev_SOCK_IS_OPEN] = TRUE; }
	if (  GPRS_stateVars.flags.modemResponse == MRSP_ERROR ) { e_eventos[e_ev_M_RSP_ERROR] = TRUE; }
	if ( cTimer > 0 ) { e_eventos[e_ev_CTIMER_NOT_0] = TRUE; }
	if ( pTryes > 0 ) { e_eventos[e_ev_P_TRYES_NOT_0] = TRUE; }
	if ( qTryes > 0 ) { e_eventos[e_ev_Q_TRYES_NOT_0] = TRUE; }

	// Corro la FSM
	switch ( GPRS_stateVars.state.subState ) {
	case gSST_OPENSOCKET_00:
		GPRS_stateVars.state.subState = gTR_E00();
		break;
	case gSST_OPENSOCKET_01:
		if ( e_eventos[e_ev_SOCK_IS_OPEN] ) {
			GPRS_stateVars.state.subState = gTR_E01();
		} else {
			GPRS_stateVars.state.subState = gTR_E04();
		}
		break;
	case gSST_OPENSOCKET_02:
		if ( e_eventos[e_ev_CTIMER_NOT_0] ) {
			GPRS_stateVars.state.subState = gTR_E03();
		} else {
			GPRS_stateVars.state.subState = gTR_E02();
		}
		break;
	case gSST_OPENSOCKET_03:
		GPRS_stateVars.state.subState = gTR_E05();
		break;
	case gSST_OPENSOCKET_04:
		if ( e_eventos[e_ev_CTIMER_NOT_0] ) {
			GPRS_stateVars.state.subState = gTR_E06();
		} else {
			GPRS_stateVars.state.subState = gTR_E07();
		}
		break;
	case gSST_OPENSOCKET_05:
		if ( e_eventos[e_ev_SOCK_IS_OPEN] ) {
			GPRS_stateVars.state.subState = gTR_E13();
		} else if ( e_eventos[e_ev_SOCK_IS_OPEN] ) {
			GPRS_stateVars.state.subState = gTR_E14();
		} else {
			GPRS_stateVars.state.subState = gTR_E08();
		}
		break;
	case gSST_OPENSOCKET_06:
		if ( e_eventos[e_ev_P_TRYES_NOT_0] ) {
			GPRS_stateVars.state.subState = gTR_E11();
		} else {
			GPRS_stateVars.state.subState = gTR_E09();
		}
		break;
	case gSST_OPENSOCKET_07:
		if ( e_eventos[e_ev_Q_TRYES_NOT_0] ) {
			GPRS_stateVars.state.subState = gTR_E12();
		} else {
			GPRS_stateVars.state.subState = gTR_E10();
		}
		break;
	default:
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\ntkGprs::ERROR sst_gprsSocket: subState  (%d) NOT DEFINED\r\n\0"),GPRS_stateVars.state.subState );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		pv_cambiarEstado(gST_OPENSOCKET,gST_MODEMAPAGADO);
		break;
	}
}
//------------------------------------------------------------------------------------
static int gTR_E00(void)
{

	// Evento inicial. Solo salta al primer estado operativo.
	// Inicializo cTimer para esperar hasta 10s que cierre el socket si
	// esta abierto

	// Cierro el socket por las dudas.
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	// Paso primero a modo comando
	FreeRTOS_write( &pdUART0, "+++AT\r\0", sizeof("+++AT\r\0") );
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	// Y mando el comando de cerrar el socket. Si esta cerrado va a responder ERROR pero no importa
	FreeRTOS_write( &pdUART0, "AT*E2IPC=1\r\0", sizeof("AT*E2IPC=1\r\0") );
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	// No muestro la respuesta del modem ya que si esta cerrado va a indicar ERROR y
	// puede confundir al operador.
	//g_printRxBuffer();

	cTimer = 30;	// Espero hasta 30s que el socket este cerrado.

	g_printExitMsg("E00\0");
	return(gSST_OPENSOCKET_01);
}
//------------------------------------------------------------------------------------
static int gTR_E01(void)
{

	// Espero 1s
	if ( cTimer > 0 ) {
		--cTimer;
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	}

	//g_printExitMsg("E01\0");
	return(gSST_OPENSOCKET_02);
}
//------------------------------------------------------------------------------------
static int gTR_E02(void)
{

	// Espere 30s por el socket cerrado pero no cerro. Pruebo apagar el modem.

	g_printExitMsg("E02\0");
	return( pv_cambiarEstado(gST_OPENSOCKET, gST_MODEMAPAGADO) );
}
//------------------------------------------------------------------------------------
static int gTR_E03(void)
{

	// Vuelvo a esperar
	//g_printExitMsg("E03\0");
	return(gSST_OPENSOCKET_01);
}
//------------------------------------------------------------------------------------
static int gTR_E04(void)
{

	// El socket esta cerrado. Continuo.

	qTryes = 3;		// Voy a probar hasta 3 veces mandar el comando de abrir socket

	g_printExitMsg("E04\0");
	return(gSST_OPENSOCKET_03);
}
//------------------------------------------------------------------------------------
static int gTR_E05(void)
{

	// Envio el comando AT para abrir el socket

size_t xBytes;

	cTimer = 5;	// Consulto c/5s si abrio el socket
	pTryes = 6;	// Lo hago 6 veces.

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	xBytes = snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT*E2IPO=1,\"%s\",%s\r\n\0"),systemVars.serverAddress,systemVars.serverPort);
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Debug
	tickCount = xTaskGetTickCount();
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs: OPEN SOCKET (%d)\r\n\0"),tickCount,qTryes );
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("E05\0");
	return(gSST_OPENSOCKET_04);
}
//------------------------------------------------------------------------------------
static int gTR_E06(void)
{

	// Espero 1s
	if ( cTimer > 0 ) {
		--cTimer;
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	}

	//g_printExitMsg("E06\0");
	return(gSST_OPENSOCKET_04);
}
//------------------------------------------------------------------------------------
static int gTR_E07(void)
{
	// Leo y Evaluo la respuesta al comando AT*E2IPO ( open socket )
	// La respuesta correcta debe ser CONNECT
	// La evalua la tarea tkGprsRX !!! pero de todos modos lo confirmo aqui.

size_t pos;

	GPRS_stateVars.flags.modemResponse  = MRSP_NONE;

	if ( g_strstr("CONNECT\0", &pos ) == TRUE ) {
		GPRS_stateVars.flags.modemResponse = MRSP_CREG;
		GPRS_stateVars.flags.socketStatus = SOCKET_OPEN;
	}

	if ( g_strstr("ERROR\0", &pos ) == TRUE ) {
		GPRS_stateVars.flags.modemResponse = MRSP_ERROR;
		GPRS_stateVars.flags.socketStatus = SOCKET_CLOSED;
	}

	g_printRxBuffer();

	g_printExitMsg("E07\0");
	return(gSST_OPENSOCKET_05);
}
//------------------------------------------------------------------------------------
static int gTR_E08(void)
{

	if ( pTryes > 0 ) {
		--pTryes;
	}

	g_printExitMsg("E08\0");
	return(gSST_OPENSOCKET_06);
}
//------------------------------------------------------------------------------------
static int gTR_E09(void)
{

	if ( qTryes > 0 ) {
		--qTryes;
	}

	g_printExitMsg("E09\0");
	return(gSST_OPENSOCKET_07);
}
//------------------------------------------------------------------------------------
static int gTR_E10(void)
{

	// Salgo a apagar el modem.

	g_printExitMsg("E10\0");
	return( pv_cambiarEstado(gST_OPENSOCKET, gST_MODEMAPAGADO) );
}
//------------------------------------------------------------------------------------
static int gTR_E11(void)
{
	// Espero otros 5s
	cTimer = 5;

	g_printExitMsg("E11\0");
	return(gSST_OPENSOCKET_04);
}
//------------------------------------------------------------------------------------
static int gTR_E12(void)
{
	// Vuelvo a reintentar abrir el socket
	g_printExitMsg("E12\0");
	return(gSST_OPENSOCKET_03);
}
//------------------------------------------------------------------------------------
static int gTR_E13(void)
{
	// El socket abrio: cambio de estado volviendo al STANDBY

	g_printExitMsg("E13\0");
	return( pv_cambiarEstado(gST_OPENSOCKET, gST_STANDBY) );
}
//------------------------------------------------------------------------------------
static int gTR_E14(void)
{
	// El socket indico ERROR.
	if ( qTryes > 0 ) {
		--qTryes;
	}

	g_printExitMsg("E14\0");
	return(gSST_OPENSOCKET_07);
}
//------------------------------------------------------------------------------------

