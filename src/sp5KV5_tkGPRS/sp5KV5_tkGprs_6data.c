/*
 * sp5KV5_3CH_tkGprs_6data.c
 *
 *  Created on: 24 de may. de 2016
 *      Author: pablo
 */


#include "../sp5KV5_3CH.h"
#include "sp5KV5_tkGprs.h"

static int gTR_G00(void);
static int gTR_G01(void);
static int gTR_G02(void);
static int gTR_G03(void);
static int gTR_G04(void);
static int gTR_G05(void);
static int gTR_G06(void);
static int gTR_G07(void);
static int gTR_G08(void);
static int gTR_G09(void);

static int pv_loadTail(void);
static int pv_loadHeader(void);
static int pv_loadDataFrame( frameData_t *Aframe,  StatBuffer_t  *pxFFStatBuffer);

// Eventos locales
typedef enum {
	g_ev_CTIMER_NOT_0 = 0,
	g_ev_MORE_RCS_4TX,
	g_ev_SRV_RSP_OK,
	g_ev_MORE_RCS_4DEL,
	g_ev_SOCK_IS_CLOSED,

} t_eventos_ssdata;

#define sm_DATA_EVENT_COUNT 5


static u08 cTimer;
static u08 txRcdsInWindow = FRAMEXTXWINDOW;
static s08 moreRcds4Tx = FALSE;
static t_modemResponse serverResponse = MRSP_NONE;
static s08 memRcds4Del = FALSE;

//------------------------------------------------------------------------------------
void sm_DATAFRAME(void)
{
s08 g_eventos[sm_DATA_EVENT_COUNT];
u08 i;

	// Inicializo la lista local de eventos.
	for ( i=0; i < sm_DATA_EVENT_COUNT; i++ ) {
		g_eventos[i] = FALSE;
	}

	// Evaluo solo los eventos del estado DATAFRAME.
	if ( cTimer > 0 ) { g_eventos[g_ev_CTIMER_NOT_0] = TRUE; }
	if ( moreRcds4Tx == TRUE ) { g_eventos[g_ev_MORE_RCS_4TX] = TRUE; }
	if ( serverResponse ==  MRSP_OK ) { g_eventos[g_ev_SRV_RSP_OK] = TRUE; }
	if ( memRcds4Del == TRUE ) { g_eventos[g_ev_MORE_RCS_4DEL] = TRUE; }
	if ( GPRS_stateVars.flags.socketStatus == SOCKET_CLOSED ) { g_eventos[g_ev_SOCK_IS_CLOSED] = TRUE; }

	// MSG RELOAD
	if ( g_checkReloadConfig(gST_DATAFRAME) ) {
		return;
	}

	// Corro la FSM
	switch ( GPRS_stateVars.state.subState ) {
	case gSST_DATAFRAME_00:
		GPRS_stateVars.state.subState = gTR_G00();
		break;
	case gSST_DATAFRAME_01:
		if ( g_eventos[g_ev_MORE_RCS_4TX] )  {
			GPRS_stateVars.state.subState = gTR_G01();
		} else {
			GPRS_stateVars.state.subState = gTR_G02();
		}
		break;
	case gSST_DATAFRAME_02:
		if ( g_eventos[g_ev_SRV_RSP_OK] )  {
			GPRS_stateVars.state.subState = gTR_G05();
		} else {
			GPRS_stateVars.state.subState = gTR_G03();
		}
		break;
	case gSST_DATAFRAME_03:
		if ( g_eventos[g_ev_SOCK_IS_CLOSED] ) {
			GPRS_stateVars.state.subState = gTR_G09();
		} else if ( g_eventos[g_ev_CTIMER_NOT_0] )  {
			GPRS_stateVars.state.subState = gTR_G04();
		} else {
			GPRS_stateVars.state.subState = gTR_G06();
		}
		break;
	case gSST_DATAFRAME_04:
		if ( g_eventos[g_ev_MORE_RCS_4DEL] )  {
			GPRS_stateVars.state.subState = gTR_G07();
		} else {
			GPRS_stateVars.state.subState = gTR_G08();
		}
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

	// Evento inicial. Llego desde standby con el socket abierto.
	// Chequea la memoria y envia el header.

u16 pos;

	FF_seek(); // Ajusta la posicion del puntero de lectura al primer registro a leer
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	// HEADER:
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	// Armo el header en el buffer
	pos = pv_loadHeader();
	FreeRTOS_write( &pdUART0, gprs_printfBuff, pos );

	// DebugMsg
	snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0" ));
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	txRcdsInWindow = FRAMEXTXWINDOW;	// Cantidad de registros maximo a trasmitir en una ventana
	moreRcds4Tx = TRUE;
	if ( GPRS_stateVars.counters.nroLOTEtryes > 0 ) {
		--GPRS_stateVars.counters.nroLOTEtryes;
	}

	g_printExitMsg("G00\0");
	return(gSST_DATAFRAME_01);
}
//------------------------------------------------------------------------------------
static int gTR_G01(void)
{
	// Evaluo si hay mas registros para trasmitir y envio uno.

u16 pos;
frameData_t Aframe;
StatBuffer_t pxFFStatBuffer;

	// Paso1: Transmito.
	// Leo memoria
	FF_fread( &Aframe, sizeof(Aframe));
	FF_stat(&pxFFStatBuffer);

	// Siempre trasmito los datos aunque vengan papasfritas.
	pos = pv_loadDataFrame(&Aframe, &pxFFStatBuffer);

	// Trasmito por el modem.
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, gprs_printfBuff, pos );

	// Imprimo
	u_debugPrint(D_GPRS, "TX->{\0", sizeof("TX->{\0") );
	pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ), PSTR("}\r\n\0"));
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );
	// Agrego mem.stats
	if (pxFFStatBuffer.errno > 0 ) {
		snprintf_P( gprs_printfBuff,  sizeof(gprs_printfBuff), PSTR(" ERROR (%d) MEM(%d) [%d/%d/%d][%d/%d]\r\n\0"), pxFFStatBuffer.errno,txRcdsInWindow, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	} else {
		snprintf_P( gprs_printfBuff, sizeof(gprs_printfBuff), PSTR("TX->MEM(%d) [%d/%d/%d][%d/%d]\r\n\0"), txRcdsInWindow, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	}
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );


	// Paso 2: Evaluo para el proximo ciclo.
	// 1 - La memoria esta vacia en absoluto ( Free = MAX )
	//     o en lectura, es decir que lei todos los registros ocupados.
	//
	FF_stat(&pxFFStatBuffer);
	if ( pxFFStatBuffer.rcdsFree == FF_MAX_RCDS) {
		// Memoria vacia en absoluto
		moreRcds4Tx = FALSE;
	} else if ( pxFFStatBuffer.rcdsFree == 0 ) {
		// Memoria llena
		moreRcds4Tx = TRUE;
	} else if ( pxFFStatBuffer.RD == pxFFStatBuffer.HEAD ) {
		// Memoria con datos pero todos trasmitidos
		moreRcds4Tx = FALSE;
	}

	// 2 - Complete una window
	if ( --txRcdsInWindow == 0 )
		moreRcds4Tx = FALSE;

	g_printExitMsg("G01\0");
	return(gSST_DATAFRAME_01);
}
//------------------------------------------------------------------------------------
static int gTR_G02(void)
{
	// Envio la cola e inicializo el timer de espera de la respuesta del server

u16 pos;

	// TAIL ( No mando el close) :
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	pos = pv_loadTail();

	// Trasmito
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DebugMsg
	snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0" ));
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Frame DATA enviado\r\n\0"));
	u_logPrint(gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Solo espero hasta 10s la respuesta.
	cTimer = 10;
	serverResponse = MRSP_NONE;

	g_printExitMsg("G02\0");
	return(gSST_DATAFRAME_02);
}
//------------------------------------------------------------------------------------
static int gTR_G03(void)
{

	g_printExitMsg("G03\0");
	return(gSST_DATAFRAME_03);
}
//------------------------------------------------------------------------------------
static int gTR_G04(void)
{

size_t pos;

	// Espero 1s.por la respuesta.
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	if ( cTimer > 0 ) {
		--cTimer;
	}

	// Evaluo las respuestas del server
	if ( g_strstr("RX_OK\0", &pos ) == TRUE ) {
		serverResponse = MRSP_OK;
	}

	// El server me pide que me resetee de modo de mandar un nuevo init y reconfigurarme
	if ( g_strstr("RESET\0", &pos ) == TRUE ) {

		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("Config RESET...\r\n\0" ));
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		// RESET
		u_reset();
	}
	g_printExitMsg("G04\0");
	return(gSST_DATAFRAME_02);
}
//------------------------------------------------------------------------------------
static int gTR_G05(void)
{

	// El server respondio OK. Paso a borrar los registros.

StatBuffer_t pxFFStatBuffer;

	// Muestro mensaje de respuesta del server.
	g_printRxBuffer();

	FF_stat(&pxFFStatBuffer);
	if ( FCB.ff_stat.rcds4del > 0 ) {
		memRcds4Del = TRUE;
	} else {
		memRcds4Del = FALSE;
	}

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Frame DATA confirmado\r\n\0"));
	u_logPrint(gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("G05\0");
	return(gSST_DATAFRAME_04);
}
//------------------------------------------------------------------------------------
static int gTR_G06(void)
{

	g_printExitMsg("G06\0");
	return( pv_cambiarEstado(gST_DATAFRAME,gST_STANDBY) );
}
//------------------------------------------------------------------------------------
static int gTR_G07(void)
{

	// Borro de a un registro.

StatBuffer_t pxFFStatBuffer;

	FF_del();
	FF_stat(&pxFFStatBuffer);

	if ( FCB.ff_stat.rcds4del > 0 ) {
		memRcds4Del = TRUE;
	} else {
		memRcds4Del = FALSE;
	}

	tickCount = xTaskGetTickCount();
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] FSstat::DEL->[wrPtr=%d,rdPtr=%d,delPtr=%d][Free=%d,4del=%d]\r\n\0"),tickCount, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("G07\0");
	return(gSST_DATAFRAME_04);
}
//------------------------------------------------------------------------------------
static int gTR_G08(void)
{
	// Luego de trasmitir y borrar un window muestro el fstat

StatBuffer_t pxFFStatBuffer;
u08 pos;

	FF_stat(&pxFFStatBuffer);

	pos = snprintf_P( gprs_printfBuff, sizeof(gprs_printfBuff), PSTR("TX->DELmem [%d/%d/%d][%d/%d]"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	if ( pxFFStatBuffer.rcdsFree == FF_MAX_RCDS) {
		pos += snprintf_P( &gprs_printfBuff[pos], (sizeof(gprs_printfBuff)-pos),PSTR(" Mem.Empty."));
	}
	pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ) ,PSTR("\r\n\r\n\0"));
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );;

	GPRS_stateVars.counters.nroLOTEtryes = MAXTRYESLOTE;

	// Si el server me manda que me resetee...
	g_GPRSprocessReset();

	g_printExitMsg("G08\0");
	return( pv_cambiarEstado(gST_DATAFRAME,gST_STANDBY) );
}
//------------------------------------------------------------------------------------
static int gTR_G09(void)
{

	g_printExitMsg("G09\0");
	return( pv_cambiarEstado(gST_DATAFRAME,gST_STANDBY) );
}
//------------------------------------------------------------------------------------
static int pv_loadTail(void)
{

u16 pos = 0;

	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));

	pos = snprintf_P( gprs_printfBuff, ( sizeof(gprs_printfBuff) - pos ),PSTR(" HTTP/1.1\n") );
	pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ),PSTR("Host: www.spymovil.com\n" ));
	pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ),PSTR("\n\n\0" ));
	return(pos);

}
//------------------------------------------------------------------------------------
static int pv_loadHeader(void)
{

u16 pos = 0;

	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	pos = snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GET " ));
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("%s"), systemVars.serverScript );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("?DLGID=%s"), systemVars.dlgId );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&PASSWD=%s"), systemVars.passwd );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&VER=%s"), SP5K_REV );

	return(pos);
}
//------------------------------------------------------------------------------------
static int pv_loadDataFrame( frameData_t *Aframe,  StatBuffer_t  *pxFFStatBuffer)
{

u16 pos;
u08 channel;
u08 err;

	// Armo el frame
	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));

	// Indice de la linea
	pos = snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("&CTL=%d"), pxFFStatBuffer->RD );

	// Calidad del frame ( por medidas + memoria ).
	err = Aframe->status + pxFFStatBuffer->errno;
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&ST=%d"), err );

	// Aqui indico si los datos leidos de memoria son correctos o hubo un error.
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&LINE=") );
	// Fecha y hora
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe->rtc.year,Aframe->rtc.month,Aframe->rtc.day );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ), PSTR("%02d%02d%02d"),Aframe->rtc.hour,Aframe->rtc.min, Aframe->rtc.sec );

#ifdef UTE_8CH
	// Valores analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR(",%s=%.2f"),systemVars.aChName[channel],Aframe->analogIn[channel] );
	}
	// Datos digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
		pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ), PSTR(",%sP=%.02f,%sL=%d"), systemVars.dChName[channel],Aframe->dIn.pulses[channel],systemVars.dChName[channel],Aframe->dIn.level[channel] );
	}
#endif

#ifdef OSE_3CH
	// Valores analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR(",%s=%.2f,"),systemVars.aChName[channel],Aframe->analogIn[channel] );
	}
	// Datos digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
		pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ), PSTR(",%sP=%.02f"), systemVars.dChName[channel],Aframe->dIn.pulses[channel] );
	}
	// Bateria
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ), PSTR(",bt=%.2f"),Aframe->batt );
#endif

#ifdef OSE_POZOS
	// Configuracion de canales analogicos
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR(",%s=%.0f"),systemVars.aChName[0],Aframe->analogIn[0] );
	// Datos digitales
	pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ), PSTR(",%s=%d"), systemVars.dChName[0l],Aframe->dIn.level[0] );
	pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ), PSTR(",%s=%d"), systemVars.dChName[1],Aframe->dIn.level[1] );
#endif

	return(pos);
}
//------------------------------------------------------------------------------------
