/*
 * sp5KV5_3CH_tkGprs_standby.c
 *
 *  Created on: 18 de may. de 2016
 *      Author: pablo
 */


#include "../sp5KV5_3CH.h"
#include "sp5KV5_3CH_tkGprs.h"

static int gTR_D00(void);
static int gTR_D01(void);
static int gTR_D02(void);
static int gTR_D03(void);
static int gTR_D04(void);
static int gTR_D05(void);
static int gTR_D06(void);
static int gTR_D07(void);
static int gTR_D08(void);
static int gTR_D09(void);
static int gTR_D10(void);
static int gTR_D11(void);
static int gTR_D12(void);
static int gTR_D13(void);
static int gTR_D14(void);
static int gTR_D15(void);
static int gTR_D16(void);

// Eventos locales
typedef enum {
	d_ev_CTIMER_NOT_0 = 0,
	d_ev_MSGRELOAD,
	d_ev_NEXT_FRAME_INIT,
	d_ev_NEXT_FRAME_CONF,
	d_ev_NEXT_FRAME_DATA,
	d_ev_SOCK_IS_OPEN,
	d_ev_PWRCONTINUO,

} t_eventos_ssStandby;

#define sm_STANDBY_EVENT_COUNT 7

static u08 cTimer;

//------------------------------------------------------------------------------------
void sm_STANDBY(void)
{
s08 d_eventos[sm_STANDBY_EVENT_COUNT];
u08 i;

	// Inicializo la lista local de eventos.
	for ( i=0; i < sm_STANDBY_EVENT_COUNT; i++ ) {
		d_eventos[i] = FALSE;
	}

	// Evaluo solo los eventos del estado STANDBY.
	if ( GPRS_stateVars.flags.msgReload == TRUE ) { d_eventos[d_ev_MSGRELOAD] = TRUE; }
	if ( cTimer > 0 ) { d_eventos[d_ev_CTIMER_NOT_0] = TRUE; }
	if ( systemVars.pwrMode == PWR_CONTINUO ) { d_eventos[d_ev_PWRCONTINUO] = TRUE; }
	if ( GPRS_stateVars.flags.socketStatus ==SOCKET_OPEN ) { d_eventos[d_ev_SOCK_IS_OPEN] = TRUE; }
	if ( GPRS_stateVars.state.nextFrame == INIT_FRAME ) { d_eventos[d_ev_NEXT_FRAME_INIT] = TRUE; }
	if ( GPRS_stateVars.state.nextFrame == CONF_FRAME ) { d_eventos[d_ev_NEXT_FRAME_CONF] = TRUE; }
	if ( GPRS_stateVars.state.nextFrame == DATA_FRAME ) { d_eventos[d_ev_NEXT_FRAME_DATA] = TRUE; }

	// Corro la FSM
	switch ( GPRS_stateVars.state.subState ) {
	case gSST_STANDBY_00:
		GPRS_stateVars.state.subState = gTR_D00();
		break;
	case gSST_STANDBY_01:
		if ( d_eventos[d_ev_NEXT_FRAME_INIT] ) {
			GPRS_stateVars.state.subState = gTR_D01();
		} else {
			GPRS_stateVars.state.subState = gTR_D04();
		}
		break;
	case gSST_STANDBY_02:
		if ( d_eventos[d_ev_SOCK_IS_OPEN] ) {
			GPRS_stateVars.state.subState = gTR_D02();
		} else {
			GPRS_stateVars.state.subState = gTR_D03();
		}
		break;
	case gSST_STANDBY_03:
		if ( d_eventos[d_ev_NEXT_FRAME_CONF] ) {
			GPRS_stateVars.state.subState = gTR_D05();
		} else {
			GPRS_stateVars.state.subState = gTR_D08();
		}
		break;
	case gSST_STANDBY_04:
		if ( d_eventos[d_ev_SOCK_IS_OPEN] ) {
			GPRS_stateVars.state.subState = gTR_D06();
		} else {
			GPRS_stateVars.state.subState = gTR_D07();
		}
		break;
	case gSST_STANDBY_05:
		if ( d_eventos[d_ev_NEXT_FRAME_DATA] ) {
			GPRS_stateVars.state.subState = gTR_D09();
		} else {
			GPRS_stateVars.state.subState = gTR_D12();
		}
		break;
	case gSST_STANDBY_06:
		if ( d_eventos[d_ev_SOCK_IS_OPEN] ) {
			GPRS_stateVars.state.subState = gTR_D10();
		} else {
			GPRS_stateVars.state.subState = gTR_D11();
		}
		break;
	case gSST_STANDBY_07:
		if ( d_eventos[d_ev_PWRCONTINUO] ) {
			GPRS_stateVars.state.subState = gTR_D14();
		} else {
			GPRS_stateVars.state.subState = gTR_D13();
		}
		break;
	case gSST_STANDBY_08:
		if ( d_eventos[d_ev_CTIMER_NOT_0] ) {
			GPRS_stateVars.state.subState = gTR_D15();
		} else {
			GPRS_stateVars.state.subState = gTR_D16();
		}
		break;
	default:
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\ntkGprs::ERROR sst_gprsStandby: subState  (%d) NOT DEFINED\r\n\0"),GPRS_stateVars.state.subState );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		pv_cambiarEstado(gST_STANDBY,gST_MODEMAPAGADO);
		break;
	}
}
//------------------------------------------------------------------------------------
static int gTR_D00(void)
{

	// Evento inicial. Solo salta al primer estado operativo.

	g_printExitMsg("D00\0");
	return(gSST_STANDBY_01);
}
//------------------------------------------------------------------------------------
static int gTR_D01(void)
{

	g_printExitMsg("D01\0");
	return(gSST_STANDBY_02);
}
//------------------------------------------------------------------------------------
static int gTR_D02(void)
{
	// Socket abierto: cambio a INIT

	g_printExitMsg("D02\0");
	return( pv_cambiarEstado(gST_STANDBY,gST_INITFRAME) );
}
//------------------------------------------------------------------------------------
static int gTR_D03(void)
{

	// Socket closed

	g_printExitMsg("D03\0");
	return( pv_cambiarEstado(gST_STANDBY,gST_OPENSOCKET ) );
}
//------------------------------------------------------------------------------------
static int gTR_D04(void)
{

	g_printExitMsg("D04\0");
	return(gSST_STANDBY_03);
}
//------------------------------------------------------------------------------------
static int gTR_D05(void)
{

	g_printExitMsg("D05\0");
	return(gSST_STANDBY_04);
}
//------------------------------------------------------------------------------------
static int gTR_D06(void)
{
	// Socket abierto: cambio a CONF

	g_printExitMsg("D06\0");
	return( pv_cambiarEstado(gST_STANDBY,gST_CONFFRAME) );
}
//------------------------------------------------------------------------------------
static int gTR_D07(void)
{
	// Socket closed

	g_printExitMsg("D07\0");
	return( pv_cambiarEstado(gST_STANDBY,gST_OPENSOCKET ) );
}
//------------------------------------------------------------------------------------
static int gTR_D08(void)
{
	// Veo si hay datos en memoria.

StatBuffer_t pxFFStatBuffer;

	FF_stat(&pxFFStatBuffer);

	// Por defecto fijo para trasmitir
	GPRS_stateVars.state.nextFrame = DATA_FRAME;

	// 1 -La memoria esta vacia en absoluto ( Free = MAX )
	// o en lectura, es decir que lei todos los registros ocupados.
	//
	if ( pxFFStatBuffer.rcdsFree == FF_MAX_RCDS) {
		// Memoria vacia en absoluto: No trasmito
		GPRS_stateVars.state.nextFrame = NO_FRAME;
	} else if ( pxFFStatBuffer.rcdsFree == 0 ) {
		// Memoria llena: Trasmito
		GPRS_stateVars.state.nextFrame = DATA_FRAME;
	} else if ( pxFFStatBuffer.RD == pxFFStatBuffer.HEAD ) {
		// Memoria con datos pero todos trasmitidos
		GPRS_stateVars.state.nextFrame = NO_FRAME;
	}

	g_printExitMsg("D08\0");
	return(gSST_STANDBY_05);
}
//------------------------------------------------------------------------------------
static int gTR_D09(void)
{

	g_printExitMsg("D09\0");
	return(gSST_STANDBY_06);
}
//------------------------------------------------------------------------------------
static int gTR_D10(void)
{
	// Socket abierto: cambio a CONF

	g_printExitMsg("D10\0");
	return( pv_cambiarEstado(gST_STANDBY,gST_DATAFRAME) );
}
//------------------------------------------------------------------------------------
static int gTR_D11(void)
{
	// Socket closed

	g_printExitMsg("D11\0");
	return( pv_cambiarEstado(gST_STANDBY,gST_OPENSOCKET ) );
}
//------------------------------------------------------------------------------------
static int gTR_D12(void)
{
	g_printExitMsg("D12\0");
	return(gSST_STANDBY_07);
}
//------------------------------------------------------------------------------------
static int gTR_D13(void)
{
	// No hay mas que transmitir en modo discreto: apago

	g_printExitMsg("D13\0");
	return( pv_cambiarEstado(gST_STANDBY,gST_MODEMAPAGADO) );
}
//------------------------------------------------------------------------------------
static int gTR_D14(void)
{
	// No hay mas que transmitir en modo continuo: espero 30s

	cTimer = 30;

	g_printExitMsg("D14\0");
	return( gSST_STANDBY_08 );
}
//------------------------------------------------------------------------------------
static int gTR_D15(void)
{
	// Espero 1s

	if (cTimer > 0 ) {
		--cTimer;
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	}

	//g_printExitMsg("D15\0");
	return(gSST_STANDBY_08);
}
//------------------------------------------------------------------------------------
static int gTR_D16(void)
{

	g_printExitMsg("D16\0");
	return(gSST_STANDBY_01);
}
//------------------------------------------------------------------------------------

