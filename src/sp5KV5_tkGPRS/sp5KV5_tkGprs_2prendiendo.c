/*
 * sp5KV5_3CH_tkGprs_prendiendo.c
 *
 *  Created on: 17 de may. de 2016
 *      Author: pablo
 *
 * En este estado prendo el modem y lo dejo pronto para aceptar comadnos.
 *
 */

#include "../sp5KV5_3CH.h"
#include "../sp5KV5_tkGPRS/sp5KV5_tkGprs.h"

static int gTR_B00(void);
static int gTR_B01(void);
static int gTR_B02(void);
static int gTR_B03(void);
static int gTR_B04(void);
static int gTR_B05(void);
static int gTR_B06(void);
static int gTR_B07(void);
static int gTR_B08(void);
static int gTR_B09(void);
static int gTR_B10(void);

// Eventos locales
typedef enum {
	b_ev_HWTRYES_NOT_0 = 0,
	b_ev_SWTRYES_NOT_0,
	b_ev_CTIMER_NOT_0,
	b_ev_M_RSP_OK,
} t_eventos_ssPrendiendo;

#define sm_PRENDIENDO_EVENT_COUNT 4

static u08 HWtryes;		// Reintentos HW de prender modem
static u08 SWtryes;		// Reintentos SW
static u08 Ctimer;		// Contador de segundos

#define HW_TRYES	3			// Reintentos de prender HW el modem
#define SW_TRYES	3			// Reintentos de hacer toggle del SW del modem

//------------------------------------------------------------------------------------
void sm_MODEMPRENDIENDO(void)
{
s08 b_eventos[sm_PRENDIENDO_EVENT_COUNT];
u08 i;

	// Inicializo la lista local de eventos.
	for ( i=0; i < sm_PRENDIENDO_EVENT_COUNT; i++ ) {
		b_eventos[i] = FALSE;
	}

	// Evaluo solo los eventos del estado APAGADO.
	// HWtryes > 0
	if ( HWtryes > 0 ) { b_eventos[b_ev_HWTRYES_NOT_0] = TRUE; }
	// SWtryes > 0
	if ( SWtryes > 0 ) { b_eventos[b_ev_SWTRYES_NOT_0] = TRUE; }
	// Ctimer > 0
	if ( Ctimer > 0 ) { b_eventos[b_ev_CTIMER_NOT_0] = TRUE; }
	// Modem response OK
	if ( GPRS_stateVars.flags.modemResponse ==  MRSP_OK ) { b_eventos[b_ev_M_RSP_OK] = TRUE; }

	// MSG RELOAD
	if ( g_checkReloadConfig(gST_MODEMPRENDIENDO) ) {
		return;
	}

	// Corro la FSM
	switch ( GPRS_stateVars.state.subState ) {
	case gSST_MODEMPRENDIENDO_00:
		GPRS_stateVars.state.subState = gTR_B00();
		break;
	case gSST_MODEMPRENDIENDO_01:
		GPRS_stateVars.state.subState = gTR_B01();
		break;
	case gSST_MODEMPRENDIENDO_02:
		GPRS_stateVars.state.subState = gTR_B02();
		break;
	case gSST_MODEMPRENDIENDO_03:
		if ( b_eventos[b_ev_CTIMER_NOT_0] ) {
			GPRS_stateVars.state.subState = gTR_B04();
		} else {
			GPRS_stateVars.state.subState = gTR_B03();
		}
		break;
	case gSST_MODEMPRENDIENDO_04:
		if ( b_eventos[b_ev_M_RSP_OK] ) {
			GPRS_stateVars.state.subState = gTR_B05();
		} else {
			GPRS_stateVars.state.subState = gTR_B06();
		}
		break;
	case gSST_MODEMPRENDIENDO_05:
		if ( b_eventos[b_ev_SWTRYES_NOT_0] ) {
			GPRS_stateVars.state.subState = gTR_B07();
		} else {
			GPRS_stateVars.state.subState = gTR_B08();
		}
		break;
	case gSST_MODEMPRENDIENDO_06:
		if ( b_eventos[b_ev_HWTRYES_NOT_0] ) {
			GPRS_stateVars.state.subState = gTR_B09();
		} else {
			GPRS_stateVars.state.subState = gTR_B10();
		}
		break;
	default:
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\ntkGprs::ERROR sst_gprsPrendiendo: subState  (%d) NOT DEFINED\r\n\0"),GPRS_stateVars.state.subState );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		pv_cambiarEstado(gST_MODEMPRENDIENDO,gST_MODEMAPAGADO);
		break;
	}

}
//------------------------------------------------------------------------------------
static int gTR_B00(void)
{

	// Inicializo.
	HWtryes = HW_TRYES;
	SWtryes = SW_TRYES;

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Modem Prendido\r\n\0"));
	u_logPrint(gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("B00\0");
	return(gSST_MODEMPRENDIENDO_01);
}
//------------------------------------------------------------------------------------
static int gTR_B01(void)
{
	// Prendo el modem HW
	MODEM_HWpwrOn();
	SWtryes = SW_TRYES;

	if ( HWtryes > 0 ) {
		--HWtryes;
	}

	// Espero 1s que se estabilize la fuente.
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	g_printExitMsg("B01\0");
	return(gSST_MODEMPRENDIENDO_02);
}
//------------------------------------------------------------------------------------
static int gTR_B02(void)
{
	// Genero un pulso de SW para encenderlo

	// Genero el toggle
	MODEM_SWswitchHIGH();
	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );
	MODEM_SWswitchLOW();

	if ( SWtryes > 0 ) {
		--SWtryes;
	}

	// Espero 10s que prenda.
	Ctimer = 10;

	g_printExitMsg("B02\0");
	return(gSST_MODEMPRENDIENDO_03);
}
//------------------------------------------------------------------------------------
static int gTR_B03(void)
{
	// El modem deberia haber prendido. Mando un comando AT
	// para ver si contesta.

size_t pos;

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);

	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT\r\0", sizeof("AT\r\0") );

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	// Leo y Evaluo la respuesta al comando AT
	GPRS_stateVars.flags.modemResponse = MRSP_NONE;
	if ( g_strstr("OK\0", &pos ) == TRUE ) {
		GPRS_stateVars.flags.modemResponse = MRSP_OK;
	}

	// Muestro el resultado.
	g_printRxBuffer();

	g_printExitMsg("B03\0");
	return(gSST_MODEMPRENDIENDO_04);
}
//------------------------------------------------------------------------------------
static int gTR_B04(void)
{

	// Espero que expire Ctimer para interrogar al modem.
	if ( Ctimer > 0 ) {
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		--Ctimer;
	}

	//g_printExitMsg("B04\0");
	return(gSST_MODEMPRENDIENDO_03);
}
//------------------------------------------------------------------------------------
static int gTR_B05(void)
{

	// El modem respondio OK por lo que esta prendido.
	// Cambio de estado

	g_printExitMsg("B05\0");
	return( pv_cambiarEstado(gST_MODEMPRENDIENDO, gST_CONFIGURAR) );
}
//------------------------------------------------------------------------------------
static int gTR_B06(void)
{

	g_printExitMsg("B06\0");
	return(gSST_MODEMPRENDIENDO_05);
}
//------------------------------------------------------------------------------------
static int gTR_B07(void)
{
	g_printExitMsg("B07\0");
	return(gSST_MODEMPRENDIENDO_02);
}
//------------------------------------------------------------------------------------
static int gTR_B08(void)
{
	g_printExitMsg("B08\0");
	return(gSST_MODEMPRENDIENDO_06);
}
//------------------------------------------------------------------------------------
static int gTR_B09(void)
{
	SWtryes = SW_TRYES;

	g_printExitMsg("B09\0");
	return(gSST_MODEMPRENDIENDO_01);
}
//------------------------------------------------------------------------------------
static int gTR_B10(void)
{
	// El modem no prendio luego de todos los intentos.
	// Salgo a apagar el modem.

	// Apago el modem
	MODEM_HWpwrOff();

	g_printExitMsg("B10\0");
	return( pv_cambiarEstado(gST_MODEMPRENDIENDO, gST_MODEMAPAGADO) );
}
//------------------------------------------------------------------------------------
