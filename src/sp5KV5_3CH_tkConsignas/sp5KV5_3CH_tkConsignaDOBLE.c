/*
 * sp5KV5_3CH_tkConsignaDoble.c
 *
 *  Created on: 10 de jun. de 2016
 *      Author: pablo
 */


#include "../sp5KV5_3CH.h"
#include "sp5KV5_3CH_tkConsignas.h"

#ifdef CONSIGNA
// ------------------------------------------------------------------------------------
// DOBLE CONSIGNA
// ------------------------------------------------------------------------------------
// Estados
typedef enum {	dcST_00 = 0,
				dcST_01,
				dcST_02,
				dcST_03,
				dcST_04,
} t_dobleCOnsignaState;

// Transiciones
static int cTR_DC00(void);
static int cTR_DC01(void);
static int cTR_DC02(void);
static int cTR_DC03(void);
static int cTR_DC04(void);
static int cTR_DC05(void);
static int cTR_DC06(void);
static int cTR_DC07(void);
static int cTR_DC08(void);
static int cTR_DC09(void);

void pv_setDConsignaInicial ( void );

// Eventos
typedef enum {
	dc_ev_CTIMER_NOT_0,
	dc_ev_CONS_DIA,
	dc_ev_CONS_NOCHE,
	dc_ev_MSG_RELOAD,
} t_eventos_dobleConsigna;

#define sm_DOBLECONSIGNA_EVENT_COUNT 4

static u08 tkDC_state;
static u08 consigna4aplicar;

static u08 cTimer;		// Contador de segundos

// ------------------------------------------------------------------------------------
void sm_DOBLECONSIGNA(void)
{
s08 dc_eventos[sm_DOBLECONSIGNA_EVENT_COUNT];
u08 i;

	// Inicializo la lista local de eventos.
	for ( i=0; i < sm_DOBLECONSIGNA_EVENT_COUNT; i++ ) {
		dc_eventos[i] = FALSE;
	}

	// Evaluo
	if ( cTimer > 0 ) { dc_eventos[dc_ev_CTIMER_NOT_0] = TRUE; }
	if ( consigna4aplicar == CONSIGNA_DIURNA ) { dc_eventos[dc_ev_CONS_DIA] = TRUE; }
	if ( consigna4aplicar == CONSIGNA_NOCTURNA ) { dc_eventos[dc_ev_CONS_NOCHE] = TRUE; }

	// Corro la FSM
	switch ( tkDC_state ) {
	case dcST_00:
		tkDC_state = cTR_DC00();
		break;
	case dcST_01:
		if ( dc_eventos[dc_ev_CTIMER_NOT_0] ) {
			tkDC_state = cTR_DC01();
		} else {
			tkDC_state = cTR_DC02();
		}
		break;
	case dcST_02:
		if ( dc_eventos[dc_ev_CONS_DIA] ) {
			tkDC_state = cTR_DC03();
		} else if ( dc_eventos[dc_ev_CONS_NOCHE] ) {
			tkDC_state = cTR_DC04();
		} else {
			tkDC_state = cTR_DC05();
		}
		break;
	case dcST_03:
		if ( dc_eventos[dc_ev_CTIMER_NOT_0] ) {
			tkDC_state = cTR_DC06();
		} else {
			tkDC_state = cTR_DC07();
		}
		break;
	case dcST_04:
		if ( dc_eventos[dc_ev_CTIMER_NOT_0] ) {
			tkDC_state = cTR_DC08();
		} else {
			tkDC_state = cTR_DC09();
		}
		break;
	default:
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("\r\ntkConsignas::ERROR Doble Consigna: State  (%d) NOT DEFINED\r\n\0"),tkDC_state );
		FreeRTOS_write( &pdUART1, cons_printfBuff, sizeof(cons_printfBuff) );
		tkDC_state = dcST_00;
		break;
	}

}
//------------------------------------------------------------------------------------
static int cTR_DC00(void)
{

	// Inicializo.
	cTimer = 30;
	pv_setDConsignaInicial();

	pv_consignaPrintExitMsg(0);
	return(dcST_01);
}
//------------------------------------------------------------------------------------
static int cTR_DC01(void)
{

	// Espero 30s

	 if ( cTimer > 0 ) {
		 vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		 --cTimer;
	 }

	//pv_consignaPrintExitMsg(1);
	return(dcST_01);
}
//------------------------------------------------------------------------------------
static int cTR_DC02(void)
{

	// Chequeo que consigna aplicar

RtcTimeType_t rtcDateTime;
u16 now;

	consigna4aplicar = CONSIGNA_NONE;

	// Hora actual en minutos.
	 RTC_read(&rtcDateTime);
	 now = rtcDateTime.hour * 60 + rtcDateTime.min;

	 if ( now == systemVars.consigna.horaConsDia ) {
		 consigna4aplicar = CONSIGNA_DIURNA;
		 goto quit;
	 }

	if ( now == systemVars.consigna.horaConsNoc ) {
		 consigna4aplicar = CONSIGNA_NOCTURNA;
		 goto quit;
	}

quit:

	pv_consignaPrintExitMsg(2);
	return(dcST_02);
}
//------------------------------------------------------------------------------------
static int cTR_DC03(void)
{

	// Fijo la consigna DIURNA y espero 30s para reconfigurarla

	u_setConsignaDiurna();
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Set Consigna Diurna\r\n\0"), u_now() );
	u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

	cTimer = 30;

	pv_consignaPrintExitMsg(3);
	return(dcST_03);
}
//------------------------------------------------------------------------------------
static int cTR_DC04(void)
{
	// Fijo la consigna NOCTURNA y espero 30s para reconfigurarla

	u_setConsignaNocturna();
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Set Consigna Nocturna\r\n\0"), u_now() );
	u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

	cTimer = 30;

	pv_consignaPrintExitMsg(4);
	return(dcST_04);
}
//------------------------------------------------------------------------------------
static int cTR_DC05(void)
{

	// Inicializo.
	cTimer = 30;

	pv_consignaPrintExitMsg(5);
	return(dcST_01);
}
//------------------------------------------------------------------------------------
static int cTR_DC06(void)
{
	// Espero 30s
	 if ( cTimer > 0 ) {
		 vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		 --cTimer;
	 }

	pv_consignaPrintExitMsg(6);
	return(dcST_03);
}
//------------------------------------------------------------------------------------
static int cTR_DC07(void)
{
	// Reconfiguro la consigna DIURNA
	u_setConsignaDiurna();
	cTimer = 30;
	consigna4aplicar = CONSIGNA_NONE;

	pv_consignaPrintExitMsg(7);
	return(dcST_01);
}
//------------------------------------------------------------------------------------
static int cTR_DC08(void)
{
	// Espero 30s
	 if ( cTimer > 0 ) {
		 vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		 --cTimer;
	 }

	pv_consignaPrintExitMsg(8);
	return(dcST_04);
}
//------------------------------------------------------------------------------------
static int cTR_DC09(void)
{
	// Reconfiguro la consigna NOCTURNA
	u_setConsignaNocturna();
	cTimer = 30;
	consigna4aplicar = CONSIGNA_NONE;


	pv_consignaPrintExitMsg(9);
	return(dcST_01);
}
//------------------------------------------------------------------------------------
void pv_setDConsignaInicial ( void )
{
	// Determino cual consigna corresponde aplicar y la aplico
RtcTimeType_t rtcDateTime;
u16 now;

	// Hora actual en minutos.
	RTC_read(&rtcDateTime);
	now = rtcDateTime.hour * 60 + rtcDateTime.min;

	if ( systemVars.consigna.type == CONSIGNA_OFF ) {
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: OFF\r\n\0"), u_now() );
		u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

	// Consignas ON:

	// Caso 1: C.Diurna < C.Nocturna
	//           C.diurna                      C.nocturna
	// |----------|-------------------------------|---------------|
	// 0         hhmm1                          hhmm2            24
	//   nocturna             diurna                 nocturna

	if ( systemVars.consigna.horaConsDia < systemVars.consigna.horaConsNoc ) {

		// Caso A:
		if ( now <= systemVars.consigna.horaConsDia ) {
			u_setConsignaNocturna();
			snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: Nocturna\r\n\0"), u_now() );
			u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
			return;
		}
		// Caso B:
		if ( ( systemVars.consigna.horaConsDia <= now ) && ( now <= systemVars.consigna.horaConsNoc )) {
			u_setConsignaDiurna();
			snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: Diurna\r\n\0"), u_now() );
			u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
			return;
		}

		// Caso C:
		if ( now > systemVars.consigna.horaConsNoc ) {
			u_setConsignaNocturna();
			snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: Nocturna\r\n\0"), u_now() );
			u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
			return;
		}

		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("ERROR al setear consignas: horas incompatibles\r\n\0"));
		u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
		return;

	}


	// Caso 2: C.Nocturna < Diurna
	//           C.Nocturna                      C.diurna
	// |----------|-------------------------------|---------------|
	// 0         hhmm2                          hhmm1            24
	//   diurna             nocturna                 diurna

	if (  systemVars.consigna.horaConsNoc < systemVars.consigna.horaConsDia ) {

		// Caso A:
		if ( now <= systemVars.consigna.horaConsNoc ) {
			u_setConsignaDiurna();
			snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: Diurna\r\n\0"), u_now() );
			u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
			return;
		}
		// Caso B:
		if ( ( systemVars.consigna.horaConsNoc <= now ) && ( now <= systemVars.consigna.horaConsDia )) {
			u_setConsignaNocturna();
			snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: Nocturna\r\n\0"), u_now() );
			u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
			return;
		}

		// Caso C:
		if ( now > systemVars.consigna.horaConsDia ) {
			u_setConsignaDiurna();
			snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: Diurna\r\n\0"), u_now() );
			u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
			return;
		}

		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("ERROR al setear consignas: horas incompatibles\r\n\0"));
		u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
		return;

	}


}
//------------------------------------------------------------------------------------
void initDobleConsigna(void)
{
	tkDC_state = dcST_00;
	consigna4aplicar = CONSIGNA_NONE;
}
//------------------------------------------------------------------------------------

#endif
