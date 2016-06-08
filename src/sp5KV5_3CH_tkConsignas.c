/*
 * sp5KV5_3CH_tkConsignas.c
 *
 *  Created on: 1 de jun. de 2016
 *      Author: pablo
 */



#include "sp5KV5_3CH.h"

static char cons_printfBuff[CHAR128];

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

static void sm_DOBLECONSIGNA(void);
static void sm_CONSIGNAOFF(void);

static void pv_DCprintExitMsg(u08 code);
void pv_setConsignaInicial ( void );

// Eventos
typedef enum {
	dc_ev_CTIMER_NOT_0,
	dc_ev_CONS_DIA,
	dc_ev_CONS_NOCHE,
	dc_ev_MSG_RELOAD,
} t_eventos_dobleConsigna;

#define sm_DOBLECONSIGNA_EVENT_COUNT 4

#define max(a,b) ( a < b ? b : a)
#define min(a,b) ( a > b ? b : a)

static u08 cTimer;		// Contador de segundos

static u08 tkDC_state;
static u08 consigna4aplicar;
// ------------------------------------------------------------------------------------
// FUZZY
// ------------------------------------------------------------------------------------
#define MAXNAME 10
#define BGAP 0.05

struct io_type {			// Variables linguisticas de entrada/salida.
   char name[MAXNAME];
   float value;
   struct mf_type *membership_functions;
   struct io_type *next;
   };

struct mf_type {			// Definicion de las funciones de pertenencia ( trapecios )
   char name[MAXNAME];		// de una variable linguistica.
   float value;
   float xA,xB,xC,xD;
   struct mf_type *next;
   };

// Entradas fuzzy
struct io_type *System_Inputs;
struct io_type io_PB;
struct mf_type mf_pb_baja, mf_pb_medio, mf_pb_alta;

struct io_type io_DP;
struct mf_type mf_dp_baja, mf_dp_medio, mf_dp_alta;

// Salidas fuzzy
struct io_type *System_Outputs;
struct io_type io_PW;
struct mf_type mf_pw_corto, mf_pw_medio, mf_pw_largo;

void initialize_FuzzySystem(void);
void readFuzzyInputs(void);
void fuzzificacion(void);
void rule_evaluation(void);
void compute_degree_of_membership(struct mf_type *mf,float xin );
float compute_area_trapezoide(struct mf_type *mf );
void defuzzificacion(void);

// ------------------------------------------------------------------------------------
void fuzzy_test(void)
{
	// Funcion para ir probando desde linea de comandos el funcionamiento.

	initialize_FuzzySystem();

	readFuzzyInputs();
	fuzzificacion();
	rule_evaluation();
	defuzzificacion();
}
// ------------------------------------------------------------------------------------
void initialize_FuzzySystem(void)
{
	// Aqui defino las funciones de pertenencia de las entradas y salidas.
	//

	// ENTRADAS FUZZY:

	System_Inputs = &io_PB;

	strcpy(io_PB.name, "P_BAJA");	// Presion BAJA.
	io_PB.next = &io_DP;
	io_PB.value = 0;
	io_PB.membership_functions = &mf_pb_baja;

		// Los limites de c/funcion de pertenencia los marco separando la maxima presion
	    // admisible ( systemVars ) y dividiendola en 3 tramos.
		// Parametro configurable.
		strcpy( mf_pb_baja.name,"PB_bajo");
		mf_pb_baja.xA = 0.0;
		mf_pb_baja.xB = systemVars.cc_maxPb/4;
		mf_pb_baja.xC = systemVars.cc_maxPb/4;
		mf_pb_baja.xD = systemVars.cc_maxPb/2;
		mf_pb_baja.value = 0;
		mf_pb_baja.next = &mf_pb_medio;

		strcpy( mf_pb_medio.name, "PB_medio");
		mf_pb_medio.xA = systemVars.cc_maxPb/4;
		mf_pb_medio.xB = systemVars.cc_maxPb/2;
		mf_pb_medio.xC = systemVars.cc_maxPb/2;
		mf_pb_medio.xD = systemVars.cc_maxPb*3/4;
		mf_pb_medio.value = 0;
		mf_pb_medio.next = &mf_pb_alta;

		strcpy( mf_pb_alta.name, "PB_alta");
		mf_pb_alta.xA = systemVars.cc_maxPb/2;
		mf_pb_alta.xB = systemVars.cc_maxPb*3/4;
		mf_pb_alta.xC = 100;	// Max PB ( kg ) admisible
		mf_pb_alta.xD = 100;
		mf_pb_alta.value = 0;
		mf_pb_alta.next = NULL;

	strcpy(io_DP.name, "DIF_PRES");		// Diferencia de presion.
	io_DP.next = NULL;
	io_DP.value = 0;
	io_DP.membership_functions = &mf_dp_baja;

		strcpy( mf_dp_baja.name,"DP_baja");
		mf_dp_baja.xA = BGAP;		// 50
		mf_dp_baja.xB = 2*BGAP;		// 100
		mf_dp_baja.xC = 2*BGAP;		// 100
		mf_dp_baja.xD = 3*BGAP;		// 150
		mf_dp_baja.value = 0;
		mf_dp_baja.next = &mf_dp_medio;

		strcpy( mf_dp_medio.name, "DP_medio");
		mf_dp_medio.xA = 2*BGAP;	// 100
		mf_dp_medio.xB = 3*BGAP;	// 150
		mf_dp_medio.xC = 3*BGAP;	// 150
		mf_dp_medio.xD = 4*BGAP;	// 200
		mf_dp_medio.value = 0;
		mf_dp_medio.next = &mf_dp_alta;

		strcpy( mf_dp_alta.name, "DP_alta");
		mf_dp_alta.xA = 3*BGAP;		// 150
		mf_dp_alta.xB = 4*BGAP;		// 200
		mf_dp_alta.xC = 100;		// MAX_DP ( kg)
		mf_dp_alta.xD = 100;
		mf_dp_alta.value = 0;
		mf_dp_alta.next = NULL;

	// SALIDAS FUZZY
	// Para simplificar el calculo del centroide, las funciones de pertenencia
	// de las salidas deben ser simetricas.!!!!

	System_Outputs = &io_PW;

	strcpy(io_PW.name, "PW");	// Presion BAJA.
	io_PW.next = NULL;
	io_PW.value = 0;
	io_PW.membership_functions = &mf_pw_corto;

		strcpy( mf_pw_corto.name,"PW_corto");
		mf_pw_corto.xA = 0.0;
		mf_pw_corto.xB = systemVars.cc_maxPW/3;
		mf_pw_corto.xC = systemVars.cc_maxPW/3;
		mf_pw_corto.xD = systemVars.cc_maxPW*2/3;
		mf_pw_corto.value = 0;
		mf_pw_corto.next = &mf_pw_medio;

		strcpy( mf_pw_medio.name, "PW_medio");
		mf_pw_medio.xA = systemVars.cc_maxPW/3;
		mf_pw_medio.xB = systemVars.cc_maxPW*2/3;
		mf_pw_medio.xC = systemVars.cc_maxPW*2/3;
		mf_pw_medio.xD = systemVars.cc_maxPW;
		mf_pw_medio.value = 0;
		mf_pw_medio.next = &mf_pw_largo;

		strcpy( mf_pw_largo.name, "PW_largo");
		mf_pw_largo.xA = systemVars.cc_maxPW*2/3;
		mf_pw_largo.xB = systemVars.cc_maxPW;
		mf_pw_largo.xC = systemVars.cc_maxPW;
		mf_pw_largo.xD = systemVars.cc_maxPW*4/3;
		mf_pw_largo.value = 0;
		mf_pw_largo.next = NULL;

}
// ------------------------------------------------------------------------------------
void readFuzzyInputs(void)
{
	// Leo la entrada crisp ( pB ), calculo la otra entrada dp y
	// seteo en la estructura de entradas los valores para poder fuzzificarlos
	//
	io_PB.value = systemVars.cc_pBTest;
	io_DP.value = fabs(systemVars.cc_pBTest - systemVars.cc_pRef);

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("FUZZY INPUTS:: PB=%.03f, DP=%.03f\r\n\0"), io_PB.value, io_DP.value );
	u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

}
// ------------------------------------------------------------------------------------
void fuzzificacion(void)
{
	// Recorro la lista de entrada ( en horizontal y vertical ) y voy
	// evaluando el grado de pertenencia a c/funcion.

struct io_type *si;
struct mf_type *mf;

	for(si=System_Inputs;si!=NULL;si=si->next) {
		for(mf=si->membership_functions;mf!=NULL;mf=mf->next) {
			compute_degree_of_membership(mf,si->value);
		}
	}

}
// ------------------------------------------------------------------------------------
void rule_evaluation(void)
{
	// Aqui se definen las reglas
//	  	         1         2       3
//            pb_baja   pb_medio  pb_alta
//	1 dp_bajo   corto      medio    corto
//	2 dp_medio  medio      medio    corto
//	3 dp_alto   largo      medio    corto

	// Reglas que compiten por el pulso corto
	// corto = max( min(
	// a = min(1,1); b = min(3,1); c = min(3,2); d= min(3,3); e = min(2,3)
	// corto = max(a,b,c,d,e);
	mf_pw_corto.value = 0;
	mf_pw_corto.value = max(mf_pw_corto.value, min( mf_pb_baja.value, mf_dp_baja.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("Corto_1=%.03f (%.03f[%.03f,%.03f])r\n\0"), mf_pw_corto.value, mf_pb_baja.value, mf_dp_baja.value );
	u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_corto.value = max(mf_pw_corto.value, min( mf_pb_alta.value, mf_dp_baja.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("Corto_2=%.03f (%.03f[%.03f,%.03f])r\n\0"), mf_pw_corto.value, mf_pb_alta.value, mf_dp_baja.value );
	u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_corto.value = max(mf_pw_corto.value, min( mf_pb_alta.value, mf_dp_medio.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("Corto_3=%.03f (%.03f[%.03f,%.03f])r\n\0"), mf_pw_corto.value, mf_pb_alta.value, mf_dp_medio.value );
	u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_corto.value = max(mf_pw_corto.value, min( mf_pb_alta.value, mf_dp_alta.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("Corto_4=%.03f (%.03f[%.03f,%.03f])r\n\0"), mf_pw_corto.value, mf_pb_alta.value, mf_dp_alta.value );
	u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_medio.value = 0;
	mf_pw_medio.value = max(mf_pw_medio.value, min( mf_pb_baja.value, mf_dp_medio.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("Medio_1=%.03f (%.03f[%.03f,%.03f])r\n\0"), mf_pw_medio.value, mf_pb_baja.value, mf_dp_medio.value );
	u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_medio.value = max(mf_pw_medio.value, min( mf_pb_medio.value, mf_dp_baja.value) );
	mf_pw_medio.value = max(mf_pw_medio.value, min( mf_pb_medio.value, mf_dp_medio.value) );
	mf_pw_medio.value = max(mf_pw_medio.value, min( mf_pb_medio.value, mf_dp_alta.value) );

	mf_pw_largo.value = 0;
	mf_pw_largo.value = max(mf_pw_largo.value, min( mf_pb_baja.value, mf_dp_alta.value) );

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("FUZZY RULES:: PW_corto=%.03f, PW_medio=%.03f, PW_largo=%.03f\r\n\0"), mf_pw_corto.value, mf_pw_medio.value, mf_pw_largo.value );
	u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

}
// ------------------------------------------------------------------------------------
void defuzzificacion(void)
{

struct io_type *so;
struct mf_type *mf;
float sum_of_products, sum_of_areas, area, centroide;

	for(so=System_Outputs;so!=NULL;so=so->next) {
		sum_of_products = 0;
		sum_of_areas = 0;
		for(mf=so->membership_functions;mf!=NULL;mf=mf->next) {
			area = compute_area_trapezoide(mf);
			centroide = mf->xA + ( mf->xD - mf->xA)/2;
			sum_of_products += area * centroide;
			sum_of_areas += area;
			snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("UNFUZZY:: [%s](A=%.03f,Cent=%.03f)\r\n\0"), mf->name, area, centroide );
			u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

		}

		if ( sum_of_areas == 0 ) {
			so->value = 0;
			return;
		}

		so->value = sum_of_products / sum_of_areas;

		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("UNFUZZY END:: [%s]=%.03f\r\n\0"), so->name,so->value );
		u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

	}
}
// ------------------------------------------------------------------------------------
void compute_degree_of_membership(struct mf_type *mf,float xin )
{
	// Asumimos que las funciones de membresia son triangulares y calculo
	// el valor de esta, para una entrada dada.

float xa = mf->xA;
float xb = mf->xB;
float xc = mf->xC;
float xd = mf->xD;
float res;

	if ( xin < xa) {
		res = 0;
	} else if ( ( xa <= xin ) && ( xin < xb) ) {
		res = ( (xin - xa) / ( xb - xa));
	} else if ( ( xb <= xin ) && ( xin < xc ) ) {
		res = 1;
	} else if ( ( xc <= xin ) && ( xin < xd ) ) {
		res = ( ( xd - xin ) / ( xd - xc ));
	} else {
		res = 0;
	}
	mf->value = res;

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("FUZZY:: [%s](%.03f,%.03f,%.03f,%.03f)->(%.03f::%.03f)\r\n\0"), mf->name, mf->xA, mf->xB, mf->xC,mf->xD, xin, mf->value );
	u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

}
// ------------------------------------------------------------------------------------
float compute_area_trapezoide(struct mf_type *mf )
{
float base,top,side_1,side_2, area;

	base = mf->xD - mf->xA;
	side_1 = (mf->xB - mf->xA) * mf->value;
	side_2 = (mf->xD - mf->xC) * mf->value;
	top = base - side_1 - side_2;
	area = ( base + top ) * mf->value / 2;
	return(area);

}
// ------------------------------------------------------------------------------------
void tkConsignas(void * pvParameters)
{

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("starting tkConsignas..\r\n\0"));
	FreeRTOS_write( &pdUART1, cons_printfBuff, sizeof(cons_printfBuff) );

	tkDC_state = dcST_00;
	consigna4aplicar = CONSIGNA_NONE;
	//
	for( ;; )
	{

		u_clearWdg(WDG_CSG);

		// Espero hasta 100ms por un mensaje.
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 100 / portTICK_RATE_MS ) );
		// Si llego un mensaje, prendo la flag correspondiente.
		if ( xResult == pdTRUE ) {
			if ( ( ulNotifiedValue & TK_PARAM_RELOAD ) != 0 ) {
				// Inicializo todo
				tkDC_state = dcST_00;
				consigna4aplicar = CONSIGNA_NONE;
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
			vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
			break;
		default:
			snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("tkConsignas::ERROR type(%d) NOT DEFINED..\r\n\0"),systemVars.consigna.type);
	 		FreeRTOS_write( &pdUART1, cons_printfBuff,sizeof(cons_printfBuff) );
	 		systemVars.consigna.type  = CONSIGNA_OFF;
	 		break;
		}
	}
}
// ------------------------------------------------------------------------------------
static void sm_CONSIGNAOFF(void)
{
	// Espero indefinidamente.
	 vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

}
// ------------------------------------------------------------------------------------
static void sm_DOBLECONSIGNA(void)
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
	pv_setConsignaInicial();

	pv_DCprintExitMsg(0);
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

	//pv_DCprintExitMsg(1);
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

	pv_DCprintExitMsg(2);
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

	pv_DCprintExitMsg(3);
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

	pv_DCprintExitMsg(4);
	return(dcST_04);
}
//------------------------------------------------------------------------------------
static int cTR_DC05(void)
{

	// Inicializo.
	cTimer = 30;

	pv_DCprintExitMsg(5);
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

	pv_DCprintExitMsg(6);
	return(dcST_03);
}
//------------------------------------------------------------------------------------
static int cTR_DC07(void)
{
	// Reconfiguro la consigna DIURNA
	u_setConsignaDiurna();
	cTimer = 30;
	consigna4aplicar = CONSIGNA_NONE;

	pv_DCprintExitMsg(7);
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

	pv_DCprintExitMsg(8);
	return(dcST_04);
}
//------------------------------------------------------------------------------------
static int cTR_DC09(void)
{
	// Reconfiguro la consigna NOCTURNA
	u_setConsignaNocturna();
	cTimer = 30;
	consigna4aplicar = CONSIGNA_NONE;


	pv_DCprintExitMsg(9);
	return(dcST_01);
}
//------------------------------------------------------------------------------------
static void pv_DCprintExitMsg(u08 code)
{

u32 tickCount;

	tickCount = xTaskGetTickCount();
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(".[%06lu] tkConsigna(DC)::exit anDC_%02d\r\n\0"), tickCount,code);
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );
}
//------------------------------------------------------------------------------------
void pv_setConsignaInicial ( void )
{
	// Determino cual consigna corresponde aplicar y la aplico.
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

	//           C.diurna                      C.nocturna
	// |----------|-------------------------------|---------------|
	// 0         hhmm1                          hhmm2            24
	//   nocturna             diurna                 nocturna

	if ( now <= systemVars.consigna.horaConsDia ) {
		u_setConsignaNocturna();
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: Nocturna\r\n\0"), u_now() );
		u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

	if ( ( now > systemVars.consigna.horaConsDia ) && ( now <= systemVars.consigna.horaConsNoc )) {
		u_setConsignaDiurna();
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: Diurna\r\n\0"), u_now() );
		u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

	if ( now > systemVars.consigna.horaConsNoc ) {
		u_setConsignaNocturna();
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: Nocturna\r\n\0"), u_now() );
		u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

}
//------------------------------------------------------------------------------------
