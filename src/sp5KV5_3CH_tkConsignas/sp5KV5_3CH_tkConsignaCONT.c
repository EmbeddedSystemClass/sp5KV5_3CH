/*
 * sp5KV5_3CH_tkConsignaCONT.c
 *
 *  Created on: 10 de jun. de 2016
 *      Author: pablo
 */

#include "../sp5KV5_3CH.h"
#include "sp5KV5_3CH_tkConsignas.h"

#ifdef CONSIGNA
// ------------------------------------------------------------------------------------
// CONSIGNA CONTINUA
// ------------------------------------------------------------------------------------
// Estados
typedef enum {	ccST_00 = 0,
				ccST_01,
				ccST_02,
				ccST_03,
} t_consignaContinuaState;

// Transiciones
static int cTR_CC00(void);
static int cTR_CC01(void);
static int cTR_CC02(void);
static int cTR_CC03(void);
static int cTR_CC04(void);
static int cTR_CC05(void);
static int cTR_CC06(void);

// Eventos
typedef enum {
	cc_ev_PWTIMER_NOT_0,
	cc_ev_PW_NOT_0,
	cc_ev_FRAMEREADY,
} t_eventos_consignaContinua;

#define sm_CONSIGNACONTINUA_EVENT_COUNT 3

typedef enum { V_CERRAR, V_ABRIR } t_modoOperarValvulas;
void pv_DCsetValvulas ( u08 vAlta, u08 vBaja );

// ------------------------------------------------------------------------------------

#define MAXNAME 12
#define BGAP 0.05

#define max(a,b) ( a < b ? b : a)
#define min(a,b) ( a > b ? b : a)

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
struct mf_type mf_pb_muyBaja, mf_pb_baja, mf_pb_media, mf_pb_alta, mf_pb_muyAlta;

struct io_type io_DP;
struct mf_type  mf_dp_baja, mf_dp_media, mf_dp_alta;

// Salidas fuzzy
struct io_type *System_Outputs;
struct io_type io_PW;
struct mf_type mf_pw_corto, mf_pw_medio, mf_pw_largo;

void pv_calcularPW_LINEAL(void);
void pv_calcularPW_FUZZY(void);

void initialize_FuzzySystem(void);
void readFuzzyInputs(void);
void fuzzificacion(void);
void rule_evaluation(void);
void compute_degree_of_membership(struct mf_type *mf,float xin );
float compute_area_trapezoide(struct mf_type *mf );
void defuzzificacion(void);

static u08 tkCC_state;
static float pulseWidth;
static u16 pwTimerX100ms;

#define FUZZY_CYCLE_IN_SEC	60	// Tiempo en el que se repite el ciclo

//------------------------------------------------------------------------------------
void sm_CONSIGNACONTINUA(void)
{
s08 cc_eventos[sm_CONSIGNACONTINUA_EVENT_COUNT];
u08 i;

	// Inicializo la lista local de eventos.
	for ( i=0; i < sm_CONSIGNACONTINUA_EVENT_COUNT; i++ ) {
		cc_eventos[i] = FALSE;
	}

	// Evaluo
	if ( pwTimerX100ms > 0  ) { cc_eventos[cc_ev_PWTIMER_NOT_0] = TRUE; }
	if ( pulseWidth != 0.0 ) { cc_eventos[cc_ev_PW_NOT_0] = TRUE; }
	if ( f_dc_frameReady == TRUE ) { cc_eventos[cc_ev_FRAMEREADY] = TRUE; }

	// Corro la FSM
	switch ( tkCC_state ) {
	case ccST_00:
		tkCC_state = cTR_CC00();
		break;
	case ccST_01:
		if ( cc_eventos[cc_ev_FRAMEREADY] ) {
			tkCC_state = cTR_CC02();
		} else {
			tkCC_state = cTR_CC01();
		}
		break;
	case ccST_02:
		if ( cc_eventos[cc_ev_PW_NOT_0] ) {
			tkCC_state = cTR_CC03();
		} else {
			tkCC_state = cTR_CC04();
		}
		break;
	case ccST_03:
		if ( cc_eventos[cc_ev_PWTIMER_NOT_0] ) {
			tkCC_state = cTR_CC05();
		} else {
			tkCC_state = cTR_CC06();
		}
		break;
	default:
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("\r\ntkConsignas::ERROR Consigna Continua: State  (%d) NOT DEFINED\r\n\0"),tkCC_state );
		FreeRTOS_write( &pdUART1, cons_printfBuff, sizeof(cons_printfBuff) );
		tkCC_state = ccST_00;
		break;
	}

}
// ------------------------------------------------------------------------------------
static int cTR_CC00(void)
{

	// Inicializo.
	f_dc_frameReady = FALSE;

	pv_consignaPrintExitMsg(0);
	return(ccST_01);
}
//------------------------------------------------------------------------------------
static int cTR_CC01(void)
{
	// No hago nada. solo espero por un mensaje de la tarea de poleo.
	//pv_consignaPrintExitMsg(1);
	return(ccST_01);
}
//------------------------------------------------------------------------------------
static int cTR_CC02(void)
{
	// Llego un mensaje que estan los datos de la preson listos.
	// Arranco una regulacion.
	// Regulo aplicando la logica fuzzy

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy START:: [%s]\r\n\0"), u_now() );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	// Borro la flag que me prendio la tk_Analog.
	f_dc_frameReady = FALSE;

	pv_calcularPW_LINEAL();
	//pv_calcularPW_FUZZY();

	pv_consignaPrintExitMsg(2);
	return(ccST_02);
}
//------------------------------------------------------------------------------------
static int cTR_CC03(void)
{
	// Aplico un pulso de apertura de valvulas para regulacion

	if ( ( systemVars.cc_pBTest - systemVars.cc_pRef ) > 0 ) {
		// pB alta: debo cerrar la valvula para que baje similar a aplicar
		// la consigna nocturna, o sea ingresar agua desde pA
		pv_DCsetValvulas(V_ABRIR, V_CERRAR);
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy_VALVES: A-OPEN, B-CLOSE\r\n\0"));
	} else {
		// pB baja: debo abrir la valvula para que aumente, similar a aplicar
		// la consigna diurna, o sea dejar escapar agua
		pv_DCsetValvulas(V_CERRAR, V_ABRIR);
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy_VALVES: A-CLOSE, B-OPEN\r\n\0"));
	}

	pwTimerX100ms = (u16)(10 * pulseWidth ); // Tiempo en intervalos de 100ms que hay que esperar

	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	pv_consignaPrintExitMsg(3);
	return(ccST_03);
}
//------------------------------------------------------------------------------------
static int cTR_CC04(void)
{
	// No hay que aplicar pulso: salgo.

	pv_consignaPrintExitMsg(4);
	return(ccST_01);
}
//------------------------------------------------------------------------------------
static int cTR_CC05(void)
{
	// Espero el tiempo del ancho del pulso aplicado PW
	// No es un sleep de 1s sino que como la tarea se ejecuta c/100ms,
	// el minimo intervalo para contar son estos 100ms.
	// Esto me asegura que paso por aqui c/100ms.

	 if ( pwTimerX100ms > 0 ) {
		 --pwTimerX100ms;
	 }

	//pv_consignaPrintExitMsg(5);
	return(ccST_03);
}
//------------------------------------------------------------------------------------
static int cTR_CC06(void)
{

	// Aplico un pulso de cierre de regulacion y salgo a
	// esperar 1 minuto por otro ciclo.
	pv_DCsetValvulas(V_CERRAR, V_CERRAR);
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy_VALVES: A-CLOSE, B-CLOSE\r\n\0"));
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy END\r\n\0"));
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	pv_consignaPrintExitMsg(6);
	return(ccST_01);
}
//------------------------------------------------------------------------------------
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
	io_PB.membership_functions = &mf_pb_muyBaja;

		// Los limites de c/funcion de pertenencia los marco separando la maxima presion
	    // admisible ( systemVars ) y dividiendola en 3 tramos.
		// Parametro configurable.
		strcpy( mf_pb_muyBaja.name,"PB_muyBaja");
		mf_pb_muyBaja.xA = 0.0;
		mf_pb_muyBaja.xB = 0.0;
		mf_pb_muyBaja.xC = 0.0;
		mf_pb_muyBaja.xD = systemVars.cc_maxPb/4;
		mf_pb_muyBaja.value = 0;
		mf_pb_muyBaja.next = &mf_pb_baja;

		strcpy( mf_pb_baja.name,"PB_baja");
		mf_pb_baja.xA = 0.0;
		mf_pb_baja.xB = systemVars.cc_maxPb/4;
		mf_pb_baja.xC = systemVars.cc_maxPb/4;
		mf_pb_baja.xD = systemVars.cc_maxPb/2;
		mf_pb_baja.value = 0;
		mf_pb_baja.next = &mf_pb_media;

		strcpy( mf_pb_media.name, "PB_media");
		mf_pb_media.xA = systemVars.cc_maxPb/4;
		mf_pb_media.xB = systemVars.cc_maxPb/2;
		mf_pb_media.xC = systemVars.cc_maxPb/2;
		mf_pb_media.xD = systemVars.cc_maxPb*3/4;
		mf_pb_media.value = 0;
		mf_pb_media.next = &mf_pb_alta;

		strcpy( mf_pb_alta.name, "PB_alta");
		mf_pb_alta.xA = systemVars.cc_maxPb/2;
		mf_pb_alta.xB = systemVars.cc_maxPb*3/4;
		mf_pb_alta.xC = 100;	// Max PB ( kg ) admisible
		mf_pb_alta.xD = 100;
		mf_pb_alta.value = 0;
		mf_pb_alta.next = &mf_pb_muyAlta;

		strcpy( mf_pb_muyAlta.name, "PB_muyAlta");
		mf_pb_muyAlta.xA = systemVars.cc_maxPb/2;
		mf_pb_muyAlta.xB = systemVars.cc_maxPb*3/4;
		mf_pb_muyAlta.xC = 100;	// Max PB ( kg ) admisible
		mf_pb_muyAlta.xD = 100;
		mf_pb_muyAlta.value = 0;
		mf_pb_muyAlta.next = NULL;

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
		mf_dp_baja.next = &mf_dp_media;

		strcpy( mf_dp_media.name, "DP_media");
		mf_dp_media.xA = 2*BGAP;	// 100
		mf_dp_media.xB = 3*BGAP;	// 150
		mf_dp_media.xC = 3*BGAP;	// 150
		mf_dp_media.xD = 4*BGAP;	// 200
		mf_dp_media.value = 0;
		mf_dp_media.next = &mf_dp_alta;

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

frameData_t Cframe;

//	io_PB.value = systemVars.cc_pBTest;
//	io_DP.value = fabs(systemVars.cc_pBTest - systemVars.cc_pRef);

	memset(&Cframe,'\0', sizeof(frameData_t));
	u_readAnalogFrame (&Cframe);
	io_PB.value = Cframe.analogIn[1];
	io_DP.value = fabs(io_PB.value - systemVars.cc_pRef);

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy_inputs:: PB=%.03f, Pref=%.03f, DP=%.03f\r\n\0"), io_PB.value, systemVars.cc_pRef, io_DP.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

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
//	1 dp_bajo   corto      corto    corto
//	2 dp_medio  medio      medio    corto
//	3 dp_alto   largo      medio    corto
/*
 *                 1          2       3       4        5
 *            	muyBaja		baja	media	alta	muyAlta
 *  1-dp_bajo	corto		corto	corto	corto	medio
 *  2-dp_medio	corto		medio	medio	medio	corto
 *  3-dp_alto	medio		alto	alto	alto	medio
 *
 */

	// Reglas que compiten por el pulso corto
	// corto = max( min(
	// a = min(1,1); b = min(3,1); c = min(3,2); d= min(3,3); e = min(2,3)
	// corto = max(a,b,c,d,e);

	mf_pw_corto.value = 0;
	mf_pw_corto.value = max(mf_pw_corto.value, min( mf_pb_muyBaja.value, mf_dp_baja.value) );	//1,1
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy_rules::MAX( min[%.03f,%.03f],"),mf_pb_muyBaja.value, mf_dp_baja.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_corto.value = max(mf_pw_corto.value, min( mf_pb_baja.value, mf_dp_baja.value) );		//2,1
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" min[%.03f,%.03f],"),mf_pb_baja.value, mf_dp_baja.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_corto.value = max(mf_pw_corto.value, min( mf_pb_media.value, mf_dp_baja.value) );		//3,1
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("min[%.03f,%.03f],"), mf_pb_media.value, mf_dp_baja.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_corto.value = max(mf_pw_corto.value, min( mf_pb_alta.value, mf_dp_baja.value) );		//4,1
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("min[%.03f,%.03f],"), mf_pb_alta.value, mf_dp_baja.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_corto.value = max(mf_pw_corto.value, min( mf_pb_muyBaja.value, mf_dp_media.value) );	//1,2
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("min[%.03f,%.03f],"), mf_pb_muyBaja.value, mf_dp_media.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_corto.value = max(mf_pw_corto.value, min( mf_pb_muyAlta.value, mf_dp_media.value) );		//5,2
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("min[%.03f,%.03f] ) = %.03f (pwCorto)\r\n\0"), mf_pb_muyAlta.value, mf_dp_media.value,mf_pw_corto.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	/*
	 *                 1          2       3       4        5
	 *            	muyBaja		baja	media	alta	muyAlta
	 *  ---------------------|--------|-------|-------|----------
	 *  1-dp_bajo |										medio
	 *  2-dp_medio|				medio	medio	medio
	 *  3-dp_alto |	medio								medio
	 *
	 */

	mf_pw_medio.value = 0;
	mf_pw_medio.value = max(mf_pw_medio.value, min( mf_pb_muyAlta.value, mf_dp_baja.value) );		//5,1
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy_rules::MAX( min[%.03f,%.03f],"), mf_pb_muyAlta.value, mf_dp_baja.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_medio.value = max(mf_pw_medio.value, min( mf_pb_baja.value, mf_dp_media.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("min[%.03f,%.03f],"), mf_pb_baja.value, mf_dp_media.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_medio.value = max(mf_pw_medio.value, min( mf_pb_media.value, mf_dp_media.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("min[%.03f,%.03f],"), mf_pb_media.value, mf_dp_media.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_medio.value = max(mf_pw_medio.value, min( mf_pb_alta.value, mf_dp_media.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("min[%.03f,%.03f],"), mf_pb_alta.value, mf_dp_media.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_medio.value = max(mf_pw_medio.value, min( mf_pb_muyBaja.value, mf_dp_alta.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("min[%.03f,%.03f],"), mf_pb_muyBaja.value, mf_dp_alta.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_medio.value = max(mf_pw_medio.value, min( mf_pb_muyAlta.value, mf_dp_alta.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("min[%.03f,%.03f] ) = %.03f (pwMedio)\r\n\0"), mf_pb_muyAlta.value, mf_dp_alta.value,mf_pw_medio.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	/*
	 *                 1          2       3       4        5
	 *            	muyBaja		baja	media	alta	muyAlta
	 *  ---------------------|--------|-------|-------|----------
	 *  1-dp_bajo	|
	 *  2-dp_medio	|
	 *  3-dp_alto	|			alto	alto	alto
	 *
	 */

	mf_pw_largo.value = 0;
	mf_pw_largo.value = max(mf_pw_largo.value, min( mf_pb_baja.value, mf_dp_alta.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy_rules::MAX( min[%.03f,%.03f],"), mf_pb_baja.value, mf_dp_alta.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_largo.value = max(mf_pw_largo.value, min( mf_pb_media.value, mf_dp_alta.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("min[%.03f,%.03f],"), mf_pb_media.value, mf_dp_alta.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	mf_pw_largo.value = max(mf_pw_largo.value, min( mf_pb_alta.value, mf_dp_alta.value) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy_rules::MAX( min[%.03f,%.03f] ) = %.03f (pwLargo)\r\n\0"), mf_pb_alta.value, mf_dp_alta.value, mf_pw_largo.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy_rules::PW_corto=%.03f, PW_medio=%.03f, PW_largo=%.03f\r\n\0"), mf_pw_corto.value, mf_pw_medio.value, mf_pw_largo.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

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
			snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy_outputs:: [%s](A=%.03f,Cent=%.03f)\r\n\0"), mf->name, area, centroide );
			u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

		}

		if ( sum_of_areas == 0 ) {
			so->value = 0;
		} else {
			so->value = sum_of_products / sum_of_areas;
		}

		pulseWidth = so->value;
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy_OUTPUT:: [%s]=%.03f\r\n\0"), so->name,so->value );
		u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" DEBUG_CONSIGNAS:: [pB=%.03f][pRef=%.03f][dp=%.03f]->pw=%.03f\r\n\0"),io_PB.value, systemVars.cc_pRef, io_DP.value,so->value );
		u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

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

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" fuzzy_membership:: [%s](%.03f,%.03f,%.03f,%.03f)->(%.03f::%.03f)\r\n\0"), mf->name, mf->xA, mf->xB, mf->xC,mf->xD, xin, mf->value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

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
void initConsignaContinua(void)
{

}
// ------------------------------------------------------------------------------------
void pv_DCsetValvulas ( u08 vAlta, u08 vBaja )
{
	// Activacion simultaneamente las  2 valvulas
	// Para abrir una valvula debemos poner una fase 10.
	// Para cerrar es 01

	// Por las dudas, reconfiguro el MCP
//	return;

	//pvMCP_init_MCP1(1);

	switch(vAlta) {
	case V_CERRAR: // Cierro la valvula de alta ( PA )
		MCP_outsPulse( systemVars.consigna.chVA , 1, 100 );		// Cierro la valvula 1
		break;
	case V_ABRIR: // Abro la valvula de alta
		 MCP_outsPulse( systemVars.consigna.chVA , 0, 100 );	// Abro la valvula 1
		 break;
	}

	switch(vBaja) {
	case V_CERRAR: // Cierro la valvula de altja ( PB )
		MCP_outsPulse( systemVars.consigna.chVB , 1, 100 );		// Cierro la valvula 2
		break;
	case V_ABRIR: // Abro la valvula de baja
		 MCP_outsPulse( systemVars.consigna.chVB , 0, 100 );	// Abro la valvula 2
		 break;
	}

	 // Dejo el sistema de salidas en reposo para que no consuma
	 MCP_outputA1Disable();
	 MCP_outputA2Disable();
	 MCP_outputB1Disable();
	 MCP_outputB2Disable();

	 MCP_outputsSleep();
}
//------------------------------------------------------------------------------------
void pv_calcularPW_LINEAL(void)
{
	// Calculo el ancho de pulso en modo lineal

frameData_t Cframe;

	memset(&Cframe,'\0', sizeof(frameData_t));
	u_readAnalogFrame (&Cframe);
	io_PB.value = Cframe.analogIn[1];
	io_DP.value = fabs(io_PB.value - systemVars.cc_pRef);

	// Calculo lineal
	if ( io_DP.value <= 0.05 ) {
		pulseWidth = 0;
	} else if (  ( io_DP.value > 0.05 ) &&  ( io_DP.value <= 0.5 ) ) {
		pulseWidth = (u08)( systemVars.cc_maxPW * 2/3 * io_DP.value / 0.5 );
	} else {
		pulseWidth = systemVars.cc_maxPW * 2/3;
	}

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" CCONT_LINEAL_INPUTS:: PB=%.03f, Pref=%.03f, DP=%.03f\r\n\0"), io_PB.value, systemVars.cc_pRef, io_DP.value );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(" CCONT_LINEAL_PW:: [pw=%.03f]\r\n\0"), pulseWidth );
	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );

}
//------------------------------------------------------------------------------------
void pv_calcularPW_FUZZY(void)
{
	// Calculo el ancho de pulso usando logica fuzzy

	initialize_FuzzySystem();
	readFuzzyInputs();
	fuzzificacion();
	rule_evaluation();
	defuzzificacion();	// En la variable pulseWidth esta cuanto debe esperar por el pulso
}
//------------------------------------------------------------------------------------

#endif
