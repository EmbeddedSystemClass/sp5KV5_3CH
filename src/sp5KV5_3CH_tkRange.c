/*
 * sp5KV5_PZ_tkRange.c
 *
 *  Created on: 22 de jun. de 2016
 *      Author: pablo
 *
 *  Retorno:
 *   1 : No puede detectar los flancos del pulso
 *  -1 : La distancia medida es > rango_maximo
 *
 */


#include "sp5KV5_3CH.h"

#ifdef OSE_POZOS

static char range_printfBuff[CHAR128];

TimerHandle_t pollingTimer;

// Estados
typedef enum {	rgST_R00 = 0,
				rgST_R01,
				rgST_R02,

} t_rangeState;

// Eventos
typedef enum {
	rg_ev_RELOADCONFIG = 0,		// EV_MSGreload
	rg_ev_START2POLL,			// EV_f_start2poll
	rg_ev_POLL_NOW,
	rg_ev_DIN_CHANGE,
	rg_ev_POLL,
	rg_ev_cCOUNT_NOT_0,
} t_rangeEventos;

#define rgEVENT_COUNT		6

static s08 rgEventos[rgEVENT_COUNT];

// transiciones
static int rgTR_00(void);
static int rgTR_01(void);
static int rgTR_02(void);
static int rgTR_03(void);
static int rgTR_04(void);
static int rgTR_05(void);
static int rgTR_06(void);

static struct {
	s08 msgReload;			// flags de los mensajes recibidos.
	s08 start2poll;			// flag que habilita a polear.
	s08 msgPollNow;			// mensaje de POLL_FRAME
	s08 saveFrameInBD;		//
	s08 dInChange;
	s08 pingStatus;
} RANGE_flags;

static struct {
	u16 secs4poll;
	u08 cCount;
} RANGE_counters;

static u08 tkRANGE_state = rgST_R00;	// Estado
static frameData_t Aframe;

static u08 din0,din1;

#define MRANGE_DIFF		5	// Maxima diferencia (en cms) entre medidas consecutivas
#define MAX_PING_TRYES	3	// Cantidad de poleos.

static s16 distBuffer[MAX_PING_TRYES];

// Funciones generales
void  pv_RANGEtimerCallback( TimerHandle_t pxTimer );
static void pv_RANGEgetNextEvent(void);
static void pv_RANGEfsm(void);
static void pv_RANGEprintExitMsg(u08 code);
static s08 pv_awaitLineHIGH(void);
static s08 pv_awaitLineLOW(void);
static s16 pv_ping(void );
static s08  pv_checkDIN4Change(void);
static void pv_pollInit(void);

#define TIMER1_START ( TCCR1B |= ( 1 << CS11 ))	// Prescaler x8: cuento de a uS
#define TIMER1_STOP ( TCCR1B &= ~( 1 << CS11 ))

static s16 reported_distancia;

//--------------------------------------------------------------------------------------
void tkRange(void * pvParameters)
{

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("starting tkRange..\r\n\0"));
	FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );

	tkRANGE_state = rgST_R00;		// Estado inicial.
	RANGE_flags.msgReload = FALSE;	// No tengo ningun mensaje de reload pendiente.
	RANGE_flags.msgPollNow = FALSE;

	 Aframe.analogIn[0] = 0;
	 Aframe.dIn.pulses[0] = 0;
	 Aframe.dIn.pulses[1] = 0;

	reported_distancia = 0;

	u_rangeSignal(STOP);

	// Inicializo las entradas digitales.
	din0 = ( ( RM_DIN0_PIN & _BV(RM_DIN0_BIT) ) >> RM_DIN0_BIT );
	din1 = ( ( RM_DIN1_PIN & _BV(RM_DIN1_BIT) ) >> RM_DIN1_BIT );

	// Arranco el timer de poleo.
	// Interrumpe c/1s.
	if ( xTimerStart( pollingTimer, 0 ) != pdPASS )
		u_panic(P_RANGE_TIMERSTART);

	//
	for( ;; )
	{

		u_clearWdg(WDG_RANGE);

		// Espero hasta 100ms por un mensaje.
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 100 / portTICK_RATE_MS ) );
		// Si llego un mensaje, prendo la flag correspondiente.
		if ( xResult == pdTRUE ) {

			if ( ( ulNotifiedValue & TK_PARAM_RELOAD ) != 0 ) {
				// Mensaje de reload configuration.
				RANGE_flags.msgReload = TRUE;
			}

			if ( ( ( ulNotifiedValue & TK_READ_FRAME ) != 0 ) && ( systemVars.wrkMode == WK_SERVICE )) {
				// Mensaje de polear un frame ( estando en modo servicio )
				RANGE_flags.msgPollNow = TRUE;
			}
		}

		// Analizo los eventos.
		pv_RANGEgetNextEvent();
		// Corro la maquina de estados.
		pv_RANGEfsm();
	}

}
/*------------------------------------------------------------------------------------*/
void tkRangeInit(void)
{
	// Esta funcion se utiliza  antes de arrancar el FRTOS de modo que cree
	// el timer que necesitamos en este modulo
	// Expira c/1sec

	pollingTimer = xTimerCreate (  "POLL_T",
	                     /* The timer period in ticks, must be greater than 0. */
	                     ( 1000 / portTICK_PERIOD_MS) ,
	                     /* The timers will auto-reload themselves when they expire. */
	                     pdTRUE,
	                     /* Assign each timer a unique id equal to its array index. */
	                     ( void * ) NULL,
	                     /* Each timer calls the same callback when it expires. */
						 pv_RANGEtimerCallback
	                   );

	if ( pollingTimer == NULL )
		u_panic(P_RANGE_TIMERCREATE);
}
//------------------------------------------------------------------------------------
void pv_RANGEtimerCallback( TimerHandle_t pxTimer )
{
	// El timer esta en reload c/1 sec, aqui contamos los secs para
	// completar poleo y lo indico prendiendo la flag correspondiente.
	// En consigna continua poleo c/60s.
	// En modo service poleo c/15s
	// En otro modo, poleo c/systemVars.timerPoll

	// Ajusto los timers.
	if ( RANGE_counters.secs4poll > 0 ) {
		--RANGE_counters.secs4poll;
	}

	// Control del poleo
	if ( RANGE_counters.secs4poll == 0 ) {

		switch(systemVars.wrkMode) {
		case WK_NORMAL:
			RANGE_counters.secs4poll = systemVars.timerPoll;
			RANGE_flags.start2poll = TRUE;
			break;
		case WK_MONITOR_FRAME:
			RANGE_counters.secs4poll = 15;
			RANGE_flags.start2poll = TRUE;
			break;
		case WK_SERVICE:
			RANGE_counters.secs4poll = 0xFFFF;
			break;
		}
	}

	// Como se ejecuta 1 vez por sec, aqui chequeo si las entradas digitales cambian
	pv_checkDIN4Change();

}
//--------------------------------------------------------------------------------------
static void pv_RANGEgetNextEvent(void)
{
// Evaluo todas las condiciones que generan los eventos que disparan las transiciones.
// Tenemos un array de eventos y todos se evaluan.

u08 i;

	// Inicializo la lista de eventos.
	for ( i=0; i < rgEVENT_COUNT; i++ ) {
		rgEventos[i] = FALSE;
	}

	// Evaluo los eventos
	// Llego un mensaje de reconfiguracion
	if ( RANGE_flags.msgReload == TRUE ) { rgEventos[rg_ev_RELOADCONFIG] = TRUE; }
	// Expiro el timer de espera y hay que comenzar un poleo
	if ( RANGE_flags.start2poll == TRUE ) { rgEventos[rg_ev_START2POLL] = TRUE; }
	// En modo sevice por comando dieron un read frame
	if ( RANGE_flags.msgPollNow == TRUE ) { rgEventos[rg_ev_POLL_NOW] = TRUE; }
	// Cambiaron las entradas digitales
	if ( RANGE_flags.dInChange == TRUE ) { rgEventos[rg_ev_DIN_CHANGE] = TRUE; }
	// el contador de eventos no ha expirado
	if ( RANGE_counters.cCount > 0 ) { rgEventos[rg_ev_cCOUNT_NOT_0] = TRUE; }

}
/*------------------------------------------------------------------------------------*/
static void pv_RANGEfsm(void)
{
	// El manejar la FSM con un switch por estado y no por transicion me permite
	// priorizar las transiciones.
	// Luego de c/transicion debe venir un break así solo evaluo de a 1 transicion por loop.
	//

	switch ( tkRANGE_state ) {
	case rgST_R00:
		tkRANGE_state = rgTR_00();
		break;
	case rgST_R01:
		if (  rgEventos[rg_ev_RELOADCONFIG] ) {
			tkRANGE_state = rgTR_01();
		} else if ( rgEventos[rg_ev_POLL_NOW] ){
			tkRANGE_state = rgTR_02();
		} else if (rgEventos[rg_ev_START2POLL] ) {
			tkRANGE_state = rgTR_03();
		} else if (rgEventos[rg_ev_DIN_CHANGE] ) {
			tkRANGE_state = rgTR_04();
		}
		break;
	case rgST_R02:
		if (  rgEventos[rg_ev_cCOUNT_NOT_0] ) {
			tkRANGE_state = rgTR_05();
		} else {
			tkRANGE_state = rgTR_06();
		}
		break;
	default:
		snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("tkRange::ERROR state NOT DEFINED..\r\n\0"));
		FreeRTOS_write( &pdUART1, range_printfBuff,sizeof(range_printfBuff) );
		tkRANGE_state  = rgST_R00;
		break;

	}
}
/*------------------------------------------------------------------------------------*/
static int rgTR_00(void)
{
	// Transicion inicial: solo inicializo el sistema

	RANGE_flags.msgReload = FALSE;
	RANGE_flags.start2poll = FALSE;
	RANGE_flags.saveFrameInBD = FALSE;
	RANGE_counters.secs4poll = 15;

	pv_RANGEprintExitMsg(0);
	return(rgST_R01);
}
/*------------------------------------------------------------------------------------*/
static int rgTR_01(void)
{
	// Llego un mensaje de autoreload configuration

	RANGE_flags.msgReload = FALSE;

	xTimerStop(  pollingTimer, 0 );
	RANGE_counters.secs4poll = 15;
	xTimerStart( pollingTimer, 0 );

	pv_RANGEprintExitMsg(1);
	return(rgST_R01);
}
//------------------------------------------------------------------------------------
static int rgTR_02(void)
{

	// Tengo un mensaje que debo polear( x modo cmd, read Frame)

	RANGE_flags.msgPollNow = FALSE;		// Polear ahora
	RANGE_flags.saveFrameInBD = FALSE;	// Pero no salvar en memoria los datos.

	snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("(S) Polling...\r\n\0"));
	FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );

	pv_pollInit();

	pv_RANGEprintExitMsg(2);
	return(rgST_R02);
}
//------------------------------------------------------------------------------------
static int rgTR_03(void)
{

	// Inicio un poleo normal porque expiro el timer ( comandado por timerPoll )
	RANGE_flags.start2poll = FALSE;

	pv_pollInit();

	// En modo monitor o service frame no guardo en memoria.
	switch(systemVars.wrkMode) {
	case WK_NORMAL:
		RANGE_flags.saveFrameInBD = TRUE;
		break;
	case WK_MONITOR_FRAME:
		RANGE_flags.saveFrameInBD = FALSE;
		break;
	case WK_SERVICE:
		RANGE_flags.saveFrameInBD = FALSE;
		break;
	}

	pv_RANGEprintExitMsg(3);
	return(rgST_R02);
}
//------------------------------------------------------------------------------------
static int rgTR_04(void)
{
	// Cambio el DIN: debo polear

	RANGE_flags.dInChange = FALSE;
	RANGE_flags.saveFrameInBD = TRUE;

	pv_pollInit();

	pv_RANGEprintExitMsg(4);
	return(rgST_R02);
}
//------------------------------------------------------------------------------------
static int rgTR_05(void)
{

	// Poleo CCOUNT veces para luego promediar.
	// Espero 1s entre poleos.

	if ( RANGE_counters.cCount > 0 ) {
		--RANGE_counters.cCount;
	}

	// Poleo: Mido el nivel
	distBuffer[RANGE_counters.cCount] = pv_ping();

	vTaskDelay( ( TickType_t)( 1500 / portTICK_RATE_MS ) );

	pv_RANGEprintExitMsg(5);
	return(rgST_R02);
}
//------------------------------------------------------------------------------------
static int rgTR_06(void)
{
	// Mido
	// Apago el sensor.
	// Completo el frame con fechaHora y datos digitales.
	// Imprimo
	// Si corresponde, salvo en BD.

u16 pos = 0;
size_t bWrite;
StatBuffer_t pxFFStatBuffer;
s16 distancia = 0;
u08 i;
s08 frameValid = TRUE;

	u_rangeSignal(STOP);

	// Vemos si tenemos una medida correcta ( diferencia entre ellas menor a MRANGE_DIFF cms )
	for (i=1;i<MAX_PING_TRYES;i++) {
		// Si alguna diferencia es grande, anulo la medida.
		if ( abs( distBuffer[i-1] - distBuffer[i] ) >  MRANGE_DIFF ) {
			frameValid = FALSE;
			break;
		}
	}

	// Armo el frame.
	RTC_read(&Aframe.rtc);

	if ( frameValid ) {
		// Range valido:
		Aframe.status = 0;
		// Promedio
		for (i=0;i<MAX_PING_TRYES;i++) {
			distancia += distBuffer[i];
		}
		 distancia /= MAX_PING_TRYES;
		 reported_distancia =  distancia;

	} else {
		// Error en medidas.Uso el range anterior
		Aframe.status = 128;
	}

	// La distancia que guardo para luego reportar es la ultima calculada bien
	 Aframe.analogIn[0] = reported_distancia;

	// Guardo los datos digitales
	Aframe.dIn.level[0] = din0;
	Aframe.dIn.level[1] = din1;

	// Guardo en BD ?
	if ( RANGE_flags.saveFrameInBD ) {
		RANGE_flags.saveFrameInBD = FALSE;
		bWrite = FF_fwrite( &Aframe, sizeof(Aframe));
		FF_stat(&pxFFStatBuffer);

		if ( bWrite != sizeof(Aframe) ) {
			// Error de escritura ??
			snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("WR ERROR: (%d)\r\n\0"),pxFFStatBuffer.errno);
			FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );
		} else {
			// Stats de memoria
			snprintf_P( range_printfBuff, sizeof(range_printfBuff), PSTR("MEM [%d/%d/%d][%d/%d]\r\n\0"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
			u_debugPrint(D_BASIC, range_printfBuff, sizeof(range_printfBuff) );
		}

	}

	// Imprimo el frame.
	if ( Aframe.status != 0 ) {
		pos = snprintf_P( range_printfBuff, sizeof(range_printfBuff), PSTR("DATA(err){" ));
	} else {
		pos = snprintf_P( range_printfBuff, sizeof(range_printfBuff), PSTR("DATA {" ));
	}

	// timeStamp.
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR("%02d%02d%02d,"),Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );
	// Valores
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR("%s=%.0f,"),systemVars.aChName[0],Aframe.analogIn[0l] );
	// Valores digitales
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR("%s=%d,"), systemVars.dChName[0],Aframe.dIn.level[0]);
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR("%s=%d"), systemVars.dChName[1],Aframe.dIn.level[1]);

	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR("}\r\n\0") );
	u_logPrint (range_printfBuff, sizeof(range_printfBuff) );

	// Me preparo para un nuevo poleo
	RANGE_flags.start2poll = FALSE;

	pv_RANGEprintExitMsg(10);
	return(rgST_R01);
}
//------------------------------------------------------------------------------------
static void pv_RANGEprintExitMsg(u08 code)
{

u32 tickCount;

	tickCount = xTaskGetTickCount();
	snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR(".[%06lu] tkRange::exit rgTR_%02d\r\n\0"), tickCount,code);
	u_debugPrint(D_DATA, range_printfBuff, sizeof(range_printfBuff) );
}
//------------------------------------------------------------------------------------
s16 u_readTimeToNextPoll(void)
{
s16 retVal = -1;

	// Lo determina en base al time elapsed y el timerPoll.
	// El -1 indica un modo en que no esta poleando.
	if ( ( systemVars.wrkMode == WK_NORMAL ) || ( systemVars.wrkMode == WK_MONITOR_FRAME )) {
		retVal = RANGE_counters.secs4poll;
	}

	return (retVal);
}
/*------------------------------------------------------------------------------------*/
void u_readDataFrame (frameData_t *dFrame)
{
	memcpy(dFrame, &Aframe, sizeof(Aframe) );
}
//----------------------------------------------------------------------------------------
static s08 pv_awaitLineHIGH(void)
{
	// Espero que PB2 este arriba.

u08 pin;
TickType_t xTicksToWait = 200;
TimeOut_t xTimeOut;
s08 retS = FALSE;

	vTaskSetTimeOutState( &xTimeOut );

	while (1) {
		pin  = ( ( RM_PW_PIN & _BV(RM_PW_BIT) ) >> RM_PW_BIT );
		// High ??
		if ( pin == 1 ) {
			// Lo apago inmediatamente
			TIMER1_STOP;
			retS = TRUE;
			break;
		}
		// Timeout ??
	     if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
	         break;
	}
	return(retS);

}
//----------------------------------------------------------------------------------------
static s08 pv_awaitLineLOW(void)
{
	// Espero que PB2 este low.

u08 pin;
TickType_t xTicksToWait = 200;
TimeOut_t xTimeOut;
s08 retS = FALSE;

	vTaskSetTimeOutState( &xTimeOut );
	TCNT1 = 0;

	while (1) {
		pin  = ( ( RM_PW_PIN & _BV(RM_PW_BIT) ) >> RM_PW_BIT );
		// Low ??
		if ( pin == 0 ) {
			// Lo prendo inmediatamente
			TCNT1 = 0;
			TIMER1_START;
			retS = TRUE;
			break;
		}
		// Timeout ??
	     if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
	         break;
	}
	return(retS);
}
//----------------------------------------------------------------------------------------
static s16 pv_ping(void )
{
	// Genera un disparo y mide

u32 tickCount;
s16 distancia;

	//u_rangeSignal(RUN);

	if ( ! pv_awaitLineHIGH() ) {		// Espero un flanco de bajada
		snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("RangeMeter ERROR: no detect High L1\r\n\0"));
		FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );
		return(-2);
	}

	// Espero que baje y lo prendo: comienzo a medir
	if ( ! pv_awaitLineLOW() ) {
		snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("RangeMeter ERROR: no detect Low\r\n\0"));
		FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );
		return(-3);
	}


	if ( ! pv_awaitLineHIGH() ) {		// Espero el flanco de subida
		snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("RangeMeter ERROR: no detect High L2\r\n\0"));
		FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );
		return(-4);
	}

	//u_rangeSignal(STOP);

	distancia = (TCNT1 / 58);
	if ( distancia > systemVars.maxRange ) {
		distancia = -1;
	}

	tickCount = xTaskGetTickCount();
	snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR(".[%06lu] tkRange::ping:: (%d) Distancia=%d TCNT1=%d\r\n\0"),tickCount,RANGE_counters.cCount,distancia,TCNT1);
	u_debugPrint(D_DATA, range_printfBuff, sizeof(range_printfBuff) );

	return(distancia);
}
//----------------------------------------------------------------------------------------
static s08 pv_checkDIN4Change(void)
{
	// Leo las entradas digitales y determino si cambiaron
	// Si cambiaron prendo la flag para que se guarde inmediatamente el frame
	// asi me queda registrado cuando se prendieron/apagaron las bombas.

u08 din;
s08 changed = FALSE;

	din = ( ( RM_DIN0_PIN & _BV(RM_DIN0_BIT) ) >> RM_DIN0_BIT );
	if ( din != din0 ) {
		din0 = din;
		changed = TRUE;
	}

	din = ( ( RM_DIN1_PIN & _BV(RM_DIN1_BIT) ) >> RM_DIN1_BIT );
	if ( din != din1 ) {
		din1 = din;
		changed = TRUE;
	}

	RANGE_flags.dInChange = changed;
	return(changed);

}
//----------------------------------------------------------------------------------------
static void pv_pollInit(void)
{
u08 i;

	RANGE_counters.cCount = MAX_PING_TRYES;

	for ( i = 0; i < MAX_PING_TRYES; i++ ) {
		distBuffer[i] = 0;
	}

	u_rangeSignal(RUN);
}
//----------------------------------------------------------------------------------------

#endif
