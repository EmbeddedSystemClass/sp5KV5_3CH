/*
 * sp5KV3_tkAnalogIn.c
 *
 *  Created on: 14/4/2015
 *      Author: pablo
 */


#include "sp5KV5_3CH.h"

#if defined(OSE_3CH) || defined(UTE_8CH)

static char aIn_printfBuff[CHAR256];
TimerHandle_t pollingTimer;

// Estados
typedef enum {	anST_A00 = 0,
				anST_A01,
				anST_A02,
				anST_A03,
} t_analogState;

// Eventos
typedef enum {
	an_ev_RELOADCONFIG = 0,		// EV_MSGreload
	an_ev_START2POLL,			// EV_f_start2poll
	an_ev_CTIMER_NOT_0,			// timer/counter general
	an_ev_POLL_NOW,
} t_analogEventos;

#define anEVENT_COUNT		4

static s08 anEventos[anEVENT_COUNT];

// transiciones
static int anTR_00(void);
static int anTR_01(void);
static int anTR_02(void);
static int anTR_03(void);
static int anTR_04(void);
static int anTR_05(void);
static int anTR_06(void);
static int anTR_07(void);

static struct {
	s08 msgReload;			// flags de los mensajes recibidos.
	s08 start2poll;			// flag que habilita a polear.
	s08 msgPollNow;			// mensaje de POLL_FRAME
	s08 saveFrameInBD;		//
} AN_flags;

static struct {
	u16 cTimer;
	u08 pollFrame;
	u16 secs4poll;
	u16 secs4save;
} AN_counters;

static u08 tkAIN_state = anST_A00;				// Estado
static double rAIn[NRO_ANALOG_CHANNELS + 1];	// Almaceno los datos de conversor A/D
static frameData_t Aframe;

#define CICLOS_POLEO		3		// ciclos de poleo para promediar.
#define SECS2PWRSETTLE 		3

// Funciones generales
void  pv_ANtimerCallback( TimerHandle_t pxTimer );
static void pv_ANgetNextEvent(void);
static void pv_AINfsm(void);
static void pv_AINprintExitMsg(u08 code);

//--------------------------------------------------------------------------------------
 void tkAnalogIn(void * pvParameters)
{

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("starting tkAnalogIn..\r\n\0"));
	FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );

	tkAIN_state = anST_A00;		// Estado inicial.
	AN_flags.msgReload = FALSE;		// No tengo ningun mensaje de reload pendiente.
	AN_flags.msgPollNow = FALSE;

	// Arranco el timer de poleo.
	// Interrumpe c/1s.
	if ( xTimerStart( pollingTimer, 0 ) != pdPASS )
		u_panic(P_AIN_TIMERSTART);

	//
	for( ;; )
	{

		u_clearWdg(WDG_AIN);

		// Espero hasta 100ms por un mensaje.
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 100 / portTICK_RATE_MS ) );
		// Si llego un mensaje, prendo la flag correspondiente.
		if ( xResult == pdTRUE ) {

			if ( ( ulNotifiedValue & TK_PARAM_RELOAD ) != 0 ) {
				// Mensaje de reload configuration.
				AN_flags.msgReload = TRUE;
			}

			if ( ( ulNotifiedValue & TK_READ_FRAME ) != 0 ) {
				// Mensaje de polear un frame ( estando en modo servicio )
				if ( systemVars.wrkMode == WK_SERVICE )
					AN_flags.msgPollNow = TRUE;
			}
		}

		// Analizo los eventos.
		pv_ANgetNextEvent();
		// Corro la maquina de estados.
		pv_AINfsm();
	}

}
/*------------------------------------------------------------------------------------*/
void tkAnalogInit(void)
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
						 pv_ANtimerCallback
	                   );

	if ( pollingTimer == NULL )
		u_panic(P_OUT_TIMERCREATE);
}
//------------------------------------------------------------------------------------
void pv_ANtimerCallback( TimerHandle_t pxTimer )
{
	// El timer esta en reload c/1 sec, aqui contamos los secs para
	// completar poleo y lo indico prendiendo la flag correspondiente.
	// En consigna continua poleo c/60s.
	// En modo service poleo c/15s
	// En otro modo, poleo c/systemVars.timerPoll

	// Ajusto los timers.
	if ( AN_counters.secs4poll > 0 ) {
		--AN_counters.secs4poll;
	}

	if ( AN_counters.secs4save > 0 ) {
		--AN_counters.secs4save;
	}

	// Control del salvado de datos en memoria.
	if ( AN_counters.secs4save == 0 ) {
		AN_counters.secs4save = systemVars.timerPoll;

		if ( systemVars.wrkMode == WK_NORMAL ) {
			AN_flags.saveFrameInBD = TRUE;
		}
	}

	// Control del poleo
	if ( AN_counters.secs4poll == 0 ) {
		AN_flags.start2poll = TRUE;

		// Reajusto el timers.
		if ( systemVars.consigna.type == CONSIGNA_CONTINUA ) {
			AN_counters.secs4poll = 60;
		}  else if ( systemVars.wrkMode == WK_MONITOR_FRAME ) {
			// Monitoreo c/30s pero no salvo en memoria
			AN_counters.secs4poll = 30;
		} else {
			AN_counters.secs4poll = systemVars.timerPoll;
		}

//		snprintf_P( aIn_printfBuff,CHAR128,PSTR("DEBUG SEC4POLL=%d\r\n\0"), AN_counters.secs4poll);
//		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );

	}

}
//--------------------------------------------------------------------------------------
static void pv_ANgetNextEvent(void)
{
// Evaluo todas las condiciones que generan los eventos que disparan las transiciones.
// Tenemos un array de eventos y todos se evaluan.

u08 i;

	// Inicializo la lista de eventos.
	for ( i=0; i < anEVENT_COUNT; i++ ) {
		anEventos[i] = FALSE;
	}

	// Evaluo los eventos
	// EV01: EV_MSGreload: recargar la configuracion
	if ( AN_flags.msgReload == TRUE ) { anEventos[an_ev_RELOADCONFIG] = TRUE;	}
	// EV02: EV_f_start2poll
	if ( AN_flags.start2poll == TRUE ) { anEventos[an_ev_START2POLL] = TRUE; }
	// EV03: EV_counter_NOT_0
	if ( AN_counters.cTimer != 0 ) { anEventos[an_ev_CTIMER_NOT_0] = TRUE; }
	// EV04: EV_POLL_NOE
	if ( AN_flags.msgPollNow == TRUE ) { anEventos[an_ev_POLL_NOW] = TRUE; 	}

}
/*------------------------------------------------------------------------------------*/
static void pv_AINfsm(void)
{
	// El manejar la FSM con un switch por estado y no por transicion me permite
	// priorizar las transiciones.
	// Luego de c/transicion debe venir un break así solo evaluo de a 1 transicion por loop.
	//

	switch ( tkAIN_state ) {
	case anST_A00:
		tkAIN_state = anTR_00();
		break;
	case anST_A01:
		if (  anEventos[an_ev_RELOADCONFIG] ) {
			tkAIN_state = anTR_01();
		} else if ( anEventos[an_ev_POLL_NOW] ){
			tkAIN_state = anTR_02();
		} else if (anEventos[an_ev_START2POLL] ) {
			tkAIN_state = anTR_03();
		}
		break;
	case anST_A02:
		if ( anEventos[an_ev_CTIMER_NOT_0] ) {
			tkAIN_state = anTR_04();
		} else {
			tkAIN_state = anTR_05();
		}
		break;
	case anST_A03:
		if ( anEventos[an_ev_CTIMER_NOT_0] ) {
			tkAIN_state = anTR_06();
		} else {
			tkAIN_state = anTR_07();
		}
		break;

	default:
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("tkAnalogIn::ERROR state NOT DEFINED..\r\n\0"));
		FreeRTOS_write( &pdUART1, aIn_printfBuff,sizeof(aIn_printfBuff) );
		tkAIN_state  = anST_A00;
		break;

	}
}
/*------------------------------------------------------------------------------------*/
static int anTR_00(void)
{
	// Inicializo el sistema aqui

	AN_flags.msgReload = FALSE;
	AN_flags.start2poll = FALSE;
	AN_flags.saveFrameInBD = FALSE;
	AN_counters.secs4poll = 15;
	AN_counters.secs4save = 15;

	// PwrOn de sensores: solo en modo continuo.
	if ( systemVars.pwrMode == PWR_CONTINUO ) {
		MCP_setSensorPwr( 1 );
		MCP_setAnalogPwr( 1 );
	}

	pv_AINprintExitMsg(0);
	return(anST_A01);
}
/*------------------------------------------------------------------------------------*/
static int anTR_01(void)
{
	// MSG de autoreload configuration

	AN_flags.msgReload = FALSE;
	AN_flags.start2poll = FALSE;
	AN_flags.saveFrameInBD = FALSE;
	AN_counters.secs4poll = 15;
	AN_counters.secs4save = 15;

	// PwrOn de sensores: solo en modo continuo
	if ( systemVars.pwrMode == PWR_CONTINUO ) {
		MCP_setSensorPwr( 1 );
		MCP_setAnalogPwr( 1 );
	}

	pv_AINprintExitMsg(1);
	return(anST_A01);
}
/*------------------------------------------------------------------------------------*/
static int anTR_02(void)
{

	// Tengo un mensaje que debo polear.
	AN_flags.msgPollNow = FALSE;

	AN_flags.start2poll = FALSE;

	// Es service: no salvo en EE
	AN_flags.saveFrameInBD = FALSE;

	// Para no tener problemas con el timer
	AN_counters.secs4poll = 60;
	AN_counters.secs4save = 60;

	// Si estoy en modo discreto, debo prender la fuente y esperar que se asiente.
	if ( systemVars.pwrMode == PWR_DISCRETO ) {
		MCP_setSensorPwr( 1 );
		MCP_setAnalogPwr( 1 );
		AN_counters.cTimer = SECS2PWRSETTLE;
	} else {
		AN_counters.cTimer = 0;
	}

	pv_AINprintExitMsg(2);
	return(anST_A02);
}
//------------------------------------------------------------------------------------
static int anTR_03(void)
{

	// Inicio un poleo.
	AN_flags.start2poll = FALSE;

	// Si estoy en modo discreto, debo prender la fuente y esperar que se asiente.
	if ( systemVars.pwrMode == PWR_DISCRETO ) {
		MCP_setSensorPwr( 1 );
		MCP_setAnalogPwr( 1 );
		AN_counters.cTimer = SECS2PWRSETTLE;
	} else {
		AN_counters.cTimer = 0;
	}

	pv_AINprintExitMsg(3);
	return(anST_A02);
}
/*------------------------------------------------------------------------------------*/
static int anTR_04(void)
{
	// Espero que se asiente la fuente de los sensores

	// Espero 1 segundo
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	if ( AN_counters.cTimer > 0 )
		--AN_counters.cTimer;

	//pv_AINprintExitMsg(4);
	return(anST_A02);
}
/*------------------------------------------------------------------------------------*/
static int anTR_05(void)
{

s08 retS;
u16 adcRetValue;
u08 i;

	// Init Data Structure
	for ( i=0; i < (NRO_ANALOG_CHANNELS + 1); i++ )
		rAIn[i] = 0;

	// Hago una conversion dummy
	retS = ADS7827_readCh0( &adcRetValue);
	vTaskDelay( ( TickType_t)( 1500 / portTICK_RATE_MS ) );

	// Inicializo el contador de poleos
	AN_counters.cTimer = CICLOS_POLEO;

	pv_AINprintExitMsg(5);
	return(anST_A03);

}
/*------------------------------------------------------------------------------------*/
static int anTR_06(void)
{
	// Genero un poleo y espero 1s

s08 retS;
u16 adcRetValue;
u08 i;

	// Espero 1 segundo
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	if ( AN_counters.cTimer > 0 )
		--AN_counters.cTimer;

#ifdef OSE_3CH
	// Genero un poleo.
	retS = ADS7827_readCh0( &adcRetValue);	// AIN0->ADC3;
	rAIn[0] += adcRetValue;
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("\tch_0,adc_3,val=%d,r0=%.0f\r\n\0"),adcRetValue, rAIn[0]);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );

	retS = ADS7827_readCh1( &adcRetValue); // AIN1->ADC5;
	rAIn[1] += adcRetValue;
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("\tch_1,adc_5,val=%d,r1=%.0f\r\n\0"),adcRetValue, rAIn[1]);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );

	retS = ADS7827_readCh2( &adcRetValue); // AIN2->ADC7;
	rAIn[2] += adcRetValue;
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("\tch_2,adc_7,val=%d,r1=%.0f\r\n\0"), adcRetValue, rAIn[2]);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );

	retS = ADS7827_readBatt( &adcRetValue); // BATT->ADC1;
	rAIn[3] += adcRetValue;
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("\tch_3,adc_1,val=%d,r1=%.0f\r\n\0"), adcRetValue, rAIn[3]);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
#endif

#ifdef UTE_8CH
	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++ ) {
		retS = ADS7828_read(i, &adcRetValue);
		rAIn[i] += adcRetValue;
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("\tch_%02d,adc_%02d,val=%d,r%02d=%.0f\r\n\0"),i,i,adcRetValue,i,rAIn[i]);
		u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}
#endif

	pv_AINprintExitMsg(6);
	return(anST_A03);

}
/*------------------------------------------------------------------------------------*/
static int anTR_07(void)
{
	// En modo discreto apago los sensores.
	// Promedio
	// Convierto a magnitud.
	// Completo el frame con fechaHora y datos digitales.
	// Imprimo
	// Si corresponde, salvo en BD.


u32 tickCount;
double I,M;
u08 i;
u16 D;
u08 channel;
u16 pos = 0;
size_t bWrite;
StatBuffer_t pxFFStatBuffer;

	//  En modo discreto debo apagar sensores
	if ( systemVars.pwrMode == PWR_DISCRETO )  {
		MCP_setSensorPwr( 0 );
		MCP_setAnalogPwr( 0 );
	}

	// Promedio canales analogicos y bateria
	for ( channel = 0; channel < (NRO_ANALOG_CHANNELS + 1); channel++) {
		rAIn[channel] /= CICLOS_POLEO;
		tickCount = xTaskGetTickCount();
		snprintf_P( aIn_printfBuff,CHAR128,PSTR(".[%06lu] tkAnalogIn::trD06 AvgCh[%d]=%.02f\r\n\0"), tickCount, channel, rAIn[channel]);
		u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	// Convierto los canales analogicos a magnitudes.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS ; channel++) {
		// Calculo la corriente medida en el canal
		I = rAIn[channel] * systemVars.Imax[channel] / 4096;
		// Calculo la pendiente
		M = 0;
		D = systemVars.Imax[channel] - systemVars.Imin[channel];
		if ( D != 0 ) {
			M = ( systemVars.Mmax[channel]  -  systemVars.Mmin[channel] ) / D;
			rAIn[channel] = systemVars.Mmin[channel] + M * ( I - systemVars.Imin[channel] );
		} else {
			// Error: denominador = 0.
			rAIn[channel] = -999;
		}
	}

#ifdef OSE_3CH

	// Convierto la bateria.
	rAIn[NRO_ANALOG_CHANNELS] = (15 * rAIn[NRO_ANALOG_CHANNELS]) / 4096;	// Bateria

	// DEBUG
	for ( channel = 0; channel <= NRO_ANALOG_CHANNELS; channel++) {
		tickCount = xTaskGetTickCount();
		snprintf_P( aIn_printfBuff,CHAR128,PSTR(".[%06lu] tkAnalogIn::trD06 MagCh[%d]=%.02f\r\n\0"), tickCount, channel, rAIn[channel]);
		u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}
#endif

#ifdef UTE_8CH
	// DEBUG
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		tickCount = xTaskGetTickCount();
		snprintf_P( aIn_printfBuff,CHAR128,PSTR(".[%06lu] tkAnalogIn::trD06 MagCh[%d]=%.02f\r\n\0"), tickCount, channel, rAIn[channel]);
		u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}
#endif

	// Armo el frame.
	RTC_read(&Aframe.rtc);

	// Analogico
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		Aframe.analogIn[channel] = rAIn[channel];
	}

#ifdef OSE_3CH
	// Bateria
	Aframe.batt = rAIn[3];
#endif

	// Digital
	u_readDigitalCounters( &Aframe.dIn, TRUE );
	// Convierto los pulsos a los valores de la magnitud.
	for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++ ) {
		Aframe.dIn.pulses[i] *=  systemVars.magPP[i];
	}

	// Guardo en BD ?
	if ( AN_flags.saveFrameInBD ) {
		AN_flags.saveFrameInBD = FALSE;
		bWrite = FF_fwrite( &Aframe, sizeof(Aframe));
		FF_stat(&pxFFStatBuffer);

		if ( bWrite != sizeof(Aframe) ) {
			// Error de escritura ??
			snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("WR ERROR: (%d)\r\n\0"),pxFFStatBuffer.errno);
		} else {
			// Stats de memoria
			snprintf_P( aIn_printfBuff, sizeof(aIn_printfBuff), PSTR("MEM [%d/%d/%d][%d/%d]\r\n\0"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		}
		u_debugPrint(D_BASIC, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

#ifdef UTE_8CH
	// Imprimo el frame.
	pos = snprintf_P( aIn_printfBuff, sizeof(aIn_printfBuff), PSTR("FRAME::{" ));
	// timeStamp.
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%02d%02d%02d"),Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );
	// Valores analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR(",%s=%.02f"),systemVars.aChName[channel],Aframe.analogIn[channel] );
	}
	// Valores digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR(",%s{P=%.02f,L=%d,T=%d}"), systemVars.dChName[channel],Aframe.dIn.pulses[channel],Aframe.dIn.level[channel],Aframe.dIn.secsUp[channel] );
	}

	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("}\r\n\0") );
	u_logPrint (aIn_printfBuff, sizeof(aIn_printfBuff) );

#endif

#ifdef OSE_3CH
	// Imprimo el frame.
	pos = snprintf_P( aIn_printfBuff, sizeof(aIn_printfBuff), PSTR("FRAME::{" ));
	// timeStamp.
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%02d%02d%02d"),Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );
	// Valores analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR(",%s=%.02f"),systemVars.aChName[channel],Aframe.analogIn[channel] );
	}
	// Valores digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR(",%sP=%.02f"), systemVars.dChName[channel],Aframe.dIn.pulses[channel] );
	}
	// Bateria
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR(",bt=%.02f}\r\n\0"),Aframe.batt );
	FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
#endif


#ifdef CONSIGNA
	// Envio un mensaje a la tarea de la consigna diciendole que estan los datos listos
	if ( systemVars.consigna.type == CONSIGNA_CONTINUA ) {
		while ( xTaskNotify(xHandle_tkConsignas, TKC_FRAME_READY , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
	//	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("DEBUG SEND MSG 2 CC\r\n\0"));
	//	FreeRTOS_write( &pdUART1, aIn_printfBuff,sizeof(aIn_printfBuff) );
	}
#endif

	AN_flags.start2poll = FALSE;

	pv_AINprintExitMsg(7);
	return(anST_A01);
}
//------------------------------------------------------------------------------------
static void pv_AINprintExitMsg(u08 code)
{

u32 tickCount;

	tickCount = xTaskGetTickCount();
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR(".[%06lu] tkAnalogIn::exit anTR_%02d\r\n\0"), tickCount,code);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
}
//------------------------------------------------------------------------------------
s16 u_readTimeToNextPoll(void)
{
s16 retVal = -1;

	// Lo determina en base al time elapsed y el timerPoll.
	// El -1 indica un modo en que no esta poleando.
	if ( ( systemVars.wrkMode == WK_NORMAL ) || ( systemVars.wrkMode == WK_MONITOR_FRAME )) {
		retVal = AN_counters.secs4save;
	}

	return (retVal);
}
/*------------------------------------------------------------------------------------*/
void u_readDataFrame (frameData_t *dFrame)
{

	memcpy(dFrame, &Aframe, sizeof(Aframe) );
}
/*------------------------------------------------------------------------------------*/

#endif

