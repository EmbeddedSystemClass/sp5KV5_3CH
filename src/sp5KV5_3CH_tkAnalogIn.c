/*
 * sp5KV3_tkAnalogIn.c
 *
 *  Created on: 14/4/2015
 *      Author: pablo
 */


#include "sp5KV5_3CH.h"

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
} t_analogEventos;

#define anEVENT_COUNT		3

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
	s08 slotComplete;		// indica que expiro el tiempo de un slot.
	s08 saveFrameInBD;		//
} AN_flags;

static struct {
	u16 cTimer;
	u08 pollFrame;
	u16 secs2poll;
} AN_counters;

static u08 tkAIN_state = anST_A00;				// Estado
static double rAIn[NRO_ANALOG_CHANNELS + 1];	// Almaceno los datos de conversor A/D
static frameData_t Aframe;

#define CICLOS_POLEO		3		// ciclos de poleo para promediar.
#define SECS2PWRSETTLE 		3
#define SECS_IN_SLOT		15		// cada slot corresponde a 15s

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

			if ( ( ulNotifiedValue & TKA_READ_FRAME ) != 0 ) {
				// Mensaje de polear un frame ( estando en modo servicio )
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
	// completar un slot y lo indico prendiendo la flag correspondiente.

static u08 slot_timer = SECS_IN_SLOT;

	if (--slot_timer == 0 ) {
		// Compete un slot.
		slot_timer = SECS_IN_SLOT;
		AN_flags.slotComplete = TRUE;
	}

	if ( --AN_counters.secs2poll == 0 ) {
		AN_counters.secs2poll = systemVars.timerPoll;
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
		} else if ( AN_flags.start2poll ) {
			tkAIN_state = anTR_02();
		} else {
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

	AN_flags.start2poll = FALSE;
	AN_flags.slotComplete = FALSE;
	AN_flags.saveFrameInBD = FALSE;
	AN_counters.pollFrame = 1;
	AN_counters.secs2poll = ( AN_counters.pollFrame * SECS_IN_SLOT );

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
	AN_flags.slotComplete = FALSE;
	AN_flags.saveFrameInBD = FALSE;
	AN_counters.pollFrame = 1;
	AN_counters.secs2poll = ( AN_counters.pollFrame * SECS_IN_SLOT );

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

	pv_AINprintExitMsg(2);
	return(anST_A02);
}
/*------------------------------------------------------------------------------------*/
static int anTR_03(void)
{
	// Evaluo las condiciones para poder polear
	// Los slots son de 15s y los pauta pv_ANtimerCallback. Aqui cuento estos slots
	// para ver cuando poleo y si corresponde a un poleo de datos para alamacenar o
	// para el sistema de consignas.

	// Espero 1 segundo
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	if ( AN_counters.cTimer > 0 )
		--AN_counters.cTimer;

	// Chequeo el status para evaluar AN_flags.start2poll.
	AN_flags.start2poll = FALSE;
	AN_flags.saveFrameInBD = FALSE;

	if ( AN_flags.slotComplete ) {

		// complete un slot, pasaron 15s.Reseteo la flag.
		AN_flags.slotComplete = FALSE;
		--AN_counters.pollFrame;

		// Veo si complete un intervalo de poleo. ( pollFrame = N sots = timerPoll )
		if ( AN_counters.pollFrame == 0 ) {
			AN_counters.pollFrame = ( systemVars.timerPoll / SECS_IN_SLOT );
			AN_counters.secs2poll = ( AN_counters.pollFrame * SECS_IN_SLOT );

			// En modo nomal debo polear porque se completo un pollFrame
			if ( systemVars.wrkMode == WK_NORMAL ) {
				AN_flags.start2poll = TRUE;
				AN_flags.saveFrameInBD = TRUE;
			}
		}

		// Si la consigna es continua, poleo en cada slot
		//if ( systemVars.consigna.status == CONSIGNA_CONTINUA ) {
		//	AN_flags.start2poll = TRUE;
		//}

		// En modo MONITOR_FRAME poleo en cada slot
		if ( systemVars.wrkMode == WK_MONITOR_FRAME  ) {
			AN_flags.start2poll = TRUE;
			AN_counters.secs2poll = SECS_IN_SLOT;
		}
	}

	// Sin importar cuando, en modo service, si tengo un mensaje debo polear.
	if ( ( systemVars.wrkMode == WK_SERVICE ) && ( AN_flags.msgPollNow ) ) {
		AN_flags.start2poll = TRUE;
	}

	// Actualizo las flags.( ya las evalue )
	AN_flags.msgPollNow = FALSE;

	//pv_AINprintExitMsg(3);
	return(anST_A01);
}
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

	// Espero 1 segundo
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	if ( AN_counters.cTimer > 0 )
		--AN_counters.cTimer;

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

	// Promedio canales analogicos 0..2 y bateria (3)
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

	// Convierto la bateria.
	rAIn[NRO_ANALOG_CHANNELS] = (15 * rAIn[NRO_ANALOG_CHANNELS]) / 4096;	// Bateria

	// DEBUG
	for ( channel = 0; channel <= NRO_ANALOG_CHANNELS; channel++) {
		tickCount = xTaskGetTickCount();
		snprintf_P( aIn_printfBuff,CHAR128,PSTR(".[%06lu] tkAnalogIn::trD06 MagCh[%d]=%.02f\r\n\0"), tickCount, channel, rAIn[channel]);
		u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}


	// Armo el frame.
	RTC_read(&Aframe.rtc);
	// Analogico
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		Aframe.analogIn[channel] = rAIn[channel];
	}
	// Bateria
	Aframe.batt = rAIn[3];
	// Digital
	u_readDigitalCounters( &Aframe.dIn, TRUE );
	Aframe.dIn.pulses[0] *=  systemVars.magPP[0];
	Aframe.dIn.pulses[1] *=  systemVars.magPP[1];

	// Guardo en BD ?
	if ( AN_flags.saveFrameInBD ) {
		bWrite = FF_fwrite( &Aframe, sizeof(Aframe));
		FF_stat(&pxFFStatBuffer);

		if ( bWrite != sizeof(Aframe) ) {
			// Error de escritura ??
			snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("WR ERROR: (%d)\r\n\0"),pxFFStatBuffer.errno);
		} else {
			// Stats de memoria
			snprintf_P( aIn_printfBuff, sizeof(aIn_printfBuff), PSTR("MEM [%d/%d/%d][%d/%d]\r\n\0"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		}
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	// Imprimo el frame.
	pos = snprintf_P( aIn_printfBuff, sizeof(aIn_printfBuff), PSTR("frame::{" ));
	// timeStamp.
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%02d%02d%02d,"),Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );
	// Valores analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%s=%.02f,"),systemVars.aChName[channel],Aframe.analogIn[channel] );
	}
	// Valores digitales
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%sP=%.02f,%sL=%d,"), systemVars.dChName[0],Aframe.dIn.pulses[0],systemVars.dChName[0],Aframe.dIn.level[0],systemVars.dChName[0]);
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%sP=%.02f,%sL=%d"), systemVars.dChName[1],Aframe.dIn.pulses[1],systemVars.dChName[1],Aframe.dIn.level[1],systemVars.dChName[1]);
	// Bateria
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR(",bt=%.02f}\r\n\0"),Aframe.batt );
	FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );

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
		retVal = AN_counters.secs2poll;
	}

	return (retVal);
}
/*------------------------------------------------------------------------------------*/
void u_readAnalogFrame (frameData_t *dFrame)
{

	memcpy(dFrame, &Aframe, sizeof(Aframe) );
}
/*------------------------------------------------------------------------------------*/
