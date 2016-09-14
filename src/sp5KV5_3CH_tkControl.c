/*
 * sp5KV3_tkControl.c
 *
 *  Created on: 7/4/2015
 *      Author: pablo
 *
 *  Tareas de control generales del SP5K
 *  - Recibe un mensaje del timer del led para indicar si debe prender o apagarlo.
 */

#include "sp5KV5_3CH.h"

static char ctl_printfBuff[CHAR128];

void pv_flashLeds(void);
void pv_wdgInit(void);
void pv_checkWdg(void );
void pv_dailyReset(void);
void pv_autoServiceExit(void);
void pv_checkTerminal(void);
void pv_checkTilt(void);

TimerHandle_t controlTimer;
void  pv_controlTimerCallBack( TimerHandle_t pxTimer );
s08 f_controlCallback;

void pv_switchTerminal( t_onOff modo );
static t_terminalStatus f_terminalStatus;
static u16 c_terminalTimeOut;
static s08 f_alarmFired = FALSE;

typedef enum { LED_KA = 0, LED_MODEM } t_led;
void pv_switchLed(t_led led, t_onOff modo);

//------------------------------------------------------------------------------------
void tkControl(void * pvParameters)
{

( void ) pvParameters;
u16 ffRcd;
StatBuffer_t pxFFStatBuffer;
u16 pos;
s08 loadParamStatus = FALSE;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	MCP_init();				// Esto prende la terminal.
	pv_switchTerminal(ON);

	// Load systemVars
	if  ( u_loadSystemParams() == TRUE ) {
		loadParamStatus = TRUE;
	} else {
		u_loadDefaults();
		u_saveSystemParams();
		loadParamStatus = FALSE;
	}

	// Configuro el ID en el bluetooth: debe hacerse antes que nada
	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("AT+NAME%s\r\n"),systemVars.dlgId);
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );

	// Mensaje de load Status.
	if ( loadParamStatus ) {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Load config OK.\r\n\0") );
	} else {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Load config ERROR: defaults !!\r\n\0") );
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	pv_wdgInit();

	// Inicializo la memoria EE ( fileSysyem)
	ffRcd = FF_fopen();
	FF_stat(&pxFFStatBuffer);
	if ( pxFFStatBuffer.errno != pdFF_ERRNO_NONE ) {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("FSInit ERROR (%d)[%d]\r\n\0"),ffRcd, pxFFStatBuffer.errno);
	} else {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("FSInit OK\r\nMEMsize=%d, wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d\r\n\0"),FF_MAX_RCDS, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Arranco el timer de control  de la terminal
	f_controlCallback = FALSE;
	if ( xTimerStart( controlTimer, 0 ) != pdPASS )
		u_panic(P_CTL_TIMERSTART);

	pos = snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Modules:: BASIC\0"));

#ifdef OSE_3CH
	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR("+PRESION\0"));
#endif

#ifdef CONSIGNA
	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR("+CONSIGNA\0"));
#endif

#ifdef OSE_POZOS
	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR("+POZOS\0"));
#endif

#ifdef UTE_8CH
	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR("+8AIN\0"));
#endif

	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR("\r\n"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("starting tkControl..\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Pongo en modo sleep los DRV de las valvulas ya que en esta version no tengo
	// tkOutputs.
	//MCP_outputsReset();
	MCP_outputsSleep();

	// Tilt init.
	systemVars.tilt = ( TILT_PIN & _BV(0) ) >> 0;

	// Habilito arrancar las otras tareas
	startTask = TRUE;

	// Loop
	for( ;; )
	{

		u_clearWdg(WDG_CTL);

		// Espero por el timer 1s para ejecutar las tareas.
		while ( !f_controlCallback )
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

		f_controlCallback = FALSE;

		pv_checkWdg();
		pv_dailyReset();
		pv_flashLeds();
		pv_autoServiceExit();
		pv_checkTerminal();
		pv_checkTilt();
	}
}
//------------------------------------------------------------------------------------
void tkControlInit(void)
{
	// Esta funcion se utiliza  antes de arrancar el FRTOS de modo que cree
	// el timer que necesitamos en este modulo
	// Creo el timer y lo arranco
	controlTimer = xTimerCreate (  "CTL_T",
	                     /* The timer period in ticks, must be greater than 0. */
	                     ( 1000 / portTICK_PERIOD_MS) ,
	                     /* The timers will auto-reload themselves when they expire. */
	                     pdTRUE,
	                     /* Assign each timer a unique id equal to its array index. */
	                     ( void * ) NULL,
	                     /* Each timer calls the same callback when it expires. */
						 pv_controlTimerCallBack
	                   );

	if ( controlTimer == NULL )
		u_panic(P_CTL_TIMERCREATE);

}

//------------------------------------------------------------------------------------
void pv_controlTimerCallBack( TimerHandle_t pxTimer )
{
	// Luego de haber arrancado el FREERTOS, a los 2 mins. expira este timer.
	// y debo ver si apago la terminal o la dejo prendida.
	f_controlCallback = TRUE;
}

//------------------------------------------------------------------------------------
void pv_wdgInit(void)
{
u08 pos;

	pos = snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Watchdog init (0x%X"),wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" PORF"));
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" EXTRF"));
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" BORF"));
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" WDRF"));
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" JTRF"));
	}
	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" )\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
}

//------------------------------------------------------------------------------------
void pv_checkWdg(void )
{
	// Cada tarea periodicamente pone su wdg flag en 0. Esto hace que al chequearse c/3s
	// deban estar todas en 0 para asi resetear el wdg del micro.

static u08 l_secCounter = 3;

	if ( --l_secCounter > 0 )
		return;

	l_secCounter = 3;

	if ( systemWdg == 0 ) {
		wdt_reset();
		systemWdg = WDG_CTL + WDG_CMD + WDG_GPRSTX + WDG_GPRSRX;

#ifdef CONSIGNA
		systemWdg += WDG_CSG;
#endif

#ifdef OSE_3CH
	systemWdg += WDG_AIN + WDG_DIN;
#endif

#ifdef UTE_8CH
	systemWdg += WDG_AIN + WDG_DIN;
#endif

#ifdef OSE_POZOS
	systemWdg += WDG_RANGE;
#endif

	}
}

//------------------------------------------------------------------------------------
void pv_dailyReset(void)
{
	// Se invoca c/1 sec.
	// Se usa para resetear el micro 1 vez al dia.

static u32 l_secCounter = 86400;	// segundos en 1 dia

	if ( --l_secCounter > 0 )
		return;

	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Going to daily reset..\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	// RESET
	u_reset();

}
//------------------------------------------------------------------------------------
void pv_flashLeds(void)
{
	// Ejecuto c/2 secs.
	// Prendo los leds por 20ms y los apago.

static u08 l_secCounter = 3;

	if ( --l_secCounter > 0 )
		return;

	l_secCounter = 3;

	// Prendo: solo si hay una terminal conectada
	if ( u_terminalPwrStatus() == T_PRENDIDA ) {

		pv_switchLed(LED_KA, ON);

		if (u_modemPwrStatus() == ON )
			pv_switchLed(LED_MODEM, ON);

	}

	// no es necesario ya que lo que demora las MCP son suficientes.
	//vTaskDelay( 1 );

	// Apago
	pv_switchLed(LED_KA, OFF);
	pv_switchLed(LED_MODEM, OFF);

}
//------------------------------------------------------------------------------------
void pv_checkTerminal(void)
{
u08 pin;

	// Leo el estado del pin de la terminal
	u_readTermsw(&pin);
	if ( systemVars.termsw != pin ) {
		systemVars.termsw = pin;
	}

	// Si estoy en modo CONTINUO no apago la terminal
	if ( ( systemVars.pwrMode == PWR_CONTINUO) && ( f_terminalStatus == T_APAGADA ) ) {
		// prendo
		pv_switchTerminal(ON);
		return;
	}

	if ( systemVars.pwrMode == PWR_DISCRETO) {

		// Si la terminal esta apagada y se activo el switch la prendo
		if ( ( f_terminalStatus == T_APAGADA ) && ( systemVars.termsw == 1 ) ) {
			// prendo
			pv_switchTerminal(ON);
			snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Terminal going on ..\r\n\0"));
			vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
			FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
			return;
		}

		// Si la terminal estaba prendida pero termSw = 1: No hago nada.
		if ( ( f_terminalStatus == T_PRENDIDA ) && ( systemVars.termsw == 1 ) ) {
			return;
		}

		if ( ( f_terminalStatus == T_PRENDIDA ) && ( systemVars.termsw == 0 ) ) {
			// Cuento 1s.
			if ( c_terminalTimeOut > 0 )
				--c_terminalTimeOut;

			if ( c_terminalTimeOut == 0 ) {
				// Apago
				snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Terminal going off ..\r\n\0"));
				FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
				vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
				pv_switchTerminal(OFF);
				return;
			}
		}
	}
}
//------------------------------------------------------------------------------------
void pv_checkTilt(void)
{
static u08 tilt = 0;

	// El sensor de tilt se utiliza para ver si el equipo se inclino ( porque esta flotando)
	//
	// Valor anterior
	tilt = systemVars.tilt;
	// Valor actual
	systemVars.tilt = ( TILT_PIN & _BV(0) ) >> 0;

	if ( systemVars.tiltEnabled == FALSE )
		return;

	// Deteccion que se movio.
	if ( ( tilt == 0 ) && ( systemVars.tilt == 1) ) {
		// Se movio. Disparo un llamado y quedo alarmado.
		if ( ! f_alarmFired ) {
			f_alarmFired = TRUE;
			snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Flood alarm fired...\r\n\0"));
			FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

			// Mando un mensaje a tkGPRS para que disque inmediatamente
			while ( xTaskNotify(xHandle_tkGprsTx, TKC_FLOODING , eSetBits ) != pdPASS ) {
				vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
			}

		}
	}
}
//------------------------------------------------------------------------------------
void pv_switchTerminal( t_onOff modo )
{
	switch(modo) {
	case ON:
		MCP_setTermPwr(1);
		f_terminalStatus = T_PRENDIDA;
		c_terminalTimeOut = TERMINAL_TIMEOUT;
		break;
	case OFF:
		MCP_setTermPwr(0);
		f_terminalStatus = T_APAGADA;
		c_terminalTimeOut = 0;
		break;
	}
}
//------------------------------------------------------------------------------------
void pv_autoServiceExit(void)
{
	// Se invoca c/1 sec.
	// Se usa para salir del modo service a los 30 mins.

static u16 l_secCounter = 1800;

	// En modo service, cuento.
	if ( systemVars.wrkMode != WK_NORMAL ) {
		if ( --l_secCounter == 0 ) {
			snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Automatic exit of service mode..\r\n\0"));
			FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
			// RESET
			u_reset();
		}
	} else {
		// En WRK_NORMAL reseteo el timer.
		l_secCounter = 1800;
	}
}
//------------------------------------------------------------------------------------
void pv_switchLed(t_led led, t_onOff modo)
{
static s08 led_KA = OFF;
static s08 led_MODEM = OFF;


	switch(led) {
	case LED_KA:
		if ( (modo == ON) && ( led_KA == OFF) ) {
			// Prendo
			led_KA = ON;
			MCP_setLed_LogicBoard(1);		// Led placa logica
			cbi(LED_KA_PORT, LED_KA_BIT);	// Led placa analogica ( kalive )
		} else if ( (modo == OFF) && ( led_KA == ON) ) {
			// Apago
			led_KA = OFF;
			MCP_setLed_LogicBoard(0);		// Led placa logica
			sbi(LED_KA_PORT, LED_KA_BIT);	// Led placa analogica ( kalive )
		}
		break;
	case LED_MODEM:
		if ( (modo == ON) && ( led_MODEM == OFF) ) {
			// Prendo
			led_MODEM = ON;
			cbi(LED_MODEM_PORT, LED_MODEM_BIT);
		} else if ( (modo == OFF) && ( led_MODEM == ON) ) {
			// Apago
			led_MODEM = OFF;
			sbi(LED_MODEM_PORT, LED_MODEM_BIT);
		}
		break;
	}
}
//----------------------------------------------------------------------------------------
void u_restartTimerTerminal(void)
{
	c_terminalTimeOut = TERMINAL_TIMEOUT;
}
//------------------------------------------------------------------------------------
t_terminalStatus u_terminalPwrStatus(void)
{
	return(f_terminalStatus);
}
//------------------------------------------------------------------------------------
s08 u_checkAlarmFloding(void)
{
	return (f_alarmFired);
}
//------------------------------------------------------------------------------------
