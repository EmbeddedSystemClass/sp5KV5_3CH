/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include "sp5KV5_3CH.h"

static char cmd_printfBuff[CHAR128];
char *argv[16];

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);
static u08 pv_makeArgv(void);

void pv_cmdRdRTC(void);
void pv_cmdRdEE(void);
void pv_cmdRdADC(void);
void pv_cmdRdMCP(void);
void pv_cmdRdDIN(void);
void pv_cmdRdTERMSW(void);
s08 pv_cmdWrDebugLevel(char *s);
s08 pv_cmdWrLog(char *s);
s08 pv_cmdWrkMode(char *s0, char *s1);
s08 pv_cmdWrEE(char *s0, char *s1);
static void pv_readMemory(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdClearScreen(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdRedialFunction(void);
static void cmdStatusFunction(void);
static void cmdReadFunction(void);
static void cmdWriteFunction(void);
/*------------------------------------------------------------------------------------*/
void tkCmd(void * pvParameters)
{

u08 c;
u08 ticks;
( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	cmdlineInit();
	cmdlineSetOutputFunc(pvFreeRTOS_UART1_writeChar);

	cmdlineAddCommand((u08 *)("cls"), cmdClearScreen );
	cmdlineAddCommand((u08 *)("help"), cmdHelpFunction);
	cmdlineAddCommand((u08 *)("reset"), cmdResetFunction);
	cmdlineAddCommand((u08 *)("read"), cmdReadFunction);
	cmdlineAddCommand((u08 *)("write"), cmdWriteFunction);
	cmdlineAddCommand((u08 *)("redial"), cmdRedialFunction);
	cmdlineAddCommand((u08 *)("status"), cmdStatusFunction);


	// Espero la notificacion para arrancar
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("starting tkCmd..\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	ticks = 1;
	FreeRTOS_ioctl( &pdUART1,ioctlSET_TIMEOUT, &ticks );

	// loop
	for( ;; )
	{
		u_clearWdg(WDG_CMD);

		if ( u_terminalPwrStatus() ) {
			// Solo si la terminal esta prendida leo datos

			c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
			// el read se bloquea 50ms. lo que genera la espera.
			while ( FreeRTOS_read( &pdUART1, &c, 1 ) == 1 ) {
				cmdlineInputFunc(c);
			}

			// Mientras escribo comandos no apago la terminal
			if (c=='\r') {
				u_restartTimerTerminal();
			}

			/* run the cmdline execution functions */
			cmdlineMainLoop();

		} else {
			// Genero una espera para poder entrar en sleep mode
			vTaskDelay( ( TickType_t)( 250 / portTICK_RATE_MS ) );
		}

	}
}
/*------------------------------------------------------------------------------------*/
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\x1B[2J\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void cmdHelpFunction(void)
{

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %dch %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, NRO_ANALOG_CHANNELS, SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Available commands are:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-cls\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-reset {default,memory}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-redial\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-status\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write rtc YYMMDDhhmm\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR( "  wrkmode [service | monitor {sqe|frame}], pwrmode [continuo|discreto] \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerpoll, timerdial, dlgid, gsmband, maxrange, tilt {on|off}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pwrsave [modo {on|off}, {hhmm1}, {hhmm2}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debuglevel +/-{none,basic,mem,data,gprs,digital,all} \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  log {on,off} \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#ifdef OSE_3CH
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  A{0..2} aname imin imax mmin mmax\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif
#ifdef OSE_POZOS
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  C0 aname\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  D{0..1} dname magp\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn, roaming {on|off}, port, ip, script, passwd\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

#ifdef CONSIGNA
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  consigna {off|doble|continua}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  dcons hhmm1,hhmm2,chVA,chVB\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(" ccons { maxpb, pref, pbtest, maxPW }\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(" fuzzy\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  save\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) mcp devId regAddr regValue\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) ee addr string\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) led {0|1},gprspwr {0|1},gprssw {0|1},termpwr {0|1},sensorpwr {0|1},analogpwr {0|1}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

#ifdef CONSIGNA
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) clearQ0 clearQ1\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) output {reset|noreset,sleep|nosleep},{enable|disable}{A1|A2|B1|B2}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM)        phase {A1,A2,B1,B2} {01|10}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM)        pulse {0..3} {01|10} {ms}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM)        vopen,vclose {0..3}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM)        consigna{diurna|nocturna}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif

#ifdef OSE_POZOS
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) rangectl {run|stop}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) atcmd {cmd}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  mcp {0|1} regAddr\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  dcd,din {0|1}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc, adc {ch}, frame\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee addr lenght\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) frame,memory,gprs\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  defaults \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

}
/*------------------------------------------------------------------------------------*/
static void cmdResetFunction(void)
{
	pv_makeArgv();

	// Reset memory ??
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {
		FF_rewind();
	}

	cmdClearScreen();
	// RESET
	u_reset();

}
/*------------------------------------------------------------------------------------*/
static void cmdRedialFunction(void)
{
	// Envio un mensaje a la tk_Gprs para que recargue la configuracion y disque al server
	// Notifico en modo persistente. Si no puedo me voy a resetear por watchdog. !!!!
	while ( xTaskNotify(xHandle_tkGprsTx,TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

}
/*------------------------------------------------------------------------------------*/
static void cmdStatusFunction(void)
{

RtcTimeType_t rtcDateTime;
u16 pos;
u08 channel;
frameData_t Cframe;
StatBuffer_t pxFFStatBuffer;

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %dch %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, NRO_ANALOG_CHANNELS, SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Last reset info
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Wdg (0x%X"),wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" PORF"));
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" EXTRF"));
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" BORF"));
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" WDRF"));
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" JTRF"));
	}
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" )\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DlgId */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("dlgid: %s\r\n\0"), systemVars.dlgId );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Fecha y Hora */
	RTC_read(&rtcDateTime);
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("rtc: %02d/%02d/%04d "),rtcDateTime.day,rtcDateTime.month, rtcDateTime.year );
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("%02d:%02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Server:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* APN */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn: %s\r\n\0"), systemVars.apn );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER IP:SERVER PORT */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server ip:port: %s:%s\r\n\0"), systemVars.serverAddress,systemVars.serverPort );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER SCRIPT */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server script: %s\r\n\0"), systemVars.serverScript );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER PASSWD */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  passwd: %s\r\n\0"), systemVars.passwd );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// MODEM ---------------------------------------------------------------------------------------
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Modem:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Modem band */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  band: "));
	switch ( systemVars.gsmBand) {
	case 0:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(900)"));
		break;
	case 1:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(1800)"));
		break;
	case 2:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("dual band (900/1800)"));
		break;
	case 3:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("pcs (1900)"));
		break;
	case 4:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("gsm (850)"));
		break;
	case 5:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("dual band (1900/850)"));
		break;
	case 6:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("triband (900/1800/1900)"));
		break;
	case 7:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("triband (850/1800/1900)"));
		break;
	case 8:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("cuatriband (850/900/1800/1900)"));
		break;
	}
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* ROAMING */
	if ( systemVars.roaming == TRUE ) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  roaming ON\r\n\0"));
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  roaming OFF\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DLGIP */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  dlg ip: %s\r\n\0"), systemVars.dlgIp );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CSQ */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  signalQ: csq=%d, dBm=%d\r\n\0"), systemVars.csq, systemVars.dbm );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// RI/TERMFLAG
	if ( systemVars.ri == 0 ) { pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ri=ON,\0")); }
 	if ( systemVars.ri == 1 ) { pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ri=OFF,\0"));}
 	if ( systemVars.termsw == 1 ) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("term=ON\r\n\0")); }
 	if ( systemVars.termsw == 0 ) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("term=OFF\r\n\0"));}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SYSTEM ---------------------------------------------------------------------------------------
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">System:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Memoria */
	FF_stat(&pxFFStatBuffer);
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  memory: wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d \r\n"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* WRK mode (NORMAL / SERVICE) */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  wrkmode: "));
	/* WRK mode (NORMAL / SERVICE) */
	switch (systemVars.wrkMode) {
	case WK_NORMAL:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("normal\r\n"));
		break;
	case WK_SERVICE:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("service\r\n"));
		break;
	case WK_MONITOR_FRAME:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("monitor_frame\r\n"));
		break;
	case WK_MONITOR_SQE:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("monitor_sqe\r\n"));
		break;
	default:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ERROR\r\n"));
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* PWR mode (CONTINUO / DISCRETO) */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pwrmode: "));
	switch (systemVars.pwrMode) {
	case PWR_CONTINUO:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("continuo\r\n"));
		break;
	case PWR_DISCRETO:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("discreto\r\n"));
		break;
	default:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ERROR\r\n"));
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Pwr.Save */
	if ( systemVars.pwrSave == modoPWRSAVE_ON ) {
		pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pwrSave ON: "));
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("%04d->"),u_convertMINS2hhmm( systemVars.pwrSaveStartTime));
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("%04d\r\n"),u_convertMINS2hhmm( systemVars.pwrSaveEndTime));
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pwrSave OFF\r\n"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Timers */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerPoll [%ds]: %d\r\n\0"),systemVars.timerPoll, u_readTimeToNextPoll() );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

#ifdef OSE_3CH
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerDial: [%lus]: %li\r\n\0"), systemVars.timerDial, u_readTimeToNextDial() );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif

#ifdef OSE_POZOS
	/* MaxRange */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  maxRange: %d\r\n\0"),systemVars.maxRange );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif

	/* DebugLevel */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debugLevel: "));
	if ( systemVars.debugLevel == D_NONE) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("none") );
	} else {
		if ( (systemVars.debugLevel & D_BASIC) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+basic")); }
		if ( (systemVars.debugLevel & D_DATA) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+data")); }
		if ( (systemVars.debugLevel & D_GPRS) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+gprs")); }
		if ( (systemVars.debugLevel & D_MEM) != 0)   { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+mem")); }
		if ( (systemVars.debugLevel & D_CONSIGNA) != 0)   { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+cons")); }
		if ( (systemVars.debugLevel & D_DIGITAL) != 0)  { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+digital")); }
		if ( (systemVars.debugLevel & D_DEBUG) != 0)  { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+debug")); }
	}
	snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Log
	if ( systemVars.log == ON ) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  log: ON\r\n\0"));
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  log: OFF\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

#ifdef OSE_3CH
	/* TILT */
	if ( systemVars.tiltEnabled == TRUE ) {
		pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Tilt: enable (%d)"), systemVars.tilt );
	} else {
		pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Tilt: disabled (%d)"), systemVars.tilt );
	}
	if ( u_checkAlarmFloding() ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(**)"));
	}
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif

	/* CONFIG */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Config:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Bateria
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  batt{0-15V}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

#ifdef OSE_3CH
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  a%d{%d-%dmA/%d-%.02f}(%s)\r\n\0"),channel, systemVars.Imin[channel],systemVars.Imax[channel],systemVars.Mmin[channel],systemVars.Mmax[channel], systemVars. aChName[channel] );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
	/* Configuracion de canales digitales */
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  d%d{%.02f p/p} (%s)\r\n\0"),channel, systemVars.magPP[channel],systemVars.dChName[channel]);
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
#endif

#ifdef OSE_POZOS
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  c0(%s)\r\n\0"), systemVars. aChName[0] );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	/* Configuracion de canales digitales */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  c1(%s)\r\n\0"), systemVars.dChName[0]);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  c2(%s)\r\n\0"),systemVars.dChName[1]);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif

	/* Consignas */
#ifdef CONSIGNA
	switch ( systemVars.consigna.type ) {
	case CONSIGNA_OFF:
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  consignas: OFF\r\n\0"));
		break;
	case CONSIGNA_DOBLE:
		pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  consignas:"));
		if ( systemVars.consigna.consignaAplicada == CONSIGNA_DIURNA ) {
			pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(diurna) "));
		} else {
			pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(nocturna) "));
		}
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("C.Dia:[%04d],\0"), u_convertMINS2hhmm( systemVars.consigna.horaConsDia ) );
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("C.Noc:[%04d],\0"), u_convertMINS2hhmm( systemVars.consigna.horaConsNoc ));
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("[chVA=%d],[chVB=%d]\r\n\0"), systemVars.consigna.chVA,systemVars.consigna.chVB);
		break;
	case CONSIGNA_CONTINUA:
		pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  consignas: (continua): "));
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" maxPW=%d"), systemVars.cc_maxPW );
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" maxPB=%.02f"), systemVars.cc_maxPb );
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" pRef=%.02f"), systemVars.cc_pRef );
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" pTest=%.02f\r\n"), systemVars.cc_pBTest );
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif

	/* VALUES --------------------------------------------------------------------------------------- */
	memset(&Cframe,'\0', sizeof(frameData_t));

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Values:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

#ifdef UTE_8CH
	u_readDataFrame (&Cframe);
	pos = snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff), PSTR("  "));
	// TimeStamp.
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff),PSTR( "%04d%02d%02d,"),Cframe.rtc.year,Cframe.rtc.month,Cframe.rtc.day );
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%02d%02d%02d,"),Cframe.rtc.hour,Cframe.rtc.min, Cframe.rtc.sec );

	// Valores analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%s=%.02f,"),systemVars.aChName[channel],Cframe.analogIn[channel] );
	}

	// Valores digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%sP=%.02f,%sL=%d,"), systemVars.dChName[0],Cframe.dIn.pulses[0],systemVars.dChName[0],Cframe.dIn.level[0]);	}
#endif

#ifdef OSE_3CH
	u_readDataFrame (&Cframe);
	pos = snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff), PSTR("  "));
	// TimeStamp.
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff),PSTR( "%04d%02d%02d,"),Cframe.rtc.year,Cframe.rtc.month,Cframe.rtc.day );
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%02d%02d%02d,"),Cframe.rtc.hour,Cframe.rtc.min, Cframe.rtc.sec );

	// Valores analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%s=%.02f,"),systemVars.aChName[channel],Cframe.analogIn[channel] );
	}

	// Valores digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%sP=%.02f,"), systemVars.dChName[channel],Cframe.dIn.pulses[channel]);
	}
	// Bateria
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("bt=%.02f\r\n\0"),Cframe.batt );
#endif

#ifdef OSE_POZOS
	u_readDataFrame (&Cframe);
	pos = snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff), PSTR("  "));
	// TimeStamp.
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff),PSTR( "%04d%02d%02d,"),Cframe.rtc.year,Cframe.rtc.month,Cframe.rtc.day );
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%02d%02d%02d,"),Cframe.rtc.hour,Cframe.rtc.min, Cframe.rtc.sec );

	// Range
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%s=%.0f,"),systemVars.aChName[0],Cframe.analogIn[0] );

	// Valores digitales
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%s=%d,"), systemVars.dChName[0],Cframe.dIn.level[0]);
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%s=%d\r\n\0"), systemVars.dChName[1],Cframe.dIn.level[1]);

#endif

	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void cmdReadFunction(void)
{
char *p;
s08 retS = FALSE;
u08 argc;

	argc = pv_makeArgv();

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		pv_cmdRdEE();
		return;
	}

	// ADC
	// read adc channel
	if (!strcmp_P( strupr(argv[1]), PSTR("ADC\0"))) {
		pv_cmdRdADC();
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		pv_cmdRdRTC();
		return;
	}

	// MCP
	// read mcp 0|1|2 addr
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0"))) {
		pv_cmdRdMCP();
		return;
	}

	// DIN
	// read din 0|1
	if (!strcmp_P( strupr(argv[1]), PSTR("DIN\0"))) {
		pv_cmdRdDIN();
		return;
	}

 	// TERMSW
 	if (!strcmp_P( strupr(argv[1]), PSTR("TERMSW\0"))) {
 		pv_cmdRdTERMSW();
 		return;
 	}

	// DEFAULT
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULTS\0"))) {
		u_loadDefaults();
		return;
	}

	// FRAME
	if (!strcmp_P( strupr(argv[1]), PSTR("FRAME\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {

#if defined(OSE_3CH) || defined(UTE_8CH)
		while ( xTaskNotify(xHandle_tkAIn, TK_READ_FRAME , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
#endif

#ifdef OSE_POZOS
		while ( xTaskNotify(xHandle_tkRange, TK_READ_FRAME , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
#endif
		return;
	}

	// MEMORY
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		pv_readMemory();
		return;
	}

	// GPRS RSP.
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0"))) {
		p = FreeRTOS_UART_getFifoPtr(&pdUART0);
		FreeRTOS_write( &pdUART1, "rx->", sizeof("rx->")  );
		FreeRTOS_write( &pdUART1, p, UART0_RXBUFFER_LEN );
		FreeRTOS_write( &pdUART1, "\r\n\0", sizeof("\r\n\0")  );
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
static void cmdWriteFunction(void)
{
s08 retS = FALSE;
u08 argc;

	argc = pv_makeArgv();

	// SAVE
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE\0"))) {
		retS = u_saveSystemParams();
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PASSWD
	if (!strcmp_P( strupr(argv[1]), PSTR("PASSWD\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.passwd, '\0', sizeof(systemVars.passwd));
			memcpy(systemVars.passwd, argv[2], sizeof(systemVars.passwd));
			systemVars.passwd[PASSWD_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// DLGID
	if (!strcmp_P( strupr(argv[1]), PSTR("DLGID\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memcpy(systemVars.dlgId, argv[2], sizeof(systemVars.dlgId));
			systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// APN
	if (!strcmp_P( strupr(argv[1]), PSTR("APN\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.apn, '\0', sizeof(systemVars.apn));
			memcpy(systemVars.apn, argv[2], sizeof(systemVars.apn));
			systemVars.apn[APN_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// ROAMING
	if (!strcmp_P( strupr(argv[1]), PSTR("ROAMING\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON"))) { systemVars.roaming = TRUE; }
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { systemVars.roaming = FALSE; }
		pv_snprintfP_OK();
		return;
	}

	// SERVER PORT
	if (!strcmp_P( strupr(argv[1]), PSTR("PORT\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.serverPort, '\0', sizeof(systemVars.serverPort));
			memcpy(systemVars.serverPort, argv[2], sizeof(systemVars.serverPort));
			systemVars.serverPort[PORT_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER IP
	if (!strcmp_P( strupr(argv[1]), PSTR("IP\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.serverAddress, '\0', sizeof(systemVars.serverAddress));
			memcpy(systemVars.serverAddress, argv[2], sizeof(systemVars.serverAddress));
			systemVars.serverAddress[IP_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER SCRIPT
	if (!strcmp_P( strupr(argv[1]), PSTR("SCRIPT\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.serverScript, '\0', sizeof(systemVars.serverScript));
			memcpy(systemVars.serverScript, argv[2], sizeof(systemVars.serverScript));
			systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* DEBUGLEVEL */
	if (!strcmp_P( strupr(argv[1]), PSTR("DEBUGLEVEL\0"))) {
		retS = pv_cmdWrDebugLevel(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* LOG */
	if (!strcmp_P( strupr(argv[1]), PSTR("LOG\0"))) {
		retS = pv_cmdWrLog(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* WRKMODE */
	if (!strcmp_P( strupr(argv[1]), PSTR("WRKMODE\0"))) {
		retS = pv_cmdWrkMode(argv[2],argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CANALES ANALOGICOS

#ifdef OSE_POZOS
	if (!strcmp_P( strupr(argv[1]), PSTR("C0\0"))) {
		retS = u_configAnalogCh( 0, argv[2],NULL,NULL,NULL,NULL);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}
#endif

#ifdef OSE_3CH

	if (!strcmp_P( strupr(argv[1]), PSTR("A0\0"))) {
		retS = u_configAnalogCh( 0, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[1]), PSTR("A1\0"))) {
		retS = u_configAnalogCh( 1, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[1]), PSTR("A2\0"))) {
		retS = u_configAnalogCh( 2, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}
#endif

#ifdef UTE_8CH
	if (!strcmp_P( strupr(argv[1]), PSTR("A1\0"))) {
		retS = u_configAnalogCh( 1, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[1]), PSTR("A2\0"))) {
		retS = u_configAnalogCh( 2, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[1]), PSTR("A3\0"))) {
		retS = u_configAnalogCh( 3, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[3]), PSTR("A4\0"))) {
		retS = u_configAnalogCh( 4, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[3]), PSTR("A5\0"))) {
		retS = u_configAnalogCh( 5, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[3]), PSTR("A6\0"))) {
		retS = u_configAnalogCh( 6, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[3]), PSTR("A7\0"))) {
		retS = u_configAnalogCh( 7, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}
#endif

	// CANALES DIGITALES
	if (!strcmp_P( strupr(argv[1]), PSTR("D0\0"))) {
		u_configDigitalCh( 0, argv[2],argv[3]);
		pv_snprintfP_OK();
		return;
	}

	if (!strcmp_P( strupr(argv[1]), PSTR("D1\0"))) {
		u_configDigitalCh( 1, argv[2],argv[3]);
		pv_snprintfP_OK();
		return;
	}

	// TIMERPOLL
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL\0"))) {
		retS = u_configTimerPoll(argv[2]);

		// tk_aIn: notifico en modo persistente. Si no puedo, me voy a resetear por watchdog. !!!!
		while ( xTaskNotify(xHandle_tkAIn, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}

		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

#ifdef OSE_POZOS
	// MAXRANGE
	if (!strcmp_P( strupr(argv[1]), PSTR("MAXRANGE\0"))) {
		retS = u_configMaxRange(argv[2]);

		// tk_range: notifico en modo persistente. Si no puedo, me voy a resetear por watchdog. !!!!
		while ( xTaskNotify(xHandle_tkRange, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}

		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}
#endif

#ifdef OSE_3CH

	// TILT
	if (!strcmp_P( strupr(argv[1]), PSTR("TILT\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON"))) { systemVars.tiltEnabled = TRUE; }
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { systemVars.tiltEnabled = FALSE; }
		pv_snprintfP_OK();
		return;
	}

	// TIMERDIAL
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERDIAL\0"))) {
		retS = u_configTimerDial(argv[2]);

		// tk_Gprs:
		while ( xTaskNotify(xHandle_tkGprsTx, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}

		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PWRSAVE
	if (!strcmp_P( strupr(argv[1]), PSTR("PWRSAVE\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON"))) { u_configPwrSave ( modoPWRSAVE_ON, argv[3], argv[4] ); }
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { u_configPwrSave ( modoPWRSAVE_OFF, argv[3], argv[4] ); }
		pv_snprintfP_OK();
		return;
	}

	// PWRMODE
	if (!strcmp_P( strupr(argv[1]), PSTR("PWRMODE\0"))) {

		if ((!strcmp_P(strupr(argv[2]), PSTR("CONTINUO")))) {
			retS = u_configPwrMode(PWR_CONTINUO);
		}

		if ((!strcmp_P(strupr(argv[2]), PSTR("DISCRETO")))) {
			retS = u_configPwrMode(PWR_DISCRETO);
		}

		// tk_Gprs:
		while ( xTaskNotify(xHandle_tkGprsTx, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}

		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}
#endif

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		retS = u_wrRtc(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

#ifdef CONSIGNA
	// CONSIGNA
	// consigna {off|doble|continua}
	if (!strcmp_P( strupr(argv[1]), PSTR("CONSIGNA\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { u_configConsignasModo( CONSIGNA_OFF); }
		if (!strcmp_P( strupr(argv[2]), PSTR("DOBLE"))) { u_configConsignasModo( CONSIGNA_DOBLE); }
		if (!strcmp_P( strupr(argv[2]), PSTR("CONTINUA"))) {
			if ( systemVars.pwrMode != PWR_CONTINUO ) {
				snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("WARN: Debe pasar a pwrModo continuo antes !!!\r\n\0"));
			 	FreeRTOS_write( &pdUART1, cmd_printfBuff,sizeof(cmd_printfBuff) );
			} else {
				u_configConsignasModo( CONSIGNA_CONTINUA); }
		}
		pv_snprintfP_OK();
		return;
	}
	// Parametros de la doble consigna
	// dcons hhmm1,hhmm2,chVA,chVB
	if (!strcmp_P( strupr(argv[1]), PSTR("DCONS\0"))) {
		u_configDobleConsignas( argv[2],argv[3], argv[4],argv[5]);
		pv_snprintfP_OK();
		return;
	}
	// Parametros de prueba en consigna continua
	// ccons { maxpb, pref, pbtest
	if (!strcmp_P( strupr(argv[1]), PSTR("CCONS\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("MAXPB\0"))) {
			systemVars.cc_maxPb = atof(argv[3]);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("PREF\0"))) {
			systemVars.cc_pRef = atof(argv[3]);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("PBTEST\0"))) {
			systemVars.cc_pBTest = atof(argv[3]);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("MAXPW\0"))) {
			systemVars.cc_maxPW = atoi(argv[3]);
		}

		pv_snprintfP_OK();
		return;
	}

	// Testeo del sistema fuzzy
	if (!strcmp_P( strupr(argv[1]), PSTR("FUZZY\0"))) {
		fuzzy_test();
		pv_snprintfP_OK();
		return;
	}
#endif

	//----------------------------------------------------------------------
	// COMANDOS USADOS PARA DIAGNOSTICO
	// DEBEMOS ESTAR EN MODO SERVICE
	//----------------------------------------------------------------------

	// GSMBAND:
	// Debo estar en modo service ya que para que tome el valor debe resetearse
	if (!strcmp_P( strupr(argv[1]), PSTR("GSMBAND\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			systemVars.gsmBand = atoi(argv[2]);
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// EE: write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = pv_cmdWrEE( argv[2], argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// LED
	// write led 0|1
	if (!strcmp_P( strupr(argv[1]), PSTR("LED\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setLed_LogicBoard( (u08)atoi(argv[2]) );
		if ( atoi(argv[2]) == 1 ) {
			cbi(LED_KA_PORT, LED_KA_BIT);
		} else {
			sbi(LED_KA_PORT, LED_KA_BIT);
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSPWR\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setGprsPwr( (u08) atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsSW
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSSW\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setGprsSw( (u08) atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// termPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("TERMPWR\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setTermPwr( (u08)atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// sensorPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("SENSORPWR\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setSensorPwr( (u08)atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// analogPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOGPWR\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setAnalogPwr( (u08) atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// analogPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("CLEARQ\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		if ( atoi(argv[2]) == 0 ) {
			cbi(Q_PORT, Q0_CTL_PIN);
			vTaskDelay( ( TickType_t)( 1) );
			sbi(Q_PORT, Q0_CTL_PIN);
		}
		if ( atoi(argv[2]) == 1 ) {
			cbi(Q_PORT, Q1_CTL_PIN);
			vTaskDelay( ( TickType_t)( 1) );
			sbi(Q_PORT, Q1_CTL_PIN);
		}
		return;
	}

	// MCP
	// write mcp 0|1|2 addr value
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		switch( atoi(argv[2] )) {
		case 0:
			retS = MCP_write( MCP0_ADDR, atoi(argv[3]), atoi(argv[4]) );
			break;
		case 1:
			retS = MCP_write( MCP1_ADDR, atoi(argv[3]), atoi(argv[4]) );
			break;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

#ifdef CONSIGNA

	 // OUTPUTS
	// {reset|noreset|sleep|nosleep},{enable|disable}{A1,A2,B1,B2}
	if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUT\0"))) {

		// reset
		if (!strcmp_P( strupr(argv[2]), PSTR("RESET\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			retS = MCP_outputsReset();
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// noreset
		if (!strcmp_P( strupr(argv[2]), PSTR("NORESET\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			retS = MCP_outputsNoReset();
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// sleep
		if (!strcmp_P( strupr(argv[2]), PSTR("SLEEP\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			retS = MCP_outputsSleep();
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// nosleep
		if (!strcmp_P( strupr(argv[2]), PSTR("NOSLEEP\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			retS = MCP_outputsNoSleep();
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// enable 0,1,2,3
		if (!strcmp_P( strupr(argv[2]), PSTR("ENABLE\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			if ( !strcmp_P( strupr(argv[3]), PSTR("A1"))) {
				retS = MCP_outputA1Enable();
			} else if ( !strcmp_P( strupr(argv[3]), PSTR("A2"))) {
				retS = MCP_outputA2Enable();
			} else if ( !strcmp_P( strupr(argv[3]), PSTR("B1"))) {
				retS = MCP_outputB1Enable();
			} else if ( !strcmp_P( strupr(argv[3]), PSTR("B2"))) {
				retS = MCP_outputB2Enable();
			} else {
				retS = FALSE;
			}
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// disable 0,1,2,3
		if (!strcmp_P( strupr(argv[2]), PSTR("DISABLE\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			if ( !strcmp_P( strupr(argv[3]), PSTR("A1"))) {
				retS = MCP_outputA1Disable();
			} else if ( !strcmp_P( strupr(argv[3]), PSTR("A2"))) {
				retS = MCP_outputA2Disable();
			} else if ( !strcmp_P( strupr(argv[3]), PSTR("B1"))) {
				retS = MCP_outputB1Disable();
			} else if ( !strcmp_P( strupr(argv[3]), PSTR("B2"))) {
				retS = MCP_outputB2Disable();
			} else {
				retS = FALSE;
			}
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// phase {A1,A2,B1,B2} 01|10
		if (!strcmp_P( strupr(argv[2]), PSTR("PHASE\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			if ( !strcmp_P( strupr(argv[3]), PSTR("A1"))) {
				if (!strcmp_P( strupr(argv[4]), PSTR("01\0"))) { retS = MCP_outputA1Phase_01(); }
				if (!strcmp_P( strupr(argv[4]), PSTR("10\0"))) { retS = MCP_outputA1Phase_10(); }

			} else if ( !strcmp_P( strupr(argv[3]), PSTR("A2"))) {
				if (!strcmp_P( strupr(argv[4]), PSTR("01\0"))) { retS = MCP_outputA2Phase_01(); }
				if (!strcmp_P( strupr(argv[4]), PSTR("10\0"))) { retS = MCP_outputA2Phase_10(); }

			} else if ( !strcmp_P( strupr(argv[3]), PSTR("B1"))) {
				if (!strcmp_P( strupr(argv[4]), PSTR("01\0"))) { retS = MCP_outputB1Phase_01(); }
				if (!strcmp_P( strupr(argv[4]), PSTR("10\0"))) { retS = MCP_outputB1Phase_10(); }

			} else if ( !strcmp_P( strupr(argv[3]), PSTR("B2"))) {
				if (!strcmp_P( strupr(argv[4]), PSTR("01\0"))) { retS = MCP_outputB2Phase_01(); }
				if (!strcmp_P( strupr(argv[4]), PSTR("10\0"))) { retS = MCP_outputB2Phase_10(); }
			} else {
				retS = FALSE;
			}
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// pulse {0,1,2,3} 01|10 ms
		if (!strcmp_P( strupr(argv[2]), PSTR("PULSE\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			if (!strcmp_P( strupr(argv[4]), PSTR("01\0"))) { retS = MCP_outsPulse((u08) (atoi(argv[3])), 0, (u16) (atoi(argv[5])) ); }
			if (!strcmp_P( strupr(argv[4]), PSTR("10\0"))) { retS = MCP_outsPulse((u08) (atoi(argv[3])), 1, (u16) (atoi(argv[5])) ); }
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// vopen 0,1,2,3
		if (!strcmp_P( strupr(argv[2]), PSTR("VOPEN\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			u_vopen((u08) (atoi(argv[3])));
			pv_snprintfP_OK();
			return;
		}

		// vclose 0,1,2,3
		if (!strcmp_P( strupr(argv[2]), PSTR("VCLOSE\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			u_close((u08) (atoi(argv[3])));
			pv_snprintfP_OK();
			return;
		}

		// consigna { diurna | nocturna }
		if (!strcmp_P( strupr(argv[2]), PSTR("CONSIGNA\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("DIURNA\0"))) {
				u_setConsignaDiurna();
				snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Consigna DIURNA..\r\n\0"));
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("NOCTURNA\0"))) {
				u_setConsignaNocturna();
				snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Consigna NOCTURNA..\r\n\0"));
			}
			FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
			pv_snprintfP_OK();
			return;
		}
	}
#endif

#ifdef OSE_POZOS
	// RANGE control
	if (!strcmp_P( strupr(argv[1]), PSTR("RANGECTL\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		if (!strcmp_P( strupr(argv[2]), PSTR("RUN"))) {
			u_rangeSignal(RUN);
			retS = TRUE;
		}
		if (!strcmp_P( strupr(argv[2]), PSTR("STOP"))) {
			u_rangeSignal(STOP);
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}
#endif

	// ATCMD
	// Envia un comando al modem.
	if (!strcmp_P( strupr(argv[1]), PSTR("ATCMD\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		snprintf( cmd_printfBuff,sizeof(cmd_printfBuff),"%s\r",argv[2] );
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
		FreeRTOS_write( &pdUART0, cmd_printfBuff, sizeof(cmd_printfBuff) );

		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("sent->%s\r\n\0"),argv[2] );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
// FUNCIONES PRIVADAS
//-------------------------------------------------------------------------------------
s08 pv_cmdWrEE(char *s0, char *s1)
{
u08 length = 0;
char *p;
s08 retS = FALSE;

	p = s1;
	while (*p != 0) {
		p++;
		length++;
	}
//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("S=[%s](%d)\r\n\0"),s1, length);
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	retS = EE_write( (u16)(atoi(s0)), s1, length );
	return(retS);
}
/*------------------------------------------------------------------------------------*/
s08 pv_cmdWrkMode(char *s0, char *s1)
{
s08 retS = FALSE;

	if ((!strcmp_P(strupr(s0), PSTR("SERVICE")))) {
		systemVars.wrkMode = WK_SERVICE;
		retS = TRUE;
		goto quit;
	}

	if ((!strcmp_P(strupr(s0), PSTR("MONITOR")))) {

		if ((!strcmp_P( strupr(s1), PSTR("SQE")))) {
			systemVars.wrkMode = WK_MONITOR_SQE;
			retS = TRUE;
			goto quit;
		}

		if ((!strcmp_P( strupr(s1), PSTR("FRAME")))) {
			systemVars.wrkMode = WK_MONITOR_FRAME;
			retS = TRUE;
			goto quit;
		}
	}

quit:

	if ( retS ) {
#ifdef OSE_3CH
		// tk_aIn: Notifico en modo persistente. Si no puedo me voy a resetear por watchdog. !!!!
		while ( xTaskNotify(xHandle_tkAIn, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
#endif

#ifdef OSE_POZOS
		// tk_range: Notifico en modo persistente. Si no puedo me voy a resetear por watchdog. !!!!
		while ( xTaskNotify(xHandle_tkRange, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
#endif

		// tk_Gprs:
		while ( xTaskNotify(xHandle_tkGprsTx, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
	}

	return(retS);
}
/*------------------------------------------------------------------------------------*/
s08 pv_cmdWrDebugLevel(char *s)
{

	if ((!strcmp_P( strupr(s), PSTR("NONE")))) {
		systemVars.debugLevel = D_NONE;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("+BASIC")))) {
		systemVars.debugLevel += D_BASIC;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-BASIC")))) {
		if ( ( systemVars.debugLevel & D_BASIC) != 0 ) {
			systemVars.debugLevel -= D_BASIC;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+DATA")))) {
		systemVars.debugLevel += D_DATA;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-DATA")))) {
		if ( ( systemVars.debugLevel & D_DATA) != 0 ) {
			systemVars.debugLevel -= D_DATA;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+MEM")))) {
		systemVars.debugLevel += D_MEM;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-MEM")))) {
		if ( ( systemVars.debugLevel & D_MEM) != 0 ) {
			systemVars.debugLevel -= D_MEM;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+CONSIGNA")))) {
		systemVars.debugLevel += D_CONSIGNA;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-CONSIGNA")))) {
		if ( ( systemVars.debugLevel & D_CONSIGNA) != 0 ) {
			systemVars.debugLevel -= D_CONSIGNA;
			return(TRUE);
		}
	}
	if ((!strcmp_P( strupr(s), PSTR("+GPRS")))) {
		systemVars.debugLevel += D_GPRS;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-GPRS")))) {
		if ( ( systemVars.debugLevel & D_GPRS) != 0 ) {
			systemVars.debugLevel -= D_GPRS;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+DIGITAL")))) {
		systemVars.debugLevel += D_DIGITAL;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-DIGITAL")))) {
		if ( ( systemVars.debugLevel & D_DIGITAL) != 0 ) {
			systemVars.debugLevel -= D_DIGITAL;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+DEBUG")))) {
		systemVars.debugLevel += D_DEBUG;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-DEBUG")))) {
		if ( ( systemVars.debugLevel & D_DEBUG) != 0 ) {
			systemVars.debugLevel -= D_DEBUG;
			return(TRUE);
		}
	}
	if ((!strcmp_P( strupr(s), PSTR("ALL")))) {
		systemVars.debugLevel = D_DATA + D_GPRS + D_MEM + D_DIGITAL + D_CONSIGNA + D_DEBUG;
		return(TRUE);
	}

	return(FALSE);
}
/*------------------------------------------------------------------------------------*/
s08 pv_cmdWrLog(char *s)
{

	if ((!strcmp_P( strupr(s), PSTR("ON")))) {
		systemVars.log = ON;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("OFF")))) {
		systemVars.log = OFF;
		return(TRUE);
	}

	return(FALSE);
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdRTC(void)
{
RtcTimeType_t rtcDateTime;
s08 retS = FALSE;
u08 pos;

	retS = RTC_read(&rtcDateTime);
	if (retS ) {
		pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n"));
		pos += snprintf_P( &cmd_printfBuff[pos],(sizeof(cmd_printfBuff) - pos ),PSTR("%02d/%02d/%04d "),rtcDateTime.day,rtcDateTime.month, rtcDateTime.year );
		pos += snprintf_P( &cmd_printfBuff[pos],(sizeof(cmd_printfBuff) - pos ),PSTR("%02d:%02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdEE(void)
{
	// read ee address length
	// address: argv[2]
	// length: argv[3]

s08 retS = FALSE;

	memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	retS = EE_read( (u16)(atoi(argv[2])), cmd_printfBuff, atoi(argv[3]) );
	if ( retS ) {
		// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
		snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
	retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdADC(void)
{
	// read adc channel
	// channel: argv[2]

s08 retS = FALSE;
u16 adcRetValue = 9999;

	switch(atoi(argv[2])) {
	case 0:
		retS = ADS7827_readCh0( &adcRetValue );
		break;
	case 1:
		retS = ADS7827_readCh1( &adcRetValue );
		break;
	case 2:
		retS = ADS7827_readCh2( &adcRetValue );
		break;
	case 3:
		retS = ADS7827_readBatt( &adcRetValue );
		break;
	}
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ACD[%d]=%d\r\n\0"), atoi(argv[2]), adcRetValue );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
	return;
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdMCP(void)
{
	// read mcp 0|1|2 addr
	// mcpDevide: argv[2]
	// devAddress: argv[3]

s08 retS = FALSE;
u08 regValue;

	switch( atoi(argv[2] )) {
	case 0:
		retS = MCP_read( MCP0_ADDR, atoi(argv[3]), &regValue );
		break;
	case 1:
		retS = MCP_read( MCP1_ADDR, atoi(argv[3]), &regValue );
		break;
	}
	if (retS ) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n[reg 0X%03x]=[0X%03x]\r\n\0"),atoi(argv[3]),regValue);
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdDIN(void)
{
s08 retS = FALSE;
u08 pin;

	switch( atoi(argv[2] )) {
	case 0:
		retS = MCP_queryDin0(&pin);
		break;
	case 1:
		retS = MCP_queryDin1(&pin);
		break;
	}
	if (retS ) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\nDIN%d=%d\r\n\0"),atoi(argv[2]),pin);
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;

}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdTERMSW(void)
{
u08 pin;

	u_readTermsw(&pin);
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\nTERMSW=%d\r\n\0"),pin);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;

}
//------------------------------------------------------------------------------------
static u08 pv_makeArgv(void)
{
// A partir de la linea de comando, genera un array de punteros a c/token
//
char *token = NULL;
char parseDelimiters[] = " ";
int i = 0;

	// inicialmente todos los punteros deben apuntar a NULL.
	memset(argv, 0, sizeof(argv) );

	// Genero los tokens delimitados por ' '.
	token = strtok(SP5K_CmdlineBuffer, parseDelimiters);
	argv[i++] = token;
	while ( (token = strtok(NULL, parseDelimiters)) != NULL ) {
		argv[i++] = token;
		if (i == 16) break;
	}
	return(( i - 1));
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_OK(void )
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_ERR(void)
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_readMemory(void)
{
StatBuffer_t pxFFStatBuffer;
frameData_t Aframe;
size_t bRead;
u08 pos, channel;

	FF_seek();
	while(1) {
		bRead = FF_fread( &Aframe, sizeof(Aframe));
		if ( bRead != sizeof(Aframe))
			break;

		// imprimo
		FF_stat(&pxFFStatBuffer);
		pos = snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff), PSTR("RD:[%d/%d/%d][%d/%d] "), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("frame::{" ));
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%02d%02d%02d,"),Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );

#ifdef OSE_3CH
		for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
			pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%s=%.2f,"),systemVars.aChName[channel],Aframe.analogIn[channel] );
		}
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%sP=%.2f,%sL=%d,"), systemVars.dChName[0],Aframe.dIn.pulses[0],systemVars.dChName[0],Aframe.dIn.level[0]);
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%sP=%.2f,%sL=%d"), systemVars.dChName[1],Aframe.dIn.pulses[1],systemVars.dChName[1],Aframe.dIn.level[1]);
		// Bateria
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR(",bt=%.2f}\r\n\0"),Aframe.batt );

#endif

#ifdef OSE_POZOS
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%s=%d,"),systemVars.aChName[0],Aframe.analogIn[0l] );
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%s=%d,"), systemVars.dChName[0],Aframe.dIn.level[0]);
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%s=%d\r\n\0"), systemVars.dChName[1],Aframe.dIn.level[1]);
#endif
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
}
/*------------------------------------------------------------------------------------*/
