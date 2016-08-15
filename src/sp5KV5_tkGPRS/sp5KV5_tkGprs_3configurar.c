/*
 * sp5KV5_3CH_tkGprs_configurar.c
 *
 *  Created on: 17 de may. de 2016
 *      Author: pablo
 *
 * En este estado configuro las variables del modem y lo dejo
 * listo para abrir un socket.
 *
 */


#include "../sp5KV5_3CH.h"
#include "sp5KV5_tkGprs.h"

static int gTR_C00(void);
static int gTR_C01(void);
static int gTR_C02(void);
static int gTR_C03(void);
static int gTR_C04(void);
static int gTR_C05(void);
static int gTR_C06(void);
static int gTR_C07(void);
static int gTR_C08(void);
static int gTR_C09(void);
static int gTR_C10(void);
static int gTR_C11(void);
static int gTR_C12(void);
static int gTR_C13(void);
static int gTR_C14(void);
static int gTR_C15(void);
static int gTR_C16(void);
static int gTR_C17(void);
static int gTR_C18(void);
static int gTR_C19(void);
static int gTR_C20(void);
static int gTR_C21(void);
static int gTR_C22(void);

// Eventos locales al estado OnOFFLINE.
typedef enum {
	c_ev_GSMBAND_OK = 0,
	c_ev_CTIMER_NOT_0,
	c_ev_P_TRYES_NOT_0,
	c_ev_WKMONITOR_SQE,
	c_ev_Q_TRYES_NOT_0,
	c_ev_CREGRSP_OK,
	c_ev_IPASSIGNED,
} t_eventos_ssConfigurar;

#define sm_CONFIGURAR_EVENT_COUNT 7

static u08 cTimer;
static u08 pTryes;
static u08 qTryes;

//------------------------------------------------------------------------------------
void sm_CONFIGURAR(void)
{
	// Maquina de estados del estado ONOFFLINE

s08 c_eventos[sm_CONFIGURAR_EVENT_COUNT];
u08 i;

	// Evaluo solo los eventos del estado OFF.
	// Inicializo la lista local de eventos.
	for ( i=0; i < sm_CONFIGURAR_EVENT_COUNT; i++ ) {
		c_eventos[i] = FALSE;
	}

	// ev_BAND_OK. NET gprsBand is correct.
	if (GPRS_stateVars.flags.gsmBandOK == TRUE  ) { c_eventos[c_ev_GSMBAND_OK] = TRUE; }
	if ( cTimer > 0 ) { c_eventos[c_ev_CTIMER_NOT_0] = TRUE; }
	if ( pTryes > 0 ) { c_eventos[c_ev_P_TRYES_NOT_0] = TRUE; }
	if ( systemVars.wrkMode == WK_MONITOR_SQE ) { c_eventos[c_ev_WKMONITOR_SQE] = TRUE; }
	if ( qTryes > 0 ) { c_eventos[c_ev_Q_TRYES_NOT_0] = TRUE; }
	// ev_CREGRSP_OK		CREGrsp == +CREG 0,1
	if (  GPRS_stateVars.flags.modemResponse == MRSP_CREG ) { c_eventos[c_ev_CREGRSP_OK] = TRUE; }
	// ev_IPASSIGNED		E2IPA: 000
	if (  GPRS_stateVars.flags.modemResponse == MRSP_E2IPA ) { c_eventos[c_ev_IPASSIGNED] = TRUE; }

	// MSG RELOAD
	if ( g_checkReloadConfig(gST_CONFIGURAR) ) {
		return;
	}

	switch ( GPRS_stateVars.state.subState ) {
	case gSST_CONFIGURAR_00:
		GPRS_stateVars.state.subState = gTR_C00();
		break;
	case gSST_CONFIGURAR_01:
		GPRS_stateVars.state.subState = gTR_C01();
		break;
	case gSST_CONFIGURAR_02:
		if ( c_eventos[c_ev_GSMBAND_OK] )  {
			GPRS_stateVars.state.subState = gTR_C03();
		} else {
			GPRS_stateVars.state.subState = gTR_C02();
		}
		break;
	case gSST_CONFIGURAR_03:
		if ( c_eventos[c_ev_CTIMER_NOT_0] )  {
			GPRS_stateVars.state.subState = gTR_C04();
		} else {
			GPRS_stateVars.state.subState = gTR_C05();
		}
		break;
	case gSST_CONFIGURAR_04:
		if ( c_eventos[c_ev_CREGRSP_OK] )  {
			GPRS_stateVars.state.subState = gTR_C09();
		} else {
			GPRS_stateVars.state.subState = gTR_C06();
		}
		break;
	case gSST_CONFIGURAR_05:
		if ( c_eventos[c_ev_P_TRYES_NOT_0] )  {
			GPRS_stateVars.state.subState = gTR_C07();
		} else {
			GPRS_stateVars.state.subState = gTR_C08();
		}
		break;
	case gSST_CONFIGURAR_06:
		if ( c_eventos[c_ev_CTIMER_NOT_0] )  {
			GPRS_stateVars.state.subState = gTR_C10();
		} else {
			GPRS_stateVars.state.subState = gTR_C11();
		}
		break;
	case gSST_CONFIGURAR_07:
		if ( c_eventos[c_ev_WKMONITOR_SQE] )  {
			GPRS_stateVars.state.subState = gTR_C12();
		} else {
			GPRS_stateVars.state.subState = gTR_C13();
		}
		break;
	case gSST_CONFIGURAR_08:
		GPRS_stateVars.state.subState = gTR_C14();
		break;
	case gSST_CONFIGURAR_09:
		if ( c_eventos[c_ev_CTIMER_NOT_0] )  {
			GPRS_stateVars.state.subState = gTR_C15();
		} else {
			GPRS_stateVars.state.subState = gTR_C16();
		}
		break;
	case gSST_CONFIGURAR_10:
		if ( c_eventos[c_ev_IPASSIGNED] )  {
			GPRS_stateVars.state.subState = gTR_C20();
		} else {
			GPRS_stateVars.state.subState = gTR_C17();
		}
		break;
	case gSST_CONFIGURAR_11:
		if ( c_eventos[c_ev_P_TRYES_NOT_0] )  {
			GPRS_stateVars.state.subState = gTR_C19();
		} else {
			GPRS_stateVars.state.subState = gTR_C18();
		}
		break;
	case gSST_CONFIGURAR_12:
		if ( c_eventos[c_ev_Q_TRYES_NOT_0] )  {
			GPRS_stateVars.state.subState = gTR_C21();
		} else {
			GPRS_stateVars.state.subState = gTR_C22();
		}
		break;
	default:
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\ntkGprs::ERROR sst_gprsConfigurar: subState  (%d) NOT DEFINED\r\n\0"),GPRS_stateVars.state.subState);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		pv_cambiarEstado(gST_CONFIGURAR,gST_MODEMAPAGADO);
		break;
	}

}
/*------------------------------------------------------------------------------------*/
static int gTR_C00(void)
{

	// Configuro el modem.

	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );
	//
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n%s: GPRS configure:\r\n\0"), u_now() );
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT&D0&C1\r\0", sizeof("AT&D0&C1\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();


	// Configuro la secuencia de escape +++AT
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*E2IPS=2,8,2,1020,1,15\r\0", sizeof("AT*E2IPS=2,8,2,1020,1,15\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// SMS Envio: Los envio en modo texto
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT+CMGF=1\r\0", sizeof("AT+CMGF=1\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// SMS Recepcion: No indico al TE ni le paso el mensaje
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT+CNMI=1,0\r\0", sizeof("AT+CNMI=1,0\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// SMS indicacion: Bajando el RI por 100ms.
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*E2SMSRI=100\r\0", sizeof("AT*E2SMSRI=100\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// Deshabilito los mensajes
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*E2IPEV=0,0\r\0", sizeof("AT*E2IPEV=0,0\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Modem Configurado.\r\n\0"));
	u_logPrint(gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("C00\0");
	return(gSST_CONFIGURAR_01);
}
//------------------------------------------------------------------------------------
static int gTR_C01(void)
{
	// Configuro la banda.

char bandBuffer[32];
char *ts = NULL;
u08 modemBand;
size_t xBytes;

	// Vemos si la banda configurada es la correcta. Si no la reconfiguro.

	// Leo la banda que tiene el modem configurada
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*EBSE?\r\0", sizeof("AT*EBSE?\r\0") );

	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// Extraigo de la respuesta la banda
	memcpy(bandBuffer, FreeRTOS_UART_getFifoPtr(&pdUART0), sizeof(bandBuffer) );
	ts = strchr(bandBuffer, ':');
	ts++;
	modemBand = atoi(ts);

	tickCount = xTaskGetTickCount();
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("mBAND=%d,sBAND=%d\r\n\0"),modemBand, systemVars.gsmBand);
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	GPRS_stateVars.flags.gsmBandOK = TRUE;
	if ( modemBand != systemVars.gsmBand ) {
		// Debo reiniciar el modem
		GPRS_stateVars.flags.gsmBandOK = FALSE;	// Para que luego el modem se resetee en el siguiente estado

		// Reconfiguro.
		xBytes = snprintf_P( gprs_printfBuff,CHAR256,PSTR("AT*EBSE=%d\r\0"),systemVars.gsmBand );
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
		g_flushRXBuffer();
		FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff));
		vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

		// Guardo el profile
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
		g_flushRXBuffer();
		FreeRTOS_write( &pdUART0, "AT&W\r\0", sizeof("AT&W\r\0") );
		vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n%s: GPRS Reconfiguro GSM_BAND a modo %d:\r\n\0"), u_now(),systemVars.gsmBand);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	g_printExitMsg("C01\0");
	return(gSST_CONFIGURAR_02);
}
//------------------------------------------------------------------------------------
static int gTR_C02(void)
{
	// Debo reiniciar el modem para que tome la nueva banda

	g_printExitMsg("C02\0");
	return( pv_cambiarEstado(gST_CONFIGURAR,gST_MODEMAPAGADO) );
}
//------------------------------------------------------------------------------------
static int gTR_C03(void)
{

	// Trato de atachearme a la red

	cTimer = 6;		// a intervalos de 6s entre consultas
	pTryes = 10;	// Pregunto hasta 10 veces

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	FreeRTOS_write( &pdUART0, "AT+CREG?\r\0", sizeof("AT+CREG?\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Banda GRPS OK.\r\n\0"));
	u_logPrint(gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("C03\0");
	return(gSST_CONFIGURAR_03);
}
//------------------------------------------------------------------------------------
static int gTR_C04(void)
{
	// Espero 1 segundo

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	if ( cTimer > 0 ) {
		--cTimer;
	}

	//g_printExitMsg("C04\0");
	return(gSST_CONFIGURAR_03);
}
//------------------------------------------------------------------------------------
static int gTR_C05(void)
{
	// Chequeo la respuesta

size_t pos;

	// Leo y Evaluo la respuesta al comando AT+CREG ( home network )
	GPRS_stateVars.flags.modemResponse = MRSP_NONE;
	if ( g_strstr("+CREG: 0,1\0", &pos ) == TRUE ) {
		GPRS_stateVars.flags.modemResponse = MRSP_CREG;
	}
	//( roaming !!!. Se usa en Concordia )
	//if ( systemVars.roaming == TRUE ) {
	//	if (g_strstr("+CREG: 0,5\0", &pos ) == TRUE ) {
	//		GPRSrsp = RSP_CREG;
	//	}
	//}

	g_printRxBuffer();

	g_printExitMsg("C05\0");
	return(gSST_CONFIGURAR_04);
}
//------------------------------------------------------------------------------------
static int gTR_C06(void)
{

	g_printExitMsg("C06\0");
	return(gSST_CONFIGURAR_05);
}
//------------------------------------------------------------------------------------
static int gTR_C07(void)
{

	// Reintento
	if ( pTryes > 0 ) {
		--pTryes;
	}

	cTimer = 6;	// a intervalos de 6s entre consultas

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	FreeRTOS_write( &pdUART0, "AT+CREG?\r\0", sizeof("AT+CREG?\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

	g_printExitMsg("C07\0");
	return(gSST_CONFIGURAR_03);
}
//------------------------------------------------------------------------------------
static int gTR_C08(void)
{
	// Debo reiniciar el modem.

	g_printExitMsg("C08\0");
	return( pv_cambiarEstado(gST_CONFIGURAR,gST_MODEMAPAGADO) );
}
//------------------------------------------------------------------------------------
static int gTR_C09(void)
{

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Vinculado a la red gprs.\r\n\0"));
	u_logPrint(gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Leo el SQE
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n%s: query SQE:\r\n\0"), u_now());
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	cTimer = 5;		// Espero 5s desde que doy el comando hasta que pregunto.

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	FreeRTOS_write( &pdUART0, "AT+CSQ\r\0", sizeof("AT+CSQ\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

	g_printExitMsg("C09\0");
	return(gSST_CONFIGURAR_06);
}
//------------------------------------------------------------------------------------
static int gTR_C10(void)
{
	// Espero 1 segundo
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	if ( cTimer > 0 ) {
		--cTimer;
	}

	//g_printExitMsg("C10\0");
	return(gSST_CONFIGURAR_06);
}
//------------------------------------------------------------------------------------
static int gTR_C11(void)
{

	// Chequeo la respuesta al SQE

size_t pos;
char csqBuffer[32];
char *ts = NULL;

	if ( g_strstr("CSQ:\0", &pos ) == TRUE ) {

		g_printRxBuffer();

		memcpy(csqBuffer, g_getRxBuffer(), sizeof(csqBuffer) );
		ts = strchr(csqBuffer, ':');
		ts++;
		systemVars.csq = atoi(ts);
		systemVars.dbm = 113 - 2 * systemVars.csq;

		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\nCSQ=%d,DBM=%d\r\n\0"),systemVars.csq,systemVars.dbm);
		u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Calidad de señal %d db.\r\n\0"),systemVars.dbm );
		u_logPrint(gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	g_printExitMsg("C11\0");
	return(gSST_CONFIGURAR_07);
}
//------------------------------------------------------------------------------------
static int gTR_C12(void)
{

	// En modo monitor SQE pregunto c/15s.

	cTimer = 15;	// Repregunto c/15s.

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	FreeRTOS_write( &pdUART0, "AT+CSQ\r\0", sizeof("AT+CSQ\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

	g_printExitMsg("C12\0");
	return(gSST_CONFIGURAR_06);
}
//------------------------------------------------------------------------------------
static int gTR_C13(void)
{

	// Fijo el APN

size_t xBytes;

	// APN
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n%s: GPRS set APN:\r\n\0"), u_now());
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	xBytes = snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT+CGDCONT=1,\"IP\",\"%s\"\r\0"),systemVars.apn);
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	qTryes = 3;	// Pregunto hasta 3 veces por la IP

	g_printExitMsg("C13\0");
	return(gSST_CONFIGURAR_08);
}
//------------------------------------------------------------------------------------
static int gTR_C14(void)
{
	// Pido la IP

	cTimer = 10;	// espero 10s antes de consultar
	pTryes = 6;		// Pregunto hasta 6 veces antes de reenviar el comando

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n[%s] GPRS ask IP:\r\n\0"), u_now());
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	FreeRTOS_write( &pdUART0, "AT*E2IPA=1,1\r\0", sizeof("AT*E2IPA=1,1\r\0") );

	g_printExitMsg("C14\0");
	return(gSST_CONFIGURAR_09);
}
//------------------------------------------------------------------------------------
static int gTR_C15(void)
{
	// Espero 1 segundo
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	if ( cTimer > 0 ) {
		--cTimer;
	}

	//g_printExitMsg("C15\0");
	return(gSST_CONFIGURAR_09);
}
//------------------------------------------------------------------------------------
static int gTR_C16(void)
{

size_t pos;

	// Leo y Evaluo la respuesta al comando AT+CREG ( home network )
	GPRS_stateVars.flags.modemResponse = MRSP_NONE;
	if ( g_strstr("E2IPA: 000\0", &pos ) == TRUE ) {
		GPRS_stateVars.flags.modemResponse = MRSP_E2IPA;
	}

	g_printRxBuffer();

	g_printExitMsg("C16\0");
	return(gSST_CONFIGURAR_10);
}
//------------------------------------------------------------------------------------
static int gTR_C17(void)
{

	if ( pTryes > 0 ) {
		--pTryes;
	}

	g_printExitMsg("C17\0");
	return(gSST_CONFIGURAR_11);
}
//------------------------------------------------------------------------------------
static int gTR_C18(void)
{

	if  ( qTryes > 0 ) {
		--qTryes;
	}

	g_printExitMsg("C18\0");
	return(gSST_CONFIGURAR_12);
}
//------------------------------------------------------------------------------------
static int gTR_C19(void)
{
	// Reinicio cTimer

	cTimer = 10;

	g_printExitMsg("C19\0");
	return(gSST_CONFIGURAR_09);
}
//------------------------------------------------------------------------------------
static int gTR_C20(void)
{
	// Tengo la IP asignada: la leo para actualizar systemVars.ipaddress y salgo

char *ts = NULL;
int i=0;
char c;

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*E2IPI=0\r\0", sizeof("AT*E2IPI=0\r\0") );
	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );

	//  Muestro la IP en pantalla
	g_printRxBuffer();

	// Extraigo la IP del token. Voy a usar el buffer  de print ya que la respuesta
	// puede ser grande.
	memcpy(gprs_printfBuff, FreeRTOS_UART_getFifoPtr(&pdUART0), sizeof(gprs_printfBuff) );

	ts = strchr( gprs_printfBuff, '\"');
	ts++;
	while ( (c= *ts) != '\"') {
		systemVars.dlgIp[i++] = c;
		ts++;
	}
	systemVars.dlgIp[i++] = '\0';
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\nIPADDRESS=[%s]\r\n\0"),systemVars.dlgIp);
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Cambio de estado. Vengo de prender el modem por lo que
	// debo mandar INITS
	g_setSocketStatus(SOCKET_CLOSED);
	// Voy a reintentar hasta 3 inits en pwrDiscreto.
	GPRS_stateVars.counters.nroINITS = 4;
	GPRS_stateVars.state.nextFrame = INIT_FRAME;

	GPRS_stateVars.counters.nroLOTEtryes = MAXTRYESLOTE;

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Direccion IP asignada.\r\n\0"));
	u_logPrint(gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("C20\0");
	return( pv_cambiarEstado(gST_CONFIGURAR,gST_STANDBY) );
}
//------------------------------------------------------------------------------------
static int gTR_C21(void)
{

	g_printExitMsg("C21\0");
	return(gSST_CONFIGURAR_08);
}
//------------------------------------------------------------------------------------
static int gTR_C22(void)
{
	// Apago.

	g_printExitMsg("C22\0");
	return( pv_cambiarEstado(gST_CONFIGURAR,gST_MODEMAPAGADO) );
}
//------------------------------------------------------------------------------------

