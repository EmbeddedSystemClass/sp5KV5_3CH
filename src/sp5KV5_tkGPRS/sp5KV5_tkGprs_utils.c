/*
 * sp5KV4_8CH_tkGprs_utils.c
 *
 *  Created on: 22 de abr. de 2016
 *      Author: pablo
 */

#include "../sp5KV5_3CH.h"
#include "sp5KV5_tkGprs.h"

//------------------------------------------------------------------------------------
void g_flushRXBuffer(void)
{
	g_setModemResponse(MRSP_NONE);
	memset(gprsRx.buffer,0, UART0_RXBUFFER_LEN );
	gprsRx.ptr = 0;

}
//------------------------------------------------------------------------------------
s08 g_strstr(char *rsp, size_t *pos)
{
	// Busca el string rsp en el buffer local de recepcion del gprs.
	// Devuelve la posicion relativa

u16 i;
char *p,*q, *res = NULL;
s08 inString = FALSE;
s08 rollover = FALSE;
s08 retS = FALSE;

	// Busca un substring en un string circular
	p = (char *)(gprsRx.buffer);
	q = rsp;

	// Chequeo que el substrig no sea vacio.
    if ( *rsp == 0) {
    	*pos = 0;
		goto quit;
    }

	// Recorro el string base. Lo recorro todo ya que como es circular, puede
	// tener un \0 en el medio.
    i = 0;
	inString = FALSE;
    while(1) {
		if ( *p == *q ) {
			if ( inString == FALSE ) {
				res = p;	// Guardo la primer posicion de coincidencia.
				*pos = i;
			}
			inString = TRUE;
			q++;
			if ( *q == '\0')
				// Esta es la unica condicion de salida valida.
				break;
		} else {
			// Reinicio las condiciones de busqueda
			inString = FALSE;
			q = rsp;
			*pos = 0;
			if ( rollover )	// Ya di vuelta y no coincide.
				break;
		}
		// Avanzo
		p++;
		i++;
		if ( i == UART0_RXBUFFER_LEN ) {
			// Llegue al final. Rollover.
			i = 0;
			p = (char *)(gprsRx.buffer);
			rollover = TRUE;
		}
    }

    if ( ! inString) {
 		 // FAIL
		res = NULL;
  	}

 quit:

	if ( res != NULL) {
		// FOUND
		retS = TRUE;
	}
	return(retS);

}
//------------------------------------------------------------------------------------
void g_printRxBuffer(void)
{

	// Imprime la respuesta a un comando.
	// Utiliza el buffer de RX.

	if ( (systemVars.debugLevel & D_GPRS) != 0) {
		tickCount = xTaskGetTickCount();
		snprintf_P( gprsRX_printfBuff,sizeof(gprsRX_printfBuff),PSTR(".[%06lu] tkGprs: Rsp=\r\n\0"),tickCount  );
		FreeRTOS_write( &pdUART1, gprsRX_printfBuff, sizeof(gprsRX_printfBuff) );

		// Imprimo todo el buffer de RX ( 640b). Sale por \0.
		FreeRTOS_write( &pdUART1, gprsRx.buffer, UART0_RXBUFFER_LEN );
		// Agrego un CRLF por las dudas
		FreeRTOS_write( &pdUART1, "\r\n\0", sizeof("\r\n\0") );
	}
}
//------------------------------------------------------------------------------------
char *g_getRxBuffer(void)
{
	return (gprsRx.buffer);
}
//------------------------------------------------------------------------------------
s08 u_modemPwrStatus(void)
{
	if ( GPRS_stateVars.state.state == gST_MODEMAPAGADO ) {
		return(OFF);
	} else {
		return(ON);
	}
}
//--------------------------------------------------------------------------------------
void g_setSocketStatus( u08 socketStatus)
{
	GPRS_stateVars.flags.socketStatus = socketStatus;

}
//-------------------------------------------------------------------------------------
void g_setModemResponse( u08 modemStatus)
{
	//GPRS_flags.modemResponse = modemStatus;
}
//-------------------------------------------------------------------------------------
void g_printExitMsg(char *code)
{
	tickCount = xTaskGetTickCount();
	memset( gprs_printfBuff,'\0',sizeof(gprs_printfBuff));
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs: exit %s\r\n\0"), tickCount,code);
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );
}
//------------------------------------------------------------------------------------
t_tkGprs_subState pv_cambiarEstado( t_tkGprs_state actualState, t_tkGprs_state newState )
{

	// Dejo el registro del estado del que vengo.
	GPRS_stateVars.state.oldState = actualState;

	// Cambio al nuevo estado
	switch( newState ) {
	case gST_MODEMAPAGADO:
		GPRS_stateVars.state.state = gST_MODEMAPAGADO;
		GPRS_stateVars.state.subState = gSST_MODEMAPAGADO_00;
		break;
	case gST_MODEMPRENDIENDO:
		GPRS_stateVars.state.state = gST_MODEMPRENDIENDO;
		GPRS_stateVars.state.subState = gSST_MODEMPRENDIENDO_00;
		break;
	case gST_CONFIGURAR:
		GPRS_stateVars.state.state = gST_CONFIGURAR;
		GPRS_stateVars.state.subState = gSST_CONFIGURAR_00;
		break;
	case gST_STANDBY:
		GPRS_stateVars.state.state = gST_STANDBY;
		GPRS_stateVars.state.subState = gSST_STANDBY_00;
		break;
	case gST_OPENSOCKET:
		GPRS_stateVars.state.state = gST_OPENSOCKET;
		GPRS_stateVars.state.subState = gSST_OPENSOCKET_00;
		break;
	case gST_INITFRAME:
		GPRS_stateVars.state.state = gST_INITFRAME;
		GPRS_stateVars.state.subState = gSST_INITFRAME_00;
		break;
	case gST_CONFFRAME:
		GPRS_stateVars.state.state = gST_CONFFRAME;
		GPRS_stateVars.state.subState = gSST_CONFFRAME_00;
		break;
	case gST_DATAFRAME:
		GPRS_stateVars.state.state = gST_DATAFRAME;
		GPRS_stateVars.state.subState = gSST_DATAFRAME_00;
		break;
	}

	return(GPRS_stateVars.state.subState);
}
//------------------------------------------------------------------------------------
void g_GPRSprocessServerClock(void)
{
/* Extraigo el srv clock del string mandado por el server y si el drift con la hora loca
 * es mayor a 5 minutos, ajusto la hora local
 * La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:PWRM=DISC:</h1>
 *
 */

char *p, *s;
unsigned long serverDate, localDate;
char rtcStr[12];
s08 retS = FALSE;
RtcTimeType_t ltime;

	tickCount = xTaskGetTickCount();
	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "CLOCK");
	if ( p == NULL )
		return;

	// Incremento para que apunte al str.con la hora.
	p += 6;
	serverDate = atol(p);
	if ( ! RTC_read(&ltime) )
		return;

	// Calculo la hora recibida desde el server:
	ltime.year -= 2000;
	snprintf_P( rtcStr,sizeof(rtcStr),PSTR("%02d%02d%02d%02d%02d\0"), ltime.year,ltime.month, ltime.day,ltime.hour,ltime.min);
	localDate = atol(rtcStr);

	memset(gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	// Comparo con la hora local.
	if ( abs( serverDate - localDate ) > 5 ) {
		// La diferencia es de mas de 5s: debo reajustar.
		memset(rtcStr, '\0', sizeof(rtcStr));
		memcpy(rtcStr,p, sizeof(rtcStr));
		retS = u_wrRtc(rtcStr);
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs::UPDATE SrvTime: %lu, LocalTime: %lu\r\n\0"),tickCount, serverDate, localDate);
	} else {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs::RTC OK; SrvTime: %lu, LocalTime: %lu\r\n\0"),tickCount, serverDate, localDate);
	}

	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

}
//------------------------------------------------------------------------------------
u08 g_GPRSprocessPwrMode(void)
{
char *s;
u08 ret = 0;

	tickCount = xTaskGetTickCount();
	s = FreeRTOS_UART_getFifoPtr(&pdUART0);

	if ( strstr( s, "PWRM=DISC") != NULL ) {
		u_configPwrMode(PWR_DISCRETO);
		ret = 1;
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs:: Reconfig PWRM to DISC\r\n\0"),tickCount);
	}
	else if (strstr( s, "PWRM=CONT") != NULL ) {
		u_configPwrMode(PWR_CONTINUO);
		ret = 1;
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs:: Reconfig PWRM to CONT\r\n\0"),tickCount);
	}
	else {
		// Para el caso que no halla recibido el parametro PWRM
		goto quit;
	}

	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

quit:
	return(ret);
}
//------------------------------------------------------------------------------------
u08 g_GPRSprocessTimerPoll(void)
{
//	La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:TPOLL=600:PWRM=DISC:</h1>

char *p, *s;
u08 ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";

	tickCount = xTaskGetTickCount();
	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "TPOLL");
	if ( p == NULL )
		goto quit;

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	// TPOLL

	token = strsep(&stringp,delim);	// timerPoll
	u_configTimerPoll(token);
	ret = 1;
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs:: Reconfig TPOLL\r\n\0"),tickCount);

	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

quit:
	return(ret);
}
//------------------------------------------------------------------------------------
u08 g_GPRSprocessTimerDial(void)
{
	//	La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:CD=1230:CN=0530</h1>

char *p, *s;
u08 ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";

	tickCount = xTaskGetTickCount();
	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "TDIAL");
	if ( p == NULL )
		goto quit;

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	// TDIAL

	token = strsep(&stringp,delim);	// timerDial
	u_configTimerDial(token);
	ret = 1;
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs:: Reconfig TDIAL\r\n\0"),tickCount);

	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

quit:
	return(ret);
}
//------------------------------------------------------------------------------------
void g_GPRSprocessReset(void)
{
	// El server me respondio con RESET para que borre las flags de alarma.

char *p, *s;

	tickCount = xTaskGetTickCount();
	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "RESET");
	if ( p != NULL ) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("Going to RESET...\r\n\0" ));
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		// RESET
		u_reset();
	}
}
//------------------------------------------------------------------------------------
u08 g_GPRSprocessPwrSave(void)
{
//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRS=1,2230,0600:D0=q0,1.00:D1=q1,1.00</h1>
//  Las horas estan en formato HHMM.

u08 ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *p1,*p2;
u08 modo;
char *p, *s;

	tickCount = xTaskGetTickCount();
	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "PWRS");
	if ( p == NULL )
		goto quit;

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	//PWRS

	token = strsep(&stringp,delim);	// modo
	modo = atoi(token);
	p1 = strsep(&stringp,delim);	// startTime
	p2 = strsep(&stringp,delim); 	// endTime

	u_configPwrSave(modo, p1, p2);
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs:: Reconfig PWRS\r\n\0"),tickCount );
	ret = 1;

	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

quit:
	return(ret);

}
//--------------------------------------------------------------------------------------
u08 g_GPRSprocessTilt(void)
{
//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:TILT=ON:PWRS=1,2230,0600:D0=q0,1.00:D1=q1,1.00</h1>

u08 ret = 0;
char *p, *s;

	tickCount = xTaskGetTickCount();
	s = FreeRTOS_UART_getFifoPtr(&pdUART0);

	p = strstr(s, "TILT=ON");
	if ( p != NULL ) {
		ret = 1;
		systemVars.tiltEnabled = TRUE;
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs:: Reconfig TILT ON\r\n\0"),tickCount );
		goto quit;
	}

	p = strstr(s, "TILT=OFF");
	if ( p != NULL ) {
		ret = 1;
		systemVars.tiltEnabled = FALSE;
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs:: Reconfig TILT OFF\r\n\0"),tickCount );
		goto quit;
	}

quit:
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );
	return(ret);

}
//--------------------------------------------------------------------------------------
u08 g_GPRSprocessAch(u08 channel)
{
//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:A0=pA,0,20,0,6:A1=pB,0,20,0,10:A2=pC,0,20,0,10:D0=q0,1.00:D1=q1,1.00</h1>

u08 ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *chName,*s_iMin,*s_iMax,*s_mMin,*s_mMax;
char *s;

	tickCount = xTaskGetTickCount();
	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	switch (channel) {
	case 0:
		stringp = strstr(s, "A0=");
		break;
	case 1:
		stringp = strstr(s, "A1=");
		break;
	case 2:
		stringp = strstr(s, "A2=");
		break;
	default:
		ret = 0;
		goto quit;
		break;
	}

	if ( stringp == NULL ) {
		ret = 0;
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,stringp,31);

	stringp = localStr;
	token = strsep(&stringp,delim);	//A0

	chName = strsep(&stringp,delim);	//name
	s_iMin = strsep(&stringp,delim);	//iMin
	s_iMax = strsep(&stringp,delim);	//iMax
	s_mMin = strsep(&stringp,delim);	//mMin
	s_mMax = strsep(&stringp,delim);	//mMax

	u_configAnalogCh( channel, chName,s_iMin,s_iMax,s_mMin,s_mMax );
	ret = 1;

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs::Reconfig A%d\r\n\0"),tickCount, channel);
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

quit:
	return(ret);
}
//--------------------------------------------------------------------------------------
u08 g_GPRSprocessDch(u08 channel)
{

//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:A0=pA,0,20,0,6:A1=pB,0,20,0,10:A2=pC,0,20,0,10:D0=q0,1.00:D1=q1,1.00</h1>

u08 ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *chName, *s_magPP;
char *s;

	tickCount = xTaskGetTickCount();
	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	switch (channel) {
	case 0:
		stringp = strstr(s, "D0=");
		break;
	case 1:
		stringp = strstr(s, "D1=");
		break;
	default:
		ret = 0;
		goto quit;
		break;
	}

	if ( stringp == NULL ) {
		ret = 0;
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,stringp,31);

	stringp = localStr;
	token = strsep(&stringp,delim);	//D0

	chName = strsep(&stringp,delim);	//name
	s_magPP = strsep(&stringp,delim);	//magPp
	u_configDigitalCh( channel, chName, s_magPP );
	ret = 1;

	tickCount = xTaskGetTickCount();
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs:: Reconfig D%d channel\r\n\0"),tickCount, channel );
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

quit:

	return(ret);

}
//--------------------------------------------------------------------------------------
s08 g_checkReloadConfig(t_tkGprs_state gprsState )
{
	if ( GPRS_stateVars.flags.msgReload == TRUE ) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\ntkGprs::GPRS reload Conf (%d)..\r\n\0"),gprsState );
		u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );
		pv_cambiarEstado(gprsState,gST_MODEMAPAGADO);
		return(TRUE);
	} else {
		return(FALSE);
	}
}

//--------------------------------------------------------------------------------------
