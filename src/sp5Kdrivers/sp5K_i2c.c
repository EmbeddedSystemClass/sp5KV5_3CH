/*
 * sp5K_i2c.c
 *
 *  Created on: 18/10/2015
 *      Author: pablo
 */


#include "sp5K_i2c.h"
#include "../sp5KV5_3CH.h"

static void pvI2C_setBitRate(int bitrateKHz);
inline void pvI2C_sendStart(void);
inline void pvI2C_sendStop(void);
inline void pvI2C_sendByte(u08 data);
inline void pvI2C_readByte(u08 ackFlag);
inline u08 pvI2C_getStatus(void);
inline s08 pvI2C_waitForComplete(void);
inline void pvI2C_disable(void);

//------------------------------------------------------------------------------------
void i2c_init(void)
{
	// set pull-up resistors on I2C bus pins
	//sbi(PORTC, 0);  // i2c SCL
	//sbi(PORTC, 1);  // i2c SDA
	PORTC |= 1 << SCL;
	PORTC |= 1 << SDA;
	// set i2c bit rate to 100KHz
	pvI2C_setBitRate(100);
}
//------------------------------------------------------------------------------------
s08 I2C_masterWrite ( const u08 devAddress, const u08 devAddressLength, const u16 byteAddress, char *pvBuffer, size_t xBytes  )
{

u08 tryes = 0;
u08 i2c_status;
char txbyte;
s08 retV = FALSE;
u08 i;

i2c_retry:

#ifdef DEBUG_I2C
	snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MW0: 0x%02x,0x%02x,0x%02x,0x%02x\r\n\0"),devAddress,devAddressLength, (u16)(byteAddress), xBytes );
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif

	if (tryes++ >= I2C_MAXTRIES) goto i2c_quit;

	// Pass1) START: Debe retornar 0x08 (I2C_START) o 0x10 (I2C_REP_START) en condiciones normales
	pvI2C_sendStart();
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
	snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MW1: 0x%02x\r\n\0"),i2c_status );
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
	if ( i2c_status == TW_MT_ARB_LOST) goto i2c_retry;
	if ( (i2c_status != TW_START) && (i2c_status != TW_REP_START) ) goto i2c_quit;

	// Pass2) (SLA_W) Send slave address. Debo recibir 0x18 ( SLA_ACK )
	txbyte = devAddress | TW_WRITE;
	pvI2C_sendByte(txbyte);
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
	snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MW2: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
	// Check the TWSR status
	if ((i2c_status == TW_MT_SLA_NACK) || (i2c_status == TW_MT_ARB_LOST)) goto i2c_retry;
	if (i2c_status != TW_MT_SLA_ACK) goto i2c_quit;

	// Pass3) Envio la direccion fisica donde comenzar a escribir.
	// En las memorias es una direccion de 2 bytes.En el DS1344 o el BusController es de 1 byte
	// Envio primero el High 8 bit i2c address. Debo recibir 0x28 ( DATA_ACK)
	if ( devAddressLength == 2 ) {
		txbyte = (byteAddress) >> 8;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
		snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MW3H: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}

	// Envio el Low 8 byte i2c address.
	if ( devAddressLength >= 1 ) {
		txbyte = (byteAddress) & 0x00FF;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
		snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MW3L: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}

	// Pass4) Trasmito todos los bytes del buffer. Debo recibir 0x28 (DATA_ACK)
	for ( i=0; i < xBytes; i++ ) {
		txbyte = *pvBuffer++;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
		snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MW4: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}

	retV = TRUE;

i2c_quit:

	// Pass5) STOP
	pvI2C_sendStop();
	vTaskDelay( ( TickType_t)( 20 / portTICK_RATE_MS ) );

	// En caso de error libero la interface.
	if (retV == FALSE)
		pvI2C_disable();

	return(retV);


}
//------------------------------------------------------------------------------------
s08 I2C_masterRead  ( const u08 devAddress, const u08 devAddressLength, const u16 byteAddress, char *pvBuffer, size_t xBytes  )
{
	// En el caso del ADC, el read no lleva la parte de mandar la SLA+W. !!!!!

u08 tryes = 0;
u08 i2c_status;
char txbyte;
s08 retV = FALSE;
u08 i;

i2c_retry:

#ifdef DEBUG_I2C
	snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MR0: 0x%02x,0x%02x,0x%02x,0x%02x\r\n\0"),devAddress,devAddressLength, (u16)(byteAddress), xBytes );
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif

	if (tryes++ >= I2C_MAXTRIES) goto i2c_quit;

	// Pass1) START: Debe retornar 0x08 (I2C_START) o 0x10 (I2C_REP_START) en condiciones normales
	pvI2C_sendStart();
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
	snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MR1: 0x%02x\r\n\0"),i2c_status );
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
	if ( i2c_status == TW_MT_ARB_LOST) goto i2c_retry;
	if ( (i2c_status != TW_START) && (i2c_status != TW_REP_START) ) goto i2c_quit;

	// En el caso del ADC, el read no lleva la parte de mandar la SLA+W. !!!!!
	if ( devAddress == ADS7828_ADDR)
		goto SRL_R;

	// Pass2) (SLA_W) Send slave address. Debo recibir 0x18 ( SLA_ACK )
	txbyte = devAddress | TW_WRITE;
	pvI2C_sendByte(txbyte);
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	// Check the TWSR status
	i2c_status = pvI2C_getStatus();
	if ((i2c_status == TW_MT_SLA_NACK) || (i2c_status == TW_MT_ARB_LOST)) goto i2c_retry;
	if (i2c_status != TW_MT_SLA_ACK) goto i2c_quit;
#ifdef DEBUG_I2C
	snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MR2: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
	// Pass3) Envio la direccion fisica donde comenzar a escribir.
	// En las memorias es una direccion de 2 bytes.En el DS1344 o el BusController es de 1 byte
	// Envio primero el High 8 bit i2c address. Debo recibir 0x28 ( DATA_ACK)
	if ( devAddressLength == 2 ) {
		txbyte = (byteAddress) >> 8;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
		snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MR3H: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}

	// Envio el Low 8 byte i2c address.
	if ( devAddressLength >= 1 ) {
		txbyte = (byteAddress) & 0x00FF;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
		snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MR3L: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}


	// Pass4) REPEATED START. Debo recibir 0x10 ( REPEATED_START)
	pvI2C_sendStart();
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
	snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MR4: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
	if (i2c_status == TW_MT_ARB_LOST) goto i2c_retry;
	if ( (i2c_status != TW_START) && (i2c_status != TW_REP_START) ) goto i2c_quit;

SRL_R:
	// Pass5) (SLA_R) Send slave address + READ. Debo recibir un 0x40 ( SLA_R ACK)
	txbyte = devAddress | TW_READ;
	pvI2C_sendByte(txbyte);
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	// Check the TWSR status
	i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
	snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MR5: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
	if ((i2c_status == TW_MR_SLA_NACK) || (i2c_status == TW_MR_ARB_LOST)) goto i2c_retry;
	if (i2c_status != TW_MR_SLA_ACK) goto i2c_quit;

	// Pass6) Leo todos los bytes requeridos y respondo con ACK.
	for ( i=0; i < (xBytes-1); i++ ) {
		pvI2C_readByte(ACK);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
		snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MR6(%d): 0x%02x,0x%02x\r\n\0"),i,TWDR,i2c_status );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
		if (i2c_status != TW_MR_DATA_ACK) goto i2c_quit;
		*pvBuffer++ = TWDR;
	}

	// Pass7) Acepto el ultimo byte y respondo con NACK
	pvI2C_readByte(NACK);
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
	snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MR6(%d): 0x%02x,0x%02x\r\n\0"),i,TWDR,i2c_status );
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
	if (i2c_status != TW_MR_DATA_NACK) goto i2c_quit;
	*pvBuffer++ = TWDR;

	// I2C read OK.
	retV = TRUE;

i2c_quit:

	// Pass5) STOP
	pvI2C_sendStop();

	// En caso de error libero la interface.
	if (retV == FALSE)
		pvI2C_disable();

//	if ( retV ) {
//		snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MR RET TRUE ( %d )\r\n\0"),retV );
//	} else {
//		snprintf_P( debug_printfBuff,CHAR128,PSTR("I2C_MR RET FALSE ( %d )\r\n\0"),retV );
//	}

//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
	return(retV);

}
//------------------------------------------------------------------------------------
// FUNCIONES AUXILIARES PRIVADAS DE I2C
//------------------------------------------------------------------------------------
static void pvI2C_setBitRate(int bitrateKHz)
{
int bitrate_div;

	// SCL freq = F_CPU/(16+2*TWBR*4^TWPS)

	// set TWPS to zero
	//cbi(TWSR, TWPS0);
	//cbi(TWSR, TWPS1);
	((TWSR) &= ~(1 << (TWPS0)));
	((TWSR) &= ~(1 << (TWPS1)));
	// SCL freq = F_CPU/(16+2*TWBR))

	// calculate bitrate division
	bitrate_div = ((F_CPU/1000l)/bitrateKHz);
	if(bitrate_div >= 16) {
		bitrate_div = (bitrate_div-16)/2;
	}

	TWBR = bitrate_div;
}
//------------------------------------------------------------------------------------
inline void pvI2C_sendStart(void)
{
	// Genera la condicion de START en el bus I2C.
	// TWCR.TWEN = 1 : Habilita la interface TWI
	// TWCR.TWSTA = 1: Genera un START
	// TWCR.TWINT = 1: Borra la flag e inicia la operacion del TWI
	//
	// Cuando el bus termino el START, prende la flag TWCR.TWINT y el codigo
	// de status debe ser 0x08 o 0x10.
	//

	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

}
//-----------------------------------------------------------------------------------
inline void pvI2C_sendStop(void)
{
	// Genera la condicion STOP en el bus I2C.
	// !!! El TWINT NO ES SETEADO LUEGO DE UN STOP
	// por lo tanto no debo esperar que termine.

	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

}
//------------------------------------------------------------------------------------
inline void pvI2C_sendByte(u08 data)
{

	// Envia el data por el bus I2C.
	TWDR = data;
	// Habilita la trasmision y el envio de un ACK.
	TWCR = (1 << TWINT) | (1 << TWEN);

}
//------------------------------------------------------------------------------------
inline void pvI2C_readByte(u08 ackFlag)
{
	// begin receive over i2c
	if( ackFlag )
	{
		// ackFlag = TRUE: ACK the recevied data
		TWCR = (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
	}
	else
	{
		// ackFlag = FALSE: NACK the recevied data
		TWCR = (1 << TWINT) | (1 << TWEN);
	}

}
//------------------------------------------------------------------------------------
inline u08 pvI2C_getStatus(void)
{
	// retieve current i2c status from i2c TWSR
	return( TWSR & 0xF8 );
}
//------------------------------------------------------------------------------------
inline s08 pvI2C_waitForComplete(void)
{
TickType_t xTicksToWait = 3;		// 3 ticks ( 30ms es el maximo tiempo que espero )
TimeOut_t xTimeOut;

	// inicializo el timeout
	vTaskSetTimeOutState( &xTimeOut );

	// wait for i2c interface to complete operation
	while( !(TWCR & (1<<TWINT)) )
	{
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE ) {
			// Timed out
			return(FALSE);
		}
	}

	return(TRUE);
}
//------------------------------------------------------------------------------------
inline void pvI2C_disable(void)
{
	// Deshabilito la interfase TWI
	TWCR &= ~(1 << TWEN);

}
//------------------------------------------------------------------------------------
