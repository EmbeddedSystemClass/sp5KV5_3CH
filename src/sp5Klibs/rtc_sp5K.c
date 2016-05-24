/*
 * sp5KFRTOS_rtc.c
 *
 *  Created on: 01/11/2013
 *      Author: root
 *
 * Funciones del RTC DS1340-33 modificadas para usarse con FRTOS.
 *
 *
 */
//------------------------------------------------------------------------------------

#include "rtc_sp5K.h"

static char pv_bcd2dec(char num);
static char pv_dec2bcd(char num);

//------------------------------------------------------------------------------------
// Funciones de uso general
//------------------------------------------------------------------------------------
s08 RTC_read(RtcTimeType_t *rtc)
{
	// Retorna la hora formateada en la estructura RtcTimeType_t
	// No retornamos el valor de EOSC ni los bytes adicionales.

u08 oscStatus;
u08 data[8];
size_t xReturn = 0U;
u16 val = 0;
u08 xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = RTC_DEVADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	// Luego indicamos en que posicion del periferico queremos leer: largo
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion
	val = 0;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	// Por ultimo leemos 8 bytes.
	xBytes = sizeof(data);
	xReturn = FreeRTOS_read(&pdI2C, &data, xBytes);
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( FALSE );
	}

	// Decodifico los resultados.
	oscStatus = 0;
	if ( (data[0] & 0x80) == 0x80 ) {		// EOSC
		oscStatus = 1;
	}
	rtc->sec = pv_bcd2dec(data[0] & 0x7F);
	rtc->min = pv_bcd2dec(data[1]);
	rtc->hour = pv_bcd2dec(data[2] & 0x3F);
	rtc->day = pv_bcd2dec(data[4] & 0x3F);
	rtc->month = pv_bcd2dec(data[5] & 0x1F);
	rtc->year = pv_bcd2dec(data[6]) + 2000;

	return(TRUE);
}
//------------------------------------------------------------------------------------
s08 RTC_write(RtcTimeType_t *rtc)
{
	// Setea el RTC con la hora pasada en la estructura RtcTimeType

u08 data[8];
size_t xReturn = 0U;
u16 val = 0;
u08 xBytes = 0;

	data[0] = 0;	// EOSC = 0 ( rtc running)
	data[1] = pv_dec2bcd(rtc->min);
	data[2] = pv_dec2bcd(rtc->hour);
	data[3] = 0;
	data[4] = pv_dec2bcd(rtc->day);
	data[5] = pv_dec2bcd(rtc->month);
	data[6] = pv_dec2bcd(rtc->year);

	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
	val = RTC_DEVADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH,&val);
	val = 0;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( FALSE );
	}

	return(TRUE);
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static char pv_dec2bcd(char num)
{
	// Convert Decimal to Binary Coded Decimal (BCD)
	return ((num/10 * 16) + (num % 10));
}
//------------------------------------------------------------------------------------
static char pv_bcd2dec(char num)
{
	// Convert Binary Coded Decimal (BCD) to Decimal
	return ((num/16 * 10) + (num % 16));
}
//------------------------------------------------------------------------------------
