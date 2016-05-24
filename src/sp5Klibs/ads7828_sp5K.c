/*
 * ads7828_sp5KFRTOS.c
 *
 *  Created on: 15/4/2015
 *      Author: pablo
 */

#include "ads7828_sp5K.h"
#include "../sp5KV5_3CH.h"

//------------------------------------------------------------------------------------
s08 ADS7828_read(u08 channel, u16 *value)
{

u08 ads7828Channel;
u08 ads7828CmdByte;
u08 buffer[2];
u16 retValue;
size_t xReturn = 0U;
u16 val = 0;
u08 xBytes = 0;
s08 retS = FALSE;

	if ( channel > 7) {
		goto quit;
	}
	// Convierto el canal 0-7 al C2/C1/C0 requerido por el conversor.
	ads7828Channel = (((channel>>1) | (channel&0x01)<<2)<<4) | ADS7828_CMD_SD;
	// do conversion
	// Armo el COMMAND BYTE
	ads7828CmdByte = ads7828Channel & 0xF0;	// SD=1 ( single end inputs )
	ads7828CmdByte |= ADS7828_CMD_PDMODE2;	// Internal reference ON, A/D converter ON

	// start conversion on requested channel
	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = ADS7828_ADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	// Luego indicamos en que posicion del periferico queremos leer: largo
	val = 0;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion
	val = 0;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

	// Escribo en el ADS
	xBytes = 1;
	xReturn = FreeRTOS_write(&pdI2C, &ads7828CmdByte, xBytes);
	if (xReturn != xBytes ) {
		goto quit;
	}

	// Espero el settle time
	vTaskDelay(1);

	// retrieve conversion result
	xBytes = 2;
	xReturn = FreeRTOS_read(&pdI2C, &buffer, xBytes);
	if (xReturn != xBytes ) {
		goto quit;
	}

	retValue = (buffer[0]<<8) | buffer[1];
	// pack bytes and return result
	*value = retValue;

	// Apago el conversor
//	ads7828CmdByte = ads7828Channel & 0xF0;
//	ads7828CmdByte |= ADS7828_CMD_PDMODE0;	// Internal reference OFF, A/D converter OFF
//	status = pvADS7828_write( &ads7828CmdByte);

	retS = TRUE;
quit:
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);
	return (retS);

}
//------------------------------------------------------------------------------------


