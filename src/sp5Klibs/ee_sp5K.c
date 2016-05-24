/*
 * ee_sp5K.c
 *
 *  Created on: 26/10/2015
 *      Author: pablo
 */

#include "ee_sp5K.h"

//------------------------------------------------------------------------------------
s08 EE_read( u16 eeAddress, char *data, u08 length )
{
	// Lee en la EE, desde la posicion 'address', 'length' bytes
	// y los deja en el buffer apuntado por 'data'

size_t xReturn = 0U;
u16 val = 0;
u08 xBytes = 0;

		// Lo primero es obtener el semaforo
		FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
		// Luego indicamos el periferico i2c en el cual queremos leer
		val = EE_ADDR;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
		// Luego indicamos la direccion desde donde leer: largo ( 2 bytes )
		val = 2;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
		// y direccion
		val = eeAddress;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

		// Por ultimo leemos.
		xBytes = length;
		xReturn = FreeRTOS_read(&pdI2C, data, xBytes);
		// Y libero el semaforo.
		FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

		if (xReturn != xBytes ) {
			return ( FALSE );
		}

		return(TRUE);

}
//------------------------------------------------------------------------------------
s08 EE_write( u16 eeAddress, char *data, u08 length )
{
	// Escribe en la EE a partir de la posicion 'address', la cantidad
	// 'length' de bytes apuntados por 'data'
	// Puedo estar escribiendo un pageWrite entonces debo controlar no
	// salime de la pagina.
	//
size_t xReturn = 0U;
u16 val = 0;
u08 xBytes = 0;
u16 n, pageBytesFree;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = EE_ADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	// Luego indicamos la direccion a partir de donde escribir: largo ( 2 bytes )
	val = 2;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion
	val = eeAddress;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

	// Por ultimo escribimos xBytes.
	// Controlo no hacer page overflow
	xBytes = length;
	n = length % EE_PAGESIZE;
	pageBytesFree = (n+1)*EE_PAGESIZE - length;
	if ( pageBytesFree < length ) {
		xBytes = pageBytesFree;
	}
	xReturn = FreeRTOS_write(&pdI2C, data, xBytes);
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( FALSE );
	}

	return(TRUE);

}
//------------------------------------------------------------------------------------
