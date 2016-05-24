/*
 * FRTOS-IO.h
 *
 *  Created on: 2/10/2015
 *      Author: pablo
 */

#ifndef SRC_FRTOS_IO_FRTOS_IO_H_
#define SRC_FRTOS_IO_FRTOS_IO_H_

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <../sp5Klibs/avrlibdefs.h>
#include <../sp5Klibs/avrlibtypes.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "sp5K_uart.h"

typedef enum {
		pUART0 = 0,
		pUART1,
		pI2C
} t_perifericos;

//#define DEBUG_I2C

/* Peripheral handles are void * for data hiding purposes. */
typedef const void * Peripheral_Descriptor_t;

/* Types that define valid read(), write() and ioctl() functions. */
typedef size_t ( *Peripheral_write_Function_t )( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );
typedef size_t ( *Peripheral_read_Function_t )( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );
typedef portBASE_TYPE ( *Peripheral_ioctl_Function_t )( Peripheral_Descriptor_t const pxPeripheral, uint32_t ulRequest, void *pvValue );

//------------------------------------------------------------------------------------
// Estructura generica de todos los perifericos.
// Los elementos particulares quedan en una estructura que se accede por el punter
// phDevice

typedef struct {

	t_perifericos	portId;
	SemaphoreHandle_t xBusSemaphore;		//
	u08 xBlockTime;							// ticks to block in read operations. Set by ioctl

	Peripheral_write_Function_t write;		// The function used to write to the peripheral
	Peripheral_read_Function_t read;		// The function used to read from the peripheral
	Peripheral_ioctl_Function_t ioctl;		// The function used for ioctl access to the peripheral

	void *phDevice;		// puntero a una estructura con los miembros adecuados al periferico

} Peripheral_Control_t;

Peripheral_Control_t pdUART0, pdUART1, pdI2C;

Peripheral_Descriptor_t FreeRTOS_open(const u08 port, const u32 flags);
int FreeRTOS_ioctl( Peripheral_Descriptor_t const xPeripheral, uint32_t ulRequest, void *pvValue );
#define FreeRTOS_write( xPeripheral, pvBuffer, xBytes ) ( ( Peripheral_Control_t * ) xPeripheral )->write( ( ( Peripheral_Control_t * ) xPeripheral ), ( pvBuffer ), ( xBytes ) )
#define FreeRTOS_read( xPeripheral, pvBuffer, xBytes ) ( ( Peripheral_Control_t * ) xPeripheral )->read( ( ( Peripheral_Control_t * ) xPeripheral ), ( pvBuffer ), ( xBytes ) )

//------------------------------------------------------------------------------------
// Estructura particular de UARTs
//------------------------------------------------------------------------------------
#define FIFO	0
#define QUEUE	1
//-----------------------------------------------------------------------
// Defino el tipo de almacenamiento que requiere la aplicacion
// 0: queue, 1: fifo
#define UART_RXFIFO		1
#define UART_TXFIFO		2
#define UART_RXQUEUE	4
#define UART_TXQUEUE	8
//-----------------------------------------------------------------------
// Defino el tamanio de los buffers
// La UART0 es la que tiene el modem
#define  UART0_RXBUFFER_LEN ( ( u16 ) ( 640 ))
#define  UART0_TXBUFFER_LEN ( ( u08 ) ( 128 ))

// La UART1 es la de la terminal RS232
#define  UART1_RXBUFFER_LEN ( ( u08 ) ( 128 ))
#define  UART1_TXBUFFER_LEN ( ( u08 ) ( 128 ))

typedef struct {

	// En las UART solo requiero un estructura de almacenamiento de datos.
	u08 rxBufferType;
	void *rxStruct;			// uso puntero void para poder asignarle cualquier estructura.
	u16 rxBufferLength;

	u08 txBufferType;
	void *txStruct;
	u16 txBufferLength;

} UART_device_control_t;

//------------------------------------------------------------------------------------

int FreeRTOS_UART_open( Peripheral_Control_t * const pxPeripheralControl, const u32 flags );
size_t FreeRTOS_UART_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );
size_t FreeRTOS_UART_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );
portBASE_TYPE FreeRTOS_UART_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue );
void pvFreeRTOS_UART1_writeChar (unsigned char c);

char *FreeRTOS_UART_getFifoPtr(Peripheral_Control_t *UART);

//------------------------------------------------------------------------------------
// Estructura particular de I2Cs
//------------------------------------------------------------------------------------

typedef struct {

	// En las UART solo requiero un estructura de almacenamiento de datos.
	u08 devAddress;
	u08 byteAddressLength;
	u16 byteAddress;

} I2C_device_control_t;

portBASE_TYPE FreeRTOS_I2C_open( Peripheral_Control_t * const pxPeripheralControl, const u32 flags );
size_t FreeRTOS_I2C_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );
size_t FreeRTOS_I2C_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );
portBASE_TYPE FreeRTOS_I2C_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue );

//------------------------------------------------------------------------------------

#define ioctlQUEUE_FLUSH			1
#define ioctlUART_ENABLE			2
#define ioctlUART_DISABLE			3
#define ioctlOBTAIN_BUS_SEMPH		4
#define ioctlRELEASE_BUS_SEMPH		5

#define ioctlSET_TIMEOUT			6
#define ioctl_UART_CLEAR_RX_BUFFER	7
#define ioctl_UART_CLEAR_TX_BUFFER	8

#define ioctl_I2C_SET_DEVADDRESS			10
#define ioctl_I2C_SET_BYTEADDRESS			11
#define ioctl_I2C_SET_BYTEADDRESSLENGTH		12


#endif /* SRC_FRTOS_IO_FRTOS_IO_H_ */
