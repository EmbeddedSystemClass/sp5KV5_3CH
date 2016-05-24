/*
 * sp5K_uart.h
 *
 *  Created on: 4/10/2015
 *      Author: pablo
 *
 *  Un tema importante es la estructura donde almacenamos los buffers.
 *  Si usamos queues, debemos tener en cuenta que el largo maximo viene dado por
 *  una variable tipo unsigned char que entonces solo almacena 255 valores.
 */

#ifndef SRC_SP5KDRIVERS_SP5K_UART_H_
#define SRC_SP5KDRIVERS_SP5K_UART_H_

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <../sp5Klibs/avrlibdefs.h>
#include <../sp5Klibs/avrlibtypes.h>

#include "FreeRTOS.h"
#include "FRTOS-IO.h"
//-----------------------------------------------------------------------
/* Constants for writing to UCSRB. */
#define serRX_INT_ENABLE				( ( unsigned portCHAR ) 0x80 )
#define serRX_ENABLE					( ( unsigned portCHAR ) 0x10 )
#define serTX_INT_ENABLE				( ( unsigned portCHAR ) 0x20 )
#define serTX_ENABLE					( ( unsigned portCHAR ) 0x08 )

/* Constants for writing to UCSRC. */
#define serUCSRC_SELECT					( ( unsigned portCHAR ) 0x00 )
#define serEIGHT_DATA_BITS				( ( unsigned portCHAR ) 0x06 )

#define UARTCTL_PORT	PORTD
#define UARTCTL_PIN		PIND
#define UARTCTL			4
#define UARTCTL_DDR		DDRD
#define UARTCTL_MASK	0x10

void pvUARTInit( const int UARTx );
void vUartInterruptOn(int UARTx);
void vUartInterruptOff(int UARTx);

typedef void * fifoHandle_t;

typedef struct {
	u08 *buff;
	u16 head;
	u16 tail;
	u16 uxMessageWaiting;
	u16 length;
} fifo_handle_s;

//------------------------------------------------------------------------------------

fifoHandle_t xFifoCreate ( const u16 length, int flags );
BaseType_t xFifoSend( fifoHandle_t xFifo,const char *cChar, TickType_t xTicksToWait );
BaseType_t xFifoSendFromISR( fifoHandle_t xFifo,const char *cChar, TickType_t xTicksToWait );
BaseType_t xFifoReceive( fifoHandle_t xFifo,char *cChar, TickType_t xTicksToWait );
BaseType_t xFifoReceiveFromISR( fifoHandle_t xFifo, char *cChar, TickType_t xTicksToWait );
int xFifoReset( fifoHandle_t xFifo );
size_t uxFifoSpacesAvailable( fifoHandle_t xFifo );
size_t uxFifoMessagesWaiting( fifoHandle_t xFifo );

#endif /* SRC_SP5KDRIVERS_SP5K_UART_H_ */
