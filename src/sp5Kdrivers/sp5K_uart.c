/*
 * sp5K_uart.c
 *
 *  Created on: 4/10/2015
 *      Author: pablo
 *
 *  TRASMISION:
 *  El mecanismo elegido por eficiencia es tener una queue en la cual se escriben los
 *  caracteres a trasmitir y se habilita la interrupcion.
 *  Esta, saca los caracteres y los va enviando a la UART.
 *  Esto hace, que la rutina de interrupcion deba saber a priori cual es la cola de trasmision.
 *  Como es una ISR, no se le puede pasar como parametro.
 *  Por otro lado, no  importaria mucho tenerlo definido de antemano ya que se trata de un
 *  sistema embebido.
 */


#include "sp5K_uart.h"
#include "FRTOS-IO.h"

/*------------------------------------------------------------------------------------*/
void pvUARTInit( const int UARTx )
{

u16 bauddiv;

//	uartsHWEnable();	// Habilito el buffer L365.

	portENTER_CRITICAL();

	bauddiv = 8;
	//
	switch ( UARTx ) {
	case pUART0:
		outb(UBRR0L, bauddiv);
		outb(UBRR0H, bauddiv>>8);
		// Enable the Rx interrupt.  The Tx interrupt will get enabled
		// later. Also enable the Rx and Tx.
		// enable RxD/TxD and interrupts
		outb(UCSR0B, BV(RXCIE0)| BV(RXEN0)|BV(TXEN0));
		// Set the data bits to 8N1.
		UCSR0C = ( serUCSRC_SELECT | serEIGHT_DATA_BITS );
		outb(UCSR1A, BV(U2X0));
		break;
	case pUART1:
		outb(UBRR1L, bauddiv);
		outb(UBRR1H, bauddiv>>8);
		outb(UCSR1B, BV(RXCIE1)| BV(RXEN1)|BV(TXEN1));
		UCSR1C = ( serUCSRC_SELECT | serEIGHT_DATA_BITS );
		outb(UCSR1A, BV(U2X1));
		break;
	}

	portEXIT_CRITICAL();

	return;
}
/*------------------------------------------------------------------------------------*/
void vUartInterruptOn(int UARTx)
{
	unsigned portCHAR ucByte;

	// Habilito la interrupcion del UART lo que hace que se ejecute la ISR_TX y
	// esta vaya a la TXqueue y si hay datos los comienze a trasmitir.

	switch(UARTx) {
	case pUART0:
		ucByte = UCSR0B;
		ucByte |= serTX_INT_ENABLE;
		UCSR0B = ucByte;
		break;
	case pUART1:
		ucByte = UCSR1B;
		ucByte |= serTX_INT_ENABLE;
		UCSR1B = ucByte;
		break;
	}

}
/*------------------------------------------------------------------------------------*/
void vUartInterruptOff(int UARTx)
{
	unsigned portCHAR ucByte;

	// Habilito la interrupcion del UART lo que hace que se ejecute la ISR_TX y
	// esta vaya a la TXqueue y si hay datos los comienze a trasmitir.

	switch(UARTx) {
	case pUART0:
		ucByte = UCSR0B;
		ucByte &= ~serTX_INT_ENABLE;
		UCSR0B = ucByte;
		break;
	case pUART1:
		ucByte = UCSR1B;
		ucByte &= ~serTX_INT_ENABLE;
		UCSR1B = ucByte;
		break;
	}

}
/*------------------------------------------------------------------------------------*/
// FIFO: FUNCIONES PRIVADAS DE MANEJO DE FIFOS.
// QUEUE: Son provistas por el FreeRTOS.
/*------------------------------------------------------------------------------------*/
fifoHandle_t xFifoCreate ( const u16 length,int flags  )
{
fifo_handle_s *pxNewFifo;
fifoHandle_t xReturn = NULL;
int8_t *pcAllocatedBuffer;
u08 *dataBuffer;

	// Aloco el espacio para el buffer de datos.
	dataBuffer = ( u08 * ) pvPortMalloc( length + 1);

	// Aloco espacio para la estructura.
	pcAllocatedBuffer = ( int8_t * ) pvPortMalloc( sizeof(fifo_handle_s ));
	if( pcAllocatedBuffer != NULL )
	{
		pxNewFifo = ( fifoHandle_t * ) pcAllocatedBuffer;
		pxNewFifo->head = 0;	// start
		pxNewFifo->tail = 0;	// end
		pxNewFifo->uxMessageWaiting = 0;
		pxNewFifo->buff = dataBuffer;
		pxNewFifo->length = length;		// REVISAR

		xReturn = pxNewFifo;
	}

	return(xReturn);

}
/*------------------------------------------------------------------------------------*/
int xFifoReset( fifoHandle_t xFifo )
{
fifo_handle_s *pxFifo;

	taskENTER_CRITICAL();
	pxFifo = xFifo;
	pxFifo->head = 0;
	pxFifo->tail = 0;
	pxFifo->uxMessageWaiting = 0;
	memset(pxFifo->buff,0, pxFifo->length );	// REVISAR
	taskEXIT_CRITICAL();
	return(0);
}
/*------------------------------------------------------------------------------------*/
BaseType_t xFifoSend( fifoHandle_t xFifo,const char *cChar, TickType_t xTicksToWait )
{
	// Si el buffer esta lleno, descarto el valor recibido
	// Guardo en el lugar apuntado por head y avanzo.

fifo_handle_s *pxFifo = xFifo;
s08 ret = errQUEUE_FULL;

	taskENTER_CRITICAL();
	// Si el buffer esta vacio ajusto los punteros
	if( pxFifo->uxMessageWaiting == 0) {
		pxFifo->head = pxFifo->tail = 0;
	}

	if ( pxFifo->uxMessageWaiting < pxFifo->length ) {
		pxFifo->buff[pxFifo->head] = *cChar;
		++pxFifo->uxMessageWaiting;
		// Avanzo en modo circular
		pxFifo->head = ( pxFifo->head  + 1 ) % ( pxFifo->length );
		ret = pdTRUE;
    }
	taskEXIT_CRITICAL();
	return(ret);

}
/*------------------------------------------------------------------------------------*/
BaseType_t xFifoSendFromISR( fifoHandle_t xFifo,const char *cChar, TickType_t xTicksToWait )
{
	// Si el buffer esta lleno, descarto el valor recibido
	// Guardo en el lugar apuntado por head y avanzo.

fifo_handle_s *pxFifo = xFifo;
s08 ret = errQUEUE_FULL;

	// Si el buffer esta vacio ajusto los punteros !!!
	if( pxFifo->uxMessageWaiting == 0) {
		pxFifo->head = pxFifo->tail = 0;
	}

	if ( pxFifo->uxMessageWaiting < pxFifo->length ) {
		pxFifo->buff[pxFifo->head] = *cChar;
		++pxFifo->uxMessageWaiting;
		// Avanzo en modo circular
		pxFifo->head = ( pxFifo->head  + 1 ) % ( pxFifo->length );
		ret = pdTRUE;
    }
	return(ret);

}
/*------------------------------------------------------------------------------------*/
BaseType_t xFifoReceive( fifoHandle_t xFifo, char *cChar, TickType_t xTicksToWait )
{

fifo_handle_s *pxFifo = xFifo;
s08 ret = pdFALSE;

	taskENTER_CRITICAL();
	//  Si el buffer esta vacio retorno.
	if( pxFifo->uxMessageWaiting == 0) {
		//pxFifo->head = pxFifo->tail = 0;
		taskEXIT_CRITICAL();
		return(ret);
	}

	*cChar = pxFifo->buff[pxFifo->tail];
	--pxFifo->uxMessageWaiting;
	// Avanzo en modo circular
	pxFifo->tail = ( pxFifo->tail  + 1 ) % ( pxFifo->length );
	ret = pdTRUE;
	taskEXIT_CRITICAL();

	return(ret);
}
/*------------------------------------------------------------------------------------*/
BaseType_t xFifoReceiveFromISR( fifoHandle_t xFifo, char *cChar, TickType_t xTicksToWait )
{

fifo_handle_s *pxFifo = xFifo;
s08 ret = pdFALSE;

	// Cannot block in an ISR

	//  Si el buffer esta vacio retorno.
	if( pxFifo->uxMessageWaiting == 0) {
		pxFifo->head = pxFifo->tail = 0;
		return(ret);
	}

	*cChar = pxFifo->buff[pxFifo->tail];
	--pxFifo->uxMessageWaiting;
	// Avanzo en modo circular
	pxFifo->tail = ( pxFifo->tail  + 1 ) % ( pxFifo->length );
	ret = pdTRUE;

	return(ret);
}
/*------------------------------------------------------------------------------------*/
// UART ISR:
/*------------------------------------------------------------------------------------*/
ISR( USART0_UDRE_vect )
{
/* Handler (ISR) de TX0.
 * El handler maneja la trasmision de datos por la uart0.
 * Para trasmitir, usando la funcion xUart0PutChar se encolan los datos y prende la
 * flag de interrupcion.
 * La rutina de interrupcion ejecuta este handler (SIGNAL) en el cual si hay
 * datos en la cola los extrae y los trasmite.
 * Si la cola esta vacia (luego del ultimo) apaga la interrupcion.
*/

s08 cTaskWoken;
char cChar;
s08 res = pdFALSE;
UART_device_control_t *pUart;

	pUart = pdUART0.phDevice;

	if ( pUart->txBufferType == QUEUE ) {
		res = xQueueReceiveFromISR( pUart->txStruct, &cChar, &cTaskWoken );
	} else {
		res = xFifoReceiveFromISR( pUart->txStruct, &cChar, 0 );
	}

	if( res == pdTRUE ) {
		/* Send the next character queued for Tx. */
		outb (UDR0, cChar);
	} else {
		/* Queue empty, nothing to send. */
		vUartInterruptOff(pUART0);
	}
}
/*------------------------------------------------------------------------------------*/
ISR( USART1_UDRE_vect )
{
	// Handler (ISR) de TX.
s08 cTaskWoken;
char cChar;
s08 res  = pdFALSE;
UART_device_control_t *pUart;

	pUart = pdUART1.phDevice;

	if ( pUart->txBufferType == QUEUE ) {
		res = xQueueReceiveFromISR( pUart->txStruct, &cChar, &cTaskWoken );
	} else {
		res = xFifoReceiveFromISR( pUart->txStruct, &cChar, 0 );
	}

	if( res == pdTRUE ) {
		/* Send the next character queued for Tx. */
		outb (UDR1, cChar);
	} else {
		/* Queue empty, nothing to send. */
		vUartInterruptOff(pUART1);
	}
}
/*------------------------------------------------------------------------------------*/
ISR( USART0_RX_vect )
{
/* Handler (ISR) de RX0.
 * Este handler se encarga de la recepcion de datos.
 * Cuando llega algun datos por el puerto serial lo recibe este handler y lo va
 * guardando en la cola de recepcion
*/

char cChar;
UART_device_control_t *pUart;

	pUart = pdUART0.phDevice;

	/* Get the character and post it on the queue of Rxed characters.
	 * If the post causes a task to wake force a context switch as the woken task
	 * may have a higher priority than the task we have interrupted.
    */
	cChar = inb(UDR0);

	if ( pUart->rxBufferType == QUEUE ) {
		if( xQueueSendFromISR( pUart->rxStruct, &cChar, 0 ) ) {
			taskYIELD();
		}
	} else {
		if( xFifoSendFromISR( pUart->rxStruct, &cChar, 0 ) ) {
			taskYIELD();
		}
	}
}
/*------------------------------------------------------------------------------------*/
ISR( USART1_RX_vect )
{
/* Handler (ISR) de RX1.
 * Este handler se encarga de la recepcion de datos.
 * Cuando llega algun datos por el puerto serial lo recibe este handler y lo va
 * guardando en la cola de recepcion
*/

char cChar;
UART_device_control_t *pUart;

	pUart = pdUART1.phDevice;

	/* Get the character and post it on the queue of Rxed characters.
	 * If the post causes a task to wake force a context switch as the woken task
	 * may have a higher priority than the task we have interrupted.
    */
	cChar = inb(UDR1);

	if ( pUart->rxBufferType == QUEUE ) {
		if( xQueueSendFromISR( pUart->rxStruct, &cChar, pdFALSE ) ) {
			taskYIELD();
		}
	} else {
		if( xFifoSendFromISR( pUart->rxStruct, &cChar, pdFALSE ) ) {
			taskYIELD();
		}
	}
}
/*------------------------------------------------------------------------------------*/
size_t uxFifoSpacesAvailable( fifoHandle_t xFifo )
{

fifo_handle_s *pxFifo = xFifo;

	return( pxFifo->length - pxFifo->uxMessageWaiting);
}
/*------------------------------------------------------------------------------------*/
size_t uxFifoMessagesWaiting( fifoHandle_t xFifo )
{

fifo_handle_s *pxFifo = xFifo;

	return(pxFifo->uxMessageWaiting);
}
/*------------------------------------------------------------------------------------*/
