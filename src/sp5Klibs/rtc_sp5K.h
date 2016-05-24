/*------------------------------------------------------------------------------------
 * rtc_sp5KFRTOS.h
 * Autor: Pablo Peluffo @ 2015
 * Basado en Proycon AVRLIB de Pascal Stang.
 *
 * Son funciones que impelementan la API de acceso al RTC del sistema SP5K con FRTOS.
 * Para su uso debe estar inicializado el semaforo del bus I2C, que se hace llamando a i2cInit().
 *
 *
*/

#ifndef AVRLIBFRTOS_RTC_SP5KFRTOS_H_
#define AVRLIBFRTOS_RTC_SP5KFRTOS_H_

#include "FRTOS-IO.h"
#include "sp5K_i2c.h"

// Direccion del bus I2C donde esta el DS1340
#define RTC_DEVADDR		   	0xD0

typedef struct
{
	// Tamanio: 7 byte.
	// time of day
	u08 sec;
	u08 min;
	u08 hour;
	// date
	u08 day;
	u08 month;
	u16 year;

} RtcTimeType_t;


s08 RTC_read(RtcTimeType_t *rtc);
s08 RTC_write(RtcTimeType_t *rtc);


#endif /* AVRLIBFRTOS_RTC_SP5KFRTOS_H_ */
