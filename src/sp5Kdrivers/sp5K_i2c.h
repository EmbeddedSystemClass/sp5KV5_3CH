/*
 * sp5K_i2c.h
 *
 *  Created on: 18/10/2015
 *      Author: pablo
 */

#ifndef SRC_SP5KDRIVERS_SP5K_I2C_H_
#define SRC_SP5KDRIVERS_SP5K_I2C_H_

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/twi.h>

#include "avrlibdefs.h"
#include "avrlibtypes.h"
#include "global.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "FRTOS-IO.h"

#define SCL		0
#define SDA		1
#define I2C_MAXTRIES	5

#define ACK 1
#define NACK 0

void i2c_init(void);
s08 I2C_masterWrite ( const u08 devAddress, const u08 devAddressLength, const u16 byteAddress, char *pvBuffer, size_t xBytes );
s08 I2C_masterRead  ( const u08 devAddress, const u08 devAddressLength, const u16 byteAddress, char *pvBuffer, size_t xBytes );

#endif /* SRC_SP5KDRIVERS_SP5K_I2C_H_ */
