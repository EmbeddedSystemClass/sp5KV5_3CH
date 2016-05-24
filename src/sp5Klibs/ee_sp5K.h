/*
 * ee_sp5K.h
 *
 *  Created on: 26/10/2015
 *      Author: pablo
 */

#ifndef SRC_SP5KLIBS_EE_SP5K_H_
#define SRC_SP5KLIBS_EE_SP5K_H_

#include "../sp5KV5_3CH.h"
#include "FRTOS-IO.h"

//------------------------------------------------------------------------------------
// Identificacion en el bus I2C
#define EE_ADDR			0xA0
#define EE_PAGESIZE		64

s08 EE_read( u16 eeAddress, char *data, u08 length );
s08 EE_write( u16 eeAddress, char *data, u08 length );

#endif /* SRC_SP5KLIBS_EE_SP5K_H_ */
