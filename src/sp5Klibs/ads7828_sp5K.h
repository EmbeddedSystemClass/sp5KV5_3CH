/*
 * ads7828_sp5KFRTOS.h
 *
 *  Created on: 15/4/2015
 *      Author: pablo
 */

#ifndef SRC_SP5KLIBS_ADS7828_SP5K_H_
#define SRC_SP5KLIBS_ADS7828_SP5K_H_

#include "FRTOS-IO.h"
#include "sp5K_i2c.h"

#define 	ADS7828_ADDR			0x90

#define 	ADS7828_CMD_SD   		0x80	//ADS7828 Single-ended/Differential Select bit.

#define 	ADS7828_CMD_PDMODE0		0x00	//ADS7828 Mode 0: power down
#define 	ADS7828_CMD_PDMODE1 	0x04	//ADS7828 Mode 1: Ref OFF, converter ON
#define 	ADS7828_CMD_PDMODE2   	0x08	//ADS7828 Mode 2: Ref ON, converter OFF
#define 	ADS7828_CMD_PDMODE3   	0x0C	//ADS7828 Mode 3: Ref ON, converter ON.

s08 ADS7828_read(u08 channel, u16 *value);
#define ADS7827_readCh0( value ) ( ADS7828_read(3, value))
#define ADS7827_readCh1( value ) ( ADS7828_read(5, value))
#define ADS7827_readCh2( value ) ( ADS7828_read(7, value))
#define ADS7827_readBatt( value ) ( ADS7828_read(1, value))

#endif /* SRC_SP5KLIBS_ADS7828_SP5K_H_ */
