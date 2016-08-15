/*
 * sp5KV5_3CH_tkConsignas.h
 *
 *  Created on: 10 de jun. de 2016
 *      Author: pablo
 */

#ifndef SP5KV5_3CH_TKCONSIGNAS_SP5KV5_3CH_TKCONSIGNAS_H_
#define SP5KV5_3CH_TKCONSIGNAS_SP5KV5_3CH_TKCONSIGNAS_H_

char cons_printfBuff[CHAR128];

s08 f_dc_frameReady;

void sm_DOBLECONSIGNA(void);
void sm_CONSIGNAOFF(void);
void sm_CONSIGNACONTINUA(void);

void initDobleConsigna(void);
void initConsignaContinua(void);

void pv_consignaPrintExitMsg(u08 code);

#endif /* SP5KV5_3CH_TKCONSIGNAS_SP5KV5_3CH_TKCONSIGNAS_H_ */
