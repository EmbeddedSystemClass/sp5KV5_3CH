/*
 * sp5KV5_3CH_tkConsignaNone.c
 *
 *  Created on: 10 de jun. de 2016
 *      Author: pablo
 */

#include "../sp5KV5_3CH.h"
#include "sp5KV5_3CH_tkConsignas.h"

#ifdef CONSIGNA
// ------------------------------------------------------------------------------------
void sm_CONSIGNAOFF(void)
{
	// Espero indefinidamente.
	 vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

}
// ------------------------------------------------------------------------------------

#endif

