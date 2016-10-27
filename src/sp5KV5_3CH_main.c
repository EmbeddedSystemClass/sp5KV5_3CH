/*
 *
 * git commit -a -m "beta 473293390 001"
 *
 * git remote add origin https://github.com/ppeluffo/sp5KV5_3CH.git
 * git push -u origin master

 * git remote add REM_SP5KV4 https://github.com/ppeluffo/sp5KV4.git
 * git push -u REM_SP5KV4 master
 * -------------------------------------------------------------------------------------------------------
 * VERSION 5.0.2 @ 2016-09-23:
 * ---------------------------
 * 1- Pongo un define SERIAL que se usa en conjunto con el define UTE_8CH.
 * Lo que hace es que cuando el log = OFF, igual loguea los frames relevados y les pone
 * un tag para que sean reconocidos por el software del server.
 * Por otro lado, el modem lo deja siempre en estado APAGADO.
 * Tampoco guardo en memoria.
 *
 * VERSION 5.0.1 @ 2016-09-08: POZOS
 * ---------------------------------
 * Los sensores miden hasta 700cms, y los pulsos son de 58us/cms, por lo tanto el ancho del pulso
 * en los 7mts es de 40600us, 2 bytes unsigned.
 * El bug se genera porque en la tkRange, usamos un s16, por lo tanto quedan solo valores de +/- 32768.
 * En tkRange::pv_ping elimino el control de maxRange y modifico para pasar la distancia por parametro
 * y retornar el status de la medida.
 * En globalStatus reporto todos los errores:
 *
 * VERSION 5.0.0 @ 2016-08-08:
 * ---------------------------
 * 1- Arreglo es estado de modem apagado ya que contiene bugs.
 * 2- En caso que se activa el tilt, se avisa a la tarea de GPRS por un mensaje
 *
 * Configurar consigna continua en GPRS.
 *
 * VERSION 5.0.0 @ 2016-08-29:
 * ---------------------------
 * 1- Defino una funcion logprint para mejorar el entendimiento de los mensajes.
 * 2- Configuro el ID del Bluetooth con el dlgId cuando arranco
 * 3- Arreglo bug en tkGprs.modemApagado
 * 4- Arreglo que la consigna nocturna pueda se anterior a la diurna
 * 5- El server me puede mandar resetear en cualquier momento ( en continuo sobre todo )
 * 6- Agego al frame trasmitido la bateria ( bug )
 * 7- El mismo firmware base lo uso para POZOS,OSE_3CH,UTE_8CH,CONSIGNAS usando #defines
 * 8- OSE_3CH: los canales digitales solo cuentan pulsos, no miden niveles ya que el circuito
 *    de la placa analogica esta hecho para esto, por lo que elimino todo lo referente a niveles
 *    de este firmware.
 * 9- POZOS: Cuentan pulsos y niveles ya que el circuito lo permite.
 * -------------------------------------------------------------------------------------------------------
 * V5.0.0
 * Al haber encontrado el problema del sistema en que la fuente de 3.6 caia al trasmitir y por eso
 * reseteaba el MCP de la placa superior, en esta version vamos a hacer 3 cosas importantes:
 *
 * * PwrSave
 * * Se excede en tiempo con el lote. Revisar el server
 * * Luego de excederse con el lote, espera solo 15s con el modem apagado
 *
 *  1- Al configurar la consigna continua, chequea que este en modo pwrMode continuo.
 *     En la configuracion por GPRS debemos cuidar primero configurar el pwrMode.
 *
 * WATCHDOG:
 * Para hacer un mejor seguimiento de las fallas, agrego a c/estado un nro.
 * Por otro lado, el WDG lo manejo en modo interrupcion / reset de modo que ante
 * un problema, la interrupcion guarda el estado y luego se resetea.
 * Al arrancar, leo el estado y lo trasmito.
 *
 * Agrego en la funcion de espera de I2C un timeout de modo que salgo con FALSE y eso hace
 * que el resto de las funciones indique un error.
 * Esto en ppio. podria evitar un error de reset por wdg.
 *
 * !! Agregar el salir automaticamente luego de 30 mins del modo service.
 *
 * V4.1.3:
 * - Agrego en el frame de datos un campo que indique la calidad del frame.
 *   Queda del tipo:
 *   CTL=106&ST=0&LINE=20140607,083358,pA>62.12,pB>62.12,pc>4.23,q0>103.28,v1>22.4,bt>11.29
 *
 * V4.1.2:
 * - Modifico la inicializacion de WDT siguiendo las recomendaciones de Atmel y verifico el
 *   funcionamiento de c/wdg.
 *   http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
 *   http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
 * - El WDT se prende por fuses grabandolos en 0xFF(low), 0xC9(high), 0xFD(extended)
 *   Esto me asegura que no se van a borrar por hw.
 * - Borro todo lo referente a DCD ya que no lo uso.
 * - Agrego a tkAnalog una funcion que chequea la consistencia de los timers en c/ciclo y si estan
 *   mal resetea al uC
 * - Idem en tkGprs.
 * - Tambien controlo no exceder 12hs sin discar.
 * - Elimino el control del pin de la terminal de tkDigital y lo dejo todo en tkControl.
 * - En tkControl agrego la rutina pv_modemControl para controlar que el modem no quede
 *   prendido mas de 10mins. Con esto si alguna rutina queda en loop, no apagaria el modem y
 *   podriamos agarrar el problema.
 *
 * V4.0.9:
 * El problema es que el ADC no se prende y la lectura de los canales es erronea.
 * La razon es que el MCP se resetea a default y por lo tanto no prende los sensores ni los 3.3V.
 * Esto es porque el pigtail del modem irradia energia e introduce un pulso que resetea al MCP.
 * Lo resolvemos en 3 flancos diferentes:
 * 1- En la rutina MCP_testAndSet ( con la que prendemos los 3.3V ), verificamos que el MCP este configurado
 * y sino lo reconfiguramos.
 * 2- Hay veces que esto no es suficiente ya que entre tr02 y tr06, si se resetea el MCP no nos damos
 * cuenta hasta que leemos el ADC.
 * Entonces en tr02 mandamos un mensaje a tkGPRS que no trasmita, y en tr06 que puede trasmitir.
 * Con esto controlo no irradiar energia que resetee al MCP mientras estoy poleando ( 15s ).
 * En tkGPRS lo chequeo antes de perdir una IP y antes de abrir un socket.
 * 3- En caso que nada funcione, en el modulo tkAnalog si una medida me da error ( lectura del ADC ), descarto
 * la medida y la sustituyo por el valor anterior.
 * Como la medida de los sensores se hace en 4 pasos, si el primero ( tr05) da error ( por estar apagado el ADC ),
 * puedo corregirlo y prenderlo.
 *
 * V4.1.0:
 * Modifico el manejo de consignas para incorporar una FSM
 * El mismo problema de desconfiguracion puede ocurrir al setear las consignas.
 * Para esto entonces fijo que para setear las consignas el modem deba estar apagado.
 * La tarea tkConsignas consulta el estado del modem con u_modemPwrStatus()
 *
 * ----------------------------------------------------------------------------------------------------------------
 */

#include "sp5KV5_3CH.h"

static void pv_initMPU(void);

//----------------------------------------------------------------------------------------
// http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
// http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
//
// Function Pototype
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

// Function Implementation
void wdt_init(void)
{
	// Leo el MCUSR para saber cual fue la causa del reset
	mcusr_mirror = MCUSR;
	// e inmediatamente lo pongo en 0 por si se resetea saber la causa
    MCUSR = 0;
    // y deshabilito el wdg para dejar que el micro arranque y no quede en un loop de resets
    //  wdt_disable();
    // Como los fusibles estan para que el WDG siempre este prendido, lo reconfiguro a 8s lo
    // antes posible
    wdt_enable(WDTO_8S);
    return;
}
//------------------------------------------------------------------------------------

int main(void)
{
unsigned int i,j;

	//----------------------------------------------------------------------------------------
	// Rutina NECESARIA para que al retornar de un reset por WDG no quede en infinitos resets.
	// Copiado de la hoja de datos.

	wdgStatus.resetCause = mcusr_mirror;
	// Genero un delay de 1s. para permitir que el micro se estabilize.
	// Lleva tiempo a los osciladores estabilizarse.
	for (i=0; i<1000; i++)
		for (j=0; j<1000; j++)
				;

	//----------------------------------------------------------------------------------------

	wdt_reset();
	pv_initMPU();
	FreeRTOS_open(pUART0, ( UART_RXFIFO + UART_TXQUEUE ));
	FreeRTOS_open(pUART1, ( UART_RXFIFO + UART_TXQUEUE ));
	FreeRTOS_open(pI2C, 0);

	/* Arranco el RTOS. */
	startTask = FALSE;

	// Creo los semaforos
	sem_SYSVars = xSemaphoreCreateMutex();

	// Inicializacion de modulos de las tareas que deben hacerce antes
	// de arrancar el FRTOS
	tkControlInit();

#ifdef OSE_3CH
	tkAnalogInit();
#endif

#ifdef UTE_8CH
	tkAnalogInit();
#endif

#ifdef OSE_POZOS
	tkRangeInit();
#endif

	tkGprsInit();

	// Creo las tasks
	xTaskCreate(tkCmd, "CMD", tkCmd_STACK_SIZE, NULL, tkCmd_TASK_PRIORITY,  &xHandle_tkCmd);
	xTaskCreate(tkControl, "CTL", tkControl_STACK_SIZE, NULL, tkControl_TASK_PRIORITY,  &xHandle_tkControl);
	xTaskCreate(tkGprsTx, "GPTX", tkGprsTx_STACK_SIZE, NULL, tkGprsTx_TASK_PRIORITY,  &xHandle_tkGprsTx);
	xTaskCreate(tkGprsRx, "GPRX", tkGprsRx_STACK_SIZE, NULL, tkGprsRx_TASK_PRIORITY,  &xHandle_tkGprsRx);

#ifdef OSE_3CH
	xTaskCreate(tkDigitalIn, "DIN", tkDigitalIn_STACK_SIZE, NULL, tkDigitalIn_TASK_PRIORITY,  &xHandle_tkDigitalIn);
	xTaskCreate(tkAnalogIn, "AIN", tkAIn_STACK_SIZE, NULL, tkAIn_TASK_PRIORITY,  &xHandle_tkAIn);
#endif

#ifdef UTE_8CH
	xTaskCreate(tkDigitalIn, "DIN", tkDigitalIn_STACK_SIZE, NULL, tkDigitalIn_TASK_PRIORITY,  &xHandle_tkDigitalIn);
	xTaskCreate(tkAnalogIn, "AIN", tkAIn_STACK_SIZE, NULL, tkAIn_TASK_PRIORITY,  &xHandle_tkAIn);
#endif

#ifdef CONSIGNA
	xTaskCreate(tkConsignas, "CONS", tkCons_STACK_SIZE, NULL, tkCons_TASK_PRIORITY,  &xHandle_tkConsignas);
#endif

#ifdef OSE_POZOS
	xTaskCreate(tkRange, "RANGE", tkRange_STACK_SIZE, NULL, tkRange_TASK_PRIORITY,  &xHandle_tkRange );
#endif

	systemWdg = WDG_CTL + WDG_CMD + WDG_GPRSTX + WDG_GPRSRX;

#ifdef CONSIGNA
	systemWdg += WDG_CSG;
#endif

#ifdef OSE_3CH
	systemWdg += WDG_AIN + WDG_DIN;
#endif

#ifdef UTE_8CH
	systemWdg += WDG_AIN + WDG_DIN;
#endif

#ifdef OSE_POZOS
	systemWdg += WDG_RANGE;
#endif
	/* Arranco el RTOS. */
	vTaskStartScheduler();

	// En caso de panico, aqui terminamos.
	exit (1);
	//return 0;
}
/*------------------------------------------------------------------------------------*/
static void pv_initMPU(void)
{
	// Son acciones que se hacen antes de arrancar el RTOS


	// Configuracion de pines:
	// El pin de control de la terminal es entrada
	cbi(TERMSW_DDR, TERMSW_BIT);
	// El pin de DCD es entrada
	cbi(DCD_DDR, DCD_BIT);
	// Leds
	sbi(LED_KA_DDR, LED_KA_BIT);		// El pin del led de KA ( PD6 ) es una salida.
	sbi(LED_MODEM_DDR, LED_MODEM_BIT);	// El pin del led de KA ( PD6 ) es una salida.
	// inicialmente los led quedan en 0


#ifdef OSE_POZOS
	// RANGE METER
	cbi(RM_PW_DDR, RM_PW_BIT);		// PW es entrada
	cbi(RM_DIN0_DDR, RM_DIN0_BIT);	// DIN0 es entrada
	cbi(RM_DIN1_DDR, RM_DIN1_BIT);	// DIN1 es entrada

	sbi(RM_RUN_DDR, RM_RUN_BIT);	// RUN es salida
#endif

#if defined(OSE_3CH) || defined(OSE_POZOS)
	sbi(LED_KA_PORT, LED_KA_BIT);
	sbi(LED_MODEM_PORT, LED_MODEM_BIT);

	// Los pines del micro que resetean los latches de caudal son salidas.
	sbi(Q_DDR, Q0_CTL_PIN);
	sbi(Q_DDR, Q1_CTL_PIN);
#endif

#ifdef UTE_8CH
	// Leds de placa superior
	cbi(LED_KA_PORT, LED_KA_BIT);
	cbi(LED_MODEM_PORT, LED_MODEM_BIT);
	//
	// Pines de entradas digitales.
	// Entradas de pulso(LATCHs)
	cbi(D0_IN_DDR, D0_IN);
	cbi(D1_IN_DDR, D1_IN);
	cbi(D2_IN_DDR, D2_IN);
	cbi(D3_IN_DDR, D3_IN);
	//
	// Entradas de nivel logico
	cbi(D0_LV_DDR, D0_LV);
	cbi(D1_LV_DDR, D1_LV);
	cbi(D2_LV_DDR, D2_LV);
	cbi(D3_LV_DDR, D3_LV);
	//
	// Salidas para resetear los latches
	sbi(D0_CLR_DDR, D0_CLR);
	sbi(D1_CLR_DDR, D1_CLR);
	sbi(D2_CLR_DDR, D2_CLR);
	sbi(D3_CLR_DDR, D3_CLR);

#endif

	// Configuro el modo de Sleep.
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
}
/*------------------------------------------------------------------------------------*/
void vApplicationIdleHook( void )
{

	for(;;) {

//		vCoRoutineSchedule();
		if ( ( u_terminalPwrStatus() == T_APAGADA ) && ( systemVars.pwrMode == PWR_DISCRETO)) {
			sleep_mode();
		}
	}

}
/*------------------------------------------------------------------------------------*/
