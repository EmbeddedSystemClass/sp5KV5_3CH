/*
 * sp5K.h
 *
 * Created on: 27/12/2013
 *      Author: root
 */

#ifndef SP5K_H_
#define SP5K_H_

#include <avr/io.h>			/* include I/O definitions (port names, pin names, etc) */
//#include <avr/signal.h>		/* include "signal" names (interrupt names) */
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <string.h>
#include <compat/deprecated.h>
#include <util/twi.h>
#include <util/delay.h>
#include <ctype.h>
#include <util/delay.h>
#include <avr/cpufunc.h>

#include "sp5Klibs/avrlibdefs.h"
#include "sp5Klibs/avrlibtypes.h"
#include "sp5Klibs/global.h"			// include our global settings
#include "file_sp5K.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "FRTOS-IO.h"

#include "cmdline.h"
#include "sp5K_i2c.h"
#include "sp5K_uart.h"

#include "mcp_sp5K.h"
#include "ads7828_sp5K.h"
#include "rtc_sp5K.h"

// DEFINICION DEL TIPO DE SISTEMA
//----------------------------------------------------------------------------
#define SP5K_REV "5.0.0"
#define SP5K_DATE "@ 20160511"

#define SP5K_MODELO "sp5KV3 HW:avr1284P R5.0"
#define SP5K_VERSION "FW:FRTOS8"

#define CHAR64		64
#define CHAR128	 	128
#define CHAR256	 	256

//----------------------------------------------------------------------------
// TASKS
/* Stack de las tareas */
#define tkCmd_STACK_SIZE		512
#define tkControl_STACK_SIZE	512
#define tkDigitalIn_STACK_SIZE	512
#define tkAIn_STACK_SIZE		512
#define tkGprsTx_STACK_SIZE		512
#define tkGprsRx_STACK_SIZE		512

/* Prioridades de las tareas */
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkControl_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkDigitalIn_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1 )
#define tkAIn_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkGprsTx_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkGprsRx_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )

/* Prototipos de tareas */
void tkCmd(void * pvParameters);
void tkControl(void * pvParameters);
void tkDigitalIn(void * pvParameters);
void tkControlInit(void);
void tkAnalogIn(void * pvParameters);
void tkAnalogInit(void);
void tkGprsTx(void * pvParameters);
void tkGprsRx(void * pvParameters);

void tkGprsInit(void);

TaskHandle_t xHandle_tkCmd, xHandle_tkControl, xHandle_tkDigitalIn, xHandle_tkAIn, xHandle_tkGprsTx, xHandle_tkGprsRx;

s08 startTask;
typedef struct {
	u08 resetCause;
	u08 mcusr;
} wdgStatus_t;

wdgStatus_t wdgStatus;

// Mensajes entre tareas
#define TK_PARAM_RELOAD			0x01	// to tkAnalogIN: reload
#define TKA_READ_FRAME			0x02	// to tkAnalogIN: (mode service) read a frame

//------------------------------------------------------------------------------------

xSemaphoreHandle sem_SYSVars;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )
typedef enum { WK_IDLE = 0, WK_NORMAL, WK_SERVICE, WK_MONITOR_FRAME, WK_MONITOR_SQE  } t_wrkMode;
typedef enum { PWR_CONTINUO = 0, PWR_DISCRETO } t_pwrMode;
typedef enum { OFF = 0, ON = 1 } t_onOff;
typedef enum { D_NONE = 0, D_BASIC = 1, D_DATA = 2, D_GPRS = 4, D_MEM = 8, D_DIGITAL = 16, D_CONSIGNA = 32, D_DEBUG = 64 } t_debug;
typedef enum { T_APAGADA = 0, T_PRENDIDA = 1 } t_terminalStatus;
typedef enum { MDM_PRENDIDO = 0, MDM_APAGADO } t_modemStatus;
typedef enum { modoPWRSAVE_OFF = 0, modoPWRSAVE_ON } t_pwrSave;

//------------------------------------------------------------------------------------

#define NRO_DIGITAL_CHANNELS	2
#define NRO_ANALOG_CHANNELS		3

#define DLGID_LENGTH		12
#define APN_LENGTH			32
#define PORT_LENGTH			7
#define IP_LENGTH			24
#define SCRIPT_LENGTH		64
#define PASSWD_LENGTH		15
#define PARAMNAME_LENGTH	5

typedef struct {
	u08 level[NRO_ANALOG_CHANNELS];		// 2
	double pulses[NRO_ANALOG_CHANNELS];	// 8
} dinData_t;

typedef struct {
	// size = 7+5+5+4+3*4+1 = 33 bytes
	RtcTimeType_t rtc;						// 7
	dinData_t dIn;							// 12
	double analogIn[NRO_ANALOG_CHANNELS];	// 12
	double batt;							// 4

} frameData_t;	// 38 bytes

typedef struct {
	// Variables de trabajo.
	// Tamanio: 302 bytes para 3 canales.

	u08 dummyBytes;
	u08 initByte;

	char dlgId[DLGID_LENGTH];
	char apn[APN_LENGTH];
	char serverPort[PORT_LENGTH];
	char serverAddress[IP_LENGTH];
	char serverIp[IP_LENGTH];
	char dlgIp[IP_LENGTH];
	char serverScript[SCRIPT_LENGTH];
	char passwd[PASSWD_LENGTH];

	u08 csq;
	u08 dbm;
	u08 ri;
	u08 termsw;

	u16 timerPoll;
	u32 timerDial;

	t_wrkMode wrkMode;
	t_pwrMode pwrMode;

	u08 logLevel;		// Nivel de info que presentamos en display.
	u08 debugLevel;		// Indica que funciones debugear.
	u08 gsmBand;

	u08 pwrSave;
	u16 pwrSaveStartTime;
	u16 pwrSaveEndTime;

	// Nombre de los canales
	char aChName[NRO_ANALOG_CHANNELS][PARAMNAME_LENGTH];
	char dChName[NRO_DIGITAL_CHANNELS][PARAMNAME_LENGTH];

	// Configuracion de Canales analogicos
	u08 Imin[NRO_ANALOG_CHANNELS];				// Coeficientes de conversion de I->magnitud (presion)
	u08 Imax[NRO_ANALOG_CHANNELS];
	u08 Mmin[NRO_ANALOG_CHANNELS];
	double Mmax[NRO_ANALOG_CHANNELS];

	// Configuracion de canales digitales
	double magPP[2];

	s08 roaming;

	u08 tilt;
	s08 tiltEnabled;

} systemVarsType;	// 315 bytes

systemVarsType systemVars,tmpSV;

#define EEADDR_SV 32		// Direccion inicio de la EE de escritura del systemVars.

//------------------------------------------------------------------------------------
// FUNCIONES DE USO GENERAL.
//------------------------------------------------------------------------------------
void u_readDigitalCounters( dinData_t *dIn , s08 resetCounters );
void u_panic( u08 panicCode );
s08 u_configAnalogCh( u08 channel, char *chName, char *s_iMin, char *s_iMax, char *s_mMin, char *s_mMax );
s08 u_configDigitalCh( u08 channel, char *chName, char *s_magPP );
s08 u_configPwrMode(u08 pwrMode);
s08 u_configTimerPoll(char *s_tPoll);
s08 u_configTimerDial(char *s_tDial);
void u_configPwrSave(u08 modoPwrSave, char *s_startTime, char *s_endTime);

s08 u_saveSystemParams(void);
s08 u_loadSystemParams(void);
void u_loadDefaults(void);
char *u_now(void);
u16 u_convertHHMM2min(u16 HHMM );
u16 u_convertMINS2hhmm ( u16 mins );
void u_reset(void);

char nowStr[32];

void u_readAnalogFrame (frameData_t *dFrame);
s16 u_readTimeToNextPoll(void);

s32 u_readTimeToNextDial(void);
s08 u_modemPwrStatus(void);

s08 u_wrRtc(char *s);

void u_debugPrint(u08 debugCode, char *msg, u16 size);
void pvMCP_init_MCP1(u08 modo);

//------------------------------------------------------------------------------------
// LED
#define LED_KA_PORT		PORTD
#define LED_KA_PIN		PIND
#define LED_KA_BIT		6
#define LED_KA_DDR		DDRD
#define LED_KA_MASK		0x40

#define LED_MODEM_PORT		PORTC
#define LED_MODEM_PIN		PINC
#define LED_MODEM_BIT		3
#define LED_MODEM_DDR		DDRC
#define LED_MODEM_MASK		0x04

//------------------------------------------------------------------------------------
// Q PINES
#define Q_PORT		PORTA
#define Q_DDR		DDRA
#define Q0_CTL_PIN	2
#define Q1_CTL_PIN	3

//------------------------------------------------------------------------------------
// PANIC CODES
#define P_OUT_TIMERSTART	1
#define P_OUT_TIMERCREATE	2
#define P_AIN_TIMERSTART	3
#define P_GPRS_TIMERSTART	4
#define P_CTL_TIMERCREATE	5
#define P_CTL_TIMERSTART	6

//------------------------------------------------------------------------------------
// WATCHDOG
u08 systemWdg;

#define WDG_CTL			0x01
#define WDG_CMD			0x02
#define WDG_DIN			0x04
#define WDG_AIN			0x08
#define WDG_GPRSTX		0x10
#define WDG_GPRSRX		0x20

void u_clearWdg( u08 wdgId );

//------------------------------------------------------------------------------------
// TERMINAL
// Pin de control de fuente de la terminal ( PD7)
#define TERMSW_PORT		PORTD
#define TERMSW_PIN		PIND
#define TERMSW_BIT		7
#define TERMSW_DDR		DDRD
#define TERMSW_MASK		0x80

#define TERMINAL_TIMEOUT	120			// 120 segundos para apagar la terminal.

s08 u_readTermsw(u08 *pin);
void u_restartTimerTerminal(void);
t_terminalStatus u_terminalPwrStatus(void);

//------------------------------------------------------------------------------------
// DCD
// Como el MCP23018 a veces no detecta el nivel del modem, cableamos
// el DCD a PB3
// Pin de control de fuente de la terminal ( PB3)
#define DCD_PORT		PORTB
#define DCD_PIN			PINB
#define DCD_BIT			3
#define DCD_DDR			DDRB
#define DCD_MASK		0x8

//------------------------------------------------------------------------------------

// Sensor de TILT
#define TILT_PORT		PORTB
#define TILT_PIN		PINB
#define TILT_BIT		0
#define TILT_DDR		DDRB
#define TILT_MASK		0x1

s08 u_checkAlarmFloding(void);

//------------------------------------------------------------------------------------
char debug_printfBuff[CHAR128];

#endif /* SP5K_H_ */
