/*
 * sp5KV4_8CH_tkGprs.h
 *
 *  Created on: 22 de abr. de 2016
 *      Author: pablo
 */

#ifndef SP5KV5_3CH_TKGPRS_SP5KV5_3CH_TKGPRS_H_
#define SP5KV5_3CH_TKGPRS_SP5KV5_3CH_TKGPRS_H_

u32 tickCount;

// Estados
typedef enum {
	gST_INICIAL = 0,
	gST_MODEMAPAGADO,
	gST_MODEMPRENDIENDO,
	gST_CONFIGURAR,
	gST_STANDBY,
	gST_OPENSOCKET,
	gST_INITFRAME,
	gST_CONFFRAME,
	gST_DATAFRAME,

} t_tkGprs_state;

// Subestados.
typedef enum {
	// Estado OFF
	gSST_MODEMAPAGADO_00 = 0,
	gSST_MODEMAPAGADO_01,

	gSST_MODEMPRENDIENDO_00,
	gSST_MODEMPRENDIENDO_01,
	gSST_MODEMPRENDIENDO_02,
	gSST_MODEMPRENDIENDO_03,
	gSST_MODEMPRENDIENDO_04,
	gSST_MODEMPRENDIENDO_05,
	gSST_MODEMPRENDIENDO_06,

	gSST_CONFIGURAR_00,
	gSST_CONFIGURAR_01,
	gSST_CONFIGURAR_02,
	gSST_CONFIGURAR_03,
	gSST_CONFIGURAR_04,
	gSST_CONFIGURAR_05,
	gSST_CONFIGURAR_06,
	gSST_CONFIGURAR_07,
	gSST_CONFIGURAR_08,
	gSST_CONFIGURAR_09,
	gSST_CONFIGURAR_10,
	gSST_CONFIGURAR_11,
	gSST_CONFIGURAR_12,

	gSST_STANDBY_00,
	gSST_STANDBY_01,
	gSST_STANDBY_02,
	gSST_STANDBY_03,
	gSST_STANDBY_04,
	gSST_STANDBY_05,
	gSST_STANDBY_06,
	gSST_STANDBY_07,
	gSST_STANDBY_08,

	gSST_OPENSOCKET_00,
	gSST_OPENSOCKET_01,
	gSST_OPENSOCKET_02,
	gSST_OPENSOCKET_03,
	gSST_OPENSOCKET_04,
	gSST_OPENSOCKET_05,
	gSST_OPENSOCKET_06,
	gSST_OPENSOCKET_07,

	gSST_INITFRAME_00,
	gSST_INITFRAME_01,
	gSST_INITFRAME_02,
	gSST_INITFRAME_03,
	gSST_INITFRAME_04,
	gSST_INITFRAME_05,

	gSST_CONFFRAME_00,

	gSST_DATAFRAME_00,
	gSST_DATAFRAME_01,
	gSST_DATAFRAME_02,
	gSST_DATAFRAME_03,
	gSST_DATAFRAME_04,

} t_tkGprs_subState;

typedef enum { SOCKET_CLOSED = 0, SOCKET_OPEN } t_socket;
typedef enum { MRSP_NONE = 0, MRSP_OK , MRSP_ERROR, MRSP_CONNECT, MRSP_NOCARRIER, MRSP_E2IPA, MRSP_CREG, MRSP_INIT_OK } t_modemResponse;
typedef enum { NO_FRAME = 0, INIT_FRAME, CONF_FRAME, DATA_FRAME } t_nextFrame;

typedef struct {
	s08 msgReload;
	t_modemResponse modemResponse;
	t_socket socketStatus;
	s08 gsmBandOK;
	s08 memRcds4Tx;
	s08 moreRcds4Tx;
	s08 memRcds4Del;

} GPRS_flags;

typedef struct {
	u32 cTimer;
	u08 txRcdsInWindow;
	u32 awaitSecs;
	u08 nroINITS;
	u08 nroLOTEtryes;
} GPRS_counters;

typedef struct {
	u08 nextFrame;
	u08 oldState;
	u08 state;
	u08 subState;
} GPRS_status;

struct {
	GPRS_counters counters;
	GPRS_flags flags;
	GPRS_status state;
} GPRS_stateVars;

struct {
	char buffer[UART0_RXBUFFER_LEN];
	u16 ptr;
} gprsRx;


#define FRAMEXTXWINDOW	10		// Registros por frameWindow.
#define MAXTRYESLOTE	4

char gprs_printfBuff[CHAR256];
char gprsRX_printfBuff[CHAR256];

// Acciones Generales
void sm_APAGADO(void);
void sm_MODEMPRENDIENDO(void);
void sm_CONFIGURAR(void);
void sm_STANDBY(void);
void sm_SOCKET(void);
void sm_INITFRAME(void);
void sm_DATAFRAME(void);
t_tkGprs_subState pv_cambiarEstado( t_tkGprs_state actualState, t_tkGprs_state newState );

void g_printExitMsg(char *code);
void g_setSocketStatus( u08 socketStatus);
void g_setModemResponse( u08 modemStatus);
void g_printRxBuffer(void);
void g_flushRXBuffer(void);
s08 g_strstr(char *rsp, size_t *pos);
char *g_getRxBuffer(void);

// Funciones de reconfiguracion
void g_GPRSprocessServerClock(void);
u08 g_GPRSprocessPwrMode(void);
u08 g_GPRSprocessTimerPoll(void);
u08 g_GPRSprocessTimerDial(void);
u08 g_GPRSprocessPwrSave(void);
u08 g_GPRSprocessAch(u08 channel);
u08 g_GPRSprocessDch(u08 channel);
u08 g_GPRSprocessTilt(void);
void g_GPRSprocessReset(void);
s08 g_checkReloadConfig(t_tkGprs_state gprsState );

#endif /* SP5KV5_3CH_TKGPRS_SP5KV5_3CH_TKGPRS_H_ */
