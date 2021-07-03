#ifndef _USER_H
#define _USER_H

#include "ddl.h"
#include "timer3.h"
#include "userdef.h"
#include "flash.h"
#include "uart.h"
#include "Sys_port.h"
#include "FreeRTOS.h"
#include "task.h"

/* Micro Definition */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define START_TASK_STK_SIZE				50
#define START_TASK_PRIO						1

#define   UART_BUF_MAX						128
#define   UART_RX_TMROUT    			3

/* Enum */
enum
{
	UART_IDLE = 0,
	UART_RECEIVING,
	UART_SUCCESS,
	UART_TRANSMITTING,
};


/* Struct */
typedef struct
{
	u8  state;
	u8  TxLen;
	u8  pTxBuf[UART_BUF_MAX];
	u32 RxLen;
	u8  pRxBuf[UART_BUF_MAX];
	u32 TmrCnt;
}UART_HandleTypeDef;


/* Variable Declaration */
extern TaskHandle_t Start_Task_Handler; 
extern UART_HandleTypeDef UART0;

/* Function Declaration */
extern u8 Tmr_Init(void);
extern void Tim0_IRQHandler(void);
extern void UART_StateInit(UART_HandleTypeDef* uart);
extern void UART_Clear(UART_HandleTypeDef* uart);
extern u8 Uart_SendDataS(u8 *Data,u8 len);
extern u8 UART0_SendDataS(u8 *Data,u8 len);
extern u8 SysIO_Init(void);
extern void SysPLL_Config(void);
extern u8 SysClk_Init(void);
extern u8 Tmr_Init(void);
extern void App_UartCfg(void);

extern void Start_task(void *pvParameters);

#endif
