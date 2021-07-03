/**

@name  : user.c
@Brief : the application layer file
@Auth  : Jhonas
@Data  : 2020.09.01
@Copyright : Witlink

**/
 
#ifndef _USER_C
#define _USER_C
 
#include "user.h"

UART_HandleTypeDef UART0;

u32 gMs_1_Base  = 0;
u32 gS_1_Base = 0;

/***********************			@TaskDefination				****************************/
TaskHandle_t Start_Task_Handler; 

#define LED0_TASK_STK_SIZE				50
#define LED0_TASK_PRIO						2
TaskHandle_t Led0_Task_Handler; 

#define LED1_TASK_STK_SIZE				50
#define LED1_TASK_PRIO						3
TaskHandle_t Led1_Task_Handler;

#define UART0_TASK_STK_SIZE				50
#define UART0_TASK_PRIO						4
TaskHandle_t UART0_Task_Handler;
/*******************************************************************************/

/**
 *******************************************************************************
 ** \brief TASK function
 ******************************************************************************/
/* LED0 task */
void Led0_task(void *pvParameters)
{
	u8  sta = 0;
	u32 TaskCnt = 0;
	 
	while(1)
	{
		TaskCnt++;
		sta = SYS_LED_OUT_STA;
		if(sta)
		{
			SYS_LED_OFF;
		}
		else
		{
			SYS_LED_ON;
		}
		printf("%s : %d\n","Led0_task",(int)TaskCnt);
		if(TaskCnt == 50)
		{
#ifdef TaskDelete
			vTaskDelete(Led1_Task_Handler);
			printf("%s is Delete\n","Led1_Task");
#endif

#ifdef TasSuspend
			vTaskSuspend(Led1_Task_Handler);
			printf("%s is Suspend\n","Led1_Task");
#endif
		}
		else if(TaskCnt == 100)
		{
#ifdef TasSuspend
			vTaskResume(Led1_Task_Handler);
			printf("%s is Resume\n","Led1_Task");
#endif
		}
		vTaskDelay(100);
	}
}
 
/* LED1 task */
void Led1_task(void *pvParameters)
{
	u32 TaskCnt = 0;
	
	while(1)
	{
		TaskCnt++;
		Red_LED_OFF;
		vTaskDelay(200);
		Red_LED_ON;
		printf("%s : %d\n","Led1_task",(int)TaskCnt);
		vTaskDelay(800);
	}
}

/* UART0 task */
void UART0_task(void *pvParameters)
{
	u32 TaskCnt = 0;
	
	while(1)
	{
		TaskCnt++;
		if(UART0.state == UART_SUCCESS)
		{
			UART0_SendDataS(UART0.pRxBuf,UART0.RxLen);
			UART_Clear(&UART0);
#ifdef TaskSuspendFromISR
			printf("%s is Resume\n","Led1_Task");
#endif
		}
		vTaskDelay(10);
	}
}

/* Start task */
void Start_task(void *pvParameters)
{
	printf("Start_task\n");
	taskENTER_CRITICAL();

	xTaskCreate((TaskFunction_t )Led0_task,
							(const char * 	)"Led0_task",
							(const uint16_t )LED0_TASK_STK_SIZE,
							(void * const 	)NULL,
							(UBaseType_t 		)LED0_TASK_PRIO,
							(TaskHandle_t * )&Led0_Task_Handler );
	xTaskCreate((TaskFunction_t )Led1_task,
							(const char * 	)"Led1_task",
							(const uint16_t )LED1_TASK_STK_SIZE,
							(void * const 	)NULL,
							(UBaseType_t 		)LED1_TASK_PRIO,
							(TaskHandle_t * )&Led1_Task_Handler );
	xTaskCreate((TaskFunction_t )UART0_task,
							(const char * 	)"UART0_task",
							(const uint16_t )UART0_TASK_STK_SIZE,
							(void * const 	)NULL,
							(UBaseType_t 		)UART0_TASK_PRIO,
							(TaskHandle_t * )&UART0_Task_Handler );
	vTaskDelete(Start_Task_Handler);
		
	taskEXIT_CRITICAL();
}

/*******************************************************************************
 ** \brief 初始化串口缓存
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void UART_StateInit(UART_HandleTypeDef* uart)
{
	uart->state = UART_IDLE;
	uart->RxLen = 0;
	uart->TxLen = 0;
	uart->TmrCnt = 0;
}

void UART_Clear(UART_HandleTypeDef* uart)
{
	uart->state = UART_IDLE;
	uart->RxLen = 0;
	uart->TmrCnt = 0;
}

void SysLed_Toggle(void)
{
	if(SYS_LED_OUT_STA)
	{
		SYS_LED_OFF;
	}
	else
	{
		SYS_LED_ON;
	}
}

u8 SysIO_Init(void)
{
		stc_gpio_cfg_t stcGpioCfg;
	
		DDL_ZERO_STRUCT(stcGpioCfg);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 	
	
/* system LED -- PD05 */
	  stcGpioCfg.enDir =  GpioDirOut;
		Gpio_ClrIO(SYS_LED_PORT, SYS_LED_PIN);
    Gpio_Init(SYS_LED_PORT,SYS_LED_PIN,&stcGpioCfg);

/* Test LED -- PA03 */
		stcGpioCfg.enDir =  GpioDirOut;
		Gpio_ClrIO(Red_LED_PORT, Red_LED_PIN);
    Gpio_Init(Red_LED_PORT,Red_LED_PIN,&stcGpioCfg);

/* UART0 -- TX:PA08 RX:PA10 */
    DDL_ZERO_STRUCT(stcGpioCfg);

		///<TX - PA08
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(UART0_TX_PORT, UART0_TX_PIN, &stcGpioCfg);
    Gpio_SetAfMode(UART0_TX_PORT, UART0_TX_PIN, GpioAf1);

    ///<RX - PA10
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(UART0_RX_PORT, UART0_RX_PIN, &stcGpioCfg);
    Gpio_SetAfMode(UART0_RX_PORT, UART0_RX_PIN, GpioAf1);
	
/* XTL output -- PA05 */
#if 0
    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enDrv = GpioDrvH;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.enOD = GpioOdDisable;
    ///< GPIO IO PA05初始化
    Gpio_Init(SYS_XTL_PORT, SYS_XTL_PIN, &stcGpioCfg);    
    ///< 配置PA05复用功能为XTL输出
    Gpio_SetAfMode(SYS_XTL_PORT, SYS_XTL_PIN, GpioAf6);
    
    ///< 使能XHL从PA05输出
//    Gpio_SfHClkOutputCfg(GpioSfHclkOutEnable, GpioSfHclkOutDiv1);
#endif

		return TRUE;
}

/**
 *******************************************************************************
 ** \brief 初始化定时器0，周期为1ms的定时中断
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
__STATIC_INLINE void HC32_InitTIM0(void)
{
	M0P_SYSCTRL->PERI_CLKEN_f.BASETIM = 1;

	M0P_TIM0_MODE0->CNT = 65536-((SystemCoreClock/64)/1000);//配置定时器中断周期1ms
	M0P_TIM0_MODE0->ARR = 65536-((SystemCoreClock/64)/1000);

	M0P_TIM0_MODE0->M0CR = 0x462;                           //中断使能；64倍时钟分频；16位重载计数
	M0P_TIM0_MODE0->ICLR = 0;                               //清除中断标志位
	
  NVIC_ClearPendingIRQ(TIM0_IRQn);                        //清除TIMER0中断挂起bit
  NVIC_SetPriority(TIM0_IRQn, IrqLevel2);                 //设定TIMER0优先级
	NVIC_EnableIRQ(TIM0_IRQn);                              //使能NVIC_TIMER0中断
    
	M0P_TIM0_MODE0->M0CR_f.CTEN = 1;                        //使能定时器
}

u8 Tmr_Init(void)
{	
	HC32_InitTIM0();
	
	return TRUE;
}

/**
 *******************************************************************************
 ** \brief Timer0 Interruput
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Tim0_IRQHandler(void)
{
	if(READ_BIT(M0P_TIM0_MODE0->IFR, 0x01))
	{
		CLEAR_BIT(M0P_TIM0_MODE0->ICLR, 0x01);

		/* 1ms timer base */
		gMs_1_Base++;
		
		/* LPUART0 recive time out -- > IAP LPUART */
		if(UART0.state == UART_RECEIVING)
		{
			UART0.TmrCnt++;
			if(UART0.TmrCnt > UART_RX_TMROUT)
			{
				UART0.state = UART_SUCCESS;
#ifdef TaskSuspendFromISR
				xTaskResumeFromISR(Led1_Task_Handler);
#endif
			}
		}
		
		/* 10ms timer base */
		if(gMs_1_Base%100 == 0)
		{
//			SysLed_Toggle();
		}
		
		/* 1s timer base */
		if(gMs_1_Base%1000 == 0)
		{
			gS_1_Base++;
		}
	}
}

#ifdef SYSTEM_XTH
///<请注意根据外部晶振配置宏——[SYSTEM_XTH]
void SystemClkInit_XTH(en_sysctrl_xth_freq_t enXthFreq)
{
    ///<======================== 切换至XTH32MHz ==============================    
    ///< 当使用的时钟源HCLK大于24M：设置FLASH 读等待周期为1 cycle(默认值也为1 cycle)
#if 0
    if(SysctrlXthFreq24_32MHz == enXthFreq)
    {
        Flash_WaitCycle(FlashWaitCycle1);    
    }
#endif
    
    ///< 切换时钟前（根据外部高速晶振）设置XTH频率范围,配置晶振参数，使能目标时钟，此处为SYSTEM_XTH = 32MHz
    Sysctrl_SetXTHFreq(enXthFreq);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver3);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);
    delay1ms(10);
//    Sysctrl_SysClkSwitch(SysctrlClkXTH);

#if 0	
    if(SysctrlXthFreq24_32MHz != enXthFreq)
    {
        Flash_WaitCycle(FlashWaitCycle0);    
    }
#endif
}
#endif

#ifdef SYSTEM_XTL
void SystemClkInit_XTL(void)
{
    ///< 切换时钟前（根据外部低速晶振）设置XTL晶振参数，使能目标时钟，SYSTEM_XTL = 32768Hz
    Sysctrl_XTLDriverCfg(SysctrlXtlAmp3, SysctrlXtalDriver3);
    Sysctrl_SetXTLStableTime(SysctrlXtlStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTL, TRUE);
//    Sysctrl_SysClkSwitch(SysctrlClkXTL);
#if 0
    Flash_WaitCycle(FlashWaitCycle0);    
#endif
}
#endif

#if (SYSTEM_XTH == 32000000u)
///<请注意根据外部晶振配置宏——[SYSTEM_XTH],如果使用PLL，XTH必须小于24MHz
void SysPLL_Config(void)
{
    stc_sysctrl_pll_cfg_t stcPLLCfg;    
    
    ///< 切换时钟前（根据外部高速晶振）设置XTH频率范围,配置晶振参数，使能目标时钟，此处为SYSTEM_XTH = 8MHz
    Sysctrl_SetXTHFreq(SysctrlXthFreq20_32MHz);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver3);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);
    delay1ms(10);
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq6_12MHz;     //XTH 8MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;   //PLL 输出
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;           		 //输入时钟源选择RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul6;              //8MHz x 6 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg); 
    
    ///< 当使用的时钟源HCLK大于24M：设置FLASH 读等待周期为1 cycle(默认值也为1 cycle)
    Flash_WaitCycle(FlashWaitCycle1);    

    ///< 使能PLL
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);    
    ///< 时钟切换到PLL
    Sysctrl_SysClkSwitch(SysctrlClkPLL);
  
#if 0
/* can't work */
	stc_sysctrl_pll_cfg_t nPstcPLLCfg;
	stc_sysctrl_pll_cfg_t *pPstcPLLCfg = &nPstcPLLCfg;
	
	pPstcPLLCfg->enInFreq = SysctrlPllInFreq6_12MHz;
	pPstcPLLCfg->enOutFreq = SysctrlPllOutFreq36_48MHz;
	pPstcPLLCfg->enPllClkSrc = SysctrlPllXthXtal;
	pPstcPLLCfg->enPllMul = SysctrlPllMul6;
	
	Sysctrl_SetPLLFreq(pPstcPLLCfg);
	Sysctrl_SysClkSwitch(SysctrlClkPLL);																				/* change system clock to PLL(48MHz) */
#endif
}
#endif

void App_ClkDivInit(void)
{
    //时钟分频设置
    Sysctrl_SetHCLKDiv(SysctrlHclkDiv1);
    Sysctrl_SetPCLKDiv(SysctrlPclkDiv1);
}

uint8_t SysClk_Init(void)
{
	SysPLL_Config();
	SystemClkInit_XTL();
	App_ClkDivInit();
	
	return TRUE;
}

//串口配置
void App_UartCfg(void)
{
    stc_uart_cfg_t    stcCfg;

    DDL_ZERO_STRUCT(stcCfg);

    ///< 开启外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE);///<使能uart0模块时钟

    ///<UART Init
    stcCfg.enRunMode        = UartMskMode3;          ///<模式3
    stcCfg.enStopBit        = UartMsk1bit;           ///<1bit停止位
    stcCfg.enMmdorCk        = UartMskEven;           ///<偶检验
    stcCfg.stcBaud.u32Baud  = 115200;                ///<波特率115200
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       ///<通道采样分频配置
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq(); ///<获得外设时钟（PCLK）频率值
    Uart_Init(M0P_UART0, &stcCfg);                   ///<串口初始化

    ///<UART中断使能
    Uart_ClrStatus(M0P_UART0,UartRC);                ///<清接收请求
    Uart_ClrStatus(M0P_UART0,UartTC);                ///<清接收请求
    Uart_EnableIrq(M0P_UART0,UartRxIrq);             ///<使能串口接收中断
    Uart_DisableIrq(M0P_UART0,UartTxIrq);            ///<使能串口发射中断
    EnableNvic(UART0_IRQn, IrqLevel3, TRUE);       	 ///<系统中断使能
}

///<UART0 中断服务函数
void Uart0_IRQHandler(void)
{
	u8 u8RxData = 0;
	
	if(Uart_GetStatus(M0P_UART0, UartTC))       ///发送数据中断
	{
#if 0
			LPUart_ClrStatus(M0P_UART0, UartTC);      ///<清发送中断请求
			
			LPUart_DisableIrq(M0P_UART0,UartTxIrq);   ///<禁止发送中断
			LPUart_EnableIrq(M0P_UART0,UartRxIrq);    ///<使能接收中断
#endif
	}
	
	if(Uart_GetStatus(M0P_UART0, UartRC))       ///接收数据中断
	{
#if 1
		Uart_ClrStatus(M0P_UART0, UartRC);      	///<清接收中断请求
		u8RxData = Uart_ReceiveData(M0P_UART0);   ///读取数据

		UART0.TmrCnt = 0;
		if(UART0.state == UART_IDLE)
		{
			UART0.RxLen = 0;
			UART0.state = UART_RECEIVING;
		}
		else if(UART0.state == UART_SUCCESS)
		{
			return;
		}
		
		if(UART0.RxLen < UART_BUF_MAX)
		{
			UART0.pRxBuf[UART0.RxLen] = u8RxData;
			UART0.RxLen++;
		}
		else
		{
			return;
		}
#endif
	}
}

u8 Uart_SendDataS(u8 *Data,u8 len)
{
	u8 Cnt,nData = 0;

	for(Cnt = 0;Cnt < len;Cnt++)
	{
		nData = *(Data+Cnt);
		Uart_SendDataPoll(M0P_UART0,nData);
	}

	return TRUE;
}

u8 UART0_SendDataS(u8 *Data,u8 len)
{
	memcpy(UART0.pTxBuf,Data,len);
	UART0.TxLen = len;
	Uart_SendDataS(UART0.pTxBuf,UART0.TxLen);
	UART0.TxLen = 0;
	
	return TRUE;
}

#endif
