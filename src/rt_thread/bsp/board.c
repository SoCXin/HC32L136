#include <rthw.h>
#include <rtthread.h>

#include "sysctrl.h"
#include "flash.h"
#include "board.h"
//#include "usart.h"

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}



static void SYSCTRL_Configuration(void)
{
	stc_sysctrl_clk_config_t stcCfg;
    stc_sysctrl_pll_config_t stcPLLCfg;
	
	Sysctrl_SetPeripheralGate(SysctrlPeripheralFlash, TRUE);
	Flash_WaitCycle(FlashWaitCycle1);
	Sysctrl_SetRCHTrim(SysctrlRchFreq4MHz);
	stcPLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;
	stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;
	stcPLLCfg.enPllClkSrc = SysctrlPllRch;
	stcPLLCfg.enPllMul    = SysctrlPllMul12;
	Sysctrl_SetPLLFreq(&stcPLLCfg);
	stcCfg.enClkSrc  = SysctrlClkPLL;
	stcCfg.enHClkDiv = SysctrlHclkDiv1;
	stcCfg.enPClkDiv = SysctrlPclkDiv1;
	Sysctrl_ClkInit(&stcCfg);
	SystemCoreClockUpdate();
}

#ifdef PRINT_FREQ_INFO

void print_freq_info(void)
{
	rt_kprintf("\nSystemCoreClock is %dHZ\n", (int)SystemCoreClock);
	rt_kprintf("HCLK_Frequency is %dHZ\n", (int)Sysctrl_GetHClkFreq());
	rt_kprintf("PCLK_Frequency is %dHZ\n", (int)Sysctrl_GetPClkFreq());
}
#endif

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
	/* enter interrupt */
	rt_interrupt_enter();

	rt_tick_increase();

	/* leave interrupt */
	rt_interrupt_leave();
}
/**
 * This function will initial HC32 board.
 */
void rt_hw_board_init()
{
	/* NVIC Configuration */
	NVIC_Configuration();

	/* Configure the SysTick */
	SYSCTRL_Configuration();
	SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

	/* Initial usart deriver, and set console device */
	rt_hw_usart_init();
#ifdef RT_USING_CONSOLE
	rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
	/* Print RCC freq info */
#ifdef PRINT_FREQ_INFO
	print_freq_info();
#endif
	/* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}

/*@}*/
