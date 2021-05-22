/******************************************************************************
* Copyright (C) 2016, Huada Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Huada Semiconductor Co.,Ltd ("HDSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with HDSC
* components. This software is licensed by HDSC to be adapted only
* for use in systems utilizing HDSC components. HDSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. HDSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/
/******************************************************************************/
/** \file main.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2019-04-17  1.0  Husj First version for General Timer012.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "bt.h"
#include "flash.h"
#include "gpio.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
volatile uint32_t u32PwcCapValue;
volatile uint16_t u16TIM0_CntValue;

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/


/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/*******************************************************************************
 * TIM0中断服务函数
 ******************************************************************************/
void Tim0_IRQHandler(void)
{
    static uint16_t u16TIM0_OverFlowCnt;
    
    static uint16_t u16TIM0_CapValue;
    
    //Timer0 模式1 计数溢出中断
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
        u16TIM0_OverFlowCnt++;        //计数脉宽测量开始边沿到结束边沿过程中timer的溢出次数
        
        Bt_ClearIntFlag(TIM0,BtUevIrq); //清除中断标志
    }

    //Timer0 模式1 PWC脉宽测量中断
    if(TRUE == Bt_GetIntFlag(TIM0, BtCA0Irq))
    {
        u16TIM0_CntValue = Bt_M1_Cnt16Get(TIM0);         //读取当前计数值
        u16TIM0_CapValue = Bt_M1_PWC_CapValueGet(TIM0);  //读取脉宽测量值
        
        u32PwcCapValue = u16TIM0_OverFlowCnt*0x10000 + u16TIM0_CapValue;
        
        u16TIM0_OverFlowCnt = 0;
        
        Bt_ClearIntFlag(TIM0, BtCA0Irq); //清除中断标志
    }
}

//时钟配置初始化
void App_ClockCfg(void)
{
    en_flash_waitcycle_t      enWaitCycle;
    stc_sysctrl_pll_cfg_t     stcPLLCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcPLLCfg);
    
    enWaitCycle = FlashWaitCycle1;
    Flash_WaitCycle(enWaitCycle);
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;     //RCH 4MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL 输出48MHz
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;              //输入时钟源选择RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul12;            //4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg);
    Sysctrl_SetPLLStableTime(SysctrlPllStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);
    
    Sysctrl_SysClkSwitch(SysctrlClkPLL);  ///< 时钟切换
}

//端口配置初始化
void App_Timer0PortCfg(void)
{
    stc_gpio_cfg_t          stcTIM0Port;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTIM0Port);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //GPIO 外设时钟使能
    
    //PA00设置为TIM0_CHA
    stcTIM0Port.enDir  = GpioDirIn;
    Gpio_Init(GpioPortA, GpioPin0, &stcTIM0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin0,GpioAf7);
    
    //PA01设置为TIM0_CHB
    //Gpio_Init(GpioPortA, GpioPin1, &stcTIM0Port);
    //Gpio_SetAfMode(GpioPortA,GpioPin1,GpioAf3);
}

//Timer0 配置
void App_Timer0Cfg(void)
{
    uint16_t                u16CntValue;
    stc_bt_mode1_cfg_t      stcBtBaseCfg;
    stc_bt_pwc_input_cfg_t  stcBtPwcInCfg;

    //结构体初始化清零
    DDL_ZERO_STRUCT(stcBtBaseCfg);
    DDL_ZERO_STRUCT(stcBtPwcInCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能
    
    stcBtBaseCfg.enWorkMode = BtWorkMode1;                  //定时器模式
    stcBtBaseCfg.enCT       = BtTimer;                      //定时器功能，计数时钟为内部PCLK
    stcBtBaseCfg.enPRS      = BtPCLKDiv1;                   //PCLK
    stcBtBaseCfg.enOneShot  = BtPwcOneShotDetect;           //PWC循环检测
    
    Bt_Mode1_Init(TIM0, &stcBtBaseCfg);                     //TIM0 的模式0功能初始化
    
    stcBtPwcInCfg.enTsSel  = BtTs6IAFP;                     //PWC输入选择 CHA
    stcBtPwcInCfg.enIA0Sel = BtIA0Input;                    //CHA选择IA0
    stcBtPwcInCfg.enFltIA0 = BtFltPCLKDiv16Cnt3;            //PCLK/16 3个连续有效
    //stcBtPwcInCfg.enIB0Sel = BtIB0Input;                  //CHB选择IB0
    //stcBtPwcInCfg.enFltIB0 = BtFltPCLKDiv16Cnt3;          //PCLK/16 3个连续有效
    
    Bt_M1_Input_Cfg(TIM0, &stcBtPwcInCfg);                  //PWC输入设置

    Bt_M1_PWC_Edge_Sel(TIM0, BtPwcRiseToFall);              //上升沿到下降沿捕获

    u16CntValue = 0;
    Bt_M1_Cnt16Set(TIM0, u16CntValue);                      //设置计数初值

    Bt_ClearIntFlag(TIM0,BtUevIrq);                         //清Uev中断标志
    Bt_ClearIntFlag(TIM0,BtCA0Irq);                         //清捕捉中断标志
    Bt_Mode1_EnableIrq(TIM0, BtUevIrq);                     //使能TIM0溢出中断
    Bt_Mode1_EnableIrq(TIM0, BtCA0Irq);                     //使能TIM0捕获中断
    EnableNvic(TIM0_IRQn, IrqLevel3, TRUE);                 //TIM0中断使能
}

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/

int32_t main(void)
{
    App_ClockCfg();         //时钟配置初始化
    
    App_Timer0PortCfg();    //端口配置初始化
    
    App_Timer0Cfg();        //Timer0配置初始化
    
    Bt_M1_Run(TIM0);        //TIM0 运行。
    
    while (1)
    {
        delay1ms(2000);
        Bt_M1_Run(TIM0);    //重复使能PWC功能
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


