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
 **   - 2019-04-18  1.0  Husj First version for Device Timer3.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "timer3.h"
#include "gpio.h"
#include "flash.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint32_t  Tim3_Capture_Value1,Tim3_Capure_Value2;
volatile int32_t   Tim3_Capture_Value;
volatile uint16_t  Tim3_Uev_Cnt;
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
 * TIM3中断服务函数
 ******************************************************************************/
void Tim3_IRQHandler(void)
{
    static uint8_t i;
    
    //Timer3 捕获中断A
    if(TRUE == Tim3_GetIntFlag(Tim3CA0Irq))
    {
        if(0 == i)
        {
            Tim3_Capture_Value1 = M0P_TIM3_MODE23->CCR0A_f.CCR0A;   //第一次读取捕获值
            Tim3_Uev_Cnt = 0;
            i++;
        }
        else 
        {
            Tim3_Capure_Value2 = M0P_TIM3_MODE23->CCR0A_f.CCR0A;  //第二次读取捕获值
            Tim3_Capture_Value = Tim3_Uev_Cnt * 0xFFFF + Tim3_Capure_Value2 - Tim3_Capture_Value1;  //两次捕获之间的差值
            
            Tim3_Uev_Cnt = 0;
            
            i = 0;
        }
        
        Tim3_ClearIntFlag(Tim3CA0Irq);  //清中断标志
    }
    
    //timer3计数溢出中断
    if(TRUE == Tim3_GetIntFlag(Tim3UevIrq))
    {
        Tim3_Uev_Cnt++;
        Tim3_ClearIntFlag(Tim3UevIrq);
    }
}

//时钟初始化
void App_ClockCfg(void)
{
    en_flash_waitcycle_t         enFlashWait;
    stc_sysctrl_pll_cfg_t     stcPLLCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcPLLCfg);
    
    enFlashWait = FlashWaitCycle1;                      //读等待周期设置为1（当HCLK大于24MHz时必须至少为1）
    Flash_WaitCycle(enFlashWait);                       // Flash 等待1个周期
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;     //RCH 4MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL 输出48MHz
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;              //输入时钟源选择RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul12;            //4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg);
    Sysctrl_SetPLLStableTime(SysctrlPllStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);
    
    Sysctrl_SysClkSwitch(SysctrlClkPLL);///< 时钟切换
}


//Timer3 Port端口配置
void App_Timer3PortCfg(void)
{
    stc_gpio_cfg_t            stcTIM3Port;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTIM3Port);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //端口外设时钟使能
    
    stcTIM3Port.enDir  = GpioDirIn;
    
    Gpio_Init(GpioPortA, GpioPin8, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortA,GpioPin8,GpioAf2);               //PA08设置为TIM3_CH0A
    
#if 0

    Gpio_Init(GpioPortA, GpioPin7, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortA,GpioPin7,GpioAf4);               //PA07设置为TIM3_CH0B    
    Gpio_Init(GpioPortB, GpioPin10, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortB,GpioPin10,GpioAf5);              //PB10设置为TIM3_CH1A
    
    Gpio_Init(GpioPortB, GpioPin0, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortB,GpioPin0,GpioAf2);               //PB00设置为TIM3_CH1B
    
    Gpio_Init(GpioPortB, GpioPin8, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortB,GpioPin8,GpioAf6);               //PB08设置为TIM3_CH2A
    
    Gpio_Init(GpioPortB, GpioPin15, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortB,GpioPin15,GpioAf2);              //PB15设置为TIM3_CH2B
#endif
}


//Timer3 配置
void App_Timer3Cfg(void)
{
    uint16_t                        u16ArrValue;
    uint16_t                        u16CntValue;

    stc_tim3_mode23_cfg_t        stcTim3BaseCfg;
    stc_tim3_m23_input_cfg_t     stcTim3PortCapCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTim3BaseCfg);
    DDL_ZERO_STRUCT(stcTim3PortCapCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTim3, TRUE);   //Timer3外设时钟使能
    
    stcTim3BaseCfg.enWorkMode    = Tim3WorkMode2;             //锯齿波模式
    stcTim3BaseCfg.enCT          = Tim3Timer;                 //定时器功能，计数时钟为内部PCLK
    stcTim3BaseCfg.enPRS         = Tim3PCLKDiv64;             //PCLK/64
    stcTim3BaseCfg.enCntDir      = Tim3CntUp;                 //向上计数，在三角波模式时只读
    
    Tim3_Mode23_Init(&stcTim3BaseCfg);                        //TIM3 的模式23功能初始化
        
    stcTim3PortCapCfg.enCHxACmpCap   = Tim3CHxCapMode;        //CHB通道设置为捕获模式
    stcTim3PortCapCfg.enCHxACapSel   = Tim3CHxCapFallRise;    //CHB通道上升沿下降沿捕获都使能
    stcTim3PortCapCfg.enCHxAInFlt    = Tim3FltPCLKDiv16Cnt3;  //PCLK/16 3个连续有效
    stcTim3PortCapCfg.enCHxAPolarity = Tim3PortPositive;      //正常输入输出
    
    Tim3_M23_PortInput_Cfg(Tim3CH0, &stcTim3PortCapCfg);   //端口输入初始化配置
    //Tim3_M23_PortInput_Cfg(Tim3CH1, &stcTim3PortCapCfg);   //端口输入初始化配置
    //Tim3_M23_PortInput_Cfg(Tim3CH2, &stcTim3PortCapCfg);   //端口输入初始化配置
    
    u16ArrValue = 0xFFFF;
    Tim3_M23_ARRSet(u16ArrValue, TRUE);                       //设置重载值,并使能缓存
    
    u16CntValue = 0;
    Tim3_M23_Cnt16Set(u16CntValue);                           //设置计数初值
    
    Tim3_ClearAllIntFlag();                                   //清中断标志
    Tim3_Mode23_EnableIrq(Tim3CA0Irq);                        //使能TIM3 CB0比较/捕获中断
    Tim3_Mode23_EnableIrq(Tim3UevIrq);                        //使能TIM3 Uev更新中断
    EnableNvic(TIM3_IRQn, IrqLevel1, TRUE);                   //TIM3中断使能
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
    App_ClockCfg();      //时钟初始化
    
    App_Timer3PortCfg(); //Timer3 Port端口配置
    
    App_Timer3Cfg();     //Timer3 配置
    
    Tim3_M23_Run();      //运行。
    
    while (1)
    {
      
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


