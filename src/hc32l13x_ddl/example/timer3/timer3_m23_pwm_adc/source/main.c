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
 **   - 2019-04-18  1.0  Husj First version for Timer3.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "timer3.h"
#include "gpio.h"
#include "flash.h"
#include "adc.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint16_t u16AdcResult02;
uint16_t u16AdcResult03;
uint16_t u16AdcResult05;

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
    
    //Timer3 模式23 更新中断
    if(TRUE == Tim3_GetIntFlag(Tim3UevIrq))
    {
        if(0 == i)
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN,TRUE);  //LED 引脚输出高电平
            i++;
        }
        else if(1 == i)
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN,FALSE); //LED 引脚输出低电平
            i = 0;
        }
        
        Tim3_ClearIntFlag(Tim3UevIrq);  //清中断标志
    }
}

void Adc_IRQHandler(void)
{
    static uint8_t i;
  
    //ADC 插队(JQR)扫描采样完成中断
    if(TRUE == Adc_GetIrqStatus(AdcMskIrqJqr))
    {
        u16AdcResult02 = (uint16_t)Adc_GetJqrResult(AdcJQRCH0MUX); //读取JQR通道0(PA02)的采样值
        u16AdcResult03 = (uint16_t)Adc_GetJqrResult(AdcJQRCH1MUX); //读取JQR通道1(PA03)的采样值
        u16AdcResult05 = (uint16_t)Adc_GetJqrResult(AdcJQRCH2MUX); //读取JQR通道2(PA05)的采样值
    
        if(0 == i)
        {
            Tim3_M23_CCR_Set(Tim3CCR0A, 0x3000); //设置CH0 通道A比较值
            Tim3_M23_CCR_Set(Tim3CCR1A, 0x3000); //设置CH1 通道A比较值
            Tim3_M23_CCR_Set(Tim3CCR2A, 0x3000); //设置CH2 通道A比较值
            i++;
        }
        else if(1 == i)
        {
            Tim3_M23_CCR_Set(Tim3CCR0A, 0x6000); //设置CH0 通道A比较值
            Tim3_M23_CCR_Set(Tim3CCR1A, 0x6000); //设置CH1 通道A比较值
            Tim3_M23_CCR_Set(Tim3CCR2A, 0x6000); //设置CH2 通道A比较值
            i = 0;
        }
        
        Adc_ClrIrqStatus(AdcMskIrqJqr);  //清中断标志
    }
}

//时钟初始化
void App_ClockCfg(void)
{
    en_flash_waitcycle_t      enFlashWait;
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
    stc_gpio_cfg_t               stcTIM3Port;
    stc_gpio_cfg_t               stcLEDPort;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTIM3Port);
    DDL_ZERO_STRUCT(stcLEDPort);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //端口外设时钟使能
    
    stcLEDPort.enDir  = GpioDirOut;
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &stcLEDPort);        //PD14设置为LED输出
    
    stcTIM3Port.enDir  = GpioDirOut;
    
    Gpio_Init(GpioPortA, GpioPin8, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortA,GpioPin8,GpioAf2);               //PA08设置为TIM3_CH0A
    
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
}

//ADC 插队扫描采样配置
void App_AdcCfg(void)
{
    stc_adc_cfg_t              stcAdcCfg;
    stc_adc_jqr_cfg_t          stcAdcJqrCfg;
    
    //结构体初始化清零    
    DDL_ZERO_STRUCT(stcAdcCfg);
    DDL_ZERO_STRUCT(stcAdcJqrCfg);
        
    //Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
    
    Gpio_SetAnalogMode(GpioPortA, GpioPin2);        //PA02配置为模拟通道
    Gpio_SetAnalogMode(GpioPortA, GpioPin3);        //PA03配置为模拟通道   
    Gpio_SetAnalogMode(GpioPortA, GpioPin5);        //PA05配置为模拟通道
    
    
    //ADC配置
    Adc_Enable();
    M0P_BGR->CR_f.BGR_EN = 0x1u;//BGR必须使能
    M0P_BGR->CR_f.TS_EN  = 0x0u;
    delay100us(1);
    
    stcAdcCfg.enAdcMode         = AdcScanMode;          //连续采样模式
    stcAdcCfg.enAdcClkDiv       = AdcMskClkDiv2;        //Adc工作时钟 PCLK/2
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle8Clk;  //采样时钟 8个周期
    stcAdcCfg.enAdcRefVolSel    = AdcMskRefVolSelAVDD;  //内部AVDD
    stcAdcCfg.enAdcOpBuf        = AdcMskBufDisable;     //内部电压跟随器关闭
    stcAdcCfg.enInRef           = AdcMskInRefDisable;   //内部参考电压Disable
    
    Adc_Init(&stcAdcCfg);                               //Adc初始化
    
    Adc_CfgJqrChannel(AdcJQRCH0MUX, AdcExInputCH2);     //配置插队扫描转换通道
    Adc_CfgJqrChannel(AdcJQRCH1MUX, AdcExInputCH3);
    Adc_CfgJqrChannel(AdcJQRCH2MUX, AdcExInputCH5);     //采样顺序CH2 --> CH1 --> CH0
    
    stcAdcJqrCfg.bJqrDmaTrig = FALSE;   //禁止JQR转换触发DMA
    stcAdcJqrCfg.u8JqrCnt = 3;          //转换起始通道(3-1已在库函数内计算)
    Adc_JqrModeCfg(&stcAdcJqrCfg);      //配置插队扫描转换模式
    
    Adc_JqrExtTrigCfg(AdcMskTrigTimer3, TRUE);          //Timer0触发插队扫描转换
    
    Adc_EnableIrq();                                    //使能Adc中断
    EnableNvic(ADC_IRQn, IrqLevel1, TRUE);          //Adc开中断
}

//Timer3 配置
void App_Timer3Cfg(uint16_t u16Period, uint16_t u16CHxACompare, uint16_t u16CHxBCompare)
{
    uint16_t                     u16CntValue;
    uint8_t                      u8ValidPeriod;
    stc_tim3_mode23_cfg_t        stcTim3BaseCfg;
    stc_tim3_m23_compare_cfg_t   stcTim3PortCmpCfg;
    stc_tim3_m23_adc_trig_cfg_t  stcTim3TrigAdc;
    stc_tim3_m23_dt_cfg_t        stcTim3DeadTimeCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTim3BaseCfg);
    DDL_ZERO_STRUCT(stcTim3PortCmpCfg);
    DDL_ZERO_STRUCT(stcTim3TrigAdc);
    DDL_ZERO_STRUCT(stcTim3DeadTimeCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTim3, TRUE);   //Timer3外设时钟使能
        
    stcTim3BaseCfg.enWorkMode    = Tim3WorkMode3;             //三角波模式
    stcTim3BaseCfg.enCT          = Tim3Timer;                 //定时器功能，计数时钟为内部PCLK
    stcTim3BaseCfg.enPRS         = Tim3PCLKDiv1;              //PCLK
    //stcTim3BaseCfg.enCntDir    = Tim3CntUp;                 //向上计数，在三角波模式时只读
    stcTim3BaseCfg.enPWMTypeSel  = Tim3ComplementaryPWM;      //互补输出PWM
    stcTim3BaseCfg.enPWM2sSel    = Tim3SinglePointCmp;        //单点比较功能
    stcTim3BaseCfg.bOneShot      = FALSE;                     //循环计数
    stcTim3BaseCfg.bURSSel       = FALSE;                     //上下溢更新
    
    Tim3_Mode23_Init(&stcTim3BaseCfg);                        //TIM3 的模式23功能初始化
    
    Tim3_M23_ARRSet(u16Period, TRUE);                         //设置重载值,并使能缓存
    
    Tim3_M23_CCR_Set(Tim3CCR0A, u16CHxACompare);              //设置比较值A,(PWM互补模式下只需要设置比较值A)
    Tim3_M23_CCR_Set(Tim3CCR1A, u16CHxACompare);
    Tim3_M23_CCR_Set(Tim3CCR2A, u16CHxACompare);
    
    stcTim3PortCmpCfg.enCHxACmpCtrl   = Tim3PWMMode2;         //OCREFA输出控制OCMA:PWM模式2
    stcTim3PortCmpCfg.enCHxAPolarity  = Tim3PortPositive;     //正常输出
    stcTim3PortCmpCfg.bCHxACmpBufEn   = TRUE;                 //A通道缓存控制
    stcTim3PortCmpCfg.enCHxACmpIntSel = Tim3CmpIntNone;       //A通道比较控制:无
    
    stcTim3PortCmpCfg.enCHxBCmpCtrl   = Tim3PWMMode2;         //OCREFB输出控制OCMB:PWM模式2(PWM互补模式下也要设置，避免强制输出)
    stcTim3PortCmpCfg.enCHxBPolarity  = Tim3PortPositive;     //正常输出
    //stcTim3PortCmpCfg.bCHxBCmpBufEn = TRUE;                 //B通道缓存控制使能
    stcTim3PortCmpCfg.enCHxBCmpIntSel = Tim3CmpIntNone;       //B通道比较控制:无
    
    Tim3_M23_PortOutput_Cfg(Tim3CH0, &stcTim3PortCmpCfg);  //比较输出端口配置
    Tim3_M23_PortOutput_Cfg(Tim3CH1, &stcTim3PortCmpCfg);  //比较输出端口配置
    Tim3_M23_PortOutput_Cfg(Tim3CH2, &stcTim3PortCmpCfg);  //比较输出端口配置
    
    stcTim3TrigAdc.bEnTrigADC    = TRUE;                      //使能ADC触发全局控制
    stcTim3TrigAdc.bEnUevTrigADC = TRUE;                      //Uev更新触发ADC
    Tim3_M23_TrigADC_Cfg(&stcTim3TrigAdc);                    //触发ADC配置
    
    stcTim3DeadTimeCfg.bEnDeadTime      = TRUE;
    stcTim3DeadTimeCfg.u8DeadTimeValue  = 0xFF;
    Tim3_M23_DT_Cfg(&stcTim3DeadTimeCfg);                     //死区设置
    
    u8ValidPeriod = 1;                                        //事件更新周期设置，0表示三角波每半个周期更新一次，每+1代表延迟半个周期
    Tim3_M23_SetValidPeriod(u8ValidPeriod);                   //间隔周期设置
    
    u16CntValue = 0;
    Tim3_M23_Cnt16Set(u16CntValue);                           //设置计数初值
    
    Tim3_ClearAllIntFlag();                                   //清中断标志
    Tim3_Mode23_EnableIrq(Tim3UevIrq);                        //使能TIM3 UEV更新中断
    EnableNvic(TIM3_IRQn, IrqLevel0, TRUE);                   //TIM3中断使能
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
    App_ClockCfg();                       //时钟初始化
    
    App_Timer3PortCfg();                  //Timer3 Port端口配置
    
    App_Timer3Cfg(0x9000, 0x6000, 0);     //Timer3 配置:周期 0x9000; CH0/1/2通道A比较值0x6000; CH0/1/2通道B比较值互补模式不需要设置
    
    App_AdcCfg();                         //ADC 插队扫描采样配置
    
    Tim3_M23_EnPWM_Output(TRUE, FALSE);   //端口输出使能
    
    Tim3_M23_Run();                       //运行
    
    while (1);
    
    
    
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


