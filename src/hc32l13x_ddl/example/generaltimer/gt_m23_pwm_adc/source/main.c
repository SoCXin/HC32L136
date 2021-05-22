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
 **   - 2017-04-17  1.0  Husj First version for General Timer012.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "bt.h"
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
 * TIM0中断服务函数
 ******************************************************************************/
void Tim0_IRQHandler(void)
{
    static uint8_t i;
    //Timer0 模式23 更新中断
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
        if(0 == i)
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);  //LED 引脚输出高电平
            i++;
        }
        else if(1 == i)
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);  //LED 引脚输出低电平
            i = 0;
        }
        
        Bt_ClearIntFlag(TIM0,BtUevIrq);  //清中断标志
    }
}

void Adc_IRQHandler(void)
{
    static uint8_t i;
    
    //ADC 插队扫描JQR中断
    if(TRUE == Adc_GetIrqStatus(AdcMskIrqJqr))
    {
        u16AdcResult02 = (uint16_t)Adc_GetJqrResult(AdcJQRCH0MUX);  //读取插队扫描通道0(PA02)的采样数据
        u16AdcResult03 = (uint16_t)Adc_GetJqrResult(AdcJQRCH1MUX);  //读取插队扫描通道0(PA03)的采样数据
        u16AdcResult05 = (uint16_t)Adc_GetJqrResult(AdcJQRCH2MUX);  //读取插队扫描通道0(PA05)的采样数据
    
        if(0 == i)
        {
            Bt_M23_CCR_Set(TIM0, BtCCR0A, 3600); //设置通道A比较值
            i++;
        }
        else if(1 == i)
        {
            Bt_M23_CCR_Set(TIM0, BtCCR0A, 1200); //设置通道A比较值
            i = 0;
        }
        Adc_ClrIrqStatus(AdcMskIrqJqr);  //清中断标志
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
    stc_gpio_cfg_t             stcTIM0Port;
    stc_gpio_cfg_t             stcLEDPort;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTIM0Port);
    DDL_ZERO_STRUCT(stcLEDPort);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //GPIO 外设时钟使能
    
    stcLEDPort.enDir  = GpioDirOut;
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &stcLEDPort);     //PD14设置为LED输出
    
    stcTIM0Port.enDir  = GpioDirOut;
    
    Gpio_Init(GpioPortA, GpioPin0, &stcTIM0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin0,GpioAf7);            //PA00设置为TIM0_CHA
    
    Gpio_Init(GpioPortA, GpioPin1, &stcTIM0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin1,GpioAf3);            //PA01设置为TIM0_CHB
}

//ADC 插队扫描配置
void App_AdcCfg(void)
{
    stc_adc_cfg_t              stcAdcCfg;
    stc_adc_jqr_cfg_t          stcAdcJqrCfg;
    
    DDL_ZERO_STRUCT(stcAdcCfg);
    DDL_ZERO_STRUCT(stcAdcJqrCfg);
    
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
    
    stcAdcJqrCfg.bJqrDmaTrig = FALSE;
    stcAdcJqrCfg.u8JqrCnt = 3;                      //转换起始通道(3-1已在库函数内计算)
    Adc_JqrModeCfg(&stcAdcJqrCfg);                  //配置插队扫描转换模式
    
    Adc_JqrExtTrigCfg(AdcMskTrigTimer0, TRUE);         //Timer0触发插队扫描转换
    
    Adc_ClrIrqStatus(AdcMskIrqJqr);
    Adc_EnableIrq();                                   //使能Adc中断
    EnableNvic(ADC_IRQn, IrqLevel1, TRUE);         //Adc开中断
}

//Timer0 配置
void App_Timer0Cfg(uint16_t u16Period, uint16_t u16CHxACompare, uint16_t u16CHxBCompare)
{
    uint16_t                   u16CntValue;
    uint8_t                    u8ValidPeriod;
    stc_bt_mode23_cfg_t        stcBtBaseCfg;
    stc_bt_m23_compare_cfg_t   stcBtPortCmpCfg;
    stc_bt_m23_adc_trig_cfg_t  stcBtTrigAdc;
    stc_bt_m23_dt_cfg_t        stcBtDeadTimeCfg;
    
    DDL_ZERO_STRUCT(stcBtBaseCfg);
    DDL_ZERO_STRUCT(stcBtPortCmpCfg);
    DDL_ZERO_STRUCT(stcBtTrigAdc);
    DDL_ZERO_STRUCT(stcBtDeadTimeCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE);   //Base Timer外设时钟使能      
        
    stcBtBaseCfg.enWorkMode    = BtWorkMode3;              //三角波模式
    stcBtBaseCfg.enCT          = BtTimer;                  //定时器功能，计数时钟为内部PCLK
    stcBtBaseCfg.enPRS         = BtPCLKDiv1;               //PCLK
    //stcBtBaseCfg.enCntDir    = BtCntUp;                  //向上计数，在三角波模式时只读
    stcBtBaseCfg.enPWMTypeSel  = BtComplementaryPWM;       //互补输出PWM
    stcBtBaseCfg.enPWM2sSel    = BtSinglePointCmp;         //单点比较功能
    stcBtBaseCfg.bOneShot      = FALSE;                    //循环计数
    stcBtBaseCfg.bURSSel       = FALSE;                    //上下溢更新
    
    Bt_Mode23_Init(TIM0, &stcBtBaseCfg);                   //TIM0 的模式23功能初始化
    
    Bt_M23_ARRSet(TIM0, u16Period, TRUE);                  //设置重载值,并使能缓存

    Bt_M23_CCR_Set(TIM0, BtCCR0A, u16CHxACompare);         //设置比较值A,(PWM互补模式下只需要设置比较值A)
    
    stcBtPortCmpCfg.enCH0ACmpCtrl   = BtPWMMode2;          //OCREFA输出控制OCMA:PWM模式2
    stcBtPortCmpCfg.enCH0APolarity  = BtPortPositive;      //正常输出
    stcBtPortCmpCfg.bCh0ACmpBufEn   = TRUE;                //A通道缓存控制
    stcBtPortCmpCfg.enCh0ACmpIntSel = BtCmpIntNone;        //A通道比较控制:无
    
    stcBtPortCmpCfg.enCH0BCmpCtrl   = BtPWMMode2;          //OCREFB输出控制OCMB:PWM模式2(PWM互补模式下也要设置，避免强制输出)
    stcBtPortCmpCfg.enCH0BPolarity  = BtPortPositive;      //正常输出
    //stcBtPortCmpCfg.bCH0BCmpBufEn   = TRUE;              //B通道缓存控制使能
    stcBtPortCmpCfg.enCH0BCmpIntSel = BtCmpIntNone;        //B通道比较控制:无
    
    Bt_M23_PortOutput_Cfg(TIM0, &stcBtPortCmpCfg);         //比较输出端口配置
    
    stcBtTrigAdc.bEnTrigADC    = TRUE;                     //使能ADC触发全局控制
    stcBtTrigAdc.bEnUevTrigADC = TRUE;                     //Uev更新触发ADC
    Bt_M23_TrigADC_Cfg(TIM0, &stcBtTrigAdc);               //触发ADC配置
    
    u8ValidPeriod = 1;                                     //事件更新周期设置，0表示三角波每半个周期更新一次，每+1代表延迟半个周期
    Bt_M23_SetValidPeriod(TIM0,u8ValidPeriod);             //间隔周期设置
    
    stcBtDeadTimeCfg.bEnDeadTime      = TRUE;
    stcBtDeadTimeCfg.u8DeadTimeValue  = 0xFF;
    Bt_M23_DT_Cfg(TIM0, &stcBtDeadTimeCfg);             //死区配置

    
    u16CntValue = 0;
    
    Bt_M23_Cnt16Set(TIM0, u16CntValue);                    //设置计数初值
    
    Bt_ClearAllIntFlag(TIM0);                              //清中断标志
    Bt_Mode23_EnableIrq(TIM0,BtUevIrq);                    //使能TIM0 UEV更新中断
    EnableNvic(TIM0_IRQn, IrqLevel0, TRUE);                //TIM0中断使能   
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
    
    App_Timer0Cfg(4800, 2400, 0);         //Timer0 配置:周期 4800(5K); 通道A比较值2400; 通道B比较值互补模式不需要设置
    
    App_Timer0PortCfg();                  //Timer3 Port端口配置
        
    App_AdcCfg();                              //Adc 插队扫描配置
    
    Bt_M23_EnPWM_Output(TIM0, TRUE, FALSE);    //TIM0 端口输出使能
    
    Bt_M23_Run(TIM0);                          //TIM0 运行。

    while (1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


