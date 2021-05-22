/******************************************************************************
* Copyright (C) 2017, Huada Semiconductor Co.,Ltd All rights reserved.
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
 **   - 2019-04-22  Husj    First Version
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "adt.h"
#include "adc.h"
#include "gpio.h"
#include "flash.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 ******************************************************************************/


/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint16_t u16AdcResult00;
uint16_t u16AdcResult02;
uint16_t u16AdcResult05;

/******************************************************************************
 * Local type definitions ('typedef')                                         
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/


/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

void Tim4_IRQHandler(void)
{
    //AdvTimer4 下溢中断
    if(TRUE == Adt_GetIrqFlag(M0P_ADTIM4, AdtUDFIrq))
    { 
        //用户可在此处更改PWM的占空比 GCMCR寄存器
        //用户可在此处更改PWM的周期 PERBR寄存器
      
        Adt_ClearIrqFlag(M0P_ADTIM4, AdtUDFIrq);  //AdvTimer4 下溢中断标志清零
    }
}

void Adc_IRQHandler(void)
{
    static uint8_t i;
    //ADC 插队扫描中断
    if(TRUE == Adc_GetIrqStatus(AdcMskIrqJqr))
    {
        u16AdcResult00 = (uint16_t)Adc_GetJqrResult(AdcJQRCH0MUX); //读取通道0(PA00)的采样结果
        u16AdcResult02 = (uint16_t)Adc_GetJqrResult(AdcJQRCH1MUX); //读取通道1(PA02)的采样结果
        u16AdcResult05 = (uint16_t)Adc_GetJqrResult(AdcJQRCH2MUX); //读取通道2(PA05)的采样结果

        if(0 == i)
        {
            //设置GCMCR寄存器，通过缓存传送GCMCR-->GCMAR，改变CHA通道的PWM占空比,CHB通道的PWM占空比也跟随变化
            Adt_SetCompareValue(M0P_ADTIM4, AdtCompareC, 0x5555);
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);
            i++;
        }
        else
        {
            //设置GCMCR寄存器，通过缓存传送GCMCR-->GCMAR，改变CHA通道的PWM占空比,CHB通道的PWM占空比也跟随变化
            Adt_SetCompareValue(M0P_ADTIM4, AdtCompareC, 0x9999);
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);
             i = 0;
        }
        
        Adc_ClrIrqStatus(AdcMskIrqJqr);  //插队扫描中断标志清零
    }
}

///< 时钟初始化
void App_ClockInit(void)
{
    en_flash_waitcycle_t      enFlashWait;
    stc_sysctrl_pll_cfg_t     stcPLLCfg;
    
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
    
    Sysctrl_SysClkSwitch(SysctrlClkPLL);   ///< 时钟切换
}


///< AdvTimer端口初始化
void App_AdvTimerPortInit(void)
{
    stc_gpio_cfg_t         stcTIM4Port;
    stc_gpio_cfg_t         stcTIM5Port;
    stc_gpio_cfg_t         stcTIM6Port;
    
    DDL_ZERO_STRUCT(stcTIM4Port);
    DDL_ZERO_STRUCT(stcTIM5Port);
    DDL_ZERO_STRUCT(stcTIM6Port);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //端口外设时钟使能
    
    stcTIM4Port.enDir  = GpioDirOut;
    stcTIM5Port.enDir  = GpioDirOut;
    stcTIM6Port.enDir  = GpioDirOut;
    
    Gpio_Init(GpioPortA, GpioPin8, &stcTIM4Port);  
    Gpio_SetAfMode(GpioPortA,GpioPin8,GpioAf6);     //PA08设置为TIM4_CHA
    
    Gpio_Init(GpioPortA, GpioPin11, &stcTIM4Port);
    Gpio_SetAfMode(GpioPortA,GpioPin11,GpioAf7);     //PA11设置为TIM4_CHB
#if 0    
    Gpio_Init(GpioPortA, GpioPin4, &stcTIM5Port);
    Gpio_SetAfMode(GpioPortA,GpioPin4,GpioAf5);     //PA04设置为TIM5_CHA
    
    Gpio_Init(GpioPortA, GpioPin5, &stcTIM5Port);
    Gpio_SetAfMode(GpioPortA,GpioPin5,GpioAf5);     //PA05设置为TIM5_CHB

    Gpio_Init(GpioPortB, GpioPin11, &stcTIM6Port);
    Gpio_SetAfMode(GpioPortB,GpioPin11,GpioAf5);    //PB11设置为TIM6_CHA
    
    Gpio_Init(GpioPortB, GpioPin1, &stcTIM6Port);
    Gpio_SetAfMode(GpioPortB,GpioPin1,GpioAf4);     //PB01设置为TIM6_CHB

#endif
    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &stcTIM4Port);
}


///< AdvTimer初始化
void App_AdvTimerInit(uint16_t u16Period, uint16_t u16CHA_PWMDuty,\
                      uint16_t u16CHB_PWMDuty, uint16_t u16Deadtime)
{
    stc_adt_basecnt_cfg_t     stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIM4ACfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIM4BCfg;
    stc_adt_validper_cfg_t    stcAdtValidPerCfg;
    stc_adt_irq_trig_cfg_t    stcAdtIrqTrigCfg;


    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIM4ACfg);
    DDL_ZERO_STRUCT(stcAdtTIM4BCfg);
    DDL_ZERO_STRUCT(stcAdtValidPerCfg);
    DDL_ZERO_STRUCT(stcAdtIrqTrigCfg);
        
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);  //ADT外设时钟使能    
    
    stcAdtBaseCntCfg.enCntMode = AdtTriangleModeA;             //三角波A
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0;
    
    Adt_Init(M0P_ADTIM4, &stcAdtBaseCntCfg);                      //ADT载波、计数模式、时钟配置
    
    Adt_SetPeriod(M0P_ADTIM4, u16Period);                         //周期设置
    Adt_SetPeriodBuf(M0P_ADTIM4, u16Period);                      //周期缓存值设置并打开周期的缓存传送功能
    
    Adt_SetCompareValue(M0P_ADTIM4, AdtCompareA, u16CHA_PWMDuty);    //通用比较基准值寄存器A设置
    
    //u16Compare = 0;
    Adt_SetCompareValue(M0P_ADTIM4, AdtCompareB, u16CHB_PWMDuty);    //通用比较基准值寄存器B设置
    
    Adt_SetCompareValue(M0P_ADTIM4, AdtCompareC, u16CHA_PWMDuty);    //通用比较基准值寄存器C设置

    Adt_EnableValueBuf(M0P_ADTIM4, AdtCHxA, TRUE);
    //Adt_EnableValueBuf(M0P_ADTIM4, AdtCHxB, TRUE);

    stcAdtTIM4ACfg.enCap = AdtCHxCompareOutput;            //比较输出
    stcAdtTIM4ACfg.bOutEn = TRUE;                          //CHA输出使能
    stcAdtTIM4ACfg.enPerc = AdtCHxPeriodKeep;              //计数值与周期匹配时CHA电平保持不变
    stcAdtTIM4ACfg.enCmpc = AdtCHxCompareInv;              //计数值与比较值A匹配时，CHA电平翻转
    stcAdtTIM4ACfg.enStaStp = AdtCHxStateSelSS;            //CHA起始结束电平由STACA与STPCA控制
    stcAdtTIM4ACfg.enStaOut = AdtCHxPortOutLow;            //CHA起始电平为低
    stcAdtTIM4ACfg.enStpOut = AdtCHxPortOutLow;            //CHA结束电平为低
    Adt_CHxXPortCfg(M0P_ADTIM4, AdtCHxA, &stcAdtTIM4ACfg); //端口CHA配置
    
    stcAdtTIM4BCfg.enCap = AdtCHxCompareOutput;
    stcAdtTIM4BCfg.bOutEn = TRUE;
    stcAdtTIM4BCfg.enPerc = AdtCHxPeriodKeep;
    stcAdtTIM4BCfg.enCmpc = AdtCHxCompareInv;
    stcAdtTIM4BCfg.enStaStp = AdtCHxStateSelSS;
    stcAdtTIM4BCfg.enStaOut = AdtCHxPortOutHigh;               //CHA起始电平为高
    stcAdtTIM4BCfg.enStpOut = AdtCHxPortOutLow;
    Adt_CHxXPortCfg(M0P_ADTIM4, AdtCHxB, &stcAdtTIM4BCfg);   //端口CHB配置

    Adt_SetDTUA(M0P_ADTIM4, u16Deadtime);
    Adt_SetDTDA(M0P_ADTIM4, u16Deadtime);
    Adt_CfgDT(M0P_ADTIM4, TRUE, TRUE);                       //死区配置
    
    stcAdtIrqTrigCfg.bAdtUnderFlowTrigEn = TRUE;
    Adt_IrqTrigCfg(M0P_ADTIM4, &stcAdtIrqTrigCfg);           //下溢中断触发ADC转换
    
    stcAdtValidPerCfg.enValidCdt = AdtPeriodCnteMax;            //以三角波波峰为计数条件
    stcAdtValidPerCfg.enValidCnt = AdtPeriodCnts1;              //每隔1个周期有效
    Adt_SetValidPeriod(M0P_ADTIM4, &stcAdtValidPerCfg);         //配置有效周期间隔寄存器
    
    Adt_ClearAllIrqFlag(M0P_ADTIM4);
    Adt_CfgIrq(M0P_ADTIM4, AdtUDFIrq, TRUE);  //下溢中断配置
    EnableNvic(ADTIM4_IRQn, IrqLevel3, TRUE);
}

///< ADC插队扫描初始化
void APP_AdcInit(void)
{
    stc_adc_cfg_t              stcAdcCfg;
    stc_adc_jqr_cfg_t          stcAdcJqrCfg;
    
    DDL_ZERO_STRUCT(stcAdcCfg);
    DDL_ZERO_STRUCT(stcAdcJqrCfg);
        
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    Gpio_SetAnalogMode(GpioPortA, GpioPin0);        //PA00
    Gpio_SetAnalogMode(GpioPortA, GpioPin2);        //PA02    
    Gpio_SetAnalogMode(GpioPortA, GpioPin5);        //PA05
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
    
    //ADC配置
    Adc_Enable();
    M0P_BGR->CR_f.BGR_EN = 0x1u;//BGR必须使能
    M0P_BGR->CR_f.TS_EN  = 0x0u;
    delay100us(10);
    
    stcAdcCfg.enAdcMode         = AdcScanMode;          //连续采样模式
    stcAdcCfg.enAdcClkDiv       = AdcMskClkDiv2;        //Adc工作时钟 PCLK/2
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle8Clk;  //采样时钟 8个周期
    stcAdcCfg.enAdcRefVolSel    = AdcMskRefVolSelAVDD;  //内部AVDD
    stcAdcCfg.enAdcOpBuf        = AdcMskBufDisable;     //内部电压跟随器关闭
    stcAdcCfg.enInRef           = AdcMskInRefDisable;   //内部参考电压Disable
    
    Adc_Init(&stcAdcCfg);                               //Adc初始化
    
    Adc_CfgJqrChannel(AdcJQRCH0MUX, AdcExInputCH0);     //配置插队扫描转换通道
    Adc_CfgJqrChannel(AdcJQRCH1MUX, AdcExInputCH2);
    Adc_CfgJqrChannel(AdcJQRCH2MUX, AdcExInputCH5);     //ADC采样顺序CH2 --> CH1 --> CH0
    
    stcAdcJqrCfg.bJqrDmaTrig = FALSE;
    stcAdcJqrCfg.u8JqrCnt = 3;                       //转换起始通道(3-1已在库函数内计算)
    Adc_JqrModeCfg(&stcAdcJqrCfg);                   //配置插队扫描转换模式
    
    Adc_JqrExtTrigCfg(AdcMskTrigTimer4, TRUE);          //Timer4触发插队扫描转换
    
    Adc_ClrIrqStatus(AdcMskIrqJqr);
    Adc_EnableIrq();                                    //使能Adc中断
    EnableNvic(ADC_IRQn, IrqLevel1, TRUE);          //Adc开中断
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
    stc_adt_sw_sync_t         stcAdtSwSync;

    DDL_ZERO_STRUCT(stcAdtSwSync);
    
    App_ClockInit();     //时钟初始化
    
    App_AdvTimerInit(0xEEEE, 0x9999, 0, 0x1000);//AdvTimer初始化
    //配置为三角波模式: 初始化周期0xEEEE, CHA占空比设置0x9999，CHB占空比设置0,死区时间0x1000
    
    App_AdvTimerPortInit();    //AdvTimer端口初始化
    
    APP_AdcInit();             //ADC插队扫描初始化
    
    stcAdtSwSync.bAdTim4 = TRUE;    //Timer4软件同步使能
    Adt_SwSyncStart(&stcAdtSwSync);
    
    while(1)
    {
        //用户可在此处更改PWM的占空比 GCMCR寄存器
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


