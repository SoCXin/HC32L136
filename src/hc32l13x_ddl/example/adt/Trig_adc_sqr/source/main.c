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
uint16_t u16AdcRestult0;
uint16_t u16AdcRestult2;
   
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

void Adc_IRQHandler(void)
{
    //ADC 顺序扫描采样中断
    if(TRUE == Adc_GetIrqStatus(AdcMskIrqSqr))
    {
        u16AdcRestult0 = (uint16_t)Adc_GetSqrResult(AdcSQRCH0MUX); //读取通道0(PA00)的采样结果
        u16AdcRestult2 = (uint16_t)Adc_GetSqrResult(AdcSQRCH1MUX); //读取通道1(PA02)的采样结果

        Adc_ClrIrqStatus(AdcMskIrqSqr);  //顺序扫描中断标志清零
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

///< AdvTimer初始化
void App_AdvTimerInit(uint16_t u16Period)
{
    stc_adt_basecnt_cfg_t     stcAdtBaseCntCfg;
    stc_adt_irq_trig_cfg_t    stcAdtIrqTrigCfg;
    
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtIrqTrigCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);  //ADT外设时钟使能
    
    //ADT Timer4触发ADC采样，Frequency: 1K
    stcAdtBaseCntCfg.enCntMode = AdtSawtoothMode;              //锯齿波
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0;                //PCLK
    
    Adt_Init(M0P_ADTIM4, &stcAdtBaseCntCfg);                   //ADT载波、计数模式、时钟配置
    
    Adt_SetPeriod(M0P_ADTIM4, u16Period);                      //周期设置

    stcAdtIrqTrigCfg.bAdtOverFlowTrigEn = TRUE;
    Adt_IrqTrigCfg(M0P_ADTIM4, &stcAdtIrqTrigCfg);          //上溢中断触发ADC转换
}


///< ADC初始化
void App_AdcInit(void)
{
    stc_adc_cfg_t              stcAdcCfg;
    stc_adc_sqr_cfg_t          stcAdcSqrCfg;
    
    DDL_ZERO_STRUCT(stcAdcCfg);
    DDL_ZERO_STRUCT(stcAdcSqrCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    Gpio_SetAnalogMode(GpioPortA, GpioPin0);        //PA00
    Gpio_SetAnalogMode(GpioPortA, GpioPin2);        //PA02
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
    
    //ADC配置
    Adc_Enable();
    M0P_BGR->CR_f.BGR_EN = 0x1u;  //BGR必须使能
    M0P_BGR->CR_f.TS_EN  = 0x0u;
    delay100us(10);
    
    stcAdcCfg.enAdcMode         = AdcScanMode;          //连续采样模式
    stcAdcCfg.enAdcClkDiv       = AdcMskClkDiv2;        //Adc工作时钟 PCLK/2
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle8Clk;  //采样时钟 8个周期
    stcAdcCfg.enAdcRefVolSel    = AdcMskRefVolSelAVDD;  //内部AVDD
    stcAdcCfg.enAdcOpBuf        = AdcMskBufDisable;     //内部电压跟随器关闭
    stcAdcCfg.enInRef           = AdcMskInRefDisable;   //内部参考电压Disable
    
    Adc_Init(&stcAdcCfg);                               //Adc初始化
    
    Adc_CfgSqrChannel(AdcSQRCH0MUX, AdcExInputCH0);     //配置顺序扫描转换通道
    Adc_CfgSqrChannel(AdcSQRCH1MUX, AdcExInputCH2);     //扫描顺序CH1 --> CH0
    
    stcAdcSqrCfg.bSqrDmaTrig = FALSE;
    stcAdcSqrCfg.enResultAcc = AdcResultAccDisable;
    stcAdcSqrCfg.u8SqrCnt = 2;                       //起始转换通道(2-1已在库函数内计算)
    Adc_SqrModeCfg(&stcAdcSqrCfg);                   //配置顺序扫描转换模式
    
    Adc_SqrExtTrigCfg(AdcMskTrigTimer4, TRUE);          //Timer4触发顺序扫描转换
    
    Adc_ClrIrqStatus(AdcMskIrqSqr);
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
    App_ClockInit();          //时钟初始化
    

    App_AdvTimerInit(48000);  //AdvTimer初始化, 周期1K: 48M * 1000us = 48000
    
    App_AdcInit();            //ADC初始化
    
    Adt_StartCount(M0P_ADTIM4);   //timer4启动

    while(1)
    {
        //用户可在此处更改PWM的占空比 GCMCR寄存器
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


