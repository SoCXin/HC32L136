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
 **   - 2019-04-22 Husj    First Version
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "adt.h"
#include "lpm.h"
#include "gpio.h"
#include "flash.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/


/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

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
    stc_gpio_cfg_t         stcTIM5Port;
    stc_gpio_cfg_t         stcButtonPort;

    DDL_ZERO_STRUCT(stcTIM5Port);
    DDL_ZERO_STRUCT(stcButtonPort);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //端口外设时钟使能

    stcTIM5Port.enDir  = GpioDirOut;
    //PA09设置为TIM9_CHA
    Gpio_Init(GpioPortA, GpioPin9, &stcTIM5Port);
    Gpio_SetAfMode(GpioPortA,GpioPin9,GpioAf7);

    //PB00设置为TIM5_CHB
    Gpio_Init(GpioPortB, GpioPin0, &stcTIM5Port);
    Gpio_SetAfMode(GpioPortB,GpioPin0,GpioAf4);

    //PD04设置为GPIO<--SW2控制脚
    stcButtonPort.enDrv  = GpioDrvH;
    stcButtonPort.enDir  = GpioDirIn;
    Gpio_Init(GpioPortD, GpioPin4, &stcButtonPort);
}

///< AdvTimer初始化
void App_AdvTimerInit(uint16_t u16Period, uint16_t u16CHA_PWMDuty, uint16_t u16CHB_PWMDuty)
{
    stc_adt_basecnt_cfg_t     stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIM5ACfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIM5BCfg;

    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIM5ACfg);
    DDL_ZERO_STRUCT(stcAdtTIM5BCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);//ADT外设时钟使能


    stcAdtBaseCntCfg.enCntMode = AdtSawtoothMode;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div4;
    Adt_Init(M0P_ADTIM5, &stcAdtBaseCntCfg);                      //ADT载波、计数模式、时钟配置

    Adt_SetPeriod(M0P_ADTIM5, u16Period);                         //周期设置

    Adt_SetCompareValue(M0P_ADTIM5, AdtCompareA, u16CHA_PWMDuty);  //通用比较基准值寄存器A设置

    Adt_SetCompareValue(M0P_ADTIM5, AdtCompareB, u16CHB_PWMDuty);  //通用比较基准值寄存器B设置

    stcAdtTIM5ACfg.enCap = AdtCHxCompareOutput;            //比较输出
    stcAdtTIM5ACfg.bOutEn = TRUE;                          //CHA输出使能
    stcAdtTIM5ACfg.enPerc = AdtCHxPeriodLow;               //计数值与周期匹配时CHA电平输出低
    stcAdtTIM5ACfg.enCmpc = AdtCHxCompareHigh;             //计数值与比较值A匹配时，CHA电平输出高
    stcAdtTIM5ACfg.enStaStp = AdtCHxStateSelSS;            //CHA起始结束电平由STACA与STPCA控制
    stcAdtTIM5ACfg.enStaOut = AdtCHxPortOutLow;            //CHA起始电平为低
    stcAdtTIM5ACfg.enStpOut = AdtCHxPortOutLow;           //CHA结束电平为高
    stcAdtTIM5ACfg.enDisSel = AdtCHxDisSel2;               //刹车模式选2(LP 刹车)
    stcAdtTIM5ACfg.enDisVal = AdtTIMxDisValLow;
    Adt_CHxXPortCfg(M0P_ADTIM5, AdtCHxA, &stcAdtTIM5ACfg);   //端口CHA配置

    stcAdtTIM5BCfg.enCap = AdtCHxCompareOutput;
    stcAdtTIM5BCfg.bOutEn = TRUE;
    stcAdtTIM5BCfg.enPerc = AdtCHxPeriodInv;               //计数值与周期匹配时CHA电平翻转
    stcAdtTIM5BCfg.enCmpc = AdtCHxCompareInv;              //计数值与比较值A匹配时，CHA电平翻转
    stcAdtTIM5BCfg.enStaStp = AdtCHxStateSelSS;
    stcAdtTIM5BCfg.enStaOut = AdtCHxPortOutLow;
    stcAdtTIM5BCfg.enStpOut = AdtCHxPortOutLow;
    stcAdtTIM5BCfg.enDisSel = AdtCHxDisSel2;
    stcAdtTIM5BCfg.enDisVal = AdtTIMxDisValLow;
    Adt_CHxXPortCfg(M0P_ADTIM5, AdtCHxB, &stcAdtTIM5BCfg);   //端口CHB配置
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
    App_ClockInit();         //时钟初始化

    App_AdvTimerInit(0xC000, 0x8000, 0x4000);  //AdvTimer5初始化
    //配置为锯齿波模式: 初始化周期0xC000, CHA占空比设置0x8000，CHB占空比设置0x4000

    App_AdvTimerPortInit();    //AdvTimer端口初始化

    Adt_StartCount(M0P_ADTIM5); //AdvTimer5运行

#if 1
    //按下user按键(PD04)，则进入低功耗模式，否则在等待
    //注：若芯片处于低功耗模式，则芯片无法使用SWD进行调式和下载功能。
    //如需要再次下载程序，需要将芯片复位，且不按user按键
    while (1 == Gpio_GetInputIO(GpioPortA,GpioPin7));

    //Cfg and goto DeepSleep
    Lpm_GotoDeepSleep(TRUE);
#endif

    while(1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


