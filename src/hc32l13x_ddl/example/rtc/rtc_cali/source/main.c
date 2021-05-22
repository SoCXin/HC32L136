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
 **   - 2016-02-16  1.0  XYZ First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "rtc.h"
#include "sysctrl.h"
#include "gpio.h"


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
void App_GpioCfg(void);
void App_RtcCfg(void);
void App_ClkCfg(void);

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
    // 时钟相关配置
    App_ClkCfg();  

    // 端口相关配置    
    App_GpioCfg(); 

    // RTC寄存器相关配置    
    App_RtcCfg();
    
    // RTC的1Hz输出
    Rtc_Hz1Cmd(RtcHz1selHighPricision, TRUE);
    
    //RTC计数器的使能
    Rtc_Cmd(TRUE);  
    
    while (1)
    {        
        ;
    }
}

//端口相关配置
void App_GpioCfg(void)
{
    stc_gpio_cfg_t GpioInitStruct;
    
    DDL_ZERO_STRUCT(GpioInitStruct);                     //初始值清零
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);//GPIO外设时钟打开
    
    GpioInitStruct.enDir = GpioDirOut;
    Gpio_Init(GpioPortB,GpioPin14,&GpioInitStruct);
    Gpio_SetAfMode(GpioPortB,GpioPin14,GpioAf5);         //1hz输出口PB14
}

//RTC寄存器相关配置
void App_RtcCfg(void)
{
    stc_rtc_initstruct_t RtcInitStruct;
    
    DDL_ZERO_STRUCT(RtcInitStruct);                      //初始值清零
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);//RTC模块时钟打开 
    
    RtcInitStruct.rtcAmpm = RtcPm;                       //12小时制
    RtcInitStruct.rtcClksrc = RtcClkRcl;                 //内部低速时钟
    RtcInitStruct.rtcTime.u8Second = 0x00;               //配置时钟
    RtcInitStruct.rtcTime.u8Minute = 0x45;
    RtcInitStruct.rtcTime.u8Hour   = 0x17;
    RtcInitStruct.rtcTime.u8Day    = 0x16;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x03;
    RtcInitStruct.rtcTime.u8Month  = 0x04;
    RtcInitStruct.rtcTime.u8Year   = 0x19;
    RtcInitStruct.rtcCompen = RtcCompenEnable;           // 使能时钟误差补偿
    RtcInitStruct.rtcCompValue = 0;                      //补偿值  根据实际情况进行补偿
    Rtc_Init(&RtcInitStruct);
}

//时钟配置
void App_ClkCfg(void)
{
    Sysctrl_SetRCHTrim(SysctrlRchFreq24MHz);       //内部高速时钟频率TRIM值加载    
    Sysctrl_SetRTCAdjustClkFreq(SysctrlRTC24MHz);  //补偿高速时钟源   
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);  //使能RCL时钟
}


/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


