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
 **   - 2016-02-16  1.0   First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "pca.h"
#include "lpm.h"
#include "gpio.h"
#include "pcnt.h"
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

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
static void App_GpioInit(void);
static void App_PcntInit(void);

/**
 ******************************************************************************
 ** \brief  中断入口函数
 **
 ** \return 无
 ******************************************************************************/
void Pcnt_IRQHandler(void)
{
    Pcnt_ClrItStatus(PcntOV);
    
    if(Gpio_GetInputIO(STK_LED_PORT, STK_LED_PIN) == FALSE)
    {
        Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);
    }
    else if(Gpio_GetInputIO(STK_LED_PORT, STK_LED_PIN) == TRUE)
    {
        Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);
    }    
}

/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
static void App_GpioInit(void)
{
    stc_gpio_cfg_t GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirIn;
    Gpio_Init(GpioPortB,GpioPin5,&GpioInitStruct);
    Gpio_SetAfMode(GpioPortB,GpioPin5,GpioAf6);              //PB05作为PCNT_S0

    GpioInitStruct.enDir = GpioDirOut;
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &GpioInitStruct);   //PD14配置成输出，控制板上蓝色LED
    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);    
}

/**
 ******************************************************************************
 ** \brief  配置PCNT
 **
 ** \return 无
 ******************************************************************************/
static void App_PcntInit(void)
{
    stc_pcnt_initstruct_t PcntInitStruct;
    DDL_ZERO_STRUCT(PcntInitStruct);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralPcnt, TRUE);
    
    PcntInitStruct.Pcnt_S0Sel = PcntS0PNoinvert;  //S0输入极性不取反
    PcntInitStruct.Pcnt_Clk   = PcntCLKPclk;      //采样时钟
    PcntInitStruct.Pcnt_Mode  = PcntSingleMode; //单通道脉冲计数模式
    PcntInitStruct.Pcnt_FltEn = TRUE;          //滤波使能
    PcntInitStruct.Pcnt_DebTop = 5;            //滤波计数器阈值
    PcntInitStruct.Pcnt_ClkDiv = 5;            //滤波时钟分频系数
    PcntInitStruct.Pcnt_TocrEn = FALSE;        //超时控制使能
    PcntInitStruct.Pcnt_Dir    = PcntDirUp;   //向上计数
    Pcnt_Init(&PcntInitStruct);
    
    Pcnt_SetB2T(10);
    Pcnt_ClrItStatus(PcntOV);
    Pcnt_ItCfg(PcntOV, TRUE);
    EnableNvic(PCNT_IRQn, IrqLevel3, TRUE);    
}

/**
 ******************************************************************************
 ** \brief  主函数
 **
 ** \return 无
 ******************************************************************************/
int32_t main(void)
{       
    App_GpioInit();
    App_PcntInit();

    Pcnt_Cmd(TRUE);                                      //使能PCNT
    while (1)
    {
        
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


