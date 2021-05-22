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
 **   - 2017-05-28 LiuHL    First Version
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "vc.h" 
#include "gpio.h"
#include "adc.h"
#include "bt.h"
#include "bgr.h"
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
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief  中断服务程序
 **
 ** \return 无
 ******************************************************************************/
void Tim0_IRQHandler(void)
{ 
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
        if(Gpio_GetInputIO(STK_LED_PORT, STK_LED_PIN) == FALSE)
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);
        }
        else if(Gpio_GetInputIO(STK_LED_PORT, STK_LED_PIN) == TRUE)
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);
        }
        Bt_ClearIntFlag(TIM0,BtUevIrq);
    }
}

/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
void GPIO_Cfg(void)
{
    stc_gpio_cfg_t GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);
    
    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirOut;
    Gpio_Init(GpioPortA,GpioPin6,&GpioInitStruct);
    Gpio_SetAfMode(GpioPortA,GpioPin6,GpioAf5);              //PA06作为VC0_OUT

    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &GpioInitStruct);   //PD14配置成输出，控制板上蓝色LED
    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);    
    
    Gpio_SetAnalogMode(GpioPortC,GpioPin0);                  //模拟输入
    
    Gpio_SfTimGCfg(GpioSfTim0G,GpioSf3);
}

/**
 ******************************************************************************
 ** \brief  初始化VC0
 **
 ** \return 无
 ******************************************************************************/
void VC_Cfg(void)
{
    stc_vc_channel_cfg_t VcInitStruct;
    DDL_ZERO_STRUCT(VcInitStruct);
    VcInitStruct.enVcChannel = VcChannel0;
    VcInitStruct.enVcCmpDly  = VcDelay10mv;          //VC0迟滞电压约为10mV
    VcInitStruct.enVcBiasCurrent = VcBias10ua;       //VC0功耗为10uA
    VcInitStruct.enVcFilterTime  = VcFilter28us;     //VC输出滤波时间约为28us
    VcInitStruct.enVcInPin_P     = VcInPCh0;         //VC0_CH的P端连接PC00
    VcInitStruct.enVcInPin_N     = AiBg1p2;          //VC0_CH的N端连接内核1.2V
    VcInitStruct.enVcOutCfg   = VcOutTIMBK;       //作为定时器刹车
    VcInitStruct.bFlten          = TRUE;             //使能滤波
    Vc_Init(&VcInitStruct);
}

/**
 ******************************************************************************
 ** \brief  初始化TIM2
 **
 ** \return 无
 ******************************************************************************/
void TIM0M0_Cfg(void)
{
    stc_bt_mode0_cfg_t     BtInitStruct;    
    uint16_t                  u16ArrValue;
    uint16_t                  u16CntValue;    
    DDL_ZERO_STRUCT(BtInitStruct);

    BtInitStruct.enWorkMode = BtWorkMode0;                  //定时器模式0
    BtInitStruct.enCT       = BtTimer;                      //定时器功能，计数时钟为内部PCLK
    BtInitStruct.enPRS      = BtPCLKDiv8;                   //PCLK/8
    BtInitStruct.enCntMode  = Bt16bitArrMode;               //自动重载16位计数器/定时器
    BtInitStruct.bEnTog     = FALSE;                        //翻转输出关闭CHA、CHB输出均为低电平
    BtInitStruct.bEnGate    = TRUE;                         //门口使能，端口GATE有效且定时器使能才工作
    BtInitStruct.enGateP    = BtGatePositive;               //端口GATE低电平有效
    Bt_Mode0_Init(TIM0, &BtInitStruct);                     //TIM0 的模式0功能初始化

    Bt_ClearIntFlag(TIM0,BtUevIrq);                         //清中断标志
    EnableNvic(TIM0_IRQn, IrqLevel3, TRUE);                 //使能中断向量
    Bt_Mode0_EnableIrq(TIM0);                               //使能TIM0中断——溢出中断
    //计数范围：6000-0xffff
    u16ArrValue = 6000;
    Bt_M0_ARRSet(TIM0, u16ArrValue);                        //设置重载值
    u16CntValue = 6000;
    Bt_M0_Cnt16Set(TIM0, u16CntValue);                      //设置计数初值
}

int main(void)
{
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);//开GPIO时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralVcLvd, TRUE);//开LVD时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);//开adc时钟  
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能  
    M0P_BGR->CR_f.BGR_EN = 0x1u;                 //BGR必须使能
    Bgr_BgrEnable();                 //BGR必须使能
 
    VC_Cfg();                      //配置VC
    GPIO_Cfg();                    //配置GPIO
    TIM0M0_Cfg();                  //配置TIM0
    Vc_Cmd(VcChannel0, TRUE);         //使能VC0
    Bt_M0_Run(TIM0);                  //使能TIM0工作
    while (1)
    {

    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


