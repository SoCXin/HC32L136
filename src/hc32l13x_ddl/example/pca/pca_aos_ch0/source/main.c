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
 **   - 2016-02-16  1.0  XYZ First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "pca.h"
#include "lpm.h"
#include "gpio.h"
#include "vc.h"
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

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
__IO uint8_t rising_flag;
__IO uint8_t falling_flag;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void App_GpioInit(void);
static void App_VcInit(void);
static void App_PcaInit(void);

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/


/**
******************************************************************************
    ** \brief  主函数
    ** 
    ** @param  无
    ** \retval 无
    **
******************************************************************************/ 
int32_t main(void)
{  
    App_GpioInit();
    App_VcInit();
    App_PcaInit();

    Pca_StartPca(TRUE);                               //使能PCA开始计数
    while(1)
    {
    }
}


/**
******************************************************************************
    ** \brief  RTC中断入口函数
    ** 
    ** @param  无
    ** \retval 无
    **
******************************************************************************/ 
void Pca_IRQHandler(void)
{
    if(Pca_GetItStatus(PcaCcf0) != FALSE)    //捕获到PCA_CH0的上升沿或下降沿
    {
        Pca_ClrItStatus(PcaCcf0);              //清除中断标志位
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
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    //PB14设置为VC1的P端输入
    GpioInitStruct.enDrv  = GpioDrvH;
    GpioInitStruct.enDir  = GpioDirIn;
    Gpio_Init(GpioPortB, GpioPin14, &GpioInitStruct);
    
    //PA07设置为VC1_OUT
    GpioInitStruct.enDir  = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin7, &GpioInitStruct);
    Gpio_SetAfMode(GpioPortA,GpioPin7,GpioAf6);
    
    Gpio_SfPcaCfg(GpioSfPcaCH0, GpioSf4);            //PCA_CH0 捕获输入选择VC1_OUT
}
/*******************************************************************************
 * VC1 初始化配置程序
 ******************************************************************************/
static void App_VcInit(void)
{
    stc_vc_channel_cfg_t  VcInitStruct;

    DDL_ZERO_STRUCT(VcInitStruct);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralVcLvd, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
    
    Bgr_BgrEnable();                 //BGR必须使能
    delay10us(2);            
    
    VcInitStruct.enVcChannel      = VcChannel1;        //VC的通道1
    VcInitStruct.enVcCmpDly       = VcDelay10mv;       //迟滞电压10mv
    VcInitStruct.enVcBiasCurrent  = VcBias1200na;      //偏置电流1.2ua
    VcInitStruct.enVcFilterTime   = VcFilter14us;      //输出滤波时间14us
    VcInitStruct.enVcInPin_P      = VcInPCh11;         //VC1的P端输入选择PB14
    VcInitStruct.enVcInPin_N      = AiLdo;             //VC1的N端选择内部1.2V基准电压
    VcInitStruct.bFlten           = TRUE;              //滤波输出使能
    Vc_Init(&VcInitStruct);    
    Vc_Cmd(VcChannel1, TRUE);                          //VC1使能
}
/**
 ******************************************************************************
 ** \brief  配置PCA
 **
 ** \return 无
 ******************************************************************************/
static void App_PcaInit(void)
{
    stc_pcacfg_t  PcaInitStruct;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralPca, TRUE);
    
    PcaInitStruct.pca_clksrc = PcaPclkdiv32;
    PcaInitStruct.pca_cidl   = FALSE;
    PcaInitStruct.pca_ecom   = PcaEcomDisable;       //禁止比较器功能
    PcaInitStruct.pca_capp   = PcaCappEnable;        //允许上升沿捕获
    PcaInitStruct.pca_capn   = PcaCapnEnable;        //允许下降沿捕获
    PcaInitStruct.pca_mat    = PcaMatDisable;        //禁止匹配功能
    PcaInitStruct.pca_tog    = PcaTogDisable;        //禁止翻转控制功能
    PcaInitStruct.pca_pwm    = PcaPwm8bitDisable;    //禁止PWM控制输出
    PcaInitStruct.pca_epwm   = PcaEpwmDisable;       //禁止16bitPWM输出
    Pca_M0Init(&PcaInitStruct);  

    Pca_ClrItStatus(PcaCcf0);                     //清除PCA_CH0的中断
    Pca_ConfModulexIt(PcaModule0, TRUE);          //使能PCA模块0中断
    EnableNvic(PCA_IRQn, IrqLevel3, TRUE);         //使能PCA中断向量    
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


