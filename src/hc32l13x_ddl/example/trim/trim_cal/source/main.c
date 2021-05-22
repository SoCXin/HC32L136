/******************************************************************************
* Copyright (C) 2018, Huada Semiconductor Co.,Ltd All rights reserved.
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
 **   - 2018-05-10  1.0  Lux First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "trim.h"
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

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static volatile uint8_t u8TrimTestFlag   = 0;
/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
static void App_SysClkInit(void);
static void App_TrimCalInit(void);
static void App_UserKeyInit(void);

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return int32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
int32_t main(void)
{
    ///< 系统时钟初始化
    App_SysClkInit();
    ///< 按键功能初始化
    App_UserKeyInit();
    ///< Trim 校准功能初始化
    App_TrimCalInit();    
    
    ///< 等待USER KEY按下(低电平)后程序继续执行
    while(TRUE == Gpio_GetInputIO(STK_USER_PORT, STK_USER_PIN));
    
    ///< 开启TRIM校准流程
    Trim_Run();
    
    while(1)
    {
        ///< 校准结束,此时的TRIM值即为最佳频率值
        if(0xFF == u8TrimTestFlag)
        {
            break;
        }
    }
    
    while(1);
}


/*******************************************************************************
 * TRIM 中断服务程序
 ******************************************************************************/   
void ClkTrim_IRQHandler(void)
{
    volatile uint32_t u32CalCnt;
    
    if(Trim_GetIntFlag(TrimStop))
    {
        u32CalCnt = Trim_CalCntGet();
        ///<参考计数值计数完成（10ms)时，查看待校准计数值是否也为（10ms）计数值,或是否在允许误差范围内，此处为32768/100 = 328(±0.3%)
        ///<可根据实际修改该比较范围，提高TRIM校准精度。
        if ((u32CalCnt <= (328u + 1u)) &&
            (u32CalCnt >= (328u - 1u)))
        {
            Trim_Stop();
            ///< 校准结束,此时的TRIM值即为最佳频率值
            u8TrimTestFlag = 0xFFu;
        }
        else
        {
            Trim_Stop();
            ///< 为达到目标精度，TRIM值增加1，继续校准
            M0P_SYSCTRL->RCL_CR_f.TRIM += 1;       
            Trim_Run();           
        }
        
    }
    
    if(Trim_GetIntFlag(TrimCalCntOf))  //参考校准时间设置过长，导致待校准计数器溢出，此时需要重新配置参考校准时间及校准精度
    {
        u8TrimTestFlag = 0;
    }
   
}

static void App_UserKeyInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    ///< 端口方向配置->输出
    stcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->低驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->上拉
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    stcGpioCfg.enCtrlMode = GpioAHB;
    
    ///< GPIO IO USER KEY初始化
    Gpio_Init(STK_USER_PORT, STK_USER_PIN, &stcGpioCfg);
    
}

static void App_SysClkInit(void)
{    
    //设置初始TRIM值,本样例采用从小到大的方式来TRIM，故此处设置为'1',后续不断更新该值后进行TRIM，直到符合条件（校准）为止
    M0P_SYSCTRL->RCL_CR_f.TRIM = 1;
    ///< 使能待校准时钟，本样例对RCL 32768Hz进行校准
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
    
    ///< 使能被监测时钟，本样例使用XTL
    ///< XTL 配置及使能
    Sysctrl_XTLDriverCfg(SysctrlXtlAmp3, SysctrlXtalDriver3);
    Sysctrl_SetXTLStableTime(SysctrlXtlStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTL, TRUE); 
}

static void App_TrimCalInit(void)
{
    stc_trim_cfg_t stcCfg;

    //打开TRIM外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTrim, TRUE);
     
    //TRIM校准流程
    stcCfg.enMON     = TrimMonDisable;
    stcCfg.enREFCLK  = TrimRefMskXTL;
    stcCfg.enCALCLK  = TrimCalMskRCL;
    stcCfg.u32RefCon = 328u;                        //10ms校准时间（增加该时间可提高TRIM精度）
    stcCfg.u32CalCon = 0xFFFFFFFFu;                 //配置为默认值
    Trim_Init(&stcCfg);
    
    ///< 打开TRIM中断使能
    Trim_EnableIrq();
    ///< 使能并配置TRIM 系统中断
    EnableNvic(CLKTRIM_IRQn, IrqLevel3, TRUE);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


