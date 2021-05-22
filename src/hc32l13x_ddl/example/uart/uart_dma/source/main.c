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
 **   - 2017-05-17  1.0  cj First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "uart.h"
#include "gpio.h"
#include "sysctrl.h"
#include "dmac.h"

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
uint8_t u8TxData[8] = {0xAA,0x55};
uint8_t u8RxData[8]={0X00};

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                             
 ******************************************************************************/
 
/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
 void App_UartCfg(void);
 void App_DmaCfg(void);
 void App_UartPortInit(void);

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
    //uart端口配置
    App_UartPortInit();
    
    //UART模块配置
    App_UartCfg();
    
    //DMA参数配置
    App_DmaCfg();
    
    while(1)
    {
        if(5 == Dma_GetStat(DmaCh0))                  //完成一次通道传输（UART1_RX -> RAM缓存  2字节）
        {
            Dma_DisableChannel(DmaCh0);               //禁用通道0
            delay10us(100);
            Dma_EnableChannel(DmaCh1);                //使能通道1
            Dma_ClrStat(DmaCh0);                      //清除通道0状态值    
        } 
        if(5 == Dma_GetStat(DmaCh1))                  //完成一次通道传输（RAM缓存 -> UART1_TX  2字节）
        {
            Dma_DisableChannel(DmaCh1);               //禁用通道1
            Dma_EnableChannel(DmaCh0);                //使能通道0
            Dma_ClrStat(DmaCh1);                      //清除通道0状态值    
        }     
    }
}

//UART模块配置
void App_UartCfg(void)
{
    stc_uart_cfg_t  stcCfg;
    stc_uart_baud_t stcBaud;

    DDL_ZERO_STRUCT(stcCfg);                               //初始化变量
    DDL_ZERO_STRUCT(stcBaud);                              //初始化变量
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);//使能UART1外设时钟门控开关

    stcCfg.enRunMode = UartMskMode3;                       //模式3
    stcCfg.enStopBit = UartMsk1bit;                        //1位停止位
    stcCfg.enMmdorCk = UartMskEven;                        //偶校验
    stcCfg.stcBaud.u32Baud = 9600;                         //波特率9600
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;             //通道采样分频配置
    stcCfg.stcBaud.u32Pclk = Sysctrl_GetPClkFreq();        //获得外设时钟（PCLK）频率值
    Uart_Init(M0P_UART1, &stcCfg);                         //串口初始化

    Uart_ClrStatus(M0P_UART1,UartRC);                      //清接收请求
    Uart_ClrStatus(M0P_UART1,UartTC);                      //清发送请求
        
    Uart_EnableFunc(M0P_UART1,UartDmaTxFunc);              //使能DMA发送, DMA相关通道使能后，如果Tx Buff为空，会立马启动传输
    Uart_EnableFunc(M0P_UART1,UartDmaRxFunc);              //使能DMA接收
}

//DMA参数配置
void App_DmaCfg(void)
{
    stc_dma_cfg_t stcDmaCfg;
    
    DDL_ZERO_STRUCT(stcDmaCfg);                                //初始化变量

    Sysctrl_SetPeripheralGate(SysctrlPeripheralDma,TRUE);      //使能DMAC外设时钟门控开关
    
    //rx dma配置
    stcDmaCfg.u32SrcAddress = 0x40000100;                      //接收数据寄存器地址
    stcDmaCfg.u32DstAddress = (uint32_t)&u8RxData[0];          //接收缓冲
    stcDmaCfg.enSrcAddrReloadCtl = DmaMskSrcAddrReloadEnable;  //使能DMA源地址重载
    stcDmaCfg.enSrcBcTcReloadCtl = DmaMskBcTcReloadEnable;     //使能BC[3:0]和CONFA:TC[15:0]的重载功能
    stcDmaCfg.enDestAddrReloadCtl = DmaMskDstAddrReloadEnable; //使能DMA目的地址重载
    stcDmaCfg.enTransferMode = DmaMskContinuousTransfer;       //连续传输，DMAC传输完成时不清除CONFA:ENS位
    stcDmaCfg.enDstAddrMode = DmaMskDstAddrInc;                //目的地址自增
    stcDmaCfg.enSrcAddrMode = DmaMskSrcAddrFix;                //源地址固定
    stcDmaCfg.u16BlockSize = 1;                                //块传输个数
    stcDmaCfg.u16TransferCnt = 2;                              //块传输次数
    stcDmaCfg.enMode = DmaMskBlock;                            //块(Block)传输
    stcDmaCfg.enTransferWidth = DmaMsk8Bit;                    // 8 bit  字节传输
    stcDmaCfg.enRequestNum = DmaUart1RxTrig;                   //DMA硬件触发源位Uart1Rx
    stcDmaCfg.enPriority = DmaMskPriorityFix;                  //DMA 各通道优先级固定 (CH0>CH1)
    
    Dma_Enable();                                              //DMA模块使能
    Dma_InitChannel(DmaCh0, &stcDmaCfg);                       //DMA通道0初始化
        
    DDL_ZERO_STRUCT(stcDmaCfg);                                //初始化变量
        
    //tx dma配置
    stcDmaCfg.u32SrcAddress = (uint32_t)&u8RxData[0];          //接收数据缓存
    stcDmaCfg.u32DstAddress = 0x40000100;                      //发送数据寄存器地址
    stcDmaCfg.enSrcAddrReloadCtl = DmaMskSrcAddrReloadEnable;  //使能DMA源地址重载
    stcDmaCfg.enSrcBcTcReloadCtl = DmaMskBcTcReloadEnable;     //使能BC[3:0]和CONFA:TC[15:0]的重载功能
    stcDmaCfg.enDestAddrReloadCtl = DmaMskDstAddrReloadEnable; //使能DMA目的地址重载
    stcDmaCfg.enTransferMode = DmaMskOneTransfer;              //一次传输，DMAC传输完成时清除CONFA:ENS位
    stcDmaCfg.enDstAddrMode = DmaMskDstAddrFix;                //目的地址固定
    stcDmaCfg.enSrcAddrMode = DmaMskSrcAddrInc;                //源地址自增
    stcDmaCfg.u16BlockSize = 1;                                //块传输个数
    stcDmaCfg.u16TransferCnt = 2;                              //块传输次数，一次传输一个字节，传输两次
    stcDmaCfg.enMode = DmaMskBlock;                            //块(Block)传输
    stcDmaCfg.enTransferWidth = DmaMsk8Bit;                    // 8 bit  字节传输
    stcDmaCfg.enRequestNum = DmaUart1TxTrig;                   //DMA硬件触发源位Uart1Tx
    stcDmaCfg.enPriority = DmaMskPriorityFix;                  //DMA 各通道优先级固定 (CH0>CH1)

    Dma_InitChannel(DmaCh1, &stcDmaCfg);                       //DMA通道1初始化
    Dma_EnableChannel(DmaCh0);                                 //使能通道1
}

//uart端口配置
void App_UartPortInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    DDL_ZERO_STRUCT(stcGpioCfg);                               //初始化变量
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);     //使能GPIO外设时钟门控开关
    
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortA,GpioPin2,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin2,GpioAf1);                //PA02 配置为UART1 TX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortA,GpioPin3,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin3,GpioAf1);                //PA03 配置为UART1 RX
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


