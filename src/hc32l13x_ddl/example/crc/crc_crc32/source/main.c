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
 **   - 2016-02-16  1.0  XYZ First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "crc.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
 
/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
uint8_t  au8CrcTestData[8] = {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0};
uint32_t u32TestDataLen    = 8;
uint32_t u32RefCrc32       = 0xA85A34A3;
uint32_t u32ErrRefCrc32    = 0xEEEEEEEE;
/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

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
    uint32_t    u32CrcResult = 0;
    volatile uint8_t u8TestFlag = 0;
    volatile en_result_t enResult = Error;
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralCrc, TRUE);        ///< 开启CRC外设时钟模块
    
    /* 8位位宽下输入数据测试示例 */
    u32CrcResult =  CRC32_Get8(au8CrcTestData, u32TestDataLen);   ///< 产生CRC32编码
    if(u32RefCrc32 == u32CrcResult)                               ///< 判断CRC32编码值是否正确
    {
        u8TestFlag = Ok;
    }    
    ///< 检验待校验数据与校验值是否匹配，如果匹配返回TRUE,否则返回Error
    enResult = CRC32_Check8(au8CrcTestData, u32TestDataLen, u32CrcResult);
    
    /* 16位位宽下输入数据测试示例 */
    u32CrcResult =  CRC32_Get16((uint16_t  *)au8CrcTestData, u32TestDataLen/2);   ///< 产生CRC32编码
    if(u32RefCrc32 == u32CrcResult)                                               ///< 判断CRC32编码值是否正确
    {
        u8TestFlag = Ok;
    }    
    ///< 检验待校验数据与校验值是否匹配，如果匹配返回TRUE,否则返回Error
    enResult = CRC32_Check16((uint16_t  *)au8CrcTestData, u32TestDataLen/2, u32CrcResult);
    
    /* 32位位宽下输入数据测试示例 */
    u32CrcResult =  CRC32_Get32((uint32_t  *)au8CrcTestData, u32TestDataLen/4);   ///< 产生CRC32编码
    if(u32RefCrc32 == u32CrcResult)                                               ///< 判断CRC32编码值是否正确
    {
        u8TestFlag = Ok;
    }    
    ///< 检验待校验数据与校验值是否匹配，如果匹配返回TRUE,否则返回FALSE
    enResult = CRC32_Check32((uint32_t  *)au8CrcTestData, u32TestDataLen/4, u32CrcResult);    
    
    while (1)
    {
        ;
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


