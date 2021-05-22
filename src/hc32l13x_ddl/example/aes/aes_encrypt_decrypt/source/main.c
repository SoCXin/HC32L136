/*******************************************************************************
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
 ** Main Module
 **
 ** \brief This sample shows how to use AES to encrypt or decrypt data.
 **
 ** History:
 **   - 2018-05-15  1.0  Lux        version 1.0
 **
 ******************************************************************************/
 
/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "aes.h"

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
//< 存放待加密的数据
uint32_t u32AESTestData[4] = {0x33221100, 0x77665544, 0xBBAA9988, 0xFFEEDDCC};
//< AEC密匙
uint32_t u32AESTestKey[8]  = {0x03020100, 0x07060504, 0x0B0A0908, 0x0F0E0D0C};
//< 存放加密后的数据
uint32_t pu32Ciphertext[4] = {0};
//< 存放解密后的数据
uint32_t pu32Plaintext[4]  = {0};
/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                             
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
void App_AesKey128Test (void);
/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample shows how to use AES to encrypt or decrypt data.
 **
 ******************************************************************************/
int32_t main(void)
{    
    ///< 使用128位密匙加密和解密测试
    App_AesKey128Test();
    
    while (1) ///< 如果测试失败，则无法运行到此行
    {
        ;
    }
}

///< 使用128位密匙加密和解密测试
void App_AesKey128Test (void)
{
    stc_aes_cfg_t stcAesCfg;                                  
 
    uint8_t u8Idx;
    
    ///< 打开AES外设门控时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAes, TRUE);
    
    ///<KEY 128
    stcAesCfg.pu32Plaintext = u32AESTestData;       ///< AES 明文指针
    stcAesCfg.pu32Cipher    = pu32Ciphertext;       ///< AES 密文指针
    stcAesCfg.pu32Key       = u32AESTestKey;        ///< AES 密钥指针
    ///< AES 加密
    AES_Encrypt(&stcAesCfg);
    ///< AES 解密
    stcAesCfg.pu32Plaintext = pu32Plaintext;
    AES_Decrypt(&stcAesCfg);
  
    for(u8Idx = 0;u8Idx< sizeof(pu32Ciphertext)/sizeof(pu32Ciphertext[0]);u8Idx++)
    {
        ///< 如果解密出来的数据和原数据不匹配，则在此死循环。
        while(pu32Plaintext[u8Idx] != u32AESTestData[u8Idx]);
    }
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


