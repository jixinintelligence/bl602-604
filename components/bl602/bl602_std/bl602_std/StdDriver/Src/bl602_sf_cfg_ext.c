/**
  ******************************************************************************
  * @file    bl602_sf_cfg_ext.c
  * @version V1.0
  * @date
  * @brief   This file is the standard driver c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 Bouffalo Lab</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Bouffalo Lab nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "bl602_glb.h"
#include "bl602_sf_cfg.h"
#include "bl602_sf_cfg_ext.h"
#include "bl602_xip_sflash.h"
#include "bl602_romdriver.h"

/** @addtogroup  BL602_Peripheral_Driver
 *  @{
 */

/** @addtogroup  SF_CFG_EXT
 *  @{
 */

/** @defgroup  SF_CFG_EXT_Private_Macros
 *  @{
 */
#define BFLB_FLASH_CFG_MAGIC                    "FCFG"

/*@} end of group SF_CFG_EXT_Private_Macros */

/** @defgroup  SF_CFG_EXT_Private_Types
 *  @{
 */

/*@} end of group SF_CFG_EXT_Private_Types */

/** @defgroup  SF_CFG_EXT_Private_Variables
 *  @{
 */

/*@} end of group SF_CFG_EXT_Private_Variables */

/** @defgroup  SF_CFG_EXT_Global_Variables
 *  @{
 */

/*@} end of group SF_CFG_EXT_Global_Variables */

/** @defgroup  SF_CFG_EXT_Private_Fun_Declaration
 *  @{
 */

/*@} end of group SF_CFG_EXT_Private_Fun_Declaration */

/** @defgroup  SF_CFG_EXT_Private_Functions
 *  @{
 */

/****************************************************************************//**
 * @brief  Init internal flash GPIO according to flash GPIO config
 *
 * @param  None
 *
 * @return None
 *
*******************************************************************************/
void ATTR_TCM_SECTION SF_Cfg_Init_Internal_Flash_Gpio(void)
{
    GLB_GPIO_Cfg_Type gpioCfg = {
        .gpioPin = GLB_GPIO_PIN_0,
        .gpioFun = GPIO_FUN_SWGPIO,
        .gpioMode = GPIO_MODE_INPUT,
        .pullType = GPIO_PULL_NONE,
        .drive = 0,
        .smtCtrl = 1,
    };

    /* Turn on Flash pad, GPIO23 - GPIO28 */
    for(uint32_t pin=23;pin<29;pin++){
        gpioCfg.gpioPin = pin;
        if(pin==24){
            gpioCfg.pullType = GPIO_PULL_DOWN;
        }else{
            gpioCfg.pullType = GPIO_PULL_NONE;
        }
        GLB_GPIO_Init(&gpioCfg);
    }
}

/****************************************************************************//**
 * @brief  Get flash config according to flash ID
 *
 * @param  flashID: Flash ID
 * @param  pFlashCfg: Flash config pointer
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION SF_Cfg_Get_Flash_Cfg_Need_Lock(uint32_t flashID,SPI_Flash_Cfg_Type * pFlashCfg)
{
    uint8_t buf[sizeof(SPI_Flash_Cfg_Type)+8];
    uint32_t crc,*pCrc;

    if(flashID==0){
        XIP_SFlash_Read_Via_Cache_Need_Lock(8+BL602_FLASH_XIP_BASE,buf,sizeof(SPI_Flash_Cfg_Type)+8);
        if(BL602_MemCmp(buf,BFLB_FLASH_CFG_MAGIC,4)==0){
            crc=BFLB_Soft_CRC32((uint8_t *)buf+4,sizeof(SPI_Flash_Cfg_Type));
            pCrc=(uint32_t *)(buf+4+sizeof(SPI_Flash_Cfg_Type));
            if(*pCrc==crc){
                BL602_MemCpy_Fast(pFlashCfg,(uint8_t *)buf+4,sizeof(SPI_Flash_Cfg_Type));
                return SUCCESS ;
            }
        }
    }else{
        return RomDriver_SF_Cfg_Get_Flash_Cfg_Need_Lock(flashID, pFlashCfg);
    }

    return ERROR;
}

/*@} end of group SF_CFG_EXT_Private_Functions */

/*@} end of group SF_CFG_EXT */

/*@} end of group BL602_Peripheral_Driver */
