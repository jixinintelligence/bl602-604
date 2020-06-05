/**
  ******************************************************************************
  * @file    bl602_xip_sflash_ext.c
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

#include "string.h"
#include "bl602_xip_sflash_ext.h"
#include "bl602_sf_ctrl_ext.h"

/** @addtogroup  BL602_Peripheral_Driver
 *  @{
 */

/** @addtogroup  XIP_SFLASH_EXT
 *  @{
 */

/** @defgroup  XIP_SFLASH_EXT_Private_Macros
 *  @{
 */

/*@} end of group XIP_SFLASH_EXT_Private_Macros */

/** @defgroup  XIP_SFLASH_EXT_Private_Types
 *  @{
 */

/*@} end of group XIP_SFLASH_EXT_Private_Types */

/** @defgroup  XIP_SFLASH_EXT_Private_Variables
 *  @{
 */

/*@} end of group XIP_SFLASH_EXT_Private_Variables */

/** @defgroup  XIP_SFLASH_EXT_Global_Variables
 *  @{
 */

/*@} end of group XIP_SFLASH_EXT_Global_Variables */

/** @defgroup  XIP_SFLASH_EXT_Private_Fun_Declaration
 *  @{
 */

/*@} end of group XIP_SFLASH_EXT_Private_Fun_Declaration */

/** @defgroup  XIP_SFLASH_EXT_Private_Functions
 *  @{
 */

/****************************************************************************//**
 * @brief  XIP SFlash option save
 *
 * @param  aesEnable: AES enable status pointer
 *
 * @return None
 *
*******************************************************************************/
void ATTR_TCM_SECTION XIP_SFlash_Opt_Enter(uint8_t *aesEnable)
{
    *aesEnable=SF_Ctrl_Is_AES_Enable();
    if(*aesEnable){
        SF_Ctrl_AES_Disable();
    }
}

/****************************************************************************//**
 * @brief  XIP SFlash option restore
 *
 * @param  aesEnable: AES enable status
 *
 * @return None
 *
*******************************************************************************/
void ATTR_TCM_SECTION XIP_SFlash_Opt_Exit(uint8_t aesEnable)
{
    if(aesEnable){
        SF_Ctrl_AES_Enable();
    }
}

/****************************************************************************//**
 * @brief  Read data from flash via XIP
 *
 * @param  addr: addr: flash read start address
 * @param  data: data: data pointer to store data read from flash
 * @param  len: len: data length to read
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION XIP_SFlash_Read_Via_Cache_Need_Lock(uint32_t addr,uint8_t *data, uint32_t len)
{
    uint32_t offset;

    if(addr>=BL602_FLASH_XIP_BASE && addr<BL602_FLASH_XIP_END){
        offset=SF_Ctrl_Get_Flash_Image_Offset();
        SF_Ctrl_Set_Flash_Image_Offset(0);
        /* Flash read */
        BL602_MemCpy_Fast(data,(void *)(addr),len);
        SF_Ctrl_Set_Flash_Image_Offset(offset);
    }

    return SUCCESS;
}

/*@} end of group XIP_SFLASH_EXT_Private_Functions */

/*@} end of group XIP_SFLASH_EXT */

/*@} end of group BL602_Peripheral_Driver */
