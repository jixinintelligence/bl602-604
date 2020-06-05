/**
  ******************************************************************************
  * @file    bl602_sf_ctrl.c
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

#include "bl602_sf_ctrl.h"

/** @addtogroup  BL602_Peripheral_Driver
 *  @{
 */

/** @addtogroup  SF_CTRL
 *  @{
 */

/** @defgroup  SF_CTRL_Private_Macros
 *  @{
 */

/*@} end of group SF_CTRL_Private_Macros */

/** @defgroup  SF_CTRL_Private_Types
 *  @{
 */

/*@} end of group SF_CTRL_Private_Types */

/** @defgroup  SF_CTRL_Private_Variables
 *  @{
 */

/*@} end of group SF_CTRL_Private_Variables */

/** @defgroup  SF_CTRL_Global_Variables
 *  @{
 */

/*@} end of group SF_CTRL_Global_Variables */

/** @defgroup  SF_CTRL_Private_Fun_Declaration
 *  @{
 */

/*@} end of group SF_CTRL_Private_Fun_Declaration */

/** @defgroup  SF_CTRL_Private_Functions
 *  @{
 */

/*@} end of group SF_CTRL_Private_Functions */

/** @defgroup  SF_CTRL_Public_Functions
 *  @{
 */

/****************************************************************************//**
 * @brief  Check is serial flash controller AES enable
 *
 * @param  None
 *
 * @return Wether AES is enable
 *
*******************************************************************************/
uint8_t ATTR_TCM_SECTION SF_Ctrl_Is_AES_Enable(void)
{
    uint32_t tmpVal;

    tmpVal=BL_RD_REG(SF_CTRL_BASE,SF_CTRL_SF_AES);
    return BL_IS_REG_BIT_SET(tmpVal,SF_CTRL_SF_AES_EN);
}

/****************************************************************************//**
 * @brief  Get flash controller clock delay value
 *
 * @param  None
 *
 * @return Clock delay value
 *
*******************************************************************************/
__WEAK
uint8_t ATTR_TCM_SECTION SF_Ctrl_Get_Clock_Delay(void)
{
    uint32_t tmpVal;

    tmpVal=BL_RD_REG(SF_CTRL_BASE,SF_CTRL_0);

    if(BL_GET_REG_BITS_VAL(tmpVal,SF_CTRL_SF_IF_READ_DLY_EN)==0){
        return 0;
    }else{
        return BL_GET_REG_BITS_VAL(tmpVal,SF_CTRL_SF_IF_READ_DLY_N) +1;
    }
}

/****************************************************************************//**
 * @brief  Set flash controller clock delay value
 *
 * @param  delay: Clock delay value
 *
 * @return None
 *
*******************************************************************************/
__WEAK
void ATTR_TCM_SECTION SF_Ctrl_Set_Clock_Delay(uint8_t delay)
{
    uint32_t tmpVal;

    tmpVal=BL_RD_REG(SF_CTRL_BASE,SF_CTRL_0);

    if(delay>0){
        tmpVal=BL_SET_REG_BIT(tmpVal,SF_CTRL_SF_IF_READ_DLY_EN);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,SF_CTRL_SF_IF_READ_DLY_N,delay-1);
    }else{
        tmpVal=BL_CLR_REG_BIT(tmpVal,SF_CTRL_SF_IF_READ_DLY_EN);
    }

    BL_WR_REG(SF_CTRL_BASE,SF_CTRL_0,tmpVal);
}

/*@} end of group SF_CTRL_Public_Functions */

/*@} end of group SF_CTRL */

/*@} end of group BL602_Peripheral_Driver */
