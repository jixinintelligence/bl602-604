/**
  ******************************************************************************
  * @file    bl602_sflash_ext.c
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

#include "bl602_l1c.h"
#include "bl602_sflash_ext.h"
#include "bl602_sf_ctrl.h"
#include "l1c_reg.h"

/** @addtogroup  BL602_Peripheral_Driver
 *  @{
 */

/** @addtogroup  SFLASH_EXT
 *  @{
 */

/** @defgroup  SFLASH_EXT_Private_Macros
 *  @{
 */

/*@} end of group SFLASH_EXT_Private_Macros */

/** @defgroup  SFLASH_EXT_Private_Types
 *  @{
 */

/*@} end of group SFLASH_EXT_Private_Types */

/** @defgroup  SFLASH_EXT_Private_Variables
 *  @{
 */

/*@} end of group SFLASH_EXT_Private_Variables */

/** @defgroup  SFLASH_EXT_Global_Variables
 *  @{
 */

/*@} end of group SFLASH_EXT_Global_Variables */

/** @defgroup  SFLASH_EXT_Private_Fun_Declaration
 *  @{
 */

/*@} end of group SFLASH_EXT_Private_Fun_Declaration */

/** @defgroup  SFLASH_EXT_Private_Functions
 *  @{
 */

/*@} end of group SFLASH_EXT_Private_Functions */

/** @defgroup  SFLASH_EXT_Public_Functions
 *  @{
 */

/****************************************************************************//**
 * @brief  Read flash register with read command
 *
 * @param  flashCfg: Serial flash parameter configuration pointer
 * @param  readRegCmd: read command
 * @param  regValue: register value pointer to store data
 * @param  regLen: register value length
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION SFlash_Read_Reg_With_Cmd(SPI_Flash_Cfg_Type *flashCfg,uint8_t readRegCmd,uint8_t *regValue,uint8_t regLen)
{
    uint8_t * const flashCtrlBuf=(uint8_t *)SF_CTRL_BUF_BASE;
    SF_Ctrl_Cmd_Cfg_Type flashCmd;
    uint32_t cnt=0;

    if(((uint32_t)&flashCmd)%4==0){
        BL602_MemSet4((uint32_t *)&flashCmd,0,sizeof(flashCmd)/4);
    }else{
        BL602_MemSet(&flashCmd,0,sizeof(flashCmd));
    }

    flashCmd.cmdBuf[0]=readRegCmd<<24;
    flashCmd.rwFlag=SF_CTRL_READ;
    flashCmd.nbData=regLen;

    SF_Ctrl_SendCmd(&flashCmd);

    while(SET==SF_Ctrl_GetBusyState()){
        BL602_Delay_US(1);
        cnt++;
        if(cnt>1000){
            return ERROR;
        }
    }

    BL602_MemCpy(regValue,flashCtrlBuf,regLen);
    return SUCCESS;
}

/****************************************************************************//**
 * @brief  Write flash register with write command
 *
 * @param  flashCfg: Serial flash parameter configuration pointer
 * @param  writeRegCmd: write command
 * @param  regValue: register value pointer storing data
 * @param  regLen: register value length
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION SFlash_Write_Reg_With_Cmd(SPI_Flash_Cfg_Type *flashCfg,uint8_t writeRegCmd,uint8_t *regValue,uint8_t regLen)
{
    uint8_t * const flashCtrlBuf=(uint8_t *)SF_CTRL_BUF_BASE;
    uint32_t cnt=0;
    SF_Ctrl_Cmd_Cfg_Type flashCmd;

    if(((uint32_t)&flashCmd)%4==0){
        BL602_MemSet4((uint32_t *)&flashCmd,0,sizeof(flashCmd)/4);
    }else{
        BL602_MemSet(&flashCmd,0,sizeof(flashCmd));
    }
    BL602_MemCpy(flashCtrlBuf,regValue,regLen);

    flashCmd.cmdBuf[0]=writeRegCmd<<24;
    flashCmd.rwFlag=SF_CTRL_WRITE;
    flashCmd.nbData=regLen;

    SF_Ctrl_SendCmd(&flashCmd);

    /* take 40ms for tw(write status register) as default */
    while(SET==SFlash_Busy(flashCfg)){
        BL602_Delay_US(100);
        cnt++;
        if(cnt>400){
            return ERROR;
        }
    }

    return SUCCESS;
}

/****************************************************************************//**
 * @brief  Enable cache
 *
 * @param  wayDisable: cache way disable config
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION SFlash_Cache_Enable_Set(uint8_t wayDisable)
{
    uint32_t tmpVal;
    uint32_t cnt=0;

    /* Set cacheable to 0 */
    tmpVal=BL_RD_REG(L1C_BASE,L1C_CONFIG);
    tmpVal=BL_CLR_REG_BIT(tmpVal,L1C_CACHEABLE);
    tmpVal=BL_SET_REG_BIT(tmpVal,L1C_BYPASS);
    tmpVal=BL_CLR_REG_BIT(tmpVal,L1C_WAY_DIS);
    tmpVal=BL_CLR_REG_BIT(tmpVal,L1C_CNT_EN);
    BL_WR_REG(L1C_BASE,L1C_CONFIG,tmpVal);

    tmpVal=BL_RD_REG(L1C_BASE,L1C_CONFIG);
    /*Set Tag RAM to zero */
    tmpVal=BL_CLR_REG_BIT(tmpVal,L1C_INVALID_EN);
    BL_WR_REG(L1C_BASE,L1C_CONFIG,tmpVal);
    /* Left space for hardware change status*/
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    tmpVal=BL_SET_REG_BIT(tmpVal,L1C_INVALID_EN);
    BL_WR_REG(L1C_BASE,L1C_CONFIG,tmpVal);
    /* Left space for hardware change status*/
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    /* Polling for invalid done:100US*/
    do{
        cnt++;
        tmpVal=BL_RD_REG(L1C_BASE,L1C_CONFIG);
    }while(!BL_IS_REG_BIT_SET(tmpVal,L1C_INVALID_DONE)&&cnt<100*32);


    tmpVal=BL_SET_REG_BIT(tmpVal,L1C_BYPASS);
    BL_WR_REG(L1C_BASE,L1C_CONFIG,tmpVal);

    tmpVal=BL_CLR_REG_BIT(tmpVal,L1C_BYPASS);
    tmpVal=BL_CLR_REG_BIT(tmpVal,L1C_WAY_DIS);
    tmpVal=BL_SET_REG_BIT(tmpVal,L1C_CNT_EN);
    BL_WR_REG(L1C_BASE,L1C_CONFIG,tmpVal);

    tmpVal|=(wayDisable<<L1C_WAY_DIS_POS);
    /* If way disable is 0x0f, cacheable can't be set */
    if(wayDisable!=0x0f){
        tmpVal=BL_SET_REG_BIT(tmpVal,L1C_CACHEABLE);
    }else{
        tmpVal=BL_CLR_REG_BIT(tmpVal,L1C_CACHEABLE);
    }
    BL_WR_REG(L1C_BASE,L1C_CONFIG,tmpVal);

    return SUCCESS;
}

/****************************************************************************//**
 * @brief  Flush cache
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION SFlash_Cache_Flush(void)
{
    uint32_t tmpVal;

    /* Disable early respone */
    tmpVal=BL_RD_REG(L1C_BASE,L1C_CONFIG);
    SFlash_Cache_Enable_Set((tmpVal>>L1C_WAY_DIS_POS)&0xf);
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();

    return SUCCESS;
}

/****************************************************************************//**
 * @brief  Enable cache read from flash with cache
 *
 * @param  flashCfg: Serial flash parameter configuration pointer
 * @param  ioMode: flash controller interface mode
 * @param  contRead: Wether enable cont read mode
 * @param  wayDisable: cache way disable config
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION SFlash_Cache_Read_Enable(SPI_Flash_Cfg_Type *flashCfg,
    SF_Ctrl_IO_Type ioMode,uint8_t contRead,uint8_t wayDisable)
{
    BL_Err_Type stat;

    /* Cache now only support 32 bytes read */
    stat=SFlash_Set_IDbus_Cfg(flashCfg,ioMode,contRead,0,32);
    if(SUCCESS!=stat){
        return stat;
    }

    return SFlash_Cache_Enable_Set(wayDisable);
}

/****************************************************************************//**
 * @brief  Sflash restore from power down
 *
 * @param  pFlashCfg: Flash configuration pointer
 * @param  flashContRead: Whether enable continuous read
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION SFlash_Restore_From_Powerdown(SPI_Flash_Cfg_Type *pFlashCfg,uint8_t flashContRead)
{
    BL_Err_Type stat=SUCCESS;
    uint32_t jdecId=0;
    uint8_t tmp[8];
    uint8_t ioMode=pFlashCfg->ioMode&0xf;

    /* Wake flash up from power down */
    SFlash_Releae_Powerdown(pFlashCfg);
    BL602_Delay_US(120);

    SFlash_GetJedecId(pFlashCfg,(uint8_t *)&jdecId);

    if(SF_CTRL_QO_MODE==ioMode||SF_CTRL_QIO_MODE==ioMode){
        SFlash_Qspi_Enable(pFlashCfg);
    }

    if(((pFlashCfg->ioMode>>4)&0x01)==1){
        /* unwrap */
        L1C_Set_Wrap(DISABLE);
    }else{
        /* burst wrap */
        L1C_Set_Wrap(ENABLE);
        /* For command that is setting register instead of send command, we need write enable */
        SFlash_Write_Enable(pFlashCfg);
        SFlash_SetBurstWrap(pFlashCfg);
    }

    if(flashContRead){
        stat=SFlash_Read(pFlashCfg,ioMode,1,0x00000000,(uint8_t *)tmp, sizeof(tmp));
        SF_Ctrl_Set_Owner(SF_CTRL_OWNER_IAHB);
    }else{
        stat=SFlash_Set_IDbus_Cfg(pFlashCfg,ioMode,0,0,32);
    }
    return stat;
}

/*@} end of group SFLASH_EXT_Public_Functions */

/*@} end of group SFLASH_EXT */

/*@} end of group BL602_Peripheral_Driver */
