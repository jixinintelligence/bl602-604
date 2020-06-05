/*
 * Copyright (c) 2020 Bouffalolab.
 *
 * This file is part of
 *     *** Bouffalolab Software Dev Kit ***
 *      (see www.bouffalolab.com).
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
 */
#ifndef __BLSP_MEDIA_BOOT_H__
#define __BLSP_MEDIA_BOOT_H__

#include "stdint.h"
#include "stdio.h"
#include "string.h"

/** @addtogroup  BL606_BLSP_Boot2
 *  @{
 */

/** @addtogroup  BLSP_MEDIA_BOOT
 *  @{
 */

/** @defgroup  BLSP_MEDIA_BOOT_Public_Types
 *  @{
 */

/*@} end of group BLSP_MEDIA_BOOT_Public_Types */

/** @defgroup  BLSP_MEDIA_BOOT_Public_Constants
 *  @{
 */

/*@} end of group BLSP_MEDIA_BOOT_Public_Constants */

/** @defgroup  BLSP_MEDIA_BOOT_Public_Macros
 *  @{
 */

/*@} end of group BLSP_MEDIA_BOOT_Public_Macros */

/** @defgroup  BLSP_MEDIA_BOOT_Public_Functions
 *  @{
 */
int32_t BLSP_MediaBoot_Read(uint32_t addr,uint8_t *data, uint32_t len);
uint32_t BLSP_MediaBoot_Get_Flash_Cfg(uint32_t bootheaderAddr);
int32_t BLSP_MediaBoot_Cache_Enable(uint8_t contRead);
int32_t BLSP_MediaBoot_Main(uint32_t cpuBootheaderAddr[BFLB_BOOT2_CPU_MAX],uint8_t cpuRollBack[BFLB_BOOT2_CPU_MAX],
                            uint8_t foreGo);
void BLSP_Boot2_Show_Timer(void);
int32_t ATTR_TCM_SECTION BLSP_MediaBoot_Set_Encrypt(uint8_t index,Boot_Image_Config *bootImgCfg);

/*@} end of group BLSP_MEDIA_BOOT_Public_Functions */

/*@} end of group BLSP_MEDIA_BOOT */

/*@} end of group BL606_BLSP_Boot2 */

#endif /* __BLSP_MEDIA_BOOT_H__ */
