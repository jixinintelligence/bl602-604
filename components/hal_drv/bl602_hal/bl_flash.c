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
#include <bl602_romdriver.h>

#include "bl_flash.h"

static struct {
    uint32_t magic;
    SPI_Flash_Cfg_Type flashCfg;
} boot2_flashCfg; //XXX Dont change the name of varaible, since we refer this boot2_partition_table in linker script

int bl_flash_erase(uint32_t addr, int len)
{
    /*We assume mid zeor is illegal*/
    if (0 == boot2_flashCfg.flashCfg.mid) {
        return -1;
    }
    RomDriver_XIP_SFlash_Erase_With_Lock(
            &boot2_flashCfg.flashCfg,
            addr,
            len
    );
    return 0;
}

int bl_flash_write(uint32_t addr, uint8_t *src, int len)
{
    /*We assume mid zeor is illegal*/
    if (0 == boot2_flashCfg.flashCfg.mid) {
        return -1;
    }

    RomDriver_XIP_SFlash_Write_With_Lock(
            &boot2_flashCfg.flashCfg,
            addr,
            src,
            len
    );
    return 0;
}

int bl_flash_read(uint32_t addr, uint8_t *dst, int len)
{
    /*We assume mid zeor is illegal*/
    if (0 == boot2_flashCfg.flashCfg.mid) {
        return -1;
    }

    RomDriver_XIP_SFlash_Read_With_Lock(
            &boot2_flashCfg.flashCfg,
            addr,
            dst,
            len
    );
    return 0;
}

static void _dump_flash_config()
{
    extern uint8_t __boot2_flashCfg_src;
    printf("======= FlashCfg magiccode @%p, code 0x%08lX =======\r\n",
            &__boot2_flashCfg_src,
            boot2_flashCfg.magic
    );
    printf("mid \t\t0x%X\r\n", boot2_flashCfg.flashCfg.mid);
    printf("clkDelay \t0x%X\r\n", boot2_flashCfg.flashCfg.clkDelay);
    printf("clkInvert \t0x%X\r\n", boot2_flashCfg.flashCfg.clkInvert);
    printf("sector size\t%uKBytes\r\n", boot2_flashCfg.flashCfg.sectorSize);
    printf("page size\t%uBytes\r\n", boot2_flashCfg.flashCfg.pageSize);
    puts("---------------------------------------------------------------\r\n");
}

int bl_flash_config_update(void)
{
    _dump_flash_config();

    return 0;
}
