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
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "bl_boot2.h"
#include "bl_flash.h"
#include "hal_boot2.h"

#define PARTITION_BOOT2_RAM_ADDR_ACTIVE (0x42049C00)
#define PARTITION_HEADER_BOOT2_RAM_ADDR (0x42049C04)
#define PARTITION_BOOT2_FLASH_HEADER    (0x42049d14)
#define PARTITION_BOOT2_FLASH_CONFIG    (0x42049d18)
#define PARTITION_MAGIC                 (0x54504642)
#define PARTITION_FW_PART_NAME          "FW"
#define PARTITION_FW_PART_HEADER_SIZE   (0x1000)
//TODO use header file from project
#define FW_XIP_ADDRESS                  (0x23000000)

static struct {
    uint8_t partition_active_idx;
    uint8_t pad[3];
    PtTable_Stuff_Config table;
} boot2_partition_table;//XXX Dont change the name of varaible, since we refer this boot2_partition_table in linker script

void hal_boot2_set_ptable_opt(HALpPtTable_Flash_Erase erase, HALpPtTable_Flash_Write write)
{
    PtTable_Set_Flash_Operation((pPtTable_Flash_Erase)erase, (pPtTable_Flash_Write)write);
}

int hal_boot2_update_ptable(HALPartition_Entry_Config *ptEntry_hal)
{
    int ret;
    //FIXME force covert
    PtTable_Entry_Config *ptEntry = (PtTable_Entry_Config*)ptEntry_hal;

    ptEntry->activeIndex = !ptEntry->activeIndex;
    (ptEntry->age)++;
    ret = PtTable_Update_Entry(NULL,!boot2_partition_table.partition_active_idx, &boot2_partition_table.table, ptEntry);
    return ret;
}

static void _dump_partition(void)
{
    int i;
    PtTable_Stuff_Config *part = &boot2_partition_table.table;

    printf("======= PtTable_Config @%p=======\r\n", part);
    printf("magicCode 0x%08X;", (unsigned int)(part->ptTable.magicCode));
    printf(" version 0x%04X;", part->ptTable.version);
    printf(" entryCnt %u;", part->ptTable.entryCnt);
    printf(" age %lu;", part->ptTable.age);
    printf(" crc32 0x%08X\r\n", (unsigned int)part->ptTable.crc32);

    printf("idx  type device activeIndex     name   Address[0]  Address[1]  Length[0]   Length[1]   age\r\n");
    for (i = 0; i < part->ptTable.entryCnt; i++) {
        printf("[%02d] ", i);
        printf(" %02u", part->ptEntries[i].type);
        printf("     %u", part->ptEntries[i].device);
        printf("         %u", part->ptEntries[i].activeIndex);
        printf("      %8s", part->ptEntries[i].name);
        printf("  %p", (void*)(part->ptEntries[i].Address[0]));
        printf("  %p", (void*)(part->ptEntries[i].Address[1]));
        printf("  %p", (void*)(part->ptEntries[i].maxLen[0]));
        printf("  %p", (void*)(part->ptEntries[i].maxLen[1]));
        printf("  %lu\r\n", (part->ptEntries[i].age));
    }
}

int hal_boot2_partition_bus_addr(const char *name, uint32_t *addr0, uint32_t *addr1, uint32_t *size0, uint32_t *size1, int *active)
{
    int i;
    uint32_t addr0_t, addr1_t;

    if (PARTITION_MAGIC != boot2_partition_table.table.ptTable.magicCode) {
        return -EIO;
    }

    /*Get Target partition*/
    for (i = 0; i < boot2_partition_table.table.ptTable.entryCnt; i++) {
        if (0 == strcmp((char *)&(boot2_partition_table.table.ptEntries[i].name[0]), name)) {
            break;
        }
    }
    if (boot2_partition_table.table.ptTable.entryCnt == i) {
        return -ENOENT;
    }
    addr0_t = boot2_partition_table.table.ptEntries[i].Address[0];
    addr1_t = boot2_partition_table.table.ptEntries[i].Address[1];
    *active = boot2_partition_table.table.ptEntries[i].activeIndex;
    *size0 = boot2_partition_table.table.ptEntries[i].maxLen[0];
    *size1 = boot2_partition_table.table.ptEntries[i].maxLen[1];

    /*cal partition address*/
    for (i = 0; i < boot2_partition_table.table.ptTable.entryCnt; i++) {
        if (0 == strcmp((char *)&(boot2_partition_table.table.ptEntries[i].name[0]), PARTITION_FW_PART_NAME)) {
            break;
        }
    }
    if (boot2_partition_table.table.ptTable.entryCnt == i) {
        return -ECANCELED;
    }
    /*Make sure target partition is after FW partition*/
    if ( (addr0_t && (addr0_t < boot2_partition_table.table.ptEntries[i].Address[0])) ||
         (addr0_t && (addr0_t < boot2_partition_table.table.ptEntries[i].Address[1])) ||
         (addr1_t && (addr1_t < boot2_partition_table.table.ptEntries[i].Address[0])) ||
         (addr1_t && (addr1_t < boot2_partition_table.table.ptEntries[i].Address[1]))) {
        return -EINVAL;
    }
    if ((0 != boot2_partition_table.table.ptEntries[i].activeIndex) &&
        (1 != boot2_partition_table.table.ptEntries[i].activeIndex)) {
        return -EFAULT;
    }
    *addr0 = addr0_t - boot2_partition_table.table.ptEntries[i].Address[boot2_partition_table.table.ptEntries[i].activeIndex] - PARTITION_FW_PART_HEADER_SIZE + FW_XIP_ADDRESS;
    *addr1 = addr1_t - boot2_partition_table.table.ptEntries[i].Address[boot2_partition_table.table.ptEntries[i].activeIndex] - PARTITION_FW_PART_HEADER_SIZE + FW_XIP_ADDRESS;

    return 0;
}

int hal_boot2_partition_bus_addr_active(const char *name, uint32_t *addr, uint32_t *size)
{
    uint32_t addr0, addr1;
    uint32_t size0, size1;
    int active, ret;

    if ((ret = hal_boot2_partition_bus_addr(name, &addr0, &addr1, &size0, &size1, &active))) {
        return ret;
    }
    *addr = active ? addr1 : addr0;
    *size = active ? size1 : size0;

    return 0;
}

int hal_boot2_partition_bus_addr_inactive(const char *name, uint32_t *addr, uint32_t *size)
{
    uint32_t addr0, addr1;
    uint32_t size0, size1;
    int active, ret;

    if ((ret = hal_boot2_partition_bus_addr(name, &addr0, &addr1, &size0, &size1, &active))) {
        return ret;
    }
    *addr = active ? addr0 : addr1;
    *size = active ? size0 : size1;

    return 0;
}

int hal_boot2_partition_addr(const char *name, uint32_t *addr0, uint32_t *addr1, uint32_t *size0, uint32_t *size1, int *active)
{
    int i;

    if (PARTITION_MAGIC != boot2_partition_table.table.ptTable.magicCode) {
        return -EIO;
    }

    /*Get Target partition*/
    for (i = 0; i < boot2_partition_table.table.ptTable.entryCnt; i++) {
        if (0 == strcmp((char *)&(boot2_partition_table.table.ptEntries[i].name[0]), name)) {
            break;
        }
    }
    if (boot2_partition_table.table.ptTable.entryCnt == i) {
        return -ENOENT;
    }
    *addr0 = boot2_partition_table.table.ptEntries[i].Address[0];
    *addr1 = boot2_partition_table.table.ptEntries[i].Address[1];
    *size0 = boot2_partition_table.table.ptEntries[i].maxLen[0];
    *size1 = boot2_partition_table.table.ptEntries[i].maxLen[1];
    *active = boot2_partition_table.table.ptEntries[i].activeIndex;

    return 0;
}

int hal_boot2_partition_addr_active(const char *name, uint32_t *addr, uint32_t *size)
{
    uint32_t addr0, addr1;
    uint32_t size0, size1;
    int active, ret;

    if ((ret = hal_boot2_partition_addr(name, &addr0, &addr1, &size0, &size1, &active))) {
        return ret;
    }
    *addr = active ? addr1 : addr0;
    *size = active ? size1 : size0;

    return 0;
}

int hal_boot2_partition_addr_inactive(const char *name, uint32_t *addr, uint32_t *size)
{
    uint32_t addr0, addr1;
    uint32_t size0, size1;
    int active, ret;

    if ((ret = hal_boot2_partition_addr(name, &addr0, &addr1, &size0, &size1, &active))) {
        return ret;
    }
    *addr = active ? addr0 : addr1;
    *size = active ? size0 : size1;

    return 0;
}

uint8_t hal_boot2_get_active_partition(void)
{
    return boot2_partition_table.partition_active_idx;
}

int hal_boot2_get_active_entries(int type, HALPartition_Entry_Config *ptEntry_hal)
{
    PtTable_Entry_Config *ptEntry = (PtTable_Entry_Config*)ptEntry_hal;
    if (PtTable_Get_Active_Entries(&boot2_partition_table.table, type, ptEntry)) {
        return -1;
    }
    return 0;
}

int hal_boot2_dump(void)
{
    _dump_partition();
    return 0;
}

int hal_boot2_init(void)
{
    boot2_partition_table.partition_active_idx = *(uint8_t*)PARTITION_BOOT2_RAM_ADDR_ACTIVE;

    printf("[HAL] [BOOT2] Active Partition[%u] consumed %d Bytes\r\n",
            boot2_partition_table.partition_active_idx,
            sizeof(PtTable_Stuff_Config)
    );
    _dump_partition();
    bl_flash_config_update();

    return 0;
}
