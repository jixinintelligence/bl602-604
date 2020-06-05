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
#ifndef _REG_RF_EPA_H_
#define _REG_RF_EPA_H_

#include "co_int.h"
#include "arch.h"
#include "_reg_access.h"

static inline uint32_t rf_epa_vector_03_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001400);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

static inline void rf_epa_vector_03_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001400);
    localVal &= ~((uint32_t)0x00000008);
    localVal |= (x << 3)&((uint32_t)0x00000008);
    REG_PL_WR(0x40001400,localVal);
}

static inline uint32_t rf_epa_vector_02_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001400);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

static inline void rf_epa_vector_02_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001400);
    localVal &= ~((uint32_t)0x00000004);
    localVal |= (x << 2)&((uint32_t)0x00000004);
    REG_PL_WR(0x40001400,localVal);
}

static inline uint32_t rf_epa_vector_01_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001400);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

static inline void rf_epa_vector_01_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001400);
    localVal &= ~((uint32_t)0x00000002);
    localVal |= (x << 1)&((uint32_t)0x00000002);
    REG_PL_WR(0x40001400,localVal);
}

static inline uint32_t rf_epa_vector_00_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001400);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

static inline void rf_epa_vector_00_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001400);
    localVal &= ~((uint32_t)0x00000001);
    localVal |= (x << 0)&((uint32_t)0x00000001);
    REG_PL_WR(0x40001400,localVal);
}

static inline uint32_t rf_epa_rx2on_vector_03_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001404);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

static inline void rf_epa_rx2on_vector_03_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001404);
    localVal &= ~((uint32_t)0x00000008);
    localVal |= (x << 3)&((uint32_t)0x00000008);
    REG_PL_WR(0x40001404,localVal);
}

static inline uint32_t rf_epa_rx2on_vector_02_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001404);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

static inline void rf_epa_rx2on_vector_02_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001404);
    localVal &= ~((uint32_t)0x00000004);
    localVal |= (x << 2)&((uint32_t)0x00000004);
    REG_PL_WR(0x40001404,localVal);
}

static inline uint32_t rf_epa_rx2on_vector_01_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001404);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

static inline void rf_epa_rx2on_vector_01_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001404);
    localVal &= ~((uint32_t)0x00000002);
    localVal |= (x << 1)&((uint32_t)0x00000002);
    REG_PL_WR(0x40001404,localVal);
}

static inline uint32_t rf_epa_rx2on_vector_00_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001404);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

static inline void rf_epa_rx2on_vector_00_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001404);
    localVal &= ~((uint32_t)0x00000001);
    localVal |= (x << 0)&((uint32_t)0x00000001);
    REG_PL_WR(0x40001404,localVal);
}

static inline uint32_t rf_epa_rxon_vector_03_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001408);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

static inline void rf_epa_rxon_vector_03_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001408);
    localVal &= ~((uint32_t)0x00000008);
    localVal |= (x << 3)&((uint32_t)0x00000008);
    REG_PL_WR(0x40001408,localVal);
}

static inline uint32_t rf_epa_rxon_vector_02_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001408);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

static inline void rf_epa_rxon_vector_02_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001408);
    localVal &= ~((uint32_t)0x00000004);
    localVal |= (x << 2)&((uint32_t)0x00000004);
    REG_PL_WR(0x40001408,localVal);
}

static inline uint32_t rf_epa_rxon_vector_01_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001408);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

static inline void rf_epa_rxon_vector_01_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001408);
    localVal &= ~((uint32_t)0x00000002);
    localVal |= (x << 1)&((uint32_t)0x00000002);
    REG_PL_WR(0x40001408,localVal);
}

static inline uint32_t rf_epa_rxon_vector_00_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001408);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

static inline void rf_epa_rxon_vector_00_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001408);
    localVal &= ~((uint32_t)0x00000001);
    localVal |= (x << 0)&((uint32_t)0x00000001);
    REG_PL_WR(0x40001408,localVal);
}

static inline uint32_t rf_epa_tx2on_vector_03_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x4000140c);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

static inline void rf_epa_tx2on_vector_03_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x4000140c);
    localVal &= ~((uint32_t)0x00000008);
    localVal |= (x << 3)&((uint32_t)0x00000008);
    REG_PL_WR(0x4000140c,localVal);
}

static inline uint32_t rf_epa_tx2on_vector_02_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x4000140c);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

static inline void rf_epa_tx2on_vector_02_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x4000140c);
    localVal &= ~((uint32_t)0x00000004);
    localVal |= (x << 2)&((uint32_t)0x00000004);
    REG_PL_WR(0x4000140c,localVal);
}

static inline uint32_t rf_epa_tx2on_vector_01_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x4000140c);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

static inline void rf_epa_tx2on_vector_01_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x4000140c);
    localVal &= ~((uint32_t)0x00000002);
    localVal |= (x << 1)&((uint32_t)0x00000002);
    REG_PL_WR(0x4000140c,localVal);
}

static inline uint32_t rf_epa_tx2on_vector_00_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x4000140c);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

static inline void rf_epa_tx2on_vector_00_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x4000140c);
    localVal &= ~((uint32_t)0x00000001);
    localVal |= (x << 0)&((uint32_t)0x00000001);
    REG_PL_WR(0x4000140c,localVal);
}

static inline uint32_t rf_epa_tx2paon_vector_03_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001410);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

static inline void rf_epa_tx2paon_vector_03_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001410);
    localVal &= ~((uint32_t)0x00000008);
    localVal |= (x << 3)&((uint32_t)0x00000008);
    REG_PL_WR(0x40001410,localVal);
}

static inline uint32_t rf_epa_tx2paon_vector_02_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001410);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

static inline void rf_epa_tx2paon_vector_02_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001410);
    localVal &= ~((uint32_t)0x00000004);
    localVal |= (x << 2)&((uint32_t)0x00000004);
    REG_PL_WR(0x40001410,localVal);
}

static inline uint32_t rf_epa_tx2paon_vector_01_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001410);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

static inline void rf_epa_tx2paon_vector_01_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001410);
    localVal &= ~((uint32_t)0x00000002);
    localVal |= (x << 1)&((uint32_t)0x00000002);
    REG_PL_WR(0x40001410,localVal);
}

static inline uint32_t rf_epa_tx2paon_vector_00_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001410);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

static inline void rf_epa_tx2paon_vector_00_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001410);
    localVal &= ~((uint32_t)0x00000001);
    localVal |= (x << 0)&((uint32_t)0x00000001);
    REG_PL_WR(0x40001410,localVal);
}

static inline uint32_t rf_epa_txpaon_vector_03_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001414);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

static inline void rf_epa_txpaon_vector_03_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001414);
    localVal &= ~((uint32_t)0x00000008);
    localVal |= (x << 3)&((uint32_t)0x00000008);
    REG_PL_WR(0x40001414,localVal);
}

static inline uint32_t rf_epa_txpaon_vector_02_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001414);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

static inline void rf_epa_txpaon_vector_02_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001414);
    localVal &= ~((uint32_t)0x00000004);
    localVal |= (x << 2)&((uint32_t)0x00000004);
    REG_PL_WR(0x40001414,localVal);
}

static inline uint32_t rf_epa_txpaon_vector_01_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001414);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

static inline void rf_epa_txpaon_vector_01_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001414);
    localVal &= ~((uint32_t)0x00000002);
    localVal |= (x << 1)&((uint32_t)0x00000002);
    REG_PL_WR(0x40001414,localVal);
}

static inline uint32_t rf_epa_txpaon_vector_00_getf(void)
{
    uint32_t localVal = REG_PL_RD(0x40001414);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

static inline void rf_epa_txpaon_vector_00_setf(uint32_t x)
{
    uint32_t localVal = REG_PL_RD(0x40001414);
    localVal &= ~((uint32_t)0x00000001);
    localVal |= (x << 0)&((uint32_t)0x00000001);
    REG_PL_WR(0x40001414,localVal);
}

#endif