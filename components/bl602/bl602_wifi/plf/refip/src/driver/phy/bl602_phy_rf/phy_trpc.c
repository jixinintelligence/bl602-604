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
#include "phy_trpc.h"
#include "phy.h"
#include "bl602_rf_private.h"
#include <stdio.h>

#define TRPC_MIN(a, b)   ((a) <= (b) ? (a) : (b))
#define TRPC_MAX(a, b)   ((a) >= (b) ? (a) : (b))
#define TRPC_DIV10(x)    ((x * 51)/512)


/// max available power for each rate
static const int8_t txpwr_vs_rate_table[3][8] = 
{
    // 11b 1,2,5.5,11 Mbps
    {
        [0] = 20 - 2,
        [1] = 20 - 2,
        [2] = 20 - 2,
        [3] = 20 - 2,
        [4] = 20 - 2,  // not used
        [5] = 20 - 2,  // not used
        [6] = 20 - 2,  // not used
        [7] = 20 - 2   // not used
    },
    // 11g 6,9,12,18,24,36,48,54 Mbps
    {
        [0] = 20 - 2,  // BPSK 1/2
        [1] = 20 - 2,  // BPSK 3/4
        [2] = 20 - 2,  // QPSK 1/2
        [3] = 20 - 2,  // QPSK 3/4
        [4] = 18 - 2,  // 16QAM 1/2
        [5] = 18 - 2,  // 16QAM 3/4
        [6] = 16 - 2,  // 64QAM 2/3
        [7] = 16 - 2   // 64QAM 3/4
    },
    // 11n MCS0~MCS7
    {
        [0] = 20 - 2,  // BPSK 1/2
        [1] = 20 - 2,  // QPSK 1/2
        [2] = 20 - 2,  // QPSK 3/4
        [3] = 18 - 2,  // 16QAM 1/2
        [4] = 18 - 2,  // 16QAM 3/4
        [5] = 16 - 2,  // 64QAM 2/3
        [6] = 16 - 2,  // 64QAM 3/4
        [7] = 16 - 2   // 64QAM 5/6
    }
};

struct trpc_env_tag trpc_env;


void trpc_init()
{
    trpc_env.channel_freq = 2442;
    trpc_env.power_dbm_max_rf = TRPC_DIV10(rf_pri_get_txgain_max());
    trpc_env.power_dbm_min_rf = TRPC_DIV10(rf_pri_get_txgain_min());
    trpc_env.power_dbm_lim_reg = 30;  // TODO: change with region
    trpc_env.temperature = 25;
    trpc_env.temperature_compensate = 0;

#if CFG_PHY_TRPC_DEBUG
    printf("[DEBUG][TRPC] trpc_init 11B MCS0 %d\r\n", trpc_get_default_power_idx(PHY_FORMATMOD_11B, 0));
    printf("[DEBUG][TRPC] trpc_init 11B MCS3 %d\r\n", trpc_get_default_power_idx(PHY_FORMATMOD_11B, 3));
    printf("[DEBUG][TRPC] trpc_init 11G MCS0 %d\r\n", trpc_get_default_power_idx(PHY_FORMATMOD_11G, 0));
    printf("[DEBUG][TRPC] trpc_init 11G MCS2 %d\r\n", trpc_get_default_power_idx(PHY_FORMATMOD_11G, 2));
    printf("[DEBUG][TRPC] trpc_init 11G MCS7 %d\r\n", trpc_get_default_power_idx(PHY_FORMATMOD_11G, 7));
    printf("[DEBUG][TRPC] trpc_init 11N MCS0 %d\r\n", trpc_get_default_power_idx(PHY_FORMATMOD_11N, 0));
    printf("[DEBUG][TRPC] trpc_init 11N MCS3 %d\r\n", trpc_get_default_power_idx(PHY_FORMATMOD_11N, 3));
    printf("[DEBUG][TRPC] trpc_init 11N MCS7 %d\r\n", trpc_get_default_power_idx(PHY_FORMATMOD_11N, 7));
#endif
}

int8_t trpc_get_rf_max_power()
{
    return trpc_env.power_dbm_max_rf;
}

int8_t trpc_get_rf_min_power()
{
    return trpc_env.power_dbm_min_rf;
}

uint8_t trpc_get_default_power_idx(uint8_t formatmod, uint8_t mcs)
{
    // make sure input is in range
    formatmod = TRPC_MIN(formatmod, PHY_FORMATMOD_11N);
    mcs = TRPC_MIN(mcs, formatmod == PHY_FORMATMOD_11B ? 3 : 7);

    return trpc_get_power_idx(formatmod, mcs, txpwr_vs_rate_table[formatmod][mcs]);
}

uint8_t trpc_get_power_idx(uint8_t formatmod, uint8_t mcs, int8_t pwr_dbm)
{
    int8_t pwr_dbm_top;
    int8_t pwr_dbm_bottom;
    int8_t pwr_dbm_guaranteed;
    uint8_t backoff_steps;

    // make sure input is in range
    formatmod = TRPC_MIN(formatmod, PHY_FORMATMOD_11N);
    mcs = TRPC_MIN(mcs, formatmod == PHY_FORMATMOD_11B ? 3 : 7);

    // compare to limits
    pwr_dbm_top = TRPC_MIN(trpc_env.power_dbm_lim_reg, trpc_env.power_dbm_max_rf);
    pwr_dbm_bottom = trpc_env.power_dbm_min_rf;
    pwr_dbm_guaranteed = txpwr_vs_rate_table[formatmod][mcs];

    // make sure target power is in range
    pwr_dbm = TRPC_MIN(pwr_dbm, pwr_dbm_guaranteed);
    pwr_dbm = TRPC_MIN(pwr_dbm, pwr_dbm_top);
    pwr_dbm = TRPC_MAX(pwr_dbm, pwr_dbm_bottom);

#if CFG_PHY_TRPC_DEBUG
    printf("formatmod %d, mcs %d, power top %d, bottom %d, guaranted %d, target %d\r\n",
            formatmod, mcs, pwr_dbm_top, pwr_dbm_bottom, pwr_dbm_guaranteed, pwr_dbm);
#endif

    // backoff from the rf max power
    // the max power of rf changes with frequency (channel) and temperature
    // it is adjusted by thermal calibration and channel update
    backoff_steps = trpc_env.power_dbm_max_rf - pwr_dbm;
    backoff_steps+= formatmod == PHY_FORMATMOD_11B ? 3 : 0; // power of dsss/cck is 3db higher than ofdm
    backoff_steps = TRPC_MIN(15, backoff_steps);

    // the index is shift 2 bits to align with hardware
    return backoff_steps << 2;
}

void trpc_update_vs_channel(int8_t channel_MHz)
{
    trpc_env.channel_freq = channel_MHz;
    trpc_env.power_dbm_max_rf = TRPC_DIV10(rf_pri_get_txgain_max());
    trpc_env.power_dbm_min_rf = TRPC_DIV10(rf_pri_get_txgain_min());
}

void trpc_update_vs_temperature(int8_t temperature)
{
    trpc_env.temperature = temperature;
    trpc_env.power_dbm_max_rf = TRPC_DIV10(rf_pri_get_txgain_max());
    trpc_env.power_dbm_min_rf = TRPC_DIV10(rf_pri_get_txgain_min());
}
