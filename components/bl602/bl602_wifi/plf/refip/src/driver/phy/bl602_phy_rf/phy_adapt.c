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
#include "phy.h"
#include "phy_hal.h"
#include "hal_desc.h"

#include <stdint.h>
#include <stdio.h>

#define NUM_ADAPT_MAX   (4)
#define INPUT_BUF_SIZE  (8)
#define CELP_ALPHA      (5)
#define CELP_EXP        (6)
#define CELP_HYST       (5)
#define PA_ABS(a)          (((a)>=0) ? a : -a)

// ================================
// Format Modulation selection
// 0 NON-HT
// 1 NON-HT-DUP-OFDM
// 2 HT-MF
// 3 HT-GF
// 4 VHT

// ================================
//  +-----+---------+------+
//  | MCS | LegRate | Mbps |
//  +-----+---------+------+
//  |  0  | 4'b0000 |   1  |
//  |  1  | 4'b0001 |   2  |
//  |  2  | 4'b0010 | 5.5  |
//  |  3  | 4'b0011 |  11  |
//  |  4  | 4'b1011 |   6  |
//  |  5  | 4'b1111 |   9  |
//  |  6  | 4'b1010 |  12  |
//  |  7  | 4'b1110 |  18  |
//  |  8  | 4'b1001 |  24  |
//  |  9  | 4'b1101 |  36  |
//  | 10  | 4'b1000 |  48  |
//  | 11  | 4'b1100 |  54  |
//  +-----+---------+------+

// TODO: Version 2.0: 
//   1) introduce cmd list
//   2) issue all cmd to upper layer and handled there
//   3) clear cmd list if an abnormal situation is detected


typedef struct {
    int8_t rssi;
    int8_t lna;
    float  ppm;
    uint8_t new;
} input_t;

typedef struct {
    uint8_t used;
    uint32_t vif_tag;

    input_t input_buffer[INPUT_BUF_SIZE];
    int8_t input_buffer_ptr;
    uint32_t last_update;

    /* received signal strength */
    int8_t rss;
    int8_t rss_acq;
    int8_t rss_trk;
    int8_t rss_state;
    uint8_t rss_hit_count;
    uint32_t rss_count;

    /* received interference strength */
    int8_t ris;

    /* clock error */
    float ce;
    int8_t ce_in;
    int8_t ce_acq;
    int8_t ce_trk;
    int8_t ce_state;
    int8_t ce_num_up_cmds;
    int8_t ce_num_dn_cmds;

} pa_state_t;

typedef struct {
    /** Receive Vector 1a */
    uint32_t    leg_length         :12;
    uint32_t    leg_rate           : 4;
    uint32_t    ht_length          :16;
    /** Receive Vector 1b */
    uint32_t    _ht_length         : 4; // FIXME
    uint32_t    short_gi           : 1;
    uint32_t    stbc               : 2;
    uint32_t    smoothing          : 1;
    uint32_t    mcs                : 7;
    uint32_t    pre_type           : 1;
    uint32_t    format_mod         : 3;
    uint32_t    ch_bw              : 2;
    uint32_t    n_sts              : 3;
    uint32_t    lsig_valid         : 1;
    uint32_t    sounding           : 1;
    uint32_t    num_extn_ss        : 2;
    uint32_t    aggregation        : 1;
    uint32_t    fec_coding         : 1;
    uint32_t    dyn_bw             : 1;
    uint32_t    doze_not_allowed   : 1;
    /** Receive Vector 1c */
    uint32_t    antenna_set        : 8;
    uint32_t    partial_aid        : 9;
    uint32_t    group_id           : 6;
    uint32_t    reserved_1c        : 1;
    int32_t     rssi1              : 8;
    /** Receive Vector 1d */
    int32_t     rssi2              : 8;
    //int32_t     rssi3              : 8;
    //int32_t     rssi4              : 8;
    int32_t     agc_lna            : 4;
    int32_t     agc_rbb1           : 5;
    int32_t     agc_dg             : 7;

    uint32_t    reserved_1d        : 8;
    /** Receive Vector 2a */
    uint32_t    rcpi               : 8;
    uint32_t    evm1               : 8;
    uint32_t    evm2               : 8;
    uint32_t    freqoff_lo         : 8;
    //uint32_t    evm3               : 8;
    /** Receive Vector 2b */
    uint32_t    freqoff_hi         : 8;
    //uint32_t    evm4               : 8;
    uint32_t    reserved2b_1       : 8;
    uint32_t    reserved2b_2       : 8;
    uint32_t    reserved2b_3       : 8;
} rvec_t;

static pa_state_t pa_env[NUM_ADAPT_MAX];

void pa_init()
{
    int i, j;

    for (i = 0; i < NUM_ADAPT_MAX; i++) {
        pa_env[i].rss_state = 0;
        pa_env[i].rss_count = 0;
        pa_env[i].last_update = 0;
        pa_env[i].input_buffer_ptr = 0;
        pa_env[i].ce_state = 0;
        pa_env[i].ce = 0;
        pa_env[i].ce_num_up_cmds = 0;
        pa_env[i].ce_num_dn_cmds = 0;
        for (j = 0; j < INPUT_BUF_SIZE; j++) {
            pa_env[i].input_buffer[j].new = 0;
            pa_env[i].input_buffer[j].rssi = 0;
            pa_env[i].input_buffer[j].lna = 0;
            pa_env[i].input_buffer[j].ppm = 0;
        }
    }
}

// bind pa resource to a virtual interface
int8_t pa_alloc(uint32_t vif_addr)
{
    uint8_t i;
    for (i = 0; i < NUM_ADAPT_MAX; i++) {
        if (pa_env[i].used == 0) {
            pa_env[i].used = 1;
            pa_env[i].vif_tag = vif_addr;
            return i;
        }
    }

    return -1;
}

void pa_free(uint8_t id)
{
    if (id >= NUM_ADAPT_MAX) {
        return;
    }

    pa_env[id].used = 0;
    pa_env[id].vif_tag = 0;
}

void pa_reset(uint8_t id)
{
    if (id >= NUM_ADAPT_MAX) {
        return;
    }
}

static float calc_ppm_ofdm(uint16_t rxv_freqoff)
{
    int16_t freqoff;
    float ppm;
    
    freqoff = (int16_t)rxv_freqoff;
    ppm = -1 * freqoff * 20.0 / 2440;   // 602 as the center
    // ppm = ppm/(s_nbmdm_on+1);

    return ppm;
}

static float calc_ppm_dsss(uint8_t rxv_freqoff)
{
    int8_t freqoff;
    float ppm;

    freqoff = (int8_t)rxv_freqoff;
    ppm = freqoff * 0.7;  // 602 as the center

    return ppm;
}

static float calc_ppm(rvec_t *rvec)
{
    if (rvec->format_mod == 0 && rvec->leg_rate < 4) {
        return calc_ppm_dsss(rvec->freqoff_lo);
    }
    else {
        return calc_ppm_ofdm(rvec->freqoff_lo + (rvec->freqoff_hi << 8));
    }
}

void pa_input(uint8_t id, struct rx_hd *rhd)
{
    rvec_t *rvec;
    pa_state_t *pa;

    if (id >= NUM_ADAPT_MAX) {
        return;
    }

    // Retrieve RSSI/CE/GAIN information from the RX vector
    rvec = (rvec_t*)&(rhd->recvec1a);

    // Fill input
    pa = &pa_env[id];
    pa->last_update = rhd->tsflo;
    pa->input_buffer[pa->input_buffer_ptr].new = 1;
    pa->input_buffer[pa->input_buffer_ptr].rssi = rvec->rssi1;
    pa->input_buffer[pa->input_buffer_ptr].lna = rvec->agc_lna;
    pa->input_buffer[pa->input_buffer_ptr].ppm = calc_ppm(rvec);
    pa->input_buffer_ptr = pa->input_buffer_ptr == (INPUT_BUF_SIZE-1) ? 0 : pa->input_buffer_ptr + 1;
}

volatile float _global_ppm = 0.0;

void pa_adapt(uint8_t id)
{
    pa_state_t *pa;
    input_t *inp;
    int32_t rssi_sum, rssi_max, rssi_min, rssi_input;
    int32_t rssi;
    int i;
    static uint32_t count = 0;

    if (id >= NUM_ADAPT_MAX) {
        return;
    }

    count = count + 1;
    pa = &pa_env[id];
    inp = pa->input_buffer_ptr == 0 ? &pa->input_buffer[INPUT_BUF_SIZE-1] : &pa->input_buffer[pa->input_buffer_ptr-1];
    rssi = inp->rssi;

    // sanity check if no input for a long while
    if (inp->new == 0) {
        // restore
        return;
    }

    // RSS calculation
    do {
        if (pa->rss_state) {
            // restart if the input rssi is rejected for a long while
            if (pa->rss_hit_count > 4) {
                pa->rss_hit_count = 0;
                pa->rss_state = 0;
                pa->rss_count = 0;

                #if CFG_PHY_ADAPT_RSSI_PRINT
                printf("restart rss loop\r\n");
                #endif

                continue;
            }

            // reject rare numbers
            if (rssi > 0 || rssi < -100) {
                continue;
            }

            // reject large variations
            if ((rssi - pa->rss > 10) || (rssi - pa->rss < -10)) {
                pa->rss_hit_count++;
                continue;
            }

            // update
            pa->rss_hit_count = 0;
            pa->rss += (16*((int16_t)rssi - (int16_t)pa->rss)) >> 6;

            if ((count & 0x3) == 0) {
                //printf("pa %lud, rssi %ld, rss trk %d, ppm %.2f\r\n",pa->last_update,rssi,pa->rss,inp->ppm);
                _global_ppm = inp->ppm;
            }
            #if CFG_PHY_ADAPT_RSSI_PRINT
            if ((count & 0x7) == 0) {
                //printf("pa %lud, rssi %ld, rss trk %d, ppm %.2f\r\n",pa->last_update,rssi,pa->rss,inp->ppm);
            }
            #endif
        }
        else {
            if (pa->rss_count >= 7) {
                pa->rss_state = 1;

                rssi_max = -100;
                rssi_min = +100;
                rssi_sum = 0;
                for (i = 0; i < 6; i++) {
                    rssi_input = pa->input_buffer[(pa->input_buffer_ptr-1-i) % INPUT_BUF_SIZE].rssi;
                    rssi_sum += rssi_input;
                    rssi_max = rssi_input > rssi_max ? rssi_input : rssi_max;
                    rssi_min = rssi_input < rssi_min ? rssi_input : rssi_min;
                }
                
                pa->rss = (int8_t)((rssi_sum - rssi_max - rssi_min)/4);

                #if CFG_PHY_ADAPT_RSSI_PRINT
                printf("pa %lud, rss acq %d\r\n",pa->last_update,pa->rss);
                #endif
            }
            else {
                pa->rss_count += 1;
            }
        }
    } while (0);

    // CE calculation
    if ((count & 0xf) == 0xf) {
        do {
            if ((rssi - pa->rss > 10) || (rssi - pa->rss < -10)) {
                continue;
            }

            if (PA_ABS(inp->ppm) > 2) {
                if (rssi < -85) {
                    pa->ce += inp->ppm * 0.03125;
                }
                else {
                    pa->ce += inp->ppm * 0.125;
                }
            }
            

            #if 0
            if (pa->ce_state) {
                if ((rssi - pa->rss > 10) || (rssi - pa->rss < -10)) {
                    continue;
                }

                // reject large variations
                if (inp->ppm > 60 || inp->ppm < -60) {
                    continue;
                }
                
                pa->ce += inp->ppm * 0.125;
                printf("pa %lud, ce trk %f\r\n",pa->last_update,pa->ce);

            }
            else {
                if (pa->rss_state) {
                    pa->ce_state = 1;
                    ce_min = -100;
                    ce_max = +100;
                    ce_sum = 0;
                    for (i = 0; i < 6; i++) {
                        ce_input = pa->input_buffer[(pa->input_buffer_ptr-1-i) % INPUT_BUF_SIZE].ppm;
                        ce_sum += ce_input;
                        ce_max = ce_input > ce_max ? ce_input : ce_max;
                        ce_min = ce_input < ce_min ? ce_input : ce_min;
                    }
                    
                    pa->ce = (ce_sum - ce_max - ce_min)/4;

                    printf("pa %lud, ce acq %f\r\n",pa->last_update,pa->ce);
                }
            }
            #endif
        } while (0);
    
        // CE adjust
        do {
            // if (pa->ce_state == 0) {
            //     continue;
            // }

            if (PA_ABS(pa->ce) < CELP_HYST) {
                //printf("pa %lud, ce trk %.2f\r\n",pa->last_update,pa->ce);
                continue;
            }

            printf("pa %lud, ce trk %.2f, ",pa->last_update,pa->ce);
            printf("action: capcode %d", hal_get_capcode());
            GLOBAL_INT_DISABLE();
            if (pa->ce > 0 && hal_get_capcode() > 0) {
                hal_set_capcode(hal_get_capcode()-1);
            }
            if (pa->ce < 0 && hal_get_capcode() < 63) {
                hal_set_capcode(hal_get_capcode()+1);
            }
            pa->ce = 0;
            GLOBAL_INT_RESTORE();
            printf(" -> %d\r\n", hal_get_capcode());

            #if 0
            if (pa->ce > 0) {
                pa->ce_num_dn_cmds += (int)(pa->ce/2);`
            }
            else {
                pa->ce_num_up_cmds += (int)(pa->ce/2);
            }

            if (pa->ce_num_dn_cmds > pa->ce_num_up_cmds) {
                pa->ce_num_dn_cmds -= pa->ce_num_up_cmds;
                pa->ce_num_up_cmds = 0;
            }
            else {
                pa->ce_num_up_cmds -= pa->ce_num_dn_cmds;
                pa->ce_num_dn_cmds = 0;
            }

            printf("    action: up %d, dn %d, capcode %d", pa->ce_num_up_cmds, pa->ce_num_dn_cmds, hal_get_capcode());

            if (pa->ce_num_up_cmds) {
                GLOBAL_INT_DISABLE();
                if ( pa->ce_num_up_cmds + hal_get_capcode() < 63) {
                    hal_set_capcode(hal_get_capcode() + pa->ce_num_up_cmds);
                }
                else {
                    hal_set_capcode(63);
                }
                GLOBAL_INT_RESTORE();
                pa->ce_num_up_cmds = 0;
            }

            if (pa->ce_num_dn_cmds) {
                GLOBAL_INT_DISABLE();
                if ( hal_get_capcode() >= pa->ce_num_dn_cmds) {
                    hal_set_capcode(hal_get_capcode() - pa->ce_num_dn_cmds);
                }
                else {
                    hal_set_capcode(0);
                }
                GLOBAL_INT_RESTORE();
                pa->ce_num_dn_cmds = 0;
            }

            printf("->%d\r\n", hal_get_capcode());
            #endif

        } while (0);
    }

    // CCA adjust

    // AGC adjust

    // SM adjust

}
