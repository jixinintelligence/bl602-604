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
#ifndef RF_CALIB_DATA_H
#define RF_CALIB_DATA_H

// #include "co_int.h"
#include "bl602_rf_private.h"

/// Definitions of RF without LO calibration structure
typedef struct
{
    uint32_t gpadc_oscode      : 12;

    uint32_t rx_offset_i       : 10;
    uint32_t rx_offset_q       : 10;

    uint32_t rbb_cap1_fc_i     : 6;
    uint32_t rbb_cap1_fc_q     : 6;
    uint32_t rbb_cap2_fc_i     : 6;
    uint32_t rbb_cap2_fc_q     : 6;    

    uint32_t tx_dc_comp_i      : 12;
    uint32_t tx_dc_comp_q      : 12;

    uint32_t tmx_cs            : 3;
    uint32_t txpwr_att_rec     : 3;
    uint32_t pa_pwrmx_osdac    : 4;
    uint32_t tmx_csh           : 3;
    uint32_t tmx_csl           : 3;

    uint32_t tsen_refcode_rfcal : 12;
    uint32_t tsen_refcode_corner : 12;

    uint32_t rc32k_code_fr_ext : 10;
    uint32_t rc32m_code_fr_ext : 8;
    uint32_t saradc_oscode     : 10;
    uint16_t fcal_4osmx        : 4;

} rf_calib1_tag;

/// Definitions of LO dependent calibration structure #pragma pack (2)
typedef struct 
{
    uint16_t fcal       : 8;
    uint16_t acal       : 5;
} rf_calib2_tag;

// 
typedef struct 
{
    uint32_t rosdac_i          : 6;
    uint32_t rosdac_q          : 6;
    uint32_t rx_iq_gain_comp   : 11;
    uint32_t rx_iq_phase_comp  : 10;    
} rf_calib3_tag;

// 
typedef struct 
{
    uint32_t tosdac_i          : 6;
    uint32_t tosdac_q          : 6;
    uint32_t tx_iq_gain_comp   : 11;
    uint32_t tx_iq_phase_comp  : 10;
} rf_calib4_tag;

/// Definitions of the RF calibration result structure
typedef volatile struct
{
    uint32_t        inited;
    rf_calib1_tag   cal;
    rf_calib2_tag   lo[E_RF_CHANNEL_NUM];
    rf_calib3_tag   rxcal[E_RF_RXCAL_GAIN_CNT];      // 0:rosdac_ctrl_hw1.rosdac_i/q_gc0 + rxiq_ctrl_hw1, .., 3:rosdac_ctrl_hw2.rosdac_i/q_gc3 + rxiq_ctrl_hw4
    rf_calib4_tag   txcal[E_RF_TXCAL_GAIN_CNT];       // 0:tosdac_ctrl_hw1.tbb_tosdac_i/q_gc0 + tx_iq_gain_hw0, .., 7:tosdac_ctrl_hw4.tbb_tosdac_i/q_gc7 + tx_iq_gain_hw7
} rf_calib_data_tag;

/// export rf calibration data
extern rf_calib_data_tag* rf_calib_data;

/// init calibration data
void rf_pri_init_calib_mem(void);

#endif
