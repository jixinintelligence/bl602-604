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
#include <string.h>
#include "bl602_rf_calib_data.h"

#if 0
#define RF_CALIB_DATA_ADDR      (0x40020000)

rf_calib_data_tag* rf_calib_data = (rf_calib_data_tag *)RF_CALIB_DATA_ADDR;
#else
static rf_calib_data_tag data;
rf_calib_data_tag* rf_calib_data = &data;
#endif


void rf_pri_init_calib_mem(void)
{
    memset((void*)rf_calib_data, 0, sizeof(rf_calib_data_tag));
    uint8_t i;
    for(i=0;i<4;i++){
        rf_calib_data->rxcal[i].rosdac_i = 32;
        rf_calib_data->rxcal[i].rosdac_q = 32;   
        rf_calib_data->rxcal[i].rx_iq_gain_comp = 1024;
        rf_calib_data->rxcal[i].rx_iq_phase_comp = 0;             
    }
    // rf_calib_data->cal.rx_offset_i = 0;
    // rf_calib_data->cal.rx_offset_q = 0;
    rf_calib_data->cal.rbb_cap1_fc_i = 32;
    rf_calib_data->cal.rbb_cap1_fc_q = 32;
    rf_calib_data->cal.rbb_cap2_fc_i = 32;
    rf_calib_data->cal.rbb_cap2_fc_q = 32;
    rf_calib_data->cal.tx_dc_comp_i = 0;
    rf_calib_data->cal.tx_dc_comp_q = 0;    
    for(i=0;i<8;i++){
        rf_calib_data->txcal[i].tosdac_i = 32;
        rf_calib_data->txcal[i].tosdac_q = 32;   
        rf_calib_data->txcal[i].tx_iq_gain_comp = 1024;
        rf_calib_data->txcal[i].tx_iq_phase_comp = 0;             
    }
    rf_calib_data->cal.tmx_csh = 6;
    rf_calib_data->cal.tmx_csl = 6;//7;

    #if 0 
    rf_calib_data->lo[E_RF_CHANNEL_2412M-1].fcal = 0x7c;
    rf_calib_data->lo[E_RF_CHANNEL_2417M-1].fcal = 0x7b;
    rf_calib_data->lo[E_RF_CHANNEL_2422M-1].fcal = 0x7a;
    rf_calib_data->lo[E_RF_CHANNEL_2427M-1].fcal = 0x79;
    rf_calib_data->lo[E_RF_CHANNEL_2432M-1].fcal = 0x78;
    rf_calib_data->lo[E_RF_CHANNEL_2437M-1].fcal = 0x78;
    rf_calib_data->lo[E_RF_CHANNEL_2442M-1].fcal = 0x77;
    rf_calib_data->lo[E_RF_CHANNEL_2447M-1].fcal = 0x76;
    rf_calib_data->lo[E_RF_CHANNEL_2452M-1].fcal = 0x75;
    rf_calib_data->lo[E_RF_CHANNEL_2457M-1].fcal = 0x74;
    rf_calib_data->lo[E_RF_CHANNEL_2462M-1].fcal = 0x74;
    rf_calib_data->lo[E_RF_CHANNEL_2467M-1].fcal = 0x73;
    rf_calib_data->lo[E_RF_CHANNEL_2472M-1].fcal = 0x72;
    rf_calib_data->lo[E_RF_CHANNEL_2484M-1].fcal = 0x70;
    rf_calib_data->lo[E_RF_CHANNEL_2505P6M-1].fcal = 0x67;

    rf_calib_data->lo[E_RF_CHANNEL_2412M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2417M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2422M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2427M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2432M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2437M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2442M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2447M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2452M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2457M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2462M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2467M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2472M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2484M-1].acal = 0xf;
    rf_calib_data->lo[E_RF_CHANNEL_2505P6M-1].acal = 0xf;
    #endif

    rf_calib_data->inited = 0;
}
