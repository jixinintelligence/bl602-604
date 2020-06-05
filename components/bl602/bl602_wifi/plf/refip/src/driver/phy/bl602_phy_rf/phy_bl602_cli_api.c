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
#include "phy_bl602_cli.c"
#include <bl602_rf_private.h>
#include <rfc_bl602.h>
#include <bl_phy_api.h>

//export api

//equal cmd_init
void bl_mfg_phy_init()
{
    printf("start rf/rc/phy init\r\n");
    //40Mhz
    rfc_init(40*1000*1000);
    phy_mdm_reset();
    phy_init(&s_phycfg); 
    phy_set_channel(PHY_BAND_2G4, PHY_CHNL_BW_20, 0, 2412, 2412, 0);
}

void bl_mfg_rf_cal()
{
    //alway reset for full calibration flow running
    rf_pri_init(1);
}

//equal cmd_rx_start_at
void bl_mfg_rx_start()
{
    // reset
    phy_mdm_reset();
    phy_macbyp_reset();
    // clear
    phy_macbyp_clr_status();
    // start
    phy_macbyp_rx_start();
    printf("at_rx_start ok\r\n");
}

void bl_mfg_rx_stop()
{
    printf("at_rx_poll ok\r\n");
    phy_macbyp_rx_print_status_at();
    phy_macbyp_reset();
}

void bl_mfg_channel_switch(uint8_t chan_no)
{
    static const uint32_t no_to_chanfreq_table[14] = {2412, 2417, 2422, 2427, 
                                                      2432, 2437, 2442, 2447, 
                                                      2452, 2457, 2462, 2467, 
                                                      2472, 2484};
                                                     
    uint32_t chanfreq = no_to_chanfreq_table[chan_no];
    printf("switch channel to %ld MHz\r\n",chanfreq);
    rfc_config_channel(chanfreq);
    // phy_set_channel(PHY_BAND_2G4, PHY_CHNL_BW_20, 0, chanfreq, chanfreq, 0);
}

int8_t bl_mfg_tx11n_start(uint8_t mcs_n, uint16_t frame_len, uint8_t pwr_dbm)
{
    int32_t mcs = mcs_n;
    int32_t payload_len = frame_len;
    int32_t filter_sel = 0;
    int32_t diggain = 32;
    int32_t dvga = 0;
    int32_t ifs_us = 100;

    if (mcs < 0 || mcs > 7) {
        printf("mcs error\r\n");
        return -1;
    }
    
    if (frame_len == 0) {
        printf("frame_len error\r\n");
        return -1;
    }

    pwr_dbm = pwr_dbm * 10;
    // configure txcfg
    txcfg_reset(&s_txcfg);
    s_txcfg.macbyp_mode = MACBYP_TX_BURST;
    s_txcfg.macbyp_nframes_in_burst = 0;
    s_txcfg.macbyp_ifs_us = ifs_us;
    s_txcfg.formatmod = HT_MM;
    s_txcfg.ht_mcs = s_rate_table_11n[mcs];
    s_txcfg.ht_length = payload_len;
    s_txcfg.fe_ofdm_filter_sel = filter_sel ? 0xff00 : 0x0000;
    s_txcfg.fe_ofdm_gain = diggain;
    s_txcfg.fe_dvga_qdb = dvga;
    s_txcfg.pwrlevel = rfc_get_power_level(RFC_FORMATMOD_11N, pwr_dbm);
    // reset
    tx_stop();
    phy_mdm_reset();
    phy_macbyp_reset();

    // configure and start transmit
    phy_start_transmit(&s_txcfg);
    
    return 0;
}

int8_t bl_mfg_tx11b_start(uint8_t mcs_b, uint16_t frame_len, uint8_t pwr_dbm)
{
    int32_t mcs = mcs_b;
    int32_t payload_len = frame_len;
    int32_t filter_sel = 0;
    int32_t diggain_dsss = 32;
    int32_t dvga = 0;
    int32_t ifs_us = 100;

    if (mcs < 1 || mcs > 4) {
        printf("mcs error\r\n");
        return -1;
    }
    
    if (frame_len == 0) {
        printf("frame_len error\r\n");
        return -1;
    }    

    pwr_dbm = pwr_dbm * 10;
    // configure txcfg
    txcfg_reset(&s_txcfg);
    s_txcfg.macbyp_mode = MACBYP_TX_BURST;
    s_txcfg.macbyp_nframes_in_burst = 0;
    s_txcfg.macbyp_ifs_us = ifs_us;
    s_txcfg.formatmod = NONHT;
    s_txcfg.lm_rate = LM11B_MCS1 + (mcs - 1);
    s_txcfg.lm_length = payload_len;
    s_txcfg.fe_dsss_filter_sel = filter_sel;
    s_txcfg.fe_dsss_gain = diggain_dsss;
    s_txcfg.fe_dvga_qdb = dvga;
    s_txcfg.pwrlevel = rfc_get_power_level(RFC_FORMATMOD_11B, pwr_dbm);

    // reset
    tx_stop();
    phy_macbyp_reset();
    phy_mdm_reset();

    // configure and start transmit
    phy_start_transmit(&s_txcfg);
    return 0;
}

void bl_mfg_tx_stop()
{
    tx_stop();   
}
