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
#include "phy_bl602.h"
#include "bl_config.h"
#include "phy.h"
#include "dbg.h"
#include "rf.h"

#include "hal_machw.h"
#include "reg_mac_core.h"

#include "reg_mdm_stat.h"
#include "reg_mdm_cfg.h"
#include "reg_mdmdsss_cfg.h"

#include "reg_riu.h"
#include "reg_rc.h"
#include "reg_karst_if.h"
#include "reg_macbypass.h"
#include "rd.h"

#include "reg/reg_bl_rc2.h"
#include "rfc_bl602.h"
#include "reg/reg_rf.h"

#include "reg/reg_bz_phy.h"
#include "reg/reg_bz_phy_agc.h"

#include "phy_trpc.h"
#include "phy_adapt.h"


#define PHY_BAND_2G4_MIN_FREQ    (2412)
#define PHY_BAND_2G4_MAX_FREQ    (2484)

/*
 * TYPEDEFS
 ****************************************************************************************
 */
/// Structure containing the parameters of the PHY configuration
struct phy_bl602_cfg_tag
{
    uint32_t reserved;
};

/// PHY driver context.
struct phy_env_tag
{
    /// Currently configuration parameters
    struct phy_bl602_cfg_tag cfg;
    /// Currently configured 20MHz primary frequency (in MHz)
    uint16_t chnl_prim20_freq;
    /// Currently configured nMHz contiguous channel (of primary 80 if 80+80) (not used)
    uint16_t chnl_center1_freq;
    /// Currently configured secondary 80MHz channel if 80+80, unused otherwise  (not used)
    uint16_t chnl_center2_freq;
    /// Current selected band
    uint8_t band;
    /// 20/40/80/160/80+80 channel type
    uint8_t chnl_type;
};


/*
 * VARIABLES
 ****************************************************************************************
 */
struct phy_env_tag phy_env[1];
extern const uint32_t agcmem[];
static int8_t rxgain[] = {0,9,18,25,32,38,45,53,60};
static int8_t rxgain_offset_vs_temperature = 0;

/*
 * IMPLEMENTATIONS (Common)
 ****************************************************************************************
 */
static inline void mdelay(uint32_t ms)
{
    volatile uint32_t n = ms*40000;
    while (n--);
}

static void agc_download()
{
    uint32_t* p_agcmem = (uint32_t*)(PHY_BL602_AGC_MEM_ADDR);

    riu_agcfsmreset_setf(1);
    mdm_agcmemclkforce_setf(1);

    // Write AGC memory
    for(int i = 0; i < PHY_BL602_AGC_MEM_SIZE/4; i++)
    {
        *p_agcmem = agcmem[i];
        p_agcmem++;
    }

    mdm_agcmemclkforce_setf(0);
    riu_agcfsmreset_setf(0);
}

void phy_config_rxgain(int offset)
{
    if (offset == rxgain_offset_vs_temperature) {
        return;
    }

    rxgain_offset_vs_temperature = offset;
    
    rc2_rx0_lna_gain_0_setf(rxgain_offset_vs_temperature + rxgain[0]); 
    rc2_rx0_lna_gain_1_setf(rxgain_offset_vs_temperature + rxgain[1]); 
    rc2_rx0_lna_gain_2_setf(rxgain_offset_vs_temperature + rxgain[2]);
    rc2_rx0_lna_gain_3_setf(rxgain_offset_vs_temperature + rxgain[3]);
    rc2_rx0_lna_gain_4_setf(rxgain_offset_vs_temperature + rxgain[4]);
    rc2_rx0_lna_gain_5_setf(rxgain_offset_vs_temperature + rxgain[5]);
    rc2_rx0_lna_gain_6_setf(rxgain_offset_vs_temperature + rxgain[6]);
    rc2_rx0_lna_gain_7_setf(rxgain_offset_vs_temperature + rxgain[7]);
    rc2_rx0_lna_gain_8_setf(rxgain_offset_vs_temperature + rxgain[8]);
}

/*
 * IMPLEMENTATIONS (Private Driver)
 ****************************************************************************************
 */

void agc_config()
{
    riu_htstfgainen_setf(0);
    riu_rifsdeten_setf(0);
    riu_fe20gain_setf(0);
    riu_fe40gain_setf(0);
    riu_vpeakadcqdbv_setf(-8+256);
    riu_adcpowmindbm_setf(-98+256);
    riu_adcpowsupthrdbm_setf(-50+256);
    // sat detects too late for some power levels
    riu_satdelay50ns_setf(8);
    riu_sathighthrdbv_setf(-4+64);
    riu_satlowthrdbv_setf(-8+64);
    riu_satthrdbv_setf(-7+64);
    // riu_satdelay50ns_setf(8);
    // riu_sathighthrdbv_setf(-2+64);
    // riu_satlowthrdbv_setf(-10+64);
    // riu_satthrdbv_setf(-9+64);
    // riu_sathighthrdbv_setf(-6+64);
    // riu_satlowthrdbv_setf(-10+64);
    // riu_satthrdbv_setf(-9+64);
    riu_crossdnthrqdbm_setf(-400+512);
    riu_crossupthrqdbm_setf(-400+512);
    riu_rampupgapqdb_setf(18);
    riu_rampupndlindex_setf(5);
    riu_rampdngapqdb_setf(40);
    riu_rampdnndlindex_setf(7);
    riu_adcpowdisthrdbv_setf(-40+128);
    // riu_adcpowdisndl_setf(3);
    riu_idinbdpowgapdnqdbm_setf(24);

    // pkdet pkdet & lnaGainCp2Min & true
    riu_evt0op3_setf(62);
    riu_evt0op2_setf(55);
    riu_evt0op1_setf(1);
    riu_evt0pathcomb_setf(0);
    riu_evt0opcomb_setf(1);

    // sat (vgaGainCp2Min || lnaGainCp2Min) && sat
    riu_evt1op1_setf(57);
    riu_evt1op2_setf(55); // riu_evt1op2_setf(55);
    // riu_evt1op2_setf(0); // riu_evt1op2_setf(55);
    riu_evt1op3_setf(20); //riu_evt1op3_setf(20);
    riu_evt1pathcomb_setf(0);
    riu_evt1opcomb_setf(2);

    // det (rfGainCp2Min & rampUp & ~InbdPowInf)
    riu_evt2op1_setf(15);
    riu_evt2op2_setf(23);
    riu_evt2op3_setf(42);
    riu_evt2pathcomb_setf(0);
    riu_evt2opcomb_setf(5);

    // dis (adcPowDis || rampDn) && rfGainCp2Max
    riu_evt3op1_setf(25);
    riu_evt3op2_setf(0);// riu_evt3op2_setf(24);
    riu_evt3op3_setf(14);
    riu_evt3opcomb_setf(2);

    // dsssdet: dsssDet & true & ~(lnaGain<Max)
    rc2_evt4op1_setf(63);
    rc2_evt4op2_setf(1);
    rc2_evt4op3_setf(54);
    rc2_evt4opcomb_setf(5);

    rc2_pkdet_mode_setf(0);
    rc2_pkdet_cnt_thr_setf(2);
    rc2_pkdet_cnt_thr_setf(2);
    
    #if 0 // fpga project
    rc2_rx0_vga_idx_max_setf(12);
    rc2_rx0_vga_idx_min_setf(3);
    rc2_rx0_lna_idx_max_setf(8);
    rc2_rx0_lna_idx_min_setf(0);
    rc2_rx0_lna_gain_0_setf(13);//rc2_rx0_lna_gain_0_setf(9);
    rc2_rx0_lna_gain_1_setf(19);//rc2_rx0_lna_gain_1_setf(15);
    rc2_rx0_lna_gain_2_setf(24);//rc2_rx0_lna_gain_2_setf(21);
    rc2_rx0_lna_gain_3_setf(27);
    rc2_rx0_lna_gain_4_setf(33);
    rc2_rx0_lna_gain_5_setf(39);
    rc2_rx0_lna_gain_6_setf(45);
    rc2_rx0_lna_gain_7_setf(52);
    rc2_rx0_lna_gain_8_setf(58);
    #else
        #if 1
        rc2_rx0_vga_idx_max_setf(12); // rc2_rx0_vga_idx_max_setf(12);
        rc2_rx0_vga_idx_min_setf(3); // rc2_rx0_vga_idx_min_setf(3);
        
        rc2_rx0_lna_idx_max_setf(8); // rc2_rx0_lna_idx_max_setf(8);
        rc2_rx0_lna_idx_min_setf(0); // rc2_rx0_lna_idx_min_setf(0);

        phy_config_rxgain(0);
        #else
        rc2_rx0_vga_idx_max_setf(12); // rc2_rx0_vga_idx_max_setf(12);
        rc2_rx0_vga_idx_min_setf(3); // rc2_rx0_vga_idx_min_setf(3);
        
        rc2_rx0_lna_idx_max_setf(8); // rc2_rx0_lna_idx_max_setf(8);
        rc2_rx0_lna_idx_min_setf(0); // rc2_rx0_lna_idx_min_setf(0);

        rc2_rx0_lna_gain_0_setf(-6);  // rc2_rx0_lna_gain_0_setf(18);
        rc2_rx0_lna_gain_1_setf(3);  // rc2_rx0_lna_gain_1_setf(21);
        rc2_rx0_lna_gain_2_setf(12); // rc2_rx0_lna_gain_2_setf(26);
        rc2_rx0_lna_gain_3_setf(19); // rc2_rx0_lna_gain_3_setf(32);
        rc2_rx0_lna_gain_4_setf(26); // rc2_rx0_lna_gain_4_setf(38);
        rc2_rx0_lna_gain_5_setf(32); // rc2_rx0_lna_gain_5_setf(43);
        rc2_rx0_lna_gain_6_setf(29); // rc2_rx0_lna_gain_6_setf(51);
        rc2_rx0_lna_gain_7_setf(47); // rc2_rx0_lna_gain_7_setf(55);
        rc2_rx0_lna_gain_8_setf(60); // rc2_rx0_lna_gain_8_setf(61);

        rf_gain_ctrl7_gc_rmxgm_setf(0);
        rf_gain_ctrl6_gc_rmxgm_setf(0);
        rf_gain_ctrl5_gc_rmxgm_setf(0);
        rf_gain_ctrl4_gc_rmxgm_setf(0);
        rf_gain_ctrl3_gc_rmxgm_setf(0);
        rf_gain_ctrl2_gc_rmxgm_setf(0);
        rf_gain_ctrl1_gc_rmxgm_setf(0);
        rf_gain_ctrl0_gc_rmxgm_setf(0);

        rf_rmxgm_10m_mode_en_setf(1);
        #endif
    #endif

    #if 1
    riu_inbdpowmindbm_setf(-98+256);
    riu_inbdpowsupthrdbm_setf(-92+256);
    riu_inbdpowinfthrdbm_setf(-93+256);
    rc2_inbdpow_adj_thr_dbm_setf(-75+256);
    rc2_inbdpowsupthr_adj_en_setf(1);
    rc2_inbdpowinfthr_adj_en_setf(1);
    #else
    riu_inbdpowmindbm_setf(-98+256);
    riu_inbdpowsupthrdbm_setf(-60+256);
    riu_inbdpowinfthrdbm_setf(-70+256);
    rc2_inbdpowsupthr_adj_en_setf(0);
    rc2_inbdpowinfthr_adj_en_setf(0);
    #endif

    rc2_reflevofdmthd_en_setf(1);
    rc2_reflevofdmthd_setf(256); // rc2_reflevofdmthd_setf(300);
    // rc2_reflevofdmthd_setf(256); // rc2_reflevofdmthd_setf(300);
    // rc2_reflevofdmthd_setf(292);
    rc2_reflevdsssthd_en_setf(1);
    rc2_reflevdsssthd_setf(380);
    rc2_reflevdssscontthd_en_setf(1);
    rc2_reflevdssscontthd_setf(256); // rc2_reflevdssscontthd_setf(1024);
    rc2_inbdpowfastvalid_cnt_setf(64);
}

void mdm_reset(void)
{
    uint8_t loop_cnt = 10;
    mdm_swreset_set(0x1111);
    while (loop_cnt--) {
    }
    mdm_swreset_set(0x0);
}

static int phy_mdm_conf(uint8_t chan_type)
{
    int mdmconf = 0;

    switch (chan_type)
    {
        case PHY_CHNL_BW_20:
            mdmconf = 0;
            break;
        // case PHY_CHNL_BW_40:
        //     mdmconf = 2;
        //     break;
        default:
            ASSERT_ERR(0);
            break;
    }

    return mdmconf;
}

void phy_hw_set_channel(uint8_t band, uint16_t freq, uint16_t freq1, uint8_t chantype)
{
    dbg(D_INF "%s: band=%d freq=%d freq1=%d chantype=%d\n",__func__,band,freq,freq1,chantype);

    ASSERT_ERR(chantype == PHY_CHNL_BW_20);
    ASSERT_ERR(band == PHY_BAND_2G4);
    
    riu_ofdmonly_setf(0);
    mdm_rxdsssen_setf(1);
    mdm_mdmconf_setf(0);
    mdm_reset();
    
    // 3us for TX RAMPUP <=> TXRFRAMPUP = 360
    mdm_txstartdelay_setf(180);
    mdm_txctrl1_pack(0, 0, 28, 19);
    mdm_txctrl3_pack(720, 1080);
    mdm_tbe_count_adjust_20_setf(0); // TBE for 60MHz
    mdm_dcestimctrl_pack(0, 0, 0, 15, 15);
    mdm_waithtstf_setf(7); // For FPGA, divide value by 2 due to timing constraints
    mdm_tddchtstfmargin_setf(6);
    mdm_smoothctrl_set(0x01880C06); // smooth enable/auto-selection
    mdm_tbectrl2_set(0x00007F03);

    // No ACI margin in BW=20MHz due to latency on HTSIG decoding
    riu_rwnxagcaci20marg0_set(0);
    riu_rwnxagcaci20marg1_set(0);
    riu_rwnxagcaci20marg2_set(0);
    
    // Configure maximum BW
    mdm_txcbwmax_setf(chantype);
    // #ifndef BL602_PHY_TEST
    //     blmac_max_supported_bw_setf(chantype);
    // #endif

    // Reset RX IQ compensation if available
    if (riu_iqcomp_getf())
    {
        riu_iqestiterclr_setf(1);
    }
    
    // Set RF LO frequency
    rf_set_channel(chantype, freq1);

    // Update TRPC
    trpc_update_vs_channel(freq1);
}

static void tx_power_config()
{
}


static void phy_hw_init(const struct phy_bl602_cfg_tag *cfg)
{

    /* Modem Init */
    mdm_mdmconf_set(phy_mdm_conf(BW_20MHZ));
    mdm_reset();

    // Set Rx Mode
    mdm_rxmode_set(MDM_RXSTBCEN_BIT | MDM_RXGFEN_BIT  |
                   MDM_RXMMEN_BIT   | MDM_RXDSSSEN_BIT);
    mdm_rxnssmax_setf(mdm_nss_getf() - 1);
    mdm_rxndpnstsmax_setf(mdm_nsts_getf() - 1);
    mdm_rxldpcen_setf(mdm_ldpcdec_getf());
    mdm_rxvhten_setf(phy_vht_supported());
    mdm_rxmumimoen_setf(mdm_mumimorx_getf());
    mdm_rxmumimoapeplenen_setf(mdm_mumimorx_getf());

    // Set DSSS precomp
    mdm_precomp_setf(45);
    
    // Rx frame violation check
    //   [31:15]: VHT
    //   [14: 4]: HT
    //   [ 3: 0]: NON-HT
    mdm_rxframeviolationmask_setf(0xFFFFFFFF);

    // Set Tx Mode
    mdm_txmode_set(MDM_TXSTBCEN_BIT | MDM_TXGFEN_BIT  |
                   MDM_TXMMEN_BIT   | MDM_TXDSSSEN_BIT);
    mdm_txnssmax_setf(mdm_nss_getf() - 1);
    mdm_ntxmax_setf(mdm_ntx_getf() - 1);
    mdm_txcbwmax_setf(mdm_chbw_getf());
    mdm_txldpcen_setf(mdm_ldpcenc_getf());
    mdm_txvhten_setf(phy_vht_supported());
    mdm_txmumimoen_setf(mdm_mumimotx_getf());

    // AGC reset mode
    // Don't turn off RF if rxreq de-asserted for few cycles after a RXERR
    mdm_rxtdctrl1_set(mdm_rxtdctrl1_get()|1);

    // Enable automatic smoothing filter selection from SNR, then disable force
    mdm_cfgsmoothforce_setf(0);
    mdm_smoothsnrthrhigh_setf(27);
    mdm_smoothsnrthrmid_setf(15);
    
    // limit NDBPSMAX to 1x1 80 MCS7 LGI(292.5Mb/s) / SGI (325.0Mb/s)
    mdm_rxctrl1_set(0x04920492);

    // Enable RC clock
    mdm_rcclkforce_setf(1);

    /* RIU Init */
    riu_txshift4044_setf(2);

    if (riu_iqcomp_getf())
    {
        riu_rxiqphaseesten_setf(0);
        riu_rxiqgainesten_setf(0);
        riu_rxiqphasecompen_setf(0);
        riu_rxiqgaincompen_setf(0);
        riu_iqestiterclr_setf(0);
    }

    // limit RIU to 1 or 2 antenna active depending on modem capabilities
    riu_activeant_setf(1);

    // limit AGC with a single antenna (path0)
    riu_combpathsel_setf(1);

    // CCA timeout
    riu_rwnxagcccatimeout_set(4000000); // 50ms
    riu_irqmacccatimeouten_setf(1);

    /* configure tx power */
    tx_power_config();
    
    /* configure AGC */
    agc_config();

    /* download AGC memory*/
    agc_download();

    // /* Enable Incremental fcal and LO unlock interrupt 
    //  * incremental calibration must be started after RC clock is ready
    //  */
    // rf_pri_start_inc_cal();
}

/*
 * IMPLEMENTATIONS (Driver API)
 ****************************************************************************************
 */
void phy_init(const struct phy_cfg_tag *config)
{
    const struct phy_bl602_cfg_tag *cfg = (const struct phy_bl602_cfg_tag *)&config->parameters;

    phy_hw_init(cfg);

    phy_env->cfg               = *cfg;
    phy_env->band              = PHY_BAND_2G4;
    phy_env->chnl_type         = PHY_CHNL_BW_OTHER;
    phy_env->chnl_prim20_freq  = PHY_UNUSED;
    phy_env->chnl_center1_freq = PHY_UNUSED;
    phy_env->chnl_center2_freq = PHY_UNUSED;

    // init transmitter rate power control
    trpc_init();

    // init phy adaptive features
    pa_init();
}

void phy_reset()
{
    // TODO
}

void phy_get_channel(struct phy_channel_info *info, uint8_t index)
{
    if (index > 0)
    {
        dbg(D_ERR D_PHY "%s: radio %d does not exist\n", __func__, index);
    }
    
    // Map the band, channel type, primary channel index and center 1 index on info1
    info->info1 = phy_env->band | (phy_env->chnl_type << 8) | (phy_env->chnl_prim20_freq << 16);

    // Map center 2 index on info2
    info->info2 =  phy_env->chnl_center1_freq | (phy_env->chnl_center2_freq << 16);
}

void phy_set_channel(uint8_t band, uint8_t type, uint16_t prim20_freq,
                        uint16_t center1_freq, uint16_t center2_freq, uint8_t index)
{
    // if (phy_env->band == band && phy_env->chnl_type == type &&
    //     phy_env->chnl_prim20_freq == prim20_freq &&
    //     phy_env->chnl_center1_freq == center1_freq)
    // {
    //     printf("%s: Setting same channel, do nothing\r\n",  __func__);
    //     return;
    // }

    if ((center1_freq < PHY_BAND_2G4_MIN_FREQ || center1_freq > PHY_BAND_2G4_MAX_FREQ) &&
        (band == PHY_BAND_2G4)){
        printf("invalid freq:index[%u] c:%d c1:%d bw:%d\r\n", index,
                    prim20_freq, center1_freq, (1 << type) * 20);
        return;
    }

    //TODO: fixed band type to 20MHz. leo
    printf("<<<< %s: c:%d c1:%d bw:%d band:%d\r\n", __func__,
                    prim20_freq, center1_freq, (1 << type) * 20, band);

    if (band != PHY_BAND_2G4 && band != PHY_BAND_5G){
        printf("invalid band:%d\r\n", band);
        return;
    }
    phy_hw_set_channel(band, prim20_freq, center1_freq, type);

    phy_env->band              = band;
    phy_env->chnl_type         = type;
    phy_env->chnl_prim20_freq  = prim20_freq;
    phy_env->chnl_center1_freq = center1_freq;
}

void phy_get_version(uint32_t *version_1, uint32_t *version_2)
{
    *version_1 = mdm_hdmconfig_get();
    *version_2 = 0; // TODO Add version reading for other PHY elements than modem.
}

bool phy_vht_supported(void)
{
    #if NX_MDM_VER > 20
    return ((mdm_vht_getf() != 0) || (mdm_chbw_getf() > PHY_CHNL_BW_40));
    #else
    return true;
    #endif
}

uint8_t phy_get_nss(void)
{
    return (mdm_nss_getf() - 1);
}

uint8_t phy_get_ntx(void)
{
    return (mdm_ntx_getf() - 1);
}

void phy_stop(void)
{
    // TODO
    // if (blmac_current_state_getf() != HW_IDLE)
    //     dbg(D_ERR "%s MAC state != IDLE\n", __func__);
}

bool phy_bfmee_supported(void)
{
    return (mdm_bfmee_getf() != 0);
}

bool phy_bfmer_supported(void)
{
    return (mdm_bfmer_getf() != 0);
}

bool phy_mu_mimo_rx_supported(void)
{
    return (mdm_mumimorx_getf() != 0);
}

bool phy_mu_mimo_tx_supported(void)
{
    return (mdm_mumimotx_getf() != 0);
}

bool phy_ldpc_tx_supported(void)
{
    return (mdm_ldpcenc_getf() != 0);
}

bool phy_ldpc_rx_supported(void)
{
    return (mdm_ldpcdec_getf() != 0);
}

uint8_t phy_get_mac_freq(void)
{
    return PHY_BL602_MACCORE_FREQ_MHZ;
}

void phy_get_rf_gain_idx(int8_t *power, uint8_t *idx)
{
    int32_t power_dbm = *power;

    *idx = rfc_get_power_level(RFC_FORMATMOD_11N, power_dbm * 10);
}

void phy_get_trpc_idx(uint8_t formatmod, uint8_t mcs, int8_t power, uint8_t *idx)
{
    ASSERT_ERR(formatmod <= PHY_FORMATMOD_11N);
    ASSERT_ERR((formatmod == PHY_FORMATMOD_11B) && (mcs <= 3));
    ASSERT_ERR((formatmod == PHY_FORMATMOD_11G) && (mcs <= 7));
    ASSERT_ERR((formatmod == PHY_FORMATMOD_11N) && (mcs <= 7));

    *idx = trpc_get_power_idx(formatmod, mcs, power);
}

void phy_get_rf_gain_capab(int8_t *max, int8_t *min)
{
    *max = trpc_get_rf_max_power();  // dBm
    *min = trpc_get_rf_min_power();  // dBm
}

void bz_phy_reset()
{
    int tx_rampup_time_us = 8;
	int tx_rampdn_time_us = 4; // 4
	int tx_padzero_time_us = 0;

    // configure phy registers
	bz_phy_tx_rampup_fm_on_setf(1);
	bz_phy_tx_rampup_time_us_setf(tx_rampup_time_us);
	bz_phy_tx_rampdn_fm_on_setf(1);
	bz_phy_tx_rampdn_time_us_setf(tx_rampdn_time_us);
	bz_phy_tx_rampdn_pad0_time_us_setf(tx_padzero_time_us);	
	bz_phy_rx_proc_time_mlsd_us_setf(32);   // 30
	bz_phy_rx_proc_time_direct_us_setf(30); // rtl bug
	bz_phy_rx_proc_time_eq_us_setf(10);
	bz_phy_rx_proc_time_viterbi_us_setf(30);
	bz_phy_rx_dfe_notch_en_setf(0);
	bz_phy_rx_dfe_toc_en_setf(1);

	// configure agc regsiters
	bz_agc_rbb_ind_min_setf(4);
}

/**
* Empty functions to compatible with karst rf
*
*/
void mpif_clk_init(void)
{
    
}

#if RW_MUMIMO_RX_EN
void phy_set_group_id_info(uint32_t membership_addr, uint32_t userpos_addr)
{
    int i;

    // Set membership status
    for(i=0; i<MDM_MUMIMO_GROUPID_TAB_COUNT; i++)
    {
        mdm_mumimo_groupid_tab_set(i, co_read32p(membership_addr + 4 * i));
    }

    // Set user position
    for(i=0; i<MDM_MUMIMO_USERPOSITION_TAB_COUNT; i++)
    {
        mdm_mumimo_userposition_tab_set(i, co_read32p(userpos_addr + 4 * i));
    }
}
#endif

#if NX_DEBUG_DUMP
void phy_get_diag_state(struct dbg_debug_info_tag *dbg_info)
{
}
#endif


/*
 * IMPLEMENTATIONS (ISR)
 ****************************************************************************************
 */
void phy_mdm_isr(void)
{
    dbg(D_ERR D_PHY "%s: TODO\n", __func__);
}

void phy_rc_isr(void)
{
    dbg("%s: cca timeout\n", __func__);
    uint32_t irq_status = riu_rwnxmacintstatmasked_get();

    riu_rwnxmacintack_clear(irq_status);

    // TODO: CCA timeout recovery
    if (irq_status & RIU_IRQMACCCATIMEOUTMASKED_BIT) {
        mdm_reset();
        //ASSERT_REC(!(irq_status & RIU_IRQMACCCATIMEOUTMASKED_BIT));
    }
}
