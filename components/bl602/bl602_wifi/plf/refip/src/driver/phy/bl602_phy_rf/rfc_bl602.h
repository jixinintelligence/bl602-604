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
#ifndef _RFC_BL602_H_
#define _RFC_BL602_H_

#include <stdint.h>

enum
{
    RFC_OFF = 0,
    RFC_ON  = 1
};

enum 
{
    RFC_PM_MUX_TO_ADC = 0,
    RFC_PM_MUX_TO_IQCOMP = 1,
    RFC_PM_MUX_TO_BLE = 2
};

enum 
{
    RFC_TXDFE_IN_MUX_TO_BB = 0,
    RFC_TXDFE_IN_MUX_TO_SRAM = 1,
    RFC_TXDFE_IN_MUX_TO_SINGEN = 2,
    RFC_TXDFE_IN_MUX_TO_PAD = 3
};

enum 
{
    RFC_FSM_PD = 0,
    RFC_FSM_SB = 1,
    RFC_FSM_LO = 2,
    RFC_FSM_RX = 3,
    RFC_FSM_TX = 4,
    RFC_FSM_FORCE_OFF = 15
};

enum 
{
    RFC_BBMODE_WLAN = 0,
    RFC_BBMODE_BLE = 1
};

enum 
{
	RFC_SG_SINGLE_TONE = 0,
	RFC_SG_TWO_TONE,
	RFC_SG_RAMP
};

enum {
    RFC_PC_AUTO = 0,
    RFC_PC_WLAN_11B,
    RFC_PC_WLAN_11G,
    RFC_PC_WLAN_11N,
    RFC_PC_BT_BLE
};

enum {
    RFC_FORMATMOD_11B = 0,
    RFC_FORMATMOD_11G = 1,
    RFC_FORMATMOD_11N = 2
};

enum {
    RFC_BW_20M = 0,
    RFC_BW_10M = 1
};

void rfc_init(uint32_t xtalfreq);
void rfc_reset();
void rfc_config_channel(uint32_t channel_freq);
void rfc_config_channel_sw(uint32_t channel_freq);
void rfc_write_reg(uint32_t a, uint32_t d);
uint32_t rfc_read_reg(uint32_t a);

void rfc_config_power(uint32_t mode,uint32_t tbb_boost,uint32_t tbb,uint32_t tmx);
void rfc_config_bandwidth(uint32_t mode);

uint32_t rfc_get_power_level(uint32_t mode, int32_t power);

/*
 * TXDFE DECLARATIONS
 ****************************************************************************************
 */
void rfc_txdfe_start();
void rfc_txdfe_stop();
void rfc_txdfe_mux(int8_t signal_source);
void rfc_txdfe_set_dvga(int8_t dvga_qdb);
void rfc_txdfe_set_iqgaincomp(uint8_t en,uint16_t coeff);
void rfc_txdfe_set_iqphasecomp(uint8_t en,int16_t coeff);
void rfc_txdfe_set_dccomp(int16_t dcc_i,int16_t dcc_q);
void rfc_txdfe_set_iqswap(uint8_t swap_on);

/*
 * RXDFE DECLARATIONS
 ****************************************************************************************
 */
void rfc_rxdfe_start();
void rfc_rxdfe_stop();
void rfc_rxdfe_set_iqgaincomp(uint8_t en,uint16_t coeff);
void rfc_rxdfe_set_iqphasecomp(uint8_t en,int16_t coeff);
void rfc_rxdfe_set_dccomp(int16_t dcc_i,int16_t dcc_q);
void rfc_rxdfe_set_iqswap(uint8_t swap_on);
void rfc_rxdfe_set_notch0(uint8_t en,uint8_t alpha,int8_t nrmfc);
void rfc_rxdfe_set_notch1(uint8_t en,uint8_t alpha,int8_t nrmfc);

/*
 * SG DECLARATIONS
 ****************************************************************************************
 */
void rfc_sg_start(uint32_t inc_step,uint32_t gain_i,uint32_t gain_q,uint32_t addr_i,uint32_t addr_q);
void rfc_sg_stop();

/*
 * PM DECLARATIONS
 ****************************************************************************************
 */

uint32_t rfc_pm_start(uint32_t insel,int32_t freq_cw,uint32_t acclen,uint32_t rshiftlen,
                     int32_t *raw_acc_i,int32_t *raw_acc_q);
void rfc_pm_stop();

/*
 * HWCTRL DECLARATIONS
 ****************************************************************************************
 */
void rfc_hwctrl_txrfgain(uint8_t hwctrl_on);
void rfc_hwctrl_rxgain(uint8_t hwctrl_on);
void rfc_hwctrl_txdvga(uint8_t hwctrl_on);
void rfc_hwctrl_calparam(uint8_t hwctrl_on);

/*
 * FSM DECLARATIONS
 ****************************************************************************************
 */
void rfc_lo_set_regs(uint32_t xtal_freq);
void rfc_lo_set_fcal_lut(uint32_t* ptr_fcal_2404,uint32_t* ptr_fcal_2484);
void rfc_lo_set_channel(uint32_t chanfreq_MHz);

void rfc_fsm_force(uint8_t state);

/*
 * COEX DECLARATIONS
 ****************************************************************************************
 */
void rfc_coex_force_to(uint32_t force_enable, uint32_t bbmode);

void rfc_dump();
#endif
