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
#include "phy_tcal.h"
#include "phy_hal.h"
#include "phy_trpc.h"
#include "ke_timer.h"
#include "mm_timer.h"
#include "reg_riu.h"
#include "bl602_rf_private.h"
#include "phy_bl602.h"


#define RFCAL_TEMPERATURE   (25)
#define TXPWR_SLOPE_VAL     (-5)
#define TXPWR_SLOPE_FWL     (8)
#define TXPWR_SCALE         (10)
#define RXGAIN_SLOPE_VAL    (-6)
#define RXGAIN_SLOPE_FWL    (8)

#define TCAL_TIMER_MM       (0)  // using MM timer or external timer

#define TCAL_ABS(x)         ((x) >= 0 ? (x) : -(x))
#define TCAL_RSHIFT(x,r)    ((x) >= 0 ? (TCAL_ABS(x) >> (r)) : -(TCAL_ABS(x) >> (r)))

enum {
    TCAL_TXGAIN = 0,
    TCAL_TXIQ,
    TCAL_RXGAIN,
    TCAL_RXIQ,
    TCAL_NUM_CALS
};

struct tcal_tag
{
    struct mm_timer_tag timer;
    bool tmr_on;
    int16_t prev_temperature;
    #if 0
    int16_t temperature;
    uint32_t temperature_update_time;
    #endif
    uint32_t last_action_time[TCAL_NUM_CALS];
    uint32_t last_action_temperature[TCAL_NUM_CALS];
    int32_t last_action_out[TCAL_NUM_CALS];
    bool inited;
};

#if CFG_RF_TCAL_SIM
struct tcal_sim_tag tcal_sim;
#endif
static struct tcal_tag tcal_env;
static void phy_tcal_rxgain(int16_t curr_temperature);
static void phy_tcal_txpwr(int16_t curr_temperature);
static void phy_tcal_txiq(int16_t curr_temperature);
static void phy_tcal_rxiq(int16_t curr_temperature);
void phy_tcal_handle();

void phy_tcal_init()
{
    phy_tcal_reset();
    phy_tcal_start();
}

void phy_tcal_reset()
{
#if TCAL_TIMER_MM
    if (tcal_env.tmr_on) {
        mm_timer_clear(&tcal_env.timer);
    }
#endif

    memset(&tcal_env, 0, sizeof(struct tcal_tag));
    tcal_env.tmr_on = false;
    tcal_env.timer.cb = phy_tcal_handle;
    tcal_env.timer.env = &tcal_env;
    tcal_env.inited = false;
}

void phy_tcal_start()
{
#if TCAL_TIMER_MM
    if (!tcal_env.tmr_on) {
        mm_timer_set(&tcal_env.timer, ke_time() + PHY_TCAL_INTERVAL_US);
        tcal_env.tmr_on = true;
    }
#endif
    tcal_env.inited = true;
}

void phy_tcal_stop()
{
#if TCAL_TIMER_MM
    if (tcal_env.tmr_on) {
        mm_timer_clear(&tcal_env.timer);
        tcal_env.tmr_on = false;
    }
#endif
}

void phy_tcal_handle()
{
    bool status;
    int16_t curr_temperature;
    int8_t i;

#if TCAL_TIMER_MM
    if (!tcal_env.tmr_on)
        return;
#endif
    if (!tcal_env.inited)
        return;
        
    // Get current temperature (could be conflicted with the other tasks using GPADC)
    status = hal_get_temperature(&curr_temperature);
    if (status == false)
    {
        return;
    }

    if (!tcal_env.inited)
    {
        for (i = 0; i < TCAL_NUM_CALS; i++) 
        {
            tcal_env.last_action_temperature[i] = curr_temperature;
        }
        tcal_env.prev_temperature = curr_temperature;
        tcal_env.inited = true;
        return;        
    }

    // Check if the temperature is reasonable
    curr_temperature = (curr_temperature > 125) ? 125 : curr_temperature;
    curr_temperature = (curr_temperature < -40) ? -40 : curr_temperature;

    #if CFG_RF_TCAL_SIM == 0
    if ((curr_temperature - tcal_env.prev_temperature) > +4  || 
        (curr_temperature - tcal_env.prev_temperature) < -4) 
    {
        tcal_env.prev_temperature = curr_temperature;
        return;
    }
    #endif

    phy_tcal_rxgain(curr_temperature);
    phy_tcal_txpwr(curr_temperature);
    phy_tcal_txiq(curr_temperature);
    phy_tcal_rxiq(curr_temperature);

    
    tcal_env.prev_temperature = curr_temperature;

    #if 0
    uint32_t curr_time;
    curr_time = ke_time();
    tcal_env.temperature = curr_temperature;
    tcal_env.temperature_update_time = curr_time;
    #endif

#if TCAL_TIMER_MM
    // Restart temperature calibration timer
    mm_timer_set(&tcal_env.timer, ke_time() + PHY_TCAL_INTERVAL_US);
#endif
}

void phy_tcal_callback(int16_t temperature)
{
#if CFG_RF_TCAL
    hal_set_temperature(temperature);
    phy_tcal_handle();
#endif
}

static void phy_tcal_rxgain(int16_t curr_temperature)
{
    int16_t delta_temp;
    int16_t delta_gain;

    delta_temp = curr_temperature - RFCAL_TEMPERATURE;
    delta_gain = TCAL_RSHIFT(delta_temp * RXGAIN_SLOPE_VAL, RXGAIN_SLOPE_FWL);
    #if CFG_RF_TCAL_SIM == 0
    if (delta_gain != tcal_env.last_action_out[TCAL_RXGAIN])
    #endif
    {        
        GLOBAL_INT_DISABLE();
        phy_config_rxgain(delta_gain);
        GLOBAL_INT_RESTORE();


        tcal_env.last_action_temperature[TCAL_RXGAIN] = curr_temperature;
        tcal_env.last_action_out[TCAL_RXGAIN] = delta_gain;
        #if CFG_RF_TCAL_SIM
        tcal_sim.rxgain_vpeak_dbv = delta_gain;
        tcal_sim.rxgain_temperature = curr_temperature;
        #endif
    }
}

//  * keep rf gain unchanged at high temperature region
//  * reduce rf gain at low temperature region
static void phy_tcal_txpwr(int16_t curr_temperature)
{
    int16_t delta_temp;
    int16_t delta_pwr;

    delta_temp = curr_temperature - RFCAL_TEMPERATURE;
    delta_pwr = TCAL_RSHIFT(delta_temp * TXPWR_SLOPE_VAL * TXPWR_SCALE, TXPWR_SLOPE_FWL);
    
    // decrease txgain when temperature is low
    // donot increase txgain when temperature is high (~nonlinearity)
    #if CFG_RF_TCAL_SIM == 0
    if (delta_pwr > 0) 
    #endif
    {   
        rf_pri_update_txgain_tempos(delta_pwr);
        trpc_update_vs_temperature(curr_temperature);
        tcal_env.last_action_temperature[TCAL_TXGAIN] = curr_temperature;
        tcal_env.last_action_out[TCAL_TXGAIN] = delta_pwr;

        #if CFG_RF_TCAL_SIM
        tcal_sim.txpwr_offset_db = delta_pwr;
        tcal_sim.txpwr_temperature = curr_temperature;
        #endif
    }
}

// TODO:
static void phy_tcal_txiq(int16_t curr_temperature)
{

}

// TODO: Eliminate rxiq by re-trigger runtime iq estimation module
static void phy_tcal_rxiq(int16_t curr_temperature)
{

}
