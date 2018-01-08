/******************************************************************************
 * Copyright (C) 2017 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file pwm.h
 * @brief PWM module
 * 
 * This module is responsible for configuration and operation of the 16 PWM
 * channels from DRS-UDC v2.1 board. It provides a higher level of abstraction
 * for the ePWM driver provided by controlSUITE.
 *
 * @author gabriel.brunheira
 * @date 24/11/2017
 *
 */

#ifndef PWM_H_
#define PWM_H_

#include "boards/udc_c28.h"
#include "SFO_V7.h"

#define PWM_ENABLED     1
#define PWM_DISABLED    0

#define NUM_MAX_PWM_MODULES   8

typedef enum {
        PWM_Sync_Master,
        PWM_Sync_Slave
} pwm_sync_t;

typedef enum {
        PWM_ChB_Independent,
        PWM_ChB_Complementary
} cfg_pwm_channel_b_t;

typedef volatile struct
{
    uint16_t    num_modules;
    uint16_t    pwm_state[NUM_MAX_PWM_MODULES];
    volatile struct EPWM_REGS *pwm_regs[NUM_MAX_PWM_MODULES];
} pwm_modules_t;

/**
 * The following three declarations are required in order to use the SFO library
 * functions. For this reason the naming convetion for global variables was
 * ignored.
 */
extern int16_t  MEP_ScaleFactor;
extern uint16_t SFO_status;
extern volatile struct EPWM_REGS *ePWM[9];

extern pwm_modules_t g_pwm_modules;

extern uint16_t set_pwm_freq(volatile struct EPWM_REGS *p_pwm_module,
                             double freq);
extern void set_pwm_deadtime(volatile struct EPWM_REGS *p_pwm_module,
                             uint16_t deadtime);
extern void set_pwm_sync_phase(volatile struct EPWM_REGS *p_pwm_module,
                               uint16_t degrees);
extern void cfg_pwm_sync(volatile struct EPWM_REGS *p_pwm_module,
                         pwm_sync_t sync_mode, uint16_t phase_degrees);
extern void cfg_pwm_channel_b(volatile struct EPWM_REGS *p_pwm_module,
                              cfg_pwm_channel_b_t cfg_channel_b);
extern void set_pwm_primary_module(volatile struct EPWM_REGS *p_pwm_module,
                            uint16_t primary_module);

extern void set_pwm_duty_chA(volatile struct EPWM_REGS *p_pwm_module,
                             float duty_pu);
extern void set_pwm_duty_chB(volatile struct EPWM_REGS *p_pwm_module,
                             float duty_pu);
extern void set_pwm_duty_hbridge(volatile struct EPWM_REGS *p_pwm_module,
                                 float duty_pu);

extern void init_pwm_module(volatile struct EPWM_REGS *p_pwm_module,
                            double freq, uint16_t primary_module,
                            pwm_sync_t sync_mode, uint16_t phase_degrees,
                            cfg_pwm_channel_b_t cfg_channel_b, uint16_t deadtime);

extern void enable_pwm_output(uint16_t pwm_module);
extern void disable_pwm_output(uint16_t pwm_module);
extern void enable_pwm_outputs(void);
extern void disable_pwm_outputs(void);

extern void enable_pwm_tbclk(void);
extern void disable_pwm_tbclk(void);

extern void enable_pwm_interrupt(volatile struct EPWM_REGS *p_pwm_module);
extern void disable_pwm_interrupt(volatile struct EPWM_REGS *p_pwm_module);

extern void init_pwm_mep_sfo(void);
extern void tune_pwm_mep_sfo(void);

#endif /* PWM_H_ */
