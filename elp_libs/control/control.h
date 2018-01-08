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
 * @file control.h
 * @brief Brief description of module
 * 
 * Detailed description
 *
 * @author gabriel.brunheira
 * @date 27/11/2017
 *
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>
#include "dsp/dsp.h"
#include "pwm/pwm.h"

/* Library-wide limits */

#define NUM_MAX_INPUT_SIGNALS       12
#define NUM_MAX_NET_SIGNALS         32
#define NUM_MAX_OUTPUT_SIGNALS      16

#define NUM_MAX_DSP_ERROR           4
#define NUM_MAX_DSP_SRLIM           3
#define NUM_MAX_DSP_LPF             4
#define NUM_MAX_DSP_PI              6
#define NUM_MAX_DSP_IIR_2P2Z        6
#define NUM_MAX_DSP_IIR_3P3Z        3
#define NUM_MAX_DSP_VDCLINK_FF      2
#define NUM_MAX_DSP_VECT_PRODUCT    2


/**
 * Collection of DSP modules used by Control Framework
 */
typedef volatile struct
{
    dsp_error_t         dsp_error[NUM_MAX_DSP_ERROR];
    dsp_srlim_t         dsp_srlim[NUM_MAX_DSP_SRLIM];
    dsp_lpf_t           dsp_lpf[NUM_MAX_DSP_LPF];
    dsp_pi_t            dsp_pi[NUM_MAX_DSP_PI];
    dsp_iir_2p2z_t      dsp_iir_2p2z[NUM_MAX_DSP_IIR_2P2Z];
    dsp_iir_3p3z_t      dsp_iir_3p3z[NUM_MAX_DSP_IIR_3P3Z];
    dsp_vdclink_ff_t    dsp_ff[NUM_MAX_DSP_VDCLINK_FF];
    dsp_vect_product_t  dsp_vect_product[NUM_MAX_DSP_VECT_PRODUCT];
} dsp_modules_t;


/**
 * Control Framework entity. This struct groups information regarding a
 * particular Control Framework implementation, including:
 *
 *      - Set of net signals for internal DSP modules interconnection
 *      - Set of output signals for duty cycles, for example.
 *      - Set of DSP modules
 */
typedef volatile struct
{
    float           net_signals[NUM_MAX_NET_SIGNALS];
    float           output_signals[NUM_MAX_OUTPUT_SIGNALS];
    dsp_modules_t   dsp_modules;
} control_framework_t;


extern volatile control_framework_t g_controller_ctom;
extern volatile control_framework_t g_controller_mtoc;

extern void init_control_framework(volatile control_framework_t *p_controller);

#endif /* CONTROL_H_ */
