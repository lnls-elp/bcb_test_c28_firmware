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
 * @file siggen.h
 * @brief Signal generator module
 *
 * This module implements a real-time parametric digital signal generator. It
 * supports some broadly used signals, like sinusoidals, trapezoids, squares,
 * triangular, etc.
 *
 * @author gabriel.brunheira
 * @date 25/10/2017
 *
 */


#ifndef SIGGEN_H_
#define SIGGEN_H_

#include <stdint.h>

#define NUM_SIGGEN_AUX_PARAM    4
#define NUM_SIGGEN_AUX_VAR      4

#define PARAM_DEFAULT_SINE      {}

typedef enum
{
    Sine,
    DampedSine,
    Trapezoidal,
    PRBS
} siggen_type_t;

typedef volatile struct siggen_t siggen_t;

struct siggen_t
{
    uint16_t 		enable;
	siggen_type_t	type;
	uint16_t		num_cycles;
	float           freq;
    float           amplitude;
    float           offset;
    float           n;
    float           num_samples;
	float			aux_param[NUM_SIGGEN_AUX_PARAM];
	float           aux_var[NUM_SIGGEN_AUX_VAR];
	float           freq_sampling;
	volatile float 	*out;
	void			(*run_siggen)(siggen_t *p_siggen);
};

extern void init_siggen(siggen_t *p_siggen, float freq_sampling,
                        volatile float *out);

extern uint16_t cfg_siggen(siggen_t *p_siggen, siggen_type_t sig_type,
                     uint16_t num_cycles, float freq, float *aux_param);

extern void scale_siggen(siggen_t *p_siggen, float amplitude, float offset);

extern void enable_siggen(siggen_t *p_siggen);
extern void disable_siggen(siggen_t *p_siggen);
extern void reset_siggen(siggen_t *p_siggen);
extern void update_siggen(siggen_t *p_siggen);

extern void run_siggen_sine(siggen_t *p_siggen);
extern void run_siggen_square(siggen_t *p_siggen);
extern void run_siggen_triangle(siggen_t *p_siggen);
extern void run_siggen_freqSweep(siggen_t *p_siggen);
extern void run_siggen_dampedSine(siggen_t *p_siggen);
extern void run_siggen_trapezoidal(siggen_t *p_siggen);

#endif
