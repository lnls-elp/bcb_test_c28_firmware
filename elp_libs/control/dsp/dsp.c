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
 * @file dsp.c
 * @brief Digital Signal Processing Module
 *
 * This module implements digital signal processing funcionalities, including
 * digital filters, PI controller, and other useful blocks for digital control
 * implementations. It replaces legacy ELP_DCL module.
 * 
 * Ref.: Figoli, David; "Implementing a Digital Power Supply with TMS320C28x
 * Digital Signal Controllers.pdf", Texas Instruments, 2005
 *
 * @author gabriel
 * @date 27/11/2017
 *
 * TODO: insert comments
 *
 */

#include <stdint.h>
#include <math.h>
#include "dsp.h"

#pragma CODE_SECTION(run_dsp_error, "ramfuncs");
#pragma CODE_SECTION(run_dsp_srlim, "ramfuncs");
#pragma CODE_SECTION(run_dsp_lpf, "ramfuncs");
#pragma CODE_SECTION(run_dsp_pi, "ramfuncs");
#pragma CODE_SECTION(run_dsp_iir_2p2z, "ramfuncs");
#pragma CODE_SECTION(run_dsp_iir_3p3z, "ramfuncs");
#pragma CODE_SECTION(run_dsp_vdclink_ff, "ramfuncs");
#pragma CODE_SECTION(run_dsp_vect_product, "ramfuncs");

/**
 * Initialization of error signal entity.
 *
 * @param p_error
 * @param pos
 * @param neg
 * @param error
 */
void init_dsp_error(dsp_error_t *p_error, volatile float *pos,
                    volatile float *neg, volatile float *error)
{
    p_error->pos = pos;
    p_error->neg = neg;
    p_error->error = error;
    *(p_error->error) = 0.0;
}

/**
 * Reset error signal.
 *
 * @param p_error
 */
void reset_dsp_error(dsp_error_t *p_error)
{
    *(p_error->error) = 0.0;
}

/**
 * Calculate error signal. For improved speed, use the following macro:
 * CALC_ERROR(ELP_Error).
 *
 * @param p_error
 */
void run_dsp_error(dsp_error_t *p_error)
{
    *p_error->error = (*p_error->pos - *p_error->neg);
}

/**
 * Initialization of slew-rate limiter.
 *
 * Ref.: http://www.embeddedrelated.com/showarticle/646.php
 *
 * @param p_srlim
 * @param max_slewrate          [units/sec]
 * @param freq_sampling         [Hz]
 * @param in
 * @param out
 */
void init_dsp_srlim(dsp_srlim_t *p_srlim, float max_slewrate, float freq_sampling,
                    volatile float *in, volatile float *out)
{
    p_srlim->bypass = USE_MODULE;
    p_srlim->delta_max = max_slewrate / freq_sampling;
    p_srlim->in = in;
    p_srlim->out = out;
    *(p_srlim->out) = 0.0;
}

/**
 * Bypass or not the specified slew-rate limiter.
 *
 * @param p_srlim
 * @param bypass
 */
void bypass_dsp_srlim(dsp_srlim_t *p_srlim, uint16_t bypass)
{
    p_srlim->bypass = bypass;
}

/**
 * Reset slew-rate limiter.
 *
 * @param p_srlim
 */
void reset_dsp_srlim(dsp_srlim_t *p_srlim)
{
    *(p_srlim->out) = 0.0;
}

/**
 * Run slew-rate limiter.
 *
 * @param p_srlim
 * @param bypass
 */
void run_dsp_srlim(dsp_srlim_t *p_srlim, uint16_t bypass)
{
    float delta;

    if(bypass)
    {
        *(p_srlim->out) = *(p_srlim->in);
    }
    else
    {
        delta = *(p_srlim->in) - *(p_srlim->out);
        SATURATE(delta, p_srlim->delta_max, -p_srlim->delta_max);
        *(p_srlim->out) = *(p_srlim->out) + delta;
    }
}

/**
 * Initialization of 1st-order digital low pass filter. This is a Tustin
 * discretization of the following continuous 1st-order low-pass filter:
 *
 *      H(s) = 1 / (tau*s + 1)
 *
 * @param p_lpf
 * @param freq_cut
 * @param freq_sampling
 * @param in
 * @param out
 */
void init_dsp_lpf(dsp_lpf_t *p_lpf, float freq_cut, float freq_sampling,
                  volatile float *in, volatile float *out)
{
    float wt, k, a;

    wt = (2.0 * 3.141592653589793 * freq_cut) / freq_sampling;
    k = wt / (2.0 + wt);
    a = (2 - wt)/(2 + wt);

    p_lpf->k = k;
    p_lpf->a = a;
    p_lpf->in_old = 0.0;
    p_lpf->in = in;
    p_lpf->out = out;
    *(p_lpf->out) = 0.0;
}

/**
 * Reset 1st-order digital low-pass filter.
 *
 * @param p_lpf
 */
void reset_dsp_lpf(dsp_lpf_t *p_lpf)
{
    p_lpf->in_old = 0.0;
    *(p_lpf->out) = 0.0;
}

/**
 * Run 1st-order digital low-pass-filter.
 *
 * @param p_lpf
 */
void run_dsp_lpf(dsp_lpf_t *p_lpf)
{
    float yacc;

    yacc = *(p_lpf->out) * p_lpf->a;
    yacc += p_lpf->k * (p_lpf->in_old + *(p_lpf->in));
    p_lpf->in_old = *(p_lpf->in);
    *(p_lpf->out) = yacc;
}

/**
 * Initialization of PI controller with dynamic anti-windup scheme.
 *
 * Ref.: Buso, S.; Mattavelli, P.; "Digital Control in Power Electronics"
 *
 * @param p_pi
 * @param kp
 * @param ki
 * @param freq_sampling
 * @param u_max
 * @param u_min
 * @param in
 * @param out
 */
void init_dsp_pi(dsp_pi_t *p_pi, float kp, float ki, float freq_sampling,
                 float u_max, float u_min, volatile float *in,
                 volatile float *out)
{
    p_pi->kp = kp;
    p_pi->ki = ki / freq_sampling;
    p_pi->freq_sampling = freq_sampling;
    p_pi->u_max = u_max;
    p_pi->u_min = u_min;
    p_pi->u_prop = 0.0;
    p_pi->u_int = 0.0;
    p_pi->in = in;
    p_pi->out = out;
    *(p_pi->out) = 0.0;
}

/**
 * Reset PI controller.
 *
 * @param p_pi
 */
void reset_dsp_pi(dsp_pi_t *p_pi)
{
    p_pi->u_prop = 0.0;
    p_pi->u_int = 0.0;
    *(p_pi->out) = 0.0;
}

/**
 * Run PI controller.
 *
 * @param p_pi
 */
void run_dsp_pi(dsp_pi_t *p_pi)
{
    float dyn_max;
    float dyn_min;
    float temp;

    temp = *(p_pi->in) * p_pi->kp;
    SATURATE(temp, p_pi->u_max, p_pi->u_min);
    p_pi->u_prop = temp;

    dyn_max = (p_pi->u_max - temp);
    dyn_min = (p_pi->u_min - temp);

    temp = p_pi->u_int + *(p_pi->in) * p_pi->ki;
    SATURATE(temp, dyn_max, dyn_min);
    p_pi->u_int = temp;

    *(p_pi->out) = p_pi->u_int + p_pi->u_prop;
}

/**
 * Initialization of 2nd-order digital IIR filter. Implemented with Transposed
 * Direct-Form II.
 *
 * @param p_iir
 * @param b0
 * @param b1
 * @param b2
 * @param a1
 * @param a2
 * @param u_max
 * @param u_min
 * @param in
 * @param out
 */
void init_dsp_iir_2p2z(dsp_iir_2p2z_t *p_iir, float b0, float b1, float b2,
                       float a1, float a2, float u_max, float u_min,
                       volatile float *in, volatile float *out)
{
    p_iir->u_max = u_max;
    p_iir->u_min = u_min;
    p_iir->b0 = b0;
    p_iir->b1 = b1;
    p_iir->b2 = b2;
    p_iir->a1 = a1;
    p_iir->a2 = a2;
    p_iir->w1 = 0.0;
    p_iir->w2 = 0.0;
    p_iir->in = in;
    p_iir->out = out;
    *(p_iir->out) = 0.0;
}

/**
 * Initialization of 2nd-order digital IIR filter as a notch-filter.
 *
 * Ref.: Welch, T. B.; Wright, C. H. G.; Morrow, M. G.; "Real-Time Digital
 * Signal Processing from MATLAB to C with the TMS320C6x DSPs", 2nd Edition
 *
 * @param p_iir
 * @param alpha
 * @param freq_cut
 * @param freq_sampling
 * @param u_max
 * @param u_min
 * @param in
 * @param out
 */
void init_dsp_notch_2p2z(dsp_iir_2p2z_t *p_iir, float alpha, float freq_cut,
                         float freq_sampling, float u_max, float u_min,
                         volatile float *in, volatile float *out)
{
    float beta = cos(2.0 * 3.141592653589793 * (freq_cut/freq_sampling));

    SATURATE(alpha, 0.99999, 0.0);

    p_iir->u_max = u_max;
    p_iir->u_min = u_min;
    p_iir->b0 = (1.0 + alpha)/2.0;
    p_iir->b1 = -beta*(1.0 + alpha);
    p_iir->b2 = p_iir->b0;
    p_iir->a1 = p_iir->b1;
    p_iir->a2 = alpha;
    p_iir->w1 = 0.0;
    p_iir->w2 = 0.0;
    p_iir->in = in;
    p_iir->out = out;
    *(p_iir->out) = 0.0;
}

/**
 * Reset 2nd-order digital IIR filter.
 *
 * @param p_iir
 */
void reset_dsp_iir_2p2z(dsp_iir_2p2z_t *p_iir)
{
    p_iir->w1 = 0.0;
    p_iir->w2 = 0.0;
    *(p_iir->out) = 0.0;
}

/**
 * Run 2nd-order digital IIR filter.
 *
 * @param p_iir
 */
void run_dsp_iir_2p2z(dsp_iir_2p2z_t *p_iir)
{
    float w0, yacc;

    yacc = *(p_iir->in) * p_iir->b0;
    yacc += p_iir->w1;

    SATURATE(yacc, p_iir->u_max, p_iir->u_min);

    w0 = *(p_iir->in) * p_iir->b1;
    w0 += p_iir->w2;
    w0 -= yacc * p_iir->a1;
    p_iir->w1 = w0;

    w0 = *(p_iir->in) * p_iir->b2;
    w0 -= yacc * p_iir->a2;
    p_iir->w2 = w0;

    *(p_iir->out) = yacc;
}

/**
 * Initialization of 3rd-order digital IIR filter. Implemented with Transposed
 * Direct-Form II.
 *
 * @param p_iir
 * @param b0
 * @param b1
 * @param b2
 * @param b3
 * @param a1
 * @param a2
 * @param a3
 * @param u_max
 * @param u_min
 * @param in
 * @param out
 */
void init_dsp_iir_3p3z(dsp_iir_3p3z_t *p_iir, float b0, float b1, float b2,
                       float b3, float a1, float a2, float a3, float u_max,
                       float u_min, volatile float *in, volatile float *out)
{
    p_iir->u_max = u_max;
    p_iir->u_min = u_min;
    p_iir->b0 = b0;
    p_iir->b1 = b1;
    p_iir->b2 = b2;
    p_iir->b3 = b3;
    p_iir->a1 = a1;
    p_iir->a2 = a2;
    p_iir->a3 = a3;
    p_iir->w1 = 0.0;
    p_iir->w2 = 0.0;
    p_iir->w3 = 0.0;
    p_iir->in = in;
    p_iir->out = out;
    *(p_iir->out) = 0.0;
}

/**
 * Reset 3rd-order digital IIR filter.
 *
 * @param p_iir
 */
void reset_dsp_iir_3p3z(dsp_iir_3p3z_t *p_iir)
{
    p_iir->w1 = 0.0;
    p_iir->w2 = 0.0;
    p_iir->w3 = 0.0;
    *(p_iir->out) = 0.0;
}

/**
 * Run 3rd-order digital IIR filter.
 *
 * @param p_iir
 */
void run_dsp_iir_3p3z(dsp_iir_3p3z_t *p_iir)
{
    float w0, yacc;

    yacc = *(p_iir->in) * p_iir->b0;
    yacc += p_iir->w1;

    SATURATE(yacc, p_iir->u_max, p_iir->u_min);

    w0 = *(p_iir->in) * p_iir->b1;
    w0 += p_iir->w2;
    w0 -= yacc * p_iir->a1;
    p_iir->w1 = w0;

    w0 = *(p_iir->in) * p_iir->b2;
    w0 += p_iir->w3;
    w0 -= yacc * p_iir->a2;
    p_iir->w2 = w0;

    w0 = *(p_iir->in) * p_iir->b3;
    w0 -= yacc * p_iir->a3;
    p_iir->w3 = w0;

    *(p_iir->out) = yacc;
}

/**
 *
 * @param p_ff
 * @param vdc_nom
 * @param vdc_min
 * @param vdc_meas
 * @param in
 * @param out
 */
void init_dsp_vdclink_ff(dsp_vdclink_ff_t *p_ff, float vdc_nom, float vdc_min,
                         volatile float *vdc_meas, volatile float *in,
                         volatile float *out)
{
    p_ff->vdc_nom = vdc_nom;
    p_ff->vdc_min = vdc_min;
    p_ff->vdc_meas = vdc_meas;
    p_ff->in = in;
    p_ff->out = out;
}

/**
 *
 * @param p_ff
 */
void reset_dsp_vdclink_ff(dsp_vdclink_ff_t *p_ff)
{
    *(p_ff->out) = *(p_ff->in);
}

/**
 *
 * @param p_ff
 */
void run_dsp_vdclink_ff(dsp_vdclink_ff_t *p_ff)
{
    if( *(p_ff->vdc_meas) < p_ff->vdc_min )
    {
        *(p_ff->out) = *(p_ff->in);
    }
    else
    {
        *(p_ff->out) = *(p_ff->in) * p_ff->vdc_nom / *(p_ff->vdc_meas);
    }
}

/**
 *
 * @param p_vect_product
 * @param num_rows
 * @param num_columns
 * @param matrix
 * @param in
 * @param out
 */
void init_dsp_vect_product(dsp_vect_product_t *p_vect_product, uint16_t num_rows,
                           uint16_t num_cols,
                           volatile float matrix[num_rows][num_cols],
                           volatile float *in, volatile float *out)
{
    uint16_t r,c;

    p_vect_product->matrix.num_rows = num_rows;
    p_vect_product->matrix.num_cols = num_cols;
    p_vect_product->in = in;
    p_vect_product->out = out;


    for(r = 0; r < p_vect_product->matrix.num_rows; r++)
        {
            for(c = 0; c < p_vect_product->matrix.num_cols; c++)
            {
                p_vect_product->matrix.data[r][c] = matrix[num_rows][num_cols];

            }
        }
}

/**
 *
 * @param p_vect_product
 */
void reset_dsp_vect_product(dsp_vect_product_t *p_vect_product)
{
    uint16_t r;

    for(r = 0; r < p_vect_product->matrix.num_rows; r++)
    {
        p_vect_product->out[r] = 0.0;
    }
}

/**
 *
 * @param p_vect_product
 */
void run_dsp_vect_product(dsp_vect_product_t *p_vect_product)
{
    uint16_t r, c;

    for(r = 0; r < p_vect_product->matrix.num_rows; r++)
    {
        p_vect_product->out[r] = 0.0;

        for(c = 0; c < p_vect_product->matrix.num_cols; c++)
        {
            p_vect_product->out[r] += p_vect_product->matrix.data[r][c]*
                                      p_vect_product->in[c];
        }
    }
}
