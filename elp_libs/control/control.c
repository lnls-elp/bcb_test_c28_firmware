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
 * @file control.c
 * @brief Brief description of module
 * 
 * Detailed description
 *
 * @author gabriel.brunheira
 * @date 27/11/2017
 *
 * TODO: insert comments
 */

#include "control.h"

#pragma DATA_SECTION(g_controller_ctom,"SHARERAMS1_0");
#pragma DATA_SECTION(g_controller_mtoc,"SHARERAMS0");
volatile control_framework_t g_controller_ctom;
volatile control_framework_t g_controller_mtoc;

void init_control_framework(volatile control_framework_t *p_controller)
{
    uint16_t i;

    for(i = 0; i < NUM_MAX_NET_SIGNALS; i++)
    {
        p_controller->net_signals[i] = 0.0;
    }

    for(i = 0; i < NUM_MAX_OUTPUT_SIGNALS; i++)
    {
        p_controller->output_signals[i] = 0.0;
    }
}
