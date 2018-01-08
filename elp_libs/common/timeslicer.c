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
 * @file timeslicer.c
 * @brief Time Slicer Module
 * 
 * This module implements functions for time slicer functionality. It allows
 * the decimation of a periodic task within another periodic task.
 *
 * @author gabriel.brunheira
 * @date 24/11/2017
 *
 */


/**
 * TODO: Put here your includes
 */

#include "timeslicer.h"

timeslicer_t g_timeslicers;

void reset_timeslicers(void)
{
    uint16_t i;

    for(i = 0; i < NUM_MAX_TIMESLICERS; i++)
    {
        g_timeslicers.counter[i] = g_timeslicers.freq_ratio[i];
    }
}

void cfg_timeslicer(uint16_t id, uint16_t ratio)
{
    g_timeslicers.freq_ratio[id]  = ratio;
    g_timeslicers.counter[id]     = ratio;
}

