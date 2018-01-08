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
 * @file timeslicer.h
 * @brief Time Slicer Module
 * 
 * This module implements functions for time slicer functionality. It allows
 * the decimation of a periodic task within another periodic task.
 *
 * @author gabriel.brunheira
 * @date 24/11/2017
 *
 */

#ifndef TIMESLICER_H_
#define TIMESLICER_H_

#include <stdint.h>

#define NUM_MAX_TIMESLICERS     6
#define RUN_TIMESLICER(id)      if(timeslicers.counter[id]++ == timeslicers.freq_ratio[id]){
#define END_TIMESLICER(id)      timeslicers.counter[id] = 1;}

typedef volatile struct
{
    uint16_t  freq_ratio[NUM_MAX_TIMESLICERS];
    uint16_t  counter[NUM_MAX_TIMESLICERS];
} timeslicer_t;

extern timeslicer_t g_timeslicers;

extern void reset_timeslicers(void);
extern void cfg_timeslicer(uint16_t id, uint16_t ratio);

#endif /* TIMESLICER_H_ */
