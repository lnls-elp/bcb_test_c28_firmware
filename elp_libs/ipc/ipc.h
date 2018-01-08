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
 * @file ipc.h
 * @brief Interprocessor Communication module
 *
 * This module is responsible for definition of interprocessor communication
 * functionalities, between ARM and C28 cores.
 * 
 * @author gabriel.brunheira
 * @date 22/11/2017
 *
 */

#ifndef IPC_H_
#define IPC_H_

#include <stdint.h>
#include "common/structs.h"
#include "ps_modules/ps_modules.h"
#include "siggen/siggen.h"
#include "wfmref/wfmref.h"

/**
 * Shared resources defines
 */

#define SIZE_BUF_SAMPLES    4096

/**
 * IPC Message Defines
 */

#define IPC_MTOC_LOWPRIORITY_MSG    0x00000001  // IPC1
#define IPC_CTOM_LOWPRIORITY_MSG    0x00000001  // IPC1
#define SYNC_PULSE                  0x00000002  // IPC2
#define HARD_INTERLOCK              0x00000004  // IPC3
#define SOFT_INTERLOCK              0x00000008  // IPC4

typedef enum
{
    Turn_On = 1,
    Turn_Off,
    Open_Loop,
    Close_Loop,
    Operating_Mode,
    Reset_Interlocks,
    Unlock_UDC,
    Lock_UDC,
    Cfg_Buf_Samples,
    Enable_Buf_Samples,
    Disable_Buf_Samples,
    Set_SlowRef,
    Set_SlowRef_All_PS,
    Cfg_WfmRef,
    Select_WfmRef,
    Reset_WfmRef,
    Cfg_SigGen,
    Scale_SigGen,
    Enable_SigGen,
    Disable_SigGen,
    CtoM_Message_Error
} ipc_mtoc_lowpriority_msg_t;

typedef enum
{   Enable_HRADC_Boards,
    Disable_HRADC_Boards,
    MtoC_Message_Error
} ipc_ctom_lowpriority_msg_t;

#define GET_IPC_MTOC_LOWPRIORITY_MSG  (ipc_mtoc_lowpriority_msg_t) (g_ipc_ctom.msg_mtoc >> 4 ) & 0x0000FFFF

typedef enum
{
    No_Error_CtoM,
    Error1,
    Error2,
    Error3,
    Error4
} error_ctom_t;

typedef enum
{
    No_Error_MtoC,
    Invalid_Argument,
    Invalid_OpMode,
    IPC_LowPriority_Full,
    HRADC_Config_Error
} error_mtoc_t;

/**
 * IPC structures definitions
 */
typedef volatile struct
{
    uint32_t        msg_mtoc;
    uint16_t        msg_id;
    error_mtoc_t    error_mtoc;
    ps_module_t     ps_module[NUM_MAX_PS_MODULES];
    siggen_t        siggen[NUM_MAX_PS_MODULES];
    wfmref_t        wfmref[NUM_MAX_PS_MODULES];
    buf_t           buf_samples[NUM_MAX_PS_MODULES];
} ipc_ctom_t;

typedef volatile struct
{
    uint32_t        msg_ctom;
    uint16_t        msg_id;
    error_ctom_t    error_ctom;
    ps_module_t     ps_module[NUM_MAX_PS_MODULES];
    siggen_t        siggen[NUM_MAX_PS_MODULES];
    wfmref_t        wfmref[NUM_MAX_PS_MODULES];
    buf_t           buf_samples[NUM_MAX_PS_MODULES];
} ipc_mtoc_t;

extern ipc_ctom_t g_ipc_ctom;
extern ipc_mtoc_t g_ipc_mtoc;

extern void init_ipc(void);
extern void send_ipc_msg(uint16_t msg_id, uint32_t msg);
extern void send_ipc_lowpriority_msg(uint16_t msg_id,
                                     ipc_ctom_lowpriority_msg_t msg);

#endif /* IPC_H_ */
