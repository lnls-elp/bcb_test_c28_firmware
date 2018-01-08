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
 * @file fbp.c
 * @brief FBP v4.0 module
 * 
 * Module for control of FBP v4.0 power supplies (Low-Power Power Supply).
 *
 * @author gabriel.brunheira
 * @date 23/11/2017
 *
 */

#include "fbp.h"
#include "boards/udc_c28.h"
#include "control/control.h"
#include "ipc/ipc.h"
#include "common/timeslicer.h"
#include "HRADC_board/HRADC_Boards.h"


/**
 * Configuration parameters
 *
 * TODO: transfer this to param bank
 */
#define USE_ITLK                1
#define TIMEOUT_DCLINK_RELAY    100000

#define PWM_FREQ                50000.0     // PWM frequency [Hz]
#define PWM_DEAD_TIME           300         // PWM dead-time [ns]
#define PWM_MAX_DUTY            0.9         // Max duty cycle [p.u.]
#define PWM_MIN_DUTY            -0.9        // Min duty cycle [p.u.]
#define PWM_MAX_DUTY_OL         0.9         // Max open loop duty cycle [p.u.]
#define PWM_MIN_DUTY_OL         -0.9        // Min open loop duty cycle [p.u.]

#define MAX_REF                 10.0        // Reference over-saturation level [A]
#define MIN_REF                 -10.0       // Reference under-saturation level [A]
#define MAX_ILOAD               10.5        // Reference limit for interlock [A]
#define MAX_VLOAD               10.5        // Load voltage limit for interlock [V]
#define MIN_DCLINK              3.0         // DC Link under limit for interlock [V]
#define MAX_DCLINK              17.0        // DC Link over limit for interlock [V]
#define MAX_TEMP                80.0        // Temperature limit for interlock [ºC]

#define MAX_REF_SLEWRATE        1000000.0   // Max reference slew-rate [A/s]
#define MAX_SR_SIGGEN_OFFSET    50.0        // Max SigGen offset slew-rate [A/s]
#define MAX_SR_SIGGEN_AMP       100.0       // Max SigGen amplitude slew-rate [A/s]

//#define KP                      1.9
//#define KI                      559.0

#define KP                      3.56    // <= Testes FAC BW 1 kHz   // 0.08976   // <= Jiga Bastidor  //4.071   <= CARGA Lo = Corretora; Ro = 0.5R        //0.0 <= CARGA RESISTIVA WEG           //0.0  <= CARGA RESISTIVA WEG              //1.9          //2.8
#define KI                      73.304  //

#define CONTROL_FREQ            (2.0*PWM_FREQ)
#define CONTROL_PERIOD          (1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR       1
#define TRANSFER_BUFFER_SIZE    DECIMATION_FACTOR
#define HRADC_FREQ_SAMP         (float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK           SPI_15MHz

#define BUFFER_DECIMATION       1
#define WFMREF_SAMPLING_FREQ    4096

#define TRANSDUCER_INPUT_RATED      12.5            // ** DCCT LEM ITN 12-P **
#define TRANSDUCER_OUTPUT_RATED     0.05            // In_rated   = +/- 12.5 A
#define TRANSDUCER_OUTPUT_TYPE      Iin_bipolar     // Out_rated  = +/- 50 mA
#define HRADC_R_BURDEN              20.0            // Burden resistor = 20 R
#if (HRADC_v2_0)
    #define TRANSDUCER_GAIN         -(TRANSDUCER_INPUT_RATED/TRANSDUCER_OUTPUT_RATED)
#endif
#if (HRADC_v2_1)
    #define TRANSDUCER_GAIN         (TRANSDUCER_INPUT_RATED/TRANSDUCER_OUTPUT_RATED)
#endif

/**
 * All power supplies defines
 *
 * TODO: use new control modules definitions
 */
#define PS_ALL_ID   0x000F

#define LOAD_OVERCURRENT            0x00000001
#define LOAD_OVERVOLTAGE            0x00000002
#define DCLINK_OVERVOLTAGE          0x00000004
#define DCLINK_UNDERVOLTAGE         0x00000008
#define DCLINK_RELAY_FAIL           0x00000010
#define FUSE_FAIL                   0x00000020
#define DRIVER_FAIL                 0x00000040

#define OVERTEMP                    0x00000001

/**
 * Power supply 1 defines
 */
#define PS1_ID                          0x0000

#define PIN_OPEN_PS1_DCLINK_RELAY       CLEAR_GPDO4;
#define PIN_CLOSE_PS1_DCLINK_RELAY      SET_GPDO4;

#define PIN_STATUS_PS1_DCLINK_RELAY     GET_GPDI4
#define PIN_STATUS_PS1_DRIVER_ERROR     GET_GPDI5
#define PIN_STATUS_PS1_FUSE             GET_GPDI14

#define PS1_LOAD_CURRENT                g_controller_ctom.net_signals[0]    // HRADC0
#define PS1_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[0]    // ANI2
#define PS1_LOAD_VOLTAGE                g_controller_mtoc.net_signals[4]    // ANI6
#define PS1_TEMPERATURE                 g_controller_mtoc.net_signals[8]   // I2C Add 0x48

#define PS1_SETPOINT                    g_ipc_ctom.ps_module[0].ps_setpoint
#define PS1_REFERENCE                   g_ipc_ctom.ps_module[0].ps_reference

#define ERROR_CALCULATOR_PS1            &g_controller_ctom.dsp_modules.dsp_error[0]
#define PI_DAWU_CONTROLLER_ILOAD_PS1    &g_controller_ctom.dsp_modules.dsp_pi[0]

#define PS1_PWM_MODULATOR               g_pwm_modules.pwm_regs[0]
#define PS1_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[1]

/**
 * Power supply 2 defines
 */
#define PS2_ID                          0x0001

#define PIN_OPEN_PS2_DCLINK_RELAY       CLEAR_GPDO3;
#define PIN_CLOSE_PS2_DCLINK_RELAY      SET_GPDO3;

#define PIN_STATUS_PS2_DCLINK_RELAY     GET_GPDI11
#define PIN_STATUS_PS2_DRIVER_ERROR     GET_GPDI9
#define PIN_STATUS_PS2_FUSE             GET_GPDI16

#define PS2_LOAD_CURRENT                g_controller_ctom.net_signals[1]    // HRADC1
#define PS2_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[1]    // ANI1
#define PS2_LOAD_VOLTAGE                g_controller_mtoc.net_signals[5]   // ANI7
#define PS2_TEMPERATURE                 g_controller_mtoc.net_signals[9]   // I2C Add 0x49

#define PS2_SETPOINT                    g_ipc_ctom.ps_module[1].ps_setpoint
#define PS2_REFERENCE                   g_ipc_ctom.ps_module[1].ps_reference

#define ERROR_CALCULATOR_PS2            &g_controller_ctom.dsp_modules.dsp_error[1]
#define PI_DAWU_CONTROLLER_ILOAD_PS2    &g_controller_ctom.dsp_modules.dsp_pi[1]

#define PS2_PWM_MODULATOR               g_pwm_modules.pwm_regs[2]
#define PS2_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[3]

/**
 * Power supply 3 defines
 */
#define PS3_ID                          0x0002

#define PIN_OPEN_PS3_DCLINK_RELAY       CLEAR_GPDO1;
#define PIN_CLOSE_PS3_DCLINK_RELAY      SET_GPDO1;

#define PIN_STATUS_PS3_DCLINK_RELAY     GET_GPDI8
#define PIN_STATUS_PS3_DRIVER_ERROR     GET_GPDI1
#define PIN_STATUS_PS3_FUSE             GET_GPDI13

#define PS3_LOAD_CURRENT                g_controller_ctom.net_signals[2]    // HRADC2
#define PS3_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[2]    // ANI4
#define PS3_LOAD_VOLTAGE                g_controller_mtoc.net_signals[6]   // ANI3
#define PS3_TEMPERATURE                 g_controller_mtoc.net_signals[10]   // I2C Add 0x4A

#define PS3_SETPOINT                    g_ipc_ctom.ps_module[2].ps_setpoint
#define PS3_REFERENCE                   g_ipc_ctom.ps_module[2].ps_reference

#define ERROR_CALCULATOR_PS3            &g_controller_ctom.dsp_modules.dsp_error[2]
#define PI_DAWU_CONTROLLER_ILOAD_PS3    &g_controller_ctom.dsp_modules.dsp_pi[2]

#define PS3_PWM_MODULATOR               g_pwm_modules.pwm_regs[4]
#define PS3_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[5]

/**
 * Power supply 4 defines
 */

#define PS4_ID                          0x0003

#define PIN_OPEN_PS4_DCLINK_RELAY       CLEAR_GPDO2;
#define PIN_CLOSE_PS4_DCLINK_RELAY      SET_GPDO2;

#define PIN_STATUS_PS4_DCLINK_RELAY     GET_GPDI2
#define PIN_STATUS_PS4_DRIVER_ERROR     GET_GPDI3
#define PIN_STATUS_PS4_FUSE             GET_GPDI15

#define PS4_LOAD_CURRENT                g_controller_ctom.net_signals[3]   // HRADC3
#define PS4_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[3]    // ANI0
#define PS4_LOAD_VOLTAGE                g_controller_mtoc.net_signals[7]   // ANI5
#define PS4_TEMPERATURE                 g_controller_mtoc.net_signals[11]   // I2C Add 0x4C

#define PS4_SETPOINT                    g_ipc_ctom.ps_module[3].ps_setpoint
#define PS4_REFERENCE                   g_ipc_ctom.ps_module[3].ps_reference

#define ERROR_CALCULATOR_PS4            &g_controller_ctom.dsp_modules.dsp_error[3]
#define PI_DAWU_CONTROLLER_ILOAD_PS4    &g_controller_ctom.dsp_modules.dsp_pi[3]

#define PS4_PWM_MODULATOR               g_pwm_modules.pwm_regs[6]
#define PS4_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[7]

static uint16_t num_active_ps_modules;

#pragma CODE_SECTION(isr_init_controller, "ramfuncs");
#pragma CODE_SECTION(isr_controller, "ramfuncs");
#pragma CODE_SECTION(turn_on, "ramfuncs");
#pragma CODE_SECTION(turn_off, "ramfuncs");
#pragma CODE_SECTION(set_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(set_soft_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_soft_interlock, "ramfuncs");
#pragma CODE_SECTION(open_relay, "ramfuncs");
#pragma CODE_SECTION(close_relay, "ramfuncs");
#pragma CODE_SECTION(get_relay_status, "ramfuncs");

static void init_peripherals_drivers(uint16_t num_ps);
static void term_peripherals_drivers(uint16_t num_ps);

static uint16_t init_controller(void);
static void reset_controller(uint16_t id);
static void reset_controllers(uint16_t num_ps);
static void enable_controller();
static void disable_controller();
interrupt void isr_init_controller(void);
interrupt void isr_controller(void);

static void init_interruptions(void);
static void term_interruptions(void);

static void turn_on(uint16_t id);
static void turn_off(uint16_t id);

static void reset_interlocks(void);
static void set_hard_interlock(uint16_t id, uint32_t itlk);
static void set_soft_interlock(uint16_t id, uint32_t itlk);
interrupt void isr_hard_interlock(void);
interrupt void isr_soft_interlock(void);

static void open_relay(uint16_t id);
static void close_relay(uint16_t id);
static uint16_t get_relay_status(uint16_t id);
static void check_interlocks_ps_module(uint16_t id);
/**
 * Main function for this power supply module
 */
void main_fbp(void)
{
    uint16_t i;

    num_active_ps_modules = init_controller();
    init_peripherals_drivers(num_active_ps_modules);
    init_interruptions();
    enable_controller();

    /// TODO: include condition for re-initialization
    while(1)
    {
        for(i = 0; i < NUM_MAX_PS_MODULES; i++)
        {
            if(g_ipc_ctom.ps_module[i].ps_status.bit.active)
            {
                check_interlocks_ps_module(i);
            }
        }
    }

    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        turn_off(i);
    }

    disable_controller();
    term_interruptions();
    reset_controllers(num_active_ps_modules);
    term_peripherals_drivers(num_active_ps_modules);
}

static void init_peripherals_drivers(uint16_t num_ps)
{
    uint16_t i;

    /* Initialization of HRADC boards */

    stop_DMA();

    HRADCs_Info.enable_Sampling = 0;

    Init_DMA_McBSP_nBuffers(num_ps, DECIMATION_FACTOR, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    DELAY_US(500000);
    send_ipc_lowpriority_msg(0,Enable_HRADC_Boards);
    DELAY_US(2000000);

    for(i = 0; i < num_ps; i++)
    {
        Init_HRADC_Info(&HRADCs_Info.HRADC_boards[i], i, DECIMATION_FACTOR,
                        buffers_HRADC[i], TRANSDUCER_GAIN);
        Config_HRADC_board(&HRADCs_Info.HRADC_boards[i], Iin_bipolar,
                           HEATER_DISABLE, RAILS_DISABLE);
    }

    HRADCs_Info.n_HRADC_boards = num_ps;

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    /* Initialization of PWM modules */
    g_pwm_modules.num_modules = 8;

    PS1_PWM_MODULATOR       = &EPwm7Regs;   // PS-1 Positive polarity switches
    PS1_PWM_MODULATOR_NEG   = &EPwm8Regs;   // PS-1 Negative polarity switches

    PS2_PWM_MODULATOR       = &EPwm5Regs;   // PS-2 Positive polarity switches
    PS2_PWM_MODULATOR_NEG   = &EPwm6Regs;   // PS-2 Negative polarity switches

    PS3_PWM_MODULATOR       = &EPwm3Regs;   // PS-3 Positive polarity switches
    PS3_PWM_MODULATOR_NEG   = &EPwm4Regs;   // PS-3 Negative polarity switches

    PS4_PWM_MODULATOR       = &EPwm1Regs;   // PS-4 Positive polarity switches
    PS4_PWM_MODULATOR_NEG   = &EPwm2Regs;   // PS-4 Negative polarity switches

    disable_pwm_outputs();
    disable_pwm_tbclk();
    init_pwm_mep_sfo();

    // PS-4 PWM initialization
    init_pwm_module(PS4_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS4_PWM_MODULATOR_NEG, PWM_FREQ, 1, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    // PS-3 PWM initialization
    init_pwm_module(PS3_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS3_PWM_MODULATOR_NEG, PWM_FREQ, 3, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    // PS-2 PWM initialization
    init_pwm_module(PS2_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS2_PWM_MODULATOR_NEG, PWM_FREQ, 5, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    // PS-1 PWM initialization
    init_pwm_module(PS1_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS1_PWM_MODULATOR_NEG, PWM_FREQ, 7, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();

    /* Initialization of timers */
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, 1000000);
    CpuTimer0Regs.TCR.bit.TIE = 0;
}

static void term_peripherals_drivers(uint16_t num_ps)
{

}

static uint16_t init_controller(void)
{
    uint16_t i, num_ps;

    num_ps = 0;

    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        if(g_ipc_mtoc.ps_module[i].ps_status.bit.active)
        {
            init_ps_module(&g_ipc_ctom.ps_module[i],
                           g_ipc_mtoc.ps_module[i].ps_status.bit.model,
                           &turn_on, &turn_off, &isr_soft_interlock,
                           &isr_hard_interlock, &reset_interlocks);

            /**
             * TODO: initialize SigGen, WfmRef and Samples Buffer
             */

            num_ps = i;
        }
    }

    num_ps++;

    init_ipc();
    init_control_framework(&g_controller_ctom);

    /******************************************************************/
    /* INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 1 */
    /******************************************************************/

    /*
     *        name:     ERROR_CALCULATOR_PS1
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[0].ps_reference
     *           -:     net_signals[0]
     *         out:     net_signals[4]
     */

    init_dsp_error(ERROR_CALCULATOR_PS1, &PS1_REFERENCE,
                   &g_controller_ctom.net_signals[0],
                   &g_controller_ctom.net_signals[4]);

    /*
     *        name:     PI_DAWU_CONTROLLER_ILOAD_PS1
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[4]
     *         out:     output_signals[0]
     */

    init_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS1, KP, KI, CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[4],
                &g_controller_ctom.output_signals[0]);

    /******************************************************************/
    /* INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 2 */
    /******************************************************************/

    /*
     *        name:     ERROR_CALCULATOR_PS2
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[1].ps_reference
     *           -:     net_signals[1]
     *         out:     net_signals[5]
     */

    init_dsp_error(ERROR_CALCULATOR_PS2, &PS2_REFERENCE,
                   &g_controller_ctom.net_signals[1],
                   &g_controller_ctom.net_signals[5]);

    /*
     *        name:     PI_DAWU_CONTROLLER_ILOAD_PS2
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[5]
     *         out:     output_signals[1]
     */

    init_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS2, KP, KI, CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[5],
                &g_controller_ctom.output_signals[1]);

    /******************************************************************/
    /* INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 3 */
    /******************************************************************/

    /*
     *        name:     ERROR_CALCULATOR_PS3
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[2].ps_reference
     *           -:     net_signals[2]
     *         out:     net_signals[6]
     */

    init_dsp_error(ERROR_CALCULATOR_PS3, &PS3_REFERENCE,
                   &g_controller_ctom.net_signals[2],
                   &g_controller_ctom.net_signals[6]);

    /*
     *        name:     PI_DAWU_CONTROLLER_ILOAD_PS3
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[6]
     *         out:     output_signals[2]
     */

    init_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS3, KP, KI, CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[6],
                &g_controller_ctom.output_signals[2]);

    /******************************************************************/
    /* INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 4 */
    /******************************************************************/

    /*
     *        name:     ERROR_CALCULATOR_PS4
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[3].ps_reference
     *           -:     net_signals[3]
     *         out:     net_signals[7]
     */

    init_dsp_error(ERROR_CALCULATOR_PS4, &PS4_REFERENCE,
                   &g_controller_ctom.net_signals[3],
                   &g_controller_ctom.net_signals[7]);

    /*
     *        name:     PI_DAWU_CONTROLLER_ILOAD_PS4
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[7]
     *         out:     output_signals[3]
     */
    init_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS4, KP, KI, CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[7],
                &g_controller_ctom.output_signals[3]);

    /**********************************/
    /* INITIALIZATION OF TIME SLICERS */
    /**********************************/

    // 0: Time-slicer for WfmRef sweep decimation
    cfg_timeslicer(0, CONTROL_FREQ/WFMREF_SAMPLING_FREQ);

    // 1: Time-slicer for SamplesBuffer
    cfg_timeslicer(1, BUFFER_DECIMATION);

    reset_controllers(num_ps);

    return num_ps;
}

static void reset_controller(uint16_t id)
{

    /*switch(id)
    {
        case 0:
        {
            set_pwm_duty_hbridge(PS1_PWM_MODULATOR, 0.0);
            reset_dsp_error(ERROR_CALCULATOR_PS1);
            reset_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS1);
            PS1_REFERENCE = 0.0;
            break;
        }

        case 1:
        {
            set_pwm_duty_hbridge(PS2_PWM_MODULATOR, 0.0);
            reset_dsp_error(ERROR_CALCULATOR_PS2);
            reset_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS2);
            PS2_REFERENCE = 0.0;
            break;
        }

        case 2:
        {
            set_pwm_duty_hbridge(PS3_PWM_MODULATOR, 0.0);
            reset_dsp_error(ERROR_CALCULATOR_PS3);
            reset_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS3);
            PS3_REFERENCE = 0.0;
            break;
        }

        case 3:
        {
            set_pwm_duty_hbridge(PS4_PWM_MODULATOR, 0.0);
            reset_dsp_error(ERROR_CALCULATOR_PS4);
            reset_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS4);
            PS4_REFERENCE = 0.0;
            break;
        }

        default:
        {
            break;
        }
    }*/

    set_pwm_duty_hbridge(g_pwm_modules.pwm_regs[id*2], 0.0);
    reset_dsp_error(&g_controller_ctom.dsp_modules.dsp_error[id]);
    reset_dsp_pi(&g_controller_ctom.dsp_modules.dsp_pi[id]);
    g_ipc_ctom.ps_module[id].ps_reference = 0.0;
    reset_timeslicers();
}

static void reset_controllers(uint16_t num_ps)
{
    uint16_t i;

    for(i = 0; i < num_ps; i++)
    {
        reset_controller(i);
    }
}

static void enable_controller()
{
    stop_DMA();
    DELAY_US(5);
    start_DMA();
    HRADCs_Info.enable_Sampling = 1;
    enable_pwm_tbclk();
}

static void disable_controller()
{
}

interrupt void isr_init_controller(void)
{
    uint16_t i;

    EALLOW;
    PieVectTable.EPWM1_INT = &isr_controller;
    EDIS;

    for(i = 0; i < g_pwm_modules.num_modules; i++)
    {
        g_pwm_modules.pwm_regs[i]->ETSEL.bit.INTSEL = ET_CTR_ZERO;
        g_pwm_modules.pwm_regs[i]->ETCLR.bit.INT = 1;
    }

    PieCtrlRegs.PIEACK.all |= M_INT3;
}

interrupt void isr_controller(void)
{
    static uint16_t i;
    static float temp[4];

    //SET_DEBUG_GPIO1;

    temp[0] = (float) *(HRADCs_Info.HRADC_boards[0].SamplesBuffer);
    temp[1] = (float) *(HRADCs_Info.HRADC_boards[1].SamplesBuffer);
    temp[2] = (float) *(HRADCs_Info.HRADC_boards[2].SamplesBuffer);
    temp[3] = (float) *(HRADCs_Info.HRADC_boards[3].SamplesBuffer);

    #if 1
    temp[0] *= HRADCs_Info.HRADC_boards[0].gain;
    temp[0] += HRADCs_Info.HRADC_boards[0].offset;

    temp[1] *= HRADCs_Info.HRADC_boards[1].gain;
    temp[1] += HRADCs_Info.HRADC_boards[1].offset;

    temp[2] *= HRADCs_Info.HRADC_boards[2].gain;
    temp[2] += HRADCs_Info.HRADC_boards[2].offset;

    temp[3] *= HRADCs_Info.HRADC_boards[3].gain;
    temp[3] += HRADCs_Info.HRADC_boards[3].offset;

    PS1_LOAD_CURRENT = temp[0];
    PS2_LOAD_CURRENT = temp[1];
    PS3_LOAD_CURRENT = temp[2];
    PS4_LOAD_CURRENT = temp[3];
/*
    if(fabs(PS1_LOAD_CURRENT) > MAX_ILOAD)
    {
        set_hard_interlock(PS1_ID, LOAD_OVERCURRENT);
    }

    if(fabs(PS2_LOAD_CURRENT) > MAX_ILOAD)
    {
        set_hard_interlock(PS2_ID, LOAD_OVERCURRENT);
    }

    if(fabs(PS3_LOAD_CURRENT) > MAX_ILOAD)
    {
        set_hard_interlock(PS3_ID, LOAD_OVERCURRENT);
    }

    if(fabs(PS4_LOAD_CURRENT) > MAX_ILOAD)
    {
        set_hard_interlock(PS4_ID, LOAD_OVERCURRENT);
    }*/
    #endif


    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        #if 0
        //CLEAR_DEBUG_GPIO1;

        if(g_ipc_ctom.ps_module[i].ps_status.bit.active)
        {
            /**
             * TODO: test this iterative implementation
             */
            #if 0
            temp[i] *= HRADCs_Info.HRADC_boards[i].gain;
            temp[i] += HRADCs_Info.HRADC_boards[i].offset;
            g_controller_ctom.net_signals[i] = temp[i];

            if(fabs(g_controller_ctom.net_signals[i]) > MAX_ILOAD)
            {
                set_hard_interlock(i, LOAD_OVERCURRENT);
            }
            #endif

            if(g_ipc_ctom.ps_module[i].ps_status.bit.state > Interlock)
            {
        #endif
                switch(g_ipc_ctom.ps_module[i].ps_status.bit.state)
                {
                    case SlowRef:
                    case SlowRefSync:
                    {
                        g_ipc_ctom.ps_module[i].ps_reference = g_ipc_ctom.ps_module[i].ps_setpoint;
                        break;
                    }
                    case RmpWfm:
                    {
                        break;
                    }
                    case MigWfm:
                    {
                        break;
                    }
                    case Cycle:
                    {
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
                if(g_ipc_ctom.ps_module[i].ps_status.bit.openloop)
                {
                    g_controller_ctom.output_signals[i] =
                            0.01 * g_ipc_ctom.ps_module[i].ps_reference;

                    SATURATE(g_controller_ctom.output_signals[i],
                             PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
                }
                else
                {
                    SATURATE(g_ipc_ctom.ps_module[i].ps_reference, MAX_REF, MIN_REF);

                    run_dsp_error(&g_controller_ctom.dsp_modules.dsp_error[i]);
                    run_dsp_pi(&g_controller_ctom.dsp_modules.dsp_pi[i]);

                    SATURATE(g_controller_ctom.output_signals[i],
                             PWM_MAX_DUTY, PWM_MIN_DUTY);
                }

                set_pwm_duty_hbridge(g_pwm_modules.pwm_regs[i*2],
                                     g_controller_ctom.output_signals[i]);

        #if 0
            }
        }
        //SET_DEBUG_GPIO1;
        #endif
    }

    PS1_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS1_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PS2_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS2_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PS3_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS3_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PS4_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS4_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;

    CLEAR_DEBUG_GPIO1;

    PieCtrlRegs.PIEACK.all |= M_INT3;
}

static void init_interruptions(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT =  &isr_init_controller;
    PieVectTable.EPWM2_INT =  &isr_controller;
    EDIS;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // ePWM1
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  // ePWM2

    enable_pwm_interrupt(PS4_PWM_MODULATOR);
    enable_pwm_interrupt(PS4_PWM_MODULATOR_NEG);

    IER |= M_INT1;
    IER |= M_INT3;
    IER |= M_INT11;

    /* Enable global interrupts (EINT) */
    EINT;
    ERTM;
}

static void term_interruptions(void)
{
    /* Disable global interrupts (EINT) */
    DINT;
    DRTM;

    /* Clear enables */
    IER = 0;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 0;  // ePWM1
    PieCtrlRegs.PIEIER3.bit.INTx2 = 0;  // ePWM2

    disable_pwm_interrupt(PS4_PWM_MODULATOR);
    disable_pwm_interrupt(PS4_PWM_MODULATOR_NEG);

    /* Clear flags */
    PieCtrlRegs.PIEACK.all |= M_INT1 | M_INT3 | M_INT11;
}

static void turn_on(uint16_t id)
{
    if(g_ipc_ctom.ps_module[id].ps_status.bit.active)
    {
        if(g_ipc_ctom.ps_module[id].ps_status.bit.state == Off)
        {
            reset_controller(id);
            close_relay(id);

            DELAY_US(TIMEOUT_DCLINK_RELAY);

            #ifdef USE_ITLK
            if(!get_relay_status(id))
            {
                set_hard_interlock(id,DCLINK_RELAY_FAIL);
            }
            else
            #endif
            {
                g_ipc_ctom.ps_module[id].ps_status.bit.openloop = OPEN_LOOP;
                g_ipc_ctom.ps_module[id].ps_status.bit.state = SlowRef;

                enable_pwm_output(2*id);
                enable_pwm_output((2*id)+1);
            }
        }
    }
}

static void turn_off(uint16_t id)
{
    disable_pwm_output(2*id);
    disable_pwm_output((2*id)+1);

    open_relay(id);

    g_ipc_ctom.ps_module[id].ps_status.bit.openloop = OPEN_LOOP;
    if (g_ipc_ctom.ps_module[id].ps_status.bit.state != Interlock){
        g_ipc_ctom.ps_module[id].ps_status.bit.state = Off;
    }
    reset_controller(id);
}

static void reset_interlocks(void)
{
    g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock = 0;
    g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock = 0;

    if(g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state < Initializing)
    {
        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state = Off;
    }
}

static void set_hard_interlock(uint16_t id, uint32_t itlk)
{
    if(!(g_ipc_ctom.ps_module[id].ps_hard_interlock & itlk))
    {
        #ifdef USE_ITLK
        turn_off(id);
        #endif

        g_ipc_ctom.ps_module[id].ps_hard_interlock |= itlk;
        g_ipc_ctom.ps_module[id].ps_status.bit.state = Interlock;
    }
}

static void set_soft_interlock(uint16_t id, uint32_t itlk)
{
    if(!(g_ipc_ctom.ps_module[id].ps_soft_interlock & itlk))
    {
        #ifdef USE_ITLK
        turn_off(id);
        #endif

        g_ipc_ctom.ps_module[id].ps_soft_interlock |= itlk;
        g_ipc_ctom.ps_module[id].ps_status.bit.state = Interlock;
    }
}

interrupt void isr_hard_interlock(void)
{
    if(! (g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock &
         g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock))
    {
        #ifdef USE_ITLK
        turn_off(g_ipc_mtoc.msg_id);
        #endif

        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock |=
        g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock;

        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state = Interlock;
    }
}

interrupt void isr_soft_interlock(void)
{
    if(! (g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock &
         g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock))
    {
        #ifdef USE_ITLK
        turn_off(g_ipc_mtoc.msg_id);
        #endif

        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock |=
        g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock;

        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state = Interlock;
    }
}

static void open_relay(uint16_t id)
{
    switch(id)
    {
        case PS1_ID:
        {
            PIN_OPEN_PS1_DCLINK_RELAY;
            break;
        }

        case PS2_ID:
        {
            PIN_OPEN_PS2_DCLINK_RELAY;
            break;
        }

        case PS3_ID:
        {
            PIN_OPEN_PS3_DCLINK_RELAY;
            break;
        }

        case PS4_ID:
        {
            PIN_OPEN_PS4_DCLINK_RELAY;
            break;
        }

        default:
        {
            break;
        }
    }
}
static void close_relay(uint16_t id)
{
    switch(id)
    {
        case PS1_ID:
        {
            PIN_CLOSE_PS1_DCLINK_RELAY;
            break;
        }

        case PS2_ID:
        {
            PIN_CLOSE_PS2_DCLINK_RELAY;
            break;
        }

        case PS3_ID:
        {
            PIN_CLOSE_PS3_DCLINK_RELAY;
            break;
        }

        case PS4_ID:
        {
            PIN_CLOSE_PS4_DCLINK_RELAY;
            break;
        }

        default:
        {
            break;
        }
    }
}

static uint16_t get_relay_status(uint16_t id)
{
    switch(id)
    {
        case PS1_ID:
        {
            return PIN_STATUS_PS1_DCLINK_RELAY;
        }

        case PS2_ID:
        {
            return PIN_STATUS_PS2_DCLINK_RELAY;
        }

        case PS3_ID:
        {
            return PIN_STATUS_PS3_DCLINK_RELAY;
        }

        case PS4_ID:
        {
            return PIN_STATUS_PS4_DCLINK_RELAY;
        }

        default:
        {
            return 0;
        }
    }
}

static void check_interlocks_ps_module(uint16_t id)
{
    if(fabs(g_controller_ctom.net_signals[id]) > MAX_ILOAD)
    {
        set_hard_interlock(id, LOAD_OVERCURRENT);
    }

    if(fabs(g_controller_mtoc.net_signals[id]) > MAX_DCLINK)
    {
        set_hard_interlock(id, DCLINK_OVERVOLTAGE);
    }

    if(fabs(g_controller_mtoc.net_signals[id]) < MIN_DCLINK)
    {
        set_hard_interlock(id, DCLINK_UNDERVOLTAGE);
    }

    if(fabs(g_controller_mtoc.net_signals[id+4]) > MAX_VLOAD)
    {
        set_hard_interlock(id, LOAD_OVERVOLTAGE);
    }

    if(fabs(g_controller_mtoc.net_signals[id+8]) > MAX_TEMP)
    {
        set_soft_interlock(id, OVERTEMP);
    }

    switch(id)
    {
        case 0:
        {
            if(!PIN_STATUS_PS1_FUSE)
            {
                set_hard_interlock(id, FUSE_FAIL);
            }

            if(!PIN_STATUS_PS1_DRIVER_ERROR)
            {
                set_hard_interlock(id, DRIVER_FAIL);
            }

            if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock) &&
                 (PIN_STATUS_PS1_DCLINK_RELAY) )
            {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            else if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state > Interlock)
                      && (!PIN_STATUS_PS1_DCLINK_RELAY) )
            {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            break;
        }

        case 1:
        {
            if(!PIN_STATUS_PS2_FUSE)
            {
                set_hard_interlock(id, FUSE_FAIL);
            }

            if(!PIN_STATUS_PS2_DRIVER_ERROR)
            {
                set_hard_interlock(id, DRIVER_FAIL);
            }

            if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_PS2_DCLINK_RELAY))
            {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            else if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state > Interlock)
                      && (!PIN_STATUS_PS2_DCLINK_RELAY) )
            {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            break;
        }

        case 2:
        {
            if(!PIN_STATUS_PS3_FUSE)
            {
                set_hard_interlock(id, FUSE_FAIL);
            }

            if(!PIN_STATUS_PS3_DRIVER_ERROR)
            {
                set_hard_interlock(id, DRIVER_FAIL);
            }

            if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_PS3_DCLINK_RELAY)) {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            else if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state > Interlock)
                      && (!PIN_STATUS_PS3_DCLINK_RELAY) )
            {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            break;
        }

        case 3:
        {
            if(!PIN_STATUS_PS4_FUSE)
            {
                set_hard_interlock(id, FUSE_FAIL);
            }

            if(!PIN_STATUS_PS4_DRIVER_ERROR)
            {
                set_hard_interlock(id, DRIVER_FAIL);
            }

            if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_PS4_DCLINK_RELAY)) {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            else if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state > Interlock)
                      && (!PIN_STATUS_PS4_DCLINK_RELAY) )
            {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            break;
        }
    }
}
