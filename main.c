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
 * @file main.c
 * @brief Main file of firmware for C28 core from DRS-UDC board.
 *
 * Main file of firmware for C28 core from DRS-UDC board. This firmware
 * implements digital controllers for magnet power supplies from Sirius Project.
 * 
 * At initialization, the ARM core reads from non-volatile memory which power
 * supply model the controller is set, in order to both cores be initialized
 * with the proper power supply module (ps_module).
 *
 * @author gabriel.brunheira
 * @date 20/10/2017
 *
 */

#include <string.h>
#include "boards/udc_c28.h"
#include "ipc/ipc.h"

#define SCI_FREQ        3125000
#define LSPCLK_FREQ     C28_FREQ_MHZ*1e6/(LSPCLK_DV+1)
#define SCI_BAUD        (LSPCLK_FREQ/(SCI_FREQ*8))-1

uint16_t scia_rx_data[2];

#pragma CODE_SECTION(isr_scia_rx_fifo, "ramfuncs")

static void init_scia_fifo(void);
interrupt void isr_scia_rx_fifo(void);

/**
 * @brief Main function
 */
void main(void)
{
    /**
     * Initialize the Control System:
     * Enable peripheral clocks
     * This example function is found in the F28M36x_SysCtrl.c file.
     */
    InitSysCtrl();

    /**
     * Copy time critical code and Flash setup code to RAM
     * This includes the following functions:  InitFlash();
     * The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
     * symbols are created by the linker. Refer to the device .cmd file.
     */
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

    /**
     * Call Flash Initialization to setup flash waitstates
     * This function must reside in RAM
     */
    InitFlash();

    /**
     * Disable CPU interrupts
     */
    DINT;

    /**
     * Initialize the PIE control registers to their default state.
     * The default state is all PIE interrupts disabled and flags are cleared.
     * This function is found in the F28M36x_PieCtrl.c file.
     */
    InitPieCtrl();

    /**
     * Disable CPU interrupts and clear all CPU interrupt flags:
     */
    IER = 0x0000;
    IFR = 0x0000;

    /**
     * Initialize the PIE vector table with pointers to the shell Interrupt
     * Service Routines (ISR).
     * This will populate the entire table, even if the interrupt
     * is not used in this example.  This is useful for debug purposes.
     * The shell ISR routines are found in F28M36x_DefaultIsr.c.
     * This function is found in F28M36x_PieVect.c.
     */
    InitPieVectTable();

    init_gpios();
    init_buzzer(50);     /// Volume: 50 %
    init_scia_fifo();

    enable_pwm_tbclk();

    /* Enable global interrupts (EINT) */
    EINT;
    ERTM;

    while(1)
    {
        main_fbp();
    }
}

static void init_scia_fifo(void)
{
    EALLOW;

    GpioCtrlRegs.GPDMUX2.bit.GPIO117 = 0;
    GpioDataRegs.GPDCLEAR.bit.GPIO117 = 1;      // GPIO117: SCI_RD
    GpioCtrlRegs.GPDDIR.bit.GPIO117 = 1;

    GpioCtrlRegs.GPDMUX2.bit.GPIO118 = 2;       // GPIO118: SCITXDA

    GpioCtrlRegs.GPDMUX2.bit.GPIO119 = 2;       // GPIO119: SCIRXDA
    GpioCtrlRegs.GPDQSEL2.bit.GPIO119 = 3;

    EDIS;

    // SCICCR
    //    Character length 8 bits
    //    1 stop bit
    //    Parity not enabled
    //    Idle-line mode protocol selected
    SciaRegs.SCICCR.bit.SCICHAR =(0x0008-1);
    SciaRegs.SCICCR.bit.STOPBITS = 0;
    SciaRegs.SCICCR.bit.PARITYENA = 0;
    SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0;

    // SCICTL1:
    //    Enable RX and TX
    //    Leave SLEEP mode disabled
    //    Leave RX Err disabled
    //    Leave transmitter wakeup feature disabled
    // SCICTL2:
    //    Enable RXRDY/BRKDT interrupt
    SciaRegs.SCICTL1.bit.RXENA = 1;
    SciaRegs.SCICTL1.bit.TXENA = 1;
    SciaRegs.SCICTL2.bit.TXINTENA = 0;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
    SciaRegs.SCIHBAUD = 0x0000;
    SciaRegs.SCILBAUD = SCI_BAUD;

    // Transmit FIFO reset
    // 2 level transmit FIFO
    // Disable transmit FIFO interrupt
    // Enable SCI FIFO enhancements
    // Receive FIFO reset
    // 2 level receive FIFO
    // Enable receive FIFO interrupt
    //
    SciaRegs.SCIFFTX.all=0xC001;
    SciaRegs.SCIFFRX.all=0x0021;
    SciaRegs.SCIFFCT.all=0x0;

    // Release the SCI from reset
    // Release the FIFOs from reset
    SciaRegs.SCICTL1.all = 0x0023;
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET=1;

    EALLOW;
    PieVectTable.SCIRXINTA = &isr_scia_rx_fifo;
    EDIS;

    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;  // SCIRXINTA

    IER |= M_INT9;
}

interrupt void isr_scia_rx_fifo(void)
{

    #if UDC_SELECT
    scia_rx_data[0] = SciaRegs.SCIRXBUF.all;

    g_ipc_ctom.ps_module[0].ps_status.all = scia_rx_data[0];
    //
    // Clear Interrupt flag
    // Issue PIE acknowledge to enable more interrupts from this group
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;
    PieCtrlRegs.PIEACK.all |= M_INT9;
    #else
    //
    // Clear Interrupt flag
    // Issue PIE acknowledge to enable more interrupts from this group
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;
    PieCtrlRegs.PIEACK.all |= M_INT9;
    #endif
}

