/* TODO: Calibrate DMATransferSize for each SPI_CLK value */

#include "DMA_SPI_Interface.h"

__interrupt void local_D_INTCH1_ISR(void);
__interrupt void local_D_INTCH2_ISR(void);
void Init_DMA_McBSP_nBuffers(Uint16 n_buffers, Uint16 size_buffers, Uint16 spiClk);
void start_DMA(void);
void stop_DMA(void);

//#pragma DATA_SECTION(buffers_HRADC, "SHARERAMS1_1")

volatile Uint16 DMATransferSize[5][4] = { {1, 15, 45, 60},
										  {1, 15, 45, 60},
										  {1, 15, 45, 60},
										  {1, 15, 45, 60},
										  {1, 25, 55, 90} };

volatile Uint32 i_rdata;
volatile Uint32 dummy_data = 0x00000000;
//volatile tbuffers_HRADC buffers_HRADC;
volatile Uint32 buffers_HRADC[4][HRADC_BUFFERS_SIZE];

void Init_DMA_McBSP_nBuffers(Uint16 n_buffers, Uint16 size_buffers, Uint16 spiClk)
{


  i_rdata = 0;

  EALLOW;
  DmaRegs.DMACTRL.bit.HARDRESET = 1;
  __asm(" NOP");                           // Only 1 NOP needed per Design

  //
  // Enable channel 2: will be McBSP-A transmit
  // Enable channel 1, will be McBSP-A receive
  //
  // To accommodate a McBSP data size of > 16-bits, the
  // DMA will do two 16-bit reads per burst and
  // swap the data
  //
  // Transfer two 16-bit words per burst
  // Increment the source by 1 between words
  // Decrement the destination by 1 between words - come�a no MSB e decrementa um para o LSB;
  //
  DmaRegs.CH2.BURST_SIZE.all = 1;
  DmaRegs.CH2.SRC_BURST_STEP = 1;
  DmaRegs.CH2.DST_BURST_STEP = 1;

  DmaRegs.CH1.BURST_SIZE.all = 1;
  DmaRegs.CH1.SRC_BURST_STEP = 1;
  DmaRegs.CH1.DST_BURST_STEP = 0xFFFF;
  //
  // Interrupt every frame ( (TRANSFER_BUFFER_SIZE-2)/2 bursts/transfer)
  //
  DmaRegs.CH2.TRANSFER_SIZE = DMATransferSize[spiClk-2][n_buffers-1];
  DmaRegs.CH1.TRANSFER_SIZE = n_buffers * size_buffers - 1;

  //
  // For transmit, after each burst:
  //     Add 1 to the source: move to the next source word
  //     Subtract 1 from the destination (back to DXR2)
  //
  DmaRegs.CH2.SRC_TRANSFER_STEP = 0xFFFF;
  DmaRegs.CH2.DST_TRANSFER_STEP = 0xFFFF;
  //
  // For receive, after each burst:
  //     Subtract 1 from the source (back to DRR2)
  //     Add 3 to the destination: move to the next word
  //
  DmaRegs.CH1.SRC_TRANSFER_STEP = 0xFFFF;
  //DmaRegs.CH1.DST_TRANSFER_STEP = 2 * size_buffers + 1;
  DmaRegs.CH1.DST_TRANSFER_STEP = 2 * HRADC_BUFFERS_SIZE + 1;
  //
  // Transmit source and destination:
  //   send buffer (source) -> DMA -> McBSP (destination)
  //   Set the source to the start of the sdata buffer
  //   Set the destination to the McBSP DXR2
  //
  DmaRegs.CH2.SRC_ADDR_SHADOW = (Uint32) &dummy_data;
  DmaRegs.CH2.DST_ADDR_SHADOW = (Uint32) &McbspaRegs.DXR2.all;
  //
  // Receive source and destination:
  //   McBSP (source) -> DMA -> receive buffer (destination)
  //   Set the source to the McBSP DRR2
  //   Set the destination to the start of the receive buffer
  //
  DmaRegs.CH1.SRC_ADDR_SHADOW = (Uint32) &McbspaRegs.DRR2.all;
  DmaRegs.CH1.DST_ADDR_SHADOW = ((Uint32) &buffers_HRADC) + 1;
  DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32) &McbspaRegs.DRR2.all;
  DmaRegs.CH1.DST_BEG_ADDR_SHADOW = ((Uint32) &buffers_HRADC) + 1;
  //
  // Clear sync flag and error flag
  //
  DmaRegs.CH2.CONTROL.bit.ERRCLR = 1;
  DmaRegs.CH1.CONTROL.bit.ERRCLR = 1;
  //
  // Do not use the wrap function
  // Set the wrap size to the maximum value
  // which in effect disables it.
  //
  DmaRegs.CH2.SRC_WRAP_SIZE = 0xFFFF;
  DmaRegs.CH2.DST_WRAP_SIZE = 0xFFFF;
  DmaRegs.CH2.SRC_WRAP_STEP = 0;
  DmaRegs.CH2.DST_WRAP_STEP = 0;

  DmaRegs.CH1.SRC_WRAP_SIZE = 0xFFFF;
  DmaRegs.CH1.DST_WRAP_SIZE = n_buffers - 1;
  DmaRegs.CH1.SRC_WRAP_STEP = 0;
  DmaRegs.CH1.DST_WRAP_STEP = 2;

  //
  // Enable channel interrupt at end of transfer
  // Interrupt at end of the transfer (mode)
  // Enable peripheral interrupt event
  // Peripheral interrupt for transfer is McBSP MXEVTA
  //    McBSP transmit buffer is empty
  // Peripheral interrupt for receive is McBSP MREVTA
  //    McBSP receive buffer is full
  // Clear any interrupt flags

  DmaRegs.CH1.MODE.bit.CONTINUOUS = 1;
  DmaRegs.CH2.MODE.bit.CONTINUOUS = 1;
  DmaRegs.CH1.MODE.bit.ONESHOT = 0; //(n_buffers > 1);
  DmaRegs.CH2.MODE.bit.ONESHOT = (n_buffers > 1);

  DmaRegs.PRIORITYCTRL1.bit.CH1PRIORITY = 1;
  DmaRegs.DMACTRL.bit.PRIORITYRESET = 1;

  DmaRegs.CH2.MODE.bit.CHINTE = 1; // Disable
  DmaRegs.CH2.MODE.bit.CHINTMODE = 1;
  DmaRegs.CH2.MODE.bit.PERINTE = 1;
  DmaRegs.CH2.MODE.bit.PERINTSEL = DMA_XINT1;
  DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1;
  XIntruptRegs.XINT1CR.bit.POLARITY = 0;   // XINT1 (HRADC_BUSY_OUT) negative-edge triggering
  XIntruptRegs.XINT1CR.bit.ENABLE = 1;	   // XINT1 Enable

  DmaRegs.CH1.MODE.bit.CHINTE = 0;
  DmaRegs.CH1.MODE.bit.CHINTMODE = 1;
  DmaRegs.CH1.MODE.bit.PERINTE = 1;
  DmaRegs.CH1.MODE.bit.PERINTSEL = DMA_MREVTA;
  DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;
  EDIS;
}
//*****************************************************************************
// Start DMA transmit and receive from McBSP A
//*****************************************************************************
void start_DMA(void)
{
    //
    // Allow access to protected registers (EALLOW)
    // Enable channel 1 and channel 2
    // Disable access to protected registers (EDIS)
    //

    EALLOW;
    DmaRegs.CH1.CONTROL.bit.RUN = 1;
    DmaRegs.CH2.CONTROL.bit.RUN = 1;
    //DmaRegs.CH3.CONTROL.bit.RUN = 1;
    EDIS;
}

//*****************************************************************************
// Stop DMA transmit and receive from McBSP A
//*****************************************************************************
void stop_DMA(void)
{
    //
    // Allow access to protected registers (EALLOW)
    // Enable channel 1 and channel 2
    // Disable access to protected registers (EDIS)
    //

    DELAY_US(10);

    EALLOW;
    DmaRegs.CH1.CONTROL.bit.HALT = 1;
    DmaRegs.CH2.CONTROL.bit.HALT = 1;
    DmaRegs.CH3.CONTROL.bit.HALT = 1;

    i_rdata = DmaRegs.CH1.DST_ADDR_ACTIVE;

    DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;
    DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1;
    DmaRegs.CH3.CONTROL.bit.PERINTCLR = 1;

    DmaRegs.CH1.CONTROL.bit.SOFTRESET = 1;
    DmaRegs.CH2.CONTROL.bit.SOFTRESET = 1;
    DmaRegs.CH3.CONTROL.bit.SOFTRESET = 1;
    EDIS;
}

//*****************************************************************************
// DMA Channel 1 interrupt service routine
//*****************************************************************************
__interrupt void local_D_INTCH1_ISR(void)
{
    //
    // Allow access to protected registers (EALLOW)
    // Disable DMA CH1 by setting HALT to 1
    // Do not re-enable DMA CH1. This will be done in the channel 2 ISR
    // To receive more interrupts from this PIE group, acknowledge this interrupt
    // Disable access to protected registers (EDIS)
    //
    EALLOW;
    DmaRegs.CH1.CONTROL.bit.HALT = 1;
    DmaRegs.CH2.CONTROL.bit.HALT = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

    DmaRegs.CH1.CONTROL.bit.RUN = 1;
    DmaRegs.CH2.CONTROL.bit.RUN = 1;

    EDIS;
    return;
}

//*****************************************************************************
// DMA Channel 2 interrupt service routine
//*****************************************************************************
__interrupt void local_D_INTCH2_ISR(void)
{
    //
    // Allow access to protected registers (EALLOW)
    // Stop DMA CH2
    // Process Data
    // Re-enable DMA CH1 and CH2. Should be done every transfer
    // To receive more interrupts from this PIE group, acknowledge this interrupt
    // Check the received data against the expected data
    // If there is an error go to the error() function
    // Disable access to protected registers (EDIS)
    //
    EALLOW;
    DmaRegs.CH1.CONTROL.bit.HALT = 1;
    DmaRegs.CH2.CONTROL.bit.HALT = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

    DmaRegs.CH1.CONTROL.bit.RUN = 1;
    DmaRegs.CH2.CONTROL.bit.RUN = 1;

    EDIS;
    return;
}
