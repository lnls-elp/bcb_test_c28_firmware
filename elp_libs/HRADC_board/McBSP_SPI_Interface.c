#include "McBSP_SPI_Interface.h"

void Init_SPIMaster_McBSP(Uint16 spiClk);
void Init_SPIMaster_McBSP_HRADC_UFM(void);
void Init_SPIMaster_Gpio(void);

//===========================================================================
// Configure McBSP-A as SPI Master
//===========================================================================

void Init_SPIMaster_McBSP(Uint16 spiClk)
{
    // Reset the McBSP
    // Disable all interrupts
    // Frame sync generator reset
	// Transmitter reset
	// Receiver reset
    // Sample rate generator reset
	McbspaRegs.SPCR2.bit.FRST = 0;
	McbspaRegs.SPCR2.bit.XRST = 0;
	McbspaRegs.SPCR1.bit.RRST = 0;
	McbspaRegs.SPCR2.bit.GRST = 0;

    //CLKSTP configured without clock delay
    McbspaRegs.SPCR1.bit.CLKSTP = 0x02;

    //CLKXP positive polarity
    McbspaRegs.PCR.bit.CLKXP = 0;

    //CLKRP negative polarity
    McbspaRegs.PCR.bit.CLKRP = 0;

    // CLKX is driven by the sample rate generator
    McbspaRegs.PCR.bit.CLKXM = 1;

    //Sample rate generator input clock is LSPCLK
    McbspaRegs.SRGR2.bit.CLKSM = 1;		//
    McbspaRegs.PCR.bit.SCLKME = 0;

    //CLKGDV divider for the generated clock (CLKG)
    McbspaRegs.SRGR1.bit.CLKGDV = spiClk;

    //Transmit frame synchronization driven according FSGM bit
    McbspaRegs.PCR.bit.FSXM = 1;

    //The transmitter drives a frame-synchronization pulse on the FSX pin every time data is transferred from DXR1 to XSR1.
    McbspaRegs.SRGR2.bit.FSGM = 0;

    //FSX pin is active low
    McbspaRegs.PCR.bit.FSXP = 1;

    //RX data delay is 1 bit
    //TX data delay is 1 bit
    McbspaRegs.RCR2.bit.RDATDLY = 1;
    McbspaRegs.XCR2.bit.XDATDLY = 1;

    // Enable Sample rate generator and
    // wait at least 2 CLKG clock cycles
    McbspaRegs.SPCR2.bit.GRST = 1;
    clkg_delay_loop();

    //Enable Trasmissor e receptor
    McbspaRegs.SPCR2.bit.XRST = 1;
    McbspaRegs.SPCR1.bit.RRST = 1;
    clkg_delay_loop();

    //Enable Frame-Sincronization Logic
    McbspaRegs.SPCR2.bit.FRST = 1;


    /**************************/

    // Reset TX and RX
    // Enable interrupts for TX and RX
    // Release TX and RX
    McbspaRegs.SPCR2.bit.XRST = 0;
    McbspaRegs.SPCR1.bit.RRST = 0;
    //McbspaRegs.MFFINT.bit.XINT = 1;
    //McbspaRegs.MFFINT.bit.RINT = 1;

    //McbspaRegs.RCR1.bit.RWDLEN1 = 3;
    //McbspaRegs.XCR1.bit.XWDLEN1 = 3;

    McbspaRegs.SPCR2.bit.XRST = 1;
    McbspaRegs.SPCR1.bit.RRST = 1;
}

//===========================================================================
// Configure McBSP-A as SPI Master
//===========================================================================

void Init_SPIMaster_McBSP_HRADC_UFM(void)
{
    // Reset the McBSP
    // Disable all interrupts
    // Frame sync generator reset
	// Transmitter reset
	// Receiver reset
    // Sample rate generator reset
	McbspaRegs.SPCR2.bit.FRST = 0;
	McbspaRegs.SPCR2.bit.XRST = 0;
	McbspaRegs.SPCR1.bit.RRST = 0;
	McbspaRegs.SPCR2.bit.GRST = 0;

    //CLKSTP configured with clock delay
    McbspaRegs.SPCR1.bit.CLKSTP = 0x03;

    //CLKXP positive polarity
    McbspaRegs.PCR.bit.CLKXP = 0;

    //CLKRP positive polarity
    McbspaRegs.PCR.bit.CLKRP = 1;

    // CLKX is driven by the sample rate generator
    McbspaRegs.PCR.bit.CLKXM = 1;

    //Sample rate generator input clock is LSPCLK
    McbspaRegs.SRGR2.bit.CLKSM = 1;		//
    McbspaRegs.PCR.bit.SCLKME = 0;

    //CLKGDV divider for the generated clock (CLKG)
    McbspaRegs.SRGR1.bit.CLKGDV = SPI_5MHz;

    //Transmit frame synchronization driven according FSGM bit
    McbspaRegs.PCR.bit.FSXM = 1;

    //The transmitter drives a frame-synchronization pulse on the FSX pin every time data is transferred from DXR1 to XSR1.
    McbspaRegs.SRGR2.bit.FSGM = 0;

    //FSX pin is active low
    McbspaRegs.PCR.bit.FSXP = 1;

    //RX data delay is 1 bit
    //TX data delay is 1 bit
    McbspaRegs.RCR2.bit.RDATDLY = 1;
    McbspaRegs.XCR2.bit.XDATDLY = 1;

    // Enable Sample rate generator and
    // wait at least 2 CLKG clock cycles
    McbspaRegs.SPCR2.bit.GRST = 1;
    clkg_delay_loop();

    //Enable Trasmissor e receptor
    McbspaRegs.SPCR2.bit.XRST = 1;
    McbspaRegs.SPCR1.bit.RRST = 1;
    clkg_delay_loop();

    //Enable Frame-Sincronization Logic
    McbspaRegs.SPCR2.bit.FRST = 1;

    /**************************/

    // Reset TX and RX
    // Enable interrupts for TX and RX
    // Release TX and RX
    McbspaRegs.SPCR2.bit.XRST = 0;
    McbspaRegs.SPCR1.bit.RRST = 0;
    //McbspaRegs.MFFINT.bit.XINT = 1;
    //McbspaRegs.MFFINT.bit.RINT = 1;

    //McbspaRegs.RCR1.bit.RWDLEN1 = 3;
    //McbspaRegs.XCR1.bit.XWDLEN1 = 3;

    McbspaRegs.SPCR2.bit.XRST = 1;
    McbspaRegs.SPCR1.bit.RRST = 1;
}

//===========================================================================
// Configure GPIO's for McBSP-A as SPI Master
//===========================================================================

void Init_SPIMaster_Gpio(void)
{
    EALLOW;

    // MDXA : GPIO20 <- HRADC_SPI_IN
    // Qualification is asynchronous
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 2;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 3;

    // MDRA : GPIO21 <- HRADC_SPI_OUT
    // Qualification is asynchronous
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 2;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 3;

    // MCLKXA : GPIO22 <- HRADC_SPI_CLK
    // Qualification is asynchronous
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 2;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 3;

    // Input : GPIO23 <- HRADC_BUSY_OUT
    // Qualification is asynchronous
    // Define GPIO23 as interrupt source XINT1
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;
    GpioTripRegs.GPTRIP4SEL.bit.GPTRIP4SEL = 0x17;

    if(UDC_V2_0)
	{
        // GPIO_CS1 : GPIO128 <- HRADC_CS1
    	GpioCtrlRegs.GPEMUX1.bit.GPIO128 = 0;
    	GpioDataRegs.GPECLEAR.bit.GPIO128 = 1;
    	GpioCtrlRegs.GPEDIR.bit.GPIO128 = 1;

    	// PWM_SOC : GPIO130 <- HRADC_CNVST
    	GpioCtrlRegs.GPEMUX1.bit.GPIO130 = 1;
    	GpioDataRegs.GPESET.bit.GPIO130 = 1;
    	GpioCtrlRegs.GPEDIR.bit.GPIO130 = 1;

    	// STATUS_ADC0 : GPIO132 <- HRADC_STATUS_A
    	GpioCtrlRegs.GPEMUX1.bit.GPIO132 = 0;
    	GpioCtrlRegs.GPEDIR.bit.GPIO132 = 0;

    	// STATUS_ADC2 : GPIO134 <- HRADC_STATUS_C
    	GpioCtrlRegs.GPEMUX1.bit.GPIO134 = 0;
    	GpioCtrlRegs.GPEDIR.bit.GPIO134 = 0;
	}

	else if(UDC_V2_1)
	{
	    // GPIO_CS1 : GPIO130 <- HRADC_CS1
		GpioCtrlRegs.GPEMUX1.bit.GPIO130 = 0;
		GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;
		GpioCtrlRegs.GPEDIR.bit.GPIO130 = 1;

    	// PWM_SOC : GPIO32 <- HRADC_CNVST
		GpioDataRegs.GPBSET.bit.GPIO32 = 1;
		GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 3;
    	GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;

    	// STATUS_ADC0 : GPIO42 <- HRADC_STATUS_A
    	GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0;
    	GpioCtrlRegs.GPBDIR.bit.GPIO42 = 0;

    	// STATUS_ADC2 : GPIO45 <- HRADC_STATUS_C
    	GpioCtrlRegs.GPBMUX1.bit.GPIO45 = 0;
    	GpioCtrlRegs.GPBDIR.bit.GPIO45 = 0;
	}

	// GPIO_CS2 : GPIO129 <- HRADC_CS2
	GpioCtrlRegs.GPEMUX1.bit.GPIO129 = 0;
	GpioDataRegs.GPECLEAR.bit.GPIO129 = 1;
	GpioCtrlRegs.GPEDIR.bit.GPIO129 = 1;

	// GPIO_CONFIG : GPIO131 <- HRADC_CONFIG
	GpioCtrlRegs.GPEMUX1.bit.GPIO131 = 0;
	GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;
	GpioCtrlRegs.GPEDIR.bit.GPIO131 = 1;

	// STATUS_ADC1 : GPIO133 <- HRADC_STATUS_B
	GpioCtrlRegs.GPEMUX1.bit.GPIO133 = 0;
	GpioCtrlRegs.GPEDIR.bit.GPIO133 = 0;

	// STATUS_ADC3 : GPIO135 <- HRADC_STATUS_D
	GpioCtrlRegs.GPEMUX1.bit.GPIO135 = 0;
	GpioCtrlRegs.GPEDIR.bit.GPIO135 = 0;

	EDIS;

}
