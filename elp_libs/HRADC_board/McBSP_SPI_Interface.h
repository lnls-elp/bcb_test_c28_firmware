#ifndef MCBSP_SPI_INTERFACE_H
#define MCBSP_SPI_INTERFACE_H

#include "DSP28x_Project.h"
#include "boards/udc_c28.h"

#define SPI_25MHz			2
#define SPI_18_75MHz		3
#define SPI_15MHz			4
#define SPI_12_5MHz			5
#define SPI_10_71MHz		6
#define SPI_9_375MHz		7
#define SPI_5MHz			14

#define McBSP_CLKGDV 		SPI_10_71MHz
#define McBSP_WORD_SIZE    	20

extern void Init_SPIMaster_McBSP(Uint16 spiClk);
extern void Init_SPIMaster_McBSP_HRADC_UFM(void);
extern void Init_SPIMaster_Gpio(void);
 
#endif
