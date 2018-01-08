#include "DSP28x_Project.h"
//#include "../C28 Project/config.h"

#ifndef DMA_SPI_INTERFACE_H
#define DMA_SPI_INTERFACE_H

#define HRADC_BUFFERS_SIZE	64

typedef volatile struct
{
	Uint32 buffer_0[HRADC_BUFFERS_SIZE];
	Uint32 buffer_1[HRADC_BUFFERS_SIZE];
	Uint32 buffer_2[HRADC_BUFFERS_SIZE];
	Uint32 buffer_3[HRADC_BUFFERS_SIZE];
} tbuffers_HRADC;

extern __interrupt void local_D_INTCH1_ISR(void);
extern __interrupt void local_D_INTCH2_ISR(void);
extern void Init_DMA_McBSP_nBuffers(Uint16 n_buffers, Uint16 size_buffers, Uint16 spiClk);
extern void start_DMA(void);
extern void stop_DMA(void);

extern volatile Uint32 i_rdata;
extern volatile Uint32 dummy_data;
//extern volatile tbuffers_HRADC buffers_HRADC;
extern volatile Uint32 buffers_HRADC[4][HRADC_BUFFERS_SIZE];

#endif	/* DMA_SPI_INTERFACE_H */
