/*
 * 		FILE: 		HRADC_Boards.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	07/23/2015
 * 		MODIFIED:	07/23/2015
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code for access and control of HRADC boards
 *
 *		TODO: Finish CheckStatus_HRADC()
 *		TODO: Create functions to access flash memory (UFM)
 */

#include <math.h>
#include "HRADC_Boards.h"

/**********************************************************************************************/
//
// 	Prototype statements for functions found within this file
//
void Init_HRADC_Info(volatile HRADC_struct *hradcPtr, Uint16 ID, Uint16 buffer_size, volatile Uint32 *buffer, float transducer_gain);
void Config_HRADC_board(volatile HRADC_struct *hradcPtr, eInputType AnalogInput, Uint16 enHeater, Uint16 enRails);
Uint16 Try_Config_HRADC_board(volatile HRADC_struct *hradcPtr, eInputType AnalogInput, Uint16 enHeater, Uint16 enRails);

void SendCommand_HRADC(volatile HRADC_struct *hradcPtr, Uint16 command);
Uint16 CheckStatus_HRADC(volatile HRADC_struct *hradcPtr);

void Config_HRADC_SoC(float freq);
void Enable_HRADC_Sampling(void);
void Disable_HRADC_Sampling(void);

void Config_HRADC_Sampling_OpMode(Uint16 ID);
void Config_HRADC_UFM_OpMode(Uint16 ID);

void Erase_HRADC_UFM(Uint16 ID);
void Read_HRADC_UFM(Uint16 ID, Uint16 ufm_address, Uint16 n_words, volatile Uint16 *ufm_buffer);
void Write_HRADC_UFM(Uint16 ID, Uint16 ufm_address, Uint16 data);

void Read_HRADC_BoardData(HRADC_struct *hradcPtr);

/**********************************************************************************************/
//
// 	Global variables instantiation
// 	Structs gather all information regarding used HRADC boards
//	Place them in shared memory RAMS1
//
#pragma DATA_SECTION(HRADCs_Info, "SHARERAMS1_1")
/*#pragma DATA_SECTION(HRADC0_board, "SHARERAMS1_1")
#pragma DATA_SECTION(HRADC1_board, "SHARERAMS1_1")
#pragma DATA_SECTION(HRADC2_board, "SHARERAMS1_1")
#pragma DATA_SECTION(HRADC3_board, "SHARERAMS1_1")*/

volatile HRADCs_struct HRADCs_Info;
/*volatile HRADC_struct  HRADC0_board;
volatile HRADC_struct  HRADC1_board;
volatile HRADC_struct  HRADC2_board;
volatile HRADC_struct  HRADC3_board;*/


volatile Uint32 counterErrorSendCommand;
volatile float AverageFilter;

volatile Uint32 HRADC_BoardSelector[4] = GPE_PORT_BITS_HRADC_CS;

/**********************************************************************************************/
//
//	Initialize information of selected HRADC board
//

void Init_HRADC_Info(volatile HRADC_struct *hradcPtr, Uint16 ID, Uint16 buffer_size, volatile Uint32 *buffer, float transducer_gain )
{
	hradcPtr->ID = ID;
	hradcPtr->index_SamplesBuffer = 0;
	hradcPtr->size_SamplesBuffer = buffer_size;
	hradcPtr->SamplesBuffer = buffer;

	while(hradcPtr->SamplesBuffer < &buffer[buffer_size])
	{
		*(hradcPtr->SamplesBuffer++) = 0.0;
	}

	hradcPtr->SamplesBuffer = buffer;

	Config_HRADC_UFM_OpMode(ID);

	Read_HRADC_BoardData(hradcPtr);

	Config_HRADC_Sampling_OpMode(ID);

	if( isinf(hradcPtr->BoardData.t.GND_bipolar) || isnan(hradcPtr->BoardData.t.GND_bipolar)  )
	{
	    hradcPtr->BoardData.t.gain_Vin_bipolar =    1.0;
        hradcPtr->BoardData.t.offset_Vin_bipolar =  0.0;

        hradcPtr->BoardData.t.gain_Iin_bipolar =    1.0;
        hradcPtr->BoardData.t.offset_Iin_bipolar =  0.0;

        hradcPtr->BoardData.t.Rburden =             20.0;
	}

    hradcPtr->BoardData.t.gain_Vin_bipolar *= 		transducer_gain * HRADC_VIN_BI_P_GAIN;
    hradcPtr->BoardData.t.offset_Vin_bipolar -= 	hradcPtr->BoardData.t.gain_Vin_bipolar*HRADC_BI_OFFSET;

    hradcPtr->BoardData.t.gain_Iin_bipolar *= 		transducer_gain * (1.0/(hradcPtr->BoardData.t.Rburden * HRADC_BI_OFFSET));
    hradcPtr->BoardData.t.offset_Iin_bipolar -=		hradcPtr->BoardData.t.gain_Iin_bipolar*HRADC_BI_OFFSET;

    hradcPtr->gain = hradcPtr->BoardData.t.gain_Vin_bipolar;
    hradcPtr->offset = hradcPtr->BoardData.t.offset_Vin_bipolar;
}

/**********************************************************************************************/
//
//	Configure HRADC board with selected parameters and check status
//
void Config_HRADC_board(volatile HRADC_struct *hradcPtr, eInputType AnalogInput, Uint16 enHeater, Uint16 enRails)
{
	Uint16 command;

	if(HRADCs_Info.enable_Sampling)
	{
		return;
	}

	switch(AnalogInput)
	{
		case Vin_bipolar:
		{
			hradcPtr->gain = hradcPtr->BoardData.t.gain_Vin_bipolar;
			hradcPtr->offset = hradcPtr->BoardData.t.offset_Vin_bipolar;
			break;
		}

		case Iin_bipolar:
		{
			hradcPtr->gain = hradcPtr->BoardData.t.gain_Iin_bipolar;
			hradcPtr->offset = hradcPtr->BoardData.t.offset_Iin_bipolar;
			break;
		}

		default:
		{
			hradcPtr->gain = hradcPtr->BoardData.t.gain_Vin_bipolar;
			hradcPtr->offset = hradcPtr->BoardData.t.offset_Vin_bipolar;
			break;
		}
	}

	// Store new configuration parameters
	hradcPtr->AnalogInput = AnalogInput;
	hradcPtr->enable_Heater = enHeater;
	hradcPtr->enable_RailsMonitor = enRails;

	// Create command according to the HRADC Command Protocol (HCP)
	command = ((enRails << 8) | (enHeater << 7) | (AnalogInput << 3)) & 0x01F8;

	SendCommand_HRADC(hradcPtr, command);
	//CheckStatus_HRADC(hradcPtr);

	while(CheckStatus_HRADC(hradcPtr))
	{
		SendCommand_HRADC(hradcPtr, command);
	}
}

Uint16 Try_Config_HRADC_board(volatile HRADC_struct *hradcPtr, eInputType AnalogInput, Uint16 enHeater, Uint16 enRails)
{
	Uint16 command;

	if(HRADCs_Info.enable_Sampling)
	{
		return 1;
	}

	switch(AnalogInput)
	{
		case Vin_bipolar:
		{
			hradcPtr->gain = hradcPtr->BoardData.t.gain_Vin_bipolar;
			hradcPtr->offset = hradcPtr->BoardData.t.offset_Vin_bipolar;
			break;
		}

		case Iin_bipolar:
		{
			hradcPtr->gain = hradcPtr->BoardData.t.gain_Iin_bipolar;
			hradcPtr->offset = hradcPtr->BoardData.t.offset_Iin_bipolar;
			break;
		}

		default:
		{
			hradcPtr->gain = hradcPtr->BoardData.t.gain_Vin_bipolar;
			hradcPtr->offset = hradcPtr->BoardData.t.offset_Vin_bipolar;
			break;
		}
	}

	// Store new configuration parameters
	hradcPtr->AnalogInput = AnalogInput;
	hradcPtr->enable_Heater = enHeater;
	hradcPtr->enable_RailsMonitor = enRails;

	// Create command according to the HRADC Command Protocol (HCP)
	command = ((enRails << 8) | (enHeater << 7) | (AnalogInput << 3)) & 0x01F8;

	// Configure CPU Timer 1 for HRADC configuration timeout monitor
	ConfigCpuTimer(&CpuTimer1, C28_FREQ_MHZ, TIMEOUT_uS_HRADC_CONFIG);
	CpuTimer1Regs.TCR.all = 0x8000;

	// Try to configure HRADC board
	SendCommand_HRADC(hradcPtr, command);

	// Start timeout monitor
	StartCpuTimer1();

	// Check HRADC configuration
	while(CheckStatus_HRADC(hradcPtr))
	{
		// If timeout, stops test
		if(CpuTimer1Regs.TCR.bit.TIF)
		{
			StopCpuTimer1();
			CpuTimer1Regs.TCR.all = 0x8000;
			return 1;
		}
		else
		{
			SendCommand_HRADC(hradcPtr, command);
		}
	}

	return 0;
}

/**********************************************************************************************/
//
//	Send command to selected HRADC board
//
void SendCommand_HRADC(volatile HRADC_struct *hradcPtr, Uint16 command)
{
	Uint32 auxH;
	Uint32 auxL;

	if(HRADCs_Info.enable_Sampling)
	{
		return;
	}

	// Stop DMA controller to prevent DMA access to McBSP
	// during configuration
	stop_DMA();

	// Set appropriate Chip-Select and CONFIG signals
	//GpioDataRegs.GPECLEAR.all = 3;
	//GpioDataRegs.GPESET.all = hradcPtr->ID;
	//GpioDataRegs.GPESET.bit.GPIO131 = 1; 	// Aciona CONFIG

	HRADC_CS_CLEAR;
	HRADC_CS_SET(hradcPtr->ID);
	HRADC_CONFIG_SET;

	// Transmit command via SPI
	McbspaRegs.DXR2.all = 0x0000;
	McbspaRegs.DXR1.all = command;

	// Wait for end of transmission/reception
	while(!McbspaRegs.SPCR1.bit.RRDY){}

	// Store previous HRADC configuration and status
	auxH = (Uint32) McbspaRegs.DRR2.all << 16;
	auxL = (Uint32) McbspaRegs.DRR1.all;
	hradcPtr->StatusReg = auxH + auxL;

	// Clear DMA events triggers flags
	EALLOW;
	DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;
	DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1;
	EDIS;

	// Reset Chip-Select and CONFIG signals
	//GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;
	//GpioDataRegs.GPECLEAR.all = 3;
	HRADC_CONFIG_CLEAR;
	HRADC_CS_CLEAR;
}

/**********************************************************************************************/
//
//	Check status flags of selected HRADC board and take appropriate action
//
Uint16 CheckStatus_HRADC(volatile HRADC_struct *hradcPtr)
{
	Uint16 statusAux;

	if(HRADCs_Info.enable_Sampling)
	{
		return 1;
	}

	SendCommand_HRADC(hradcPtr,CHECK_STATUS);
	statusAux = (hradcPtr->StatusReg & 0x00000078) >> 3;

	if(statusAux != hradcPtr->AnalogInput)
	{
		counterErrorSendCommand++;
		return 1;
	}
	else
	{
		return 0;
	}
}

/**********************************************************************************************/
//
//	Configure Start-of-Conversion generator
//
void Config_HRADC_SoC(float freq)
{
	if(HRADCs_Info.enable_Sampling)
	{
		return;
	}

	/*
	 * Configure ePWM10 module as Start-of-Conversion generator
	 *
	 * 		ePWM10 is synchronized with CTR = ZERO event from ePWM1
	 */
	init_pwm_module(&EPwm10Regs, freq, 0, PWM_Sync_Slave, 0,
	                PWM_ChB_Independent, 0);
	HRADCs_Info.freq_Sampling = freq;

	EALLOW;

	if(UDC_V2_0)
	{
		/*
		 * 	Modify standard initialization:
		 *  	1. Configure as low-active signal
		 *  	2. CMPA = 15 sysclk -> 100 ns CNVST pulse
		 *  	3. Disable One-Shot Trip
		 */

		EPwm10Regs.AQCTLA.bit.CAU = AQ_SET;
		EPwm10Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
		EPwm10Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
		EPwm10Regs.CMPA.half.CMPA = 15;
		EPwm10Regs.TZSEL.bit.OSHT1 = 0;
		EPwm10Regs.TZCLR.bit.OST = 1;

		GpioCtrlRegs.GPEMUX1.bit.GPIO130 = 1;			// Set GPIO130 as HRADC_CNVST
	}

    else if(UDC_V2_1)
    {
		EPwm10Regs.ETSEL.bit.SOCAEN = 1;				// Habilita evento que dispara ADCSOC
		EPwm10Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;		// Dispara ADCSOC quando TBCTR = 0x00
		EPwm10Regs.ETPS.bit.SOCAPRD = ET_1ST;			// Dispara ADCSOC a cada TBCTR = 0x00

		GpioDataRegs.GPBSET.bit.GPIO32 	= 0x1;			// Seta GPIO32 para n�o disparar o ADC precocemente
		GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0x3;			// Set GPIO32 as HRADC_CNVST
    }

	EDIS;
}

void Enable_HRADC_Sampling(void)
{
	if(HRADCs_Info.enable_Sampling)
	{
		return;
	}
	HRADCs_Info.enable_Sampling = 1;

	//stop_DMA();
	//DELAY_US(5);
	start_DMA();
	//EnablePWMOutputs();
	enable_pwm_tbclk();

}

void Disable_HRADC_Sampling(void)
{
	if(HRADCs_Info.enable_Sampling)
	{
		//DisablePWMOutputs();
		disable_pwm_tbclk();
		DELAY_US(10);
		stop_DMA();
		HRADCs_Info.enable_Sampling = 0;
	}
}

void Config_HRADC_Sampling_OpMode(Uint16 ID)
{
	Uint32 auxH, auxL;

	if(HRADCs_Info.enable_Sampling)
	{
		return;
	}

	// Stop DMA controller to prevent DMA access to McBSP
	// during configuration
	stop_DMA();

	Init_SPIMaster_McBSP(SPI_15MHz);
    InitMcbspa20bit();

	// Set appropriate Chip-Select and CONFIG signals
	HRADC_CS_CLEAR;
	HRADC_CS_SET(ID);
	HRADC_CONFIG_SET;

	// Transmit command via SPI (UFM mode)
	McbspaRegs.DXR2.all = 0x0000;
	McbspaRegs.DXR1.all = 0x0000;	// OpMode: Sampling

	// Wait for end of transmission/reception
	while(!McbspaRegs.SPCR1.bit.RRDY){}

	// Store previous HRADC configuration and status
	auxH = (Uint32) McbspaRegs.DRR2.all << 16;
	auxL = (Uint32) McbspaRegs.DRR1.all;
	HRADCs_Info.HRADC_boards[ID].StatusReg = auxH + auxL;

	// Clear Chip-Select and CONFIG signals
	HRADC_CONFIG_CLEAR;
	HRADC_CS_CLEAR;
}

void Config_HRADC_UFM_OpMode(Uint16 ID)
{
	Uint32 auxH, auxL;

	if(HRADCs_Info.enable_Sampling)
	{
		return;
	}

	// Stop DMA controller to prevent DMA access to McBSP
	// during configuration
	stop_DMA();

	Init_SPIMaster_McBSP(SPI_15MHz);
    InitMcbspa20bit();

	// Set appropriate Chip-Select and CONFIG signals
	HRADC_CS_CLEAR;
	HRADC_CS_SET(ID);
	HRADC_CONFIG_SET;

	// Transmit command via SPI (UFM mode)
	McbspaRegs.DXR2.all = 0x0000;
	McbspaRegs.DXR1.all = 0x0002;	// OpMode: UFM

	// Wait for end of transmission/reception
	while(!McbspaRegs.SPCR1.bit.RRDY){}

	// Store previous HRADC configuration and status
	auxH = (Uint32) McbspaRegs.DRR2.all << 16;
	auxL = (Uint32) McbspaRegs.DRR1.all;
	HRADCs_Info.HRADC_boards[ID].StatusReg = auxH + auxL;

	// Clear Chip-Select and CONFIG signals
	HRADC_CONFIG_CLEAR;
	HRADC_CS_CLEAR;

	// Configure SPI to UFM access settings
	Init_SPIMaster_McBSP_HRADC_UFM();
	InitMcbspa8bit();

	// Transmit UFM opcode for Write Status Register command
	HRADC_CS_SET(ID);
	McbspaRegs.DXR1.all = UFM_OPCODE_WRDI;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	auxH = McbspaRegs.DRR1.all;
	HRADC_CS_RESET(ID);

	// Transmit UFM opcode for Write Status Register command
	McbspaRegs.DXR1.all = UFM_OPCODE_WRSR;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	auxH = McbspaRegs.DRR1.all;

	McbspaRegs.DXR1.all = 0x0000;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	auxH = McbspaRegs.DRR1.all;
	HRADC_CS_RESET(ID);

	auxH = 1;

	while(auxH)		// While UFM is busy, write is enable or protection bits activated
	{
		// Transmit UFM opcode for Read Status Register command
		McbspaRegs.DXR1.all = UFM_OPCODE_RDSR;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		auxH = McbspaRegs.DRR1.all;

		McbspaRegs.DXR1.all = 0x0000;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		auxH = (McbspaRegs.DRR1.all) & 0x000F;

		McbspaRegs.DXR1.all = 0x0000;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		auxL = McbspaRegs.DRR1.all;

		// Reset Chip-Select signals
		HRADC_CS_RESET(ID);
	}

	// Clear Chip-Select signals
	HRADC_CS_CLEAR;
}

void Erase_HRADC_UFM(Uint16 ID)
{
if(HRADCs_Info.enable_Sampling)
	{
		return;
	}

	static Uint16 aux, aux2;

	// Set appropriate Chip-Select signals
	HRADC_CS_SET(ID);

	// Transmit UFM opcode for UFM-ERASE command
	McbspaRegs.DXR1.all = UFM_OPCODE_SECTOR_ERASE;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	aux = McbspaRegs.DRR1.all;

	McbspaRegs.DXR1.all = 0x0000;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	aux = McbspaRegs.DRR1.all;

	McbspaRegs.DXR1.all = 0x0000;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	aux = McbspaRegs.DRR1.all;

	// Reset and Clear Chip-Select signals
	HRADC_CS_RESET(ID);

	aux = 1;

	while(aux)
	{
		// Transmit UFM opcode for Read Status Register command
		McbspaRegs.DXR1.all = UFM_OPCODE_RDSR;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		aux = McbspaRegs.DRR1.all;

		McbspaRegs.DXR1.all = 0x00;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		aux = (McbspaRegs.DRR1.all) & 0x01;

		McbspaRegs.DXR1.all = 0x00;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		aux2 = McbspaRegs.DRR1.all;

		// Reset and Clear Chip-Select signals
		HRADC_CS_RESET(ID);
	}

	// Transmit UFM opcode for UFM-ERASE command
	McbspaRegs.DXR1.all = UFM_OPCODE_SECTOR_ERASE;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	aux = McbspaRegs.DRR1.all;

	McbspaRegs.DXR1.all = 0x0001;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	aux = McbspaRegs.DRR1.all;

	McbspaRegs.DXR1.all = 0x0000;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	aux = McbspaRegs.DRR1.all;

	// Reset and Clear Chip-Select signals
	HRADC_CS_RESET(ID);

	aux = 1;

	while(aux)
	{
		// Transmit UFM opcode for Read Status Register command
		McbspaRegs.DXR1.all = UFM_OPCODE_RDSR;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		aux = McbspaRegs.DRR1.all;

		McbspaRegs.DXR1.all = 0x00;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		aux = (McbspaRegs.DRR1.all) & 0x01;

		McbspaRegs.DXR1.all = 0x00;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		aux2 = McbspaRegs.DRR1.all;

		// Reset and Clear Chip-Select signals
		HRADC_CS_RESET(ID);
	}


	// Clear Chip-Select signals
	HRADC_CS_CLEAR;
}

void Read_HRADC_UFM(Uint16 ID, Uint16 ufm_address, Uint16 n_words, volatile Uint16 *ufm_buffer)
{
	Uint16 dummy, n;

	if(HRADCs_Info.enable_Sampling)
	{
		return;
	}

	dummy = 0;
	n = 0;

	// Set appropriate Chip-Select signals
	HRADC_CS_SET(ID);

	// Transmit UFM opcode for READ command
	McbspaRegs.DXR1.all = UFM_OPCODE_READ;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	dummy = McbspaRegs.DRR1.all;

	// Transmit UFM data address (Extended Mode: Address size = 16 bits)
	McbspaRegs.DXR1.all = (ufm_address >> 8) & 0x00FF;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	dummy = McbspaRegs.DRR1.all;

	McbspaRegs.DXR1.all = ufm_address & 0x00FF;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	dummy = McbspaRegs.DRR1.all;

	// Iterates to receive n_words
	while(n++ < n_words)
	{
		// Receive current word, transmiting two dummy bytes (Extended Mode: Word size = 16 bits)
		McbspaRegs.DXR1.all = 0x0000;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		*(ufm_buffer) = (McbspaRegs.DRR1.all << 8) & 0xFF00;

		McbspaRegs.DXR1.all = 0x0000;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		*(ufm_buffer++) |= McbspaRegs.DRR1.all;
	}

	// Reset and Clear Chip-Select signals
	HRADC_CS_RESET(ID);
	HRADC_CS_CLEAR;
}

void Write_HRADC_UFM(Uint16 ID, Uint16 ufm_address, Uint16 data)
{
	Uint16 status, dummy;

	if(HRADCs_Info.enable_Sampling)
	{
		return;
	}

	status = 0;
	dummy = 0;

	// Set appropriate Chip-Select signals
	HRADC_CS_SET(ID);

	// Transmit UFM opcode for Write Status Register command
	McbspaRegs.DXR1.all = UFM_OPCODE_WREN;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	dummy = McbspaRegs.DRR1.all;
	HRADC_CS_RESET(ID);

	status = 1;

	while(status != 0x0002)
	{
		// Transmit UFM opcode for Read Status Register command
		McbspaRegs.DXR1.all = UFM_OPCODE_RDSR;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		status = McbspaRegs.DRR1.all;

		McbspaRegs.DXR1.all = 0x00;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		status = (McbspaRegs.DRR1.all) & 0x000F;

		McbspaRegs.DXR1.all = 0x00;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		dummy = McbspaRegs.DRR1.all;

		// Reset Chip-Select signals
		HRADC_CS_RESET(ID);
	}

	// Transmit UFM opcode for WRITE command
	McbspaRegs.DXR1.all = UFM_OPCODE_WRITE;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	status = McbspaRegs.DRR1.all;

	// Transmit UFM data address (Extended Mode: Address size = 16 bits)
	McbspaRegs.DXR1.all = (ufm_address >> 8) & 0x00FF;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	status = McbspaRegs.DRR1.all;

	McbspaRegs.DXR1.all = ufm_address & 0x00FF;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	status = McbspaRegs.DRR1.all;

	// Transmit new data
	McbspaRegs.DXR1.all = (data >> 8) & 0x00FF;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	status = McbspaRegs.DRR1.all;

	McbspaRegs.DXR1.all = data & 0x00FF;
	while(!McbspaRegs.SPCR1.bit.RRDY){}
	status = McbspaRegs.DRR1.all;

	// Reset Chip-Select signals
	HRADC_CS_RESET(ID);

	status = 1;

	while(status)
	{
		// Transmit UFM opcode for Read Status Register command
		McbspaRegs.DXR1.all = UFM_OPCODE_RDSR;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		status = McbspaRegs.DRR1.all;

		McbspaRegs.DXR1.all = 0x00;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		status = (McbspaRegs.DRR1.all) & 0x0001;

		McbspaRegs.DXR1.all = 0x00;
		while(!McbspaRegs.SPCR1.bit.RRDY){}
		dummy = McbspaRegs.DRR1.all;

		// Reset Chip-Select signals
		HRADC_CS_RESET(ID);
	}

	// Clear Chip-Select signals
	HRADC_CS_CLEAR;
}

void Read_HRADC_BoardData(HRADC_struct *hradcPtr)
{
	Read_HRADC_UFM(hradcPtr->ID, UFM_BOARDDATA_ADDRESS, UFM_BOARDDATA_SIZE, hradcPtr->BoardData.u);
}
