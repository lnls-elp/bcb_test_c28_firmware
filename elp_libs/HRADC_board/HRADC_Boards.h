/*
 * 		FILE: 		HRADC_Boards.h
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	07/23/2015
 * 		MODIFIED:	07/23/2015
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code for access and control of HRADC boards
 *
 *		TODO: Generalize HRADC_FREQ_SAMP calculation for interleaving
 */

#ifndef HRADC_BOARDS_H
#define HRADC_BOARDS_H

#define N_MAX_HRADC			4
#define CHECK_STATUS		0x01	// HCP Command for check HRADC board status
#define HEATER_DISABLE 		0
#define HEATER_ENABLE 		1
#define RAILS_DISABLE 		0
#define RAILS_ENABLE 		1

#define TIMEOUT_uS_HRADC_CONFIG 		10000

#define UFM_OPCODE_WREN			0x0006
#define UFM_OPCODE_WRDI			0x0004
#define UFM_OPCODE_RDSR			0x0005
#define UFM_OPCODE_WRSR			0x0001
#define UFM_OPCODE_READ			0x0003
#define UFM_OPCODE_WRITE		0x0002
#define UFM_OPCODE_SECTOR_ERASE	0x0020
#define UFM_OPCODE_UFM_ERASE	0x0060

#define UFM_BOARDDATA_SIZE		28
#define UFM_BOARDDATA_ADDRESS	0x0000

#define HRADC_VIN_BI_P_GAIN		(20.0/262144.0)
#define HRADC_BI_OFFSET			131072.0



/**********************************************************************************************/
//
// 	Enumerate definition for configurable analog inputs
//
typedef enum {
		Vin_bipolar,
		Vin_unipolar_p,
		Vin_unipolar_n,
		Iin_bipolar,
		Iin_unipolar_p,
		Iin_unipolar_n,
		Vref_bipolar_p,
		Vref_bipolar_n,
		GND,
		Vref_unipolar_p,
		Vref_unipolar_n,
		GND_unipolar,
		Temp,
		Reserved0,
		Reserved1,
		Reserved2
} eInputType;

typedef enum {
		HRADC_Sampling,
		HRADC_UFM
} eHRADCOpMode;

typedef enum {
	HRADC_FBP,
	HRADC_FAx
} eHRADCVar;

#include "DSP28x_Project.h"
#include "pwm/pwm.h"
#include "McBSP_SPI_Interface.h"
#include "DMA_SPI_Interface.h"

/**********************************************************************************************/
//
// HRADC board selection macros
//

#if (UDC_V2_0)
	#define GPE_PORT_BITS_HRADC_CS	{0x0, 0x1, 0x2, 0x3}
	#define HRADC_CS_CLEAR			GpioDataRegs.GPECLEAR.all = 0x3;
#endif

#if (UDC_V2_1)
	#define GPE_PORT_BITS_HRADC_CS	{0x0, 0x4, 0x2, 0x6}
	#define HRADC_CS_CLEAR			GpioDataRegs.GPECLEAR.all = 0x6;
#endif

#define HRADC_CONFIG_SET		GpioDataRegs.GPESET.bit.GPIO131 = 1;
#define HRADC_CONFIG_CLEAR		GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

#define HRADC_CS_SET(id) 		GpioDataRegs.GPESET.all = HRADC_BoardSelector[id];
//#define HRADC_CS_SET(id) 		GpioDataRegs.GPESET.all = id;

#define HRADC_CS_RESET(id)		DELAY_US(1); \
								HRADC_CS_CLEAR; \
								HRADC_CS_SET((~id) & 3); \
								DELAY_US(1); \
								HRADC_CS_CLEAR; \
								HRADC_CS_SET(id); \
								DELAY_US(1);

/**********************************************************************************************/
//
// 	HRADC Board Data from UFM
//
typedef volatile struct
{
	Uint64 		SerialNumber;			// Unique identification
	Uint16	 	CalibDate[5]; 			// Date of last calibration [DD/MM/AAAA/hh/mm]
	eHRADCVar	Variant;				// Board variant (FBP/FAx/...)
	float		Rburden;				// Nominal value of burden resistor
	float 		CalibTemp;	  			// Ambient temperature during last calibration
	float		gain_Vin_bipolar;		// Calibration gain		Vin bipolar
	float		offset_Vin_bipolar;		// Calibration offset	Vin bipolar
	float		gain_Iin_bipolar;		// Calibration gain		Iin bipolar
	float		offset_Iin_bipolar;		// Calibration offset	Iin bipolar
	float		Vref_bipolar_p;			// + Voltage reference 	Bipolar
	float		Vref_bipolar_n;			// - Voltage reference 	Bipolar
	float		GND_bipolar;			// GND					Bipolar
} tHRADC_BoardData;

typedef volatile union
{
	tHRADC_BoardData	t;
	Uint16 				u[UFM_BOARDDATA_SIZE];
} uHRADC_BoardData;


/**********************************************************************************************/
//
// 	HRADC Board Information
//
typedef volatile struct
{
	Uint16 				ID;						// Backplane position
	eHRADCOpMode		OpMode;					// Operation mode;
	eInputType			AnalogInput;			// Current analog input
	Uint16 				enable_Heater;			// Current temperature controller status
	Uint16 				enable_RailsMonitor;	// Rails monitor status
	Uint16				size_SamplesBuffer;		// Size of samples buffer
	Uint16				index_SamplesBuffer;	// Current buffer pointer position
	Uint16              UFMData;                // Auxiliary byte for UFM reads
	Uint32				StatusReg;				// Configuration/Status Register
	volatile Uint32		*SamplesBuffer;			// Pointer to samples buffer
	volatile float		gain;					// Pointer to gain value of current analog input
	volatile float		offset;				// Pointer to offset value of current analog input
	uHRADC_BoardData	BoardData;				// Calibration database
} HRADC_struct;

typedef volatile struct
{
	float			freq_Sampling;
	Uint16 			enable_Sampling;
	Uint16 			n_HRADC_boards;
	HRADC_struct 	HRADC_boards[4];
} HRADCs_struct;


/**********************************************************************************************/
//
// 	Prototype statements for variables and functions found in source code HRADC_Boards.c
//
extern volatile HRADCs_struct HRADCs_Info;
/*extern volatile HRADC_struct  HRADC0_board;
extern volatile HRADC_struct  HRADC1_board;
extern volatile HRADC_struct  HRADC2_board;
extern volatile HRADC_struct  HRADC3_board;*/

extern volatile Uint32 HRADC_BoardSelector[4];

extern volatile Uint32 counterErrorSendCommand;
extern volatile float AverageFilter;

extern void Init_HRADC_Info(volatile HRADC_struct *hradcPtr, Uint16 ID, Uint16 buffer_size, volatile Uint32 *buffer, float transducer_gain);
extern void Config_HRADC_board(volatile HRADC_struct *hradcPtr, eInputType AnalogInput, Uint16 enHeater, Uint16 enRails);
extern Uint16 Try_Config_HRADC_board(volatile HRADC_struct *hradcPtr, eInputType AnalogInput, Uint16 enHeater, Uint16 enRails);

extern void SendCommand_HRADC(volatile HRADC_struct *hradcPtr, Uint16 command);
extern Uint16 CheckStatus_HRADC(volatile HRADC_struct *hradcPtr);

extern void Config_HRADC_SoC(float freq);
extern void Enable_HRADC_Sampling(void);
extern void Disable_HRADC_Sampling(void);

extern void Config_HRADC_Sampling_OpMode(Uint16 ID);
extern void Config_HRADC_UFM_OpMode(Uint16 ID);
extern void Erase_HRADC_UFM(Uint16 ID);
extern void Read_HRADC_UFM(Uint16 ID, Uint16 ufm_address, Uint16 n_words, volatile Uint16 *ufm_buffer);
extern void Write_HRADC_UFM(Uint16 ID, Uint16 ufm_address, Uint16 data);

extern void Read_HRADC_BoardData(HRADC_struct *hradcPtr);

#endif
