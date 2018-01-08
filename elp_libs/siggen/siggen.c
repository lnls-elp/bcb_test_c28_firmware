/*
 * 		FILE: 		SigGen.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	03/04/2016
 * 		MODIFIED:	03/04/2016
 *
 * 		AUTHOR: 	Gabriel O. Brunheira  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code for Signal Generator module.
 *
 *		This library created by LNLS/ELP group implements a Digital Signal Generator,
 *		which is able to calculate iteratively (using a function call) digital samples
 *		of a specified periodic waveform.
 *
 *		These modules are intended to be used within a Digital Power Framework.
 *
 *		TODO:	- Atualizar Aux no Update;
 *				- Implementar Run para Square, Triangle, FreqSweep
 */

#include <math.h>
#include <float.h>
#include "siggen.h"

#define _USE_MATH_DEFINES
#define	PI	3.141592653589793

#pragma CODE_SECTION(Enable_ELP_SigGen, "ramfuncs");
#pragma CODE_SECTION(Disable_ELP_SigGen, "ramfuncs");
#pragma CODE_SECTION(Update_ELP_SigGen, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_SigGen_Sine, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_SigGen_Square, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_SigGen_Triangle, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_SigGen_FreqSweep, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_SigGen_DampedSine, "ramfuncs");

/*
 * Prototype statements for functions found within this file
 */

uint16_t init_siggen(siggen_t *p_siggen, float freq_sampling,
                            volatile float *out);
void Enable_ELP_SigGen(tELP_SigGen *ptr_sg);
void Disable_ELP_SigGen(tELP_SigGen *ptr_sg);
void Reset_ELP_SigGen(tELP_SigGen *ptr_sg);
void Update_ELP_SigGen(tELP_SigGen *ptr_sg);
void Run_ELP_SigGen_Sine(tELP_SigGen *ptr_sg);
void Run_ELP_SigGen_Square(tELP_SigGen *ptr_sg);
void Run_ELP_SigGen_Triangle(tELP_SigGen *ptr_sg);
void Run_ELP_SigGen_FreqSweep(tELP_SigGen *ptr_sg);
void Run_ELP_SigGen_DampedSine(tELP_SigGen *ptr_sg);
void Run_ELP_SigGen_Trapezoidal(tELP_SigGen *ptr_sg);

tELP_SigGen SignalGenerator;

/*
 * Initialization of Signal Generator module
 */

void init_siggen(siggen_t *p_siggen, float freq_sampling, volatile float *out)
{
    uint16_t i;

	DINT;
	if(p_siggen->enable == 0)
	{
	    p_siggen->freq_sampling = freq_sampling;
        p_siggen->out = out;
        p_siggen->run_siggen = &run_siggen_sine();

        scale_siggen(p_siggen, 1.0, 0.0);
        reset_siggen(p_siggen);

        cfg_siggen(p_siggen,Sine,0,0.0,);

        p_siggen->type = Sine;
        p_siggen->num_cycles = 0;
        p_siggen->num_samples = 0.0;
        p_siggen->freq = 0.0;

        for(i = 0; i < NUM_SIGGEN_AUX_PARAM; i++)
        {
            p_siggen->aux_param[0] = 0.0;
        }

        for(i = 0; i < NUM_SIGGEN_AUX_VAR; i++)
        {
            p_siggen->aux_var[0] = 0.0;
        }

        disable_siggen(p_siggen);
	}

    EINT;
}

void Enable_ELP_SigGen(tELP_SigGen *ptr_sg)
{
	if(!ptr_sg->Enable)
	{
		Reset_ELP_SigGen(ptr_sg);
		switch(ptr_sg->Type)
		{
			case Sine:
			case Square:
			case Triangle:
				Update_ELP_SigGen(ptr_sg);
				break;

			default:
				break;
		}
		ptr_sg->Enable = 1;
	}
}

void Disable_ELP_SigGen(tELP_SigGen *ptr_sg)
{
	ptr_sg->Enable = 0;
}

/*
 * Reset Signal Generator module
 */

void Reset_ELP_SigGen(tELP_SigGen *ptr_sg)
{
	ptr_sg->n = 0.0;
	*(ptr_sg->out) = 0.0;
}

/*
 * Run Signal Generator module
 */

void Run_ELP_SigGen_Sine(tELP_SigGen *ptr_sg)
{
	if(ptr_sg->Enable)
	{
		*(ptr_sg->out) = (*ptr_sg->ptr_Amp) * sin( ptr_sg->w * ptr_sg->n++ + ptr_sg->PhaseStart) + (*ptr_sg->ptr_Offset);

		if(ptr_sg->nSamples > 0)
		{
			if(ptr_sg->n >= ptr_sg->nSamples)
			{
				Disable_ELP_SigGen(ptr_sg);
			}

		}
		else if(ptr_sg->n >= ptr_sg->FreqSample)
		{
			/*
			 *  Compara��o � feita com fs, para que n seja incrementado at� completar 1 segundo.
			 *  Isso foi feito, pois ao comparar n com n_samp, distorcemos o sinal gerado, j� que
			 *  n_samp pode ser float, e n n�o.
			 *
			 *  Assim que o per�odo se completa e os par�metros do gerador s�o atualizados, de
			 *  forma a garantir uma transi��o suave entre a sen�ide gerada anteriormente e a nova
			 */
			Update_ELP_SigGen(ptr_sg);
			ptr_sg->n = 0.0;
		}
	}
}

void Run_ELP_SigGen_Square(tELP_SigGen *ptr_sg)
{

	*(ptr_sg->out) = 0.0;
}

void Run_ELP_SigGen_Triangle(tELP_SigGen *ptr_sg)
{
	*(ptr_sg->out) = 0.0;
}

void Run_ELP_SigGen_FreqSweep(tELP_SigGen *ptr_sg)
{
	*(ptr_sg->out) = 0.0;
}

void Run_ELP_SigGen_DampedSine(tELP_SigGen *ptr_sg)
{
	if(ptr_sg->Enable)
	{
		if(ptr_sg->n < ptr_sg->nSamples)
		{
			*(ptr_sg->out) = (*ptr_sg->ptr_Amp) * exp(ptr_sg->Aux * ptr_sg->n) * sin( ptr_sg->w * ptr_sg->n + ptr_sg->PhaseStart) + (*ptr_sg->ptr_Offset);
			ptr_sg->n++;
		}
		else
		{
			Disable_ELP_SigGen(ptr_sg);
		}
	}
}

void Run_ELP_SigGen_Trapezoidal(tELP_SigGen *ptr_sg)
{
	static Uint16 nCycle = 0;

	if(ptr_sg->Enable)
	{
		if(nCycle < ptr_sg->nCycles)
		{
			if(ptr_sg->n < ptr_sg->PhaseStart)
			{
				*(ptr_sg->out) = ptr_sg->n * ptr_sg->w + (*ptr_sg->ptr_Offset);
			}
			else if(ptr_sg->n < ptr_sg->PhaseEnd)
			{
				*(ptr_sg->out) = (*ptr_sg->ptr_Amp) + (*ptr_sg->ptr_Offset);
			}
			else if(ptr_sg->n < ptr_sg->nSamples)
			{
				*(ptr_sg->out) = ptr_sg->Aux * (ptr_sg->PhaseEnd - ptr_sg->n) + (*ptr_sg->ptr_Amp) + (*ptr_sg->ptr_Offset);
			}
			else
			{
				*(ptr_sg->out) = (*ptr_sg->ptr_Offset);
				nCycle++;
				ptr_sg->n = 0.0;
			}
			ptr_sg->n++;
		}
		else
		{
			Disable_ELP_SigGen(ptr_sg);
			nCycle = 0;
		}
	}
}

void Update_ELP_SigGen(tELP_SigGen *ptr_sg)
{
	//ptr_sg->nSamples = ptr_sg->FreqSample / (*ptr_sg->ptr_FreqSignal);
	//ptr_sg->w = 2.0 * PI / ptr_sg->nSamples;
	ptr_sg->w = 2.0 * PI * (*ptr_sg->ptr_FreqSignal) / ptr_sg->FreqSample;
}
