/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#include "FirstOrderLPFSimple.h"
 
void FirstOrderLPF_Init(FirstOrderLPF * lpf, float Ts, float tau)
{
	lpf->Ts = Ts;
	lpf->tau = tau;

	// Calculate filter coefficients for a  of First order Low-pass filter using the Tustin (Bilinear) transform (however without frequency warping)
	lpf->coeff_b = 1/(2*tau/Ts + 1); // nominator
	lpf->coeff_a = 1/(2*tau/Ts + 1) - 2/(2 + Ts/tau); // denominator

	lpf->inputOld = 0;
	lpf->lpfOld = 0;
}

// Filter a given input using the first order LPF
float FirstOrderLPF_Filter(FirstOrderLPF * lpf, float input)
{	
	float out = lpf->coeff_b * input + lpf->coeff_b * lpf->inputOld - lpf->coeff_a * lpf->lpfOld; // IIR difference equation implementation
	lpf->lpfOld = out;
	lpf->inputOld = input;
	return out;
}

void FirstOrderLPF_Reset(FirstOrderLPF * lpf)
{
	lpf->inputOld = 0;
	lpf->lpfOld = 0;
}

void FirstOrderLPF_ChangeTimeconstant(FirstOrderLPF * lpf, float tau)
{
	lpf->tau = tau;
	lpf->coeff_b = 1/(2*tau/lpf->Ts + 1); // nominator
	lpf->coeff_a = 1/(2*tau/lpf->Ts + 1) - 2/(2 + lpf->Ts/tau); // denominator
}
