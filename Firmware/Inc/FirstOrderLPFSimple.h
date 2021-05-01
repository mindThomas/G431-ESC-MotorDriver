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
 
#ifndef MISC_FIRSTORDERLPF_H
#define MISC_FIRSTORDERLPF_H

typedef struct FirstOrderLPF
{
	float Ts;       // Sampling Time
	float tau;            // Filter time constant

	float coeff_b;  // IIR filter coefficient (nominator polynomial)
	float coeff_a;  // IIR filter coefficient (denominator polynomial)

	float lpfOld;   // Holds previous sample output value
	float inputOld; // Holds previous sample output value
} FirstOrderLPF;


void FirstOrderLPF_Init(FirstOrderLPF * lpf, float Ts, float tau);
float FirstOrderLPF_Filter(FirstOrderLPF * lpf, float input);
void FirstOrderLPF_Reset(FirstOrderLPF * lpf);
void FirstOrderLPF_ChangeTimeconstant(FirstOrderLPF * lpf, float tau);
	
#endif
