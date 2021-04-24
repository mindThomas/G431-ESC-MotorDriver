/* Copyright (C) 2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#ifndef LIBRARIES_MOTORDRIVER_H
#define LIBRARIES_MOTORDRIVER_H

#include "SyncedPWMADC.h"
#include "Matrix.hpp"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <math.h>

class MotorDriver : SyncedPWMADC
{

private:


public:
	MotorDriver(uint32_t frequency = 10000, float maxDuty = 0.98);
	~MotorDriver();

};

#endif
