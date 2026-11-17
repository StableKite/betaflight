/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "drivers/opticalflow/opticalflow.h"

// Decoded angular rates (rad/s), quality (0..100) and integration period.
// No rotation, gyro subtraction, filtering or navigation occurs here.
// Call from the same firmware owner as the sensor tasks.
void virtualOpticalflowSetEnabled(bool enabled);
bool virtualOpticalflowSet(float xRadS, float yRadS, int16_t quality, uint32_t integrationTimeUs, bool rangeValid);
bool virtualOpticalflowDetect(opticalflowDev_t *dev);
