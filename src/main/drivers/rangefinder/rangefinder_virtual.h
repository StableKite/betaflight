/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "drivers/rangefinder/rangefinder.h"

// Decoded centimetres, not a UART/MSP packet or an estimated altitude.
// Call from the same firmware owner as the sensor tasks.
void virtualRangefinderSetEnabled(bool enabled);
bool virtualRangefinderSet(int32_t distanceCm);
bool virtualRangefinderDetect(rangefinderDev_t *dev);
