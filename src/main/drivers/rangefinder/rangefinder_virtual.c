/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include "platform.h"

#ifdef USE_VIRTUAL_RANGEFINDER
#include "common/utils.h"
#include "drivers/rangefinder/rangefinder_virtual.h"

static int32_t pendingDistanceCm = RANGEFINDER_NO_NEW_DATA;
static bool enabled;

void virtualRangefinderSetEnabled(bool value)
{
    enabled = value;
    pendingDistanceCm = RANGEFINDER_NO_NEW_DATA;
}

static void virtualRangefinderInit(rangefinderDev_t *dev)
{
    UNUSED(dev);
    pendingDistanceCm = RANGEFINDER_NO_NEW_DATA;
}

static void virtualRangefinderUpdate(rangefinderDev_t *dev)
{
    UNUSED(dev);
}

static int32_t virtualRangefinderRead(rangefinderDev_t *dev)
{
    UNUSED(dev);
    const int32_t distanceCm = pendingDistanceCm;
    pendingDistanceCm = RANGEFINDER_NO_NEW_DATA;
    return distanceCm;
}

bool virtualRangefinderSet(int32_t distanceCm)
{
    if (!enabled || distanceCm < RANGEFINDER_HARDWARE_FAILURE || distanceCm > INT16_MAX) {
        return false;
    }
    pendingDistanceCm = distanceCm;
    return true;
}

bool virtualRangefinderDetect(rangefinderDev_t *dev)
{
    if (!enabled || !dev) {
        return false;
    }
    // Generic board ADC limits, not characteristics of a physical sensor.
    dev->delayMs = 10;
    dev->maxRangeCm = INT16_MAX;
    dev->detectionConeDeciDegrees = 1800;
    dev->detectionConeExtendedDeciDegrees = 1800;
    dev->init = virtualRangefinderInit;
    dev->update = virtualRangefinderUpdate;
    dev->read = virtualRangefinderRead;
    return true;
}
#endif
