/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "platform.h"

#ifdef USE_VIRTUAL_OPTICALFLOW
#include "common/utils.h"
#include "drivers/time.h"
#include "drivers/opticalflow/opticalflow_virtual.h"

static opticalflowData_t sample;
static bool enabled;

void virtualOpticalflowSetEnabled(bool value)
{
    enabled = value;
    sample = (opticalflowData_t){ .quality = OPTICALFLOW_NO_NEW_DATA };
}

static bool finiteFloat(float value)
{
    // Preserve rejection of NaN/Inf even in a firmware fast-math build.
    STATIC_ASSERT(sizeof(float) == sizeof(uint32_t), float_size);
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static void virtualOpticalflowInit(opticalflowDev_t *dev)
{
    UNUSED(dev);
    sample = (opticalflowData_t){ .quality = OPTICALFLOW_NO_NEW_DATA };
}

static void virtualOpticalflowUpdate(opticalflowDev_t *dev)
{
    UNUSED(dev);
}

static void virtualOpticalflowRead(opticalflowDev_t *dev, opticalflowData_t *result)
{
    UNUSED(dev);
    *result = sample;
}

bool virtualOpticalflowSet(float xRadS, float yRadS, int16_t quality, uint32_t integrationTimeUs, bool rangeValid)
{
    if (!enabled || !finiteFloat(xRadS) || !finiteFloat(yRadS) || quality < 0 || quality > 100 || !integrationTimeUs) {
        return false;
    }
    sample = (opticalflowData_t){
        .timeStampUs = micros(),
        .quality = quality,
        .flowRate = {{xRadS, yRadS}},
        .rangeValid = rangeValid,
        .flowValid = true,
        .flowDtValid = true,
    };
    return true;
}

bool virtualOpticalflowDetect(opticalflowDev_t *dev)
{
    if (!enabled || !dev) {
        return false;
    }
    dev->delayMs = 10;
    dev->minRangeCm = 1;
    dev->minQualityThreshold = 0;
    // No sensor-specific bus latency is imposed by a virtual board ADC.
    dev->gyroSampleDelay = 1;
    dev->init = virtualOpticalflowInit;
    dev->update = virtualOpticalflowUpdate;
    dev->read = virtualOpticalflowRead;
    return true;
}
#endif
