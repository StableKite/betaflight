/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "canard.h"
#include "dronecan_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UAVCAN_ESC_STATUS_PAYLOAD_BYTES 14U

typedef struct dronecanEscStatus_s {
    uint32_t errorCount;
    float voltage;
    float current;
    float temperatureKelvin;
    int32_t rpm;
    uint8_t powerPct;
    uint8_t index;
} dronecanEscStatus_t;

int16_t dronecanEscBroadcastRaw(CanardInstance *ins, const int16_t *commands,
    uint8_t count, uint8_t *transferId);
bool dronecanEscDecodeStatus(const CanardRxTransfer *transfer, dronecanEscStatus_t *status);

#ifdef __cplusplus
}
#endif
