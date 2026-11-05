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

#include "dronecan_esc_protocol.h"

int16_t dronecanEscBroadcastRaw(CanardInstance *ins, const int16_t *commands,
    uint8_t count, uint8_t *transferId)
{
    if (count == 0 || count > UAVCAN_ESC_RAWCOMMAND_MAX_MOTORS) {
        return -CANARD_ERROR_INVALID_ARGUMENT;
    }

    // Tail array optimization omits the length prefix on classic CAN.
    uint8_t payload[(UAVCAN_ESC_RAWCOMMAND_MAX_MOTORS * UAVCAN_ESC_RAWCOMMAND_BITS + 7U) / 8U] = { 0 };
    uint32_t bitOffset = 0;
    for (uint8_t i = 0; i < count; i++) {
        int16_t value = commands[i];
        if (value > UAVCAN_ESC_RAWCOMMAND_MAX) {
            value = UAVCAN_ESC_RAWCOMMAND_MAX;
        } else if (value < -UAVCAN_ESC_RAWCOMMAND_MAX - 1) {
            value = -UAVCAN_ESC_RAWCOMMAND_MAX - 1;
        }
        canardEncodeScalar(payload, bitOffset, UAVCAN_ESC_RAWCOMMAND_BITS, &value);
        bitOffset += UAVCAN_ESC_RAWCOMMAND_BITS;
    }

    CanardTxTransfer tx;
    canardInitTxTransfer(&tx);
    tx.transfer_type       = CanardTransferTypeBroadcast;
    tx.data_type_signature = UAVCAN_ESC_RAWCOMMAND_SIGNATURE;
    tx.data_type_id        = UAVCAN_ESC_RAWCOMMAND_ID;
    tx.inout_transfer_id   = transferId;
    tx.priority            = CANARD_TRANSFER_PRIORITY_HIGH;
    tx.payload             = payload;
    tx.payload_len         = (uint16_t)((bitOffset + 7U) / 8U);

    return canardBroadcastObj(ins, &tx);
}

bool dronecanEscDecodeStatus(const CanardRxTransfer *transfer, dronecanEscStatus_t *status)
{
    uint16_t voltage;
    uint16_t current;
    uint16_t temperature;

    if (transfer->payload_len != UAVCAN_ESC_STATUS_PAYLOAD_BYTES
        || canardDecodeScalar(transfer, ESC_STATUS_OFFSET_ERROR_COUNT, 32, false, &status->errorCount) != 32
        || canardDecodeScalar(transfer, ESC_STATUS_OFFSET_VOLTAGE, 16, false, &voltage) != 16
        || canardDecodeScalar(transfer, ESC_STATUS_OFFSET_CURRENT, 16, false, &current) != 16
        || canardDecodeScalar(transfer, ESC_STATUS_OFFSET_TEMPERATURE, 16, false, &temperature) != 16
        || canardDecodeScalar(transfer, ESC_STATUS_OFFSET_RPM, 18, true, &status->rpm) != 18
        || canardDecodeScalar(transfer, ESC_STATUS_OFFSET_POWER_PCT, 7, false, &status->powerPct) != 7
        || canardDecodeScalar(transfer, ESC_STATUS_OFFSET_ESC_INDEX, 5, false, &status->index) != 5) {
        return false;
    }

    status->voltage = canardConvertFloat16ToNativeFloat(voltage);
    status->current = canardConvertFloat16ToNativeFloat(current);
    status->temperatureKelvin = canardConvertFloat16ToNativeFloat(temperature);
    return true;
}
