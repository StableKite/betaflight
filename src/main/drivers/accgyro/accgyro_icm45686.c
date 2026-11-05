/*
 * This file is part of Betaflight.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <string.h>

#include "accgyro_icm45686.h"
#include "accgyro_icm456xx_registers.h"

// FIFO stream rules follow TDK AN-000364 and the ICM45686 reference driver
// 0a5bed6975b5cdeecb98f95db420f3058dd88547.
enum {
    FIFO_COUNT = 0x12,
    FIFO_DATA = 0x14,
    FIFO_CONFIG0 = 0x1d,
    FIFO_CONFIG3 = 0x20,
    FIFO_CONFIG4 = 0x21,
    TIMESTAMP_CONFIG = 0x23,
    FIFO_DECIMATION = 0x28,
    SMC_CONTROL = 0xa258,
    FIFO_MODE_MASK = 0xc0,
    FIFO_FLUSH = 0x80,
    FIFO_ENABLE = 0x01,
    FIFO_DEPTH = 0x1e,
    FIFO_STREAM_MODE = 0x40,
    FIFO_FULL_COUNT = 240,
    TIMESTAMP_RESOLUTION_MASK = 0x60,
    ACCEL_CONFIG = ICM456XX_ACCEL_FS_SEL_32G | ICM456XX_ACCEL_ODR_6K4_LN,
    GYRO_CONFIG = ICM456XX_GYRO_FS_SEL_4000DPS | ICM456XX_GYRO_ODR_6K4_LN,
    POWER_CONFIG = ICM456XX_ACCEL_MODE_LN | ICM456XX_GYRO_MODE_LN,
};

static bool readReg(const icm45686Io_t *io, uint8_t reg, uint8_t *data, size_t count)
{
    return io->read && io->read(io->context, reg, data, count);
}

static bool writeRegs(const icm45686Io_t *io, uint8_t reg, const uint8_t *data, size_t count)
{
    return io->write && io->write(io->context, reg, data, count);
}

static bool writeReg(const icm45686Io_t *io, uint8_t reg, uint8_t value)
{
    return writeRegs(io, reg, &value, 1);
}

static bool readIndirect(const icm45686Io_t *io, uint16_t reg, uint8_t *value)
{
    const uint8_t address[] = {reg >> 8, reg};
    io->delayUs(4);
    if (!writeRegs(io, ICM456XX_REG_IREG_ADDR_15_8, address, sizeof(address))) {
        return false;
    }
    io->delayUs(4);
    return readReg(io, ICM456XX_REG_IREG_DATA, value, 1);
}

static bool writeIndirect(const icm45686Io_t *io, uint16_t reg, uint8_t value)
{
    const uint8_t data[] = {reg >> 8, reg, value};
    io->delayUs(4);
    const bool ok = writeRegs(io, ICM456XX_REG_IREG_ADDR_15_8, data, sizeof(data));
    io->delayUs(4);
    return ok;
}

bool icm45686InitDirect(const icm45686Io_t *io, bool retryIdentification)
{
    uint8_t id;
    if (!readReg(io, ICM456XX_WHO_AM_REGISTER, &id, 1)) {
        if (!retryIdentification) {
            return false;
        }
        io->delayUs(100);
        if (!readReg(io, ICM456XX_WHO_AM_REGISTER, &id, 1)) {
            return false;
        }
    }
    if (id != ICM45686_WHO_AM_I_CONST || !writeReg(io, ICM456XX_REG_MISC2, ICM456XX_SOFT_RESET)) {
        return false;
    }
    io->delayUs(1000);
    uint8_t status;
    if (!readReg(io, ICM456XX_INT1_STATUS0, &status, 1) || !(status & ICM456XX_INT1_STATUS_RESET_DONE) ||
        !writeReg(io, ICM456XX_ACCEL_CONFIG0, ACCEL_CONFIG) ||
        !writeReg(io, ICM456XX_GYRO_CONFIG0, GYRO_CONFIG) ||
        !writeReg(io, ICM456XX_PWR_MGMT0, POWER_CONFIG) ||
        !writeReg(io, ICM456XX_INT1_CONFIG0, ICM456XX_INT1_STATUS_EN_DRDY)) {
        return false;
    }
    io->delayUs(70000); // Datasheet gyro startup time.
    return true;
}

icm45686Result_e icm45686ReadDirect(const icm45686Io_t *io, icm45686DirectSample_t *sample)
{
    uint8_t status;
    if (!readReg(io, ICM456XX_INT1_STATUS0, &status, 1)) {
        return ICM45686_TRANSPORT;
    }
    if (!(status & ICM456XX_INT1_STATUS_DRDY)) {
        return ICM45686_PENDING;
    }
    uint8_t data[14];
    if (!readReg(io, ICM456XX_ACCEL_DATA_X1_UI, data, sizeof(data))) {
        return ICM45686_TRANSPORT;
    }
    for (unsigned axis = 0; axis < 3; ++axis) {
        sample->accel[axis] = icm456xxReadI16(data + 2 * axis, false);
        sample->gyro[axis] = icm456xxReadI16(data + 6 + 2 * axis, false);
    }
    sample->temperature = icm456xxReadI16(data + 12, false);
    return ICM45686_READY;
}

static bool flushFifo(const icm45686Io_t *io, uint8_t config3)
{
    if (!writeReg(io, FIFO_CONFIG3, config3 | FIFO_FLUSH)) {
        return false;
    }
    const uint64_t started = io->clockNs();
    do {
        uint8_t value;
        if (!readReg(io, FIFO_CONFIG3, &value, 1)) {
            return false;
        }
        if (!(value & FIFO_FLUSH)) {
            return true;
        }
        io->delayUs(100);
    } while (io->clockNs() - started < 100000000);
    return false;
}

icm45686Result_e icm45686FifoStop(const icm45686Io_t *io, icm45686FifoDevice_t *device)
{
    device->active = false;
    if (!device->saved) {
        return ICM45686_CLOSED;
    }
    // Attempt every restoration, including after a partially applied start.
    bool ok = writeReg(io, FIFO_CONFIG4, device->savedFifo[4] & ~FIFO_ENABLE);
    ok = writeReg(io, FIFO_CONFIG0, device->savedFifo[0] & ~FIFO_MODE_MASK) && ok;
    ok = flushFifo(io, device->savedFifo[3]) && ok;
    ok = writeIndirect(io, SMC_CONTROL, device->savedSmc) && ok;
    ok = writeReg(io, TIMESTAMP_CONFIG, device->savedTimestamp) && ok;
    ok = writeRegs(io, FIFO_CONFIG0, device->savedFifo, sizeof(device->savedFifo)) && ok;
    ok = writeReg(io, ICM456XX_INT1_CONFIG0, device->savedInterrupt) && ok;
    uint8_t actual[6], timestamp, smc, interrupt;
    ok = readReg(io, FIFO_CONFIG0, actual, sizeof(actual)) && ok;
    ok = readReg(io, TIMESTAMP_CONFIG, &timestamp, 1) && ok;
    ok = readIndirect(io, SMC_CONTROL, &smc) && ok;
    ok = readReg(io, ICM456XX_INT1_CONFIG0, &interrupt, 1) && ok;
    if (ok && memcmp(actual, device->savedFifo, sizeof(actual)) == 0 &&
        timestamp == device->savedTimestamp && smc == device->savedSmc && interrupt == device->savedInterrupt) {
        device->saved = false;
        return ICM45686_CLOSED;
    }
    return ICM45686_RESTORE;
}

icm45686Result_e icm45686FifoStart(const icm45686Io_t *io, icm45686FifoDevice_t *device)
{
    if (device->saved || device->active) {
        return ICM45686_INVALID;
    }
    uint8_t id, settings[2], power, decimation, endian;
    if (!readReg(io, ICM456XX_WHO_AM_REGISTER, &id, 1) ||
        !readReg(io, ICM456XX_ACCEL_CONFIG0, settings, sizeof(settings)) ||
        !readReg(io, ICM456XX_PWR_MGMT0, &power, 1) ||
        !readReg(io, FIFO_DECIMATION, &decimation, 1) ||
        !readIndirect(io, ICM456XX_RA_SREG_CTRL, &endian)) {
        return ICM45686_TRANSPORT;
    }
    if (id != ICM45686_WHO_AM_I_CONST || settings[0] != ACCEL_CONFIG || settings[1] != GYRO_CONFIG ||
        power != POWER_CONFIG || decimation) {
        return ICM45686_CONFIGURATION;
    }
    if (!readReg(io, FIFO_CONFIG0, device->savedFifo, sizeof(device->savedFifo)) ||
        !readReg(io, TIMESTAMP_CONFIG, &device->savedTimestamp, 1) ||
        !readIndirect(io, SMC_CONTROL, &device->savedSmc) ||
        !readReg(io, ICM456XX_INT1_CONFIG0, &device->savedInterrupt, 1)) {
        return ICM45686_TRANSPORT;
    }
    if ((device->savedFifo[0] & FIFO_MODE_MASK) || (device->savedFifo[4] & FIFO_ENABLE) ||
        (device->savedFifo[3] & FIFO_FLUSH)) {
        return ICM45686_CONFIGURATION;
    }
    device->saved = true;
    device->bigEndian = (endian & ICM456XX_SREG_DATA_ENDIAN_SEL_BIG) != 0;
    uint8_t config[6];
    memcpy(config, device->savedFifo, sizeof(config));
    config[0] = FIFO_DEPTH; // Maximum depth, with bypass while configuring.
    config[1] = 1;
    config[2] = 0;
    config[3] = (config[3] & 0x77) | 8;
    config[4] = (config[4] & 0xc0) | 6; // ACC + gyro, no ES, high-resolution or interface enable.
    config[5] = (config[5] & 0xc1) | 2; // Timestamp, no compression.
    const uint8_t streamTimestamp = device->savedTimestamp & ~TIMESTAMP_RESOLUTION_MASK;
    const uint8_t streamSmc = device->savedSmc | 1;
    const uint8_t streamInterrupt = device->savedInterrupt | ICM456XX_INT1_STATUS_EN_FIFO_FULL;
    const bool started = writeReg(io, FIFO_CONFIG4, device->savedFifo[4] & ~FIFO_ENABLE) &&
        writeReg(io, FIFO_CONFIG0, device->savedFifo[0] & ~FIFO_MODE_MASK) &&
        writeIndirect(io, SMC_CONTROL, streamSmc) && writeReg(io, TIMESTAMP_CONFIG, streamTimestamp) &&
        writeRegs(io, FIFO_CONFIG0, config, sizeof(config)) && flushFifo(io, config[3]) &&
        writeReg(io, ICM456XX_INT1_CONFIG0, streamInterrupt) &&
        writeReg(io, FIFO_CONFIG0, FIFO_DEPTH | FIFO_STREAM_MODE) &&
        writeReg(io, FIFO_CONFIG4, config[4] | FIFO_ENABLE);

    uint8_t actual[6], timestamp, smc, interrupt;
    config[0] |= FIFO_STREAM_MODE;
    config[4] |= FIFO_ENABLE;
    if (!started || !readReg(io, FIFO_CONFIG0, actual, sizeof(actual)) ||
        memcmp(actual, config, sizeof(actual)) != 0 ||
        !readReg(io, TIMESTAMP_CONFIG, &timestamp, 1) || timestamp != streamTimestamp ||
        !readIndirect(io, SMC_CONTROL, &smc) || smc != streamSmc ||
        !readReg(io, ICM456XX_INT1_CONFIG0, &interrupt, 1) || interrupt != streamInterrupt) {
        return icm45686FifoStop(io, device) == ICM45686_RESTORE ? ICM45686_RESTORE : ICM45686_CONFIGURATION;
    }
    device->active = true;
    return ICM45686_READY;
}

static icm45686Result_e fifoStatus(const icm45686Io_t *io)
{
    uint8_t status;
    if (!readReg(io, ICM456XX_INT1_STATUS0, &status, 1)) {
        return ICM45686_TRANSPORT;
    }
    if (status & ICM456XX_INT1_STATUS_FIFO_FULL) {
        return ICM45686_OVERFLOW;
    }
    return (status & ICM456XX_INT1_STATUS_RESET_DONE) ? ICM45686_CONFIGURATION : ICM45686_READY;
}

icm45686Result_e icm45686FifoFetch(const icm45686Io_t *io, const icm45686FifoDevice_t *device, icm45686FifoRead_t *readout)
{
    readout->count = 0;
    readout->available = 0;
    if (!device->active) {
        return ICM45686_CLOSED;
    }
    icm45686Result_e status = fifoStatus(io);
    if (status != ICM45686_READY) {
        return status;
    }
    // Discard the first count read and leave the newest packet in the stream.
    uint8_t countData[2];
    if (!readReg(io, FIFO_COUNT, countData, sizeof(countData)) ||
        !readReg(io, FIFO_COUNT, countData, sizeof(countData))) {
        return ICM45686_TRANSPORT;
    }
    readout->available = icm456xxReadU16(countData, device->bigEndian);
    if (readout->available >= FIFO_FULL_COUNT) {
        return ICM45686_OVERFLOW;
    }
    if (readout->available <= 1) {
        readout->readCompleteNs = io->clockNs();
        return ICM45686_PENDING;
    }
    size_t count = readout->available - 1;
    if (count > ICM45686_FIFO_BATCH_CAPACITY) {
        count = ICM45686_FIFO_BATCH_CAPACITY;
    }
    if (!readReg(io, FIFO_DATA, readout->data, count * ICM45686_FIFO_PACKET_BYTES)) {
        return ICM45686_TRANSPORT;
    }
    readout->readCompleteNs = io->clockNs();
    // Reject the whole batch if full or reset occurred during the data burst.
    status = fifoStatus(io);
    if (status == ICM45686_READY) {
        readout->count = count;
    }
    return status;
}

icm45686Result_e icm45686FifoDecode(const uint8_t *data, size_t count, bool bigEndian,
    icm45686FifoTimeline_t *timeline, icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY])
{
    if (!data || !count || count > ICM45686_FIFO_BATCH_CAPACITY) {
        return ICM45686_INVALID;
    }
    icm45686FifoTimeline_t next = *timeline;
    icm45686FifoFrame_t batch[ICM45686_FIFO_BATCH_CAPACITY] = {0};
    for (size_t i = 0; i < count; ++i) {
        const uint8_t *p = data + i * ICM45686_FIFO_PACKET_BYTES;
        icm45686FifoFrame_t *frame = &batch[i];
        // Reject extension/FSYNC/high-resolution and missing data fields.
        if ((p[0] & 0xfc) != 0x68 || p[13] == 0x80 || p[13] == 0x81 || p[13] == 0x7f) {
            return ICM45686_PACKET;
        }
        frame->header = p[0];
        for (unsigned axis = 0; axis < 3; ++axis) {
            frame->accel[axis] = icm456xxReadI16(p + 1 + 2 * axis, bigEndian);
            frame->gyro[axis] = icm456xxReadI16(p + 7 + 2 * axis, bigEndian);
            if (frame->accel[axis] == INT16_MIN || frame->gyro[axis] == INT16_MIN) {
                return ICM45686_PACKET;
            }
        }
        frame->temperature = p[13] <= INT8_MAX ? (int8_t)p[13] : (int8_t)((int)p[13] - 256);
        frame->deviceTick = icm456xxReadU16(p + 14, bigEndian);
        if (next.hasTick) {
            const uint16_t delta = frame->deviceTick - next.previousTick;
            // One missing sample at 6.4 kHz would cause a 312/313 us delta.
            if (delta < 100 || delta > 250 || next.elapsedUs > UINT64_MAX - delta) {
                return ICM45686_TIMELINE;
            }
            next.elapsedUs += delta;
        }
        if (next.ordinal == UINT64_MAX) {
            return ICM45686_TIMELINE;
        }
        frame->ordinal = ++next.ordinal;
        frame->deviceElapsedUs = next.elapsedUs;
        next.previousTick = frame->deviceTick;
        next.hasTick = true;
    }
    *timeline = next;
    memcpy(frames, batch, count * sizeof(*frames));
    return ICM45686_READY;
}
