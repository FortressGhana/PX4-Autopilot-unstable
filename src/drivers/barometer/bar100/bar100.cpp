/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file bar100.cpp
 * Driver for the BAR100 barometric pressure sensor (Keller LD series) connected via I2C.
 */

#include "bar100.hpp"
#include "bar100_registers.h"
#include <inttypes.h>

Bar100::Bar100(const I2CSPIDriverConfig &config) :
    I2C(config),
    I2CSPIDriver(config),
    _sample_perf(perf_alloc(PC_ELAPSED, "bar100_read")),
    _measure_perf(perf_alloc(PC_ELAPSED, "bar100_measure")),
    _comms_errors(perf_alloc(PC_COUNT, "bar100_com_err"))
{
}

Bar100::~Bar100()
{
    perf_free(_sample_perf);
    perf_free(_measure_perf);
    perf_free(_comms_errors);
}

int Bar100::init()
{
    int ret = I2C::init();
    if (ret != PX4_OK) {
        PX4_ERR("I2C init failed (%d)", ret);
        return ret;
    }

    if (!_read_memory_map()) {
        PX4_ERR("BAR100 calibration read failed — not starting");
        return PX4_ERROR;
    }

    _start();
    return PX4_OK;
}

int Bar100::probe()
{
    PX4_INFO("Probing BAR100 at I2C addr 0x%02X", BAR100_I2C_ADDR);
    set_device_address(BAR100_I2C_ADDR);

    // Try to read an MTP register — if the sensor responds, it's there.
    // This avoids triggering measurements (0xAC) which would put the sensor
    // into conversion mode and interfere with subsequent calibration reads.
    uint8_t cmd = BAR100_REG_SCALING0;
    uint8_t buf[3] = {};

    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        if (transfer(&cmd, 1, nullptr, 0) != PX4_OK) {
            px4_usleep(5000);
            continue;
        }

        px4_usleep(5000);

        if (transfer(nullptr, 0, buf, sizeof(buf)) == PX4_OK) {
            PX4_INFO("BAR100 probe OK attempt %u: [%02x %02x %02x]",
                     attempt + 1, buf[0], buf[1], buf[2]);
            return PX4_OK;
        }

        px4_usleep(5000);
    }

    PX4_ERR("BAR100 probe failed — no response");
    return -EIO;
}

// Read a single MTP 16-bit register (write address, wait, read status+MSB+LSB).
// Follows AP's transfer_with_delays pattern.
bool Bar100::_read_mtp16(uint8_t mtp_addr, uint16_t &val)
{
    uint8_t buf[3] = {};

    // Write register address
    if (transfer(&mtp_addr, 1, nullptr, 0) != PX4_OK) {
        PX4_ERR("BAR100 MTP write 0x%02X failed", mtp_addr);
        return false;
    }

    px4_usleep(2000);

    // Read status + 2 data bytes
    if (transfer(nullptr, 0, buf, sizeof(buf)) != PX4_OK) {
        PX4_ERR("BAR100 MTP read 0x%02X failed", mtp_addr);
        return false;
    }

    px4_usleep(1000);

    PX4_INFO("BAR100 MTP[0x%02X]: status=0x%02X data=[0x%02X 0x%02X]",
             mtp_addr, buf[0], buf[1], buf[2]);

    val = (static_cast<uint16_t>(buf[1]) << 8) | buf[2];
    return true;
}

bool Bar100::_read_memory_map()
{
    // Sensor must be idle for MTP reads — do NOT send 0xAC before this.
    // probe() was designed to not trigger measurements, so sensor should be idle.
    // Wait for any prior activity to settle.
    px4_usleep(150000);

    // Read customer ID registers
    uint16_t cust0 = 0, cust1 = 0;

    if (!_read_mtp16(BAR100_REG_CUST_ID0, cust0) ||
        !_read_mtp16(BAR100_REG_CUST_ID1, cust1)) {
        PX4_ERR("BAR100: CUST_ID reads failed");
        return false;
    }

    _equipment = cust0 >> 10;
    _code = (static_cast<uint32_t>(cust1) << 16) | cust0;

    // Read scaling registers
    uint16_t scaling0 = 0, scaling1 = 0, scaling2 = 0, scaling3 = 0, scaling4 = 0;

    if (!_read_mtp16(BAR100_REG_SCALING0, scaling0) ||
        !_read_mtp16(BAR100_REG_SCALING1, scaling1) ||
        !_read_mtp16(BAR100_REG_SCALING2, scaling2) ||
        !_read_mtp16(BAR100_REG_SCALING3, scaling3) ||
        !_read_mtp16(BAR100_REG_SCALING4, scaling4)) {
        PX4_ERR("BAR100: scaling reads failed");
        return false;
    }

    PX4_INFO("BAR100 raw scaling: %04x %04x %04x %04x %04x",
             scaling0, scaling1, scaling2, scaling3, scaling4);

    // Decode P-mode from scaling0 bits 0-1
    _mode = scaling0 & 0x03;
    uint16_t year  = (scaling0 >> 11) + 2000;
    uint8_t month  = (scaling0 & 0x0780) >> 7;
    uint8_t day    = (scaling0 & 0x007C) >> 2;

    switch (_mode) {
    case 0: // PR-mode: vented gauge, offset = standard atmosphere
        _P_mode = 1.01325f;
        break;
    case 1: // PA-mode: sealed gauge, offset = 1.0 bar
        _P_mode = 1.0f;
        break;
    default: // PAA-mode: absolute, no offset
        _P_mode = 0.0f;
        break;
    }

    // Combine scaling words into 32-bit IEEE 754 floats
    uint32_t pmin_bits = (static_cast<uint32_t>(scaling1) << 16) | scaling2;
    uint32_t pmax_bits = (static_cast<uint32_t>(scaling3) << 16) | scaling4;
    memcpy(&_P_min, &pmin_bits, sizeof(float));
    memcpy(&_P_max, &pmax_bits, sizeof(float));

    PX4_INFO("BAR100 cal: code=0x%08" PRIx32 " equip=%u mode=%u date=%04u-%02u-%02u",
             _code, _equipment, _mode, year, month, day);
    PX4_INFO("BAR100 cal: P_min=%.6f bar P_max=%.6f bar P_mode=%.6f",
             (double)_P_min, (double)_P_max, (double)_P_mode);

    // Validate calibration
    if (!PX4_ISFINITE(_P_min) || !PX4_ISFINITE(_P_max) || _P_min >= _P_max) {
        PX4_ERR("BAR100: invalid calibration (P_min=%.6f P_max=%.6f)", (double)_P_min, (double)_P_max);
        return false;
    }

    return true;
}

void Bar100::_start()
{
    // Trigger the first measurement, then schedule collection
    uint8_t cmd = BAR100_REQUEST_CMD;
    transfer(&cmd, 1, nullptr, 0);
    _collect_phase = true;
    ScheduleDelayed(BAR100_CONVERSION_INTERVAL_US);
}

// Two-phase RunImpl: alternates between collecting previous result and triggering next measurement.
// No blocking px4_usleep — timing is handled by ScheduleDelayed.
void Bar100::RunImpl()
{
    if (_collect_phase) {
        // Phase 2: Read previous conversion result
        perf_begin(_sample_perf);

        uint8_t buf[BAR100_MEASUREMENT_BYTES] = {};

        if (transfer(nullptr, 0, buf, sizeof(buf)) == PX4_OK) {
            uint16_t P_raw = (static_cast<uint16_t>(buf[1]) << 8) | buf[2];
            uint16_t T_raw = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];

            if (P_raw != 0 && T_raw != 0) {
                float P_bar = (static_cast<float>(P_raw) - 16384.0f) * (_P_max - _P_min) / 32768.0f
                              + _P_min + _P_mode;
                float temperature = ((T_raw >> 4) - 24) * 0.05f - 50.0f;
                float pressure_pa = P_bar * 100000.0f; // bar -> Pa

                _last_pressure = pressure_pa;
                _last_temperature = temperature;

                sensor_baro_s report{};
                report.timestamp = hrt_absolute_time();
                report.device_id = get_device_id();
                report.pressure = pressure_pa;
                report.temperature = temperature;
                _sensor_baro_pub.publish(report);

            } else {
                perf_count(_comms_errors);
            }

        } else {
            perf_count(_comms_errors);
        }

        perf_end(_sample_perf);
    }

    // Trigger next measurement
    perf_begin(_measure_perf);
    uint8_t cmd = BAR100_REQUEST_CMD;

    if (transfer(&cmd, 1, nullptr, 0) != PX4_OK) {
        perf_count(_comms_errors);
    }

    perf_end(_measure_perf);

    _collect_phase = true;
    ScheduleDelayed(BAR100_CONVERSION_INTERVAL_US);
}

void Bar100::print_status()
{
    I2CSPIDriverBase::print_status();
    PX4_INFO("P_min=%.4f P_max=%.4f P_mode=%.4f mode=%u",
             (double)_P_min, (double)_P_max, (double)_P_mode, _mode);
    PX4_INFO("pressure=%.2f Pa  temperature=%.2f C",
             (double)_last_pressure, (double)_last_temperature);
    perf_print_counter(_sample_perf);
    perf_print_counter(_measure_perf);
    perf_print_counter(_comms_errors);
}
