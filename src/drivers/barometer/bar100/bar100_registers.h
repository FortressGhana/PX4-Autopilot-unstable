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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file bar100_registers.h
 *
 * Shared defines for the BAR100 I2C driver (Keller LD series).
 */

#pragma once

// I2C address for the BAR100 (Keller LD)
#define BAR100_I2C_ADDR 0x40

// Command to start a measurement
#define BAR100_REQUEST_CMD 0xAC

// MTP (memory map) register addresses
#define BAR100_REG_CUST_ID0 0x00
#define BAR100_REG_CUST_ID1 0x01
#define BAR100_REG_SCALING0 0x12
#define BAR100_REG_SCALING1 0x13
#define BAR100_REG_SCALING2 0x14
#define BAR100_REG_SCALING3 0x15
#define BAR100_REG_SCALING4 0x16

// Measurement result size: status + P_msb + P_lsb + T_msb + T_lsb
#define BAR100_MEASUREMENT_BYTES 5

// Conversion time ~10ms per datasheet, use 20ms interval for 50Hz (matches AP driver)
#define BAR100_CONVERSION_INTERVAL_US 20000
