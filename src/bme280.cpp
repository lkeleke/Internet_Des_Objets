/*
 * Copyright (c) 2017, CATIE, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "bme280.h"

namespace sixtron {

namespace {

#define SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##__MSK)) |\
     (data & bitname##__MSK))

#define GET_BITS_POS_0(reg_data, bitname) \
    (reg_data & (bitname##__MSK))

/**\name MACROS DEFINITIONS                      */
#define INIT_VALUE                         0
#define SUCCESS                            0
#define FAILURE                            1
#define TEMP_PRESS_CALIB_DATA_LEN          26
/** Bit value manipulation                       */
#define ZERO                               0
#define ONE                                1
#define ONE_BIT_SHIFT                      1
#define TWO_BITS_SHIFT                     2
#define FOUR_BITS_SHIFT                    4
#define EIGHT_BITS_SHIFT                   8
#define SIXTEEN_BITS_SHIFT                 16

/** Temp/Press/Humidity minimum/maximum values   */
#define TEMPERATURE_MIN                    -40
#define TEMPERATURE_MAX                    85
#define PRESSURE_MIN                       30000
#define PRESSURE_MAX                       110000
#define HUMIDITY_MIN                       0
#define HUMIDITY_MAX                       419430400

#define CONTROL_MEAS__MSK                  0xFC

#define SENSOR_MODE__MSK                   0x03
#define SENSOR_MODE__POS                   0x00

#define UNCOMPENSATED_TEMPERATURE__MSK     0xFFFFF
#define UNCOMPENSATED_PRESSURE_MSK         0xFFFFF

#define OSRS_T__POS                        0x5
#define OSRS_P__POS                        0x3
#define STANDBY__POS                       0x5
#define FILTER__POS                        0x3

#define SOFTRESET_CMD                      0xB6

#define CHIP                               0x60

}

BME280::BME280(I2C *i2c, I2CAddress i2c_address) :
    _i2c(i2c), _i2c_address(i2c_address)
{
    _sensor_mode = SensorMode::NORMAL;
    t_fine = 0;
}

bool BME280::initialize()
{
    printf("\nInitializing the BME280...\n");
    if (!read_chip_id()) {
        return false;
    } else {
        printf("Chip ID: 0x%X\n", CHIP);
        _chip_id = CHIP;
    }
    if (reset() != SUCCESS) {
        return false;
    }

    // wait for chip to wake up
    ThisThread::sleep_for(1ms);

    get_calib();

    return true;
}

int BME280::reset()
{
    if (i2c_write_register(RegisterAddress::RESET, SOFTRESET_CMD) != SUCCESS) {
        return FAILURE;
    }
    ThisThread::sleep_for(2ms);
    return SUCCESS;
}

float BME280::humidity()
{
    if (isnan(temperature())) { // must be done first to get t_fine
        return NAN;
    }

    int32_t var1 = t_fine - 76800;
    var1 = (((((uncomp_data.humidity << 14) - (((int32_t) calib.dig_H4) << 20)
                                            - (((int32_t) calib.dig_H5) * var1)) + ((int32_t) 16384)) >> 15)
                    * (((((((var1 * ((int32_t) calib.dig_H6)) >> 10)
                                    * (((var1 * ((int32_t) calib.dig_H3)) >> 11) + ((int32_t) 32768)))
                            >> 10) + ((int32_t) 2097152)) * ((int32_t) calib.dig_H2) + 8192)
                            >> 14));

    var1 = (var1
                    - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t) calib.dig_H1))
                            >> 4));

    var1 = (var1 < HUMIDITY_MIN) ? HUMIDITY_MIN : var1;
    var1 = (var1 > HUMIDITY_MAX) ? HUMIDITY_MAX : var1;
    return static_cast<float>(var1 >> 12) / 1024;
}

float BME280::pressure()
{
    int64_t var1, var2, pressure;
    if (isnan(temperature())) { // must be done first to get t_fine
        return NAN;
    }

    if (uncomp_data.pressure == 0x80000) { // value in case pressure measurement was disabled
        return NAN;
    }

    var1 = ((int64_t) t_fine) - 128000;
    var2 = var1 * var1 * (int64_t) calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t) calib.dig_P5) << 17);
    var2 = var2 + (((int64_t) calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t) calib.dig_P3) >> 8)
            + ((var1 * (int64_t) calib.dig_P2) << 12);
    var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) calib.dig_P1) >> 33;

    if (!var1) {
        return FAILURE; // avoid exception caused by division by zero
    }

    pressure = 1048576 - uncomp_data.pressure;
    pressure = (((pressure << 31) - var2) * 3125) / var1;
    var1 = (((int64_t) calib.dig_P9) * (pressure >> 13) * (pressure >> 13))
            >> 25;
    var2 = (((int64_t) calib.dig_P8) * pressure) >> 19;

    pressure = ((pressure + var1 + var2) >> 8)
            + (((int64_t) calib.dig_P7) << 4);

    return (float) pressure / 256;
}

float BME280::temperature()
{
    int32_t var1, var2;
    get_raw_data();

    if (uncomp_data.temperature == 0x80000) { // value in case temp measurement was disabled
        return NAN;
    }

    //uncomp_data.temperature >>= 4;

    var1 = ((((uncomp_data.temperature >> 3) - ((int32_t) calib.dig_T1 << 1)))
                    * ((int32_t) calib.dig_T2)) >> 11;

    var2 = (((((uncomp_data.temperature >> 4) - ((int32_t) calib.dig_T1))
                                    * ((uncomp_data.temperature >> 4) - ((int32_t) calib.dig_T1))) >> 12)
                    * ((int32_t) calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    float T = (t_fine * 5 + 128) >> 8;
    return T / 100;
}

void BME280::read_env_data(bme280_environment_t &env)
{
    env.temperature = temperature();
    env.pressure = pressure();
    env.humidity = humidity();
    return;
}

int BME280::write_power_mode(SensorMode mode)
{
    int8_t sensor_mode;
    if (i2c_read_register(RegisterAddress::CONTROL_MEAS,
                    &sensor_mode) != SUCCESS) {
        return FAILURE;
    }
    sensor_mode = SET_BITS_POS_0(sensor_mode, SENSOR_MODE,
                    static_cast<int8_t>(mode));
    if (i2c_write_register(RegisterAddress::CONTROL_MEAS,
                    sensor_mode) != SUCCESS) {
        return FAILURE;
    }
    return SUCCESS;
}

int BME280::sleep()
{
    static char data[4];
    data[0] = static_cast<char>(RegisterAddress::CONTROL_HUMID);
    if (_i2c->write(static_cast<int>(_i2c_address) << 1, data, 1,
                    true) != SUCCESS) {
        return FAILURE;
    }
    if (_i2c->read(static_cast<int>(_i2c_address) << 1, data, 4,
                    false) != SUCCESS) {
        return FAILURE;
    }
    return reset();
}

void BME280::take_forced_measurement()
{
    if (_sensor_mode == SensorMode::FORCED) {
        if (i2c_write_register(RegisterAddress::CONTROL_MEAS,
                        static_cast<int8_t>((settings.osrs_t << OSRS_T__POS)
                                | (settings.osrs_p << OSRS_P__POS)
                                | static_cast<char>(_sensor_mode))) != SUCCESS) {
            return;
        }
        int8_t data;
        while (true) {
            if (i2c_read_register(RegisterAddress::STATUS, &data) != SUCCESS) {
                return;
            }
            if (!(data & 0x08)) { // STATUS measuring[0] bit (0 when the results have been transferred to the data registers)
                break;
            }
            ThisThread::sleep_for(1ms);
        }
    }
}

int BME280::set_power_mode(SensorMode mode)
{
    SensorMode last_set_mode;
    if (get_power_mode(&last_set_mode) != SUCCESS) {
        return FAILURE;
    }
    if (last_set_mode != SensorMode::SLEEP) {
        if (sleep() != SUCCESS) {
            return FAILURE;
        }
        write_power_mode(mode);
        return SUCCESS;
    }
    return FAILURE;
}

int BME280::get_power_mode(SensorMode *mode)
{
    if (mode) {
        *mode = _sensor_mode;
        return SUCCESS;
    }
    return FAILURE;
}

void BME280::set_sampling(SensorMode mode, SensorSampling temp_sampling,
        SensorSampling press_sampling, SensorSampling humid_sampling,
        SensorFilter filter, StandbyDuration duration)
{
    _sensor_mode = mode;
    settings.osrs_t = static_cast<uint8_t>(temp_sampling);
    settings.osrs_h = static_cast<uint8_t>(humid_sampling);
    settings.osrs_p = static_cast<uint8_t>(press_sampling);
    settings.filter = static_cast<uint8_t>(filter);
    settings.standby_time = static_cast<uint8_t>(duration);

    if (i2c_write_register(RegisterAddress::CONTROL_HUMID,
                    static_cast<int8_t>(settings.osrs_h)) != SUCCESS) {
        return;
    }

    if (i2c_write_register(RegisterAddress::CONTROL_MEAS,
                    static_cast<int8_t>((settings.osrs_t << OSRS_T__POS)
                            | (settings.osrs_p << OSRS_P__POS) | static_cast<char>(mode))) != SUCCESS) {
        return;
    }

    if (i2c_write_register(RegisterAddress::CONFIG,
                    static_cast<int8_t>((settings.standby_time << STANDBY__POS)
                            | (settings.filter << FILTER__POS) | 0)) != SUCCESS) {
        return;
    }
}

bool BME280::read_chip_id()
{
    int8_t chip_id = INIT_VALUE;
    if (i2c_read_register(RegisterAddress::CHIP_ID, &chip_id) != SUCCESS) {
        return FAILURE;
    }
    if (chip_id != CHIP) {
        ThisThread::sleep_for(1s);
        i2c_read_register(RegisterAddress::CHIP_ID, &chip_id);
        return (chip_id != CHIP) ? false : true;
    }
    return true;
}

void BME280::get_calib()
{
    int8_t s8_dig_1, s8_dig_2;
    int8_t s8_dig[2];

    /* Temperature-related coefficients */
    static char data[18];
    data[0] = static_cast<char>(RegisterAddress::DIG_T1);
    _i2c->write(static_cast<int>(_i2c_address) << 1, data, 1, true);
    _i2c->read(static_cast<int>(_i2c_address) << 1, data, 6, false);
    calib.dig_T1 = static_cast<uint16_t>((data[1] << 8) | (data[0]));
    calib.dig_T2 = (data[3] << 8) | (data[2]);
    calib.dig_T3 = (data[5] << 8) | (data[4]);

    /* Pressure-related coefficients */
    data[0] = static_cast<char>(RegisterAddress::DIG_P1);
    _i2c->write(static_cast<int>(_i2c_address) << 1, data, 1, true);
    _i2c->read(static_cast<int>(_i2c_address) << 1, data, 18, false);
    calib.dig_P1 = (data[1] << 8) | data[0];
    calib.dig_P2 = (data[3] << 8) | data[2];
    calib.dig_P3 = (data[5] << 8) | data[4];
    calib.dig_P4 = (data[7] << 8) | data[6];
    calib.dig_P5 = (data[9] << 8) | data[8];
    calib.dig_P6 = (data[11] << 8) | data[10];
    calib.dig_P7 = (data[13] << 8) | data[12];
    calib.dig_P8 = (data[15] << 8) | data[14];
    calib.dig_P9 = (data[17] << 8) | data[16];

    /* Humidity-related coefficients */
    i2c_read_register(RegisterAddress::DIG_H1, &s8_dig_1);
    calib.dig_H1 = static_cast<uint8_t>(s8_dig_1);
    i2c_read_two_bytes(RegisterAddress::DIG_H2, s8_dig);
    calib.dig_H2 = (s8_dig[1] << 8 | s8_dig[0]);
    i2c_read_register(RegisterAddress::DIG_H3, &s8_dig_1);
    calib.dig_H3 = static_cast<uint8_t>(s8_dig_1);
    i2c_read_two_bytes(RegisterAddress::DIG_H4, s8_dig);
    calib.dig_H4 = (s8_dig[0] << 4 | (s8_dig[1] & 0xF));
    i2c_read_register(RegisterAddress::DIG_H5, &s8_dig_1);
    i2c_read_register(
            static_cast<RegisterAddress>(static_cast<char>(RegisterAddress::DIG_H5)
                    + 1), &s8_dig_2);
    calib.dig_H5 = (s8_dig_2 << 4 | ((s8_dig_1 >> FOUR_BITS_SHIFT) & 0xF));

    i2c_read_register(RegisterAddress::DIG_H6, &calib.dig_H6);

}

void BME280::get_raw_data()
{
    char cmd;
    char data[8];

    // read PRESS_MSB, PRESS_LSB, PRESS_XLSB, TEMP_MSB, TEMP_LSB, TEMP_XLSB, HUMID_MSB, HUMID_LSB
    cmd = static_cast<char>(RegisterAddress::PRESS_MSB);
    _i2c->write((static_cast<int>(_i2c_address)) << 1, &cmd, 1, true);
    _i2c->read((static_cast<int>(_i2c_address)) << 1, data, 8);

    // raw_pressure = PRESS_MSB | PRESS_LSB | PRESS_XLSB[7:4]
    uncomp_data.pressure = static_cast<uint32_t>((data[0] << 12)
                    | (data[1] << 4) | (data[2] >> 4));
    uncomp_data.pressure &= UNCOMPENSATED_PRESSURE_MSK;

    // raw_temperature = TEMP_MSB | TEMP_LSB | TEMP_XLSB[7:4]
    uncomp_data.temperature =
            ((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));
    uncomp_data.temperature &= UNCOMPENSATED_TEMPERATURE__MSK;

    // raw_humidity = HUMID_MSB | HUMID_LSB
    uncomp_data.humidity = static_cast<int32_t>(data[6] << EIGHT_BITS_SHIFT
                    | data[7]);
}

int BME280::i2c_read_register(RegisterAddress register_address, int8_t *value)
{
    static char data;
    data = static_cast<char>(register_address);
    if (_i2c->write(static_cast<int>(_i2c_address) << 1, &data, 1,
                    true) != SUCCESS) {
        return FAILURE;
    }
    char *char_value = reinterpret_cast<char *>(value);
    if (_i2c->read(static_cast<int>(_i2c_address) << 1, char_value,
                    1) != SUCCESS) {
        return FAILURE;
    }
    return SUCCESS;
}

int BME280::i2c_read_two_bytes(RegisterAddress register_address,
        int8_t value[2])
{
    static char data[2];
    data[0] = static_cast<char>(register_address);
    if (_i2c->write(static_cast<int>(_i2c_address) << 1, data, 1,
                    true) != SUCCESS) {
        return FAILURE;
    }
    if (_i2c->read(static_cast<int>(_i2c_address) << 1, data, 2,
                    false) != SUCCESS) {
        return FAILURE;
    }
    for (int i = 0; i < 2; i++) {
        value[i] = data[i];
    }
    return SUCCESS;
}

int BME280::i2c_read_three_bytes(RegisterAddress register_address,
        int8_t value[3])
{
    char data[3];
    data[0] = static_cast<char>(register_address);
    if (_i2c->write(static_cast<int>(_i2c_address) << 1, data, 1,
                    true) != SUCCESS) {
        return FAILURE;
    }
    if (_i2c->read(static_cast<int>(_i2c_address) << 1, data, 3,
                    false) != SUCCESS) {
        return FAILURE;
    }
    for (int i = 0; i < 3; i++) {
        value[i] = data[i];
    }
    return SUCCESS;
}

int BME280::i2c_read_vector(RegisterAddress register_address,
        int16_t value[3])
{
    static char data[6];
    data[0] = static_cast<char>(register_address);
    if (_i2c->write(static_cast<int>(_i2c_address) << 1, data, 1,
                    true) != SUCCESS) {
        return FAILURE;
    }
    if (_i2c->read(static_cast<int>(_i2c_address) << 1, data, 6,
                    false) != SUCCESS) {
        return FAILURE;
    }
    for (int i = 0; i < 3; i++) {
        value[i] = (data[2 * i + 1] << EIGHT_BITS_SHIFT) | (data[2 * i]);
    }
    return SUCCESS;
}

int BME280::i2c_write_register(RegisterAddress register_address, int8_t value)
{
    static char data[2];
    data[0] = static_cast<char>(register_address);
    data[1] = static_cast<char>(value);
    if (_i2c->write(static_cast<int>(_i2c_address) << 1, data, 2) != SUCCESS) {
        return FAILURE;
    }
    return SUCCESS;
}

} // namespace sixtron
