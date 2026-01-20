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
#ifndef CATIE_SIXTRON_BME280_H_
#define CATIE_SIXTRON_BME280_H_

#include "mbed.h"

namespace sixtron {

typedef struct {
    float humidity;
    float pressure;
    float temperature;
} bme280_environment_t;

typedef struct {
    /*! humidity oversampling setting */
    uint8_t osrs_h;
    /*! pressure oversampling setting */
    uint8_t osrs_p;
    /*! temperature oversampling setting */
    uint8_t osrs_t;
    /*! filter coefficient */
    uint8_t filter;
    /*! standby time */
    uint8_t standby_time;
} bme280_settings_t;

typedef struct {
    /*! uncompensated humidity */
    int32_t humidity;
    /*! uncompensated pressure */
    uint32_t pressure;
    /*! uncompensated temperature */
    int32_t temperature;
} bme280_raw_data_t;

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} bme280_calib_data_t;

/*!
 *  \class BME280
 *  BME280 sensor driver
 */
class BME280
{
public:
    /* I2C addresses */
    enum class I2CAddress : char {
        Address1            = 0x76,
        Address2            = 0x77
    };

    enum class RegisterAddress : char {
        CHIP_ID             = 0xD0,
        RESET               = 0xE0,

        /* Calibration registers */
        DIG_T1              = 0x88,
        DIG_T2              = 0x8A,
        DIG_T3              = 0x8D,

        DIG_P1              = 0x8E,
        DIG_P2              = 0x90,
        DIG_P3              = 0x92,
        DIG_P4              = 0x94,
        DIG_P5              = 0x96,
        DIG_P6              = 0x98,
        DIG_P7              = 0x9A,
        DIG_P8              = 0x9C,
        DIG_P9              = 0x9E,
        DIG_H1              = 0xA1,
        DIG_H2              = 0xE1,
        DIG_H3              = 0xE3,
        DIG_H4              = 0xE4,
        DIG_H5              = 0xE5,
        DIG_H6              = 0xE7,

        CONTROL_HUMID       = 0xF2,
        STATUS              = 0XF3,
        CONTROL_MEAS        = 0xF4,
        CONFIG              = 0xF5,
        PRESS_MSB           = 0xF7,
        PRESS_LSB           = 0xF8,
        PRESS_XLSB          = 0xF9,
        TEMP_MSB            = 0xFA,
        TEMP_LSB            = 0xFB,
        TEMP_XLSB           = 0xFC,
        HUMID_MSB           = 0xFD,
        HUMID_LSB           = 0xFE
    };

    enum class SensorMode : char {
        SLEEP               = 0b00,
        FORCED              = 0b01,
        NORMAL              = 0b11
    };

    enum class SensorSampling : char {
        NONE                = 0b000,
        OVERSAMPLING_X1     = 0b001,
        OVERSAMPLING_X2     = 0b010,
        OVERSAMPLING_X4     = 0b011,
        OVERSAMPLING_X8     = 0b100,
        OVERSAMPLING_X16    = 0b101
    };

    enum class SensorFilter : char {
        OFF                 = 0b000,
        X2                  = 0b001,
        X4                  = 0b010,
        X8                  = 0b011,
        X16                 = 0b100
    };

    enum class StandbyDuration : char {
        MS_0_5              = 0b000,
        MS_62_5             = 0b001,
        MS_125              = 0b010,
        MS_250              = 0b011,
        MS_500              = 0b100,
        MS_1000             = 0b101,
        MS_10               = 0b110,
        MS_20               = 0b111
    };


    /*!
     *  Default BME280 contructor
     *
     *  \param i2c Instance of I2C
     *  \param i2c_address I2C address of the device
     */
    BME280(I2C *i2c, I2CAddress address = I2CAddress::Address1);

    /*!
     *  Initialize the device
     *
     *  \return true on success, false on failure
     */
    bool initialize();


    /*!
     *  Put the device to sleep mode
     *
     *  \return 0 on success, 1 on failure
     */
    int sleep();

    /*!
     *  Perform a reset of the device
     *
     *  \return 0 on success, 1 on failure
     */
    int reset();

    /*!
     *  Compute humidity
     *
     *  \return humidity on success, NAN on failure
     */
    float humidity();

    /*!
     *  Compute pressure
     *
     *  \return pressure on success, NAN on failure
     */
    float pressure();

    /*!
     *  Compute temperature
     *
     *  \return temperature on success, NAN on failure
     */
    float temperature();

    /*!
     *  Read sensor environmental parameters
     *
     *  \param env Structure filled with data on success or nan values on failure
     */
    void read_env_data(bme280_environment_t &env);


    /*!
     * Take a new measurement.
     *
     * \attention Only available in \cSensorMode::FORCED mode.
     */
    void take_forced_measurement();

    /*!
     *  Set the device power mode
     *
     *  \param mode Chosen power mode
     *  \return 0 on success, 1 on failure
     */
    int set_power_mode(SensorMode mode);

    /*!
     *  Get the device power mode
     *
     *  \param mode Pointer to the value of power mode
     *  \return 0 on success, 1 on failure
     */
    int get_power_mode(SensorMode *mode);

    /*!
     *  Set sampling settings
     *
     *  \param mode Sensor mode to set
     *  \param temp_sampling Temperature sampling to set
     *  \param press_sampling Pressure sampling to set
     *  \param humid_sampling Humidity sampling to set
     *  \param filter Filter setting to set
     *  \param duration Stand-by duration
     */
    void set_sampling(SensorMode mode = SensorMode::NORMAL,
            SensorSampling temp_sampling = SensorSampling::OVERSAMPLING_X16,
            SensorSampling press_sampling = SensorSampling::OVERSAMPLING_X16,
            SensorSampling humid_sampling = SensorSampling::OVERSAMPLING_X16,
            SensorFilter filter = SensorFilter::OFF, StandbyDuration duration =
                    StandbyDuration::MS_0_5);

    /*!
     *  \return Chip ID
     */
    char get_chip_id()
    {
        return _chip_id;
    }

    bme280_settings_t get_settings()
    {
        return settings;
    }

private:
    char _chip_id = 0;
    I2C *_i2c;
    I2CAddress _i2c_address;
    SensorMode _sensor_mode;
    bme280_settings_t settings;
    bme280_calib_data_t calib;
    bme280_raw_data_t uncomp_data;
    int32_t t_fine;

    /*!
     *  Private function to check chip ID correctness
     *
     *  \return true on success, false on failure
     */
    bool read_chip_id();

    /*!
     *  Read and store calibration data
     */
    void get_calib();

    /*!
     *  Parse raw data: pressure, humidity and temperature, then store it to
     *  uncomp_data attribute
     */
    void get_raw_data();

    /*! Set power mode
     *
     *  \param mode Power mode
     *  \return 0 on success, 1 on failure
     */
    int write_power_mode(SensorMode mode);

    /*!
     *  Read register data
     *
     *  \param register_address Address of the register
     *  \param value Pointer to the value read from the register
     *  \return 0 on success, 1 on failure
     */
    int i2c_read_register(RegisterAddress register_address, int8_t *value);

    /*!
     *  Read two successive registers data
     *  \note This function is useful to read memory-contiguous LSB/MSB registers
     *
     *  \param register_address Address of the first register
     *  \param value Array to store read data
     *  \return 0 on success, 1 on failure
     */
    int i2c_read_two_bytes(RegisterAddress register_address, int8_t value[2]);

    /*!
     *  Read three successive registers data
     *  \note This function is useful to read memory-contiguous LSB/MSB/XLSB registers
     *
     *  \param register_address Address of the first register
     *  \param value Array to store the read data
     *  \return 0 on success, 1 on failure
     */
    int i2c_read_three_bytes(RegisterAddress register_address, int8_t value[3]);

    /*!
     *  Read a 16 bits signed vector (3 dimensions) continuous read
     *
     *  \param register_address Address of the register containing the LSB
     *  of the first axis value Array to store the read values
     *  \return 0 on success, 1 on failure
     */
    int i2c_read_vector(RegisterAddress register_address, int16_t value[3]);

    /*!
     *  Write to a register
     *
     *  \param register_address Address of the register to write to
     *  \param value Data to store in the register
     *  \return 0 on success, 1 on failure
     */
    int i2c_write_register(RegisterAddress register_address, int8_t value);
};

} // namespace sixtron

#endif // CATIE_SIXTRON_BME280_H_
