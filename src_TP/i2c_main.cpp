/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
#include "bme280.h"

using namespace sixtron;

I2C i2c(I2C1_SDA, I2C1_SCL);
BME280 bme(&i2c, BME280::I2CAddress::Address1);

int main()
{
    if(!bme.initialize()){
        printf("bme faild!\n");
        return 1;
    }
    bme.set_sampling();
    while (true) {
        bme.take_forced_measurement();
        float temp = bme.temperature();
        float pres = bme.pressure();
        float humd = bme.humidity();
        printf(" Temp:  %.2f.C \n Pressure: %f Pa\n Humidity: %.2f \n", temp, pres, humd);
        ThisThread::sleep_for(1s);
    }
    
}
