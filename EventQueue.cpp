/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
#include "bme280.h"

using namespace sixtron;


I2C i2c(I2C1_SDA, I2C1_SCL);
BME280 bme(&i2c, BME280::I2CAddress::Address1);


DigitalOut led(LED1);
InterruptIn button(BUTTON1);


EventQueue queue(32 * EVENTS_EVENT_SIZE);

Thread sensorThread;
Thread ledThread;
Thread eventThread;


void print_temp_hum()
{
    bme.take_forced_measurement();
    float temp = bme.temperature();
    float humd = bme.humidity();

    printf("Temp: %.2f C | Hum: %.2f %%\n", temp, humd);
}


void sensor_task(){
        while (true) {
            queue.call(print_temp_hum);
            ThisThread::sleep_for(2s);
        }
}


void print_pressure()
{
    bme.take_forced_measurement();
    float pres = bme.pressure();

    printf("Pressure: %.2f Pa\n", pres);
}

void blink_led()
{
    led = !led;
}
void led_task(){
        while (true) {
            queue.call(blink_led);
            ThisThread::sleep_for(5s);
        }
 }

void button_pressed_isr()
{

    queue.call(print_pressure);
}


int main()
{

    if (!bme.initialize()) {
        printf("BME280 failed!\n");
        return 1;
    }

    bme.set_sampling();
    button.fall(&button_pressed_isr);

    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));

    sensorThread.start(sensor_task);


    ledThread.start(led_task);


    while (true) {
        ThisThread::sleep_for(1s);
    }
}