/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
#include "Thread.h"

namespace {
#define PERIOD_MS 2000ms
}



DigitalOut led(LED1);

using namespace rtos;

Thread t_ping;
Thread t_pong;
Mutex ping_pong;


void ping(){
    for(int i =0 ; i < 100; i++){
        ping_pong.lock();
        printf("Ping\n");
        ping_pong.unlock();
    }

}

void pong(){
    for(int i =0 ; i < 100; i++){
        ping_pong.lock();
        printf("Pong\n");
        ping_pong.unlock();
    }

}


int main(){
    
    t_ping.start(ping);
    t_pong.start(pong);
    osThreadSetPriority(osThreadGetId(), osPriorityLow);
    while (true) {
        led = !led;
        ping_pong.lock();
         printf("Alive! \n");
        ping_pong.unlock();
        ThisThread::sleep_for(PERIOD_MS / 2);
        
    }
    
}
