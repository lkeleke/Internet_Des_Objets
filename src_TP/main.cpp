/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
using namespace std::chrono;
namespace {
#define PERIOD_MS 500ms
}

DigitalOut led(LED1);
//DigitalIn button(BUTTON1);
InterruptIn button(BUTTON1);
Timer t;
Ticker flipper;
int flag; 
void flip()
{   
    led = !led;
}

void press(){
    led=1;
    t.reset();
    t.start();

}

void release(){
    led =0; 
    t.stop();
    flag=1;
}
int main()
{
    
    button.rise(&press);
    button.fall(&release);
    flag=0;

   //flipper.attach(&flip, 1);
    while (true) {
         printf("Groupe 7 : \n");
        //printf("La valeur du button est : %d \n", button.read());
        //led = !led;
        //led = button;
        if(flag){
                printf("La dure de l'appui du button est : %llu ms\n", duration_cast<milliseconds>(t.elapsed_time()).count());
                flag=0;
        }
        ThisThread::sleep_for(PERIOD_MS / 2);
    }
}
