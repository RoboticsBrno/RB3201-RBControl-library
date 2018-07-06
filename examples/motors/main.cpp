#include <iostream>
#include "RBControl_motors.hpp"

#include <Arduino.h>

void setup() {
    rb::log(INFO, "Start", "RB3201-RBControl");
    delay(500);

    rb::log(INFO, "Motors", "Init clas Motors");
    rb::Motors motors;

    rb::Motor& lmotor = motors.motor(0);
    rb::Motor& rmotor = motors.motor(1);

    while(true) {
        micros(); // update overflow
        lmotor.power(80);
        rmotor.power(100);
        motors.update();
        rb::log(INFO, "Motors", "Motors power: 50");
        delay(300);
    }
}

void loop() {

}