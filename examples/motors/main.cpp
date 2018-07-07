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
        lmotor.power(20);
        rmotor.power(20);
        motors.update();
        rb::log(INFO, "Motors", "lmotor power: {}  rmotor power: {}",
                lmotor.power(), rmotor.power());
        delay(1000);

        lmotor.power(0);
        rmotor.power(0);
        motors.update();
        rb::log(INFO, "Motors", "lmotor power: {}  rmotor power: {}",
                lmotor.power(), rmotor.power());
        delay(5000);
    }
}

void loop() {

}