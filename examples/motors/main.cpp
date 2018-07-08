#include <iostream>
#include "RBControl_manager.hpp"

#include <Arduino.h>

void setup() {
    rb::log(INFO, "Start", "RB3201-RBControl");
    delay(500);

    rb::log(INFO, "Motors", "Init clas Motors");
    rb::Manager man;

    while(true) {
        micros(); // update overflow
        man.setMotors().power(0, 50).power(1, 50).set();
        motors.update();
        rb::log(INFO, "Motors", "Motors power: 50");
        delay(300);
    }
}

void loop() {

}