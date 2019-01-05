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
        man.setMotors().power(rb::MotorId::M1, 20).power(rb::MotorId::M2, 20).set();
        rb::log(INFO, "Motors", "lmotor power: {}  rmotor power: {}",
                20, 20);
        delay(1000);

        man.setMotors().power(rb::MotorId::M1, 0).power(rb::MotorId::M2, 0).set();
        rb::log(INFO, "Motors", "lmotor power: {}  rmotor power: {}", 0, 0);
        delay(5000);
    }
}

void loop() {

}