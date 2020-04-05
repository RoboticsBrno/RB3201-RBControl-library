#include "RBControl.hpp"
#include <iostream>

#include <Arduino.h>

void setup() {
    rb::log(INFO, "Start", "RB3201-RBControl");
    delay(500);

    rb::log(INFO, "Manager", "Init manager");
    auto& man = rb::Manager::get();
    man.install();

    while (true) {
        micros(); // update overflow
        man.setMotors().power(rb::MotorId::M1, 20).power(rb::MotorId::M2, 20).set();
        rb::log(INFO, "Motors", "lmotor power: {}  rmotor power: {}",
            20, 20);
        delay(1000);

        rb::Manager::get().setMotors().power(rb::MotorId::M1, 0).power(rb::MotorId::M2, 0).set();
        rb::log(INFO, "Motors", "lmotor power: {}  rmotor power: {}", 0, 0);
        delay(5000);
    }
}

void loop() {
}
