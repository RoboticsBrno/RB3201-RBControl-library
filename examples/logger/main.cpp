#include <iostream>
#include "RBControl_logger.hpp"

#include <Arduino.h>

void setup() {
    rb::log(INFO, "Start", "RB3201-RBControl");
    delay(1500);
    rb::log(WARNING, "Tag1", "Test message 1");
    delay(15000);
    rb::log(WARNING, "Tag2", "Test message 2");
}

void loop() {

}