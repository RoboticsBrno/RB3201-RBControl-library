#pragma once

#include <freertos/FreeRTOS.h>
#include <vector>

#include "serialpcm.hpp"
#include "RBControl_pinout.hpp"
#include "_RBControl_motor.hpp"

namespace rb {

static const int channels = 16;

/// @private
class Motors {
public: 
    Motors()
        :pwm_index({12, 13, 2, 3, 8, 9, 14, 15, 4, 5, 10, 11, 1, 2, 6, 7})
    {
        for(int index = 0; index < pwm_index.size(); index += 2) {
            motors.emplace_back(pwm[pwm_index[index]], pwm[pwm_index[index + 1]], index / 2);
        }
    }

    Motor& motor(int index) {
        if(index > -1 && index < (pwm_index.size() / 2)) {
            return motors[index];
        } else {
            rb::logWarning("Motor",
                           "Wrong index of motor reference. Required: 0 - {}. Get index: {}.", 
                           (pwm_index.size() / 2), index);
            return motors[0];
        }
    }

    int motorCount() { return motors.size(); }
    
    void update() {
        pwm.update();
    }

    void stop() {
        for(rb::Motor & motor : motors) {
            motor.power(0);
        }
        update();
    }

private: 
    SerialPCM pwm {channels, {SERMOT}, RCKMOT, SCKMOT};
    std::vector<int> pwm_index;
    std::vector<Motor> motors;
};

} // namespace rb