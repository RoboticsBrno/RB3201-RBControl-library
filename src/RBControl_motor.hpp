#pragma once

#include "serialpcm.hpp"
#include "RBControl_util.hpp"

namespace rb {

class Motor{
public: 
    Motor(SerialPCM::value_type & pwm0, 
          SerialPCM::value_type & pwm1,
          int index) 
        : pwm0(pwm0), 
          pwm1(pwm1),
          index(index),
          pwm_max(255),
          pwm_max_percent(100),
          power_max(100),
          pwm_scale(static_cast<float>(pwm_max) / power_max)
    {}

    void off(bool brake = true) {
        // TODO: brake
        pwm0 = 0;
        pwm1 = 0;
    }

    void power(int power) {
        power = power * pwm_scale;
        power = clamp(power, -pwm_max, pwm_max, "Motor", 
                      "{}: Clamp the power({} <-> {})", index, -pwm_max, pwm_max);
        
        if(power > 0) {
            pwm0 = 0;
            pwm1 = power;
        } else {
            pwm0 = -power;
            pwm1 = 0;
        }
        rb::log(INFO, "Motor", "{}: pwm0: {}   pwm1: {}", index, pwm0, pwm1);
    }

    void pwmMaxPercent(int percent) {
        pwm_max_percent = clamp(percent, 0, 100, "Motor", 
                                "{}: Clamp the pwmMaxPercent(0 - 100)",
                                index);
        pwm_scale = (static_cast<float>(pwm_max * pwm_max_percent) / 100) 
                    / power_max;
    }

private:
    SerialPCM::value_type & pwm0; 
    SerialPCM::value_type & pwm1;
    int index;
    int pwm_max;
    int pwm_max_percent;
    int power_max;
    float pwm_scale;
};

} // namespace rb