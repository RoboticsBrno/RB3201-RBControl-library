#pragma once

#include "serialpcm.hpp"
#include "RBControl_util.hpp"

namespace rb {

/// @private
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
          _invert(1),
          pwm_scale(static_cast<float>(pwm_max) / power_max)
    {
        _power = 0;
        power(_power);
    }

    void off(bool brake = true) {
        // TODO: brake
        pwm0 = 0;
        pwm1 = 0;
    }

    bool power(int power) {
        _power = power;
        power = power * pwm_scale * _invert;
        power = clamp(power, -pwm_max, pwm_max, "Motor", 
                      "{}: Clamp the power({} <-> {})", index, -pwm_max, pwm_max);

        if(power == 0) {
            if(pwm0 == 0 && pwm1 == 0)
                return false;
            pwm0 = pwm1 = 0;
        } else if(power > 0) {
            if(pwm1 == power)
                return false;
            pwm0 = 0;
            pwm1 = power;
        } else {
            if(pwm0 == -power)
                return false;
            pwm0 = -power;
            pwm1 = 0;
        }
        return true;
    }
 
    int power() {
        return _power;
    }

    bool pwmMaxPercent(int percent) {
        const int new_max_percent = clamp(percent, 0, 100, "Motor", 
                                "{}: Clamp the pwmMaxPercent(0 - 100)",
                                index);
        if(new_max_percent == pwm_max_percent)
            return false;
        pwm_max_percent = new_max_percent;
        pwm_scale = (static_cast<float>(pwm_max * pwm_max_percent) / 100) 
                    / power_max;
        return true;
    }

    void invert(bool invert = true) {
        if(invert) {
            _invert = -1;
        } else {
            _invert = 1;
        }
    }

private:
    SerialPCM::value_type & pwm0; 
    SerialPCM::value_type & pwm1;
    int index;
    int pwm_max;
    int pwm_max_percent;
    int _power;
    int power_max;
    bool _invert;
    float pwm_scale;
};

} // namespace rb
