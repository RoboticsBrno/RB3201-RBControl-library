#pragma once

#include "RBControl_util.hpp"

namespace rb {

/// @private
class Encoder{
public: 
    Encoder(int pin0, int pin1)
        : m_pin0(pin0), m_pin1(pin1)
    {}

    int speed() {
        return 0;
    }

    int position() {
        return 0;
    }

    void resetSpeed() {
        m_speed = 0;
    }

    void resetPosition() {
        m_position = 0;
    }

    void process() {

    }

private:
    int m_pin0; 
    int m_pin1;
    int m_speed = 0;
    int m_position = 0;
};

} // namespace rb
