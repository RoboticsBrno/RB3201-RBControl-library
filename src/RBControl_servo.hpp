#pragma once

#include <vector>
#include <mutex>

#include <driver/gpio.h>
#include <driver/pcnt.h>
#include <driver/uart.h>

#include "RBControl_util.hpp"
#include "lx16a.hpp"

namespace rb {

class Manager;
class Encoder;

class SmartServoBus {
    friend class Manager;
public:
    SmartServoBus();
    ~SmartServoBus() {}

    void set(uint8_t id, float angle, float speed = 180.f, float speed_raise = 0.04f);
    void limit(uint8_t id,  Angle b, Angle t);

private:
    void install(uint8_t servo_count, uart_port_t uart, gpio_num_t pin);
    void update(uint32_t diff_ms);

    struct servo_info {
        servo_info(const lw::Bus::Servo& s) : servo(s) {
            current = 0xFFFF;
            target = 0xFFFF;
            speed_coef = 0.f;
            speed_target = 0.f;
            speed_raise = 0.f;
        }

        uint16_t current;
        uint16_t target;
        float speed_coef;
        float speed_target;
        float speed_raise;
        lw::Bus::Servo servo;
    };

    lw::Bus *m_bus;

    std::vector<servo_info> m_servos;
    std::mutex m_mutex;
};

};