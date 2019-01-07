#include <algorithm>
#include <chrono>
#include "RBControl_servo.hpp"

namespace rb {

SmartServoBus::SmartServoBus() : m_bus(NULL) {
}

void SmartServoBus::install(uint8_t servo_count, uart_port_t uart = UART_NUM_1, gpio_num_t pin = GPIO_NUM_32)
{
    m_mutex.lock();
    if(m_bus == NULL) {
        m_bus = new lw::Bus(uart, pin);
        for(uint8_t i = 0; i < servo_count; ++i) {
            m_servos.push_back(servo_info(m_bus->getServo(i+1)));
        }
    }
    m_mutex.unlock();
}

void SmartServoBus::set(uint8_t id, float angle, float speed, float speed_raise) {
    speed = std::max(1.f, std::min(240.f, speed)) / 10.f;
    angle = std::max(0.f, std::min(360.f, angle)) * 100;

    m_mutex.lock();
    auto& si = m_servos[id];
    if(si.current != angle) {
        if(si.target == 0xFFFF) {
            si.current = angle + 1;
        } else if((si.current > si.target) != (si.current > angle)) {
            si.speed_coef = 0.f;
        }
        si.target = angle;
        si.speed_target = speed;
        si.speed_raise = speed_raise;
    }
    m_mutex.unlock();
}

void SmartServoBus::limit(uint8_t id,  Angle b, Angle t) {
    m_mutex.lock();
    m_servos[id].servo.limit(b, t);
    m_mutex.unlock();
}

void SmartServoBus::update(uint32_t diff_ms) {
    int i =0;
    m_mutex.lock();
    for(auto &s : m_servos) {
        i++;
        if(s.current == s.target)
            continue;

        float speed = s.speed_target;
        if(s.speed_coef < 1.f) {
            s.speed_coef = std::min(1.f, s.speed_coef + s.speed_raise);
            speed *= (s.speed_coef * s.speed_coef);
        }

        int32_t dist = abs(int32_t(s.target) - int32_t(s.current));
        dist = std::max(1, std::min(dist, int32_t(speed * diff_ms)));
        if(dist > 0) {
            if (s.target < s.current) {
                s.current -= dist;
            } else {
                s.current += dist;
            }
        }

        if(dist <= 0 || s.current == s.target) {
            s.current = s.target;
            s.speed_coef = 0.f;
        }

        //printf("%d %d %f %f %f %u\n", dist, s.current, speed, s.speed_target, s.speed_coef, diff_ms);
        s.servo.move(Angle::deg(float(s.current)/100.f), std::chrono::milliseconds(diff_ms));
    }
    m_mutex.unlock();
}

}; // namespace rb