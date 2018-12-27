#include <algorithm>
#include <chrono>
#include "RBControl_servo.hpp"

namespace rb {

ServoBus::ServoBus() : m_bus(NULL) {
}

void ServoBus::install(uint8_t servo_count, uart_port_t uart = UART_NUM_1, gpio_num_t pin = GPIO_NUM_32)
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

void ServoBus::set(uint8_t id, float angle, float speed) {
    speed = std::max(1.f, std::min(240.f, speed)) / 10.f;
    angle = std::max(0.f, std::min(360.f, angle)) * 100;

    m_mutex.lock();
    if(m_servos[id].target == 0xFFFF) {
        m_servos[id].current = angle + 1;
    }
    m_servos[id].target = angle;
    m_servos[id].speed = speed;
    m_mutex.unlock();
}

void ServoBus::limit(uint8_t id,  Angle b, Angle t) {
    m_mutex.lock();
    m_servos[id].servo.limit(b, t);
    m_mutex.unlock();
}

void ServoBus::update(uint32_t diff_ms) {
    int i =0;
    m_mutex.lock();
    for(auto &s : m_servos) {
        i++;
        if(s.current == s.target)
            continue;

        int32_t dist = abs(int32_t(s.target) - int32_t(s.current));
        dist = std::min(dist, int32_t(s.speed * diff_ms));
        if(dist <= 0) {
            s.current = s.target;
        } else if (s.target < s.current) {
            s.current -= dist;
        } else {
            s.current += dist;
        }
        s.servo.move(Angle::deg(float(s.current)/100.f), std::chrono::milliseconds(diff_ms));
        //printf("%d %u\n", s.current, diff_ms);
    }
    m_mutex.unlock();
}

}; // namespace rb