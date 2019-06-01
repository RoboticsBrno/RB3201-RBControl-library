#include "RBControl_wheel.hpp"
#include <freertos/FreeRTOS.h>

#include <cmath>

namespace rb {

Wheel::Wheel()
    : m_mutex(),
      m_accel(1),
      m_speed(1),
      m_target_speed(0),
      m_final_speed(0),
      m_pos(0),
      m_target_pos(0),
      m_decel_pos(0),
      m_pos_ths(0),
      m_on_position(false),
      m_on_position_callback()
{}

Wheel::pos_type Wheel::process(time_type dt) {
    m_mutex.lock();
    if (std::isnan(m_target_pos)) {
        m_mutex.unlock();
        return 0;
    }
    update(m_speed, m_target_speed, m_accel, dt);
    bool presign = std::signbit(m_pos - m_decel_pos);
    update(m_pos, m_target_pos, m_speed, dt);
    printf("process a %f v %f p %f dt %f\n", m_accel, m_speed, m_pos, dt);
    bool postsign = std::signbit(m_pos - m_decel_pos);
    if (presign != postsign)
        m_target_speed = m_final_speed;
    callback_type callback;
    if (!m_on_position && std::fabs(m_target_pos - m_pos) <= m_pos_ths) {
        callback = m_on_position_callback;
        m_on_position = true;
    }
    m_mutex.unlock();
    if (callback)
        callback(*this);
    return m_pos;
}

void Wheel::acceleration(accel_type a) {
    std::lock_guard<std::mutex>guard(m_mutex);
    m_accel = std::fabs(a);
    calc_deceleration_point();
}
Wheel::accel_type Wheel::acceleration() {
    std::lock_guard<std::mutex>guard(m_mutex);
    return m_accel;
}

void Wheel::speed(speed_type v) {
    std::lock_guard<std::mutex>guard(m_mutex);
    m_target_speed = std::fabs(v);
    if (_is_speed_mode())
        m_target_pos = copysign(m_target_pos, v);
    else
        calc_deceleration_point();
}
Wheel::speed_type Wheel::speed(bool absolute = false) {
    std::lock_guard<std::mutex>guard(m_mutex);
    return sign_speed(m_target_speed, absolute);
}
Wheel::speed_type Wheel::current_speed(bool absolute = false) {
    std::lock_guard<std::mutex>guard(m_mutex);
    return sign_speed(m_speed, absolute);
}

void Wheel::final_speed(speed_type v) {
    std::lock_guard<std::mutex>guard(m_mutex);
    m_final_speed = std::fabs(v);
    calc_deceleration_point();
}
Wheel::speed_type Wheel::final_speed(bool absolute = false) {
    std::lock_guard<std::mutex>guard(m_mutex);
    return sign_speed(m_final_speed, absolute);
}

void Wheel::position(pos_type p) {
    std::lock_guard<std::mutex>guard(m_mutex);
    m_target_pos = p;
    m_on_position = false;
    if (!_is_speed_mode())
        calc_deceleration_point();
}
Wheel::pos_type Wheel::position() {
    std::lock_guard<std::mutex>guard(m_mutex);
    return m_target_pos;
}
Wheel::pos_type Wheel::current_position() {
    std::lock_guard<std::mutex>guard(m_mutex);
    return m_pos;
}

void Wheel::set_pos_threshold(pos_type ths) {
    std::lock_guard<std::mutex>guard(m_mutex);
    m_pos_ths = std::fabs(ths);
}

void Wheel::register_callback(callback_type callback) {
    std::lock_guard<std::mutex>guard(m_mutex);
    m_on_position_callback = callback;
}

void Wheel::driveToValue(pos_type positionAbsolute, speed_type speed, callback_type callback = nullptr) {
    this->speed(speed);
    this->register_callback(callback);
    this->position(positionAbsolute);
}
void Wheel::driveToValue(pos_type positionAbsolute, speed_type speed, accel_type acceleration, callback_type callback = nullptr, speed_type final_speed = 0) {
    this->acceleration(acceleration);
    this->final_speed(final_speed);
    this->driveToValue(positionAbsolute, speed, callback);
}
void Wheel::drive(pos_type positionRelative, speed_type speed, callback_type callback = nullptr) {
    this->driveToValue(this->position() + positionRelative, speed, callback);
}
void Wheel::drive(pos_type positionRelative, speed_type speed, accel_type acceleration, callback_type callback = nullptr, speed_type final_speed = 0) {
    this->driveToValue(this->position() + positionRelative, speed, acceleration, callback, final_speed);
}
void Wheel::drive(pos_type positionRelative) {
    this->position(this->position() + positionRelative);
}

bool Wheel::is_on_position() {
    std::lock_guard<std::mutex>guard(m_mutex);
    return m_on_position;
}

void Wheel::sync(pos_type pos) {
    std::lock_guard<std::mutex>guard(m_mutex);
    m_pos = pos;
}

void Wheel::speed_mode() {
    printf("speed_mode\n");
    position(INFINITY);
}

bool Wheel::is_speed_mode() {
    std::lock_guard<std::mutex>guard(m_mutex);
    return _is_speed_mode();
}

bool Wheel::_is_speed_mode() {
    return std::isinf(m_target_pos);
}

Wheel::speed_type Wheel::sign_speed(speed_type v, bool absolute) const {
    return absolute ? v : copysign(v, m_target_pos - m_pos);
}

void  Wheel::calc_deceleration_point() {
    pos_type s = calc_deceleration_distance();
    pos_type dist_left = std::fabs(m_target_pos - m_pos);
    if (s > dist_left) {
        m_final_speed = std::sqrt(std::pow(m_target_speed, 2) + copysign(2 * m_accel * dist_left, m_final_speed - m_target_speed));
        s = calc_deceleration_distance();
    }
    m_decel_pos = m_target_pos - copysign(s, m_target_pos - m_pos);
}

Wheel::pos_type Wheel::calc_deceleration_distance() const {
    time_type t = std::fabs(m_final_speed - m_target_speed) / m_accel;
    return 0.5 * m_accel * t * t + m_target_speed * t;
}

void Wheel::update(float& v, float target, float dv_max, float dt) {
    float dv = (target - v) * dt;
    float dv_abs = std::fabs(dv);
    if (dv_abs < dv_max * dt)
        v = target;
    else
        v += copysign(dv_max, dv);
}

} // namespace rb
