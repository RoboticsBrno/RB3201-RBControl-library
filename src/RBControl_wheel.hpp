#pragma once

#include <mutex>
#include <functional>

namespace rb {

class Wheel {
public:
    typedef float speed_type;
    typedef float pos_type;
    typedef float accel_type;
    typedef float time_type;
    typedef std::function<void(Wheel&)> callback_type;

    Wheel();

    pos_type process(time_type dt);

    void acceleration(accel_type a);
    accel_type acceleration();

    void speed(speed_type v);
    speed_type speed(bool absolute);
    speed_type current_speed(bool absolute);

    void final_speed(speed_type v);
    speed_type final_speed(bool absolute);

    void position(pos_type p);
    pos_type position();
    pos_type current_position();

    void set_pos_threshold(pos_type ths);

    void register_callback(callback_type callback);

    void driveToValue(pos_type positionAbsolute, speed_type speed, callback_type callback);
    void driveToValue(pos_type positionAbsolute, speed_type speed, accel_type acceleration, callback_type callback, speed_type final_speed);
    void drive(pos_type positionRelative, speed_type speed, callback_type callback);
    void drive(pos_type positionRelative, speed_type speed, accel_type acceleration, callback_type callback, speed_type final_speed);
    void drive(pos_type positionRelative);

    bool is_on_position();

    void sync(pos_type pos);
    
    void speed_mode();
    bool is_speed_mode();

private:
    bool _is_speed_mode();

    speed_type sign_speed(speed_type v, bool absolute) const;
    void  calc_deceleration_point();
    pos_type calc_deceleration_distance() const;

    std::mutex m_mutex;
    accel_type m_accel;
    speed_type m_speed;
    speed_type m_target_speed;
    speed_type m_final_speed;
    pos_type m_pos;
    pos_type m_target_pos;
    pos_type m_decel_pos;
    pos_type m_pos_ths;
    bool m_on_position;
    callback_type m_on_position_callback;

    static void update(float& v, float target, float dv, float dt);

    Wheel(Wheel const&) = delete;
    void operator=(Wheel const&) = delete;
};

} // namespace rb
