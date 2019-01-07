#include "RBControl_motor.hpp"
#include "RBControl_encoder.hpp"
#include "RBControl_manager.hpp"

namespace rb {

#define PWM_MAX 255
#define POWER_MAX 100

Motor::Motor(Manager& man, MotorId id, SerialPCM::value_type & pwm0, SerialPCM::value_type & pwm1) :
    m_man(man), m_pwm0(pwm0), m_pwm1(pwm1), m_id(id), m_pwm_max_percent(100),
    m_pwm_scale(static_cast<float>(PWM_MAX) / POWER_MAX)
{
    m_power = 0;
    direct_power(m_power);
}

bool Motor::direct_power(int8_t power) {
    m_power = power;

    int pwm_val = int(power) * m_pwm_scale;
    pwm_val = clamp(pwm_val, -PWM_MAX, PWM_MAX, "Motor",
                  "{}: Clamp the power({} <-> {})", static_cast<int>(m_id), -PWM_MAX, PWM_MAX);

    if(power == 0) {
        if(m_pwm0 == 0 && m_pwm1 == 0)
            return false;
        m_pwm0 = m_pwm1 = 0;
    } else if(power > 0) {
        if(m_pwm1 == power)
            return false;
        m_pwm0 = 0;
        m_pwm1 = power;
    } else {
        if(m_pwm0 == -power)
            return false;
        m_pwm0 = -power;
        m_pwm1 = 0;
    }
    return true;
}

bool Motor::direct_pwmMaxPercent(int8_t percent) {
    const int8_t new_max_percent = clamp(percent, int8_t(0), int8_t(100), "Motor",
                            "{}: Clamp the pwmMaxPercent(0 - 100)",
                            static_cast<int>(m_id));

    if(new_max_percent == m_pwm_max_percent)
        return false;
    m_pwm_max_percent = new_max_percent;
    m_pwm_scale = (static_cast<float>(PWM_MAX * m_pwm_max_percent) / 100) / POWER_MAX;
    return true;
}

void Motor::power(int8_t value) {
    m_man.setMotors().power(m_id, value).set();
}

void Motor::pwmMaxPercent(int8_t percent) {
    m_man.setMotors().pwmMaxPercent(m_id, percent).set();
}

void Motor::driveToValue(int32_t positionAbsolute, uint8_t power, std::function<void(Encoder&)> callback) {
    encoder()->driveToValue(positionAbsolute, power, callback);
}

void Motor::drive(int32_t positionRelative, uint8_t power, std::function<void(Encoder&)> callback) {
    encoder()->drive(positionRelative, power, callback);
}

Encoder *Motor::encoder() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if(!m_encoder) {
        m_encoder.reset(new Encoder(m_man, m_id));
        m_encoder->install();
    }
    return m_encoder.get();
}

}; // namespace rb 
