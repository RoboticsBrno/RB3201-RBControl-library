#ifndef _GLIBCXX_USE_C99
#define _GLIBCXX_USE_C99
#endif
#include <string>

#include "RBControl_motor.hpp"
#include "RBControl_encoder.hpp"
#include "RBControl_manager.hpp"

#include <esp_log.h>

#define TAG "RbMotor"

namespace rb {

#define PWM_MAX SerialPWM::resolution()
#define POWER_MAX 100

Motor::Motor(Manager& man, MotorId id, SerialPWM::value_type & pwm0, SerialPWM::value_type & pwm1) :
    m_man(man), m_pwm0(pwm0), m_pwm1(pwm1), m_id(id), m_pwm_max_percent(100),
    m_pwm_scale(static_cast<float>(PWM_MAX) / POWER_MAX)
{
    direct_power(0);
}

static int INV(int v) { return PWM_MAX - v; }

bool Motor::direct_power(int8_t power) {
    m_power = power;

    int pwm_val = int(power) * m_pwm_scale;
    pwm_val = clamp(pwm_val, -PWM_MAX, PWM_MAX, "Motor",
                  "{}: Clamp the power({} <-> {})", static_cast<int>(m_id), -PWM_MAX, PWM_MAX);
    
    if(power == 0) {
        if(m_pwm0 == INV(0) && m_pwm1 == INV(0))
            return false;
        m_pwm0 = m_pwm1 = INV(0);
    } else if(power > 0) {
        if(m_pwm1 == INV(pwm_val))
            return false;
        m_pwm0 = INV(0);
        m_pwm1 = INV(pwm_val);
    } else {
        if(m_pwm0 == INV(-pwm_val))
            return false;
        m_pwm0 = INV(-pwm_val);
        m_pwm1 = INV(0);
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

bool Motor::direct_stop(int8_t) {
    if(m_pwm0 == INV(PWM_MAX) && m_pwm1 == INV(PWM_MAX))
        return false;
    m_pwm0 = m_pwm1 = INV(PWM_MAX);
    return true;
}

void Motor::stop() {
    m_man.setMotors().stop(m_id).set();
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
    init_encoder();
    return m_encoder.get();
}

Regulator *Motor::regulator() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if(!m_regulator) {
        if (!s_reg_init) {
            s_reg_init = true;
            Regulator::add_preprocessor([&]() {
                if (s_motorChangeBuilder)
                    ESP_LOGW(TAG, "Motors regulator init called with instance of the MotorChangeBuilder!");
                s_motorChangeBuilder.reset(new MotorChangeBuilder(m_man));
            } );
            Regulator::add_postprocessor([&]() {
                if (!s_motorChangeBuilder) {
                    ESP_LOGE(TAG, "Motors regulator setter called without valid instance of the MotorChangeBuilder!");
                    return;
                }
                s_motorChangeBuilder->set();
                s_motorChangeBuilder.reset();
            } );
        }
        init_encoder();
        // (std::ostringstream("Motor[")<<static_cast<int>(m_id)<<"]").str() // My compiler does not know std::ostringstream::str()
        m_regulator.reset(new Regulator("Motor[" + std::to_string(static_cast<int>(m_id)) + "]"));
        m_regulator->install(
            [&]() -> Regulator::Num { return encoder()->value(); },
            [&](Regulator::Num pwr) {
                if (!s_motorChangeBuilder) {
                    ESP_LOGE(TAG, "Motor[%d] regulator setter called without valid instance of the MotorChangeBuilder!", static_cast<int>(m_id));
                    return;
                }
                s_motorChangeBuilder->power(m_id, int8_t(pwr));
            }
        );
        m_regulator->set_max_output(PWM_MAX);
        m_regulator->set_params(1000, 100, 0);
    }
    return m_regulator.get();
}

void Motor::init_encoder() {
    if(!m_encoder) {
        m_encoder.reset(new Encoder(m_man, m_id));
        m_encoder->install();
    }
}

bool Motor::s_reg_init = false;
std::unique_ptr<MotorChangeBuilder> Motor::s_motorChangeBuilder;

}; // namespace rb 
