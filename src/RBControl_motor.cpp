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
        if(m_pwm1 == INV(pwm_val) && m_pwm0 == INV(0))
            return false;
        m_pwm0 = INV(0);
        m_pwm1 = INV(pwm_val);
    } else {
        if(m_pwm0 == INV(-pwm_val) && m_pwm1 == INV(0))
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
    if (m_regulator && m_regulator->is_enabled())
        m_regulator->disable();
    m_man.setMotors().stop(m_id).set();
}

void Motor::power(int8_t value) {
    if (m_regulator && m_regulator->is_enabled())
        m_regulator->disable();
    m_man.setMotors().power(m_id, value).set();
}

void Motor::pwmMaxPercent(int8_t percent) {
    if (m_regulator)
        m_regulator->set_max_output(percent);
    m_man.setMotors().pwmMaxPercent(m_id, percent).set();
}

void Motor::driveToValue(Wheel::pos_type positionAbsolute, Wheel::speed_type speed, Wheel::callback_type callback = nullptr) {
    wheel()->driveToValue(positionAbsolute, speed, callback);
}

void Motor::drive(Wheel::pos_type positionRelative, Wheel::speed_type speed, Wheel::callback_type callback = nullptr) {
    wheel()->drive(positionRelative, speed, callback);
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
                int64_t t = esp_timer_get_time();
                s_wheel_process_time_step = (t - s_wheel_process_time) / 1e6;
                s_wheel_process_time = t;
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
        init_wheel();
        // (std::ostringstream("Motor[")<<static_cast<int>(m_id)<<"]").str() // My compiler does not know std::ostringstream::str()
        m_regulator.reset(new Regulator("Motor[" + std::to_string(static_cast<int>(m_id)) + "]"));
        m_regulator->install(
            [&]() -> Regulator::Num {
                if (m_regulator->is_enabled())
                    m_regulator->set(m_wheel->process(s_wheel_process_time_step));
                return encoder()->value();
            },
            [&](Regulator::Num pwr) {
                if (!s_motorChangeBuilder) {
                    ESP_LOGE(TAG, "Motor[%d] regulator setter called without valid instance of the MotorChangeBuilder!", static_cast<int>(m_id));
                    return;
                }
                s_motorChangeBuilder->power(m_id, int8_t(pwr));
            }
        );
        m_regulator->set_max_output(PWM_MAX);
        m_regulator->set_params(700, 50, 10);
        m_regulator->set_zero_threshold(2/m_encoder->ticks_per_rev());
        m_regulator->set_sum_zero_coef(1);
        m_wheel->set_pos_threshold(2/m_encoder->ticks_per_rev());
    }
    return m_regulator.get();
}

Wheel *Motor::wheel() {
    regulator();
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_wheel.get();
}

void Motor::syncWheel() {
    wheel()->sync(encoder()->value());
}

void Motor::init_encoder() {
    if (!m_encoder) {
        m_encoder.reset(new Encoder(m_man, m_id));
        m_encoder->install();
    }
}

void Motor::init_wheel() {
    if (!m_wheel) {
        m_wheel.reset(new Wheel());
    }
}

bool Motor::s_reg_init = false;
std::unique_ptr<MotorChangeBuilder> Motor::s_motorChangeBuilder;
int64_t Motor::s_wheel_process_time = 0;
Wheel::time_type Motor::s_wheel_process_time_step = 0;

}; // namespace rb 
