#pragma once

#include <atomic>
#include <memory>
#include <functional>

#include "RBControl_serialPWM.hpp"
#include "RBControl_util.hpp"
#include "RBControl_pinout.hpp"
#include "RBControl_encoder.hpp"
#include "RBControl_regulator.hpp"
#include "RBControl_wheel.hpp"

namespace rb {

class Manager;
class MotorChangeBuilder;

class Motor {
    friend class Manager;
    friend class MotorChangeBuilder;
public:

    /**
     * \brief Set motor power.
     * \param power of the motor <-100 - 100>
     */
    void power(int8_t value);

    /**
     * \brief Limit the maximum PWM value. If you call pwmMaxPercent(70) and then
     * power(100), the motors will spin at 70% of maximum speed.
     * \param pct of the max value <0 - 100>
     */
    void pwmMaxPercent(int8_t percent);

    /**
     * \brief Stop motor.
     */
    void stop();

    /**
     * \brief Drive motor to set position (according absolute value). See {@link Encoder::driveToValue}. 
     */
    void driveToValue(Wheel::pos_type positionAbsolute, Wheel::speed_type speed, Wheel::callback_type callback);
    /**
     * \brief Drive motor to set position (according relative value). See {@link Encoder::drive}.
     */
    void drive(Wheel::pos_type positionRelative, Wheel::speed_type speed, Wheel::callback_type callback);

    /**
     * \brief Get the Encoder instance for this motor. See {@link Encoder}.
     */
    Encoder *encoder();

    /**
     * \brief Get the Encoder instance for this motor. Same as {@link encoder}.
     */
    Encoder *enc() { return encoder(); }

    /**
     * \brief Get the Regulator instance for this motor. See {@link Regulator}.
     */
    Regulator *regulator();

    /**
     * \brief Get the Regulator instance for this motor. Same as {@link reulator}.
     */
    Regulator *reg() { return regulator(); }

    /**
     * \brief Get the Wheel instance for this motor. See {@link Wheel}.
     */
    Wheel *wheel();

    /**
     * \brief Sync virtual wheel with physical encoder.
     */
    void syncWheel();

private:
    Motor(Manager& man, MotorId id, SerialPWM::value_type & pwm0, SerialPWM::value_type & pwm1);

    bool direct_power(int8_t power);
    bool direct_pwmMaxPercent(int8_t percent);
    bool direct_stop(int8_t);

    void init_encoder();
    void init_wheel();

    Manager& m_man;

    SerialPWM::value_type & m_pwm0;
    SerialPWM::value_type & m_pwm1;
    MotorId m_id;

    std::mutex m_mutex;
    std::unique_ptr<Encoder> m_encoder;
    std::unique_ptr<Regulator> m_regulator;
    std::unique_ptr<Wheel> m_wheel;
    int8_t m_power;

    int8_t m_pwm_max_percent;
    float m_pwm_scale;

    static bool s_reg_init;
    static std::unique_ptr<MotorChangeBuilder> s_motorChangeBuilder;
    static int64_t s_wheel_process_time;
    static Wheel::time_type s_wheel_process_time_step;
};

} // namespace rb
