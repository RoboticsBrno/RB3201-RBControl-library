#pragma once

#include <atomic>
#include <memory>
#include <functional>

#include "serialpcm.hpp"
#include "RBControl_util.hpp"
#include "RBControl_pinout.hpp"
#include "RBControl_encoder.hpp"

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
     * \brief Drive motor to set position (according absolute value). See {@link Encoder::driveToValue}. 
     */
    void driveToValue(int32_t positionAbsolute, uint8_t power, std::function<void(Encoder&)> callback = nullptr);
    /**
     * \brief Drive motor to set position (according relative value). See {@link Encoder::drive}.
     */
    void drive(int32_t positionRelative, uint8_t power, std::function<void(Encoder&)> callback = nullptr);

    /**
     * \brief Get the Encoder instance for this motor. See {@link Encoder}.
     */
    Encoder *encoder();

    /**
     * \brief Get the Encoder instance for this motor. Same as {@link encoder}.
     */
    Encoder *enc() { return encoder(); }

private:
    Motor(Manager& man, MotorId id, SerialPCM::value_type & pwm0, SerialPCM::value_type & pwm1);

    bool direct_power(int8_t power);
    bool direct_pwmMaxPercent(int8_t percent);

    Manager& m_man;

    SerialPCM::value_type & m_pwm0;
    SerialPCM::value_type & m_pwm1;
    MotorId m_id;

    std::mutex m_mutex;
    std::unique_ptr<Encoder> m_encoder;
    int8_t m_power;

    int8_t m_pwm_max_percent;
    float m_pwm_scale;
};

} // namespace rb
