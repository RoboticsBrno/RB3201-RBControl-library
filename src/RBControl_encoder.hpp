#pragma once

#include <atomic>
#include <functional>

#include <driver/gpio.h>
#include <driver/pcnt.h>

#include "RBControl_util.hpp"
#include "RBControl_pinout.hpp"

namespace rb {

class Encoder;
class Manager;

class Encoder{
    friend class Manager;
    friend class Motor;
public:
    Encoder(Manager& man, MotorId id);
    ~Encoder();

    /**
     * \brief Drive motor to set position (according absolute value).
     * 
     * \param positionAbsolute absolute position on which the motor drive \n
     *        e.g. if the actual motor position (`value()`) is 1000 and the `positionAbsolute` is 100
     *        then the motor will go backward to position 100 
     * \param power maximal power of the motor when go to set position, allowed values: <0 - 100>
     * \param callback is a function which will be called after the motor arrives to set position `[optional]`
     */
    void driveToValue(int32_t positionAbsolute, uint8_t power, std::function<void(Encoder&)> callback = nullptr);
    /**
     * \brief Drive motor to set position (according relative value).
     * 
     * \param positionRelative relative position on which the motor drive \n
     *        e.g. if the actual motor position (`value()`) is 1000 and the `positionRelative` is 100
     *        then the motor will go to position 1100
     * \param power maximal power of the motor when go to set position, allowed values: <0 - 100>
     * \param callback is a function which will be call after the motor arrive to set position `[optional]`  
     */
    void drive(int32_t positionRelative, uint8_t power, std::function<void(Encoder&)> callback = nullptr);

    /**
     * \brief Get number of edges from encoder.
     * \return The number of counted edges from the first initialize 
     *         of the encoder {@link Manager::initEncoder}
     */
    int32_t value();

    /**
     * \brief Get number of edges per one second.
     * \return The number of counted edges after one second.
     */
    float speed();
private:
    static void IRAM_ATTR isrGpio(void* cookie);

    void install();

    void onEdgeIsr(int64_t timestamp, uint8_t pinLevel);
    void onPcntIsr(uint32_t status);

    void pcnt_init(pcnt_unit_t pcntUnit, gpio_num_t GPIO_A, gpio_num_t GPIO_B);

    Manager& m_manager;
    MotorId m_id;

    std::atomic<int32_t> m_counter;

    std::mutex m_time_mutex;
    int64_t m_counter_time_us_last;
    int64_t m_counter_time_us_diff;
    int32_t m_target;
    int8_t m_target_direction;
    std::function<void(Encoder&)> m_target_callback;
};

/// @private
class PcntInterruptHandler {
public:
    static PcntInterruptHandler& get(Manager *manager);

    void enable(int index);

private:
    PcntInterruptHandler(Manager *manager);
    ~PcntInterruptHandler();
    static void IRAM_ATTR isrHandler(void *cookie);
};

} // namespace rb
