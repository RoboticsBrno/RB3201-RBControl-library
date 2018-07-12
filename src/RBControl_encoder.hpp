#pragma once

#include <atomic>

#include <driver/gpio.h>
#include <driver/pcnt.h>

#include "RBControl_util.hpp"


namespace rb {

class Manager;
class Encoder;

/**
 * \brief The callback type for schedule method. Return true to schedule again.
 */
typedef void (*EncoderDoneCallback)(Encoder& enc, void *cookie);

class Encoder{
    friend class Manager;
public:
    static const int COUNT = 8;

    Encoder(Manager& man, uint8_t index);
    ~Encoder();

    void driveToValue(int32_t value, int8_t speed, EncoderDoneCallback callback, void *cookie);
    void drive(int32_t distance, int8_t speed, EncoderDoneCallback callback, void *cookie);

    int32_t value();
    float speed();
private:
    static void IRAM_ATTR isrGpio(void* cookie);

    void install();

    void onEdgeIsr(int64_t timestamp, uint8_t pinLevel);
    void onPcntIsr(uint32_t status);

    void pcnt_init(pcnt_unit_t pcntUnit, gpio_num_t GPIO_A, gpio_num_t GPIO_B);

    Manager& m_manager;
    uint8_t m_index;

    std::atomic<int32_t> m_counter;

    std::mutex m_time_mutex;
    int64_t m_counter_time_last;
    int64_t m_counter_time_diff;
    int32_t m_target;
    int8_t m_target_direction;
    void *m_target_cookie;
    EncoderDoneCallback m_target_callback;
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
