#pragma once

#include <freertos/FreeRTOS.h>

#include <driver/i2s.h>
#include <i2s_parallel.h>

#include <initializer_list>
#include <vector>

namespace rb {

class SerialPWM {
public:
    typedef int value_type;

    SerialPWM(const int channels,
        const std::initializer_list<int> data_pins,
        const int latch_pin,
        const int clock_pin,
        const int test_pin = -1,
        const int frequency = 20000,
        const int i2s = 1);

    ~SerialPWM();

    value_type& operator[](size_t index);

    void update();

    static int resolution();

private:
    SerialPWM(const SerialPWM&) = delete;

    static volatile void* i2snum2struct(const int num);

    static constexpr int sc_buffers = 2;
    static constexpr int sc_resolution = 100;
    const int c_channels;
    const int c_bytes;
    volatile void* m_i2s; // m_i2s is actually i2s_dev_t*, but this is an anonymous struct in the Espressif header i2s_struct.h and that causes a compilation error
    i2s_parallel_buffer_desc_t* m_buffer_descriptors[sc_buffers];
    uint8_t* m_buffer[sc_buffers][sc_resolution];
    int m_active_buffer;
    std::vector<value_type> m_pwm;
};

} // namespace rb
