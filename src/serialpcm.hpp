#pragma once

#include <freertos/FreeRTOS.h>
#include <i2s_parallel.h>
#include <driver/i2s.h>

#include <cstring>
#include <cstdint>
#include <initializer_list>
#include <vector>

// This file is part of RB3201-RBControl-library - https://github.com/RoboticsBrno/RB3201-RBControl-library
// Author: Jakub Streit

/// @private
class SerialPCM {
public:
    typedef int value_type;

    SerialPCM(const int channels,
              const std::initializer_list<int> data_pins,
              const int latch_pin,
              const int clock_pin,
              const int test_pin = -1,
              i2s_dev_t& i2s = I2S1)
        : c_channels (channels),
          c_bytes(((data_pins.size() + 1)>>3) + 1), // The first +1 is for latch pin
          m_i2s (&i2s),
          m_buffer_descriptors {nullptr},
          m_buffer {nullptr},
          m_active_buffer(0),
          m_pwm(channels * data_pins.size(), 0)
    {
        const int buffer_size = c_channels * c_bytes;
        for (int buffer = 0; buffer != sc_buffers; ++buffer) {
            m_buffer_descriptors[buffer] = static_cast<i2s_parallel_buffer_desc_t*>(heap_caps_malloc(sc_resolution * sizeof(i2s_parallel_buffer_desc_t), MALLOC_CAP_32BIT));
            for (int bit = 0; bit != sc_bit_depth; ++bit) {
                uint8_t* p_buffer = static_cast<uint8_t*>(heap_caps_malloc(buffer_size, MALLOC_CAP_DMA));
                for (int i = 0; i != (1<<bit); ++i) {
                    int j = (1<<(sc_bit_depth-bit-1));
                    j += (j<<1) * i - 1;
                    m_buffer_descriptors[buffer][j].memory = p_buffer;
                    m_buffer_descriptors[buffer][j].size = buffer_size;
                }
                memset(p_buffer, 0, buffer_size);
                p_buffer[0] = (1<<(data_pins.size()&7));
                m_buffer[buffer][bit] = p_buffer;
            }
            m_buffer_descriptors[buffer][sc_resolution-1].memory = nullptr;
            for (int i = 0; i != buffer_size; ++i)
                m_buffer[buffer][0][i] |= 1<<(data_pins.size() + 1);
        }

        i2s_parallel_config_t cfg;
        switch(c_bytes) {
            default: //print("!!! WRONG NUMBER OF BYTES ({}) !!! Fallback to one byte.\n", c_bytes);
            case 1: cfg.bits = I2S_PARALLEL_BITS_8 ; break;
            case 2: cfg.bits = I2S_PARALLEL_BITS_16; break;
            case 3: cfg.bits = I2S_PARALLEL_BITS_32; break;
        }

        int i = 0;
        for (int pin: data_pins)
            cfg.gpio_bus[i++] = pin;
        cfg.gpio_bus[i++] = latch_pin;
        if (test_pin != -1)
            cfg.gpio_bus[i++] = test_pin;
        for (; i != 24; ++i)
            cfg.gpio_bus[i] = -1;
        cfg.gpio_wclk = -1;
        cfg.inv_wclk = false;
        cfg.gpio_bclk = clock_pin;
        cfg.inv_bclk = false;
        cfg.clkspeed = 255;
        cfg.bufa = m_buffer_descriptors[0];
        cfg.bufb = m_buffer_descriptors[1];

        i2s_parallel_setup(m_i2s, &cfg);
        update();
    }

    ~SerialPCM() {
        i2s_driver_uninstall(static_cast<i2s_port_t>(i2snum(m_i2s)));
        for (int buffer = 0; buffer != sc_buffers; ++buffer) {
            heap_caps_free(m_buffer_descriptors[buffer]);
            m_buffer_descriptors[buffer] = nullptr;
            for (int bit = 0; bit != sc_bit_depth; ++bit) {
                heap_caps_free(m_buffer[buffer][bit]);
                m_buffer[buffer][bit] = nullptr;
            }
        }
    }

    value_type& operator [] (size_t index) { return m_pwm[index]; }

    void update() {
        m_active_buffer ^= 1;
        int i = m_pwm.size();
        for (int channel = 0; channel != m_pwm.size(); ++channel) {
            uint8_t mask = 1<<(channel / c_channels);
            const int pos = ((channel&7) | ((channel&~7)*c_bytes)) % c_channels;
            --i;
            for (int bit = 0; bit != sc_bit_depth; ++bit) {
                uint8_t& value = m_buffer[m_active_buffer][bit][pos];
                if (m_pwm[channel] & (1<<bit))
                    value |= mask;
                else
                    value &= ~mask;
            }
        }
        i2s_parallel_flip_to_buffer(m_i2s, m_active_buffer);
    }

    int resolution() const { return sc_resolution - 1; }

private:

    static const int sc_buffers = 2;
    static const int sc_bit_depth = 8;
    static const int sc_resolution = 1<<sc_bit_depth;
    const int c_channels;
    const int c_bytes;
    i2s_dev_t* m_i2s;
    i2s_parallel_buffer_desc_t* m_buffer_descriptors[sc_buffers];
    uint8_t* m_buffer[sc_buffers][sc_bit_depth];
    int m_active_buffer;
    std::vector<value_type> m_pwm;
};
