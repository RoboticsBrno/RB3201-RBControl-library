#pragma once

#include <vector>
#include <atomic>

#include <driver/spi_master.h>

namespace rb {

class LineSensor {
    friend class Manager;
public:
    static constexpr int LEDS = 8;

    struct Config {
        Config(int freq = 1350000, spi_host_device_t spi_dev = HSPI_HOST,
            gpio_num_t pin_cs = GPIO_NUM_25, gpio_num_t pin_mosi = GPIO_NUM_33,
            gpio_num_t pin_miso = GPIO_NUM_32, gpio_num_t pin_sck = GPIO_NUM_26) {
            this->freq = freq;
            this->spi_dev = spi_dev;

            this->pin_cs = pin_cs;
            this->pin_mosi = pin_mosi;
            this->pin_miso = pin_miso;
            this->pin_sck = pin_sck;
        }

        int freq;
        spi_host_device_t spi_dev;

        gpio_num_t pin_cs;
        gpio_num_t pin_mosi;
        gpio_num_t pin_miso;
        gpio_num_t pin_sck;
    };

    esp_err_t install(const Config& pins = Config());

    esp_err_t read(std::vector<uint16_t>& results, uint8_t leds_mask = 0xFF);

private:
    LineSensor();
    ~LineSensor();

    std::atomic<bool> m_installed;
    spi_device_handle_t m_spi;
};

}; // namespace rb
