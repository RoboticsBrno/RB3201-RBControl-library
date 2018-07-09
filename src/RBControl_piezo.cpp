#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>

#include "RBControl_piezo.hpp"
#include "RBControl_pinout.hpp"

#define PIEZO_PIN_MASK  ((1ULL<<PIEZO_B) | (1ULL<<PIEZO_A))
#define LEDC_DIV_NUM_HSTIMER0_V  0x3FFFF

namespace rb {

Piezo::Piezo() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = PIEZO_PIN_MASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(PIEZO_B, 1);

    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;           // timer mode
    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT; // resolution of PWM duty
    ledc_timer.timer_num = LEDC_TIMER_0;            // timer index
    ledc_timer.freq_hz = 0;                      // frequency of PWM signal

    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num   = PIEZO_A,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
    };
    ledc_channel_config(&ledc_channel);

    setTone(0);
}

Piezo::~Piezo() {
}

void Piezo::setTone(uint32_t freq) {
    if(freq != 0) {
        uint8_t bit_num = 10;
        uint64_t clk_freq = APB_CLK_FREQ;
        clk_freq <<= 8; //div_num is 8 bit decimal
        uint32_t div_num = (clk_freq >> bit_num) / freq;
        bool apb_clk = true;
        if(div_num > LEDC_DIV_NUM_HSTIMER0_V) {
            clk_freq /= 80;
            div_num = (clk_freq >> bit_num) / freq;
            if(div_num > LEDC_DIV_NUM_HSTIMER0_V) {
                div_num = LEDC_DIV_NUM_HSTIMER0_V;//lowest clock possible
            }
            apb_clk = false;
        } else if(div_num < 256) {
            div_num = 256;//highest clock possible
        }

        ESP_LOGD("Piezo", "freq %u, apbclk %d, bit_num %d, div_num %d", freq, (int)apb_clk, (int)bit_num, (div_num));

        ledc_timer_set(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, div_num,
            bit_num, apb_clk ? LEDC_APB_CLK : LEDC_REF_TICK);

        ledc_timer_resume(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0);
        ledc_timer_rst(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0);

        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0x1FF);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    } else {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 1);
    }
}

};