#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_log.h>

#include "RBControl_battery.hpp"
#include "RBControl_manager.hpp"
#include "RBControl_piezo.hpp"

#define BATTERY_ADC_SAMPLES 32
// first measure 8.580401194725383
#define BATTERY_COEF 8.3f

#define TAG "RBControlBattery"

namespace rb {

Battery::Battery(rb::Piezo& piezo, rb::Leds& leds) : m_piezo(piezo), m_leds(leds) {
    m_warningOn = false;
    m_undervoltedCounter = 0;

    gpio_set_level(POWER_OFF, 1);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << POWER_OFF);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(BATT_ADC_CHANNEL, ADC_ATTEN_DB_11);
}

Battery::~Battery() {

}

void Battery::shutdown() {
    ESP_LOGW(TAG, "Shutting down.");

    gpio_set_level(POWER_OFF, 0);
    while(true) { } // wait for poweroff
}

uint32_t Battery::voltageMv() const {
    return m_voltageMv.load();
}

uint32_t Battery::pct() const {
    const auto mv = voltageMv();
    if(mv <= VOLTAGE_MIN) {
        return 0;
    } else if(mv >= VOLTAGE_MAX) {
        return 100;
    } else {
        return (float(mv - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN)) * 100.f;
    }
}

void Battery::scheduleVoltageUpdating(Manager& man) {
    man.schedule(500, &Battery::updateVoltageTimer, this);
}

bool Battery::updateVoltageTimer(void *cookie) {
    ((Battery*)cookie)->updateVoltage();
    return true;
}

void Battery::setWarning(bool on) {
    m_warningOn = on;
    m_piezo.setTone(on ? 444 : 0);
    m_leds.red(on);
}

void Battery::updateVoltage() {
    uint32_t adc_reading = 0;
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        adc_reading += adc1_get_raw(BATT_ADC_CHANNEL);
    }
    adc_reading /= BATTERY_ADC_SAMPLES;

    const uint32_t voltage = adc_reading*BATTERY_COEF;
    m_voltageMv.store(voltage);

    ESP_LOGD(TAG, "Battery is at %u %u", adc_reading, voltage);

    if(voltage <= VOLTAGE_WARNING) {
        setWarning(!m_warningOn);
    } else if(m_warningOn) {
        setWarning(false);
    }

    if(voltage != 0 && voltage <= VOLTAGE_MIN) {
        ESP_LOGE(TAG, "Battery is at %u/%umv", adc_reading, voltage);
        if(++m_undervoltedCounter >= 10) {
            shutdown();
        }
    } else if(m_undervoltedCounter != 0) {
        --m_undervoltedCounter;
    }
}

};