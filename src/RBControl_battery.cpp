#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_log.h>

#include "RBControl_battery.hpp"
#include "RBControl_manager.hpp"
#include "RBControl_piezo.hpp"

#define BATTERY_ADC_SAMPLES 32
#define BATTERY_DEFAULT_COEF 9.f

#define TAG "RBControlBattery"

namespace rb {

Battery::Battery(rb::Piezo& piezo, rb::Leds& leds, Adafruit_MCP23017& expander) : m_piezo(piezo), m_leds(leds), m_expander(expander) {
    m_warningOn = false;
    m_undervoltedCounter = 0;
    m_coef = BATTERY_DEFAULT_COEF;

    m_expander.digitalWrite(POWER_OFF_EXPANDER, 1);
    m_expander.pinMode(POWER_OFF_EXPANDER, GPIO_MODE_OUTPUT);

    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(BATT_ADC_CHANNEL, ADC_ATTEN_DB_11);

    updateVoltage();
}

Battery::~Battery() {

}

void Battery::setCoef(float coef) {
    m_coef.store(coef);
}

float Battery::coef() const {
    return m_coef.load();
}

void Battery::shutdown() {
    ESP_LOGW(TAG, "Shutting down.");

    m_expander.digitalWrite(POWER_OFF_EXPANDER, 0);
    while(true) { } // wait for poweroff
}

uint32_t Battery::raw() const {
    return m_raw.load();
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
    man.schedule(500, [&]()->bool{
        updateVoltage();
        return true;
    });
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

    const uint32_t voltage = adc_reading * m_coef.load();
    m_raw.store(adc_reading);
    m_voltageMv.store(voltage);

    ESP_LOGD(TAG, "Battery is at %u %u", adc_reading, voltage);

    // Not connected to the battery
    if(voltage < 1000) {
        return;
    }

    if(voltage <= VOLTAGE_WARNING) {
        setWarning(!m_warningOn);
    } else if(m_warningOn) {
        setWarning(false);
    }

    if(voltage <= VOLTAGE_MIN) {
        ESP_LOGE(TAG, "Battery is at %u/%umv", adc_reading, voltage);
        if(++m_undervoltedCounter >= 10) {
            shutdown();
        }
    } else if(m_undervoltedCounter != 0) {
        --m_undervoltedCounter;
    }
}

};