#include <driver/adc.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "RBControl_battery.hpp"
#include "RBControl_manager.hpp"
#include "RBControl_piezo.hpp"

#define TAG "RBControlBattery"

namespace rb {

const int DEFAULT_REF_VOLTAGE = 1100;
const int BATTERY_ADC_SAMPLES = 32;

Battery::Battery(rb::Piezo& piezo, rb::Leds& leds, Adafruit_MCP23017& expander)
    : m_piezo(piezo)
    , m_leds(leds)
    , m_expander(expander) {
    m_warningOn = false;
    m_emergencyShutdown = true;
    m_undervoltedCounter = 0;
    m_coef = 1.0f;
}

Battery::~Battery() {
}

void Battery::install(bool disableEmergencyShutdown) {
    m_emergencyShutdown = !disableEmergencyShutdown;

    bool calibrated = false;
    int vref = DEFAULT_REF_VOLTAGE;
    auto& cfg = Manager::get().config();
    if (cfg.existsInt("vref")) {
        vref = cfg.getInt("vref");
        calibrated = true;
    }

    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(BATT_ADC_CHANNEL, ADC_ATTEN_DB_0);
    esp_adc_cal_value_t calibType = esp_adc_cal_characterize(BATT_ADC_UNIT,
        ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, vref, &m_adcChars);
    if (calibType == ESP_ADC_CAL_VAL_DEFAULT_VREF && !calibrated) {
        ESP_LOGE(TAG, "No ADC calibration. Readings might be incorrect.");
    }

    updateVoltage();

    Manager::get().schedule(500, [&]() -> bool {
        updateVoltage();
        return true;
    });
}

void Battery::setFineTuneCoef(float coef) {
    if (coef < 0.5 || coef >= 1.5) {
        ESP_LOGE(TAG, "Too large or too small fine tuning coefficient. Ignoring.");
        return;
    }
    m_coef.store(coef);
    m_voltageMv.store(rawToMv(m_raw.load()));
}

float Battery::fineTuneCoef() const {
    return m_coef.load();
}

void Battery::shutdown() {
    ESP_LOGW(TAG, "Shutting down.");

    vTaskDelay(pdMS_TO_TICKS(500));

    m_expander.digitalWrite(EXPANDER_BOARD_POWER_ON, 0);
    // Shut down nearly everything and never wake up - necessary when ESP is
    // powered from USB
    esp_deep_sleep_start();
}

uint32_t Battery::raw() const {
    return m_raw.load();
}

uint32_t Battery::voltageMv() const {
    return m_voltageMv.load();
}

uint32_t Battery::pct() const {
    const auto mv = voltageMv();
    if (mv <= VOLTAGE_MIN) {
        return 0;
    } else if (mv >= VOLTAGE_MAX) {
        return 100;
    } else {
        return (float(mv - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN)) * 100.f;
    }
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

    const uint32_t voltage = rawToMv(adc_reading);
    m_raw.store(adc_reading);
    m_voltageMv.store(voltage);

    ESP_LOGD(TAG, "Battery is at %u mV (raw %u)", voltage, adc_reading);

    // Not connected to the battery
    if (voltage < 3000) {
        return;
    }

    if (voltage <= VOLTAGE_WARNING) {
        setWarning(!m_warningOn);
    } else if (m_warningOn) {
        setWarning(false);
    }

    if (voltage <= VOLTAGE_MIN) {
        ESP_LOGE(TAG, "Battery is at %umV (raw %u)", voltage, adc_reading);
        if (++m_undervoltedCounter >= 10) {
            if (m_emergencyShutdown)
                shutdown();
        }
    } else if (m_undervoltedCounter != 0) {
        --m_undervoltedCounter;
    }
}

uint32_t Battery::rawToMv(uint32_t rawVal) {
    return esp_adc_cal_raw_to_voltage(rawVal, &m_adcChars) * m_coef.load() / BATT_DIVIDER;
}

};
