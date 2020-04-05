#pragma once

#include <atomic>

#include <esp_adc_cal.h>

#include "RBControl_leds.hpp"
#include "RBControl_piezo.hpp"

namespace rb {

class Manager;

/**
 * \brief Contains the battery state and can control the robot's power.
 */
class Battery {
    friend class Manager;

public:
    static constexpr uint32_t VOLTAGE_MIN = 3300 * 2; //!< Minimal battery voltage, in mV, at which the robot shuts down
    static constexpr uint32_t VOLTAGE_MAX = 4200 * 2; //!< Maximal battery voltage, in mV
    static constexpr uint32_t VOLTAGE_WARNING = 3500 * 2; //!< The voltage at which alert triggers
    static constexpr float BATT_DIVIDER = 10.0f / (82.0f + 10.0f); //!< Voltage divider ratio

    void setFineTuneCoef(float coef); //!< Tunes battery measurement to compensate e.g. for voltage divider error. Default 1, expected to be 0.5 to 1.5.
    float fineTuneCoef() const;

    uint32_t raw() const; //!< returns the raw value
    uint32_t pct() const; //!< returns current battery percentage
    uint32_t voltageMv() const; //!< returns current battery voltage

    void shutdown(); //!< shuts the robot down
private:
    Battery(Piezo& piezo, Leds& leds, Adafruit_MCP23017& expander);
    Battery(const Battery&) = delete;
    ~Battery();

    void install(bool disableEmergencyShutdown = false);

    void updateVoltage();
    void setWarning(bool on);
    uint32_t rawToMv(uint32_t rawVal);

    esp_adc_cal_characteristics_t m_adcChars;

    std::atomic<uint32_t> m_raw;
    std::atomic<uint32_t> m_voltageMv;
    std::atomic<float> m_coef;

    bool m_warningOn;
    bool m_emergencyShutdown;
    uint8_t m_undervoltedCounter;
    Piezo& m_piezo;
    Leds& m_leds;
    Adafruit_MCP23017& m_expander;
};

};
