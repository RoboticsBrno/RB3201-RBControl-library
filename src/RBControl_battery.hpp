#pragma once

#include <atomic>

#include "RBControl_piezo.hpp"
#include "RBControl_leds.hpp"

namespace rb {

class Manager;

/**
 * \brief Contains the battery state and can control the robot's power.
 */
class Battery {
    friend class Manager;

public: 
    static const uint32_t VOLTAGE_MIN = 3300*2; //!< Minimal battery voltage, in mV, at which the robot shuts down
    static const uint32_t VOLTAGE_MAX = 4200*2; //!< Maximal battery voltage, in mV
    static const uint32_t VOLTAGE_WARNING = 3500*2; //!< The voltage at which alert triggers

    void setCoef(float coef);
    float coef() const;

    uint32_t raw() const; //!< returns the raw value
    uint32_t pct() const; //!< returns current battery percentage
    uint32_t voltageMv() const; //!< returns current battery voltage

    void shutdown(); //!< shuts the robot down

private:
    Battery(Piezo& piezo, Leds& leds, Adafruit_MCP23017& expander);
    ~Battery();

    void scheduleVoltageUpdating(Manager& man);
    bool updateVoltageTimer();
    void updateVoltage();
    void setWarning(bool on);

    std::atomic<uint32_t> m_raw;
    std::atomic<uint32_t> m_voltageMv;
    std::atomic<float> m_coef;

    bool m_warningOn;
    uint8_t m_undervoltedCounter;
    Piezo& m_piezo;
    Leds& m_leds;
    Adafruit_MCP23017& m_expander;
};

};