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

    uint32_t pct() const; //!< returns current battery percentage
    uint32_t voltageMv() const; //!< returns current battery voltage

    void shutdown(); //!< shuts the robot down

private:
    Battery(Piezo& piezo, Leds& leds);
    ~Battery();

    void scheduleVoltageUpdating(Manager& man);
    static bool updateVoltageTimer(void *);
    void updateVoltage();
    void setWarning(bool on);

    std::atomic<uint32_t> m_voltageMv;
    bool m_warningOn;
    uint8_t m_undervoltedCounter;
    Piezo& m_piezo;
    Leds& m_leds;
};

};