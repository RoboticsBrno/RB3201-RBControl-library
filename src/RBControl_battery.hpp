#pragma once

#include <atomic>

#include "RBControl_piezo.hpp"
#include "RBControl_leds.hpp"

namespace rb {

class Manager;

class Battery {
    friend class Manager;

public:
    static const uint32_t VOLTAGE_MIN = 3300*2;
    static const uint32_t VOLTAGE_WARNING = 3500*2;
    static const uint32_t VOLTAGE_MAX = 4200*2;

    uint32_t pct() const;
    uint32_t voltageMv() const;

    void shutdown();

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