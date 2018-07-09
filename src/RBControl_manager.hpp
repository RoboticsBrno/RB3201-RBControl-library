#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <vector>
#include <memory>
#include <mutex>
#include <list>

#include "_RBControl_motors.hpp"
#include "RBControl_battery.hpp"
#include "RBControl_piezo.hpp"
#include "Adafruit_MCP23017.h"
#include "RBControl_leds.hpp"

namespace rb {

class MotorChangeBuilder;

typedef bool (*ManagerTimerCallback)(void *cookie);

class Manager {
    friend class MotorChangeBuilder;
public:
    Manager();
    ~Manager();

    Adafruit_MCP23017& expander() { return m_expander; }
    Piezo& piezo() { return m_piezo; }
    Battery& battery() { return m_battery; }
    Leds& leds() { return m_leds; }

    MotorChangeBuilder setMotors();
    void schedule(uint32_t period, ManagerTimerCallback callback, void *cookie);

private:
    enum EventType {
        EVENT_MOTORS,
        EVENT_MOTORS_STOP_ALL,
    };

    struct Event {
        EventType type;
        void *cookie;
    };

    struct EventMotorsData {
        bool (Motor::*setter_func)(int);
        uint8_t id;
        int8_t value;
    };

    struct Timer {
        uint32_t remaining;
        uint32_t period;
        ManagerTimerCallback callback;
        void *cookie;
    };

    void queue(EventType type, void *cookie);
    static void consumerRoutineTrampoline(void *cookie);
    void consumerRoutine();
    void processEvent(struct Event *ev);

    static bool motorsFailSafe(void *cookie);

    void setupExpander();
    
    QueueHandle_t m_queue;

    std::list<Timer> m_timers;
    std::recursive_mutex m_timers_mutex;
    
    struct timeval m_motors_last_set;
    rb::Motors m_motors;

    Adafruit_MCP23017 m_expander;
    rb::Piezo m_piezo;
    rb::Leds m_leds;
    rb::Battery m_battery;
};

class MotorChangeBuilder {
public:
    MotorChangeBuilder(Manager& manager);
    MotorChangeBuilder(const MotorChangeBuilder& o) = delete;
    MotorChangeBuilder(MotorChangeBuilder&& o);
    ~MotorChangeBuilder();
    

    MotorChangeBuilder& power(uint8_t id, int8_t value);
    MotorChangeBuilder& pwmMaxPercent(uint8_t id, uint8_t pct);
    void set();

private:
    Manager& m_manager;
    std::unique_ptr<std::vector<Manager::EventMotorsData>> m_values;
};

} // namespace rb