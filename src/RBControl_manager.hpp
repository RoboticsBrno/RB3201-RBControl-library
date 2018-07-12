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
#include "RBControl_encoder.hpp"

namespace rb {

class MotorChangeBuilder;

/**
 * \brief The callback type for schedule method. Return true to schedule again.
 */
typedef bool (*ManagerTimerCallback)(void *cookie);

/**
 * \brief The main library class. Keep an instance of it through the whole program.
 */
class Manager {
    friend class MotorChangeBuilder;
    friend class Encoder;
    friend class PcntInterruptHandler;
public:
    Manager();
    ~Manager();

    void initEncoder(uint8_t index);
    Encoder *encoder(uint8_t index) const { return m_encoders[index]; }

    Adafruit_MCP23017& expander() { return m_expander; } //!< Get the expander. LEDs and buttons are connected to it.
    Piezo& piezo() { return m_piezo; } //!< Get the piezo controller
    Battery& battery() { return m_battery; } //!< Get the battery interface
    Leds& leds() { return m_leds; } //!< Get the LEDs helper

    MotorChangeBuilder setMotors(); //!< Create motor power change builder. 
    void setMotorPower(uint8_t id, int8_t speed); //!< Set single motor power.

    /**
     * Schedule callback to fire after period ms. Return true from the callback
     * to schedule periodically, false to not (singleshot timer).
     */
    void schedule(uint32_t period, ManagerTimerCallback callback, void *cookie);

private:
    enum EventType {
        EVENT_MOTORS,
        EVENT_MOTORS_STOP_ALL,
        EVENT_ENCODER_EDGE,
        EVENT_ENCODER_PCNT,
    };


    struct EventMotorsData {
        bool (Motor::*setter_func)(int);
        uint8_t id;
        int8_t value;
    };

    struct Event {
        EventType type;
        union {
            std::vector<EventMotorsData> *motors;

            struct {
                int64_t timestamp;
                uint8_t index;
                uint8_t pinLevel;
            } encoderEdge;

            struct {
                uint32_t status;
                uint8_t index;
            } encoderPcnt;
        } data;
    };

    struct Timer {
        uint32_t remaining;
        uint32_t period;
        ManagerTimerCallback callback;
        void *cookie;
    };

    void queue(const Event *event, bool toFront = false);
    bool queueFromIsr(const Event *event, bool toFront = false);
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

    rb::Encoder *m_encoders[Encoder::COUNT];
};

/**
 * \brief Helper class for building the motor change event
 */
class MotorChangeBuilder {
public:
    MotorChangeBuilder(Manager& manager);
    MotorChangeBuilder(const MotorChangeBuilder& o) = delete;
    MotorChangeBuilder(MotorChangeBuilder&& o);
    ~MotorChangeBuilder();

    MotorChangeBuilder& power(uint8_t id, int8_t value); //!< Set current motor power to value for motor id
    MotorChangeBuilder& pwmMaxPercent(uint8_t id, uint8_t pct); //!< Limit motor id's power to pct
    void set(bool toFront = false); //!< Finish the changes and submit them

private:
    Manager& m_manager;
    std::unique_ptr<std::vector<Manager::EventMotorsData>> m_values;
};

} // namespace rb