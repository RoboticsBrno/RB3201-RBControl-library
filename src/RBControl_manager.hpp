#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <vector>
#include <memory>
#include <mutex>
#include <list>
#include <functional>

#include "RBControl_battery.hpp"
#include "RBControl_piezo.hpp"
#include "Adafruit_MCP23017.h"
#include "RBControl_leds.hpp"
#include "RBControl_encoder.hpp"
#include "RBControl_servo.hpp"
#include "RBControl_motor.hpp"

namespace rb {

class MotorChangeBuilder;

/**
 * \brief The main library class for working with the RBControl board. 
 *        Keep an instance of it through the whole program.
 */
class Manager {
    friend class MotorChangeBuilder;
    friend class Encoder;
    friend class PcntInterruptHandler;
public:

    /**
     * \brief Constructor of class Manager - main class for working with the RBControl board.
     * 
     * The `enable_motor_failsafe` parameter toggles the automatic failsafe, where the manager
     * will automatically turn all motors off if it does not receive any motor set calls
     * in 300ms. This is to stop the robot when the controller is disconnected.
     * 
     * \param enable_motor_failsafe `true` activate the automatic failsafe, `false` deactivate this system
     */
    Manager(bool enable_motor_failsafe = true);
    ~Manager();

    /**
     * \brief Initialize the UART servo bus for intelligent servos LX-16.
     * \return Instance of the class {@link SmartServoBus} which manage the intelligent servos.
     */
    SmartServoBus& initSmartServoBus(uint8_t servo_count, uart_port_t uart = UART_NUM_1, gpio_num_t pin = GPIO_NUM_32);
    /**
     * \brief Get the {@link SmartServoBus} for working with intelligent servos LX-16..
     * \return Instance of the class {@link SmartServoBus} which manage the intelligent servos.
     */
    SmartServoBus& servoBus() { return m_servos; };

    Adafruit_MCP23017& expander() { return m_expander; } //!< Get the expander {@link Adafruit_MCP23017}. LEDs and buttons are connected to it.
    Piezo& piezo() { return m_piezo; } //!< Get the {@link Piezo} controller
    Battery& battery() { return m_battery; } //!< Get the {@link Battery} interface
    Leds& leds() { return m_leds; } //!< Get the {@link Leds} helper

    Motor* motor(MotorId id) { return m_motors[static_cast<int>(id)]; }; //!< Get a motor instance 
    MotorChangeBuilder setMotors(); //!< Create motor power change builder: {@link MotorChangeBuilder}.

    /**
     * \brief Schedule callback to fire after period (in millisecond). 
     * 
     * Return true from the callback to schedule periodically, false to not (singleshot timer).
     * 
     * \param period_ms is period in which will be the schedule callback fired
     * \param callback is a function which will be schedule with the set period.
     */
    void schedule(uint32_t period_ms, std::function<bool()> callback);

private:
    enum EventType {
        EVENT_MOTORS,
        EVENT_MOTORS_STOP_ALL,
        EVENT_ENCODER_EDGE,
        EVENT_ENCODER_PCNT,
    };


    struct EventMotorsData {
        bool (Motor::*setter_func)(int8_t);
        MotorId id;
        int8_t value;
    };

    struct Event {
        EventType type;
        union {
            std::vector<EventMotorsData> *motors;

            struct {
                int64_t timestamp;
                MotorId id;
                uint8_t pinLevel;
            } encoderEdge;

            struct {
                uint32_t status;
                MotorId id;
            } encoderPcnt;
        } data;
    };

    struct Timer {
        uint32_t remaining;
        uint32_t period;
        std::function<bool()> callback;
    };

    void queue(const Event *event, bool toFront = false);
    bool queueFromIsr(const Event *event, bool toFront = false);
    static void consumerRoutineTrampoline(void *cookie);
    void consumerRoutine();
    void processEvent(struct Event *ev);

    bool motorsFailSafe();

    void setupExpander();

    QueueHandle_t m_queue;

    std::list<Timer> m_timers;
    std::recursive_mutex m_timers_mutex;

    struct timeval m_motors_last_set;
    std::vector<Motor*> m_motors;
    SerialPCM m_motors_pwm;

    Adafruit_MCP23017 m_expander;
    rb::Piezo m_piezo;
    rb::Leds m_leds;
    rb::Battery m_battery;
    rb::SmartServoBus m_servos;
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

    /**
     * \brief Set single motor power.
     * \param id of the motor (e.g. rb:MotorId::M1)
     * \param power of the motor <-100 - 100>
     **/
    MotorChangeBuilder& power(MotorId id, int8_t value);
    
    /**
     * \brief Limit motor index's power to percent.
     * \param id of the motor (e.g. rb:MotorId::M1)
     * \param percent of the maximal power of the motor <0 - 100>
     **/
    MotorChangeBuilder& pwmMaxPercent(MotorId id, int8_t percent); 
    
    /**
     * \brief Finish the changes and submit the events. 
     * \param toFront add this event to front of the event queue
     **/
    void set(bool toFront = false);

private:
    Manager& m_manager;
    std::unique_ptr<std::vector<Manager::EventMotorsData>> m_values;
};

} // namespace rb