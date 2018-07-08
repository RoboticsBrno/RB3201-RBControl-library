#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#include "RBControl_manager.hpp"

#define TAG "RBControlManager"

#define EVENT_LOOP_PERIOD (10 / portTICK_PERIOD_MS)
#define MOTORS_FAILSAFE_PERIOD 300

static int diff_ms(timeval& t1, timeval& t2) {
    return (((t1.tv_sec - t2.tv_sec) * 1000000) + 
            (t1.tv_usec - t2.tv_usec))/1000;
}

namespace rb {

Manager::Manager() {
    m_queue = xQueueCreate(32, sizeof(struct Event));

    m_motors_last_set.tv_sec = 0;
    schedule(MOTORS_FAILSAFE_PERIOD, &Manager::motorsFailSafe, this);

    xTaskCreate(&Manager::consumerRoutineTrampoline, "rbmanager_loop", 4096, this, 1, NULL);
}

Manager::~Manager() {
    vQueueDelete(m_queue);
}

void Manager::queue(EventType type, void *cookie) {
    struct Event ev = {
        .type = type,
        cookie = cookie,
    };

    while(xQueueSendToBack(m_queue, &ev, 0) != pdTRUE) {
        vTaskDelay(EVENT_LOOP_PERIOD);
    }
}

void Manager::consumerRoutineTrampoline(void *cookie) {
    ((Manager*)cookie)->consumerRoutine();
}

void Manager::consumerRoutine() {
    struct Event ev;
    struct timeval tv_last, tv_now;

    gettimeofday(&tv_last, NULL);

    while(true) {
        while(xQueueReceive(m_queue, &ev, 0) == pdTRUE) {
            processEvent(&ev);
        }

        gettimeofday(&tv_now, NULL);
        const uint32_t diff = diff_ms(tv_now, tv_last);
        tv_last = tv_now;

        m_timers_mutex.lock();
        for(auto itr = m_timers.begin(); itr != m_timers.end(); ) {
            if((*itr).remaining <= diff) {
                if(!(*itr).callback((*itr).cookie) || (*itr).period == 0) {
                    itr = m_timers.erase(itr);
                    continue;
                } else {
                    (*itr).remaining = (*itr).period;
                }
            } else {
                (*itr).remaining -= diff;
            }

            ++itr;
        }
        m_timers_mutex.unlock();

        vTaskDelay(EVENT_LOOP_PERIOD);
    }
}

void Manager::processEvent(struct Manager::Event *ev) {
    switch(ev->type) {
    case EVENT_MOTORS: {
        auto data = (std::vector<EventMotorsData>*)ev->cookie;
        bool changed = false;
        for(const auto& m : *data) {
            if((m_motors.motor(m.id).*m.setter_func)(m.value))
                changed = true;
        }
        if(changed)
            m_motors.update();
        delete data;

        gettimeofday(&m_motors_last_set, NULL);
        break;
    }
    case EVENT_MOTORS_STOP_ALL:
        const int count = m_motors.motorCount();
        bool changed = false;
        for(int i = 0; i < count; ++i) {
            if(m_motors.motor(i).power(0))
                changed = true;
        }
        if(changed)
            m_motors.update();
        break;
    }
}

void Manager::schedule(uint32_t period, ManagerTimerCallback callback, void *cookie) {
    m_timers_mutex.lock();
    m_timers.emplace_back(Timer {
        .remaining = period,
        .period = period,
        .callback = callback,
        .cookie = cookie,
    });
    m_timers_mutex.unlock();
}

bool Manager::motorsFailSafe(void *cookie) {
    Manager *man = (Manager*)cookie;
    if(man->m_motors_last_set.tv_sec != 0) {
        struct timeval now;
        gettimeofday(&now, NULL);
        if(diff_ms(now, man->m_motors_last_set) >= MOTORS_FAILSAFE_PERIOD) {
            ESP_LOGE(TAG, "Motor failsafe triggered, stopping all motors!");
            man->queue(EVENT_MOTORS_STOP_ALL, NULL);
            man->m_motors_last_set.tv_sec = 0;
        }
    }
    return true;
}

MotorChangeBuilder Manager::setMotors() {
    return MotorChangeBuilder(*this);
}

MotorChangeBuilder::MotorChangeBuilder(Manager &manager) : m_manager(manager) {
    m_values.reset(new std::vector<Manager::EventMotorsData>());
}

MotorChangeBuilder::MotorChangeBuilder(MotorChangeBuilder&& o) :
    m_manager(o.m_manager), m_values(std::move(o.m_values)) {
}

MotorChangeBuilder::~MotorChangeBuilder() {
}

MotorChangeBuilder& MotorChangeBuilder::power(uint8_t id, int8_t value) {
    m_values->emplace_back(Manager::EventMotorsData{
        .setter_func = &Motor::power,
        .id = id,
        .value = value
    });
    return *this;
}

MotorChangeBuilder& MotorChangeBuilder::pwmMaxPercent(uint8_t id, uint8_t pct) {
    m_values->emplace_back(Manager::EventMotorsData{
        .setter_func = &Motor::pwmMaxPercent,
        .id = id,
        .value = (int8_t)pct
    });
    return *this;
}

void MotorChangeBuilder::set() {
    m_manager.queue(Manager::EVENT_MOTORS, m_values.release());
}

};
