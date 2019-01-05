#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>

#include <esp_log.h>

#include "RBControl_manager.hpp"
#include "RBControl_battery.hpp"

#define TAG "RBControlManager"

#define EVENT_LOOP_PERIOD (10 / portTICK_PERIOD_MS)
#define MOTORS_FAILSAFE_PERIOD 300

static int diff_ms(timeval& t1, timeval& t2) {
    return (((t1.tv_sec - t2.tv_sec) * 1000000) +
            (t1.tv_usec - t2.tv_usec))/1000;
}

namespace rb {

Manager::Manager(bool enable_motor_failsafe) :
    m_expander(I2C_ADDR_EXPANDER, I2C_NUM_0, I2C_MASTER_SDA, I2C_MASTER_SCL),
    m_piezo(), m_leds(m_expander), m_battery(m_piezo, m_leds, m_expander), m_servos() {
    m_queue = xQueueCreate(32, sizeof(struct Event));

    m_motors_last_set.tv_sec = 0;
    if(enable_motor_failsafe) {
        schedule(MOTORS_FAILSAFE_PERIOD, &Manager::motorsFailSafe, this);
    }

    m_battery.scheduleVoltageUpdating(*this);

    setupExpander();

    for(int i = 0; i < static_cast<int>(MotorId::MAX); ++i) {
        m_encoders[i] = NULL;
    }

    xTaskCreate(&Manager::consumerRoutineTrampoline, "rbmanager_loop", 4096, this, 1, NULL);
}

Manager::~Manager() {
    vQueueDelete(m_queue);
}

void Manager::setupExpander() {
    m_expander.pinMode(EA0, GPIO_MODE_OUTPUT);
    m_expander.pinMode(EA1, GPIO_MODE_OUTPUT);
    m_expander.pinMode(EA2, GPIO_MODE_OUTPUT);
    m_expander.pinMode(EA3, GPIO_MODE_OUTPUT);
    m_expander.pinMode(EA4, GPIO_MODE_OUTPUT);
    m_expander.pinMode(EA5, GPIO_MODE_OUTPUT);
    m_expander.pinMode(EA6, GPIO_MODE_OUTPUT);
    m_expander.pinMode(EA7, GPIO_MODE_OUTPUT);

    m_expander.pinMode(SW1, GPIO_MODE_INPUT);
    m_expander.pullUp(SW1, 1);
    m_expander.pinMode(SW2, GPIO_MODE_INPUT);
    m_expander.pullUp(SW2, 1);
    m_expander.pinMode(SW3, GPIO_MODE_INPUT);
    m_expander.pullUp(SW3, 1);
}

void Manager::queue(const Event *ev, bool toFront) {
    if(!toFront) {
        while(xQueueSendToBack(m_queue, ev, 0) != pdTRUE)
            vTaskDelay(1);
    } else {
        while(xQueueSendToFront(m_queue, ev, 0) != pdTRUE)
            vTaskDelay(1);
    }

}

bool Manager::queueFromIsr(const Event *ev, bool toFront) {
    BaseType_t woken = pdFALSE;
    if(!toFront)
        xQueueSendToBackFromISR(m_queue, ev, &woken);
    else
        xQueueSendToFrontFromISR(m_queue, ev, &woken);
    return woken == pdTRUE;
}

void Manager::consumerRoutineTrampoline(void *cookie) {
    ((Manager*)cookie)->consumerRoutine();
}

void Manager::consumerRoutine() {
    struct Event ev;
    struct timeval tv_last, tv_now;

    gettimeofday(&tv_last, NULL);

    while(true) {
        while(xQueueReceive(m_queue, &ev, EVENT_LOOP_PERIOD) == pdTRUE) {
            processEvent(&ev);
        }

        gettimeofday(&tv_now, NULL);
        const uint32_t diff = diff_ms(tv_now, tv_last);
        tv_last = tv_now;

        m_servos.update(diff);

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
    }
}

void Manager::processEvent(struct Manager::Event *ev) {
    switch(ev->type) {
    case EVENT_MOTORS: {
        auto data = (std::vector<EventMotorsData>*)ev->data.motors;
        bool changed = false;
        for(const auto& m : *data) {
            if((m_motors.motor(m.id).*m.setter_func)(m.value)) {
                changed = true;
            }
        }
        if(changed) {
            m_motors.update();
        }
        delete data;

        gettimeofday(&m_motors_last_set, NULL);
        break;
    }
    case EVENT_MOTORS_STOP_ALL: {
        bool changed = false;
        for(MotorId id = MotorId::M1; id < MotorId::MAX; ++id) {
            if(m_motors.motor(id).power(0))
                changed = true;
        }
        if(changed)
            m_motors.update();
        break;
    }
    case EVENT_ENCODER_EDGE: {
        const auto& e = ev->data.encoderEdge;
        m_encoders[static_cast<int>(e.id)]->onEdgeIsr(e.timestamp, e.pinLevel);
        break;
    }
    case EVENT_ENCODER_PCNT: {
        const auto& e = ev->data.encoderPcnt;
        m_encoders[static_cast<int>(e.id)]->onPcntIsr(e.status);
        break;
    }
    }
}

void Manager::schedule(uint32_t period_ms, ManagerTimerCallback callback, void *cookie) {
    m_timers_mutex.lock();
    m_timers.emplace_back(Timer {
        .remaining = period_ms,
        .period = period_ms,
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
            const Event ev = { .type = EVENT_MOTORS_STOP_ALL, .data = {} };
            man->queue(&ev);
            man->m_motors_last_set.tv_sec = 0;
        }
    }
    return true;
}

void Manager::initEncoder(MotorId id) {
    const int iid = static_cast<int>(id);
    if(m_encoders[iid] == NULL) {
        m_encoders[iid] = new Encoder(*this, id);
        m_encoders[iid]->install();
    }
}

Encoder *Manager::encoder(MotorId id) const {
    const int iid = static_cast<int>(id);
    if(m_encoders[iid] == NULL) {
        ESP_LOGE(TAG, "Invalid Manager::encoder(%d) call, this encoder was not initialized. Did you forget to call Manager::initEncoder(%d)?", iid, iid);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return m_encoders[iid];
}

rb::SmartServoBus& Manager::initSmartServoBus(uint8_t servo_count, uart_port_t uart, gpio_num_t pin) {
    m_servos.install(servo_count, uart, pin);
    return m_servos;
}

MotorChangeBuilder Manager::setMotors() {
    return MotorChangeBuilder(*this);
}

void Manager::setMotorPower(MotorId id, int8_t speed) {
    MotorChangeBuilder b(*this);
    b.power(id, speed).set();
}

MotorChangeBuilder::MotorChangeBuilder(Manager &manager) : m_manager(manager) {
    m_values.reset(new std::vector<Manager::EventMotorsData>());
}

MotorChangeBuilder::MotorChangeBuilder(MotorChangeBuilder&& o) :
    m_manager(o.m_manager), m_values(std::move(o.m_values)) {
}

MotorChangeBuilder::~MotorChangeBuilder() {
}

MotorChangeBuilder& MotorChangeBuilder::power(MotorId id, int8_t value) {
    m_values->emplace_back(Manager::EventMotorsData{
        .setter_func = &Motor::power,
        .id = id,
        .value = value
    });
    return *this;
}

MotorChangeBuilder& MotorChangeBuilder::pwmMaxPercent(MotorId id, uint8_t percent) {
    m_values->emplace_back(Manager::EventMotorsData{
        .setter_func = &Motor::pwmMaxPercent,
        .id = id,
        .value = (int8_t)percent
    });
    return *this;
}

void MotorChangeBuilder::set(bool toFront) {
    const Manager::Event ev = {
        .type = Manager::EVENT_MOTORS,
        .data = {
            .motors = m_values.release(),
        },
    };
    m_manager.queue(&ev, toFront);
}

};
