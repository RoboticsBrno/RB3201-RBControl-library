#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#include "RBControl_battery.hpp"
#include "RBControl_manager.hpp"

#define TAG "RBControlManager"

#define MOTORS_FAILSAFE_PERIOD_MS 300
#define MOTORS_CHANNELS 16

#ifndef MOTORS_PWM_FREQUENCY
#define MOTORS_PWM_FREQUENCY 10000
#endif

namespace rb {

Manager::Manager()
    : m_queue(nullptr)
    , m_motors_pwm { MOTORS_CHANNELS, { SERMOT }, RCKMOT, SCKMOT, -1, MOTORS_PWM_FREQUENCY }
    , m_expander(I2C_ADDR_EXPANDER, I2C_NUM_0, I2C_MASTER_SDA, I2C_MASTER_SCL)
    , m_piezo()
    , m_leds(m_expander)
    , m_battery(m_piezo, m_leds, m_expander)
    , m_servos()
    , m_config("rb") {
}

Manager::~Manager() {
    if (m_queue) {
        vQueueDelete(m_queue);
    }
}

void Manager::install(ManagerInstallFlags flags) {
    if (m_queue) {
        ESP_LOGE(TAG, "The manager has already been installed, please make sure to only call install() once!");
        abort();
    }

    m_queue = xQueueCreate(32, sizeof(struct Event));

    std::vector<int> pwm_index({ 12, 13, 2, 3, 8, 9, 14, 15, 4, 5, 10, 11, 1, 2, 6, 7 });
    assert(pwm_index.size() / 2 == static_cast<size_t>(MotorId::MAX));

    for (int index = 0; index < pwm_index.size(); index += 2) {
        m_motors.emplace_back(new Motor(*this, MotorId(index / 2), m_motors_pwm[pwm_index[index]], m_motors_pwm[pwm_index[index + 1]]));
    }

    m_motors_last_set = 0;
    if (!(flags & MAN_DISABLE_MOTOR_FAILSAFE)) {
        schedule(MOTORS_FAILSAFE_PERIOD_MS, std::bind(&Manager::motorsFailSafe, this));
    }

    setupExpander();

    if (!(flags & MAN_DISABLE_PIEZO)) {
        m_piezo.install();
    }

    m_battery.install(flags & MAN_DISABLE_BATTERY_MANAGEMENT);

    TaskHandle_t task;
    xTaskCreate(&Manager::consumerRoutineTrampoline, "rbmanager_loop", 3072, this, 5, &task);
    monitorTask(task);

#ifdef RB_DEBUG_MONITOR_TASKS
    schedule(10000, [&]() { return printTasksDebugInfo(); });
#endif
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

    // This pin keeps the board in the ON state.
    m_expander.digitalWrite(EXPANDER_BOARD_POWER_ON, 1);
    m_expander.pinMode(EXPANDER_BOARD_POWER_ON, GPIO_MODE_OUTPUT);
}

void Manager::queue(const Event* ev, bool toFront) {
    if (!toFront) {
        while (xQueueSendToBack(m_queue, ev, 0) != pdTRUE)
            vTaskDelay(1);
    } else {
        while (xQueueSendToFront(m_queue, ev, 0) != pdTRUE)
            vTaskDelay(1);
    }
}

bool Manager::queueFromIsr(const Event* ev, bool toFront) {
    BaseType_t woken = pdFALSE;
    if (!toFront)
        xQueueSendToBackFromISR(m_queue, ev, &woken);
    else
        xQueueSendToFrontFromISR(m_queue, ev, &woken);
    return woken == pdTRUE;
}

void Manager::consumerRoutineTrampoline(void* cookie) {
    ((Manager*)cookie)->consumerRoutine();
}

void Manager::consumerRoutine() {
    struct Event ev;
    while (true) {
        while (xQueueReceive(m_queue, &ev, portMAX_DELAY) == pdTRUE) {
            processEvent(&ev);
        }
    }
}

void Manager::processEvent(struct Manager::Event* ev) {
    switch (ev->type) {
    case EVENT_MOTORS: {
        auto data = (std::vector<EventMotorsData>*)ev->data.motors;
        bool changed = false;
        for (const auto& m : *data) {
            if ((m_motors[static_cast<int>(m.id)].get()->*m.setter_func)(m.value)) {
                changed = true;
            }
        }
        if (changed) {
            m_motors_pwm.update();
        }
        delete data;

        m_motors_last_set = xTaskGetTickCount();
        break;
    }
    case EVENT_MOTORS_STOP_ALL: {
        bool changed = false;
        for (MotorId id = MotorId::M1; id < MotorId::MAX; ++id) {
            if (m_motors[static_cast<int>(id)]->direct_power(0))
                changed = true;
        }
        if (changed)
            m_motors_pwm.update();
        break;
    }
    case EVENT_ENCODER_EDGE: {
        const auto& e = ev->data.encoderEdge;
        m_motors[static_cast<int>(e.id)]->enc()->onEdgeIsr(e.timestamp, e.pinLevel);
        break;
    }
    case EVENT_ENCODER_PCNT: {
        const auto& e = ev->data.encoderPcnt;
        m_motors[static_cast<int>(e.id)]->enc()->onPcntIsr(e.status);
        break;
    }
    }
}

bool Manager::motorsFailSafe() {
    if (m_motors_last_set != 0) {
        const auto now = xTaskGetTickCount();
        if (now - m_motors_last_set > pdMS_TO_TICKS(MOTORS_FAILSAFE_PERIOD_MS)) {
            ESP_LOGE(TAG, "Motor failsafe triggered, stopping all motors!");
            const Event ev = { .type = EVENT_MOTORS_STOP_ALL, .data = {} };
            queue(&ev);
            m_motors_last_set = 0;
        }
    }
    return true;
}

rb::SmartServoBus& Manager::initSmartServoBus(uint8_t servo_count, gpio_num_t pin, uart_port_t uart) {
    m_servos.install(servo_count, uart, pin);
    return m_servos;
}

MotorChangeBuilder Manager::setMotors() {
    return MotorChangeBuilder(*this);
}

void Manager::monitorTask(TaskHandle_t task) {
#ifdef RB_DEBUG_MONITOR_TASKS
    m_tasks_mutex.lock();
    m_tasks.push_back(task);
    m_tasks_mutex.unlock();
#endif
}

#ifdef RB_DEBUG_MONITOR_TASKS
bool Manager::printTasksDebugInfo() {
    std::lock_guard<std::mutex> lock(m_tasks_mutex);

    printf("%16s %5s %5s\n", "Name", "prio", "stack");
    printf("==========================================\n");
    for (auto task : m_tasks) {
        auto stackMark = uxTaskGetStackHighWaterMark(task);
        auto prio = uxTaskPriorityGet(task);
        printf("%16s %5d %5d\n", pcTaskGetTaskName(task), (int)prio, (int)stackMark);
    }
    return true;
}
#endif

MotorChangeBuilder::MotorChangeBuilder(Manager& manager)
    : m_manager(manager) {
    m_values.reset(new std::vector<Manager::EventMotorsData>());
}

MotorChangeBuilder::MotorChangeBuilder(MotorChangeBuilder&& o)
    : m_manager(o.m_manager)
    , m_values(std::move(o.m_values)) {
}

MotorChangeBuilder::~MotorChangeBuilder() {
}

MotorChangeBuilder& MotorChangeBuilder::power(MotorId id, int8_t value) {
    m_values->emplace_back(Manager::EventMotorsData {
        .setter_func = &Motor::direct_power,
        .id = id,
        .value = value });
    return *this;
}

MotorChangeBuilder& MotorChangeBuilder::pwmMaxPercent(MotorId id, int8_t percent) {
    m_values->emplace_back(Manager::EventMotorsData {
        .setter_func = &Motor::direct_pwmMaxPercent,
        .id = id,
        .value = (int8_t)percent });
    return *this;
}

MotorChangeBuilder& MotorChangeBuilder::stop(MotorId id) {
    m_values->emplace_back(Manager::EventMotorsData {
        .setter_func = &Motor::direct_stop,
        .id = id,
        .value = 0 });
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
