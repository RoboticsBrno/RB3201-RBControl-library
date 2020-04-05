#include "RBControl_timers.hpp"
#include "RBControl_manager.hpp"

#define MS_TO_TICKS(ms) ((portTICK_PERIOD_MS <= ms) ? (ms / portTICK_PERIOD_MS) : 1)

namespace rb {

Timers::Timers(Manager& man)
    : m_id_counter(1) {
    xTaskCreatePinnedToCore(&Timers::taskTrampoline, "rb_timers", 4096, this, 1, &m_task, 1);
    man.monitorTask(m_task);
}

Timers::~Timers() {
}

void Timers::taskTrampoline(void* timers) {
    ((Timers*)timers)->timersTask();
}

void Timers::timersTask() {
    while (1) {
        TickType_t now = xTaskGetTickCount();
        TickType_t next = portMAX_DELAY;

        m_mutex.lock();
        for (size_t i = 0; i < m_timers.size();) {
            auto& t = m_timers[i];
            if (now >= t.next) {
                if (!t.callback()) {
                    cancelByIdxLocked(i);
                    continue;
                }
                t.next = now + t.period;
            }

            if (t.next < next)
                next = t.next;
            ++i;
        }
        m_mutex.unlock();

        now = xTaskGetTickCount();
        const auto to_wait = next <= now ? 0 : next - now;
        xTaskNotifyWait(0, 0, NULL, to_wait);
    }
}

uint16_t Timers::schedule(uint32_t period_ms, std::function<bool()> callback) {
    const TickType_t period = MS_TO_TICKS(period_ms);

    m_mutex.lock();
    const auto id = getFreeIdLocked();
    const TickType_t next = xTaskGetTickCount() + period;
    m_timers.emplace_back(timer_t {
        .callback = callback,
        .next = next,
        .period = period,
        .id = id,
    });
    m_mutex.unlock();

    xTaskNotify(m_task, 0, eNoAction);
    return id;
}

bool Timers::reset(uint16_t id, uint32_t period_ms) {
    std::lock_guard<std::recursive_mutex> l(m_mutex);

    const auto size = m_timers.size();
    for (size_t i = 0; i < size; ++i) {
        auto& t = m_timers[i];
        if (t.id != id)
            continue;

        t.period = MS_TO_TICKS(period_ms);
        t.next = xTaskGetTickCount() + t.period;
        xTaskNotify(m_task, 0, eNoAction);
        return true;
    }
    return false;
}

bool Timers::cancel(uint16_t id) {
    std::lock_guard<std::recursive_mutex> l(m_mutex);

    const auto size = m_timers.size();
    for (size_t i = 0; i < size; ++i) {
        if (m_timers[i].id == id) {
            cancelByIdxLocked(i);
            return true;
        }
    }
    return false;
}

void Timers::cancelByIdxLocked(size_t idx) {
    const auto size = m_timers.size();
    if (idx + 1 < size) {
        m_timers[idx] = m_timers[size - 1];
    }
    m_timers.pop_back();
}

uint16_t Timers::getFreeIdLocked() {
    uint16_t id = m_id_counter;
    while (1) {
        if (id == INVALID_ID) {
            ++id;
            continue;
        }

        bool found = false;
        for (const auto& t : m_timers) {
            if (t.id == id) {
                found = true;
                ++id;
                break;
            }
        }

        if (!found) {
            m_id_counter = id + 1;
            return id;
        }
    }
}

};
