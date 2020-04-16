#include "RBControl_timers.hpp"
#include "RBControl_manager.hpp"

#define MS_TO_TICKS(ms) ((portTICK_PERIOD_MS <= ms) ? (ms / portTICK_PERIOD_MS) : 1)

namespace rb {

Timers::Timers(Manager& man)
    : m_id_counter(1) {
}

Timers::~Timers() {
}

void Timers::timerCallback(TimerHandle_t timer) {
    auto* self = (Timers*)pvTimerGetTimerID(timer);

    std::lock_guard<std::recursive_mutex> l(self->m_mutex);
    for (auto& t : self->m_timers) {
        if (t.handle != timer)
            continue;

        if (t.callback()) {
            xTimerReset(t.handle, portMAX_DELAY);
        } else {
            self->cancel(t.id);
        }
        break;
    }
}

uint16_t Timers::schedule(uint32_t period_ms, std::function<bool()> callback) {
    const TickType_t period = MS_TO_TICKS(period_ms);
    auto handle = xTimerCreate("rbtimer", period, pdFALSE, this, timerCallback);

    m_mutex.lock();
    const auto id = getFreeIdLocked();
    m_timers.emplace_back(timer_t {
        .callback = callback,
        .handle = handle,
        .id = id,
    });
    xTimerStart(handle, portMAX_DELAY);
    m_mutex.unlock();

    return id;
}

bool Timers::reset(uint16_t id, uint32_t period_ms) {
    std::lock_guard<std::recursive_mutex> l(m_mutex);

    for (auto& t : m_timers) {
        if (t.id != id)
            continue;

        xTimerChangePeriod(t.handle, MS_TO_TICKS(period_ms), portMAX_DELAY);
        xTimerReset(t.handle, portMAX_DELAY);
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
    xTimerDelete(m_timers[idx].handle, portMAX_DELAY);
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
