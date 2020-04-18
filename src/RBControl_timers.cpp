#include "RBControl_timers.hpp"
#include "RBControl_manager.hpp"

namespace rb {

Timers::Timers(Manager& man)
    : m_id_counter(1) {
}

Timers::~Timers() {
}

void Timers::timerCallback(void* argsVoid) {
    callback_arg_t* args = (callback_arg_t*)argsVoid;
    auto* self = args->self;
    const auto id = args->id;

    std::lock_guard<std::recursive_mutex> l(self->m_mutex);
    for (auto& t : self->m_timers) {
        if (t.args.get() != args)
            continue;

        if (!t.callback()) {
            self->cancel(id);
        }
        break;
    }
}

uint16_t Timers::schedule(uint32_t period_ms, std::function<bool()> callback) {
    auto args = std::unique_ptr<callback_arg_t>(new callback_arg_t);
    args->self = this;

    const esp_timer_create_args_t timer_args = {
        .callback = timerCallback,
        .arg = args.get(),
        .dispatch_method = ESP_TIMER_TASK,
        .name = "rb_timer",
    };
    esp_timer_handle_t timer = nullptr;
    esp_timer_create(&timer_args, &timer);

    m_mutex.lock();
    const auto id = getFreeIdLocked();
    args->id = id;
    m_timers.emplace_back(timer_t {
        .callback = callback,
        .handle = timer,
        .args = std::move(args),
    });
    esp_timer_start_periodic(timer, uint64_t(period_ms) * 1000);
    m_mutex.unlock();

    return id;
}

bool Timers::reset(uint16_t id, uint32_t period_ms) {
    std::lock_guard<std::recursive_mutex> l(m_mutex);

    for (auto& t : m_timers) {
        if (t.args->id != id)
            continue;

        esp_timer_stop(t.handle);
        esp_timer_start_periodic(t.handle, uint64_t(period_ms) * 1000);
        return true;
    }
    return false;
}

bool Timers::cancel(uint16_t id) {
    std::lock_guard<std::recursive_mutex> l(m_mutex);

    const auto size = m_timers.size();
    for (size_t i = 0; i < size; ++i) {
        if (m_timers[i].args->id == id) {
            cancelByIdxLocked(i);
            return true;
        }
    }
    return false;
}

void Timers::cancelByIdxLocked(size_t idx) {
    auto& t = m_timers[idx];
    esp_timer_stop(t.handle);
    esp_timer_delete(t.handle);

    const auto size = m_timers.size();
    if (idx + 1 < size) {
        m_timers[idx] = std::move(m_timers[size - 1]);
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
            if (t.args->id == id) {
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
