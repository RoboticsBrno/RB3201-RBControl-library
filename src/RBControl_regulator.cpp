#include "RBControl_regulator.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#define TAG "RbRegulator"

#ifndef RBC_REGULATOR_PERIOD_MS
#define RBC_REGULATOR_PERIOD_MS 10
#endif

namespace rb {

Regulator::Regulator(std::string&& name)
    : m_mutex(),
      m_name(name),
      m_reader(nullptr),
      m_writer(nullptr),
      m_p(0),
      m_s(0),
      m_d(0),
      m_t_last(0),
      m_sum(0),
      m_e_last(0),
      m_w(0),
      m_x_max(1)
{}

Regulator::~Regulator() {
    std::lock_guard<std::mutex> guard(s_mutex);
    for (auto i = s_instances.begin(); i != s_instances.end(); ++i) {
        if (*i == this) {
            s_instances.erase(i);
            return;
        }
    }
}

void Regulator::install(InputReader inr, OutputWriter outw) {
    if (!inr || !outw) {
        ESP_LOGE(TAG, "Both InputReader and OutputWriter has to be valid functions!");
        abort();
    }
    std::lock_guard<std::mutex> sguard(s_mutex);
    if (s_instances.empty()) {
        xTaskCreate(&Regulator::process_loop, "rbreg_loop", 4096, this, 2, NULL);
    } else {
        for (auto reg: s_instances) {
            if (reg == this) {
                ESP_LOGW(TAG, "Regulator::install should be called only once per Regulator.");
                return;
            }
        }
    }
    s_instances.push_back(this);
    std::lock_guard<std::mutex> mguard(m_mutex);
    m_reader = inr;
    m_writer = outw;
}

void Regulator::process() {
    const Num y = m_reader();
    const int64_t t = esp_timer_get_time();
    const double dt = (t - m_t_last) / 1e6;
    m_mutex.lock();
    const Num e = m_w - y;
    const Num sum = m_sum + e;
    const Num dif = (e - m_e_last) * dt;
    Num x = m_p * e + m_s * sum + m_d * dif;
    if (clamp_output(x)) {
        if (x < 0) {
            if (sum > m_sum)
                m_sum = sum;
        } else {
            if (sum < m_sum)
                m_sum = sum;
        }
    } else {
        m_sum = sum;
    }
    m_e_last = e;
    m_t_last = t;
    m_mutex.unlock();
    m_writer(x);
}

void Regulator::set(Num w) {
    std::lock_guard<std::mutex> guard(m_mutex);
    m_w = w;
}

void Regulator::set_params(Num p, Num s, Num d) {
    std::lock_guard<std::mutex> guard(m_mutex);
    m_p = p;
    m_s = s;
    m_d = d;
}

void Regulator::set_max_output(Num max) {
    std::lock_guard<std::mutex> guard(m_mutex);
    m_x_max = max;
}

bool Regulator::clamp_output(Num& x) {
    if (x < -m_x_max) {
        x = -m_x_max;
        return true;
    }
    if (x > m_x_max) {
        x = m_x_max;
        return true;
    }
    return false;
}

std::mutex Regulator::s_mutex;
std::vector<Regulator*> Regulator::s_instances;
std::vector<std::function<void()> > Regulator::s_preprocessors;
std::vector<std::function<void()> > Regulator::s_postprocessors;

void Regulator::add_preprocessor(std::function<void()> fcn) {
    std::lock_guard<std::mutex> guard(s_mutex);
    s_preprocessors.push_back(fcn);
}

void Regulator::add_postprocessor(std::function<void()> fcn) {
    std::lock_guard<std::mutex> guard(s_mutex);
    s_postprocessors.push_back(fcn);
}

void Regulator::process_trampoline(void*) {
    std::lock_guard<std::mutex> guard(s_mutex);
    for (auto& fcn: s_preprocessors)
        if (fcn)
            fcn();
    for (auto reg: s_instances)
        reg->process();
    for (auto& fcn: s_postprocessors)
        if (fcn)
            fcn();
}

void Regulator::process_loop(void*) {
    s_mutex.lock();
    s_mutex.unlock();
    while (true) {
        process_trampoline(nullptr);
        s_mutex.lock();
        if (s_instances.empty())
            break;
        s_mutex.unlock();
        vTaskDelay(RBC_REGULATOR_PERIOD_MS / portTICK_PERIOD_MS);
    }
    s_mutex.unlock();
    vTaskDelete(nullptr);
}

} // namespace rb
