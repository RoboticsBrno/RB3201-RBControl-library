#include "RBControl_regulator.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#include <cmath>

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
      m_e_zero_ths(0),
      m_s_zero_coef(0),
      m_t_last(0),
      m_sum(0),
      m_e_last(0),
      m_w(NAN),
      m_x_max(1)
{}

Regulator::~Regulator() {
    std::lock_guard<mutex_type> guard(s_mutex);
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
    std::lock_guard<mutex_type> sguard(s_mutex);
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
    std::lock_guard<mutex_type> guard(m_mutex);
    m_reader = inr;
    m_writer = outw;
}

void Regulator::process() {
    std::lock_guard<mutex_type> guard(m_mutex);
    if (std::isnan(m_w))
        return;
    const Num y = m_reader();
    const int64_t t = esp_timer_get_time();
    const double dt = (t - m_t_last) / 1e6;
    Num e = m_w - y;
    if (abs(e) <= m_e_zero_ths) {
        e = 0;
        if (m_sum != 0 && m_x_max != 0 && m_s_zero_coef != 0) {
            const Num sum_dec = (m_x_max * dt) / (m_s_zero_coef * m_s);
            if (abs(sum_dec) >= abs(m_sum))
                m_sum = 0;
            else
                m_sum -= sum_dec;
        }
    }
    const Num sum = m_sum + e;
    const Num dif = (e - m_e_last) * dt;
    Num x = m_p * e + m_s * sum + m_d * dif;
    Num x1 = x;
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
    m_writer(x);
}

void Regulator::set(Num w) {
    std::lock_guard<mutex_type> guard(m_mutex);
    if (std::isnan(m_w)) {
        m_e_last = 0;
        m_sum = 0;
        m_t_last = esp_timer_get_time();
    }
    m_w = w;
}

Regulator::Num Regulator::get() {
    std::lock_guard<mutex_type> guard(m_mutex);
    return m_w;
}

void Regulator::set_params(Num p, Num s, Num d) {
    std::lock_guard<mutex_type> guard(m_mutex);
    m_p = abs(p);
    m_s = abs(s);
    m_d = abs(d);
}

void Regulator::set_max_output(Num max) {
    std::lock_guard<mutex_type> guard(m_mutex);
    m_x_max = abs(max);
}

void Regulator::set_zero_threshold(Num ths) {
    std::lock_guard<mutex_type> guard(m_mutex);
    m_e_zero_ths = abs(ths);
}

void Regulator::set_sum_zero_coef(Num coef) {
    std::lock_guard<mutex_type> guard(m_mutex);
    m_s_zero_coef = abs(coef);
}

void Regulator::disable() { set(NAN); }
bool Regulator::is_enabled() { return !std::isnan(get()); }

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

Regulator::mutex_type Regulator::s_mutex;
std::vector<Regulator*> Regulator::s_instances;
std::vector<std::function<void()> > Regulator::s_preprocessors;
std::vector<std::function<void()> > Regulator::s_postprocessors;

void Regulator::add_preprocessor(std::function<void()> fcn) {
    std::lock_guard<mutex_type> guard(s_mutex);
    s_preprocessors.push_back(fcn);
}

void Regulator::add_postprocessor(std::function<void()> fcn) {
    std::lock_guard<mutex_type> guard(s_mutex);
    s_postprocessors.push_back(fcn);
}

void Regulator::process_trampoline(void*) {
    std::lock_guard<mutex_type> guard(s_mutex);
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

Regulator::Num Regulator::abs(Num n) { return std::fabs(n); }

} // namespace rb
