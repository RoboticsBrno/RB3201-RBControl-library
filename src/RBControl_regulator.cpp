#include "RBControl_regulator.hpp"

#include <freertos/FreeRTOS.h>

#include <esp_log.h>

#define TAG "RbRegulator"

namespace rb {

Regulator::Regulator()
    : m_reader(nullptr),
      m_writer(nullptr)
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
    std::lock_guard<std::mutex> guard(s_mutex);
    if (s_instances.empty()) {
        s_instances.push_back(this);
    } else {
        for (auto reg: s_instances)
            if (reg == this) {
                ESP_LOGW(TAG, "Regulator::install should be called only once per Regulator.");
                return;
            }
        s_instances.push_back(this);
    }
    m_reader = inr;
    m_writer = outw;
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

void Regulator::process_trampoline() {
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

} // namespace rb
