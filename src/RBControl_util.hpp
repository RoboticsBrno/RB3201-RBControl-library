#pragma once

#include <chrono>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ratio>

#include "RBControl_logger.hpp"

namespace rb {

template <typename T, typename... Args>
T clamp(T value, T min, T max, const char* tag = "",
    const char* msg = NULL, Args... args) {
    if (value < min) {
        if (msg != NULL) {
            rb::logWarning(tag, msg, std::forward<Args>(args)...);
        }
        return min;
    } else if (value > max) {
        if (msg != NULL) {
            rb::logWarning(tag, msg, std::forward<Args>(args)...);
        }
        return max;
    }
    return value;
}

inline void delayMs(int ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

inline void delay(std::chrono::duration<uint32_t, std::milli> delay) {
    vTaskDelay(delay.count() / portTICK_PERIOD_MS);
}

} // namespace rb