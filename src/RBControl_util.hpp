#pragma once

#include "RBControl_logger.hpp"

namespace rb {

template <typename T, typename... Args >
T clamp(T value, T min, T max, const char* tag = "", 
        const char* msg = NULL, Args...args) {
    if(value < min) {
        if(msg != NULL) {
            rb::logWarning(tag, msg, std::forward<Args>(args)...);
        }
        return min;
    } else if (value > max) {
        if(msg != NULL) {
            rb::logWarning(tag, msg, std::forward<Args>(args)...);
        }
        return max;
    }
    return value;
}

} // namespace rb