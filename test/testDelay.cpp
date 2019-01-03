#include <unity.h>
#include "RBControl_util.hpp"

// This test only checks if the project compiles

void testDelayMs() {
    rb::delayMs(500);
}

void testDelay() {
    using namespace std::chrono_literals;
    rb::delay(500ms);
    rb::delay(1s);
    rb::delay(1h);
}

extern "C" void app_main() {
    UNITY_BEGIN();
    RUN_TEST(testDelayMs);
    RUN_TEST(testDelay);
    UNITY_END();
}