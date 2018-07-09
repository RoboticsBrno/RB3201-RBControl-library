#include "RBControl_leds.hpp"
#include "RBControl_pinout.hpp"

namespace rb {

Leds::Leds(Adafruit_MCP23017& expander) : m_expander(expander) {
    m_expander.pinMode(LED_RED, GPIO_MODE_OUTPUT);
    m_expander.pinMode(LED_YELLOW, GPIO_MODE_OUTPUT);
    m_expander.pinMode(LED_GREEN, GPIO_MODE_OUTPUT);
    m_expander.pinMode(LED_BLUE, GPIO_MODE_OUTPUT);
}

Leds::~Leds() {

}

void Leds::red(bool on) {
    m_expander.digitalWrite(LED_RED, on ? 1 : 0);
}

void Leds::yellow(bool on) {
    m_expander.digitalWrite(LED_YELLOW, on ? 1 : 0);
}

void Leds::green(bool on) {
    m_expander.digitalWrite(LED_GREEN, on ? 1 : 0);
}

void Leds::blue(bool on) {
    m_expander.digitalWrite(LED_BLUE, on ? 1 : 0);
}


};