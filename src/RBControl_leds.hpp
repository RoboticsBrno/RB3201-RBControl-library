#pragma once

#include "Adafruit_MCP23017.h"

namespace rb {

class Leds {
    friend class Manager;
public:
    void red(bool on = true);
    void yellow(bool on = true);
    void green(bool on = true);
    void blue(bool on = true);

private:
    Leds(Adafruit_MCP23017& expander);
    ~Leds();

    Adafruit_MCP23017& m_expander;
};
};