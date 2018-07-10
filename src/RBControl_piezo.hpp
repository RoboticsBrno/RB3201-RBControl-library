#pragma once

namespace rb {

/**
 * \brief Helper class for controlling the piezo.
 */
class Piezo {
    friend class Manager;
public:

    void setTone(uint32_t freq);

private:
    Piezo();
    ~Piezo();
};

};