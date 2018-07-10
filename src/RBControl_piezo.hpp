#pragma once

namespace rb {

/**
 * \brief Helper class for controlling the piezo.
 */
class Piezo {
    friend class Manager;
public:

    void setTone(uint32_t freq); //!< Set piezo to freq. Use 0 to turn off.

private:
    Piezo();
    ~Piezo();
};

};