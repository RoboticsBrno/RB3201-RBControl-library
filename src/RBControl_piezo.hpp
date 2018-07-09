#pragma once

namespace rb {

class Piezo {
    friend class Manager;
public:

    void setTone(uint32_t freq);

private:
    Piezo();
    ~Piezo();
};

};