#pragma once

namespace rb {

class Motor;

class Regulator {
public:
    Regulator(Motor& motor);
    
    void install();

    void process();
private:
    Motor& m_motor;
};

} // namespace rb
