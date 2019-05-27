#include "RBControl_regulator.hpp"

namespace rb {

Regulator::Regulator(Motor& motor)
    : m_motor(motor)
{}

void Regulator::install() {}

} // namespace rb
