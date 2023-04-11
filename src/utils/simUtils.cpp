#include "utils/simUtils.h"

#include <iostream>

namespace sim {

void Sim::updateStates(const Eigen::Vector<double, 3>& controlAccel)
{
    Eigen::Vector<double, 9> states = Eigen::Vector<double, 9>::Zero();
    states.head(3) = position_;
    states.segment(3, 3) = velocity_;
    states.tail(3) = acceleration_ + controlAccel;

    Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Zero();

    Eigen::Matrix<double, 3, 3> identity = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 3> integrator = identity * dt_;
    Eigen::Matrix<double, 3, 3> doubleIntegrator = 0.5 * identity * dt_ * dt_;

    A.block<3, 3>(0, 0) = identity;
    A.block<3, 3>(0, 3) = integrator;
    A.block<3, 3>(0, 6) = doubleIntegrator;
    A.block<3, 3>(3, 3) = identity;
    A.block<3, 3>(3, 6) = integrator;
    A.block<3, 3>(6, 6) = identity;

    states = A * states;

    position_ = states.head(3);
    velocity_ = states.segment(4, 3);
    acceleration_ = states.tail(3);
}

Eigen::Vector<double, 3> Sim::getPosition() const
{
    return position_;
}

Eigen::Vector<double, 3> Sim::getVelocity() const
{
    return velocity_;
}

Eigen::Vector<double, 3> Sim::getAcceleration() const
{
    return acceleration_;
}

} // namespace sim
