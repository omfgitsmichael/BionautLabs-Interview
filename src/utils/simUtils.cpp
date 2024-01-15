#include "utils/simUtils.h"

namespace sim {

void Sim::updateStates(double control)
{
    Eigen::Matrix<double, 2, 2> A = Eigen::Matrix<double, 2, 2>::Zero();
    A(0, 1) = 1.0;
    A(1, 0) = -params_.k / params_.m;
    A(1, 1) = -params_.b / params_.m;

    Eigen::Vector<double, 2> B = Eigen::Vector<double, 2>::Zero();
    B(1) = 1.0 / params_.m;

    Eigen::Vector<double, 2> xDot = A * x_ + B * control;
    x_ = x_ + xDot * params_.dt;
}

double Sim::getPosition() const
{
    return x_(0);
}

double Sim::getVelocity() const
{
    return x_(1);
}

} // namespace sim
