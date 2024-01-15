#include "utils/controlUtils.h"

namespace control {

double LQRControl::execute(const Eigen::Vector<double, 2>& states, double reference)
{
    // Calculate the position error integral
    const double error = states(0) - reference;
    errorIntegral_ += error * params_.dt;

    const Eigen::Vector<double, 3> x{errorIntegral_, states(0), states(1)};

    return -params_.K * x;
}

} // namespace control
