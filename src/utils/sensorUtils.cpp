#include "utils/sensorUtils.h"

namespace sensor {

Eigen::Vector<double, 2> SensorModel::execute(const Eigen::Vector<double, 2>& input)
{
    const double alpha = params_.dt / params_.tau;
    const Eigen::Vector<double, 2> output = alpha * input + (1 - alpha) * previous_;
    previous_ = output;

    return output;
}

} // namespace sensor
