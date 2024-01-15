#ifndef SENSOR_UTILS_H_
#define SENSOR_UTILS_H_

#include <Eigen/Dense>

namespace sensor {

class SensorModel {
  public:
    struct Parameters {
      double dt = 0.0;
      double tau = 0.0; /// Sensor model time constant
    };

    SensorModel(const Parameters& params) : params_(params)
    {
    }

    Eigen::Vector<double, 2> execute(const Eigen::Vector<double, 2>& states);

  private:
    Eigen::Vector<double, 2> previous_ = Eigen::Vector<double, 2>::Zero();
    Parameters params_;
};

} // namespace sensor

#endif // SENSOR_UTILS_H_
