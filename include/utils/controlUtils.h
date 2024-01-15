#ifndef CONTROL_UTILS_H_
#define CONTROL_UTILS_H_

#include <Eigen/Dense>

namespace control {

class LQRControl {
  public:
    struct Parameters {
      double dt = 0.0;
      Eigen::Matrix<double, 1, 3> K = Eigen::Matrix<double, 1, 3>::Zero();
    };

    LQRControl(const Parameters& params) : params_(params)
    {
    }

    double execute(const Eigen::Vector<double, 2>& states, double reference);

  private:
    double errorIntegral_ = 0.0;
    Parameters params_;
};

} // namespace control

#endif // CONTROL_UTILS_H_
