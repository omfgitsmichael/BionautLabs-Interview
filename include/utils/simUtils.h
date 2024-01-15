#ifndef SIM_UTILS_H_
#define SIM_UTILS_H_

#include <Eigen/Dense>

namespace sim {

class Sim {
  public:
    // Mass-spring dampener system parameters
    struct Parameters {
      double dt = 0.0;
      double m = 0.0;
      double k = 0.0;
      double b = 0.0;
    };

    Sim(Parameters params, const Eigen::Vector<double, 2>& x):
        params_(params),
        x_(x)
    {
      // Ensure the mass is greater than zero
      assert(params_.m > 0.0);
    }

    void updateStates(double control);

    double getPosition() const;

    double getVelocity() const;

  private:
    Parameters params_;
    Eigen::Vector<double, 2> x_ = Eigen::Vector<double, 2>::Zero();
};

} // namespace sim

#endif // SIM_UTILS_H_
