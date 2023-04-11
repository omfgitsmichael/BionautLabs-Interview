#ifndef SIM_UTILS_H_
#define SIM_UTILS_H_

#include <Eigen/Dense>

namespace sim {

class Sim {
  public:
    Sim(double dt,
        const Eigen::Vector<double, 3>& position,
        const Eigen::Vector<double, 3>& velocity,
        const Eigen::Vector<double, 3>& accel):
        dt_(dt),
        position_(position),
        velocity_(velocity),
        acceleration_(accel)
    {
    }

    void updateStates(const Eigen::Vector<double, 3>& control);

    Eigen::Vector<double, 3> getPosition() const;

    Eigen::Vector<double, 3> getVelocity() const;

    Eigen::Vector<double, 3> getAcceleration() const;

  private:
    double dt_ = 0.0;
    Eigen::Vector<double, 3> position_ = Eigen::Vector<double, 3>::Zero();
    Eigen::Vector<double, 3> velocity_ = Eigen::Vector<double, 3>::Zero();
    Eigen::Vector<double, 3> acceleration_ = Eigen::Vector<double, 3>::Zero();
};

} // namespace sim

#endif // SIM_UTILS_H_
