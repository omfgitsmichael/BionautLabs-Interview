#ifndef CONTROL_UTILS_H_
#define CONTROL_UTILS_H_

#include <Eigen/Dense>

namespace control {

class PIDControl {
  public:
    PIDControl()
    {
    }

    void setTimeStep(double dt);

    void setGains(double kp, double ki, double kd);

    double calculateControl(double value, double desired);

  private:
    double dt_ = 0.0;
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;

    double e_ = 0.0;
    double ePrev_ = 0.0;
    double eRate_ = 0.0;
    double eIntegral_ = 0.0;
};

class Controller {
  public:
    Controller()
    {
    }

    void setTimeStep(double dt);

    void setPosGains(double kp, double ki, double kd);

    void setVelGains(double kp, double ki, double kd);

    void setAccGains(double kp, double ki, double kd);

    double run(const Eigen::Vector<double, 3>& state, const Eigen::Vector<double, 3>& desired);

  private:
    PIDControl posControl_;
    PIDControl velControl_;
    PIDControl accControl_;
};

} // namespace control

#endif // CONTROL_UTILS_H_
