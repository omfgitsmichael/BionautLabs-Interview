#include "utils/controlUtils.h"

namespace control {

void PIDControl::setTimeStep(double dt)
{
    dt_ = dt;
}

void PIDControl::setGains(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

double PIDControl::calculateControl(double value, double desired)
{
    e_ = desired - value;
    eRate_ = (e_ - ePrev_) / dt_;
    ePrev_ = e_;
    eIntegral_ += e_ * dt_;

    return kp_ * e_ + ki_ * eIntegral_ + kd_ * eRate_;
}

void Controller::setTimeStep(double dt)
{
    posControl_.setTimeStep(dt);
    velControl_.setTimeStep(dt);
    accControl_.setTimeStep(dt);
}

void Controller::setPosGains(double kp, double ki, double kd)
{
    posControl_.setGains(kp, ki, kd);
}

void Controller::setVelGains(double kp, double ki, double kd)
{
    velControl_.setGains(kp, ki, kd);
}

void Controller::setAccGains(double kp, double ki, double kd)
{
    accControl_.setGains(kp, ki, kd);
}

double Controller::run(const Eigen::Vector<double, 3>& state, const Eigen::Vector<double, 3>& desired)
{
    // Determine position control -- outer loop
    const double posControl = posControl_.calculateControl(state(0), desired(0));

    // Determine velocity control -- mid loop
    const double velFeedForward = desired(1) + posControl;
    const double velFeedback = velControl_.calculateControl(state(1), desired(1));
    const double velControl = velFeedForward + velFeedback;

    // Determine acceleration control -- inner loop
    const double accFeedForward = desired(2) + velControl;
    const double accFeedback = accControl_.calculateControl(state(2), desired(2));
    const double accControl = accFeedForward + accFeedback;

    return accControl;
}

} // namespace control
