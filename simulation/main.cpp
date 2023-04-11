#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

#include "utils/simUtils.h"
#include "utils/pathUtils.h"

using namespace std::this_thread;     /// sleep_for, sleep_until
using namespace std::chrono_literals; /// ns, us, ms, s, h, etc.

// Sim Parameters
constexpr double tInit = 0.0;
constexpr double dt = 0.01;
constexpr double tFinal = 1.0;
constexpr int simSteps = static_cast<int>((tFinal - tInit) / dt);

int main() {
    double time = tInit;

    // Create path
    Eigen::Vector<double, 3> startPosition{0.0, 0.0, 0.0};
    Eigen::Vector<double, 3> endPosition{1.0, 1.0, 1.0};
    Eigen::Matrix<double, simSteps, 3> path = path::generateSimplePath<simSteps>(startPosition, endPosition);

    // Bot initial conditions
    Eigen::Vector<double, 3> pos{0.0, 0.0, 0.0};
    Eigen::Vector<double, 3> vel{0.0, 0.0, 0.0};
    Eigen::Vector<double, 3> accel{0.0, 0.0, 0.0};

    /// For now start in the direction of the path -- roll, pitch, yaw
    const double roll = std::atan2(endPosition(1), endPosition(2));
    const double pitch = std::atan2(endPosition(2), endPosition(0));
    const double yaw = std::atan2(endPosition(1), endPosition(0));
    Eigen::Vector<double, 3> orientation{roll, pitch, yaw};

    std::cout << orientation << std::endl;

    // Bot params
    double length = 0.005;

    // Control acceleration vector
    Eigen::Vector<double, 3> controlAccel{0.0, 0.0, 0.0};

    sim::Sim sim(dt, pos, vel, accel);

    for (int i = 0; i < simSteps; i++) {
        sim.updateStates(controlAccel);

        time += dt;

        std::cout << "Sim Time: " << time << std::endl;
        std::cout << "Sim Position: " << sim.getPosition().transpose() << std::endl;
        std::cout << "Sim Velocity: " << sim.getVelocity().transpose() << std::endl;
        std::cout << "Sim Acceleration: " << sim.getAcceleration().transpose() << std::endl;

        sleep_for(10ms);
    }

    return 0;
}
