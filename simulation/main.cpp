#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <tuple>

#include "utils/simUtils.h"
#include "utils/pathUtils.h"
#include "utils/controlUtils.h"

using namespace std::this_thread;     /// sleep_for, sleep_until
using namespace std::chrono_literals; /// ns, us, ms, s, h, etc.

std::tuple<bool, double> checkSafety(const Eigen::Vector<double, 3>& vehiclePos,
                                     const Eigen::Vector<double, 3>& path,
                                     double safetyRadius,
                                     double vehicleLength)
{
    const double error = (vehiclePos - path).norm();
    const bool result = safetyRadius > error + vehicleLength / 2.0; /// Vehicle radius is half of the length

    return std::make_tuple(result, error);
}

// Sim Parameters
constexpr double tInit = 0.0;
constexpr double dt = 0.01;
constexpr double tFinal = 10.0;
constexpr int simSteps = static_cast<int>((tFinal - tInit) / dt);

int main() {
    double time = tInit;

    // Create path
    Eigen::Vector<double, 3> startPosition{0.0, 0.0, 0.0};
    Eigen::Vector<double, 3> endPosition{0.1, 0.1, 0.1};
    Eigen::Matrix<double, simSteps, 3> path = path::generateSimplePath<simSteps>(startPosition, endPosition);

    // Bot initial conditions
    Eigen::Vector<double, 3> pos{0.0, 0.0, 0.0};
    Eigen::Vector<double, 3> vel{0.0, 0.0, 0.0};
    Eigen::Vector<double, 3> accel{0.0, 0.0, 0.0};

    // For now start in the direction of the path -- roll, pitch, yaw
    const double roll = std::atan2(endPosition(1), endPosition(2));
    const double pitch = std::atan2(endPosition(2), endPosition(0));
    const double yaw = std::atan2(endPosition(1), endPosition(0));
    Eigen::Vector<double, 3> orientation{roll, pitch, yaw};

    // Bot params
    const double length = 0.005; /// Assuming bot is sphere with length for now

    // Safety Radius
    const double R = 0.01;

    // Create controllers
    control::Controller controllerX;
    controllerX.setPosGains(1.25, 0.1, 0.5);
    controllerX.setVelGains(0.0, 0.0, 0.0);
    controllerX.setAccGains(0.25, 0.0, 0.0);
    controllerX.setTimeStep(dt);

    control::Controller controllerY;
    controllerY.setPosGains(1.25, 0.1, 0.5);
    controllerY.setVelGains(0.0, 0.0, 0.0);
    controllerY.setAccGains(0.25, 0.0, 0.0);
    controllerY.setTimeStep(dt);

    control::Controller controllerZ;
    controllerZ.setPosGains(1.25, 0.1, 0.5);
    controllerZ.setVelGains(0.0, 0.0, 0.0);
    controllerZ.setAccGains(0.25, 0.0, 0.0);
    controllerZ.setTimeStep(dt);

    // Control acceleration vector
    Eigen::Vector<double, 3> controlAccel{0.0, 0.0, 0.0};

    sim::Sim sim(dt, pos, vel, accel);

    for (int i = 0; i < simSteps; i++) {
        // Sensor feedback
        Eigen::Vector<double, 3> posMeas = sim.getPosition();
        Eigen::Vector<double, 3> velMeas = sim.getVelocity();
        Eigen::Vector<double, 3> accMeas = sim.getAcceleration();

        // Create states
        Eigen::Vector<double, 3> xMeas{posMeas(0), velMeas(0), accMeas(0)};
        Eigen::Vector<double, 3> yMeas{posMeas(1), velMeas(1), accMeas(1)};
        Eigen::Vector<double, 3> zMeas{posMeas(2), velMeas(2), accMeas(2)};

        // Check safety criteria
        bool isSafe = false;
        double deviationDistance = 0.0;
        std::tie(isSafe, deviationDistance) = checkSafety(posMeas, path.row(i), R, length);

        if (isSafe == false) {
            std::cout << "Failed to meet safety criteria!" << std::endl;
            break;
        }

        // Create desired states
        Eigen::Vector<double, 3> xDesired{path.row(i)(0), 0.0, 0.0}; /// path position, no velocity, no acceleration
        Eigen::Vector<double, 3> yDesired{path.row(i)(1), 0.0, 0.0}; /// path position, no velocity, no acceleration
        Eigen::Vector<double, 3> zDesired{path.row(i)(2), 0.0, 0.0}; /// path position, no velocity, no acceleration

        // Calculate control
        controlAccel(0) = controllerX.run(xMeas, xDesired);
        controlAccel(1) = controllerY.run(yMeas, yDesired);
        controlAccel(2) = controllerZ.run(zMeas, zDesired);

        // Update states/Propagate sim
        sim.updateStates(controlAccel);

        time += dt;

        std::cout << "Sim Time: " << time << std::endl;
        std::cout << "Is Vehicle Safe: " << isSafe << std::endl;
        std::cout << "Current Maximum Deviation Distance: " << deviationDistance + length / 2.0 << std::endl;
        std::cout << "Path Position: " << path.row(i) << std::endl;
        std::cout << "Vehicle Position: " << sim.getPosition().transpose() << std::endl;
        std::cout << "Vehicle Velocity: " << sim.getVelocity().transpose() << std::endl;
        std::cout << "Vehicle Acceleration: " << sim.getAcceleration().transpose() << std::endl;
        std::cout << "Control Acceleration Input: " << controlAccel.transpose() << std::endl;

        sleep_for(10ms);
    }

    return 0;
}
