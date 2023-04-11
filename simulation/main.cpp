#include <iostream>
#include <chrono>
#include <thread>

#include "utils/simUtils.h"

using namespace std::this_thread;     /// sleep_for, sleep_until
using namespace std::chrono_literals; /// ns, us, ms, s, h, etc.

int main() {

    // Sim Parameters
    double tInit = 0.0;
    double dt = 0.01;
    double tFinal = 10.0;

    unsigned int simSteps = static_cast<unsigned int>((tFinal - tInit) / dt);

    std::cout << simSteps << std::endl;

    double time = tInit;

    Eigen::Vector<double, 3> pos{1.0, 1.0, 1.0};
    Eigen::Vector<double, 3> vel{2.0, 2.0, 2.0};
    Eigen::Vector<double, 3> accel{3.0, 3.0, 3.0};

    Eigen::Vector<double, 3> controlAccel{0.0, 0.0, 0.0};

    sim::Sim sim(dt, pos, vel, accel);

    for (unsigned int i = 0; i < simSteps; i++) {
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
