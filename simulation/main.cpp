#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <random>

#include "utils/controlUtils.h"
#include "utils/sensorUtils.h"
#include "utils/simUtils.h"

using namespace std::this_thread;     /// sleep_for, sleep_until
using namespace std::chrono_literals; /// ns, us, ms, s, h, etc.

// Sim Parameters
constexpr double tInit = 0.0;
constexpr double dt = 0.01;
constexpr double tFinal = 50.0;
constexpr int simSteps = static_cast<int>((tFinal - tInit) / dt);

int main() {
    double time = tInit;

    // Initial conditions
    Eigen::Vector<double, 2> xTrue{0.0, 0.0};
    Eigen::Vector<double, 2> xMeas = xTrue;

    // Control signal
    double control = 4.0;

    // Reference signal
    double reference = 1.0;

    // Sim model and class
    sim::Sim::Parameters simParams;
    simParams.dt = dt;
    simParams.m = 1.0;
    simParams.b = 2.0;
    simParams.k = 3.0;
    sim::Sim sim(simParams, xTrue);

    // Controller model and class
    control::LQRControl::Parameters controlParams;
    controlParams.dt = 0.1; /// Control Rate, not sim rate
    controlParams.K = {31.6227766016838, 17.1652852703227, 4.27140897571234};
    control::LQRControl controller(controlParams);

    // Sensor Model and class
    sensor::SensorModel::Parameters sensorParams;
    sensorParams.dt = 0.1;  /// Sensor Rate, not sim rate
    sensorParams.tau = 0.105; /// Time constant based off sensor rate to produce a 0.01 second delay
    sensor::SensorModel sensor(sensorParams);
    const double mean = 0.0;
    const double posStd = 0.01;
    const double velStd = 0.01;
    std::default_random_engine posGenerator;
    std::default_random_engine velGenerator;
    std::normal_distribution<double> posNoiseGenerator(mean, posStd);
    std::normal_distribution<double> velNoiseGenerator(mean, velStd);

    std::cout << "Sim_Time, "
              << "Ref_Position, "
              << "True_Position, "
              << "Meas_Position, "
              << "True_Velocity, "
              << "Meas_Velocity, "
              << "Control_Force, " << std::endl;

    for (int i = 0; i < simSteps; i++) {
        // Run the sensor model every 0.1 seconds (100ms --> 10 steps)
        if (i % 10 == 0) {
            const double posTrue = sim.getPosition();
            const double velTrue = sim.getVelocity();
            xTrue = {posTrue, velTrue};
            
            const double posNoise = posNoiseGenerator(posGenerator);
            const double velNoise = velNoiseGenerator(velGenerator);
            const Eigen::Vector<double, 2> noise{posNoise, velNoise};
            
            xMeas = sensor.execute(xTrue) + noise;
        }

        // Create the reference signal (comment out for constant reference)
        reference = 0.1 * std::sin(0.25 * time);

        // Run the controller every 0.1 seconds (100ms --> 10 steps)
        if (i % 10 == 0) {
            control = controller.execute(xMeas, reference);
        }

        // Update states/Propagate sim
        sim.updateStates(control);

        time += dt;

        std::cout <<
            time << ", " <<
            reference << ", " <<
            sim.getPosition() << ", " <<
            xMeas(0) << ", " <<
            sim.getVelocity() << ", " <<
            xMeas(1) << ", " << 
            control << std::endl;

        sleep_for(10ms);
    }

    return 0;
}
