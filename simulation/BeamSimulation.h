#ifndef BEAM_SIMULATION_H
#define BEAM_SIMULATION_H

#include <Eigen/Dense>
#include <random>

class BeamSimulation {
public:
    BeamSimulation(double noiseLevel, double gapSize, double timeStep, double speed);

    void reset();
    Eigen::VectorXd moveBeamAndIntegrate(double P0, double w);

    double getXc() const;
    double getYc() const;

private:
    double x_c;
    double y_c;
    int k_x;
    int k_y;

    double noiseLevel;
    double gapSize;
    double timeStep;
    double speed;

    std::default_random_engine generator;
    std::normal_distribution<double> noiseDistribution;

    double gaussianBeam(double x, double y, double P0, double x_c, double y_c, double w);
    double integrateIntensity(double x_min, double x_max, double y_min, double y_max,
                              double P0, double x_c, double y_c, double w);
};

#endif // BEAM_SIMULATION_H
