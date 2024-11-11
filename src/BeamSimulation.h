#ifndef BEAM_SIMULATION_H
#define BEAM_SIMULATION_H

#include <Eigen/Dense>
#include <random>

class BeamSimulation {
public:
    BeamSimulation();

    Eigen::VectorXd moveBeamAndIntegrate(double P0, double w, double y_c, int iteration);

    double getXc() const;

    void setNoiseLevel(double noiseLevel);

private:
    double h(double x, double y, double P0, double X, double Y, double w);

    double x_c;
    int k;
    std::default_random_engine generator;
    std::normal_distribution<double> noiseDistribution;
    double noiseLevel;
};

#endif // BEAM_SIMULATION_H
