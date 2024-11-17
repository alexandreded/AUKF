#include "BeamSimulation.h"
#include <cmath>

BeamSimulation::BeamSimulation()
    : x_c(-1.0), y_c(0.0), k(1), noiseLevel(0.6), noiseDistribution(0.0, 0.6)
{
    generator.seed(std::random_device{}());
}

void BeamSimulation::setNoiseLevel(double noiseLevel) {
    this->noiseLevel = noiseLevel;
    noiseDistribution = std::normal_distribution<double>(0.0, noiseLevel);
}

double BeamSimulation::h(double x, double y, double P0, double X, double Y, double w) {
    return 2 * P0 / (M_PI * w * w) * std::exp(-2 * ((x - X)*(x - X) + (y - Y)*(y - Y)) / (w * w));
}

Eigen::VectorXd BeamSimulation::moveBeamAndIntegrate(double P0, double w, double y_c, int iteration) {
    double speed = 0.3;
    double bounds_min = -1.0;
    double bounds_max = 1.0;
    double adc_time_step = 0.001;

    // Обновление x_c
    x_c += k * speed * adc_time_step;

    // Проверка на границы и изменение направления
    if (x_c >= bounds_max) {
        x_c = bounds_max;
        k = -1;
    } else if (x_c <= bounds_min) {
        x_c = bounds_min;
        k = 1;
    }

    // Интегрирование для каждого квадранта с добавлением шума
    double noise_factor = 1.0;

    double I_A = std::abs(h(0.5, 0.5, P0, x_c, y_c, w)) + noise_factor * noiseDistribution(generator);
    double I_B = std::abs(h(-0.5, 0.5, P0, x_c, y_c, w)) + noise_factor * noiseDistribution(generator);
    double I_C = std::abs(h(-0.5, -0.5, P0, x_c, y_c, w)) + noise_factor * noiseDistribution(generator);
    double I_D = std::abs(h(0.5, -0.5, P0, x_c, y_c, w)) + noise_factor * noiseDistribution(generator);

    Eigen::VectorXd intensities(4);
    intensities << I_A, I_B, I_C, I_D;

    return intensities;
}

double BeamSimulation::getXc() const {
    return x_c;
}

double BeamSimulation::getYc() const {
    return y_c;
}
