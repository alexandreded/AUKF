#include "BeamSimulation.h"
#include <cmath>
#include <algorithm> // Для std::max

BeamSimulation::BeamSimulation()
    : x_c(0.0), y_c(0.0), k_x(1), k_y(1), noiseLevel(0.0), gapSize(0.0),
      timeStep(0.001), speed(1), noiseDistribution(0.0, 0.0)
{
    generator.seed(std::random_device{}());
}

void BeamSimulation::setNoiseLevel(double noiseLevel) {
    this->noiseLevel = std::max(0.0, noiseLevel);
    noiseDistribution = std::normal_distribution<double>(0.0, this->noiseLevel);
}

void BeamSimulation::setGapSize(double gapSize) {
    this->gapSize = std::max(0.0, gapSize);
}

void BeamSimulation::setTimeStep(double timeStep) {
    this->timeStep = timeStep;
}

void BeamSimulation::setBeamSpeed(double speed) {
    this->speed = speed;
}

void BeamSimulation::reset() {
    x_c = 0.0;
    y_c = 0.0;
    k_x = 1;
    k_y = 1;
}

double BeamSimulation::getXc() const {
    return x_c;
}

double BeamSimulation::getYc() const {
    return y_c;
}

double BeamSimulation::gaussianBeam(double x, double y, double P0, double x_c, double y_c, double w) {
    double exponent = -2.0 * ((x - x_c)*(x - x_c) + (y - y_c)*(y - y_c)) / (w * w);
    return P0 * std::exp(exponent);
}

double BeamSimulation::integrateIntensity(double x_min, double x_max, double y_min, double y_max,
                                          double P0, double x_c, double y_c, double w) {
    // Аналитическое интегрирование гауссианы по прямоугольной области с использованием функции ошибок

    double sqrt2 = std::sqrt(2.0);
    double sigma = w / sqrt2; // Стандартное отклонение

    double arg_x_max = (x_max - x_c) / sigma;
    double arg_x_min = (x_min - x_c) / sigma;
    double arg_y_max = (y_max - y_c) / sigma;
    double arg_y_min = (y_min - y_c) / sigma;

    double erf_x = std::erf(arg_x_max) - std::erf(arg_x_min);
    double erf_y = std::erf(arg_y_max) - std::erf(arg_y_min);

    double integral = (P0 / 4.0) * erf_x * erf_y;

    return integral;
}

Eigen::VectorXd BeamSimulation::moveBeamAndIntegrate(double P0, double w) {
    double bounds_min = -0.5;
    double bounds_max = 0.5;

    // Обновление x_c
    x_c += k_x * speed * timeStep;

    // Проверка на границы и изменение направления для x_c
    if (x_c > bounds_max) {
        x_c = bounds_max;
        k_x = -1;
    } else if (x_c < bounds_min) {
        x_c = bounds_min;
        k_x = 1;
    }

    // Обновление y_c
    y_c -= k_x * speed * timeStep;

    // Проверка на границы и изменение направления для y_c
    if (y_c > bounds_max) {
        y_c = bounds_max;
        k_y = -1;
    } else if (y_c < bounds_min) {
        y_c = bounds_min;
        k_y = 1;
    }

    // Размеры детекторов и позиции с учетом щели
    double detector_half_size = 0.5; // Половина размера детектора (от -0.5 до 0.5)

    // Границы детекторов с учетом щели
    // Детектор A (верхний правый)
    double x_min_A = 0.0 + gapSize / 2.0;
    double x_max_A = detector_half_size;
    double y_min_A = 0.0 + gapSize / 2.0;
    double y_max_A = detector_half_size;

    // Детектор B (верхний левый)
    double x_min_B = -detector_half_size;
    double x_max_B = 0.0 - gapSize / 2.0;
    double y_min_B = 0.0 + gapSize / 2.0;
    double y_max_B = detector_half_size;

    // Детектор C (нижний левый)
    double x_min_C = -detector_half_size;
    double x_max_C = 0.0 - gapSize / 2.0;
    double y_min_C = -detector_half_size;
    double y_max_C = 0.0 - gapSize / 2.0;

    // Детектор D (нижний правый)
    double x_min_D = 0.0 + gapSize / 2.0;
    double x_max_D = detector_half_size;
    double y_min_D = -detector_half_size;
    double y_max_D = 0.0 - gapSize / 2.0;

    // Интегрирование для каждого детектора
    double I_A = integrateIntensity(x_min_A, x_max_A, y_min_A, y_max_A, P0, x_c, y_c, w);
    double I_B = integrateIntensity(x_min_B, x_max_B, y_min_B, y_max_B, P0, x_c, y_c, w);
    double I_C = integrateIntensity(x_min_C, x_max_C, y_min_C, y_max_C, P0, x_c, y_c, w);
    double I_D = integrateIntensity(x_min_D, x_max_D, y_min_D, y_max_D, P0, x_c, y_c, w);

    // Добавляем шум
    I_A += noiseDistribution(generator);
    I_B += noiseDistribution(generator);
    I_C += noiseDistribution(generator);
    I_D += noiseDistribution(generator);

    // Гарантируем, что интенсивности неотрицательны
    I_A = std::max(0.0, I_A);
    I_B = std::max(0.0, I_B);
    I_C = std::max(0.0, I_C);
    I_D = std::max(0.0, I_D);

    Eigen::VectorXd intensities(4);
    intensities << I_A, I_B, I_C, I_D;

    return intensities;
}
