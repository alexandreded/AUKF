// BeamSimulation.cpp

#include "BeamSimulation.h"
#include <cmath>
#include <random>

BeamSimulation::BeamSimulation(double P0_, double w_, double x0_, double y0_)
    : P0(P0_), w(w_), x_c(x0_), y_c(y0_), k(1), speed(0.01), adc_time_step(14e-6),
      bounds_min(-1.0), bounds_max(1.0), noise_level(0.1),
      generator(std::random_device{}()), distribution(-0.5, 0.5)
{
    // Инициализация генератора случайных чисел выполнена в списке инициализации
}

double BeamSimulation::getXc() const {
    return x_c;
}

double BeamSimulation::getYc() const {
    return y_c;
}

void BeamSimulation::setNoiseLevel(double level) {
    noise_level = level;
}

void BeamSimulation::setSpeed(double new_speed) {
    speed = new_speed;
}

Eigen::VectorXd BeamSimulation::generateMeasurement(int iteration) {
    // Обновление положения x_c
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
    double I_A = h_function(1, 1, x_c, y_c) + randomNoise(noise_level);
    double I_B = h_function(-1, 1, x_c, y_c) + randomNoise(noise_level);
    double I_C = h_function(-1, -1, x_c, y_c) + randomNoise(noise_level);
    double I_D = h_function(1, -1, x_c, y_c) + randomNoise(noise_level);

    Eigen::VectorXd measurement(4);
    measurement << I_A, I_B, I_C, I_D;
    return measurement;
}

double BeamSimulation::h_function(double x, double y, double X, double Y) {
    return 2 * P0 / (M_PI * w * w) * std::exp(-2 * ((x - X) * (x - X) + (y - Y) * (y - Y)) / (w * w));
}

double BeamSimulation::randomNoise(double level) {
    return level * distribution(generator);
}
