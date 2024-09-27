// BeamSimulation.cpp

#include "BeamSimulation.h"
#include <cmath>
#include <cstdlib>
#include <ctime>

BeamSimulation::BeamSimulation(double P0_, double w_, double x0_, double y0_)
    : P0(P0_), w(w_), x_c(x0_), y_c(y0_), k(1), speed(1000.0), adc_time_step(14e-6),
      bounds_min(-1.0), bounds_max(1.0)
{
    // Инициализация генератора случайных чисел
    std::srand(static_cast<unsigned>(std::time(nullptr)));
}

double BeamSimulation::getXc() const {
    return x_c;
}

double BeamSimulation::getYc() const {
    return y_c;
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

    // Уровень шума
    double noise_level = 0.6;

    // Интегрирование для каждого квадранта с добавлением шума
    double I_A = std::abs(h_function(1, 1, x_c, y_c)) + randomNoise(noise_level);
    double I_B = std::abs(h_function(-1, 1, x_c, y_c)) + randomNoise(noise_level);
    double I_C = std::abs(h_function(-1, -1, x_c, y_c)) + randomNoise(noise_level);
    double I_D = std::abs(h_function(1, -1, x_c, y_c)) + randomNoise(noise_level);

    Eigen::VectorXd measurement(4);
    measurement << I_A, I_B, I_C, I_D;
    return measurement;
}

double BeamSimulation::h_function(double x, double y, double X, double Y) {
    return 2 * P0 / (M_PI * w * w) * std::exp(-2 * ((x - X) * (x - X) + (y - Y) * (y - Y)) / (w * w));
}

double BeamSimulation::erfinv(double x) {
    // Приближенная реализация функции обратной ошибки
    double a = 0.147;
    double ln1_minus_x_squared = std::log(1 - x * x);
    double numerator = 2 / (M_PI * a) + ln1_minus_x_squared / 2;
    double denominator = 1 / a * ln1_minus_x_squared;
    return std::copysign(std::sqrt(std::sqrt(numerator * numerator - denominator) - numerator), x);
}

double BeamSimulation::g_function(double Ex) {
    return erfinv(Ex) / std::sqrt(2.0);
}

double BeamSimulation::lambda_func(double x0) {
    double a = 1.0;
    double b = 0.0;
    return a * x0 + b;
}

double BeamSimulation::randomNoise(double level) {
    return level * ((double)rand() / RAND_MAX - 0.5);
}
