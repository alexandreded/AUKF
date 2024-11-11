#include "AdaptiveUnscentedKalmanFilter.h"
#include <cmath>
//#include <iostream>

AdaptiveUnscentedKalmanFilter::AdaptiveUnscentedKalmanFilter(const Eigen::VectorXd &initial_state,
                                                             const Eigen::MatrixXd &initial_covariance,
                                                             const Eigen::MatrixXd &process_noise_cov,
                                                             const Eigen::MatrixXd &measurement_noise_cov)
    : state(initial_state),
      covariance(initial_covariance),
      process_noise_cov(process_noise_cov),
      measurement_noise_cov(measurement_noise_cov),
      n(initial_state.size()),
      adapt_window(30)
{
    // Параметры по умолчанию
    alpha = 0.07;
    beta = 1.0;
    kappa = 3.0;

    lambda_ = alpha * alpha * (n + kappa) - n;
    sigma_point_count = 2 * n + 1;

    // Вычисление весов
    weights_mean.resize(sigma_point_count);
    weights_covariance.resize(sigma_point_count);

    weights_mean[0] = lambda_ / (n + lambda_);
    weights_covariance[0] = weights_mean[0] + (1 - alpha * alpha + beta);

    for (int i = 1; i < sigma_point_count; ++i) {
        weights_mean[i] = 1.0 / (2 * (n + lambda_));
        weights_covariance[i] = weights_mean[i];
    }
}

void AdaptiveUnscentedKalmanFilter::setParameters(double alpha, double beta, double kappa) {
    this->alpha = alpha;
    this->beta = beta;
    this->kappa = kappa;

    lambda_ = alpha * alpha * (n + kappa) - n;
    sigma_point_count = 2 * n + 1;

    // Обновление весов
    weights_mean[0] = lambda_ / (n + lambda_);
    weights_covariance[0] = weights_mean[0] + (1 - alpha * alpha + beta);

    for (int i = 1; i < sigma_point_count; ++i) {
        weights_mean[i] = 1.0 / (2 * (n + lambda_));
        weights_covariance[i] = weights_mean[i];
    }
}

void AdaptiveUnscentedKalmanFilter::reset(const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &initial_covariance) {
    state = initial_state;
    covariance = initial_covariance;

    // Очистка истории
    innovation_history.clear();
    residual_history.clear();
}

void AdaptiveUnscentedKalmanFilter::setProcessNoiseCovariance(double processNoise) {
    process_noise_cov = Eigen::MatrixXd::Identity(n, n) * processNoise;
}

void AdaptiveUnscentedKalmanFilter::setMeasurementNoiseCovariance(double measurementNoise) {
    measurement_noise_cov = Eigen::MatrixXd::Identity(n, n) * measurementNoise;
}

std::vector<Eigen::VectorXd> AdaptiveUnscentedKalmanFilter::computeSigmaPoints() {
    std::vector<Eigen::VectorXd> sigma_points(sigma_point_count, Eigen::VectorXd(n));
    double scaling_factor = sqrt(n + lambda_);

    Eigen::MatrixXd sqrt_covariance = covariance.llt().matrixL();

    sigma_points[0] = state;

    for (int i = 0; i < n; ++i) {
        sigma_points[i + 1] = state + scaling_factor * sqrt_covariance.col(i);
        sigma_points[n + i + 1] = state - scaling_factor * sqrt_covariance.col(i);
    }

    return sigma_points;
}

void AdaptiveUnscentedKalmanFilter::predict() {
    auto sigma_points = computeSigmaPoints();
    std::vector<Eigen::VectorXd> predicted_sigma_points(sigma_point_count, Eigen::VectorXd(n));

    // Функция перехода состояния (можно заменить на нелинейную функцию при необходимости)
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_sigma_points[i] = sigma_points[i]; // Линейная модель
    }

    // Предсказанное состояние
    Eigen::VectorXd predicted_state = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_state += weights_mean[i] * predicted_sigma_points[i];
    }

    // Предсказанная ковариация
    Eigen::MatrixXd predicted_covariance = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < sigma_point_count; ++i) {
        Eigen::VectorXd diff = predicted_sigma_points[i] - predicted_state;
        predicted_covariance += weights_covariance[i] * diff * diff.transpose();
    }

    predicted_covariance += process_noise_cov;

    // Обновляем состояние и ковариацию
    state = predicted_state;
    covariance = predicted_covariance;
}

void AdaptiveUnscentedKalmanFilter::update(const Eigen::VectorXd &measurement) {
    auto sigma_points = computeSigmaPoints();
    int measurement_dim = measurement.size();

    // Прогноз измерений
    std::vector<Eigen::VectorXd> predicted_measurements(sigma_point_count, Eigen::VectorXd(measurement_dim));
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_measurements[i] = sigma_points[i]; // Линейная модель измерения
    }

    // Предсказанное измерение
    Eigen::VectorXd predicted_measurement_mean = Eigen::VectorXd::Zero(measurement_dim);
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_measurement_mean += weights_mean[i] * predicted_measurements[i];
    }

    // Ковариация измерений
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(measurement_dim, measurement_dim);
    for (int i = 0; i < sigma_point_count; ++i) {
        Eigen::VectorXd diff = predicted_measurements[i] - predicted_measurement_mean;
        S += weights_covariance[i] * diff * diff.transpose();
    }
    S += measurement_noise_cov;

    // Кросс-ковариация
    Eigen::MatrixXd cross_covariance = Eigen::MatrixXd::Zero(n, measurement_dim);
    for (int i = 0; i < sigma_point_count; ++i) {
        Eigen::VectorXd state_diff = sigma_points[i] - state;
        Eigen::VectorXd measurement_diff = predicted_measurements[i] - predicted_measurement_mean;
        cross_covariance += weights_covariance[i] * state_diff * measurement_diff.transpose();
    }

    // Коэффициент усиления Калмана
    Eigen::MatrixXd K = cross_covariance * S.inverse();

    // Обновление состояния и ковариации
    Eigen::VectorXd innovation = measurement - predicted_measurement_mean;
    state += K * innovation;
    covariance -= K * S * K.transpose();

    // Сохранение инноваций и остатков для адаптивной оценки
    Eigen::VectorXd epsilon = measurement - predicted_measurement_mean;
    Eigen::VectorXd eta = measurement - state;
    innovation_history.push_back(epsilon);
    residual_history.push_back(eta);

    if (innovation_history.size() > adapt_window) {
        innovation_history.erase(innovation_history.begin());
        residual_history.erase(residual_history.begin());
    }

    // Адаптивная оценка ковариаций
    if (innovation_history.size() >= adapt_window) {
        // Преобразование историй в матрицы
        Eigen::MatrixXd innovation_seq(measurement_dim, adapt_window);
        Eigen::MatrixXd residual_seq(measurement_dim, adapt_window);
        for (int i = 0; i < adapt_window; ++i) {
            innovation_seq.col(i) = innovation_history[i];
            residual_seq.col(i) = residual_history[i];
        }

        // Оценка ковариаций
        Eigen::MatrixXd Q_adapt = (innovation_seq * innovation_seq.transpose()) / adapt_window;
        Eigen::MatrixXd R_adapt = (residual_seq * residual_seq.transpose()) / adapt_window;

        // Обновление ковариаций с экспоненциальным сглаживанием
        double gamma = 0.1;
        process_noise_cov = (1 - gamma) * process_noise_cov + gamma * Q_adapt;
        measurement_noise_cov = (1 - gamma) * measurement_noise_cov + gamma * R_adapt;
    }
}

Eigen::VectorXd AdaptiveUnscentedKalmanFilter::getState() const {
    return state;
}

Eigen::MatrixXd AdaptiveUnscentedKalmanFilter::getCovariance() const {
    return covariance;
}

// Функция h для распределения энергии
double AdaptiveUnscentedKalmanFilter::h(double x, double y, double P0, double X, double Y, double w) {
    return 2 * P0 / (M_PI * w * w) * std::exp(-2 * ((x - X)*(x - X) + (y - Y)*(y - Y)) / (w * w));
}

// Функция g для преобразования координаты
double AdaptiveUnscentedKalmanFilter::g(double Ex) {
    return erfinv(Ex) / std::sqrt(2.0);
}

// Функция компенсации ошибки lambda(x0)
double AdaptiveUnscentedKalmanFilter::lambdaFunc(double x0) {
    double a = 1.0;
    double b = 0.0;
    return a * x0 + b;
}

// Вычисление положения пятна
Eigen::Vector2d AdaptiveUnscentedKalmanFilter::calculateSpotPosition(double I_A, double I_B, double I_C, double I_D, double w, double x0) {
    double sum_I = I_A + I_B + I_C + I_D;
    double E_X = ((I_A + I_D) - (I_B + I_C)) / sum_I;
    double E_Y = ((I_A + I_B) - (I_C + I_D)) / sum_I;

    double x = g(E_X) * w * lambdaFunc(x0);
    double y = g(E_Y) * w * lambdaFunc(x0);

    return Eigen::Vector2d(x, y);
}

// Функция обратной ошибки (erfinv)
double AdaptiveUnscentedKalmanFilter::erfinv(double x) {
    // Используем приближение для erfinv(x)
    double a = 0.147; // Константа для приближения

    double ln = std::log(1 - x * x);
    double term = (2 / (M_PI * a)) + (ln / 2);
    double erfinv = std::copysign(std::sqrt(std::sqrt(term * term - (ln / a)) - term), x);

    return erfinv;
}
