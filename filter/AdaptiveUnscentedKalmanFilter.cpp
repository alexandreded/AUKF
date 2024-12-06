#include "AdaptiveUnscentedKalmanFilter.h"
#include <cmath>
#include <algorithm>
#include <iostream>

AdaptiveUnscentedKalmanFilter::AdaptiveUnscentedKalmanFilter(
        const Eigen::VectorXd &initial_state,
        const Eigen::MatrixXd &initial_covariance,
        const Eigen::MatrixXd &process_noise_cov,
        const Eigen::MatrixXd &measurement_noise_cov,
        double alpha, double beta, double kappa,
        int adapt_window)
    : state(initial_state),
      covariance(initial_covariance),
      process_noise_cov(process_noise_cov),
      measurement_noise_cov(measurement_noise_cov),
      alpha(alpha), beta(beta), kappa(kappa),
      adapt_window(adapt_window)
{
    n = state.size();
    m = measurement_noise_cov.rows();
    lambda_ = alpha * alpha * (n + kappa) - n;
    sigma_point_count = 2 * n + 1;

    weights_mean.resize(sigma_point_count);
    weights_covariance.resize(sigma_point_count);

    weights_mean[0] = lambda_ / (n + lambda_);
    weights_covariance[0] = weights_mean[0] + (1 - alpha*alpha + beta);
    for (int i = 1; i < sigma_point_count; ++i) {
        weights_mean[i] = 1.0 / (2 * (n + lambda_));
        weights_covariance[i] = weights_mean[i];
    }
}

void AdaptiveUnscentedKalmanFilter::predict() {
    std::vector<Eigen::VectorXd> sigma_points(sigma_point_count, Eigen::VectorXd(n));
    computeSigmaPoints(sigma_points);

    std::vector<Eigen::VectorXd> predicted_sigma_points(sigma_point_count, Eigen::VectorXd(n));
    for (int i = 0; i < sigma_point_count; ++i)
        predicted_sigma_points[i] = stateTransitionFunction(sigma_points[i]);

    Eigen::VectorXd predicted_state = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < sigma_point_count; ++i)
        predicted_state += weights_mean[i] * predicted_sigma_points[i];

    Eigen::MatrixXd P_XX = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < sigma_point_count; ++i) {
        Eigen::VectorXd diff = predicted_sigma_points[i] - predicted_state;
        P_XX += weights_covariance[i] * diff * diff.transpose();
    }

    covariance = P_XX + process_noise_cov;
    state = predicted_state;
}

void AdaptiveUnscentedKalmanFilter::update(const Eigen::VectorXd &measurement) {
    std::vector<Eigen::VectorXd> sigma_points(sigma_point_count, Eigen::VectorXd(n));
    computeSigmaPoints(sigma_points);

    std::vector<Eigen::VectorXd> predicted_measurements(sigma_point_count, Eigen::VectorXd(m));
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_measurements[i] = measurementFunction(sigma_points[i]);
    }

    Eigen::VectorXd predicted_measurement = Eigen::VectorXd::Zero(m);
    for (int i = 0; i < sigma_point_count; ++i)
        predicted_measurement += weights_mean[i] * predicted_measurements[i];

    Eigen::MatrixXd P_ZZ = Eigen::MatrixXd::Zero(m, m);
    for (int i = 0; i < sigma_point_count; ++i) {
        Eigen::VectorXd diff = predicted_measurements[i] - predicted_measurement;
        P_ZZ += weights_covariance[i] * diff * diff.transpose();
    }
    P_ZZ += measurement_noise_cov;

    Eigen::MatrixXd P_XZ = Eigen::MatrixXd::Zero(n, m);
    for (int i = 0; i < sigma_point_count; ++i) {
        Eigen::VectorXd state_diff = sigma_points[i] - state;
        Eigen::VectorXd meas_diff = predicted_measurements[i] - predicted_measurement;
        P_XZ += weights_covariance[i] * state_diff * meas_diff.transpose();
    }

    Eigen::MatrixXd K = P_XZ * P_ZZ.inverse();
    Eigen::VectorXd innovation = measurement - predicted_measurement;

    // Сохраняем инновации
    innovation_history.push_back(innovation);
    if ((int)innovation_history.size() > adapt_window)
        innovation_history.pop_front();

    // Обновление состояния
    state = state + K * innovation;
    covariance = covariance - K * P_ZZ * K.transpose();

    // Остаток
    Eigen::VectorXd residual = measurement - measurementFunction(state);
    residual_history.push_back(residual);
    if ((int)residual_history.size() > adapt_window)
        residual_history.pop_front();

    measurement_history.push_back(measurement);
    if ((int)measurement_history.size() > adapt_window)
        measurement_history.pop_front();

    adaptProcessNoiseCovariance();
    adaptMeasurementNoiseCovariance();
}

Eigen::VectorXd AdaptiveUnscentedKalmanFilter::getState() const {
    return state;
}

Eigen::MatrixXd AdaptiveUnscentedKalmanFilter::getCovariance() const {
    return covariance;
}

void AdaptiveUnscentedKalmanFilter::setProcessNoiseCovariance(const Eigen::MatrixXd &Q) {
    process_noise_cov = Q;
}

void AdaptiveUnscentedKalmanFilter::setMeasurementNoiseCovariance(const Eigen::MatrixXd &R) {
    measurement_noise_cov = R;
}

void AdaptiveUnscentedKalmanFilter::computeSigmaPoints(std::vector<Eigen::VectorXd> &sigma_points) {
    double scaling_factor = std::sqrt(n + lambda_);
    Eigen::MatrixXd sqrt_covariance = covariance.llt().matrixL();

    sigma_points[0] = state;
    for (int i = 0; i < n; ++i) {
        sigma_points[i + 1] = state + scaling_factor * sqrt_covariance.col(i);
        sigma_points[n + i + 1] = state - scaling_factor * sqrt_covariance.col(i);
    }
}

Eigen::VectorXd AdaptiveUnscentedKalmanFilter::stateTransitionFunction(const Eigen::VectorXd &s) {
    // Предполагаем стационарность: состояние - это интенсивности, которые не меняются в отсутствии входа
    // При необходимости можно добавить логику изменения состояния.
    return s;
}

Eigen::VectorXd AdaptiveUnscentedKalmanFilter::measurementFunction(const Eigen::VectorXd &s) {
    // Тождественная функция, измеряемые величины - это те же самые интенсивности
    return s;
}

Eigen::Vector2d AdaptiveUnscentedKalmanFilter::calculateSpotPosition(double w, double x0) const {
    double I_A = state(0);
    double I_B = state(1);
    double I_C = state(2);
    double I_D = state(3);

    double sum_I = I_A + I_B + I_C + I_D;
    if (sum_I == 0.0) {
        return Eigen::Vector2d(0.0, 0.0);
    }

    double E_X = ((I_A + I_D) - (I_B + I_C)) / sum_I;
    double E_Y = ((I_A + I_B) - (I_C + I_D)) / sum_I;

    double x = g(E_X) * w * x0;
    double y = g(E_Y) * w * x0;

    return Eigen::Vector2d(x, y);
}

double AdaptiveUnscentedKalmanFilter::g(double Ex) const {
    return erfinv(Ex) / std::sqrt(2.0);
}

double AdaptiveUnscentedKalmanFilter::erfinv(double x) const {
    const double epsilon = 1e-6;
    x = std::max(std::min(x, 1.0 - epsilon), -1.0 + epsilon);

    double a = 0.147;
    double ln = std::log(1 - x * x);
    double term = (2 / (M_PI * a)) + (ln / 2);
    double val = std::copysign(std::sqrt(std::sqrt(term * term - (ln / a)) - term), x);
    return val;
}

void AdaptiveUnscentedKalmanFilter::adaptProcessNoiseCovariance() {
    // Примерная логика адаптации
    if ((int)innovation_history.size() < adapt_window ||
        (int)residual_history.size() < adapt_window) {
        return;
    }

    Eigen::MatrixXd E_diff = Eigen::MatrixXd::Zero(m, m);
    for (int i = 0; i < adapt_window; ++i) {
        Eigen::VectorXd diff = residual_history[i] - innovation_history[i];
        E_diff += diff * diff.transpose();
    }
    E_diff /= adapt_window;

    Eigen::MatrixXd H_k = Eigen::MatrixXd::Identity(m, n);

    Eigen::MatrixXd term = E_diff;
    // Простейшая модель адаптации Q
    // Возможно потребуется более точный метод.
    process_noise_cov = term;
}

void AdaptiveUnscentedKalmanFilter::adaptMeasurementNoiseCovariance() {
    if ((int)innovation_history.size() < adapt_window) {
        return;
    }

    Eigen::MatrixXd E_innovations = Eigen::MatrixXd::Zero(m, m);
    int count = (int)innovation_history.size();
    for (const auto& innovation : innovation_history) {
        E_innovations += innovation * innovation.transpose();
    }
    E_innovations /= count;

    double b = 0.95;
    int k = (int)measurement_history.size();
    double d_k = (1.0 - b) / (1.0 - std::pow(b, k + 1));

    measurement_noise_cov = (1.0 - d_k) * measurement_noise_cov + d_k * E_innovations;
}