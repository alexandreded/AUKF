#include "AdaptiveUnscentedKalmanFilter.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <limits>

AdaptiveUnscentedKalmanFilter::AdaptiveUnscentedKalmanFilter(
        const Eigen::VectorXd &initial_state,
        const Eigen::MatrixXd &initial_covariance,
        const Eigen::MatrixXd &process_noise_cov,
        const Eigen::MatrixXd &measurement_noise_cov,
        double alpha, double beta, double kappa,
        int adapt_window)
    : n(0),
      m(0),
      sigma_point_count(0),
      alpha(alpha), beta(beta), kappa(kappa), lambda_(0.0),
      state(initial_state),
      covariance(initial_covariance),
      process_noise_cov(process_noise_cov),
      measurement_noise_cov(measurement_noise_cov),
      adapt_window(adapt_window)
{
    n = state.size();
    if (n <= 0) {
        throw std::invalid_argument("State vector must be non-empty.");
    }
    if (this->adapt_window <= 0) {
        throw std::invalid_argument("Adaptation window must be positive.");
    }
    if (covariance.rows() != n || covariance.cols() != n) {
        throw std::invalid_argument("Initial covariance must be NxN and match state size.");
    }
    if (this->process_noise_cov.rows() != n || this->process_noise_cov.cols() != n) {
        throw std::invalid_argument("Process noise covariance must be NxN and match state size.");
    }
    if (this->measurement_noise_cov.rows() != n || this->measurement_noise_cov.cols() != n) {
        throw std::invalid_argument("Measurement noise covariance must be NxN and match state size.");
    }

    m = this->measurement_noise_cov.rows();
    lambda_ = alpha * alpha * (n + kappa) - n;
    const double n_plus_lambda = n + lambda_;
    if (n_plus_lambda <= 1e-12) {
        throw std::invalid_argument("Invalid UKF parameters: n + lambda must be positive.");
    }
    sigma_point_count = 2 * n + 1;

    weights_mean.resize(sigma_point_count);
    weights_covariance.resize(sigma_point_count);

    weights_mean[0] = lambda_ / n_plus_lambda;
    weights_covariance[0] = weights_mean[0] + (1 - alpha*alpha + beta);
    for (int i = 1; i < sigma_point_count; ++i) {
        weights_mean[i] = 1.0 / (2 * n_plus_lambda);
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
    covariance = 0.5 * (covariance + covariance.transpose());
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

    Eigen::MatrixXd P_ZZ_reg = 0.5 * (P_ZZ + P_ZZ.transpose());
    P_ZZ_reg += Eigen::MatrixXd::Identity(m, m) * 1e-9;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(P_ZZ_reg);
    if (ldlt.info() != Eigen::Success) {
        P_ZZ_reg += Eigen::MatrixXd::Identity(m, m) * 1e-6;
        ldlt.compute(P_ZZ_reg);
    }
    if (ldlt.info() != Eigen::Success) {
        last_nis = std::numeric_limits<double>::quiet_NaN();
        last_measurement_accepted = false;
        return;
    }

    Eigen::VectorXd innovation = measurement - predicted_measurement;
    Eigen::VectorXd innovation_solved = ldlt.solve(innovation);
    if (innovation_solved.hasNaN()) {
        last_nis = std::numeric_limits<double>::quiet_NaN();
        last_measurement_accepted = false;
        return;
    }
    last_nis = innovation.dot(innovation_solved);
    if (outlier_gate_enabled && std::isfinite(last_nis) && last_nis > outlier_nis_threshold) {
        last_measurement_accepted = false;
        return;
    }
    last_measurement_accepted = true;

    Eigen::MatrixXd K = ldlt.solve(P_XZ.transpose()).transpose();

    // Сохраняем инновации для адаптивной оценки
    innovation_history.push_back(innovation);
    if ((int)innovation_history.size() > adapt_window)
        innovation_history.pop_front();

    // Обновление состояния
    state = state + K * innovation;
    covariance = covariance - K * P_ZZ_reg * K.transpose();
    covariance = 0.5 * (covariance + covariance.transpose());
    for (int i = 0; i < n; ++i) {
        covariance(i, i) = std::max(covariance(i, i), 1e-9);
    }

    // Вычисляем остаток (residual) с использованием обновленного состояния
    Eigen::VectorXd residual = measurement - measurementFunction(state);
    residual_history.push_back(residual);
    if ((int)residual_history.size() > adapt_window)
        residual_history.pop_front();

    measurement_history.push_back(measurement);
    if ((int)measurement_history.size() > adapt_window)
        measurement_history.pop_front();

    // Адаптивное обновление Q и R
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

void AdaptiveUnscentedKalmanFilter::setOutlierGate(bool enabled, double nisThreshold) {
    outlier_gate_enabled = enabled;
    outlier_nis_threshold = std::max(1e-9, nisThreshold);
}

void AdaptiveUnscentedKalmanFilter::computeSigmaPoints(std::vector<Eigen::VectorXd> &sigma_points) {
    // Регуляризация ковариационной матрицы для гарантии положительной определенности
    Eigen::MatrixXd cov_reg = 0.5 * (covariance + covariance.transpose());
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(n, n);
    Eigen::LLT<Eigen::MatrixXd> llt;
    double jitter = 1e-9;
    bool decomposition_ok = false;
    for (int attempt = 0; attempt < 6; ++attempt) {
        llt.compute(cov_reg + identity * jitter);
        if (llt.info() == Eigen::Success) {
            decomposition_ok = true;
            break;
        }
        jitter *= 10.0;
    }

    Eigen::MatrixXd sqrt_covariance = Eigen::MatrixXd::Zero(n, n);
    if (decomposition_ok) {
        sqrt_covariance = llt.matrixL();
    } else {
        Eigen::VectorXd safe_diag = cov_reg.diagonal().cwiseMax(1e-6);
        sqrt_covariance = safe_diag.cwiseSqrt().asDiagonal();
    }

    double scaling_factor = std::sqrt(std::max(n + lambda_, 1e-9));

    sigma_points[0] = state;
    for (int i = 0; i < n; ++i) {
        sigma_points[i + 1] = state + scaling_factor * sqrt_covariance.col(i);
        sigma_points[n + i + 1] = state - scaling_factor * sqrt_covariance.col(i);
    }
}

Eigen::VectorXd AdaptiveUnscentedKalmanFilter::stateTransitionFunction(const Eigen::VectorXd &s) {
    // При отсутствии явной модели динамики используем тождественную функцию
    return s;
}

Eigen::VectorXd AdaptiveUnscentedKalmanFilter::measurementFunction(const Eigen::VectorXd &s) {
    // Измеряемые величины совпадают с состоянием (интенсивности)
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

double AdaptiveUnscentedKalmanFilter::getLastNIS() const {
    return last_nis;
}

bool AdaptiveUnscentedKalmanFilter::wasLastMeasurementAccepted() const {
    return last_measurement_accepted;
}

double AdaptiveUnscentedKalmanFilter::g(double Ex) const {
    return erfinv(Ex) / std::sqrt(2.0);
}

double AdaptiveUnscentedKalmanFilter::erfinv(double x) const {
    const double epsilon = 1e-6;
    x = std::max(std::min(x, 1.0 - epsilon), -1.0 + epsilon);

    double a = 0.147;
    double ln = std::log(1 - x * x);
    double term = (2.0 / (M_PI * a)) + (ln / 2.0);
    double val = std::copysign(std::sqrt(std::sqrt(term * term - (ln / a)) - term), x);
    return val;
}

void AdaptiveUnscentedKalmanFilter::adaptProcessNoiseCovariance() {
    // Если недостаточно данных в истории, ничего не обновляем
    const std::size_t window_size = static_cast<std::size_t>(adapt_window);
    if (innovation_history.size() < window_size || residual_history.size() < window_size)
        return;

    // Оценка Q на основе разности между остатками и инновациями
    Eigen::MatrixXd Q_est = Eigen::MatrixXd::Zero(m, m);
    for (int i = 0; i < adapt_window; ++i) {
        Eigen::VectorXd diff = residual_history[i] - innovation_history[i];
        Q_est += diff * diff.transpose();
    }
    Q_est /= adapt_window;

    // Если Q предполагается диагональной, обновляем только диагональ с нижней границей
    Eigen::MatrixXd Q_new = Eigen::MatrixXd::Zero(m, m);
    for (int i = 0; i < m; ++i) {
        Q_new(i, i) = std::max(Q_est(i, i), 1e-6);
    }

    // Экспоненциальное сглаживание с коэффициентом адаптации gamma (например, 0.1)
    double gamma = 0.1;
    process_noise_cov = (1.0 - gamma) * process_noise_cov + gamma * Q_new;

    // Опционально: можно вывести текущие значения Q для мониторинга
    // std::cout << "Updated process_noise_cov (Q): " << process_noise_cov << std::endl;
}

void AdaptiveUnscentedKalmanFilter::adaptMeasurementNoiseCovariance() {
    const std::size_t window_size = static_cast<std::size_t>(adapt_window);
    if (innovation_history.size() < window_size)
        return;

    Eigen::MatrixXd innov_cov = Eigen::MatrixXd::Zero(m, m);
    for (const auto& innov : innovation_history) {
        innov_cov += innov * innov.transpose();
    }
    innov_cov /= innovation_history.size();

    // Обновление R экспоненциальным сглаживанием (коэффициент beta_R, например, 0.1)
    double beta_R = 0.1;
    measurement_noise_cov = (1.0 - beta_R) * measurement_noise_cov + beta_R * innov_cov;

    // Гарантируем, что диагональные элементы не опускаются ниже 1e-6
    for (int i = 0; i < m; ++i) {
        measurement_noise_cov(i, i) = std::max(measurement_noise_cov(i, i), 1e-6);
    }

    // Опционально: можно вывести текущие значения R для мониторинга
    // std::cout << "Updated measurement_noise_cov (R): " << measurement_noise_cov << std::endl;
}
