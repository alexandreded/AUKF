#include "AdaptiveUnscentedKalmanFilter.h"
#include <cmath>

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

    // Используем разложение Холецкого с помощью Eigen для квадратного корня ковариационной матрицы
    Eigen::MatrixXd sqrt_covariance = covariance.llt().matrixL();

    sigma_points[0] = state;

    for (int i = 0; i < n; ++i) {
        sigma_points[i + 1] = state + scaling_factor * sqrt_covariance.col(i);
        sigma_points[n + i + 1] = state - scaling_factor * sqrt_covariance.col(i);
    }

    return sigma_points;
}


//Eigen::MatrixXd AdaptiveUnscentedKalmanFilter::choleskyDecomposition(const Eigen::MatrixXd &matrix) {
 //   return matrix.llt().matrixL(); // возвращает нижнюю треугольную часть разложения Холецкого
//}

void AdaptiveUnscentedKalmanFilter::predict() {
    auto sigma_points = computeSigmaPoints();
    std::vector<Eigen::VectorXd> predicted_sigma_points(sigma_point_count, Eigen::VectorXd(n));

    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_sigma_points[i] = sigma_points[i];  // Здесь может быть нелинейная функция перехода состояния
    }

    // Вычисление среднего предсказанного состояния
    Eigen::VectorXd predicted_state = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_state += weights_mean[i] * predicted_sigma_points[i];
    }

    // Вычисление предсказанной ковариации с добавлением шума процесса
    Eigen::MatrixXd predicted_covariance = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < sigma_point_count; ++i) {
        Eigen::VectorXd diff = predicted_sigma_points[i] - predicted_state;
        predicted_covariance += weights_covariance[i] * diff * diff.transpose();
    }
    predicted_covariance += process_noise_cov;  // Добавление шума процесса

    state = predicted_state;
    covariance = predicted_covariance;
}


void AdaptiveUnscentedKalmanFilter::update(const Eigen::VectorXd &measurement) {
    auto sigma_points = computeSigmaPoints();
    int measurement_dim = measurement.size();

    // Прогноз измерений
    std::vector<Eigen::VectorXd> predicted_measurements(sigma_point_count, Eigen::VectorXd(measurement_dim));
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_measurements[i] = sigma_points[i];  // Модель измерения
    }

    // Среднее предсказанное измерение
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

    // Кросс-ковариация состояния и измерения
    Eigen::MatrixXd cross_covariance = Eigen::MatrixXd::Zero(n, measurement_dim);
    for (int i = 0; i < sigma_point_count; ++i) {
        Eigen::VectorXd state_diff = sigma_points[i] - state;
        Eigen::VectorXd measurement_diff = predicted_measurements[i] - predicted_measurement_mean;
        cross_covariance += weights_covariance[i] * state_diff * measurement_diff.transpose();
    }

    // Вычисление коэффициента Калмана
    Eigen::MatrixXd K = cross_covariance * S.inverse();

    // Обновление состояния и ковариации
    Eigen::VectorXd innovation = measurement - predicted_measurement_mean;
    state += K * innovation;
    covariance -= K * S * K.transpose();
}


Eigen::VectorXd AdaptiveUnscentedKalmanFilter::getState() const {
    return state;
}

Eigen::MatrixXd AdaptiveUnscentedKalmanFilter::getCovariance() const {
    return covariance;
}

double AdaptiveUnscentedKalmanFilter::h(double x, double y, double P0, double X, double Y, double w) {
    return 2 * P0 / (M_PI * w * w) * std::exp(-2 * ((x - X)*(x - X) + (y - Y)*(y - Y)) / (w * w));
}

double AdaptiveUnscentedKalmanFilter::g(double Ex) {
    return erfinv(Ex) / std::sqrt(2.0);
}

double AdaptiveUnscentedKalmanFilter::lambdaFunc(double x0) {
    double a = 1.0;
    double b = 0.0;
    return a * x0 + b;
}

Eigen::Vector2d AdaptiveUnscentedKalmanFilter::calculateSpotPosition(double I_A, double I_B, double I_C, double I_D, double w, double x0) {
    double sum_I = I_A + I_B + I_C + I_D;
    double E_X = ((I_A + I_D) - (I_B + I_C)) / sum_I;
    double E_Y = ((I_A + I_B) - (I_C + I_D)) / sum_I;

    double x = g(E_X) * w * lambdaFunc(x0);
    double y = g(E_Y) * w * lambdaFunc(x0);

    return Eigen::Vector2d(x, y);
}

double AdaptiveUnscentedKalmanFilter::erfinv(double x) {
    double a = 0.147;
    double ln = std::log(1 - x * x);
    double term = (2 / (M_PI * a)) + (ln / 2);
    double erfinv = std::copysign(std::sqrt(std::sqrt(term * term - (ln / a)) - term), x);

    return erfinv;
}
