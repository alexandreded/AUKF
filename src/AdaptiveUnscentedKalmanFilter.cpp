#include "AdaptiveUnscentedKalmanFilter.h"
#include <cmath>

// Конструктор
AdaptiveUnscentedKalmanFilter::AdaptiveUnscentedKalmanFilter(const Eigen::VectorXd &initial_state,
                                                             const Eigen::MatrixXd &initial_covariance,
                                                             const Eigen::MatrixXd &process_noise_cov,
                                                             const Eigen::MatrixXd &measurement_noise_cov)
    : state(initial_state),
      covariance(initial_covariance),
      process_noise_cov(process_noise_cov),
      measurement_noise_cov(measurement_noise_cov),
      n(initial_state.size()),
      m(measurement_noise_cov.rows())
{
    // Параметры по умолчанию
    alpha = 1e-3; // Обычно 1e-3
    beta = 2.0;    // Оптимально для гауссовского распределения
    kappa = 0.0;   // Часто устанавливается в 0

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
    measurement_history.clear();
}

void AdaptiveUnscentedKalmanFilter::setProcessNoiseCovariance(const Eigen::MatrixXd &process_noise_cov) {
    this->process_noise_cov = process_noise_cov;
}

void AdaptiveUnscentedKalmanFilter::setMeasurementNoiseCovariance(const Eigen::MatrixXd &measurement_noise_cov) {
    this->measurement_noise_cov = measurement_noise_cov;
}

void AdaptiveUnscentedKalmanFilter::computeSigmaPoints(std::vector<Eigen::VectorXd> &sigma_points) {
    double scaling_factor = sqrt(n + lambda_);

    // Разложение Холецкого
    Eigen::MatrixXd sqrt_covariance = covariance.llt().matrixL();

    sigma_points[0] = state;

    for (int i = 0; i < n; ++i) {
        sigma_points[i + 1] = state + scaling_factor * sqrt_covariance.col(i);
        sigma_points[n + i + 1] = state - scaling_factor * sqrt_covariance.col(i);
    }
}

Eigen::VectorXd AdaptiveUnscentedKalmanFilter::stateTransitionFunction(const Eigen::VectorXd &state) {
    // Поскольку динамика неизвестна, состояние остается неизменным
    return state;
}

Eigen::VectorXd AdaptiveUnscentedKalmanFilter::measurementFunction(const Eigen::VectorXd &state) {
    // Функция измерения: тождественная, поскольку мы измеряем интенсивности напрямую
    return state;
}

void AdaptiveUnscentedKalmanFilter::predict() {
    // Шаг 2: Расчет сигма-точек
    std::vector<Eigen::VectorXd> sigma_points(sigma_point_count, Eigen::VectorXd(n));
    computeSigmaPoints(sigma_points);

    // Шаг 3: Прогнозирование состояния
    std::vector<Eigen::VectorXd> predicted_sigma_points(sigma_point_count, Eigen::VectorXd(n));
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_sigma_points[i] = stateTransitionFunction(sigma_points[i]);
    }

    // Апостериорная оценка состояния
    Eigen::VectorXd predicted_state = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_state += weights_mean[i] * predicted_sigma_points[i];
    }

    // Ковариация апостериорной ошибки прогноза
    Eigen::MatrixXd P_XX = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < sigma_point_count; ++i) {
        Eigen::VectorXd diff = predicted_sigma_points[i] - predicted_state;
        P_XX += weights_covariance[i] * diff * diff.transpose();
    }

    // Апостериорная ковариация прогноза состояния
    covariance = P_XX + process_noise_cov; // P_{k/k-1}

    // Обновление состояния
    state = predicted_state;
}

void AdaptiveUnscentedKalmanFilter::update(const Eigen::VectorXd &measurement) {
    // Шаг 4: Прогноз измерения
    std::vector<Eigen::VectorXd> sigma_points(sigma_point_count, Eigen::VectorXd(n));
    computeSigmaPoints(sigma_points);

    std::vector<Eigen::VectorXd> predicted_measurements(sigma_point_count, Eigen::VectorXd(m));
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_measurements[i] = measurementFunction(sigma_points[i]);
    }

    // Прогнозируемое значение измерения
    Eigen::VectorXd predicted_measurement = Eigen::VectorXd::Zero(m);
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_measurement += weights_mean[i] * predicted_measurements[i];
    }

    // Ковариация измерения
    Eigen::MatrixXd P_ZZ = Eigen::MatrixXd::Zero(m, m);
    for (int i = 0; i < sigma_point_count; ++i) {
        Eigen::VectorXd diff = predicted_measurements[i] - predicted_measurement;
        P_ZZ += weights_covariance[i] * diff * diff.transpose();
    }
    P_ZZ += measurement_noise_cov; // Добавление R_k

    // Кросс-ковариация между состоянием и измерением
    Eigen::MatrixXd P_XZ = Eigen::MatrixXd::Zero(n, m);
    for (int i = 0; i < sigma_point_count; ++i) {
        Eigen::VectorXd state_diff = sigma_points[i] - state;
        Eigen::VectorXd meas_diff = predicted_measurements[i] - predicted_measurement;
        P_XZ += weights_covariance[i] * state_diff * meas_diff.transpose();
    }

    // Шаг 5: Расчет коэффициента усиления Калмана
    Eigen::MatrixXd K = P_XZ * P_ZZ.inverse();

    // Инновация
    Eigen::VectorXd innovation = measurement - predicted_measurement;

    // Сохранение инновации
    innovation_history.push_back(innovation);
    if (innovation_history.size() > adapt_window) {
        innovation_history.pop_front();
    }

    // Обновление состояния и ковариации
    state = state + K * innovation;
    covariance = covariance - K * P_ZZ * K.transpose();

    // Остаток
    Eigen::VectorXd residual = measurement - measurementFunction(state);
    residual_history.push_back(residual);
    if (residual_history.size() > adapt_window) {
        residual_history.pop_front();
    }

    // Накопление измерений
    measurement_history.push_back(measurement);
    if (measurement_history.size() > adapt_window) {
        measurement_history.pop_front();
    }

    // Адаптивная оценка Q и R
    adaptProcessNoiseCovariance();
    adaptMeasurementNoiseCovariance();
}

Eigen::VectorXd AdaptiveUnscentedKalmanFilter::getState() const {
    return state;
}

Eigen::MatrixXd AdaptiveUnscentedKalmanFilter::getCovariance() const {
    return covariance;
}

Eigen::Vector2d AdaptiveUnscentedKalmanFilter::calculateSpotPosition() const {
    // Вычисление координат (x, y) на основе отфильтрованных интенсивностей
    double I_A = state(0);
    double I_B = state(1);
    double I_C = state(2);
    double I_D = state(3);

    double sum_I = I_A + I_B + I_C + I_D;

    if (sum_I == 0) {
        // Обработка случая деления на ноль
        return Eigen::Vector2d(0.0, 0.0);
    }

    double E_X = ((I_A + I_D) - (I_B + I_C)) / sum_I;
    double E_Y = ((I_A + I_B) - (I_C + I_D)) / sum_I;

    
    // Параметры системы
    double w = 1.0;  // Ширина пучка (может быть настроена)
    double x0 = 1.0; // Параметр для функции lambdaFunc (если требуется)

    double x = g(E_X) * w * x0;
    double y = g(E_Y) * w * x0;

    return Eigen::Vector2d(x, y);
}

double AdaptiveUnscentedKalmanFilter::g(double Ex) const{
    return erfinv(Ex) / std::sqrt(2.0);
}

double AdaptiveUnscentedKalmanFilter::erfinv(double x) const{
    // Ограничение x в пределах (-1 + ε, 1 - ε)
    const double epsilon = 1e-6;
    x = std::max(std::min(x, 1.0 - epsilon), -1.0 + epsilon);

    // Аппроксимация обратной функции ошибки
    double a = 0.147;
    double ln = std::log(1 - x * x);
    double term = (2 / (M_PI * a)) + (ln / 2);
    double erfinv = std::copysign(std::sqrt(std::sqrt(term * term - (ln / a)) - term), x);

    return erfinv;
}

void AdaptiveUnscentedKalmanFilter::adaptProcessNoiseCovariance() {
    if (innovation_history.size() < adapt_window || residual_history.size() < adapt_window) {
        return;
    }

    // Вычисление E[(eta_k - epsilon_k)(eta_k - epsilon_k)^T]
    Eigen::MatrixXd E_diff = Eigen::MatrixXd::Zero(m, m);
    for (int i = 0; i < adapt_window; ++i) {
        Eigen::VectorXd diff = residual_history[i] - innovation_history[i];
        E_diff += diff * diff.transpose();
    }
    E_diff /= adapt_window;

    // Вычисление матрицы H_k
    // Для тождественной функции измерения H_k = I
    Eigen::MatrixXd H_k = Eigen::MatrixXd::Identity(m, n);

    // Решение уравнения для Q
    Eigen::MatrixXd term = E_diff - H_k * covariance * H_k.transpose() + H_k * covariance * H_k.transpose();
    Eigen::MatrixXd Q_k = term;

    // Обновляем Q
    process_noise_cov = Q_k;
}

void AdaptiveUnscentedKalmanFilter::adaptMeasurementNoiseCovariance() {
    if (innovation_history.size() < adapt_window) {
        return;
    }

    // Вычисляем ковариацию инноваций
    Eigen::MatrixXd E_innovations = Eigen::MatrixXd::Zero(m, m);
    int count = innovation_history.size();
    for (const auto& innovation : innovation_history) {
        E_innovations += innovation * innovation.transpose();
    }
    E_innovations /= count;

    // Рекурсивная формула для обновления R
    double b = 0.95; // Параметр сглаживания, можно настроить
    int k = measurement_history.size();
    double d_k = (1.0 - b) / (1.0 - std::pow(b, k + 1));

    measurement_noise_cov = (1.0 - d_k) * measurement_noise_cov + d_k * E_innovations;
}
