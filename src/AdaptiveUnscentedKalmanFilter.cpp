#include "AdaptiveUnscentedKalmanFilter.h"
#include <cmath>
#include <algorithm>

AdaptiveUnscentedKalmanFilter::AdaptiveUnscentedKalmanFilter(const Eigen::VectorXd &initial_state,
                                                             const Eigen::MatrixXd &initial_covariance,
                                                             const Eigen::MatrixXd &process_noise_cov,
                                                             const Eigen::MatrixXd &measurement_noise_cov)
    : state(initial_state),
      covariance(initial_covariance),
      process_noise_cov(process_noise_cov),
      measurement_noise_cov(measurement_noise_cov),
      n(initial_state.size()),
      measurement_dim(4) // Размерность измерения (интенсивности I_A, I_B, I_C, I_D)
{
    // Параметры по умолчанию
    alpha = 1e-3;
    beta = 2.0;
    kappa = 0.0;

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

    // Инициализация сигма-точек
    sigma_points.resize(sigma_point_count, Eigen::VectorXd(n));
    predicted_sigma_points.resize(sigma_point_count, Eigen::VectorXd(n));
    predicted_measurements.resize(sigma_point_count, Eigen::VectorXd(measurement_dim));
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

    // Очистка истории инноваций
    innovation_history.clear();
}

void AdaptiveUnscentedKalmanFilter::setProcessNoiseCovariance(double processNoise) {
    process_noise_cov = Eigen::MatrixXd::Identity(n, n) * processNoise;
}

void AdaptiveUnscentedKalmanFilter::setMeasurementNoiseCovariance(double measurementNoise) {
    measurement_noise_cov = Eigen::MatrixXd::Identity(measurement_dim, measurement_dim) * measurementNoise;
}

void AdaptiveUnscentedKalmanFilter::computeSigmaPoints() {
    // Регуляризация ковариации
    Eigen::MatrixXd regularized_covariance = covariance;
    double epsilon = 1e-6; // Малое положительное число
    regularized_covariance += epsilon * Eigen::MatrixXd::Identity(n, n);

    // Вычисление квадратного корня ковариации
    Eigen::MatrixXd sqrt_covariance = regularized_covariance.llt().matrixL();
    double scaling_factor = std::sqrt(n + lambda_);

    sigma_points[0] = state;

    for (int i = 0; i < n; ++i) {
        sigma_points[i + 1] = state + scaling_factor * sqrt_covariance.col(i);
        sigma_points[n + i + 1] = state - scaling_factor * sqrt_covariance.col(i);
    }
}

void AdaptiveUnscentedKalmanFilter::predict() {
    computeSigmaPoints();

    // Прогнозирование сигма-точек
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_sigma_points[i] = stateTransitionFunction(sigma_points[i]);
    }

    // Вычисление среднего предсказанного состояния
    Eigen::VectorXd predicted_state = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_state += weights_mean[i] * predicted_sigma_points[i];
    }

    // Вычисление предсказанной ковариации
    Eigen::MatrixXd predicted_covariance = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < sigma_point_count; ++i) {
        Eigen::VectorXd diff = predicted_sigma_points[i] - predicted_state;
        predicted_covariance += weights_covariance[i] * diff * diff.transpose();
    }
    predicted_covariance += process_noise_cov;

    state = predicted_state;
    covariance = predicted_covariance;
}

void AdaptiveUnscentedKalmanFilter::update(const Eigen::VectorXd &measurement) {
    // Прогноз измерений
    for (int i = 0; i < sigma_point_count; ++i) {
        predicted_measurements[i] = measurementFunction(predicted_sigma_points[i]);
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
        Eigen::VectorXd state_diff = predicted_sigma_points[i] - state;
        Eigen::VectorXd measurement_diff = predicted_measurements[i] - predicted_measurement_mean;
        cross_covariance += weights_covariance[i] * state_diff * measurement_diff.transpose();
    }

    // Вычисление коэффициента Калмана
    Eigen::MatrixXd K = cross_covariance * S.inverse();

    // Вычисление инновации
    Eigen::VectorXd innovation = measurement - predicted_measurement_mean;

    // Сохранение инновации в историю
    innovation_history.push_back(innovation);
    if (innovation_history.size() > N) {
        innovation_history.pop_front();
    }

    // Проверка на выброс
    if (isOutlier(innovation, S)) {
        // Если выброс, пропускаем обновление
        return;
    }

    // Обновление состояния и ковариации
    state += K * innovation;
    covariance -= K * S * K.transpose();

    // Проверка положительной определенности ковариации
    Eigen::LLT<Eigen::MatrixXd> lltOfCov(covariance);
    if (lltOfCov.info() == Eigen::NumericalIssue) {
        // Ковариация не является положительно определенной, выполняем регуляризацию
        double epsilon = 1e-6;
        covariance += epsilon * Eigen::MatrixXd::Identity(n, n);
    }

    // Адаптивная оценка ковариации шума измерения
    adaptMeasurementNoiseCovariance();
}

Eigen::VectorXd AdaptiveUnscentedKalmanFilter::getState() const {
    return state;
}

Eigen::MatrixXd AdaptiveUnscentedKalmanFilter::getCovariance() const {
    return covariance;
}

Eigen::VectorXd AdaptiveUnscentedKalmanFilter::stateTransitionFunction(const Eigen::VectorXd &state) {
    Eigen::VectorXd newState = state;

    // Модель движения лазерного пятна
    double delta_x = 0.001; // Скорость по x
    double delta_y = 0.0;   // Скорость по y
    newState(4) += delta_x; // Обновляем x
    newState(5) += delta_y; // Обновляем y

    // Модель изменения интенсивностей на основе нового положения
    double P0 = 1.0;
    double w = 1.0;
    newState(0) = h(0.5, 0.5, P0, newState(4), newState(5), w);     // I_A
    newState(1) = h(-0.5, 0.5, P0, newState(4), newState(5), w);    // I_B
    newState(2) = h(-0.5, -0.5, P0, newState(4), newState(5), w);   // I_C
    newState(3) = h(0.5, -0.5, P0, newState(4), newState(5), w);    // I_D

    return newState;
}

Eigen::VectorXd AdaptiveUnscentedKalmanFilter::measurementFunction(const Eigen::VectorXd &state) {
    Eigen::VectorXd measurement = state.head(4); // Интенсивности I_A, I_B, I_C, I_D
    return measurement;
}

Eigen::Vector2d AdaptiveUnscentedKalmanFilter::calculateSpotPosition() {
    double x = state(4);
    double y = state(5);
    return Eigen::Vector2d(x, y);
}

// Функция для вычисления интенсивности
double AdaptiveUnscentedKalmanFilter::h(double x_det, double y_det, double P0, double X_beam, double Y_beam, double w) {
    return 2 * P0 / (M_PI * w * w) * std::exp(-2 * ((x_det - X_beam)*(x_det - X_beam) + (y_det - Y_beam)*(y_det - Y_beam)) / (w * w));
}

// Адаптация ковариации шума измерения с использованием робастной оценки
void AdaptiveUnscentedKalmanFilter::adaptMeasurementNoiseCovariance() {
    if (innovation_history.size() < N) {
        // Недостаточно данных для обновления
        return;
    }

    measurement_noise_cov = computeRobustCovariance();
}

// Вычисление робастной ковариации с использованием MAD
Eigen::MatrixXd AdaptiveUnscentedKalmanFilter::computeRobustCovariance() {
    int m = innovation_history[0].size();
    int n = innovation_history.size();
    Eigen::MatrixXd innovations(n, m);

    for (int i = 0; i < n; ++i) {
        innovations.row(i) = innovation_history[i];
    }

    // Вычисляем медиану по каждому столбцу
    Eigen::RowVectorXd median = computeColumnMedian(innovations);

    // Вычисляем абсолютные отклонения от медианы
    Eigen::MatrixXd abs_deviation = (innovations.rowwise() - median).cwiseAbs();

    // Вычисляем MAD по каждому столбцу
    Eigen::RowVectorXd mad = computeColumnMedian(abs_deviation);

    // Преобразуем MAD в робастное стандартное отклонение
    Eigen::VectorXd robust_std = mad.transpose() / 0.6745; // 0.6745 — квантиль нормального распределения

    // Формируем робастную ковариационную матрицу
    Eigen::MatrixXd robust_covariance = robust_std.array().square().matrix().asDiagonal();

    return robust_covariance;
}

// Функция для вычисления медианы каждого столбца матрицы
Eigen::RowVectorXd AdaptiveUnscentedKalmanFilter::computeColumnMedian(const Eigen::MatrixXd& data) {
    Eigen::Index cols = data.cols();
    Eigen::RowVectorXd medians(cols);

    for (Eigen::Index i = 0; i < cols; ++i) {
        // Копируем столбец в вектор
        std::vector<double> colData(data.rows());
        for (Eigen::Index j = 0; j < data.rows(); ++j) {
            colData[j] = data(j, i);
        }

        // Сортируем вектор
        std::sort(colData.begin(), colData.end());

        // Вычисляем медиану
        size_t n = colData.size();
        if (n % 2 == 0) {
            medians(i) = (colData[n / 2 - 1] + colData[n / 2]) / 2.0;
        } else {
            medians(i) = colData[n / 2];
        }
    }

    return medians;
}

// Проверка на выброс с использованием расстояния Махаланобиса
bool AdaptiveUnscentedKalmanFilter::isOutlier(const Eigen::VectorXd &innovation, const Eigen::MatrixXd &S) {
    double mahalanobis_distance = innovation.transpose() * S.inverse() * innovation;
    double threshold = chiSquareThreshold(innovation.size(), 0.99); // Уровень значимости 99%
    return mahalanobis_distance > threshold;
}

// Порог для распределения хи-квадрат
double AdaptiveUnscentedKalmanFilter::chiSquareThreshold(int degrees_of_freedom, double confidence_level) {
    // Для degrees_of_freedom = 4 и confidence_level = 0.99, threshold ≈ 13.28
    if (degrees_of_freedom == 4 && confidence_level == 0.99) {
        return 13.28;
    }
    // Добавьте другие значения по необходимости
    return 0.0;
}
