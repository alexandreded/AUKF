// AdaptiveUnscentedKalmanFilter.cpp

#include "AdaptiveUnscentedKalmanFilter.h"
#include <cmath>
#include <iostream>

using namespace Eigen;

AdaptiveUnscentedKalmanFilter::AdaptiveUnscentedKalmanFilter(
    const VectorXd& initial_state,
    const MatrixXd& initial_covariance,
    const MatrixXd& process_noise_cov,
    const MatrixXd& measurement_noise_cov,
    StateTransitionFunction f_func,
    MeasurementFunction h_func,
    double alpha_param,
    double beta_param,
    double kappa_param
) : state(initial_state),
    P(initial_covariance),
    Q(process_noise_cov),
    R(measurement_noise_cov),
    alpha(alpha_param),
    beta(beta_param),
    kappa(kappa_param)
{
    n = state.size();

    lambda_ = alpha * alpha * (n + kappa) - n;

    // Инициализация весов
    Wm = VectorXd(2 * n + 1);
    Wc = VectorXd(2 * n + 1);
    Wm(0) = lambda_ / (n + lambda_);
    Wc(0) = Wm(0) + (1 - alpha * alpha + beta);
    for (int i = 1; i < 2 * n + 1; ++i) {
        Wm(i) = 1.0 / (2 * (n + lambda_));
        Wc(i) = Wm(i);
    }

    // Функции перехода и измерения
    f = f_func ? f_func : [this](const VectorXd& x) { return defaultStateTransition(x); };
    h = h_func ? h_func : [this](const VectorXd& x) { return defaultMeasurementFunction(x); };

    // Матрица управления системным шумом
    Gamma = MatrixXd::Identity(n, n);
}

VectorXd AdaptiveUnscentedKalmanFilter::getState() const {
    return state;
}

MatrixXd AdaptiveUnscentedKalmanFilter::getCovariance() const {
    return P;
}

MatrixXd AdaptiveUnscentedKalmanFilter::computeSigmaPoints(const VectorXd& state, const MatrixXd& covariance) {
    MatrixXd sigma_points(2 * n + 1, n);
    sigma_points.row(0) = state.transpose();

    MatrixXd sqrt_matrix = covariance.llt().matrixL();
    sqrt_matrix *= std::sqrt(n + lambda_);

    for (int i = 0; i < n; ++i) {
        sigma_points.row(i + 1) = (state + sqrt_matrix.col(i)).transpose();
        sigma_points.row(n + i + 1) = (state - sqrt_matrix.col(i)).transpose();
    }

    return sigma_points;
}

void AdaptiveUnscentedKalmanFilter::predict() {
    // Шаг 2: Расчет сигма-точек
    MatrixXd sigma_points = computeSigmaPoints(state, P);

    // Шаг 3: Прогнозирование состояния
    sigma_points_pred.resize(2 * n + 1, n);
    for (int i = 0; i < 2 * n + 1; ++i) {
        VectorXd sp = sigma_points.row(i).transpose();
        sigma_points_pred.row(i) = f(sp).transpose();
    }

    // Прогнозированное состояние
    x_pred = VectorXd::Zero(n);
    for (int i = 0; i < 2 * n + 1; ++i) {
        x_pred += Wm(i) * sigma_points_pred.row(i).transpose();
    }

    // Прогнозированная ковариация состояния
    P_pred = Q;
    for (int i = 0; i < 2 * n + 1; ++i) {
        VectorXd diff = sigma_points_pred.row(i).transpose() - x_pred;
        P_pred += Wc(i) * diff * diff.transpose();
    }
}

void AdaptiveUnscentedKalmanFilter::update(const VectorXd& z) {
    // Шаг 4: Прогноз измерения
    MatrixXd Z_pred(2 * n + 1, n);
    for (int i = 0; i < 2 * n + 1; ++i) {
        VectorXd sp = sigma_points_pred.row(i).transpose();
        Z_pred.row(i) = h(sp).transpose();
    }

    VectorXd z_pred = VectorXd::Zero(n);
    for (int i = 0; i < 2 * n + 1; ++i) {
        z_pred += Wm(i) * Z_pred.row(i).transpose();
    }

    // Ковариация измерения
    MatrixXd P_zz = R;
    for (int i = 0; i < 2 * n + 1; ++i) {
        VectorXd diff = Z_pred.row(i).transpose() - z_pred;
        P_zz += Wc(i) * diff * diff.transpose();
    }

    // Кросс-ковариация между состоянием и измерением
    MatrixXd P_xz = MatrixXd::Zero(n, n);
    for (int i = 0; i < 2 * n + 1; ++i) {
        VectorXd x_diff = sigma_points_pred.row(i).transpose() - x_pred;
        VectorXd z_diff = Z_pred.row(i).transpose() - z_pred;
        P_xz += Wc(i) * x_diff * z_diff.transpose();
    }

    // Шаг 5: Расчет коэффициента усиления Калмана
    MatrixXd K;
    if (P_zz.determinant() != 0) {
        K = P_xz * P_zz.inverse();
    } else {
        K = P_xz * P_zz.completeOrthogonalDecomposition().pseudoInverse();
    }

    // Шаг 6: Обновление состояния и ковариации
    VectorXd y = z - z_pred; // Инновация
    state = x_pred + K * y;
    P = P_pred - K * P_zz * K.transpose();

    // Сохранение инноваций и остатков
    VectorXd epsilon = y;
    VectorXd eta = z - h(state);
    innovation_history.push_back(epsilon);
    residual_history.push_back(eta);

    // Ограничиваем размер истории
    if (innovation_history.size() > M) {
        innovation_history.erase(innovation_history.begin());
        residual_history.erase(residual_history.begin());
    }

    // Адаптивная оценка Q и R
    adaptProcessNoiseCovariance();
    adaptMeasurementNoiseCovariance();
}

void AdaptiveUnscentedKalmanFilter::adaptProcessNoiseCovariance() {
    if (innovation_history.size() >= M) {
        // Вычисляем разницу (η_k - ε_k)
        MatrixXd delta(n, M);
        for (int i = 0; i < M; ++i) {
            delta.col(i) = residual_history[i] - innovation_history[i];
        }

        MatrixXd E_delta = (delta * delta.transpose()) / M; // Ковариация

        // Вычисляем необходимые матрицы
        MatrixXd H_k = calculateJacobian(h, state);
        MatrixXd H_k_pred = calculateJacobian(h, x_pred);
        MatrixXd P_k = P;

        // Вычисляем сумму весов для сигма-точек
        MatrixXd sum_Wc = MatrixXd::Zero(n, n);
        for (int i = 0; i < 2 * n + 1; ++i) {
            VectorXd diff = sigma_points_pred.row(i).transpose() - x_pred;
            sum_Wc += Wc(i) * diff * diff.transpose();
        }

        // Решаем уравнение (19)
        MatrixXd left_term = H_k_pred * Gamma * Q * Gamma.transpose() * H_k_pred.transpose();
        MatrixXd right_term = E_delta - H_k_pred * sum_Wc * H_k_pred.transpose() + H_k * P_k * H_k.transpose();

        // Предполагая, что Gamma = I
        MatrixXd H_prod = H_k_pred * H_k_pred.transpose();
        if (H_prod.determinant() != 0) {
            Q = 0.9 * Q + 0.1 * H_prod.inverse() * right_term;
        } else {
            Q = 0.9 * Q + 0.1 * H_prod.completeOrthogonalDecomposition().pseudoInverse() * right_term;
        }
    }
}

void AdaptiveUnscentedKalmanFilter::adaptMeasurementNoiseCovariance() {
    if (innovation_history.size() >= M) {
        // Собираем инновации в матрицу
        MatrixXd innovations(n, M);
        for (int i = 0; i < M; ++i) {
            innovations.col(i) = innovation_history[i];
        }

        MatrixXd R_new = (innovations * innovations.transpose()) / M;

        // Рекурсивное обновление R
        R = b * R + (1 - b) * R_new;
    }
}

MatrixXd AdaptiveUnscentedKalmanFilter::calculateJacobian(
    const std::function<VectorXd(const VectorXd&)>& func,
    const VectorXd& x
) {
    double eps = 1e-5;
    int n = x.size();
    VectorXd fx = func(x);
    MatrixXd jacobian(n, n);

    for (int i = 0; i < n; ++i) {
        VectorXd x_eps = x;
        x_eps(i) += eps;
        VectorXd fx_eps = func(x_eps);
        jacobian.col(i) = (fx_eps - fx) / eps;
    }

    return jacobian;
}

VectorXd AdaptiveUnscentedKalmanFilter::defaultStateTransition(const VectorXd& x) {
    return x; // Тождественная функция
}

VectorXd AdaptiveUnscentedKalmanFilter::defaultMeasurementFunction(const VectorXd& x) {
    return x; // Тождественная функция
}
