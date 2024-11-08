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
    n_x = state.size();
    n_z = measurement_noise_cov.rows(); // Предполагаем, что R квадратная

    lambda_ = alpha * alpha * (n_x + kappa) - n_x;

    // Инициализация весов
    int sigma_point_count = 2 * n_x + 1;
    Wm = VectorXd(sigma_point_count);
    Wc = VectorXd(sigma_point_count);
    Wm(0) = lambda_ / (n_x + lambda_);
    Wc(0) = Wm(0) + (1 - alpha * alpha + beta);
    for (int i = 1; i < sigma_point_count; ++i) {
        Wm(i) = 1.0 / (2 * (n_x + lambda_));
        Wc(i) = Wm(i);
    }

    // Функции перехода и измерения
    f = f_func ? f_func : [this](const VectorXd& x) { return defaultStateTransition(x); };
    h = h_func ? h_func : [this](const VectorXd& x) { return defaultMeasurementFunction(x); };

    // Матрица управления системным шумом
    Gamma = MatrixXd::Identity(n_x, n_x);
}

VectorXd AdaptiveUnscentedKalmanFilter::getState() const {
    return state;
}

MatrixXd AdaptiveUnscentedKalmanFilter::getCovariance() const {
    return P;
}

MatrixXd AdaptiveUnscentedKalmanFilter::computeSigmaPoints(const VectorXd& state, const MatrixXd& covariance) {
    int sigma_point_count = 2 * n_x + 1;
    MatrixXd sigma_points(n_x, sigma_point_count);
    sigma_points.col(0) = state;

    // Проверка положительной определенности
    Eigen::MatrixXd covariance_adjusted = covariance;
    SelfAdjointEigenSolver<MatrixXd> eigen_solver(covariance);
    VectorXd eigenvalues = eigen_solver.eigenvalues();
    if (eigenvalues.minCoeff() <= 0) {
        // Добавляем небольшое значение к диагонали
        covariance_adjusted += MatrixXd::Identity(n_x, n_x) * 1e-6;
    }

    MatrixXd sqrt_matrix = covariance_adjusted.llt().matrixL();
    sqrt_matrix *= std::sqrt(n_x + lambda_);

    for (int i = 0; i < n_x; ++i) {
        sigma_points.col(i + 1) = state + sqrt_matrix.col(i);
        sigma_points.col(i + 1 + n_x) = state - sqrt_matrix.col(i);
    }

    return sigma_points;
}

void AdaptiveUnscentedKalmanFilter::predict() {
    // Шаг 2: Расчет сигма-точек
    MatrixXd sigma_points = computeSigmaPoints(state, P);

    // Шаг 3: Прогнозирование состояния
    int sigma_point_count = 2 * n_x + 1;
    sigma_points_pred.resize(n_x, sigma_point_count);
    for (int i = 0; i < sigma_point_count; ++i) {
        VectorXd sp = sigma_points.col(i);
        sigma_points_pred.col(i) = f(sp);
    }

    // Прогнозированное состояние
    x_pred = VectorXd::Zero(n_x);
    for (int i = 0; i < sigma_point_count; ++i) {
        x_pred += Wm(i) * sigma_points_pred.col(i);
    }

    // Прогнозированная ковариация состояния
    P_pred = MatrixXd::Zero(n_x, n_x);
    for (int i = 0; i < sigma_point_count; ++i) {
        VectorXd diff = sigma_points_pred.col(i) - x_pred;
        P_pred += Wc(i) * diff * diff.transpose();
    }
    P_pred += Q;
}

void AdaptiveUnscentedKalmanFilter::update(const VectorXd& z) {
    // Шаг 4: Прогноз измерения
    int sigma_point_count = 2 * n_x + 1;
    MatrixXd Z_pred(n_z, sigma_point_count);
    for (int i = 0; i < sigma_point_count; ++i) {
        VectorXd sp = sigma_points_pred.col(i);
        Z_pred.col(i) = h(sp);
    }

    VectorXd z_pred = VectorXd::Zero(n_z);
    for (int i = 0; i < sigma_point_count; ++i) {
        z_pred += Wm(i) * Z_pred.col(i);
    }

    // Ковариация измерения
    MatrixXd P_zz = MatrixXd::Zero(n_z, n_z);
    for (int i = 0; i < sigma_point_count; ++i) {
        VectorXd diff = Z_pred.col(i) - z_pred;
        P_zz += Wc(i) * diff * diff.transpose();
    }
    P_zz += R;

    // Кросс-ковариация между состоянием и измерением
    MatrixXd P_xz = MatrixXd::Zero(n_x, n_z);
    for (int i = 0; i < sigma_point_count; ++i) {
        VectorXd x_diff = sigma_points_pred.col(i) - x_pred;
        VectorXd z_diff = Z_pred.col(i) - z_pred;
        P_xz += Wc(i) * x_diff * z_diff.transpose();
    }

    // Шаг 5: Расчет коэффициента усиления Калмана
    MatrixXd K;
    JacobiSVD<MatrixXd> svd(P_zz, ComputeThinU | ComputeThinV);
    if (svd.rank() == P_zz.rows()) {
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
        int m = innovation_history[0].size();
        MatrixXd delta(m, M);
        for (int i = 0; i < M; ++i) {
            delta.col(i) = residual_history[i] - innovation_history[i];
        }

        MatrixXd E_delta = (delta * delta.transpose()) / M; // Ковариация

        // Вычисляем необходимые матрицы
        MatrixXd H_k = calculateJacobian(h, state);
        MatrixXd H_k_pred = calculateJacobian(h, x_pred);

        // Решаем уравнение (19)
        MatrixXd left_term = H_k_pred * Gamma * Q * Gamma.transpose() * H_k_pred.transpose();
        MatrixXd right_term = E_delta - H_k_pred * P_pred * H_k_pred.transpose() + H_k * P * H_k.transpose();

        // Предполагая, что Gamma = I
        MatrixXd H_prod = H_k_pred * H_k_pred.transpose();
        if (H_prod.determinant() != 0) {
            Q = 0.9 * Q + 0.1 * H_prod.inverse() * right_term;
        } else {
            Q = 0.9 * Q + 0.1 * H_prod.completeOrthogonalDecomposition().pseudoInverse() * right_term;
        }

        // Ограничение Q для обеспечения положительной определенности
        SelfAdjointEigenSolver<MatrixXd> eigen_solver(Q);
        VectorXd eigenvalues = eigen_solver.eigenvalues();
        if (eigenvalues.minCoeff() < 0) {
            Q += MatrixXd::Identity(n_x, n_x) * (1e-6 - eigenvalues.minCoeff());
        }
    }
}

void AdaptiveUnscentedKalmanFilter::adaptMeasurementNoiseCovariance() {
    if (innovation_history.size() >= M) {
        // Собираем инновации в матрицу
        int m = innovation_history[0].size();
        MatrixXd innovations(m, M);
        for (int i = 0; i < M; ++i) {
            innovations.col(i) = innovation_history[i];
        }

        MatrixXd R_new = (innovations * innovations.transpose()) / M;

        // Рекурсивное обновление R
        R = b * R + (1 - b) * R_new;

        // Ограничение R для обеспечения положительной определенности
        SelfAdjointEigenSolver<MatrixXd> eigen_solver(R);
        VectorXd eigenvalues = eigen_solver.eigenvalues();
        if (eigenvalues.minCoeff() < 0) {
            R += MatrixXd::Identity(n_z, n_z) * (1e-6 - eigenvalues.minCoeff());
        }
    }
}

MatrixXd AdaptiveUnscentedKalmanFilter::calculateJacobian(
    const std::function<VectorXd(const VectorXd&)>& func,
    const VectorXd& x
) {
    double eps = 1e-5;
    int n = x.size();
    VectorXd fx = func(x);
    int m = fx.size();
    MatrixXd jacobian(m, n);

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
