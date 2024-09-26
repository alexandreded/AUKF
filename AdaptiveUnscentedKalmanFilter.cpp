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
    f = f_func ? f_func : std::bind(&AdaptiveUnscentedKalmanFilter::defaultStateTransition, this, std::placeholders::_1);
    h = h_func ? h_func : std::bind(&AdaptiveUnscentedKalmanFilter::defaultMeasurementFunction, this, std::placeholders::_1);

    // Матрица управления системным шумом
    Gamma = MatrixXd::Identity(n, n);
}

VectorXd AdaptiveUnscentedKalmanFilter::getState() const {
    return state;
}

MatrixXd AdaptiveUnscentedKalmanFilter::getCovariance() const {
    return P;
}

Eigen::MatrixXd AdaptiveUnscentedKalmanFilter::computeSigmaPoints(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance) {
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
    sigma_points_pred = MatrixXd(2 * n + 1, n);
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
        K = P_xz * P_zz
