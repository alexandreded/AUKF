#ifndef ADAPTIVE_UNSCENTED_KALMAN_FILTER_H
#define ADAPTIVE_UNSCENTED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <functional>
#include <vector>

class AdaptiveUnscentedKalmanFilter {
public:
    using StateTransitionFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
    using MeasurementFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;

    AdaptiveUnscentedKalmanFilter(
        const Eigen::VectorXd& initial_state,
        const Eigen::MatrixXd& initial_covariance,
        const Eigen::MatrixXd& process_noise_cov,
        const Eigen::MatrixXd& measurement_noise_cov,
        StateTransitionFunction f = nullptr,
        MeasurementFunction h = nullptr,
        double alpha = 1e-3,
        double beta = 2.0,
        double kappa = 0.0
    );

    void predict();
    void update(const Eigen::VectorXd& z);

    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;

private:
    int n_x; // Размерность состояния
    int n_z; // Размерность измерения

    // Параметры UKF
    double alpha;
    double beta;
    double kappa;
    double lambda_;

    Eigen::VectorXd Wm; // Веса для среднего
    Eigen::VectorXd Wc; // Веса для ковариации

    Eigen::VectorXd state; // Текущее состояние
    Eigen::MatrixXd P;     // Ковариация состояния
    Eigen::MatrixXd Q;     // Ковариация шума процесса
    Eigen::MatrixXd R;     // Ковариация шума измерения

    StateTransitionFunction f; // Функция перехода состояния
    MeasurementFunction h;     // Функция измерения

    // Переменные для предсказаний
    Eigen::MatrixXd sigma_points_pred;
    Eigen::VectorXd x_pred;
    Eigen::MatrixXd P_pred;

    // История для адаптивной оценки
    std::vector<Eigen::VectorXd> innovation_history;
    std::vector<Eigen::VectorXd> residual_history;

    // Параметры адаптации
    int M = 30;                   // Размер окна для оценки
    Eigen::MatrixXd Gamma;        // Матрица управления системным шумом
    double b = 0.95;              // Параметр сглаживания для R

    // Методы
    Eigen::MatrixXd computeSigmaPoints(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance);
    void adaptProcessNoiseCovariance();
    void adaptMeasurementNoiseCovariance();
    Eigen::MatrixXd calculateJacobian(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& func, const Eigen::VectorXd& x);

    // Функции по умолчанию
    Eigen::VectorXd defaultStateTransition(const Eigen::VectorXd& x);
    Eigen::VectorXd defaultMeasurementFunction(const Eigen::VectorXd& x);
};

#endif // ADAPTIVE_UNSCENTED_KALMAN_FILTER_H
