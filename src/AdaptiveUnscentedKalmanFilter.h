#ifndef ADAPTIVE_UNSCENTED_KALMAN_FILTER_H
#define ADAPTIVE_UNSCENTED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <vector>
#include <deque>

class AdaptiveUnscentedKalmanFilter {
public:
    // Конструктор
    AdaptiveUnscentedKalmanFilter(const Eigen::VectorXd &initial_state,
                                  const Eigen::MatrixXd &initial_covariance,
                                  const Eigen::MatrixXd &process_noise_cov,
                                  const Eigen::MatrixXd &measurement_noise_cov);

    // Настройка параметров фильтра
    void setParameters(double alpha, double beta, double kappa);

    // Сброс фильтра
    void reset(const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &initial_covariance);

    // Установка ковариации шума процесса
    void setProcessNoiseCovariance(const Eigen::MatrixXd &process_noise_cov);

    // Установка ковариации шума измерения
    void setMeasurementNoiseCovariance(const Eigen::MatrixXd &measurement_noise_cov);

    // Прогнозирование
    void predict();

    // Обновление с новым измерением
    void update(const Eigen::VectorXd &measurement);

    // Получение текущего состояния
    Eigen::VectorXd getState() const;

    // Получение текущей ковариации
    Eigen::MatrixXd getCovariance() const;

    // Функция для расчёта положения пятна на основе отфильтрованных интенсивностей
    Eigen::Vector2d calculateSpotPosition() const;

private:
    // Размеры
    int n; // Размерность состояния (4 интенсивности)
    int m; // Размерность измерения (4 интенсивности)
    int sigma_point_count;

    // Параметры UKF
    double alpha, beta, kappa, lambda_;

    // Векторы и матрицы состояния
    Eigen::VectorXd state;       // \hat{X}_k
    Eigen::MatrixXd covariance;  // P_k
    Eigen::MatrixXd process_noise_cov;      // Q_{k-1}
    Eigen::MatrixXd measurement_noise_cov;  // R_k

    // Весовые коэффициенты
    std::vector<double> weights_mean;       // \omega^{(m)}_i
    std::vector<double> weights_covariance; // \omega^{(c)}_i

    // История инноваций и остатков для адаптации
    std::deque<Eigen::VectorXd> innovation_history; // \epsilon_k
    std::deque<Eigen::VectorXd> residual_history;   // \eta_k
    const int adapt_window = 30; // Размер окна для адаптации

    // История измерений
    std::deque<Eigen::VectorXd> measurement_history; // Z_k

    // Функции UKF
    void computeSigmaPoints(std::vector<Eigen::VectorXd> &sigma_points);

    // Функции перехода и измерения
    Eigen::VectorXd stateTransitionFunction(const Eigen::VectorXd &state);
    Eigen::VectorXd measurementFunction(const Eigen::VectorXd &state);

    // Адаптация ковариаций
    void adaptProcessNoiseCovariance();
    void adaptMeasurementNoiseCovariance();

    // Функции для работы с интенсивностями
    double g(double Ex) const;
    double erfinv(double x) const;
};

#endif // ADAPTIVE_UNSCENTED_KALMAN_FILTER_H
