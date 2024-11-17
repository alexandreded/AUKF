#ifndef ADAPTIVE_UNSCENTED_KALMAN_FILTER_H
#define ADAPTIVE_UNSCENTED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <vector>
#include <deque>

class AdaptiveUnscentedKalmanFilter {
public:
    AdaptiveUnscentedKalmanFilter(const Eigen::VectorXd &initial_state,
                                  const Eigen::MatrixXd &initial_covariance,
                                  const Eigen::MatrixXd &process_noise_cov,
                                  const Eigen::MatrixXd &measurement_noise_cov);

    void setParameters(double alpha, double beta, double kappa);
    void reset(const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &initial_covariance);
    void setProcessNoiseCovariance(double processNoise);
    void setMeasurementNoiseCovariance(double measurementNoise);

    void predict();
    void update(const Eigen::VectorXd &measurement);

    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;

    // Функция для расчёта положения пятна
    Eigen::Vector2d calculateSpotPosition();

private:
    // Размеры
    int n; // Размерность состояния
    int measurement_dim; // Размерность измерения
    int sigma_point_count;

    // Параметры UKF
    double alpha, beta, kappa, lambda_;

    // Параметры адаптации
    const int N = 30; // Размер скользящего окна для инноваций

    // Векторы и матрицы состояния
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd process_noise_cov;
    Eigen::MatrixXd measurement_noise_cov;

    // Весовые коэффициенты
    std::vector<double> weights_mean;
    std::vector<double> weights_covariance;

    // Сигма-точки
    std::vector<Eigen::VectorXd> sigma_points;
    std::vector<Eigen::VectorXd> predicted_sigma_points;
    std::vector<Eigen::VectorXd> predicted_measurements;

    // История инноваций
    std::deque<Eigen::VectorXd> innovation_history;

    // Функции UKF
    void computeSigmaPoints();
    Eigen::VectorXd stateTransitionFunction(const Eigen::VectorXd &state);
    Eigen::VectorXd measurementFunction(const Eigen::VectorXd &state);

    // Адаптация ковариаций
    void adaptMeasurementNoiseCovariance();
    Eigen::MatrixXd computeRobustCovariance();

    // Проверка на выбросы
    bool isOutlier(const Eigen::VectorXd &innovation, const Eigen::MatrixXd &S);
    double chiSquareThreshold(int degrees_of_freedom, double confidence_level);

    // Вспомогательные функции
    double h(double x, double y, double P0, double X, double Y, double w);

    // Функция для вычисления медианы столбцов
    Eigen::RowVectorXd computeColumnMedian(const Eigen::MatrixXd& data);
};

#endif // ADAPTIVE_UNSCENTED_KALMAN_FILTER_H
