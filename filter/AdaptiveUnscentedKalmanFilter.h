#ifndef ADAPTIVE_UNSCENTED_KALMAN_FILTER_H
#define ADAPTIVE_UNSCENTED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <deque>
#include <vector>

class AdaptiveUnscentedKalmanFilter {
public:
    AdaptiveUnscentedKalmanFilter(const Eigen::VectorXd &initial_state,
                                  const Eigen::MatrixXd &initial_covariance,
                                  const Eigen::MatrixXd &process_noise_cov,
                                  const Eigen::MatrixXd &measurement_noise_cov,
                                  double alpha, double beta, double kappa,
                                  int adapt_window = 30);

    void predict();
    void update(const Eigen::VectorXd &measurement);

    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;

    void setProcessNoiseCovariance(const Eigen::MatrixXd &Q);
    void setMeasurementNoiseCovariance(const Eigen::MatrixXd &R);

    // Расчёт позиции пятна на основе интенсивностей
    Eigen::Vector2d calculateSpotPosition(double w, double x0) const;

private:
    // Вычисление сигма-точек с регуляризацией
    void computeSigmaPoints(std::vector<Eigen::VectorXd> &sigma_points);
    // Функции перехода и измерения (здесь – тождественные, так как явная динамика не задана)
    Eigen::VectorXd stateTransitionFunction(const Eigen::VectorXd &state);
    Eigen::VectorXd measurementFunction(const Eigen::VectorXd &state);
    
    // Адаптивное обновление ковариаций процесса и измерения
    void adaptProcessNoiseCovariance();
    void adaptMeasurementNoiseCovariance();

    // Вспомогательные функции для расчёта координат пятна
    double g(double Ex) const;
    double erfinv(double x) const;

private:
    int n; // размерность состояния
    int m; // размерность измерения
    int sigma_point_count;

    double alpha, beta, kappa, lambda_;

    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd process_noise_cov;
    Eigen::MatrixXd measurement_noise_cov;

    std::vector<double> weights_mean;
    std::vector<double> weights_covariance;

    std::deque<Eigen::VectorXd> innovation_history; 
    std::deque<Eigen::VectorXd> residual_history;   
    std::deque<Eigen::VectorXd> measurement_history; 

    int adapt_window;
};

#endif // ADAPTIVE_UNSCENTED_KALMAN_FILTER_H
