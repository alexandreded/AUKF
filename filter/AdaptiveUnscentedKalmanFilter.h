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

    // Расчет позиции пятна на основе состояний (интенсивностей)
    Eigen::Vector2d calculateSpotPosition(double w, double x0) const;

private:
    void computeSigmaPoints(std::vector<Eigen::VectorXd> &sigma_points);
    Eigen::VectorXd stateTransitionFunction(const Eigen::VectorXd &state);
    Eigen::VectorXd measurementFunction(const Eigen::VectorXd &state);
    
    void adaptProcessNoiseCovariance();
    void adaptMeasurementNoiseCovariance();

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
