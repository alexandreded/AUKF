#ifndef ADAPTIVE_UNSCENTED_KALMAN_FILTER_H
#define ADAPTIVE_UNSCENTED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <vector>

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

    static double h(double x, double y, double P0, double X, double Y, double w);
    static double g(double Ex);
    static double lambdaFunc(double x0);
    
    // для обратной функции ошибки
    static double erfinv(double x);

   
    Eigen::Vector2d calculateSpotPosition(double I_A, double I_B, double I_C, double I_D, double w, double x0);

private:
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd process_noise_cov;
    Eigen::MatrixXd measurement_noise_cov;
    int n;
    int sigma_point_count;
    double alpha, beta, kappa, lambda_;
    int adapt_window;
    
    std::vector<double> weights_mean;
    std::vector<double> weights_covariance;
    
    std::vector<Eigen::VectorXd> innovation_history;
    std::vector<Eigen::VectorXd> residual_history;
    
    std::vector<Eigen::VectorXd> computeSigmaPoints();

    // Удалить choleskyDecomposition  или заменить ее разложеним Холецкого из Eigen
};

#endif // ADAPTIVE_UNSCENTED_KALMAN_FILTER_H
