#include "../filter/AdaptiveUnscentedKalmanFilter.h"
#include <cassert>
#include <cmath>

int main() {
    Eigen::VectorXd initialState(4);
    initialState << 1.0, 1.2, 0.9, 1.1;

    Eigen::MatrixXd initialCov = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4) * 0.01;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4) * 0.05;

    AdaptiveUnscentedKalmanFilter filter(initialState, initialCov, Q, R, 1e-3, 2.0, 0.0);
    filter.setOutlierGate(true, 13.2767);

    for (int i = 0; i < 120; ++i) {
        Eigen::VectorXd measurement(4);
        measurement << 1.0 + 0.02 * std::sin(i * 0.05),
                       1.2 + 0.02 * std::cos(i * 0.03),
                       0.9 + 0.01 * std::sin(i * 0.07),
                       1.1 + 0.01 * std::cos(i * 0.09);

        filter.predict();
        filter.update(measurement);

        Eigen::VectorXd state = filter.getState();
        Eigen::MatrixXd cov = filter.getCovariance();
        assert(!state.hasNaN());
        assert(!cov.hasNaN());
        assert(std::isfinite(filter.getLastNIS()));
    }

    Eigen::VectorXd outlier(4);
    outlier << 1e6, 1e6, 1e6, 1e6;
    filter.predict();
    filter.update(outlier);
    assert(!filter.wasLastMeasurementAccepted());
    assert(std::isfinite(filter.getLastNIS()));

    return 0;
}
