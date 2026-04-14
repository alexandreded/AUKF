#include "../simulation/BeamSimulation.h"
#include <cassert>
#include <cmath>

int main() {
    BeamSimulation simA(0.05, 0.1, 0.01, 0.8, 42u, true);
    BeamSimulation simB(0.05, 0.1, 0.01, 0.8, 42u, true);

    for (int step = 0; step < 200; ++step) {
        Eigen::VectorXd a = simA.moveBeamAndIntegrate(10.0, 1.0);
        Eigen::VectorXd b = simB.moveBeamAndIntegrate(10.0, 1.0);
        assert(a.size() == 4);
        assert(b.size() == 4);
        for (int i = 0; i < 4; ++i) {
            assert(std::isfinite(a(i)));
            assert(a(i) >= 0.0);
            assert(std::abs(a(i) - b(i)) < 1e-12);
        }
    }

    return 0;
}
