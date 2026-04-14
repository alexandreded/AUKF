#include "../core/TrackingEngine.h"
#include <cassert>
#include <cmath>
#include <vector>

int main() {
    Config cfg;
    cfg.mode = "simulation";
    cfg.noiseLevel = 0.02;
    cfg.enableOutlierGating = true;
    cfg.outlierNisThreshold = 13.2767;
    cfg.deterministicSimulation = true;
    cfg.simulationSeed = 7;
    cfg.enableCalibrationFeedback = true;
    cfg.calibrationFeedbackRate = 0.03;
    cfg.calibrationTargetTolerance = 0.01;

    TrackingEngine engine(cfg);
    std::string error;
    std::vector<Eigen::VectorXd> noRealtimeData;
    bool initOk = engine.initialize(noRealtimeData, error);
    assert(initOk);
    assert(error.empty());

    for (int i = 0; i < 250; ++i) {
        TrackingStepResult result;
        bool ok = engine.step(result, error);
        assert(ok);
        assert(error.empty());
        assert(std::isfinite(result.time));
        assert(!result.rawMeasurement.hasNaN());
        assert(!result.filteredState.hasNaN());
        assert(std::isfinite(result.estimatedX));
        assert(std::isfinite(result.estimatedY));
        assert(std::isfinite(result.nis));
        assert(result.calibrationEnabled);
        for (int k = 0; k < 4; ++k) {
            assert(std::isfinite(result.calibrationGains(k)));
            assert(result.calibrationGains(k) > 0.0);
        }
    }

    return 0;
}
