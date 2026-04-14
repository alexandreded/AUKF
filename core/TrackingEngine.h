#ifndef TRACKING_ENGINE_H
#define TRACKING_ENGINE_H

#include "../Config.h"
#include "../filter/AdaptiveUnscentedKalmanFilter.h"
#include "../hardware/BoardDriver.h"
#include "../simulation/BeamSimulation.h"
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <string>
#include <vector>

struct TrackingStepResult {
    double time = 0.0;
    Eigen::VectorXd rawMeasurement;
    Eigen::VectorXd calibratedMeasurement;
    Eigen::VectorXd filteredState;
    double estimatedX = 0.0;
    double estimatedY = 0.0;
    bool hasGroundTruth = false;
    double trueX = 0.0;
    double trueY = 0.0;
    bool errorValid = false;
    double error = 0.0;
    double nis = 0.0;
    bool measurementAccepted = true;
    bool calibrationEnabled = false;
    bool calibrationConverged = false;
    double calibrationErrorX = 0.0;
    double calibrationErrorY = 0.0;
    Eigen::Vector4d calibrationGains = Eigen::Vector4d::Ones();
    bool hardwareFeedbackEnabled = false;
    bool hardwareFeedbackApplied = false;
    Eigen::Vector4d hardwareFrequenciesMHz = Eigen::Vector4d::Zero();
    Eigen::Vector4d hardwareAmplitudes = Eigen::Vector4d::Zero();
};

class TrackingEngine {
public:
    explicit TrackingEngine(const Config &config);

    bool initialize(const std::vector<Eigen::VectorXd> &realtimeMeasurements, std::string &error);
    bool step(TrackingStepResult &result, std::string &error);

    bool isFinished() const;
    void reset();

private:
    bool nextMeasurement(Eigen::VectorXd &measurement, double &trueX, double &trueY);
    Eigen::VectorXd applyCalibration(const Eigen::VectorXd &measurement) const;
    void updateCalibrationFeedback(const Eigen::VectorXd &measurement, bool measurementAccepted);
    bool updateHardwareFeedback(bool measurementAccepted, std::string &error);

private:
    Config config;
    std::unique_ptr<AdaptiveUnscentedKalmanFilter> kalmanFilter;
    std::unique_ptr<BeamSimulation> beamSimulation;
    std::unique_ptr<BoardDriver> boardDriver;

    std::vector<Eigen::VectorXd> loadedMeasurements;
    std::size_t currentMeasurementIndex = 0;

    double currentTime = 0.0;
    bool initialized = false;
    bool finished = false;
    bool hasGroundTruth = false;
    Eigen::Array4d calibrationGains = Eigen::Array4d::Ones();
    double lastCalibrationErrorX = 0.0;
    double lastCalibrationErrorY = 0.0;
    int calibrationStableCount = 0;
    bool calibrationConverged = false;
    std::array<float, 4> hardwareFrequenciesMHz{{0.0f, 0.0f, 0.0f, 0.0f}};
    std::array<float, 4> hardwareAmplitudes{{0.0f, 0.0f, 0.0f, 0.0f}};
    bool lastHardwareFeedbackApplied = false;
};

#endif // TRACKING_ENGINE_H
