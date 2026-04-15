#include "TrackingEngine.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>

namespace {
float clampFloat(float value, float minValue, float maxValue) {
    return std::max(minValue, std::min(maxValue, value));
}

double clampDouble(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(maxValue, value));
}

Eigen::Vector4d arrayToVector4d(const std::array<float, 4> &input) {
    Eigen::Vector4d output;
    for (int i = 0; i < 4; ++i) {
        output(i) = static_cast<double>(input[static_cast<std::size_t>(i)]);
    }
    return output;
}
}

TrackingEngine::TrackingEngine(const Config &cfg)
    : config(cfg) {}

bool TrackingEngine::initialize(const std::vector<Eigen::VectorXd> &realtimeMeasurements, std::string &error) {
    reset();
    loadedMeasurements = realtimeMeasurements;
    hasGroundTruth = (config.mode == "simulation");

    Eigen::VectorXd initialMeasurement(4);
    if (config.mode == "simulation") {
        beamSimulation = std::make_unique<BeamSimulation>(
            config.noiseLevel,
            config.gapSize,
            config.timeStep,
            config.beamSpeed,
            config.simulationSeed,
            config.deterministicSimulation);
        initialMeasurement = beamSimulation->moveBeamAndIntegrate(config.beamPower, config.beamWidth);
    } else if (config.mode == "hardware") {
        boardDriver = createBoardDriver();
        BoardConnectionConfig hwCfg;
        hwCfg.serialNumber = config.hardwareSerialNumber;
        hwCfg.applyOutputOnConnect = config.hardwareApplyOutputOnConnect;
        hwCfg.frequenciesMHz = config.hardwareFrequenciesMHz;
        hwCfg.amplitudes = config.hardwareAmplitudes;

        std::string hwError;
        if (!boardDriver->connect(hwCfg, hwError)) {
            error = "Hardware connection failed: " + hwError;
            return false;
        }

        const float minAmp = std::min(config.hardwareMinAmplitude, config.hardwareMaxAmplitude);
        const float maxAmp = std::max(config.hardwareMinAmplitude, config.hardwareMaxAmplitude);
        for (int i = 0; i < 4; ++i) {
            const std::size_t idx = static_cast<std::size_t>(i);
            const float minFreq = std::min(config.hardwareMinFrequenciesMHz[idx], config.hardwareMaxFrequenciesMHz[idx]);
            const float maxFreq = std::max(config.hardwareMinFrequenciesMHz[idx], config.hardwareMaxFrequenciesMHz[idx]);
            hardwareFrequenciesMHz[idx] = clampFloat(config.hardwareFrequenciesMHz[idx], minFreq, maxFreq);
            hardwareAmplitudes[idx] = clampFloat(config.hardwareAmplitudes[idx], minAmp, maxAmp);
        }

        if (!boardDriver->applyOutput(hardwareFrequenciesMHz, hardwareAmplitudes, hwError)) {
            error = "Hardware initial output failed: " + hwError;
            boardDriver->disconnect();
            boardDriver.reset();
            return false;
        }

        std::array<double, 4> firstMeasurement{};
        if (!boardDriver->readMeasurement(firstMeasurement, hwError)) {
            error = "Hardware measurement read failed: " + hwError;
            boardDriver->disconnect();
            boardDriver.reset();
            return false;
        }
        initialMeasurement.resize(4);
        for (int i = 0; i < 4; ++i) {
            initialMeasurement(i) = firstMeasurement[static_cast<std::size_t>(i)];
        }
    } else {
        if (loadedMeasurements.empty()) {
            error = "Нет данных для real-time режима.";
            return false;
        }
        currentMeasurementIndex = 0;
        initialMeasurement = loadedMeasurements[currentMeasurementIndex++];
    }

    try {
        Eigen::VectorXd initialState = initialMeasurement;
        Eigen::MatrixXd initialCov = Eigen::MatrixXd::Identity(4, 4);
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4) * config.processNoise;
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4) * config.measurementNoise;

        kalmanFilter = std::make_unique<AdaptiveUnscentedKalmanFilter>(
            initialState, initialCov, Q, R, config.alpha, config.beta, config.kappa);
        kalmanFilter->setOutlierGate(config.enableOutlierGating, config.outlierNisThreshold);
    } catch (const std::exception &e) {
        error = e.what();
        if (boardDriver) {
            boardDriver->disconnect();
            boardDriver.reset();
        }
        return false;
    }

    initialized = true;
    return true;
}

bool TrackingEngine::nextMeasurement(Eigen::VectorXd &measurement, double &trueX, double &trueY, std::string &error) {
    if (config.mode == "simulation") {
        measurement = beamSimulation->moveBeamAndIntegrate(config.beamPower, config.beamWidth);
        trueX = beamSimulation->getXc();
        trueY = beamSimulation->getYc();
        return true;
    }

    if (config.mode == "hardware") {
        if (!boardDriver || !boardDriver->isConnected()) {
            finished = true;
            error = "Hardware board is not connected.";
            return false;
        }
        std::array<double, 4> detectorMeasurement{};
        std::string readError;
        if (!boardDriver->readMeasurement(detectorMeasurement, readError)) {
            error = "Failed to read detector measurement from board: " + readError;
            return false;
        }
        measurement.resize(4);
        for (int i = 0; i < 4; ++i) {
            measurement(i) = detectorMeasurement[static_cast<std::size_t>(i)];
        }
        trueX = std::numeric_limits<double>::quiet_NaN();
        trueY = std::numeric_limits<double>::quiet_NaN();
        return true;
    }

    if (currentMeasurementIndex >= loadedMeasurements.size()) {
        finished = true;
        return false;
    }
    measurement = loadedMeasurements[currentMeasurementIndex++];
    trueX = std::numeric_limits<double>::quiet_NaN();
    trueY = std::numeric_limits<double>::quiet_NaN();
    return true;
}

bool TrackingEngine::step(TrackingStepResult &result, std::string &error) {
    if (!initialized) {
        error = "Engine is not initialized.";
        return false;
    }
    if (finished) {
        return false;
    }

    Eigen::VectorXd measurement(4);
    double trueX = 0.0;
    double trueY = 0.0;
    if (!nextMeasurement(measurement, trueX, trueY, error)) {
        return false;
    }
    if (measurement.hasNaN()) {
        error = "Measurement contains NaN.";
        return false;
    }

    Eigen::VectorXd calibratedMeasurement = applyCalibration(measurement);
    if (calibratedMeasurement.hasNaN()) {
        error = "Calibrated measurement contains NaN.";
        return false;
    }

    kalmanFilter->predict();
    kalmanFilter->update(calibratedMeasurement);
    Eigen::VectorXd currentState = kalmanFilter->getState();
    if (currentState.hasNaN()) {
        error = "Kalman Filter state contains NaN.";
        return false;
    }

    updateCalibrationFeedback(currentState, kalmanFilter->wasLastMeasurementAccepted());
    if (!updateHardwareFeedback(kalmanFilter->wasLastMeasurementAccepted(), error)) {
        return false;
    }

    currentTime += config.timeStep;
    Eigen::Vector2d spotPosition = kalmanFilter->calculateSpotPosition(config.beamWidth, config.x0);
    const double estimatedX = spotPosition(0);
    const double estimatedY = spotPosition(1);

    result.time = currentTime;
    result.rawMeasurement = measurement;
    result.calibratedMeasurement = calibratedMeasurement;
    result.filteredState = currentState;
    result.estimatedX = estimatedX;
    result.estimatedY = estimatedY;
    result.hasGroundTruth = hasGroundTruth;
    result.trueX = trueX;
    result.trueY = trueY;
    result.nis = kalmanFilter->getLastNIS();
    result.measurementAccepted = kalmanFilter->wasLastMeasurementAccepted();
    result.calibrationEnabled = config.enableCalibrationFeedback;
    result.calibrationConverged = calibrationConverged;
    result.calibrationErrorX = lastCalibrationErrorX;
    result.calibrationErrorY = lastCalibrationErrorY;
    result.calibrationGains = calibrationGains.matrix();
    result.hardwareFeedbackEnabled =
        (config.mode == "hardware" && config.enableCalibrationFeedback && config.calibrationDriveHardware);
    result.hardwareFeedbackApplied = lastHardwareFeedbackApplied;
    result.hardwareFrequenciesMHz = arrayToVector4d(hardwareFrequenciesMHz);
    result.hardwareAmplitudes = arrayToVector4d(hardwareAmplitudes);

    bool validCoords = std::isfinite(estimatedX) && std::isfinite(estimatedY) &&
                       std::isfinite(trueX) && std::isfinite(trueY);
    if (hasGroundTruth && validCoords) {
        result.error = std::sqrt(std::pow(estimatedX - trueX, 2.0) + std::pow(estimatedY - trueY, 2.0));
        result.errorValid = true;
    } else {
        result.error = std::numeric_limits<double>::quiet_NaN();
        result.errorValid = false;
    }

    return true;
}

bool TrackingEngine::isFinished() const {
    return finished;
}

Eigen::VectorXd TrackingEngine::applyCalibration(const Eigen::VectorXd &measurement) const {
    if (measurement.size() != 4 || !config.enableCalibrationFeedback) {
        return measurement;
    }

    Eigen::VectorXd calibrated = measurement;
    for (int i = 0; i < 4; ++i) {
        calibrated(i) = std::max(0.0, measurement(i) * calibrationGains[i]);
    }
    return calibrated;
}

void TrackingEngine::updateCalibrationFeedback(const Eigen::VectorXd &measurement, bool measurementAccepted) {
    if (!config.enableCalibrationFeedback || measurement.size() != 4) {
        lastCalibrationErrorX = 0.0;
        lastCalibrationErrorY = 0.0;
        return;
    }
    if (config.calibrationOnlyAcceptedMeasurements && !measurementAccepted) {
        return;
    }

    const double I_A = measurement(0);
    const double I_B = measurement(1);
    const double I_C = measurement(2);
    const double I_D = measurement(3);
    const double sumI = I_A + I_B + I_C + I_D;
    if (!std::isfinite(sumI) || sumI <= 1e-12) {
        return;
    }

    const double ex = ((I_A + I_D) - (I_B + I_C)) / sumI;
    const double ey = ((I_A + I_B) - (I_C + I_D)) / sumI;
    lastCalibrationErrorX = ex;
    lastCalibrationErrorY = ey;

    const double tolerance = std::max(1e-6, config.calibrationTargetTolerance);
    if (std::abs(ex) < tolerance && std::abs(ey) < tolerance) {
        calibrationStableCount++;
    } else {
        calibrationStableCount = 0;
    }
    if (calibrationStableCount >= std::max(1, config.calibrationStableWindow)) {
        calibrationConverged = true;
    }

    const double lr = std::max(0.0, config.calibrationFeedbackRate);
    if (lr <= 0.0 || calibrationConverged) {
        return;
    }

    Eigen::Array4d delta = Eigen::Array4d::Zero();
    delta[0] = -(ex + ey);
    delta[1] = ex - ey;
    delta[2] = ex + ey;
    delta[3] = -ex + ey;

    calibrationGains += lr * delta;
    for (int i = 0; i < 4; ++i) {
        calibrationGains[i] = std::max(0.5, std::min(1.5, calibrationGains[i]));
    }

    const double meanGain = calibrationGains.mean();
    if (std::isfinite(meanGain) && meanGain > 1e-9) {
        calibrationGains /= meanGain;
    }
}

bool TrackingEngine::updateHardwareFeedback(bool measurementAccepted, std::string &error) {
    lastHardwareFeedbackApplied = false;
    if (config.mode != "hardware" || !config.enableCalibrationFeedback || !config.calibrationDriveHardware) {
        return true;
    }
    if (!boardDriver || !boardDriver->isConnected()) {
        error = "Hardware feedback requested, but board is not connected.";
        return false;
    }
    if (config.calibrationOnlyAcceptedMeasurements && !measurementAccepted) {
        return true;
    }
    if (!std::isfinite(lastCalibrationErrorX) || !std::isfinite(lastCalibrationErrorY)) {
        return true;
    }

    const double deadband = std::max(0.0, config.hardwareFeedbackDeadband);
    if (std::abs(lastCalibrationErrorX) <= deadband && std::abs(lastCalibrationErrorY) <= deadband) {
        return true;
    }

    const double lr = std::max(0.0, config.calibrationFeedbackRate);
    const double freqFactor = std::max(0.0, config.hardwareFrequencyFeedbackMHzPerError);
    const double ampFactor = std::max(0.0, config.hardwareAmplitudeFeedbackPerError);
    const double freqStep = lr * freqFactor;
    const double ampStep = lr * ampFactor;
    if (freqStep <= 0.0 && ampStep <= 0.0) {
        return true;
    }

    std::array<float, 4> nextFrequencies = hardwareFrequenciesMHz;
    std::array<float, 4> nextAmplitudes = hardwareAmplitudes;

    if (freqStep > 0.0) {
        // Differential steering for X(0,2) and Y(1,3) channel pairs.
        nextFrequencies[0] = static_cast<float>(nextFrequencies[0] - freqStep * lastCalibrationErrorX);
        nextFrequencies[2] = static_cast<float>(nextFrequencies[2] + freqStep * lastCalibrationErrorX);
        nextFrequencies[1] = static_cast<float>(nextFrequencies[1] - freqStep * lastCalibrationErrorY);
        nextFrequencies[3] = static_cast<float>(nextFrequencies[3] + freqStep * lastCalibrationErrorY);
    }

    if (ampStep > 0.0) {
        Eigen::Array4d delta = Eigen::Array4d::Zero();
        delta[0] = -(lastCalibrationErrorX + lastCalibrationErrorY);
        delta[1] =  lastCalibrationErrorX - lastCalibrationErrorY;
        delta[2] =  lastCalibrationErrorX + lastCalibrationErrorY;
        delta[3] = -lastCalibrationErrorX + lastCalibrationErrorY;

        for (int i = 0; i < 4; ++i) {
            const std::size_t idx = static_cast<std::size_t>(i);
            const double gain = clampDouble(calibrationGains[i], 0.5, 1.5);
            const double baseAmplitude = static_cast<double>(config.hardwareAmplitudes[idx]) * gain;
            nextAmplitudes[idx] = static_cast<float>(baseAmplitude + ampStep * delta[i]);
        }

        double targetMean = 0.0;
        double currentMean = 0.0;
        for (int i = 0; i < 4; ++i) {
            targetMean += static_cast<double>(config.hardwareAmplitudes[static_cast<std::size_t>(i)]);
            currentMean += static_cast<double>(nextAmplitudes[static_cast<std::size_t>(i)]);
        }
        targetMean /= 4.0;
        currentMean /= 4.0;
        if (std::isfinite(targetMean) && std::isfinite(currentMean) && std::abs(currentMean) > 1e-9) {
            const double scale = targetMean / currentMean;
            for (int i = 0; i < 4; ++i) {
                const std::size_t idx = static_cast<std::size_t>(i);
                nextAmplitudes[idx] = static_cast<float>(nextAmplitudes[idx] * scale);
            }
        }
    }

    const float minAmp = std::min(config.hardwareMinAmplitude, config.hardwareMaxAmplitude);
    const float maxAmp = std::max(config.hardwareMinAmplitude, config.hardwareMaxAmplitude);
    for (int i = 0; i < 4; ++i) {
        const std::size_t idx = static_cast<std::size_t>(i);
        const float minFreq = std::min(config.hardwareMinFrequenciesMHz[idx], config.hardwareMaxFrequenciesMHz[idx]);
        const float maxFreq = std::max(config.hardwareMinFrequenciesMHz[idx], config.hardwareMaxFrequenciesMHz[idx]);
        nextFrequencies[idx] = clampFloat(nextFrequencies[idx], minFreq, maxFreq);
        nextAmplitudes[idx] = clampFloat(nextAmplitudes[idx], minAmp, maxAmp);
    }

    std::string hwError;
    if (!boardDriver->applyOutput(nextFrequencies, nextAmplitudes, hwError)) {
        error = "Hardware feedback apply failed: " + hwError;
        return false;
    }

    hardwareFrequenciesMHz = nextFrequencies;
    hardwareAmplitudes = nextAmplitudes;
    lastHardwareFeedbackApplied = true;
    return true;
}

void TrackingEngine::reset() {
    kalmanFilter.reset();
    beamSimulation.reset();
    if (boardDriver) {
        boardDriver->disconnect();
    }
    boardDriver.reset();
    loadedMeasurements.clear();
    currentMeasurementIndex = 0;
    currentTime = 0.0;
    initialized = false;
    finished = false;
    hasGroundTruth = false;
    calibrationGains = Eigen::Array4d::Ones();
    lastCalibrationErrorX = 0.0;
    lastCalibrationErrorY = 0.0;
    calibrationStableCount = 0;
    calibrationConverged = false;
    hardwareFrequenciesMHz = config.hardwareFrequenciesMHz;
    hardwareAmplitudes = config.hardwareAmplitudes;
    lastHardwareFeedbackApplied = false;
}
