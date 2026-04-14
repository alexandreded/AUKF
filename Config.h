#ifndef CONFIG_H
#define CONFIG_H

#include <array>
#include <string>

struct Config {
    double alpha = 1e-3;
    double beta = 2.0;
    double kappa = 0.0;
    double processNoise = 1.0;
    double measurementNoise = 1.0;
    bool enableOutlierGating = true;
    double outlierNisThreshold = 13.2767; // Chi-square 0.99 for 4 DOF
    bool enableCalibrationFeedback = false;
    double calibrationFeedbackRate = 0.02;
    double calibrationTargetTolerance = 0.01;
    int calibrationStableWindow = 25;
    bool calibrationOnlyAcceptedMeasurements = true;
    bool calibrationDriveHardware = true;
    double hardwareFeedbackDeadband = 0.002;
    double hardwareFrequencyFeedbackMHzPerError = 0.2;
    double hardwareAmplitudeFeedbackPerError = 0.08;
    double noiseLevel = 0.1;
    double gapSize = 0.0;
    double timeStep = 0.01;
    double beamSpeed = 1.0;
    double beamPower = 10.0;
    double beamWidth = 1.0;
    double x0 = 2.0; // Параметр масштабирования для обратного преобразования
    bool deterministicSimulation = true;
    unsigned int simulationSeed = 12345;
    int maxPlotPoints = 3000;
    int timerIntervalMs = 10;

    std::string mode = "simulation"; // "simulation", "realtime" или "hardware"
    std::string inputDataFile = "data_from_detector.json"; // Файл для real-time режима
    int hardwareSerialNumber = 0; // 0 = первый доступный
    bool hardwareApplyOutputOnConnect = true;
    std::array<float, 4> hardwareFrequenciesMHz{{80.0f, 85.0f, 90.0f, 95.0f}};
    std::array<float, 4> hardwareAmplitudes{{0.5f, 0.5f, 0.5f, 0.5f}};
    std::array<float, 4> hardwareMinFrequenciesMHz{{70.0f, 70.0f, 70.0f, 70.0f}};
    std::array<float, 4> hardwareMaxFrequenciesMHz{{110.0f, 110.0f, 110.0f, 110.0f}};
    float hardwareMinAmplitude = 0.05f;
    float hardwareMaxAmplitude = 1.0f;

    // Логирование
    std::string logFileName = "output.log";
    std::string jsonOutputFile = "output_data.json";
};

#endif // CONFIG_H
