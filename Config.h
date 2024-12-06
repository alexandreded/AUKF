#ifndef CONFIG_H
#define CONFIG_H

#include <string>

struct Config {
    double alpha = 1e-3;
    double beta = 2.0;
    double kappa = 0.0;
    double processNoise = 1.0;
    double measurementNoise = 1.0;
    double noiseLevel = 0.1;
    double gapSize = 0.0;
    double timeStep = 0.01;
    double beamSpeed = 1.0;
    double beamPower = 10.0;
    double beamWidth = 1.0;
    double x0 = 2.0; // Параметр масштабирования для обратного преобразования

    std::string mode = "simulation"; // "simulation" или "realtime"
    std::string inputDataFile = "data_from_detector.json"; // Файл для real-time режима

    // Логирование
    std::string logFileName = "output.log";
    std::string jsonOutputFile = "output_data.json";
};

#endif // CONFIG_H
