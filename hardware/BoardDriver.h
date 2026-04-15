#ifndef BOARD_DRIVER_H
#define BOARD_DRIVER_H

#include <array>
#include <memory>
#include <string>

struct BoardConnectionConfig {
    int serialNumber = 0; // 0 means "first available"
    bool applyOutputOnConnect = true;
    std::array<float, 4> frequenciesMHz{{80.0f, 85.0f, 90.0f, 95.0f}};
    std::array<float, 4> amplitudes{{0.5f, 0.5f, 0.5f, 0.5f}};
};

class BoardDriver {
public:
    virtual ~BoardDriver() = default;

    virtual bool connect(const BoardConnectionConfig &config, std::string &error) = 0;
    virtual bool readMeasurement(std::array<double, 4> &measurement, std::string &error) = 0;
    virtual bool applyOutput(const std::array<float, 4> &frequenciesMHz,
                             const std::array<float, 4> &amplitudes,
                             std::string &error) = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;
    virtual std::array<float, 4> getCurrentFrequenciesMHz() const = 0;
    virtual std::array<float, 4> getCurrentAmplitudes() const = 0;
    virtual std::string name() const = 0;
};

std::unique_ptr<BoardDriver> createBoardDriver();

#endif // BOARD_DRIVER_H
