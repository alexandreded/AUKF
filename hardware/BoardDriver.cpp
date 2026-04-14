#include "BoardDriver.h"
#include <algorithm>
#include <sstream>

namespace {
float clampFloat(float value, float minValue, float maxValue) {
    return std::max(minValue, std::min(maxValue, value));
}
}

#ifdef AUKF_HAS_AD995X_DRIVER
#include "ad995x_usb_aod_driver.h"

class Ad995xBoardDriver final : public BoardDriver {
public:
    bool connect(const BoardConnectionConfig &config, std::string &error) override {
        disconnect();

        int serial = config.serialNumber;
        if (serial < 0) serial = 0;
        if (serial > 255) serial = 255;

        int err = driver.open_device(static_cast<unsigned char>(serial));
        if (err != 0) {
            std::ostringstream oss;
            oss << "Failed to open AD995x device (error " << err << ").";
            error = oss.str();
            return false;
        }
        connected = true;

        currentFrequenciesMHz = config.frequenciesMHz;
        currentAmplitudes = config.amplitudes;
        for (int i = 0; i < 4; ++i) {
            currentAmplitudes[static_cast<std::size_t>(i)] =
                clampFloat(currentAmplitudes[static_cast<std::size_t>(i)], 0.0f, 1.0f);
        }

        if (config.applyOutputOnConnect && !applyOutput(currentFrequenciesMHz, currentAmplitudes, error)) {
            disconnect();
            return false;
        }

        return true;
    }

    bool applyOutput(const std::array<float, 4> &frequenciesMHz,
                     const std::array<float, 4> &amplitudes,
                     std::string &error) override {
        if (!connected || !driver.is_ready()) {
            error = "AD995x device is not connected.";
            return false;
        }

        float freqs[4];
        float amps[4];
        unsigned char ampEnable[4] = {1, 1, 1, 1};
        for (int i = 0; i < 4; ++i) {
            freqs[i] = frequenciesMHz[static_cast<std::size_t>(i)];
            amps[i] = clampFloat(amplitudes[static_cast<std::size_t>(i)], 0.0f, 1.0f);
        }

        int err = driver.set_frequencies(freqs, 4);
        if (err != 0) {
            std::ostringstream oss;
            oss << "AD995x set_frequencies failed (error " << err << ").";
            error = oss.str();
            return false;
        }

        err = driver.set_amplitudes(ampEnable, amps, 4);
        if (err != 0) {
            std::ostringstream oss;
            oss << "AD995x set_amplitudes failed (error " << err << ").";
            error = oss.str();
            return false;
        }

        currentFrequenciesMHz = frequenciesMHz;
        currentAmplitudes = amplitudes;
        for (int i = 0; i < 4; ++i) {
            currentAmplitudes[static_cast<std::size_t>(i)] =
                clampFloat(currentAmplitudes[static_cast<std::size_t>(i)], 0.0f, 1.0f);
        }

        return true;
    }

    void disconnect() override {
        if (connected || driver.is_ready()) {
            driver.close_device();
        }
        connected = false;
    }

    bool isConnected() const override {
        return connected;
    }

    std::array<float, 4> getCurrentFrequenciesMHz() const override {
        return currentFrequenciesMHz;
    }

    std::array<float, 4> getCurrentAmplitudes() const override {
        return currentAmplitudes;
    }

    std::string name() const override {
        return "AD995xUsbDriver";
    }

private:
    AD995xUsbDriver driver;
    bool connected = false;
    std::array<float, 4> currentFrequenciesMHz{{80.0f, 85.0f, 90.0f, 95.0f}};
    std::array<float, 4> currentAmplitudes{{0.5f, 0.5f, 0.5f, 0.5f}};
};

#else

class NullBoardDriver final : public BoardDriver {
public:
    bool connect(const BoardConnectionConfig &config, std::string &error) override {
        currentFrequenciesMHz = config.frequenciesMHz;
        currentAmplitudes = config.amplitudes;
        for (int i = 0; i < 4; ++i) {
            currentAmplitudes[static_cast<std::size_t>(i)] =
                clampFloat(currentAmplitudes[static_cast<std::size_t>(i)], 0.0f, 1.0f);
        }
        error = "AD995x driver is not available in this build.";
        connected = false;
        return false;
    }

    bool applyOutput(const std::array<float, 4> &frequenciesMHz,
                     const std::array<float, 4> &amplitudes,
                     std::string &error) override {
        currentFrequenciesMHz = frequenciesMHz;
        currentAmplitudes = amplitudes;
        for (int i = 0; i < 4; ++i) {
            currentAmplitudes[static_cast<std::size_t>(i)] =
                clampFloat(currentAmplitudes[static_cast<std::size_t>(i)], 0.0f, 1.0f);
        }
        error = "AD995x driver is not available in this build.";
        return false;
    }

    void disconnect() override {
        connected = false;
    }

    bool isConnected() const override {
        return connected;
    }

    std::array<float, 4> getCurrentFrequenciesMHz() const override {
        return currentFrequenciesMHz;
    }

    std::array<float, 4> getCurrentAmplitudes() const override {
        return currentAmplitudes;
    }

    std::string name() const override {
        return "NullBoardDriver";
    }

private:
    bool connected = false;
    std::array<float, 4> currentFrequenciesMHz{{80.0f, 85.0f, 90.0f, 95.0f}};
    std::array<float, 4> currentAmplitudes{{0.5f, 0.5f, 0.5f, 0.5f}};
};

#endif

std::unique_ptr<BoardDriver> createBoardDriver() {
#ifdef AUKF_HAS_AD995X_DRIVER
    return std::unique_ptr<BoardDriver>(new Ad995xBoardDriver());
#else
    return std::unique_ptr<BoardDriver>(new NullBoardDriver());
#endif
}
