# AUKFProject

AUKFProject is a Qt/C++ application for tracking a laser spot position from 4-channel detector intensities using an Adaptive Unscented Kalman Filter (AUKF).  
The project supports simulation, replay of detector measurements, and hardware mode with optional closed-loop calibration feedback to an AD995x-based board.

## Main Features

- Adaptive Unscented Kalman Filter (UKF) for 4D intensity state.
- NIS-based outlier gating for measurement rejection.
- Deterministic beam simulation with configurable noise, gap, and motion speed.
- Real-time replay mode from JSON detector stream.
- Hardware mode through `BoardDriver` abstraction.
- Calibration feedback: software channel-gain adaptation and optional full hardware feedback loop (frequency/amplitude corrections sent to board output).
- Qt GUI with Qwt plots for raw/filtered intensities, coordinates, error, and diagnostics.
- Structured JSON logging with metadata and per-step telemetry.

## Repository Layout

- `main.cpp` - startup and mode selection dialog.
- `Config.h` - runtime configuration and default parameters.
- `core/TrackingEngine.*` - orchestration of filter/simulation/hardware and feedback loops.
- `filter/AdaptiveUnscentedKalmanFilter.*` - AUKF and adaptive covariance logic.
- `simulation/BeamSimulation.*` - synthetic 4-quadrant detector model.
- `hardware/BoardDriver.*` - hardware abstraction and AD995x integration.
- `gui/MainWindow.*` - UI, plotting, validation, and runtime control.
- `io/DataLogger.*` - text + JSON logging.
- `tests/` - smoke tests for simulation, filter, and tracking engine.

## Dependencies

- C++14 compiler
- CMake >= 3.10
- Qt5 (`Widgets`, `Core`, `Gui`)
- Eigen3
- Qwt
- Optional for hardware driver: `libusb-1.0` and AD995x header `ad995x_usb_aod_driver.h` (expected in project root or `../ad9959_AOD_STM32/include`).

Example (Ubuntu-like):

```bash
sudo apt-get install -y \
  build-essential cmake pkg-config \
  qtbase5-dev libeigen3-dev libqwt-qt5-dev libusb-1.0-0-dev
```

## Build

### Default build (driver auto-enabled if dependencies found)

```bash
cmake -S . -B build
cmake --build build -j
```

### Force build without AD995x driver

```bash
cmake -S . -B build-nodriver -DAUKF_ENABLE_AD995X_DRIVER=OFF
cmake --build build-nodriver -j
```

## Run

```bash
./build/AUKFProject
```

At startup, choose one of the modes:

- `Simulation`
- `Realtime data`
- `Hardware`
- `Calibration (feedback)` (hardware mode + feedback enabled)

## Input Data Format (Realtime/Hardware Detector Stream)

`Config.inputDataFile` (default: `data_from_detector.json`) must contain a JSON array:

```json
[
  { "measurements": [1.23, 0.98, 1.11, 0.87] },
  { "measurements": [1.20, 1.01, 1.09, 0.90] }
]
```

Rules:

- Each item is an object.
- Field `measurements` must be an array of exactly 4 numeric values.
- Invalid rows are skipped.

## Calibration and Full Hardware Feedback

Key config parameters (see `Config.h`):

- `enableCalibrationFeedback`
- `calibrationFeedbackRate`
- `calibrationTargetTolerance`
- `calibrationStableWindow`
- `calibrationOnlyAcceptedMeasurements`
- `calibrationDriveHardware`
- `hardwareFeedbackDeadband`
- `hardwareFrequencyFeedbackMHzPerError`
- `hardwareAmplitudeFeedbackPerError`
- `hardwareMinFrequenciesMHz` / `hardwareMaxFrequenciesMHz`
- `hardwareMinAmplitude` / `hardwareMaxAmplitude`

Notes:

- Software calibration adjusts channel gains from normalized detector imbalance.
- Full hardware feedback applies differential correction to output channels.
- Current channel steering assumption in feedback logic: X-axis pair `(0, 2)`, Y-axis pair `(1, 3)`.

## Logging

Default outputs:

- text log: `output.log`
- JSON log: `output_data.json`

JSON contains:

- `metadata` (run configuration)
- `records` (time-series data)
- fields include raw/calibrated/filtered intensities, estimates, NIS/gating status, calibration status, and hardware output telemetry.

## Tests

```bash
ctest --test-dir build --output-on-failure
```

Or with explicit test build:

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Troubleshooting

- `Qwt library not found`: install Qwt dev package and ensure headers/libs are discoverable by CMake.
- Hardware mode fails with AD995x error: verify `libusb-1.0`, check `ad995x_usb_aod_driver.h` include path, and validate USB permissions/device connection.
- `Hardware connected, but detector stream is empty`: provide valid `data_from_detector.json` (or change `Config.inputDataFile`).
