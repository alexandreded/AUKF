#include "MainWindow.h"
#include <QDateTime>
#include <QDoubleValidator>
#include <QFile>
#include <QGridLayout>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QIntValidator>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QMutexLocker>
#include <QPushButton>
#include <QScrollArea>
#include <QScreen>
#include <QSplitter>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <qwt_legend.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_zoomer.h>

HardwareInputThread::HardwareInputThread(TrackingEngine *trackingEngine, QObject *parent)
    : QThread(parent), engine(trackingEngine) {}

HardwareInputThread::~HardwareInputThread() {
    requestStop();
    wait(2000);
}

void HardwareInputThread::requestStop() {
    QMutexLocker locker(&mutex);
    stopRequested = true;
}

bool HardwareInputThread::popResult(TrackingStepResult &result) {
    QMutexLocker locker(&mutex);
    if (resultQueue.empty()) {
        return false;
    }
    result = resultQueue.front();
    resultQueue.pop_front();
    return true;
}

bool HardwareInputThread::takeError(QString &errorMessage) {
    QMutexLocker locker(&mutex);
    if (fatalError.isEmpty()) {
        return false;
    }
    errorMessage = fatalError;
    fatalError.clear();
    return true;
}

bool HardwareInputThread::isWorkerFinished() const {
    QMutexLocker locker(&mutex);
    return finished;
}

void HardwareInputThread::run() {
    if (!engine) {
        QMutexLocker locker(&mutex);
        fatalError = "Hardware input thread has no tracking engine.";
        finished = true;
        return;
    }

    while (true) {
        {
            QMutexLocker locker(&mutex);
            if (stopRequested) {
                finished = true;
                return;
            }
        }

        TrackingStepResult stepResult;
        std::string error;
        const bool ok = engine->step(stepResult, error);

        QMutexLocker locker(&mutex);
        if (!ok) {
            if (!error.empty()) {
                fatalError = QString::fromStdString(error);
            }
            finished = true;
            return;
        }

        resultQueue.push_back(stepResult);
        if (resultQueue.size() > 1024) {
            resultQueue.pop_front();
        }
    }
}

MainWindow::MainWindow(const Config &cfg, QWidget *parent)
    : QMainWindow(parent), config(cfg), iteration(0), isRunning(false)
{
    setupUI();
    setupPlots();

    dataLogger = std::make_unique<DataLogger>(
                QString::fromStdString(config.logFileName),
                QString::fromStdString(config.jsonOutputFile));

    QJsonObject metadata;
    metadata["app"] = "AUKFProject";
    metadata["created_at_utc"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
    metadata["mode"] = QString::fromStdString(config.mode);
    metadata["alpha"] = config.alpha;
    metadata["beta"] = config.beta;
    metadata["kappa"] = config.kappa;
    metadata["process_noise"] = config.processNoise;
    metadata["measurement_noise"] = config.measurementNoise;
    metadata["enable_outlier_gating"] = config.enableOutlierGating;
    metadata["outlier_nis_threshold"] = config.outlierNisThreshold;
    metadata["enable_calibration_feedback"] = config.enableCalibrationFeedback;
    metadata["calibration_feedback_rate"] = config.calibrationFeedbackRate;
    metadata["calibration_target_tolerance"] = config.calibrationTargetTolerance;
    metadata["calibration_stable_window"] = config.calibrationStableWindow;
    metadata["calibration_only_accepted_measurements"] = config.calibrationOnlyAcceptedMeasurements;
    metadata["calibration_drive_hardware"] = config.calibrationDriveHardware;
    metadata["hardware_feedback_deadband"] = config.hardwareFeedbackDeadband;
    metadata["hardware_frequency_feedback_mhz_per_error"] = config.hardwareFrequencyFeedbackMHzPerError;
    metadata["hardware_amplitude_feedback_per_error"] = config.hardwareAmplitudeFeedbackPerError;
    metadata["deterministic_simulation"] = config.deterministicSimulation;
    metadata["simulation_seed"] = static_cast<int>(config.simulationSeed);
    metadata["max_plot_points"] = config.maxPlotPoints;
    metadata["timer_interval_ms"] = config.timerIntervalMs;
    metadata["hardware_serial_number"] = config.hardwareSerialNumber;
    metadata["hardware_apply_output_on_connect"] = config.hardwareApplyOutputOnConnect;
    QJsonArray hwFreq;
    QJsonArray hwAmp;
    for (int i = 0; i < 4; ++i) {
        hwFreq.append(config.hardwareFrequenciesMHz[static_cast<std::size_t>(i)]);
        hwAmp.append(config.hardwareAmplitudes[static_cast<std::size_t>(i)]);
    }
    metadata["hardware_frequencies_mhz"] = hwFreq;
    metadata["hardware_amplitudes"] = hwAmp;
    dataLogger->setMetadata(metadata);

    dataLogger->logMessage("Application started");

    if (config.mode == "realtime") {
        loadRealData();
    }

    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::updatePlots);
    updateHardwareDeviceStatus();
}

MainWindow::~MainWindow() {
    if (hardwareInputThread) {
        hardwareInputThread->requestStop();
        hardwareInputThread->wait(2000);
        hardwareInputThread.reset();
    }
    if (trackingEngine) {
        trackingEngine->reset();
        trackingEngine.reset();
    }
    if (manualBoardDriver) {
        manualBoardDriver->disconnect();
        manualBoardDriver.reset();
    }
    // При выходе ~DataLogger сохранит JSON
}

void MainWindow::setupUI() {
    QWidget *centralWidget = new QWidget(this);
    QHBoxLayout *rootLayout = new QHBoxLayout(centralWidget);
    rootLayout->setContentsMargins(10, 10, 10, 10);
    rootLayout->setSpacing(10);

    setStyleSheet(
        "QGroupBox { font-weight: 600; margin-top: 8px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }"
        "QLineEdit { min-height: 24px; padding: 2px 6px; }"
        "QPushButton { min-height: 28px; padding: 4px 10px; }");

    QGroupBox *filterGroupBox = new QGroupBox("Параметры фильтра");
    QGridLayout *filterLayout = new QGridLayout();

    QDoubleValidator *validator = new QDoubleValidator(this);
    validator->setNotation(QDoubleValidator::StandardNotation);
    validator->setLocale(QLocale::C);

    alphaEdit = new QLineEdit(QString::number(config.alpha));
    betaEdit = new QLineEdit(QString::number(config.beta));
    kappaEdit = new QLineEdit(QString::number(config.kappa));
    processNoiseEdit = new QLineEdit(QString::number(config.processNoise));
    measurementNoiseEdit = new QLineEdit(QString::number(config.measurementNoise));
    outlierGatingCheck = new QCheckBox("Включить gating по NIS");
    outlierGatingCheck->setChecked(config.enableOutlierGating);
    outlierNisThresholdEdit = new QLineEdit(QString::number(config.outlierNisThreshold));
    calibrationFeedbackCheck = new QCheckBox("Режим калибровки (feedback)");
    calibrationFeedbackCheck->setChecked(config.enableCalibrationFeedback);
    calibrationRateEdit = new QLineEdit(QString::number(config.calibrationFeedbackRate));
    calibrationToleranceEdit = new QLineEdit(QString::number(config.calibrationTargetTolerance));

    alphaEdit->setValidator(validator);
    betaEdit->setValidator(validator);
    kappaEdit->setValidator(validator);
    processNoiseEdit->setValidator(validator);
    measurementNoiseEdit->setValidator(validator);
    outlierNisThresholdEdit->setValidator(validator);
    calibrationRateEdit->setValidator(validator);
    calibrationToleranceEdit->setValidator(validator);

    const int compactEditWidth = 130;
    for (auto *edit : {alphaEdit, betaEdit, kappaEdit, processNoiseEdit, measurementNoiseEdit,
                       outlierNisThresholdEdit, calibrationRateEdit, calibrationToleranceEdit}) {
        edit->setMaximumWidth(compactEditWidth);
    }

    filterLayout->addWidget(new QLabel("Alpha:"), 0, 0);
    filterLayout->addWidget(alphaEdit, 0, 1);
    filterLayout->addWidget(new QLabel("Beta:"), 1, 0);
    filterLayout->addWidget(betaEdit, 1, 1);
    filterLayout->addWidget(new QLabel("Kappa:"), 2, 0);
    filterLayout->addWidget(kappaEdit, 2, 1);
    filterLayout->addWidget(new QLabel("Process Noise:"), 3, 0);
    filterLayout->addWidget(processNoiseEdit, 3, 1);
    filterLayout->addWidget(new QLabel("Measurement Noise:"), 4, 0);
    filterLayout->addWidget(measurementNoiseEdit, 4, 1);
    filterLayout->addWidget(outlierGatingCheck, 5, 0, 1, 2);
    filterLayout->addWidget(new QLabel("NIS Threshold:"), 6, 0);
    filterLayout->addWidget(outlierNisThresholdEdit, 6, 1);
    filterLayout->addWidget(calibrationFeedbackCheck, 7, 0, 1, 2);
    filterLayout->addWidget(new QLabel("Calibration Rate:"), 8, 0);
    filterLayout->addWidget(calibrationRateEdit, 8, 1);
    filterLayout->addWidget(new QLabel("Calibration Tolerance:"), 9, 0);
    filterLayout->addWidget(calibrationToleranceEdit, 9, 1);

    filterGroupBox->setLayout(filterLayout);

    QGroupBox *simulationGroupBox = new QGroupBox("Параметры симуляции");
    QGridLayout *simulationLayout = new QGridLayout();

    noiseLevelEdit = new QLineEdit(QString::number(config.noiseLevel));
    noiseLevelEdit->setValidator(validator);

    gapSizeEdit = new QLineEdit(QString::number(config.gapSize));
    gapSizeEdit->setValidator(validator);
    noiseLevelEdit->setMaximumWidth(compactEditWidth);
    gapSizeEdit->setMaximumWidth(compactEditWidth);

    simulationLayout->addWidget(new QLabel("Noise Level:"), 0, 0);
    simulationLayout->addWidget(noiseLevelEdit, 0, 1);
    simulationLayout->addWidget(new QLabel("Gap Size:"), 1, 0);
    simulationLayout->addWidget(gapSizeEdit, 1, 1);

    simulationGroupBox->setLayout(simulationLayout);

    QGroupBox *hardwareGroupBox = new QGroupBox("Параметры оборудования");
    QGridLayout *hardwareLayout = new QGridLayout();

    hardwareSerialEdit = new QLineEdit(QString::number(config.hardwareSerialNumber));
    hardwareSerialEdit->setValidator(new QIntValidator(0, 255, this));
    hardwareSerialEdit->setMaximumWidth(compactEditWidth);

    hardwareLayout->addWidget(new QLabel("Serial:"), 0, 0);
    hardwareLayout->addWidget(hardwareSerialEdit, 0, 1);

    for (int i = 0; i < 4; ++i) {
        hardwareFrequencyEdits[i] = new QLineEdit(
            QString::number(config.hardwareFrequenciesMHz[static_cast<std::size_t>(i)]));
        hardwareAmplitudeEdits[i] = new QLineEdit(
            QString::number(config.hardwareAmplitudes[static_cast<std::size_t>(i)]));
        hardwareFrequencyEdits[i]->setValidator(validator);
        hardwareAmplitudeEdits[i]->setValidator(validator);
        hardwareFrequencyEdits[i]->setMaximumWidth(compactEditWidth);
        hardwareAmplitudeEdits[i]->setMaximumWidth(100);
        hardwareLayout->addWidget(new QLabel(QString("F%1 (MHz):").arg(i + 1)), i + 1, 0);
        hardwareLayout->addWidget(hardwareFrequencyEdits[i], i + 1, 1);
        hardwareLayout->addWidget(new QLabel(QString("A%1:").arg(i + 1)), i + 1, 2);
        hardwareLayout->addWidget(hardwareAmplitudeEdits[i], i + 1, 3);
    }

    hardwareConnectButton = new QPushButton("Connect");
    hardwareDisconnectButton = new QPushButton("Disconnect");
    hardwareDisconnectButton->setEnabled(false);
    hardwareDeviceStatusLabel = new QLabel("Устройство: не подключено");

    hardwareLayout->addWidget(hardwareConnectButton, 5, 0, 1, 2);
    hardwareLayout->addWidget(hardwareDisconnectButton, 5, 2, 1, 2);
    hardwareLayout->addWidget(hardwareDeviceStatusLabel, 6, 0, 1, 4);
    hardwareGroupBox->setLayout(hardwareLayout);

    startButton = new QPushButton("Старт");
    stopButton = new QPushButton("Стоп");
    resetButton = new QPushButton("Сброс");
    stopButton->setEnabled(false);
    resetButton->setEnabled(false);

    connect(startButton, &QPushButton::clicked, this, &MainWindow::startSimulation);
    connect(stopButton, &QPushButton::clicked, this, &MainWindow::stopSimulation);
    connect(resetButton, &QPushButton::clicked, this, &MainWindow::resetSimulation);

    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(startButton);
    buttonLayout->addWidget(stopButton);
    buttonLayout->addWidget(resetButton);

    for (auto *edit : {alphaEdit, betaEdit, kappaEdit, processNoiseEdit, measurementNoiseEdit,
                       outlierNisThresholdEdit, calibrationRateEdit, calibrationToleranceEdit,
                       noiseLevelEdit, gapSizeEdit, hardwareSerialEdit}) {
        connect(edit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    }
    for (int i = 0; i < 4; ++i) {
        connect(hardwareFrequencyEdits[i], &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
        connect(hardwareAmplitudeEdits[i], &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    }
    connect(outlierGatingCheck, &QCheckBox::toggled, this, &MainWindow::onParametersChanged);
    connect(calibrationFeedbackCheck, &QCheckBox::toggled, this, &MainWindow::onParametersChanged);
    connect(hardwareConnectButton, &QPushButton::clicked, this, &MainWindow::connectHardwareDevice);
    connect(hardwareDisconnectButton, &QPushButton::clicked, this, &MainWindow::disconnectHardwareDevice);

    QGroupBox *intensityGroupBox = new QGroupBox("Отображение интенсивностей");
    QGridLayout *intensityLayout = new QGridLayout();
    QString labels[4] = {"I1", "I2", "I3", "I4"};
    for (int i = 0; i < 4; ++i) {
        intensityCheckBoxes[i] = new QCheckBox(labels[i]);
        intensityCheckBoxes[i]->setChecked(true);
        intensityLayout->addWidget(intensityCheckBoxes[i], i / 2, i % 2);
        int index = i;
        connect(intensityCheckBoxes[i], &QCheckBox::toggled, [this, index](bool checked){
            onIntensityCurveToggled(index, checked);
        });
    }
    intensityGroupBox->setLayout(intensityLayout);

    totalErrorLabel = new QLabel("Средняя ошибка за всё время: 0.0");
    recentErrorLabel = new QLabel("Средняя ошибка за последние 3 секунды: 0.0");
    nisLabel = new QLabel("Последний NIS: N/A");
    acceptanceLabel = new QLabel("Последнее измерение: N/A");
    calibrationStatusLabel = new QLabel("Калибровка: выключена");
    calibrationGainsLabel = new QLabel("Коэффициенты каналов: 1.000 1.000 1.000 1.000");
    hardwareFeedbackLabel = new QLabel("Аппаратная ОС: неактивна");

    QVBoxLayout *errorLabelsLayout = new QVBoxLayout();
    errorLabelsLayout->addWidget(totalErrorLabel);
    errorLabelsLayout->addWidget(recentErrorLabel);
    errorLabelsLayout->addWidget(nisLabel);
    errorLabelsLayout->addWidget(acceptanceLabel);
    errorLabelsLayout->addWidget(calibrationStatusLabel);
    errorLabelsLayout->addWidget(calibrationGainsLabel);
    errorLabelsLayout->addWidget(hardwareFeedbackLabel);

    QGroupBox *errorGroupBox = new QGroupBox("Ошибки и диагностика");
    errorGroupBox->setLayout(errorLabelsLayout);

    rawIntensityPlot = new QwtPlot(this);
    rawIntensityPlot->setTitle("Нефильтрованные интенсивности");
    rawIntensityPlot->setAxisTitle(QwtPlot::xBottom, "Время");
    rawIntensityPlot->setAxisTitle(QwtPlot::yLeft, "Интенсивность");

    filteredIntensityPlot = new QwtPlot(this);
    filteredIntensityPlot->setTitle("Фильтрованные интенсивности");
    filteredIntensityPlot->setAxisTitle(QwtPlot::xBottom, "Время");
    filteredIntensityPlot->setAxisTitle(QwtPlot::yLeft, "Интенсивность");

    estimatedCoordinatePlot = new QwtPlot(this);
    estimatedCoordinatePlot->setTitle("Координаты (фильтрованные)");
    estimatedCoordinatePlot->setAxisTitle(QwtPlot::xBottom, "Время");
    estimatedCoordinatePlot->setAxisTitle(QwtPlot::yLeft, "Координаты");

    trueCoordinatePlot = new QwtPlot(this);
    trueCoordinatePlot->setTitle("Исходные координаты");
    trueCoordinatePlot->setAxisTitle(QwtPlot::xBottom, "Время");
    trueCoordinatePlot->setAxisTitle(QwtPlot::yLeft, "Координаты");

    errorPlot = new QwtPlot(this);
    errorPlot->setTitle("Ошибка координат");
    errorPlot->setAxisTitle(QwtPlot::xBottom, "Время");
    errorPlot->setAxisTitle(QwtPlot::yLeft, "Ошибка");

    new QwtPlotZoomer(rawIntensityPlot->canvas());
    new QwtPlotPanner(rawIntensityPlot->canvas());
    new QwtPlotZoomer(filteredIntensityPlot->canvas());
    new QwtPlotPanner(filteredIntensityPlot->canvas());
    new QwtPlotZoomer(estimatedCoordinatePlot->canvas());
    new QwtPlotPanner(estimatedCoordinatePlot->canvas());
    new QwtPlotZoomer(trueCoordinatePlot->canvas());
    new QwtPlotPanner(trueCoordinatePlot->canvas());
    new QwtPlotZoomer(errorPlot->canvas());
    new QwtPlotPanner(errorPlot->canvas());

    rawIntensityPlot->insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
    filteredIntensityPlot->insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
    estimatedCoordinatePlot->insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
    trueCoordinatePlot->insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
    errorPlot->insertLegend(new QwtLegend(), QwtPlot::BottomLegend);

    QGridLayout *plotsLayout = new QGridLayout();
    plotsLayout->addWidget(rawIntensityPlot, 0, 0);
    plotsLayout->addWidget(filteredIntensityPlot, 0, 1);
    plotsLayout->addWidget(estimatedCoordinatePlot, 1, 0);
    plotsLayout->addWidget(trueCoordinatePlot, 1, 1);
    plotsLayout->addWidget(errorPlot, 2, 0, 1, 2);
    plotsLayout->setColumnStretch(0, 1);
    plotsLayout->setColumnStretch(1, 1);
    plotsLayout->setRowStretch(0, 2);
    plotsLayout->setRowStretch(1, 2);
    plotsLayout->setRowStretch(2, 2);

    QWidget *controlsWidget = new QWidget(this);
    QVBoxLayout *controlsLayout = new QVBoxLayout(controlsWidget);
    controlsLayout->setContentsMargins(0, 0, 0, 0);
    controlsLayout->setSpacing(8);
    controlsLayout->addWidget(filterGroupBox);
    controlsLayout->addWidget(simulationGroupBox);
    controlsLayout->addWidget(hardwareGroupBox);
    controlsLayout->addLayout(buttonLayout);
    controlsLayout->addWidget(intensityGroupBox);
    controlsLayout->addStretch(1);

    QScrollArea *controlsScroll = new QScrollArea(this);
    controlsScroll->setWidgetResizable(true);
    controlsScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    controlsScroll->setMinimumWidth(420);
    controlsScroll->setMaximumWidth(520);
    controlsScroll->setWidget(controlsWidget);

    QWidget *plotsWidget = new QWidget(this);
    QVBoxLayout *plotsContainerLayout = new QVBoxLayout(plotsWidget);
    plotsContainerLayout->setContentsMargins(0, 0, 0, 0);
    plotsContainerLayout->setSpacing(8);
    errorGroupBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
    plotsContainerLayout->addWidget(errorGroupBox);
    plotsContainerLayout->addLayout(plotsLayout, 1);

    QSplitter *splitter = new QSplitter(Qt::Horizontal, this);
    splitter->addWidget(controlsScroll);
    splitter->addWidget(plotsWidget);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes(QList<int>{460, 1200});
    rootLayout->addWidget(splitter);

    setCentralWidget(centralWidget);

    setMinimumSize(800, 600);
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
    int height = screenGeometry.height();
    int width = screenGeometry.width();
    resize(width * 0.8, height * 0.8);
}

void MainWindow::setupPlots() {
    QColor colors[4] = {Qt::red, Qt::blue, Qt::green, Qt::magenta};
    for (int i = 0; i < 4; ++i) {
        rawIntensityCurves[i] = new QwtPlotCurve(QString("Raw I%1").arg(i + 1));
        rawIntensityCurves[i]->attach(rawIntensityPlot);
        rawIntensityCurves[i]->setPen(QPen(colors[i]));
        rawIntensityCurves[i]->setVisible(intensityCheckBoxes[i]->isChecked());

        filteredIntensityCurves[i] = new QwtPlotCurve(QString("Filtered I%1").arg(i + 1));
        filteredIntensityCurves[i]->attach(filteredIntensityPlot);
        filteredIntensityCurves[i]->setPen(QPen(colors[i]));
        filteredIntensityCurves[i]->setVisible(intensityCheckBoxes[i]->isChecked());
    }

    estimatedPositionCurveX = new QwtPlotCurve("Estimated X");
    estimatedPositionCurveX->attach(estimatedCoordinatePlot);
    estimatedPositionCurveX->setPen(QPen(Qt::red));

    estimatedPositionCurveY = new QwtPlotCurve("Estimated Y");
    estimatedPositionCurveY->attach(estimatedCoordinatePlot);
    estimatedPositionCurveY->setPen(QPen(Qt::blue));

    truePositionCurveX = new QwtPlotCurve("True X");
    truePositionCurveX->attach(trueCoordinatePlot);
    truePositionCurveX->setPen(QPen(Qt::green));

    truePositionCurveY = new QwtPlotCurve("True Y");
    truePositionCurveY->attach(trueCoordinatePlot);
    truePositionCurveY->setPen(QPen(Qt::magenta));

    errorCurve = new QwtPlotCurve("Error");
    errorCurve->attach(errorPlot);
    errorCurve->setPen(QPen(Qt::red));

    estimatedCoordinatePlot->setAxisScale(QwtPlot::yLeft, -0.75, 0.75);
}

void MainWindow::initializeCurves() {
    timeData.clear();
    for (int i = 0; i < 4; ++i) {
        rawIntensityData[i].clear();
        filteredIntensityData[i].clear();
    }
    estimatedXData.clear();
    estimatedYData.clear();
    truthTimeData.clear();
    trueXData.clear();
    trueYData.clear();
    errorTimeData.clear();
    errorData.clear();
}

void MainWindow::trimPlotBuffers() {
    const int maxPoints = std::max(100, config.maxPlotPoints);

    auto trimVec = [maxPoints](QVector<double> &vec) {
        if (vec.size() > maxPoints) {
            vec.remove(0, vec.size() - maxPoints);
        }
    };

    trimVec(timeData);
    for (int i = 0; i < 4; ++i) {
        trimVec(rawIntensityData[i]);
        trimVec(filteredIntensityData[i]);
    }
    trimVec(estimatedXData);
    trimVec(estimatedYData);

    trimVec(truthTimeData);
    trimVec(trueXData);
    trimVec(trueYData);

    trimVec(errorTimeData);
    trimVec(errorData);
}

void MainWindow::showError(const QString &message) {
    QMessageBox::critical(this, "Ошибка", message);
}

void MainWindow::validateInput() {
    bool ok;
    double val;
    double alphaValue = alphaEdit->text().toDouble(&ok);
    if (!ok || alphaValue <= 0) throw std::runtime_error("Некорректное значение Alpha.");

    val = betaEdit->text().toDouble(&ok);
    if (!ok || val <= 0) throw std::runtime_error("Некорректное значение Beta.");

    double kappaValue = kappaEdit->text().toDouble(&ok);
    if (!ok) throw std::runtime_error("Некорректное значение Kappa.");
    constexpr double ukfStateSize = 4.0;
    if (alphaValue * alphaValue * (ukfStateSize + kappaValue) <= 0.0) {
        throw std::runtime_error("Некорректный Kappa: для UKF должно выполняться alpha^2 * (n + kappa) > 0, где n=4.");
    }

    val = processNoiseEdit->text().toDouble(&ok);
    if (!ok || val < 0) throw std::runtime_error("Некорректное значение Process Noise.");

    val = measurementNoiseEdit->text().toDouble(&ok);
    if (!ok || val < 0) throw std::runtime_error("Некорректное значение Measurement Noise.");

    val = outlierNisThresholdEdit->text().toDouble(&ok);
    if (!ok || val <= 0) throw std::runtime_error("Некорректное значение NIS Threshold.");

    val = calibrationRateEdit->text().toDouble(&ok);
    if (!ok || val < 0.0 || val > 1.0) throw std::runtime_error("Некорректное значение Calibration Rate (должно быть в диапазоне [0,1]).");

    val = calibrationToleranceEdit->text().toDouble(&ok);
    if (!ok || val <= 0.0) throw std::runtime_error("Некорректное значение Calibration Tolerance.");

    val = noiseLevelEdit->text().toDouble(&ok);
    if (!ok || val < 0) throw std::runtime_error("Некорректное значение Noise Level.");

    val = gapSizeEdit->text().toDouble(&ok);
    if (!ok || val < 0.0 || val >= 1.0) throw std::runtime_error("Некорректное значение Gap Size.");

    int serial = hardwareSerialEdit->text().toInt(&ok);
    if (!ok || serial < 0 || serial > 255) {
        throw std::runtime_error("Некорректный serial number (должен быть в диапазоне 0..255).");
    }
    for (int i = 0; i < 4; ++i) {
        const double freq = hardwareFrequencyEdits[i]->text().toDouble(&ok);
        if (!ok || freq <= 0.0) {
            throw std::runtime_error("Некорректная аппаратная частота канала.");
        }
        const double amp = hardwareAmplitudeEdits[i]->text().toDouble(&ok);
        if (!ok || amp < 0.0 || amp > 1.0) {
            throw std::runtime_error("Некорректная аппаратная амплитуда (должна быть в диапазоне [0,1]).");
        }
    }
}

void MainWindow::onParametersChanged() {
    if (isRunning) return;

    config.alpha = alphaEdit->text().toDouble();
    config.beta = betaEdit->text().toDouble();
    config.kappa = kappaEdit->text().toDouble();
    config.processNoise = processNoiseEdit->text().toDouble();
    config.measurementNoise = measurementNoiseEdit->text().toDouble();
    config.enableOutlierGating = outlierGatingCheck->isChecked();
    config.outlierNisThreshold = outlierNisThresholdEdit->text().toDouble();
    config.enableCalibrationFeedback = calibrationFeedbackCheck->isChecked();
    config.calibrationFeedbackRate = calibrationRateEdit->text().toDouble();
    config.calibrationTargetTolerance = calibrationToleranceEdit->text().toDouble();
    config.noiseLevel = noiseLevelEdit->text().toDouble();
    config.gapSize = gapSizeEdit->text().toDouble();
    config.hardwareSerialNumber = hardwareSerialEdit->text().toInt();
    for (int i = 0; i < 4; ++i) {
        config.hardwareFrequenciesMHz[static_cast<std::size_t>(i)] =
            static_cast<float>(hardwareFrequencyEdits[i]->text().toDouble());
        config.hardwareAmplitudes[static_cast<std::size_t>(i)] =
            static_cast<float>(hardwareAmplitudeEdits[i]->text().toDouble());
    }
}

void MainWindow::startSimulation() {
    if (isRunning) {
        QMessageBox::warning(this, "Ошибка", "Симуляция уже запущена.");
        return;
    }

    try {
        validateInput();
    } catch (const std::exception &e) {
        showError(e.what());
        return;
    }

    onParametersChanged();

    std::vector<Eigen::VectorXd> realtimeMeasurements;
    if (config.mode == "realtime") {
        realtimeMeasurements.reserve(static_cast<std::size_t>(loadedMeasurements.size()));
        for (const auto &measurement : loadedMeasurements) {
            realtimeMeasurements.push_back(measurement);
        }
    }

    if (config.mode == "hardware" && manualBoardDriver && manualBoardDriver->isConnected()) {
        manualBoardDriver->disconnect();
        manualBoardDriver.reset();
        updateHardwareDeviceStatus();
    }

    trackingEngine = std::make_unique<TrackingEngine>(config);
    std::string initError;
    if (!trackingEngine->initialize(realtimeMeasurements, initError)) {
        showError(QString::fromStdString(initError));
        trackingEngine.reset();
        return;
    }

    setControlsEnabled(false);

    iteration = 0;
    threadedHardwareInput = (config.mode == "hardware");
    if (threadedHardwareInput) {
        hardwareInputThread = std::make_unique<HardwareInputThread>(trackingEngine.get(), this);
        hardwareInputThread->start();
    }

    initializeCurves();
    if (config.mode == "simulation") {
        totalErrorLabel->setText("Средняя ошибка за всё время: 0.0");
        recentErrorLabel->setText("Средняя ошибка за последние 3 секунды: 0.0");
    } else {
        totalErrorLabel->setText("Средняя ошибка за всё время: N/A (нет ground truth)");
        recentErrorLabel->setText("Средняя ошибка за последние 3 секунды: N/A (нет ground truth)");
    }
    nisLabel->setText("Последний NIS: N/A");
    acceptanceLabel->setText("Последнее измерение: N/A");
    if (config.enableCalibrationFeedback) {
        calibrationStatusLabel->setText("Калибровка: активна");
    } else {
        calibrationStatusLabel->setText("Калибровка: выключена");
    }
    calibrationGainsLabel->setText("Коэффициенты каналов: 1.000 1.000 1.000 1.000");
    hardwareFeedbackLabel->setText("Аппаратная ОС: неактивна");

    isRunning = true;
    startButton->setEnabled(false);
    stopButton->setEnabled(true);
    resetButton->setEnabled(true);

    updateTimer->start(std::max(1, config.timerIntervalMs));
}

void MainWindow::stopSimulation() {
    if (!isRunning) {
        QMessageBox::warning(this, "Ошибка", "Симуляция не запущена.");
        return;
    }

    isRunning = false;
    updateTimer->stop();

    if (hardwareInputThread) {
        hardwareInputThread->requestStop();
        hardwareInputThread->wait(3000);
        hardwareInputThread.reset();
    }
    threadedHardwareInput = false;

    setControlsEnabled(true);
    trackingEngine.reset();
    updateHardwareDeviceStatus();

    startButton->setEnabled(true);
    stopButton->setEnabled(false);
}

void MainWindow::resetSimulation() {
    if (isRunning) {
        stopSimulation();
    }
    initializeCurves();
    rawIntensityPlot->replot();
    filteredIntensityPlot->replot();
    estimatedCoordinatePlot->replot();
    trueCoordinatePlot->replot();
    errorPlot->replot();
    if (config.mode == "simulation") {
        totalErrorLabel->setText("Средняя ошибка за всё время: 0.0");
        recentErrorLabel->setText("Средняя ошибка за последние 3 секунды: 0.0");
    } else {
        totalErrorLabel->setText("Средняя ошибка за всё время: N/A (нет ground truth)");
        recentErrorLabel->setText("Средняя ошибка за последние 3 секунды: N/A (нет ground truth)");
    }
    nisLabel->setText("Последний NIS: N/A");
    acceptanceLabel->setText("Последнее измерение: N/A");
    if (config.enableCalibrationFeedback) {
        calibrationStatusLabel->setText("Калибровка: активна");
    } else {
        calibrationStatusLabel->setText("Калибровка: выключена");
    }
    calibrationGainsLabel->setText("Коэффициенты каналов: 1.000 1.000 1.000 1.000");
    hardwareFeedbackLabel->setText("Аппаратная ОС: неактивна");
    resetButton->setEnabled(false);
}

void MainWindow::setControlsEnabled(bool enabled) {
    alphaEdit->setEnabled(enabled);
    betaEdit->setEnabled(enabled);
    kappaEdit->setEnabled(enabled);
    processNoiseEdit->setEnabled(enabled);
    measurementNoiseEdit->setEnabled(enabled);
    outlierGatingCheck->setEnabled(enabled);
    outlierNisThresholdEdit->setEnabled(enabled);
    calibrationFeedbackCheck->setEnabled(enabled);
    calibrationRateEdit->setEnabled(enabled);
    calibrationToleranceEdit->setEnabled(enabled);
    noiseLevelEdit->setEnabled(enabled);
    gapSizeEdit->setEnabled(enabled);
    hardwareSerialEdit->setEnabled(enabled);
    for (int i = 0; i < 4; ++i) {
        intensityCheckBoxes[i]->setEnabled(enabled);
        hardwareFrequencyEdits[i]->setEnabled(enabled);
        hardwareAmplitudeEdits[i]->setEnabled(enabled);
    }
    hardwareConnectButton->setEnabled(enabled);
    hardwareDisconnectButton->setEnabled(enabled && manualBoardDriver && manualBoardDriver->isConnected());
}

void MainWindow::updateHardwareDeviceStatus() {
    if (manualBoardDriver && manualBoardDriver->isConnected()) {
        hardwareDeviceStatusLabel->setText(
            QString("Устройство: подключено (%1)")
                .arg(QString::fromStdString(manualBoardDriver->name())));
        hardwareConnectButton->setEnabled(false);
        hardwareDisconnectButton->setEnabled(!isRunning);
    } else {
        hardwareDeviceStatusLabel->setText("Устройство: не подключено");
        hardwareConnectButton->setEnabled(!isRunning);
        hardwareDisconnectButton->setEnabled(false);
    }
}

void MainWindow::connectHardwareDevice() {
    if (isRunning) {
        QMessageBox::warning(this, "Ошибка", "Нельзя подключать устройство во время активной симуляции.");
        return;
    }

    try {
        validateInput();
    } catch (const std::exception &e) {
        showError(e.what());
        return;
    }
    onParametersChanged();

    manualBoardDriver = createBoardDriver();
    BoardConnectionConfig hwCfg;
    hwCfg.serialNumber = config.hardwareSerialNumber;
    hwCfg.applyOutputOnConnect = config.hardwareApplyOutputOnConnect;
    hwCfg.frequenciesMHz = config.hardwareFrequenciesMHz;
    hwCfg.amplitudes = config.hardwareAmplitudes;

    std::string error;
    if (!manualBoardDriver->connect(hwCfg, error)) {
        showError(QString("Не удалось подключить устройство: %1").arg(QString::fromStdString(error)));
        manualBoardDriver.reset();
    }
    updateHardwareDeviceStatus();
}

void MainWindow::disconnectHardwareDevice() {
    if (manualBoardDriver) {
        manualBoardDriver->disconnect();
        manualBoardDriver.reset();
    }
    updateHardwareDeviceStatus();
}

void MainWindow::onIntensityCurveToggled(int index, bool checked) {
    if (index >= 0 && index < 4) {
        rawIntensityCurves[index]->setVisible(checked);
        filteredIntensityCurves[index]->setVisible(checked);
        rawIntensityPlot->replot();
        filteredIntensityPlot->replot();
    }
}

void MainWindow::updatePlots() {
    if (!isRunning || !trackingEngine) return;

    if (threadedHardwareInput) {
        if (!hardwareInputThread) {
            stopSimulation();
            return;
        }

        QString workerError;
        if (hardwareInputThread->takeError(workerError)) {
            dataLogger->logMessage(QString("Engine step failed: %1").arg(workerError));
            showError(workerError);
            stopSimulation();
            return;
        }

        int processed = 0;
        TrackingStepResult stepResult;
        while (processed < 32 && hardwareInputThread->popResult(stepResult)) {
            processStepResult(stepResult);
            ++processed;
        }

        if (processed == 0 && hardwareInputThread->isWorkerFinished()) {
            stopSimulation();
        }
        return;
    }

    TrackingStepResult stepResult;
    std::string error;
    if (!trackingEngine->step(stepResult, error)) {
        if (!error.empty()) {
            dataLogger->logMessage(QString("Engine step failed: %1").arg(QString::fromStdString(error)));
            showError(QString::fromStdString(error));
        }
        stopSimulation();
        return;
    }
    processStepResult(stepResult);
}

void MainWindow::processStepResult(const TrackingStepResult &stepResult) {
    if (!stepResult.measurementAccepted) {
        dataLogger->logMessage(QString("Measurement rejected by NIS gating. NIS=%1").arg(stepResult.nis));
    }

    timeData.append(stepResult.time);
    for (int i = 0; i < 4; ++i) {
        rawIntensityData[i].append(stepResult.rawMeasurement(i));
        filteredIntensityData[i].append(stepResult.filteredState(i));
    }

    estimatedXData.append(stepResult.estimatedX);
    estimatedYData.append(stepResult.estimatedY);

    if (stepResult.hasGroundTruth) {
        truthTimeData.append(stepResult.time);
        trueXData.append(stepResult.trueX);
        trueYData.append(stepResult.trueY);
        if (stepResult.errorValid && std::isfinite(stepResult.error)) {
            errorTimeData.append(stepResult.time);
            errorData.append(stepResult.error);
        }
    }

    trimPlotBuffers();

    for (int i = 0; i < 4; ++i) {
        rawIntensityCurves[i]->setSamples(timeData, rawIntensityData[i]);
        filteredIntensityCurves[i]->setSamples(timeData, filteredIntensityData[i]);
    }

    estimatedPositionCurveX->setSamples(timeData, estimatedXData);
    estimatedPositionCurveY->setSamples(timeData, estimatedYData);
    truePositionCurveX->setSamples(truthTimeData, trueXData);
    truePositionCurveY->setSamples(truthTimeData, trueYData);
    errorCurve->setSamples(errorTimeData, errorData);

    if (stepResult.hasGroundTruth) {
        double totalError = 0.0;
        for (double e : errorData) totalError += e;
        if (!errorData.isEmpty()) totalError /= errorData.size();
        totalErrorLabel->setText(QString("Средняя ошибка за всё время: %1").arg(totalError));

        double recentError = 0.0;
        int recentCount = 0;
        for (int i = errorData.size() - 1; i >= 0; --i) {
            if (errorTimeData.last() - errorTimeData[i] <= 3.0) {
                recentError += errorData[i];
                recentCount++;
            } else {
                break;
            }
        }
        if (recentCount > 0) {
            recentError /= recentCount;
            recentErrorLabel->setText(QString("Средняя ошибка за последние 3 секунды: %1").arg(recentError));
        } else {
            recentErrorLabel->setText("Средняя ошибка за последние 3 секунды: 0.0");
        }
    } else {
        totalErrorLabel->setText("Средняя ошибка за всё время: N/A (нет ground truth)");
        recentErrorLabel->setText("Средняя ошибка за последние 3 секунды: N/A (нет ground truth)");
    }

    if (std::isfinite(stepResult.nis)) {
        nisLabel->setText(QString("Последний NIS: %1").arg(stepResult.nis));
    } else {
        nisLabel->setText("Последний NIS: N/A");
    }
    acceptanceLabel->setText(stepResult.measurementAccepted
                             ? "Последнее измерение: принято"
                             : "Последнее измерение: отклонено (gating)");

    if (stepResult.calibrationEnabled) {
        const QString stateText = stepResult.calibrationConverged ? "сходимость достигнута" : "идет подстройка";
        calibrationStatusLabel->setText(
            QString("Калибровка: %1 | eX=%2 eY=%3")
            .arg(stateText)
            .arg(stepResult.calibrationErrorX, 0, 'f', 5)
            .arg(stepResult.calibrationErrorY, 0, 'f', 5));
        calibrationGainsLabel->setText(
            QString("Коэффициенты каналов: %1 %2 %3 %4")
            .arg(stepResult.calibrationGains(0), 0, 'f', 3)
            .arg(stepResult.calibrationGains(1), 0, 'f', 3)
            .arg(stepResult.calibrationGains(2), 0, 'f', 3)
            .arg(stepResult.calibrationGains(3), 0, 'f', 3));
    } else {
        calibrationStatusLabel->setText("Калибровка: выключена");
        calibrationGainsLabel->setText("Коэффициенты каналов: 1.000 1.000 1.000 1.000");
    }

    if (stepResult.hardwareFeedbackEnabled) {
        const QString applyState = stepResult.hardwareFeedbackApplied ? "применено" : "ожидание";
        hardwareFeedbackLabel->setText(
            QString("Аппаратная ОС: %1 | F=[%2 %3 %4 %5] MHz | A=[%6 %7 %8 %9]")
                .arg(applyState)
                .arg(stepResult.hardwareFrequenciesMHz(0), 0, 'f', 3)
                .arg(stepResult.hardwareFrequenciesMHz(1), 0, 'f', 3)
                .arg(stepResult.hardwareFrequenciesMHz(2), 0, 'f', 3)
                .arg(stepResult.hardwareFrequenciesMHz(3), 0, 'f', 3)
                .arg(stepResult.hardwareAmplitudes(0), 0, 'f', 3)
                .arg(stepResult.hardwareAmplitudes(1), 0, 'f', 3)
                .arg(stepResult.hardwareAmplitudes(2), 0, 'f', 3)
                .arg(stepResult.hardwareAmplitudes(3), 0, 'f', 3));
    } else {
        hardwareFeedbackLabel->setText("Аппаратная ОС: неактивна");
    }

    rawIntensityPlot->replot();
    filteredIntensityPlot->replot();
    estimatedCoordinatePlot->replot();
    trueCoordinatePlot->replot();
    errorPlot->replot();

    dataLogger->addRecord(stepResult.time,
                          stepResult.rawMeasurement,
                          stepResult.calibratedMeasurement,
                          stepResult.filteredState,
                          stepResult.trueX,
                          stepResult.trueY,
                          stepResult.estimatedX,
                          stepResult.estimatedY,
                          stepResult.error,
                          stepResult.calibrationEnabled,
                          stepResult.calibrationConverged,
                          stepResult.calibrationErrorX,
                          stepResult.calibrationErrorY,
                          stepResult.calibrationGains,
                          stepResult.hardwareFeedbackEnabled,
                          stepResult.hardwareFeedbackApplied,
                          stepResult.hardwareFrequenciesMHz,
                          stepResult.hardwareAmplitudes);

    iteration++;
}

void MainWindow::loadRealData() {
    loadedMeasurements.clear();
    QFile file(QString::fromStdString(config.inputDataFile));
    if (!file.open(QIODevice::ReadOnly)) {
        showError("Не удалось открыть файл реальных данных.");
        return;
    }

    QByteArray data = file.readAll();
    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isArray()) {
        showError("Неверный формат файла реальных данных.");
        return;
    }

    QJsonArray arr = doc.array();
    for (const auto &val : arr) {
        if (!val.isObject()) {
            continue;
        }
        QJsonObject obj = val.toObject();
        QJsonArray measArr = obj["measurements"].toArray();
        if (measArr.size() == 4) {
            bool validRow = true;
            Eigen::VectorXd m(4);
            for (int i = 0; i < 4; ++i) {
                if (!measArr[i].isDouble()) {
                    validRow = false;
                    break;
                }
                m(i) = measArr[i].toDouble();
            }
            if (validRow && !m.hasNaN()) {
                loadedMeasurements.append(m);
            }
        }
    }
}
