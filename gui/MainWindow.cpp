#include "MainWindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <QDoubleValidator>
#include <QScreen>
#include <QGuiApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <cmath>
#include <algorithm>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_legend.h>

MainWindow::MainWindow(const Config &cfg, QWidget *parent)
    : QMainWindow(parent), config(cfg),
      currentTime(0.0), iteration(0), isRunning(false)
{
    setupUI();
    setupPlots();

    dataLogger = std::make_unique<DataLogger>(
                QString::fromStdString(config.logFileName),
                QString::fromStdString(config.jsonOutputFile));
    dataLogger->logMessage("Application started");

    if (config.mode == "realtime") {
        loadRealData();
    }

    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::updatePlots);
}

MainWindow::~MainWindow() {
    // При выходе ~DataLogger сохранит JSON
}

void MainWindow::setupUI() {
    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);

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

    alphaEdit->setValidator(validator);
    betaEdit->setValidator(validator);
    kappaEdit->setValidator(validator);
    processNoiseEdit->setValidator(validator);
    measurementNoiseEdit->setValidator(validator);

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

    filterGroupBox->setLayout(filterLayout);

    QGroupBox *simulationGroupBox = new QGroupBox("Параметры симуляции");
    QGridLayout *simulationLayout = new QGridLayout();

    noiseLevelEdit = new QLineEdit(QString::number(config.noiseLevel));
    noiseLevelEdit->setValidator(validator);

    gapSizeEdit = new QLineEdit(QString::number(config.gapSize));
    gapSizeEdit->setValidator(validator);

    simulationLayout->addWidget(new QLabel("Noise Level:"), 0, 0);
    simulationLayout->addWidget(noiseLevelEdit, 0, 1);
    simulationLayout->addWidget(new QLabel("Gap Size:"), 1, 0);
    simulationLayout->addWidget(gapSizeEdit, 1, 1);

    simulationGroupBox->setLayout(simulationLayout);

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

    for (auto *edit : {alphaEdit, betaEdit, kappaEdit, processNoiseEdit, measurementNoiseEdit, noiseLevelEdit, gapSizeEdit}) {
        connect(edit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    }

    QGroupBox *intensityGroupBox = new QGroupBox("Отображение интенсивностей");
    QHBoxLayout *intensityLayout = new QHBoxLayout();
    QString labels[4] = {"I1", "I2", "I3", "I4"};
    for (int i = 0; i < 4; ++i) {
        intensityCheckBoxes[i] = new QCheckBox(labels[i]);
        intensityCheckBoxes[i]->setChecked(true);
        intensityLayout->addWidget(intensityCheckBoxes[i]);
        int index = i;
        connect(intensityCheckBoxes[i], &QCheckBox::toggled, [this,index](bool checked){
            onIntensityCurveToggled(index,checked);
        });
    }
    intensityGroupBox->setLayout(intensityLayout);

    totalErrorLabel = new QLabel("Средняя ошибка за всё время: 0.0");
    recentErrorLabel = new QLabel("Средняя ошибка за последние 3 секунды: 0.0");

    QVBoxLayout *errorLabelsLayout = new QVBoxLayout();
    errorLabelsLayout->addWidget(totalErrorLabel);
    errorLabelsLayout->addWidget(recentErrorLabel);

    QGroupBox *errorGroupBox = new QGroupBox("Ошибки");
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

    // Зум и панорамирование
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

    mainLayout->addWidget(filterGroupBox);
    mainLayout->addWidget(simulationGroupBox);
    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(intensityGroupBox);
    mainLayout->addWidget(errorGroupBox);
    mainLayout->addLayout(plotsLayout);

    setCentralWidget(centralWidget);

    setMinimumSize(800,600);
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
    int height = screenGeometry.height();
    int width = screenGeometry.width();
    resize(width * 0.8, height * 0.8);
}

void MainWindow::setupPlots() {
    QColor colors[4] = {Qt::red, Qt::blue, Qt::green, Qt::magenta};
    for (int i = 0; i < 4; ++i) {
        rawIntensityCurves[i] = new QwtPlotCurve(QString("Raw I%1").arg(i+1));
        rawIntensityCurves[i]->attach(rawIntensityPlot);
        rawIntensityCurves[i]->setPen(QPen(colors[i]));
        rawIntensityCurves[i]->setVisible(intensityCheckBoxes[i]->isChecked());

        filteredIntensityCurves[i] = new QwtPlotCurve(QString("Filtered I%1").arg(i+1));
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
    trueXData.clear();
    trueYData.clear();
    errorData.clear();
}

void MainWindow::showError(const QString &message) {
    QMessageBox::critical(this, "Ошибка", message);
}

void MainWindow::validateInput() {
    bool ok;
    double val;
    val = alphaEdit->text().toDouble(&ok);
    if (!ok || val <= 0) throw std::runtime_error("Некорректное значение Alpha.");

    val = betaEdit->text().toDouble(&ok);
    if (!ok || val <= 0) throw std::runtime_error("Некорректное значение Beta.");

    val = kappaEdit->text().toDouble(&ok);
    if (!ok) throw std::runtime_error("Некорректное значение Kappa.");

    val = processNoiseEdit->text().toDouble(&ok);
    if (!ok || val < 0) throw std::runtime_error("Некорректное значение Process Noise.");

    val = measurementNoiseEdit->text().toDouble(&ok);
    if (!ok || val < 0) throw std::runtime_error("Некорректное значение Measurement Noise.");

    val = noiseLevelEdit->text().toDouble(&ok);
    if (!ok || val < 0) throw std::runtime_error("Некорректное значение Noise Level.");

    val = gapSizeEdit->text().toDouble(&ok);
    if (!ok || val < 0.0 || val >= 1.0) throw std::runtime_error("Некорректное значение Gap Size.");
}

void MainWindow::onParametersChanged() {
    if (isRunning) return;

    // Обновляем config из UI
    config.alpha = alphaEdit->text().toDouble();
    config.beta = betaEdit->text().toDouble();
    config.kappa = kappaEdit->text().toDouble();
    config.processNoise = processNoiseEdit->text().toDouble();
    config.measurementNoise = measurementNoiseEdit->text().toDouble();
    config.noiseLevel = noiseLevelEdit->text().toDouble();
    config.gapSize = gapSizeEdit->text().toDouble();
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

    alphaEdit->setEnabled(false);
    betaEdit->setEnabled(false);
    kappaEdit->setEnabled(false);
    processNoiseEdit->setEnabled(false);
    measurementNoiseEdit->setEnabled(false);
    noiseLevelEdit->setEnabled(false);
    gapSizeEdit->setEnabled(false);
    for (int i = 0; i < 4; ++i) {
        intensityCheckBoxes[i]->setEnabled(false);
    }

    onParametersChanged();

    Eigen::VectorXd initial_measurement(4);
    if (config.mode == "simulation") {
        beamSimulation = std::make_unique<BeamSimulation>(config.noiseLevel, config.gapSize, config.timeStep, config.beamSpeed);
        initial_measurement = beamSimulation->moveBeamAndIntegrate(config.beamPower, config.beamWidth);
    } else {
        // real-time: берем первый измерение из loadedMeasurements
        if (loadedMeasurements.isEmpty()) {
            showError("Нет данных для real-time режима.");
            return;
        }
        currentMeasurementIndex = 0;
        initial_measurement = loadedMeasurements[currentMeasurementIndex++];
    }

    Eigen::VectorXd initial_state = initial_measurement;
    Eigen::MatrixXd initial_cov = Eigen::MatrixXd::Identity(4,4);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4,4)*config.processNoise;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4,4)*config.measurementNoise;

    kalmanFilter = std::make_unique<AdaptiveUnscentedKalmanFilter>(initial_state, initial_cov, Q, R, config.alpha, config.beta, config.kappa);

    currentTime = 0.0;
    iteration = 0;
    initializeCurves();

    isRunning = true;
    startButton->setEnabled(false);
    stopButton->setEnabled(true);
    resetButton->setEnabled(true);

    updateTimer->start(10);
}

void MainWindow::stopSimulation() {
    if (!isRunning) {
        QMessageBox::warning(this, "Ошибка", "Симуляция не запущена.");
        return;
    }

    isRunning = false;
    updateTimer->stop();

    alphaEdit->setEnabled(true);
    betaEdit->setEnabled(true);
    kappaEdit->setEnabled(true);
    processNoiseEdit->setEnabled(true);
    measurementNoiseEdit->setEnabled(true);
    noiseLevelEdit->setEnabled(true);
    gapSizeEdit->setEnabled(true);
    for (int i = 0; i < 4; ++i) {
        intensityCheckBoxes[i]->setEnabled(true);
    }

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
    totalErrorLabel->setText("Средняя ошибка за всё время: 0.0");
    recentErrorLabel->setText("Средняя ошибка за последние 3 секунды: 0.0");
    resetButton->setEnabled(false);
}

void MainWindow::onIntensityCurveToggled(int index, bool checked) {
    if (index>=0 && index<4) {
        rawIntensityCurves[index]->setVisible(checked);
        filteredIntensityCurves[index]->setVisible(checked);
        rawIntensityPlot->replot();
        filteredIntensityPlot->replot();
    }
}

void MainWindow::updatePlots() {
    if (!isRunning) return;

    currentTime += config.timeStep;
    timeData.append(currentTime);

    Eigen::VectorXd measurement(4);
    double trueX, trueY;

    if (config.mode == "simulation") {
        measurement = beamSimulation->moveBeamAndIntegrate(config.beamPower, config.beamWidth);
        trueX = beamSimulation->getXc();
        trueY = beamSimulation->getYc();
    } else {
        if (currentMeasurementIndex >= loadedMeasurements.size()) {
            // Данные закончились
            stopSimulation();
            return;
        }
        measurement = loadedMeasurements[currentMeasurementIndex++];
        // Для "реальных" данных у нас может не быть trueX и trueY. 
        // Допустим, их нет. Можно поставить 0 или придумать другой способ.
        trueX = 0.0;
        trueY = 0.0;
    }

    if (measurement.hasNaN()) {
        dataLogger->logMessage("Measurement contains NaN");
        iteration++;
        return;
    }

    kalmanFilter->predict();
    kalmanFilter->update(measurement);

    Eigen::VectorXd currentState = kalmanFilter->getState();
    if (currentState.hasNaN()) {
        dataLogger->logMessage("Kalman Filter state contains NaN!");
        iteration++;
        return;
    }

    for (int i =0; i<4; ++i) {
        rawIntensityData[i].append(measurement(i));
        filteredIntensityData[i].append(currentState(i));
    }

    Eigen::Vector2d spot_position = kalmanFilter->calculateSpotPosition(config.beamWidth, config.x0);
    double estimatedX = spot_position(0);
    double estimatedY = spot_position(1);

    bool invalidCoordinates = std::isnan(estimatedX) || std::isnan(estimatedY) ||
                              std::isnan(trueX) || std::isnan(trueY) ||
                              std::isinf(estimatedX) || std::isinf(estimatedY) ||
                              std::isinf(trueX) || std::isinf(trueY);

    double errorVal = 0.0;
    if (!invalidCoordinates) {
        errorVal = std::sqrt(std::pow(estimatedX - trueX, 2) + std::pow(estimatedY - trueY, 2));
    }

    trueXData.append(trueX);
    trueYData.append(trueY);
    estimatedXData.append(estimatedX);
    estimatedYData.append(estimatedY);
    errorData.append(errorVal);

    // Обновляем кривые
    for (int i =0; i<4; ++i) {
        rawIntensityCurves[i]->setSamples(timeData, rawIntensityData[i]);
        filteredIntensityCurves[i]->setSamples(timeData, filteredIntensityData[i]);
    }

    estimatedPositionCurveX->setSamples(timeData, estimatedXData);
    estimatedPositionCurveY->setSamples(timeData, estimatedYData);
    truePositionCurveX->setSamples(timeData, trueXData);
    truePositionCurveY->setSamples(timeData, trueYData);
    errorCurve->setSamples(timeData, errorData);

    double totalError = 0.0;
    for (double e: errorData) totalError += e;
    if (!errorData.isEmpty()) totalError /= errorData.size();
    totalErrorLabel->setText(QString("Средняя ошибка за всё время: %1").arg(totalError));

    double recentError = 0.0;
    int recentCount =0;
    for (int i = errorData.size()-1; i>=0; --i) {
        if (timeData.last() - timeData[i] <= 3.0) {
            recentError += errorData[i];
            recentCount++;
        } else break;
    }
    if (recentCount > 0) {
        recentError /= recentCount;
        recentErrorLabel->setText(QString("Средняя ошибка за последние 3 секунды: %1").arg(recentError));
    }

    // Авто-масштабирование осей (пример)
    // ... (опущено для краткости, можно взять логику из исходного кода)

    rawIntensityPlot->replot();
    filteredIntensityPlot->replot();
    estimatedCoordinatePlot->replot();
    trueCoordinatePlot->replot();
    errorPlot->replot();

    // Логирование данных
    dataLogger->addRecord(currentTime, measurement, currentState, trueX, trueY, estimatedX, estimatedY, errorVal);

    iteration++;
}

void MainWindow::loadRealData() {
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
    for (auto val: arr) {
        QJsonObject obj = val.toObject();
        QJsonArray measArr = obj["measurements"].toArray();
        if (measArr.size() == 4) {
            Eigen::VectorXd m(4);
            for (int i=0; i<4; ++i)
                m(i) = measArr[i].toDouble();
            loadedMeasurements.append(m);
        }
    }
}
