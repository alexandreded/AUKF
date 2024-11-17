#include "MainWindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QDoubleValidator>
#include <QMessageBox>
#include <QGuiApplication>
#include <QScreen>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      alpha(1e-3),
      beta(2.0),
      kappa(0.0),
      processNoise(1e-5),
      measurementNoise(1e-2),
      noiseLevel(0.3),
      currentTime(0.0),
      timeStep(0.001),
      iteration(0),
      isRunning(false)
{
    setupUI();
    setupPlots();
    initializeCurves();

    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::updatePlots);

    // Установка минимального размера окна
    setMinimumSize(800, 600);

    // Масштабирование окна до 80% от размера экрана
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
    int height = screenGeometry.height();
    int width = screenGeometry.width();
    resize(width * 0.8, height * 0.8);
}

void MainWindow::setupUI() {
    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);

    // Группа для параметров фильтра
    QGroupBox *filterGroupBox = new QGroupBox("Параметры фильтра");
    QGridLayout *filterLayout = new QGridLayout();

    alphaEdit = new QLineEdit(QString::number(alpha));
    betaEdit = new QLineEdit(QString::number(beta));
    kappaEdit = new QLineEdit(QString::number(kappa));
    processNoiseEdit = new QLineEdit(QString::number(processNoise));
    measurementNoiseEdit = new QLineEdit(QString::number(measurementNoise));

    QDoubleValidator *validator = new QDoubleValidator(this);
    validator->setLocale(QLocale::C);

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

    // Группа для параметров симуляции
    QGroupBox *simulationGroupBox = new QGroupBox("Параметры симуляции");
    QGridLayout *simulationLayout = new QGridLayout();

    noiseLevelEdit = new QLineEdit(QString::number(noiseLevel));
    noiseLevelEdit->setValidator(validator);

    simulationLayout->addWidget(new QLabel("Noise Level:"), 0, 0);
    simulationLayout->addWidget(noiseLevelEdit, 0, 1);

    simulationGroupBox->setLayout(simulationLayout);

    // Кнопки управления
    startButton = new QPushButton("Старт");
    stopButton = new QPushButton("Стоп");
    stopButton->setEnabled(false);

    connect(startButton, &QPushButton::clicked, this, &MainWindow::startSimulation);
    connect(stopButton, &QPushButton::clicked, this, &MainWindow::stopSimulation);

    // Размещение кнопок
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(startButton);
    buttonLayout->addWidget(stopButton);

    // Связываем изменения параметров с функцией onParametersChanged
    connect(alphaEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    connect(betaEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    connect(kappaEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    connect(processNoiseEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    connect(measurementNoiseEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    connect(noiseLevelEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);

    // Чекбоксы для управления отображением интенсивностей
    QGroupBox *intensityGroupBox = new QGroupBox("Отображение интенсивностей");
    QHBoxLayout *intensityLayout = new QHBoxLayout();

    QString labels[4] = {"I1", "I2", "I3", "I4"};
    for (int i = 0; i < 4; ++i) {
        intensityCheckBoxes[i] = new QCheckBox(labels[i]);
        intensityCheckBoxes[i]->setChecked(true);
        intensityLayout->addWidget(intensityCheckBoxes[i]);
        int index = i; // Захватываем переменную для использования в лямбда-функции
        connect(intensityCheckBoxes[i], &QCheckBox::toggled, [this, index](bool checked) {
            onIntensityCurveToggled(index, checked);
        });
    }

    intensityGroupBox->setLayout(intensityLayout);

    // Размещение графиков
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

    // Размещение графиков в сетке
    QGridLayout *plotsLayout = new QGridLayout();
    plotsLayout->addWidget(rawIntensityPlot, 0, 0);
    plotsLayout->addWidget(filteredIntensityPlot, 0, 1);
    plotsLayout->addWidget(estimatedCoordinatePlot, 1, 0);
    plotsLayout->addWidget(trueCoordinatePlot, 1, 1);

    // Добавляем все элементы в основной лейаут
    mainLayout->addWidget(filterGroupBox);
    mainLayout->addWidget(simulationGroupBox);
    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(intensityGroupBox);
    mainLayout->addLayout(plotsLayout);

    setCentralWidget(centralWidget);
}

void MainWindow::setupPlots() {
    // Инициализация кривых интенсивностей
    QColor colors[4] = {Qt::red, Qt::blue, Qt::green, Qt::magenta};
    for (int i = 0; i < 4; ++i) {
        // Нефильтрованные интенсивности
        rawIntensityCurves[i] = new QwtPlotCurve(QString("Raw I%1").arg(i + 1));
        rawIntensityCurves[i]->attach(rawIntensityPlot);
        rawIntensityCurves[i]->setPen(QPen(colors[i]));

        // Фильтрованные интенсивности
        filteredIntensityCurves[i] = new QwtPlotCurve(QString("Filtered I%1").arg(i + 1));
        filteredIntensityCurves[i]->attach(filteredIntensityPlot);
        filteredIntensityCurves[i]->setPen(QPen(colors[i]));
    }

    // Инициализация кривых координат
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
}

void MainWindow::startSimulation() {
    if (isRunning) {
        QMessageBox::warning(this, "Ошибка", "Симуляция уже запущена.");
        return;
    }

    // Блокируем изменение параметров
    alphaEdit->setEnabled(false);
    betaEdit->setEnabled(false);
    kappaEdit->setEnabled(false);
    processNoiseEdit->setEnabled(false);
    measurementNoiseEdit->setEnabled(false);
    noiseLevelEdit->setEnabled(false);

    // Блокируем чекбоксы
    for (int i = 0; i < 4; ++i) {
        intensityCheckBoxes[i]->setEnabled(false);
    }

    // Обновляем параметры
    onParametersChanged();

    // Инициализация фильтра и симуляции
    beamSimulation = new BeamSimulation();
    beamSimulation->setNoiseLevel(noiseLevel);

    // Получаем первое измерение для инициализации состояния
    Eigen::VectorXd initial_measurement = beamSimulation->moveBeamAndIntegrate(1.0, 1.0, 0.0, 0);

    // Инициализация состояния фильтра
    Eigen::VectorXd initial_state(6); // 4 интенсивности + x + y
    initial_state << initial_measurement(0), initial_measurement(1), initial_measurement(2), initial_measurement(3), beamSimulation->getXc(), beamSimulation->getYc();
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(6, 6) * 1.0;
    Eigen::MatrixXd process_noise_cov = Eigen::MatrixXd::Identity(6, 6) * processNoise;
    Eigen::MatrixXd measurement_noise_cov = Eigen::MatrixXd::Identity(4, 4) * measurementNoise;

    kalmanFilter = new AdaptiveUnscentedKalmanFilter(initial_state, initial_covariance, process_noise_cov, measurement_noise_cov);
    kalmanFilter->setParameters(alpha, beta, kappa);

    currentTime = 0.0;
    iteration = 0;
    initializeCurves();

    isRunning = true;
    startButton->setEnabled(false);
    stopButton->setEnabled(true);

    updateTimer->start(10); // Запускаем таймер
}

void MainWindow::stopSimulation() {
    if (!isRunning) {
        QMessageBox::warning(this, "Ошибка", "Симуляция не запущена.");
        return;
    }

    isRunning = false;
    updateTimer->stop();

    // Разблокируем изменение параметров
    alphaEdit->setEnabled(true);
    betaEdit->setEnabled(true);
    kappaEdit->setEnabled(true);
    processNoiseEdit->setEnabled(true);
    measurementNoiseEdit->setEnabled(true);
    noiseLevelEdit->setEnabled(true);

    // Разблокируем чекбоксы
    for (int i = 0; i < 4; ++i) {
        intensityCheckBoxes[i]->setEnabled(true);
    }

    startButton->setEnabled(true);
    stopButton->setEnabled(false);
}

void MainWindow::updatePlots() {
    if (!isRunning) {
        return;
    }

    currentTime += timeStep;
    timeData.append(currentTime);

    // Получение зашумленных интенсивностей
    Eigen::VectorXd measurement = beamSimulation->moveBeamAndIntegrate(1.0, 1.0, 0.0, iteration);

    // Обновление фильтра
    kalmanFilter->predict();
    kalmanFilter->update(measurement);

    // Сохранение данных интенсивностей
    for (int i = 0; i < 4; ++i) {
        rawIntensityData[i].append(measurement(i));
        filteredIntensityData[i].append(kalmanFilter->getState()(i));
    }

    // Обновление кривых интенсивностей
    for (int i = 0; i < 4; ++i) {
        rawIntensityCurves[i]->setSamples(timeData, rawIntensityData[i]);
        filteredIntensityCurves[i]->setSamples(timeData, filteredIntensityData[i]);
    }

    // Получение истинных координат
    double trueX = beamSimulation->getXc();
    double trueY = beamSimulation->getYc();
    trueXData.append(trueX);
    trueYData.append(trueY);

    // Получение оцененных координат из фильтра
    Eigen::Vector2d spot_position = kalmanFilter->calculateSpotPosition();
    double estimatedX = spot_position(0);
    double estimatedY = spot_position(1);
    estimatedXData.append(estimatedX);
    estimatedYData.append(estimatedY);

    // Обновление кривых координат
    estimatedPositionCurveX->setSamples(timeData, estimatedXData);
    estimatedPositionCurveY->setSamples(timeData, estimatedYData);

    truePositionCurveX->setSamples(timeData, trueXData);
    truePositionCurveY->setSamples(timeData, trueYData);

    // Перерисовка графиков
    rawIntensityPlot->replot();
    filteredIntensityPlot->replot();
    estimatedCoordinatePlot->replot();
    trueCoordinatePlot->replot();

    iteration++;
}

void MainWindow::onParametersChanged() {
    if (isRunning) {
        return;
    }

    // Обновление параметров на основе значений из UI
    alpha = alphaEdit->text().toDouble();
    beta = betaEdit->text().toDouble();
    kappa = kappaEdit->text().toDouble();
    processNoise = processNoiseEdit->text().toDouble();
    measurementNoise = measurementNoiseEdit->text().toDouble();
    noiseLevel = noiseLevelEdit->text().toDouble();
}

void MainWindow::onIntensityCurveToggled(int index, bool checked) {
    rawIntensityCurves[index]->setVisible(checked);
    filteredIntensityCurves[index]->setVisible(checked);
}
