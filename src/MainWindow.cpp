// MainWindow.cpp

#include "MainWindow.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QHBoxLayout>
#include <QLabel>
#include <qwt_plot.h>
#include <qwt_text.h>


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), currentTime(0) {
    // Инициализация фильтра и симуляции
    int n_x = 2; // Размерность состояния (координаты x и y)
    int n_z = 4; // Размерность измерения (4 интенсивности)

    Eigen::VectorXd initial_state(n_x);
    initial_state << 0.0, 0.0;
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(n_x, n_x) * 1.0;
    Eigen::MatrixXd process_noise_cov = Eigen::MatrixXd::Identity(n_x, n_x) * 0.01;
    Eigen::MatrixXd measurement_noise_cov = Eigen::MatrixXd::Identity(n_z, n_z) * 0.1;

    // Определение функций перехода и измерения
    AdaptiveUnscentedKalmanFilter::StateTransitionFunction f = [](const Eigen::VectorXd& x) {
        return x; // Предполагаем, что состояние не меняется (постоянная скорость)
    };

    AdaptiveUnscentedKalmanFilter::MeasurementFunction h = [](const Eigen::VectorXd& x) {
        double P0 = 1.0;
        double w = 1.0;

        double x_c = x(0);
        double y_c = x(1);

        double I_A = 2 * P0 / (M_PI * w * w) * std::exp(-2 * ((1 - x_c) * (1 - x_c) + (1 - y_c) * (1 - y_c)) / (w * w));
        double I_B = 2 * P0 / (M_PI * w * w) * std::exp(-2 * ((-1 - x_c) * (-1 - x_c) + (1 - y_c) * (1 - y_c)) / (w * w));
        double I_C = 2 * P0 / (M_PI * w * w) * std::exp(-2 * ((-1 - x_c) * (-1 - x_c) + (-1 - y_c) * (-1 - y_c)) / (w * w));
        double I_D = 2 * P0 / (M_PI * w * w) * std::exp(-2 * ((1 - x_c) * (1 - x_c) + (-1 - y_c) * (-1 - y_c)) / (w * w));

        Eigen::VectorXd z(4);
        z << I_A, I_B, I_C, I_D;
        return z;
    };

    filter = std::make_unique<AdaptiveUnscentedKalmanFilter>(
        initial_state,
        initial_covariance,
        process_noise_cov,
        measurement_noise_cov,
        f,
        h
    );

    simulation = std::make_unique<BeamSimulation>(1.0, 1.0, -1.0, 0.0);

    // Создание интерфейса
    QWidget *centralWidget = new QWidget();
    QVBoxLayout *mainLayout = new QVBoxLayout();

    // Кнопки управления
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    QPushButton *startButton = new QPushButton("Start Simulation");
    QPushButton *stopButton = new QPushButton("Stop Simulation");

    connect(startButton, &QPushButton::clicked, this, &MainWindow::startSimulation);
    connect(stopButton, &QPushButton::clicked, this, &MainWindow::stopSimulation);

    buttonLayout->addWidget(startButton);
    buttonLayout->addWidget(stopButton);

    mainLayout->addLayout(buttonLayout);

    // Создание графиков
    intensityPlot = new QwtPlot(QwtText("Quadrant Intensities"));
    intensityPlot->setAxisTitle(QwtPlot::xBottom, "Time");
    intensityPlot->setAxisTitle(QwtPlot::yLeft, "Intensity");

    curveIA = new QwtPlotCurve("I_A");
    curveIB = new QwtPlotCurve("I_B");
    curveIC = new QwtPlotCurve("I_C");
    curveID = new QwtPlotCurve("I_D");

    curveIA->setPen(Qt::red);
    curveIB->setPen(Qt::green);
    curveIC->setPen(Qt::blue);
    curveID->setPen(Qt::magenta);

    curveIA->attach(intensityPlot);
    curveIB->attach(intensityPlot);
    curveIC->attach(intensityPlot);
    curveID->attach(intensityPlot);

    coordinatePlot = new QwtPlot(QwtText("Beam Position"));
    coordinatePlot->setAxisTitle(QwtPlot::xBottom, "Time");
    coordinatePlot->setAxisTitle(QwtPlot::yLeft, "X Coordinate");

    curveRealX = new QwtPlotCurve("Real X");
    curveEstimatedX = new QwtPlotCurve("Estimated X");

    curveRealX->setPen(Qt::blue);
    curveEstimatedX->setPen(Qt::red);

    curveRealX->attach(coordinatePlot);
    curveEstimatedX->attach(coordinatePlot);

    mainLayout->addWidget(intensityPlot);
    mainLayout->addWidget(coordinatePlot);

    centralWidget->setLayout(mainLayout);
    setCentralWidget(centralWidget);

    // Таймер для обновления
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updatePlots);
}

void MainWindow::startSimulation() {
    timer->start(50); // Обновление каждые 50 мс
}

void MainWindow::stopSimulation() {
    timer->stop();
}

void MainWindow::updatePlots() {
    // Генерация измерений
    Eigen::VectorXd measurement = simulation->generateMeasurement(currentTime);

    // Применение фильтра
    filter->predict();
    filter->update(measurement);

    // Получение данных
    Eigen::VectorXd state = filter->getState();

    // Обновление данных для графиков
    timeData.append(currentTime);

    IAData.append(measurement(0));
    IBData.append(measurement(1));
    ICData.append(measurement(2));
    IDData.append(measurement(3));

    double realX = simulation->getXc();
    double estimatedX = state(0);

    realXData.append(realX);
    estimatedXData.append(estimatedX);

    // Ограничение размера данных
    const int maxPoints = 1000;
    if (timeData.size() > maxPoints) {
        timeData.remove(0);
        IAData.remove(0);
        IBData.remove(0);
        ICData.remove(0);
        IDData.remove(0);
        realXData.remove(0);
        estimatedXData.remove(0);
    }

    // Обновление графиков
    curveIA->setSamples(timeData, IAData);
    curveIB->setSamples(timeData, IBData);
    curveIC->setSamples(timeData, ICData);
    curveID->setSamples(timeData, IDData);

    curveRealX->setSamples(timeData, realXData);
    curveEstimatedX->setSamples(timeData, estimatedXData);

    intensityPlot->replot();
    coordinatePlot->replot();

    currentTime++;
}
