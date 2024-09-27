// MainWindow.cpp

#include "MainWindow.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QHBoxLayout>
#include <QLabel>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), currentTime(0) {
    // Инициализация фильтра и симуляции
    int n = 4;
    Eigen::VectorXd initial_state(n);
    initial_state << 0.2, 0.2, 0.3, 0.3;
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(n, n) * 3.795;
    Eigen::MatrixXd process_noise_cov = Eigen::MatrixXd::Identity(n, n) * 7.698;
    Eigen::MatrixXd measurement_noise_cov = Eigen::MatrixXd::Identity(n, n) * 0.5978;

    filter = new AdaptiveUnscentedKalmanFilter(
        initial_state,
        initial_covariance,
        process_noise_cov,
        measurement_noise_cov
    );

    simulation = new BeamSimulation(1.0, 1.0, -1.0, 0.0);

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
    intensityPlot = new QwtPlot("Quadrant Intensities");
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

    statePlot = new QwtPlot("Filtered States");
    statePlot->setAxisTitle(QwtPlot::xBottom, "Time");
    statePlot->setAxisTitle(QwtPlot::yLeft, "State Values");

    curveState1 = new QwtPlotCurve("State 1");
    curveState2 = new QwtPlotCurve("State 2");
    curveState3 = new QwtPlotCurve("State 3");
    curveState4 = new QwtPlotCurve("State 4");

    curveState1->setPen(Qt::red);
    curveState2->setPen(Qt::green);
    curveState3->setPen(Qt::blue);
    curveState4->setPen(Qt::magenta);

    curveState1->attach(statePlot);
    curveState2->attach(statePlot);
    curveState3->attach(statePlot);
    curveState4->attach(statePlot);

    mainLayout->addWidget(intensityPlot);
    mainLayout->addWidget(statePlot);

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

    state1Data.append(state(0));
    state2Data.append(state(1));
    state3Data.append(state(2));
    state4Data.append(state(3));

    // Обновление графиков
    curveIA->setSamples(timeData, IAData);
    curveIB->setSamples(timeData, IBData);
    curveIC->setSamples(timeData, ICData);
    curveID->setSamples(timeData, IDData);

    curveState1->setSamples(timeData, state1Data);
    curveState2->setSamples(timeData, state2Data);
    curveState3->setSamples(timeData, state3Data);
    curveState4->setSamples(timeData, state4Data);

    intensityPlot->replot();
    statePlot->replot();

    currentTime++;
}
