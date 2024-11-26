#include "MainWindow.h"
// Удаляем неиспользуемые include
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QVector>
#include <QDoubleValidator>
#include <QMessageBox>
#include <cmath>
#include <algorithm>
#include <QGuiApplication>
#include <QScreen>
#include <QLocale>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_legend.h>
#include <stdexcept>
#include <QDebug>


// Конструктор MainWindow
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      alpha(1.0),
      beta(2.0),
      kappa(0.0),
      processNoise(1.0),
      measurementNoise(1.0),
      noiseLevel(0.1),
      gapSize(0.0),
      currentTime(0.0),
      timeStep(0.01),
      iteration(0),
      isRunning(false),
      kalmanFilter(nullptr),
      beamSimulation(nullptr)
{
    setupUI();
    setupPlots();

    // Настройка таймера для обновления графиков
    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::updatePlots);
}

// Метод настройки интерфейса пользователя
void MainWindow::setupUI() {
    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);

    // Группа для параметров фильтра
    QGroupBox *filterGroupBox = new QGroupBox("Параметры фильтра");
    QGridLayout *filterLayout = new QGridLayout();

    // Создаем валидатор для числовых вводов
    QDoubleValidator *validator = new QDoubleValidator(this);
    validator->setNotation(QDoubleValidator::StandardNotation);
    validator->setBottom(0.0); // Пример ограничения
    validator->setLocale(QLocale::C); // Устанавливаем локаль для ввода через точку

    // Инициализация полей ввода
    alphaEdit = new QLineEdit(QString::number(alpha));
    alphaEdit->setToolTip("Параметр Alpha определяет разброс сигма-точек вокруг среднего значения.");
    betaEdit = new QLineEdit(QString::number(beta));
    betaEdit->setToolTip("Параметр Beta использует информацию о распределении (обычно 2 для гауссовского).");
    kappaEdit = new QLineEdit(QString::number(kappa));
    kappaEdit->setToolTip("Параметр Kappa настраивает разброс сигма-точек (часто равен 0).");
    processNoiseEdit = new QLineEdit(QString::number(processNoise));
    processNoiseEdit->setToolTip("Ковариация шума процесса (Q).");
    measurementNoiseEdit = new QLineEdit(QString::number(measurementNoise));
    measurementNoiseEdit->setToolTip("Ковариация шума измерения (R).");

    // Установка валидатора
    alphaEdit->setValidator(validator);
    betaEdit->setValidator(validator);
    kappaEdit->setValidator(validator);
    processNoiseEdit->setValidator(validator);
    measurementNoiseEdit->setValidator(validator);

    // Размещение элементов в сетке
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
    noiseLevelEdit->setToolTip("Уровень шума, добавляемый к интенсивностям.");

    gapSizeEdit = new QLineEdit(QString::number(gapSize));
    gapSizeEdit->setValidator(validator);
    gapSizeEdit->setToolTip("Размер щели между детекторами (от 0 до 1).");

    simulationLayout->addWidget(new QLabel("Noise Level:"), 0, 0);
    simulationLayout->addWidget(noiseLevelEdit, 0, 1);
    simulationLayout->addWidget(new QLabel("Gap Size:"), 1, 0);
    simulationLayout->addWidget(gapSizeEdit, 1, 1);

    simulationGroupBox->setLayout(simulationLayout);

    // Кнопки управления
    startButton = new QPushButton("Старт");
    stopButton = new QPushButton("Стоп");
    resetButton = new QPushButton("Сброс");
    stopButton->setEnabled(false);
    resetButton->setEnabled(false);

    connect(startButton, &QPushButton::clicked, this, &MainWindow::startSimulation);
    connect(stopButton, &QPushButton::clicked, this, &MainWindow::stopSimulation);
    connect(resetButton, &QPushButton::clicked, this, &MainWindow::resetSimulation);

    // Размещение кнопок
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(startButton);
    buttonLayout->addWidget(stopButton);
    buttonLayout->addWidget(resetButton);

    // Связываем изменения параметров с функцией onParametersChanged
    connect(alphaEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    connect(betaEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    connect(kappaEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    connect(processNoiseEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    connect(measurementNoiseEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    connect(noiseLevelEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);
    connect(gapSizeEdit, &QLineEdit::editingFinished, this, &MainWindow::onParametersChanged);

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

    // Метки для отображения ошибок
    totalErrorLabel = new QLabel("Средняя ошибка за всё время: 0.0");
    recentErrorLabel = new QLabel("Средняя ошибка за последние 3 секунды: 0.0");

    QVBoxLayout *errorLabelsLayout = new QVBoxLayout();
    errorLabelsLayout->addWidget(totalErrorLabel);
    errorLabelsLayout->addWidget(recentErrorLabel);

    QGroupBox *errorGroupBox = new QGroupBox("Ошибки");
    errorGroupBox->setLayout(errorLabelsLayout);

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

    errorPlot = new QwtPlot(this);
    errorPlot->setTitle("Ошибка координат");
    errorPlot->setAxisTitle(QwtPlot::xBottom, "Время");
    errorPlot->setAxisTitle(QwtPlot::yLeft, "Ошибка");

    // Добавляем инструменты масштабирования и панорамирования
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

    // Добавляем легенды к графикам
    rawIntensityPlot->insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
    filteredIntensityPlot->insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
    estimatedCoordinatePlot->insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
    trueCoordinatePlot->insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
    errorPlot->insertLegend(new QwtLegend(), QwtPlot::BottomLegend);

    // Размещение графиков в сетке
    QGridLayout *plotsLayout = new QGridLayout();
    plotsLayout->addWidget(rawIntensityPlot, 0, 0);
    plotsLayout->addWidget(filteredIntensityPlot, 0, 1);
    plotsLayout->addWidget(estimatedCoordinatePlot, 1, 0);
    plotsLayout->addWidget(trueCoordinatePlot, 1, 1);
    plotsLayout->addWidget(errorPlot, 2, 0, 1, 2);

    // Добавляем все элементы в основной лейаут
    mainLayout->addWidget(filterGroupBox);
    mainLayout->addWidget(simulationGroupBox);
    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(intensityGroupBox);
    mainLayout->addWidget(errorGroupBox);
    mainLayout->addLayout(plotsLayout);

    setCentralWidget(centralWidget);

    // Установка минимального размера окна
    setMinimumSize(800, 600);

    // Масштабирование окна до 80% от размера экрана
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
    int height = screenGeometry.height();
    int width = screenGeometry.width();
    resize(width * 0.8, height * 0.8);
}

// Метод настройки графиков
void MainWindow::setupPlots() {
    // Инициализация кривых интенсивностей
    QColor colors[4] = {Qt::red, Qt::blue, Qt::green, Qt::magenta};
    for (int i = 0; i < 4; ++i) {
        // Нефильтрованные интенсивности
        rawIntensityCurves[i] = new QwtPlotCurve(QString("Raw I%1").arg(i + 1));
        rawIntensityCurves[i]->attach(rawIntensityPlot);
        rawIntensityCurves[i]->setPen(QPen(colors[i]));
        rawIntensityCurves[i]->setVisible(intensityCheckBoxes[i]->isChecked());

        // Фильтрованные интенсивности
        filteredIntensityCurves[i] = new QwtPlotCurve(QString("Filtered I%1").arg(i + 1));
        filteredIntensityCurves[i]->attach(filteredIntensityPlot);
        filteredIntensityCurves[i]->setPen(QPen(colors[i]));
        filteredIntensityCurves[i]->setVisible(intensityCheckBoxes[i]->isChecked());
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

    // Инициализация кривой ошибки
    errorCurve = new QwtPlotCurve("Error");
    errorCurve->attach(errorPlot);
    errorCurve->setPen(QPen(Qt::red));

    estimatedCoordinatePlot->setAxisScale(QwtPlot::yLeft, -0.75, 0.75);
}

// Метод инициализации данных кривых
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

// Метод запуска симуляции
void MainWindow::startSimulation() {
    if (isRunning) {
        QMessageBox::warning(this, "Ошибка", "Симуляция уже запущена.");
        return;
    }

    // Валидация пользовательского ввода
    try {
        validateInput();
    } catch (const std::exception &e) {
        showError(e.what());
        return;
    }

    // Блокируем изменение параметров
    alphaEdit->setEnabled(false);
    betaEdit->setEnabled(false);
    kappaEdit->setEnabled(false);
    processNoiseEdit->setEnabled(false);
    measurementNoiseEdit->setEnabled(false);
    noiseLevelEdit->setEnabled(false);
    gapSizeEdit->setEnabled(false);

    // Блокируем чекбоксы
    for (int i = 0; i < 4; ++i) {
        intensityCheckBoxes[i]->setEnabled(false);
    }

    // Обновляем параметры
    onParametersChanged();

    // Инициализация фильтра и симуляции
    beamSimulation = new BeamSimulation();
    beamSimulation->setNoiseLevel(noiseLevel);
    beamSimulation->setGapSize(gapSize); // Устанавливаем размер щели

    // Получаем первое измерение для инициализации состояния
    Eigen::VectorXd initial_measurement = beamSimulation->moveBeamAndIntegrate(10.0, 1.0);

    // Инициализация состояния фильтра
    Eigen::VectorXd initial_state(4); // Только 4 интенсивности
    initial_state << initial_measurement(0), initial_measurement(1), initial_measurement(2), initial_measurement(3);
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(4, 4) * 1.0;
    Eigen::MatrixXd process_noise_cov = Eigen::MatrixXd::Identity(4, 4) * processNoise;
    Eigen::MatrixXd measurement_noise_cov = Eigen::MatrixXd::Identity(4, 4) * measurementNoise;

    kalmanFilter = new AdaptiveUnscentedKalmanFilter(initial_state, initial_covariance, process_noise_cov, measurement_noise_cov);
    kalmanFilter->setParameters(alpha, beta, kappa);

    currentTime = 0.0;
    iteration = 0;
    initializeCurves();

    isRunning = true;
    startButton->setEnabled(false);
    stopButton->setEnabled(true);
    resetButton->setEnabled(true);

    updateTimer->start(10); // Запускаем таймер (каждые 10 мс)
}

// Метод остановки симуляции
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
    gapSizeEdit->setEnabled(true);

    // Разблокируем чекбоксы
    for (int i = 0; i < 4; ++i) {
        intensityCheckBoxes[i]->setEnabled(true);
    }

    startButton->setEnabled(true);
    stopButton->setEnabled(false);
}

// Метод сброса симуляции
void MainWindow::resetSimulation() {
    if (isRunning) {
        stopSimulation();
    }
    // Обнуляем данные и графики
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

// Метод отображения ошибки пользователю
void MainWindow::showError(const QString &message) {
    QMessageBox::critical(this, "Ошибка", message);
}

// Метод валидации пользовательского ввода
void MainWindow::validateInput() {
    bool ok;
    alpha = alphaEdit->text().toDouble(&ok);
    if (!ok || alpha <= 0) {
        throw std::runtime_error("Некорректное значение для Alpha.");
    }

    beta = betaEdit->text().toDouble(&ok);
    if (!ok || beta <= 0) {
        throw std::runtime_error("Некорректное значение для Beta.");
    }

    kappa = kappaEdit->text().toDouble(&ok);
    if (!ok) {
        throw std::runtime_error("Некорректное значение для Kappa.");
    }

    processNoise = processNoiseEdit->text().toDouble(&ok);
    if (!ok || processNoise < 0) {
        throw std::runtime_error("Некорректное значение для Process Noise.");
    }

    measurementNoise = measurementNoiseEdit->text().toDouble(&ok);
    if (!ok || measurementNoise < 0) {
        throw std::runtime_error("Некорректное значение для Measurement Noise.");
    }

    noiseLevel = noiseLevelEdit->text().toDouble(&ok);
    if (!ok || noiseLevel < 0) {
        throw std::runtime_error("Некорректное значение для Noise Level.");
    }

    gapSize = gapSizeEdit->text().toDouble(&ok);
    if (!ok || gapSize < 0.0 || gapSize >= 1.0) {
        throw std::runtime_error("Некорректное значение для Gap Size. Оно должно быть в диапазоне от 0 до 1.");
    }
}

// Метод обновления графиков
void MainWindow::updatePlots() {
    if (!isRunning) {
        return;
    }

    currentTime += timeStep;
    timeData.append(currentTime);

    // Получение зашумленных интенсивностей
    Eigen::VectorXd measurement = beamSimulation->moveBeamAndIntegrate(1.0, 1.0);

    // Проверка измерений
    if (measurement.hasNaN()) {
        qDebug() << "Measurement contains NaN!";
        iteration++;
        return;
    }

    // Обновление фильтра
    kalmanFilter->predict();
    kalmanFilter->update(measurement);

    // Проверка состояния фильтра
    Eigen::VectorXd currentState = kalmanFilter->getState();
    if (currentState.hasNaN()) {
        qDebug() << "Kalman Filter state contains NaN!";
        iteration++;
        return;
    }

    // Сохранение данных интенсивностей
    for (int i = 0; i < 4; ++i) {
        rawIntensityData[i].append(measurement(i));
        filteredIntensityData[i].append(kalmanFilter->getState()(i));
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

    // Проверка на NaN или Infinity
    bool invalidCoordinates = false;
    if (std::isnan(estimatedX) || std::isnan(estimatedY) ||
        std::isnan(trueX) || std::isnan(trueY)) {
        qDebug() << "One of the coordinates is NaN!";
        invalidCoordinates = true;
    }

    if (std::isinf(estimatedX) || std::isinf(estimatedY) ||
        std::isinf(trueX) || std::isinf(trueY)) {
        qDebug() << "One of the coordinates is Infinity!";
        invalidCoordinates = true;
    }

    // Вычисление ошибки только если все значения корректны
    double error = 0.0;
    if (!invalidCoordinates) {
        error = std::sqrt(std::pow(estimatedX - trueX, 2) + std::pow(estimatedY - trueY, 2));
        errorData.append(error);
    } else {
        errorData.append(0.0); // Или любое другое значение по умолчанию
    }

    // Обновление кривых интенсивностей
    for (int i = 0; i < 4; ++i) {
        rawIntensityCurves[i]->setSamples(timeData, rawIntensityData[i]);
        filteredIntensityCurves[i]->setSamples(timeData, filteredIntensityData[i]);
    }

    // Обновление кривых координат
    estimatedPositionCurveX->setSamples(timeData, estimatedXData);
    estimatedPositionCurveY->setSamples(timeData, estimatedYData);

    truePositionCurveX->setSamples(timeData, trueXData);
    truePositionCurveY->setSamples(timeData, trueYData);

    // Обновление кривой ошибки
    errorCurve->setSamples(timeData, errorData);

    // Вычисление средней ошибки за всё время
    double totalError = 0.0;
    for (double e : errorData) {
        totalError += e;
    }
    if (!errorData.isEmpty()) {
        totalError /= errorData.size();
    }
    totalErrorLabel->setText(QString("Средняя ошибка за всё время: %1").arg(totalError));

    // Вычисление средней ошибки за последние 3 секунды
    double recentError = 0.0;
    int recentCount = 0;
    for (int i = errorData.size() - 1; i >= 0; --i) {
        if (timeData.last() - timeData[i] <= 3.0) {
            recentError += errorData[i];
            recentCount++;
        } else {
            break;
        }
    }
    if (recentCount > 0) {
        recentError /= recentCount;
        recentErrorLabel->setText(QString("Средняя ошибка за последние 3 секунды: %1").arg(recentError));
    }

    // Список всех графиков
    QList<QwtPlot*> plots = { rawIntensityPlot, filteredIntensityPlot,
                              estimatedCoordinatePlot, trueCoordinatePlot, errorPlot };

    // Для каждого графика вычисляем минимальные и максимальные значения
    for (QwtPlot* plot : plots) {
        // Определяем диапазон X
        double minX = *std::min_element(timeData.begin(), timeData.end());
        double maxX = *std::max_element(timeData.begin(), timeData.end());

        // Инициализируем диапазоны Y
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::lowest();

        // Получаем все кривые, прикрепленные к текущему графику
        QList<QwtPlotItem*> attachedItems = plot->itemList(QwtPlotItem::Rtti_PlotCurve);
        QList<QwtPlotCurve*> curves;
        for (QwtPlotItem* item : attachedItems) {
            QwtPlotCurve* curve = dynamic_cast<QwtPlotCurve*>(item);
            if (curve && curve->isVisible()) {
                curves.append(curve);
            }
        }

        // Находим минимальные и максимальные Y среди всех видимых кривых
        for (QwtPlotCurve* curve : curves) {
            const QwtSeriesData<QPointF>* data = curve->data();
            for (int i = 0; i < data->size(); ++i) {
                double y = data->sample(i).y();
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
        }

        // Если нет видимых кривых, пропускаем настройку осей
        if (curves.isEmpty()) {
            continue;
        }

        // Добавляем отступы к диапазону Y
        double yMargin = (maxY - minY) * 0.1; // 10% от диапазона
        if (yMargin == 0) { // Если все Y равны, добавляем фиксированный отступ
            yMargin = 1.0;
        }

        minY -= yMargin;
        maxY += yMargin;

        // Устанавливаем диапазон осей
        plot->setAxisScale(QwtPlot::xBottom, minX, maxX);
        plot->setAxisScale(QwtPlot::yLeft, minY, maxY);

        // Обновляем оси
        plot->updateAxes();
    }

    // Перерисовка графиков
    for (QwtPlot* plot : plots) {
        plot->replot();
    }

    iteration++;
}

// Метод обработки изменения параметров
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
    gapSize = gapSizeEdit->text().toDouble();
}

// Метод управления отображением интенсивностей
void MainWindow::onIntensityCurveToggled(int index, bool checked) {
    if(index >=0 && index <4){
        rawIntensityCurves[index]->setVisible(checked);
        filteredIntensityCurves[index]->setVisible(checked);
        rawIntensityPlot->replot();
        filteredIntensityPlot->replot();
    }
}
