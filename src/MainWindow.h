#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <QTimer>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "AdaptiveUnscentedKalmanFilter.h"
#include "BeamSimulation.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);

private slots:
    void updatePlots();
    void onParametersChanged();
    void startSimulation();
    void stopSimulation();

private:
    // Параметры
    double alpha;
    double beta;
    double kappa;
    double processNoise;
    double measurementNoise;
    double noiseLevel;

    // UI элементы для настройки параметров
    QLineEdit *alphaEdit;
    QLineEdit *betaEdit;
    QLineEdit *kappaEdit;
    QLineEdit *processNoiseEdit;
    QLineEdit *measurementNoiseEdit;
    QLineEdit *noiseLevelEdit;

    // Кнопки управления
    QPushButton *startButton;
    QPushButton *stopButton;

    // Плоты
    QwtPlot *rawIntensityPlot;
    QwtPlot *filteredIntensityPlot;
    QwtPlot *estimatedCoordinatePlot;
    QwtPlot *trueCoordinatePlot;

    // Кривые интенсивностей
    QwtPlotCurve *rawIntensityCurves[4];
    QwtPlotCurve *filteredIntensityCurves[4];

    // Кривые координат
    QwtPlotCurve *estimatedPositionCurveX;
    QwtPlotCurve *estimatedPositionCurveY;
    QwtPlotCurve *truePositionCurveX;
    QwtPlotCurve *truePositionCurveY;

    // Данные для графиков
    QVector<double> timeData;
    QVector<double> rawIntensityData[4];
    QVector<double> filteredIntensityData[4];
    QVector<double> estimatedXData;
    QVector<double> estimatedYData;
    QVector<double> trueXData;
    QVector<double> trueYData;

    // Таймер для обновления
    QTimer *updateTimer;
    double currentTime;
    double timeStep;

    // Итерация
    int iteration;

    // Состояние симуляции
    bool isRunning;

    // Экземпляры классов
    AdaptiveUnscentedKalmanFilter *kalmanFilter;
    BeamSimulation *beamSimulation;

    void setupPlots();
    void setupUI();
    void initializeCurves();
    void resetSimulation();
};

#endif // MAINWINDOW_H
