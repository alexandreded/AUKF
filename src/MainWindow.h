#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <QTimer>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox> // Добавлено
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
    void updatePlots();                                 // Обновление графиков
    void onParametersChanged();                         // Обработчик изменения параметров
    void startSimulation();                             // Старт симуляции
    void stopSimulation();                              // Остановка симуляции
    void onIntensityCurveToggled(int index, bool checked); // Добавлено

private:
    // Параметры фильтра и шума
    double alpha;
    double beta;
    double kappa;
    double processNoise;
    double measurementNoise;
    double noiseLevel;

    // UI элементы для настройки параметров фильтра и шума
    QLineEdit *alphaEdit;
    QLineEdit *betaEdit;
    QLineEdit *kappaEdit;
    QLineEdit *processNoiseEdit;
    QLineEdit *measurementNoiseEdit;
    QLineEdit *noiseLevelEdit;

    // Кнопки управления
    QPushButton *startButton;
    QPushButton *stopButton;

    // Графики для отображения данных
    QwtPlot *rawIntensityPlot;            // График необработанных интенсивностей
    QwtPlot *filteredIntensityPlot;       // График фильтрованных интенсивностей
    QwtPlot *estimatedCoordinatePlot;     // График оцененных координат
    QwtPlot *trueCoordinatePlot;          // График истинных координат

    // Кривые интенсивностей
    QwtPlotCurve *rawIntensityCurves[4];      // Кривые для 4 источников необработанных интенсивностей
    QwtPlotCurve *filteredIntensityCurves[4]; // Кривые для 4 источников фильтрованных интенсивностей

    // Чекбоксы для управления отображением интенсивностей
    QCheckBox *intensityCheckBoxes[4];        // Добавлено

    // Кривые координат
    QwtPlotCurve *estimatedPositionCurveX; // Кривая для X координаты (оценка)
    QwtPlotCurve *estimatedPositionCurveY; // Кривая для Y координаты (оценка)
    QwtPlotCurve *truePositionCurveX;      // Кривая для X координаты (истинное значение)
    QwtPlotCurve *truePositionCurveY;      // Кривая для Y координаты (истинное значение)

    // Данные для графиков
    QVector<double> timeData;              // Временные метки
    QVector<double> rawIntensityData[4];   // Необработанные данные интенсивностей
    QVector<double> filteredIntensityData[4]; // Фильтрованные данные интенсивностей
    QVector<double> estimatedXData;        // Оцененная координата X
    QVector<double> estimatedYData;        // Оцененная координата Y
    QVector<double> trueXData;             // Истинная координата X
    QVector<double> trueYData;             // Истинная координата Y

    // Таймер для обновления графиков
    QTimer *updateTimer;
    double currentTime;                    // Текущее время
    double timeStep;                       // Шаг времени

    // Переменные для итерации и управления состоянием симуляции
    int iteration;
    bool isRunning;

    // Экземпляры классов для фильтра и симуляции
    AdaptiveUnscentedKalmanFilter *kalmanFilter;
    BeamSimulation *beamSimulation;

    // Методы для настройки графиков, UI и инициализации
    void setupPlots();                     // Настройка графиков
    void setupUI();                        // Настройка интерфейса пользователя
    void initializeCurves();               // Инициализация кривых на графиках
    void resetSimulation();                // Сброс параметров симуляции
};

#endif // MAINWINDOW_H
