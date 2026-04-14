#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QLabel>
#include <QTimer>

#include "../Config.h"
#include "../core/TrackingEngine.h"
#include "../io/DataLogger.h"
#include <Eigen/Dense>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>


class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(const Config &config, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void updatePlots();
    void onParametersChanged();
    void startSimulation();
    void stopSimulation();
    void resetSimulation();
    void onIntensityCurveToggled(int index, bool checked);

private:
    void setupUI();
    void setupPlots();
    void initializeCurves();
    void trimPlotBuffers();
    void showError(const QString &message);
    void validateInput();
    void loadRealData();

    Config config;

    std::unique_ptr<TrackingEngine> trackingEngine;
    std::unique_ptr<DataLogger> dataLogger;

    // Данные для реального режима (if mode == "realtime")
    QVector<Eigen::VectorXd> loadedMeasurements;

    // UI элементы
    QLineEdit *alphaEdit;
    QLineEdit *betaEdit;
    QLineEdit *kappaEdit;
    QLineEdit *processNoiseEdit;
    QLineEdit *measurementNoiseEdit;
    QCheckBox *outlierGatingCheck;
    QLineEdit *outlierNisThresholdEdit;
    QCheckBox *calibrationFeedbackCheck;
    QLineEdit *calibrationRateEdit;
    QLineEdit *calibrationToleranceEdit;
    QLineEdit *noiseLevelEdit;
    QLineEdit *gapSizeEdit;

    QPushButton *startButton;
    QPushButton *stopButton;
    QPushButton *resetButton;

    QwtPlot *rawIntensityPlot;
    QwtPlot *filteredIntensityPlot;
    QwtPlot *estimatedCoordinatePlot;
    QwtPlot *trueCoordinatePlot;
    QwtPlot *errorPlot;

    QwtPlotCurve *rawIntensityCurves[4];
    QwtPlotCurve *filteredIntensityCurves[4];
    QCheckBox *intensityCheckBoxes[4];

    QwtPlotCurve *estimatedPositionCurveX;
    QwtPlotCurve *estimatedPositionCurveY;
    QwtPlotCurve *truePositionCurveX;
    QwtPlotCurve *truePositionCurveY;
    QwtPlotCurve *errorCurve;

    QVector<double> timeData;
    QVector<double> rawIntensityData[4];
    QVector<double> filteredIntensityData[4];
    QVector<double> estimatedXData;
    QVector<double> estimatedYData;
    QVector<double> truthTimeData;
    QVector<double> trueXData;
    QVector<double> trueYData;
    QVector<double> errorTimeData;
    QVector<double> errorData;

    QLabel *totalErrorLabel;
    QLabel *recentErrorLabel;
    QLabel *nisLabel;
    QLabel *acceptanceLabel;
    QLabel *calibrationStatusLabel;
    QLabel *calibrationGainsLabel;
    QLabel *hardwareFeedbackLabel;

    QTimer *updateTimer;
    int iteration;
    bool isRunning;
};

#endif // MAINWINDOW_H
