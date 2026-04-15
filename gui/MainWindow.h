#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QLabel>
#include <QTimer>
#include <QMutex>
#include <QThread>

#include "../Config.h"
#include "../core/TrackingEngine.h"
#include "../hardware/BoardDriver.h"
#include "../io/DataLogger.h"
#include <Eigen/Dense>
#include <deque>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>

class HardwareInputThread final : public QThread {
public:
    explicit HardwareInputThread(TrackingEngine *engine, QObject *parent = nullptr);
    ~HardwareInputThread() override;

    void requestStop();
    bool popResult(TrackingStepResult &result);
    bool takeError(QString &errorMessage);
    bool isWorkerFinished() const;

protected:
    void run() override;

private:
    TrackingEngine *engine = nullptr;
    mutable QMutex mutex;
    std::deque<TrackingStepResult> resultQueue;
    bool stopRequested = false;
    bool finished = false;
    QString fatalError;
};

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
    void connectHardwareDevice();
    void disconnectHardwareDevice();

private:
    void setupUI();
    void setupPlots();
    void initializeCurves();
    void trimPlotBuffers();
    void processStepResult(const TrackingStepResult &stepResult);
    void showError(const QString &message);
    void validateInput();
    void loadRealData();
    void setControlsEnabled(bool enabled);
    void updateHardwareDeviceStatus();

    Config config;

    std::unique_ptr<TrackingEngine> trackingEngine;
    std::unique_ptr<DataLogger> dataLogger;
    std::unique_ptr<BoardDriver> manualBoardDriver;
    std::unique_ptr<HardwareInputThread> hardwareInputThread;

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
    QLineEdit *hardwareSerialEdit;
    QLineEdit *hardwareFrequencyEdits[4];
    QLineEdit *hardwareAmplitudeEdits[4];
    QPushButton *hardwareConnectButton;
    QPushButton *hardwareDisconnectButton;
    QLabel *hardwareDeviceStatusLabel;

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
    bool threadedHardwareInput = false;
};

#endif // MAINWINDOW_H
