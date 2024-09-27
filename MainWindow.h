// MainWindow.h

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "AdaptiveUnscentedKalmanFilter.h"
#include "BeamSimulation.h"
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <QTimer>
#include <QVector>

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);

private slots:
    void startSimulation();
    void stopSimulation();
    void updatePlots();

private:
    AdaptiveUnscentedKalmanFilter *filter;
    BeamSimulation *simulation;

    QTimer *timer;

    // Графики
    QwtPlot *intensityPlot;
    QwtPlotCurve *curveIA;
    QwtPlotCurve *curveIB;
    QwtPlotCurve *curveIC;
    QwtPlotCurve *curveID;

    QwtPlot *statePlot;
    QwtPlotCurve *curveState1;
    QwtPlotCurve *curveState2;
    QwtPlotCurve *curveState3;
    QwtPlotCurve *curveState4;

    QVector<double> timeData;
    QVector<double> IAData, IBData, ICData, IDData;
    QVector<double> state1Data, state2Data, state3Data, state4Data;

    int currentTime;
};

#endif // MAINWINDOW_H
