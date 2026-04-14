#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <QFile>
#include <QTextStream>
#include <QJsonArray>
#include <QJsonObject>
#include <Eigen/Dense>
#include <QString>

class DataLogger {
public:
    DataLogger(const QString &logFilePath, const QString &jsonFilePath);
    ~DataLogger();

    void setMetadata(const QJsonObject &metadataObject);
    void logMessage(const QString &message);
    void addRecord(double time,
                   const Eigen::VectorXd &rawIntensities,
                   const Eigen::VectorXd &calibratedIntensities,
                   const Eigen::VectorXd &filteredIntensities,
                   double trueX, double trueY,
                   double estX, double estY,
                   double error,
                   bool calibrationEnabled = false,
                   bool calibrationConverged = false,
                   double calibrationErrorX = 0.0,
                   double calibrationErrorY = 0.0,
                   const Eigen::Vector4d &calibrationGains = Eigen::Vector4d::Ones(),
                   bool hardwareFeedbackEnabled = false,
                   bool hardwareFeedbackApplied = false,
                   const Eigen::Vector4d &hardwareFrequenciesMHz = Eigen::Vector4d::Zero(),
                   const Eigen::Vector4d &hardwareAmplitudes = Eigen::Vector4d::Zero());

private:
    void writeJsonData();

    QFile logFile;
    QTextStream logStream;
    QFile jsonFile;
    QJsonObject metadata;
    QJsonArray jsonArray;
    bool logReady = false;
    bool jsonReady = false;
};

#endif // DATA_LOGGER_H
