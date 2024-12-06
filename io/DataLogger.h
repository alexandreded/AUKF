#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <QFile>
#include <QTextStream>
#include <QJsonArray>
#include <Eigen/Dense>
#include <QString>

class DataLogger {
public:
    DataLogger(const QString &logFilePath, const QString &jsonFilePath);
    ~DataLogger();

    void logMessage(const QString &message);
    void addRecord(double time,
                   const Eigen::VectorXd &rawIntensities,
                   const Eigen::VectorXd &filteredIntensities,
                   double trueX, double trueY,
                   double estX, double estY,
                   double error);

private:
    void writeJsonData();

    QFile logFile;
    QTextStream logStream;
    QFile jsonFile;
    QJsonArray jsonArray;
};

#endif // DATA_LOGGER_H
