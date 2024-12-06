#include "DataLogger.h"
#include <QJsonDocument>
#include <QJsonObject>

DataLogger::DataLogger(const QString &logFilePath, const QString &jsonFilePath)
    : logFile(logFilePath), jsonFile(jsonFilePath)
{
    if (!logFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        // Можно логировать ошибку или бросить исключение
    }
    logStream.setDevice(&logFile);

    if (!jsonFile.open(QIODevice::WriteOnly)) {
        // Аналогично обработка ошибки
    }
}

DataLogger::~DataLogger() {
    writeJsonData();
}

void DataLogger::logMessage(const QString &message) {
    logStream << message << "\n";
}

void DataLogger::addRecord(double time,
                           const Eigen::VectorXd &rawIntensities,
                           const Eigen::VectorXd &filteredIntensities,
                           double trueX, double trueY,
                           double estX, double estY,
                           double error)
{
    QJsonObject record;
    record["time"] = time;

    QJsonArray rawArr;
    for (int i = 0; i < rawIntensities.size(); ++i)
        rawArr.append(rawIntensities(i));
    record["raw_intensities"] = rawArr;

    QJsonArray filtArr;
    for (int i = 0; i < filteredIntensities.size(); ++i)
        filtArr.append(filteredIntensities(i));
    record["filtered_intensities"] = filtArr;

    record["true_x"] = trueX;
    record["true_y"] = trueY;
    record["estimated_x"] = estX;
    record["estimated_y"] = estY;
    record["error"] = error;

    jsonArray.append(record);
}

void DataLogger::writeJsonData() {
    QJsonDocument doc(jsonArray);
    jsonFile.write(doc.toJson());
    jsonFile.close();
    logFile.close();
}
