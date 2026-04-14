#include "DataLogger.h"
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <cmath>

DataLogger::DataLogger(const QString &logFilePath, const QString &jsonFilePath)
    : logFile(logFilePath), jsonFile(jsonFilePath)
{
    if (logFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        logReady = true;
        logStream.setDevice(&logFile);
    } else {
        qWarning() << "Failed to open log file:" << logFilePath;
    }

    if (jsonFile.open(QIODevice::WriteOnly)) {
        jsonReady = true;
    } else {
        qWarning() << "Failed to open JSON output file:" << jsonFilePath;
    }
}

DataLogger::~DataLogger() {
    writeJsonData();
}

void DataLogger::setMetadata(const QJsonObject &metadataObject) {
    metadata = metadataObject;
}

void DataLogger::logMessage(const QString &message) {
    if (!logReady) {
        return;
    }
    logStream << message << "\n";
}

void DataLogger::addRecord(double time,
                           const Eigen::VectorXd &rawIntensities,
                           const Eigen::VectorXd &calibratedIntensities,
                           const Eigen::VectorXd &filteredIntensities,
                           double trueX, double trueY,
                           double estX, double estY,
                           double error,
                           bool calibrationEnabled,
                           bool calibrationConverged,
                           double calibrationErrorX,
                           double calibrationErrorY,
                           const Eigen::Vector4d &calibrationGains,
                           bool hardwareFeedbackEnabled,
                           bool hardwareFeedbackApplied,
                           const Eigen::Vector4d &hardwareFrequenciesMHz,
                           const Eigen::Vector4d &hardwareAmplitudes)
{
    auto numberOrNull = [](double value) -> QJsonValue {
        if (std::isfinite(value)) {
            return QJsonValue(value);
        }
        return QJsonValue(QJsonValue::Null);
    };

    QJsonObject record;
    record["time"] = numberOrNull(time);

    QJsonArray rawArr;
    for (int i = 0; i < rawIntensities.size(); ++i)
        rawArr.append(rawIntensities(i));
    record["raw_intensities"] = rawArr;

    QJsonArray filtArr;
    for (int i = 0; i < filteredIntensities.size(); ++i)
        filtArr.append(filteredIntensities(i));
    record["filtered_intensities"] = filtArr;

    QJsonArray calibArr;
    for (int i = 0; i < calibratedIntensities.size(); ++i)
        calibArr.append(calibratedIntensities(i));
    record["calibrated_intensities"] = calibArr;

    record["true_x"] = numberOrNull(trueX);
    record["true_y"] = numberOrNull(trueY);
    record["estimated_x"] = numberOrNull(estX);
    record["estimated_y"] = numberOrNull(estY);
    record["error"] = numberOrNull(error);
    record["calibration_enabled"] = calibrationEnabled;
    record["calibration_converged"] = calibrationConverged;
    record["calibration_error_x"] = numberOrNull(calibrationErrorX);
    record["calibration_error_y"] = numberOrNull(calibrationErrorY);
    QJsonArray gainsArr;
    for (int i = 0; i < 4; ++i) {
        gainsArr.append(numberOrNull(calibrationGains(i)));
    }
    record["calibration_gains"] = gainsArr;
    record["hardware_feedback_enabled"] = hardwareFeedbackEnabled;
    record["hardware_feedback_applied"] = hardwareFeedbackApplied;
    QJsonArray hwFreqArr;
    QJsonArray hwAmpArr;
    for (int i = 0; i < 4; ++i) {
        hwFreqArr.append(numberOrNull(hardwareFrequenciesMHz(i)));
        hwAmpArr.append(numberOrNull(hardwareAmplitudes(i)));
    }
    record["hardware_frequencies_mhz"] = hwFreqArr;
    record["hardware_amplitudes"] = hwAmpArr;

    jsonArray.append(record);
}

void DataLogger::writeJsonData() {
    if (!jsonReady) {
        if (logReady) {
            logFile.close();
        }
        return;
    }

    QJsonObject root;
    root["metadata"] = metadata;
    root["records"] = jsonArray;
    QJsonDocument doc(root);
    jsonFile.write(doc.toJson());
    jsonFile.close();
    if (logReady) {
        logFile.close();
    }
}
