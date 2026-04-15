#include <QApplication>
#include <QMessageBox>
#include <QPushButton>
#include "gui/MainWindow.h"
#include "Config.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    app.setStyle("Fusion");

    Config config;
    // Диалог выбора режима работы
    QMessageBox msgBox;
    msgBox.setText("Выберите режим работы");
    QPushButton *simButton = msgBox.addButton("Симуляция", QMessageBox::ActionRole);
    QPushButton *realButton = msgBox.addButton("Реальные данные", QMessageBox::ActionRole);
    QPushButton *hardwareButton = msgBox.addButton("Аппаратный режим", QMessageBox::ActionRole);
    QPushButton *calibrationButton = msgBox.addButton("Калибровка (feedback)", QMessageBox::ActionRole);
    msgBox.exec();

    if (msgBox.clickedButton() == simButton) {
        config.mode = "simulation";
    } else if (msgBox.clickedButton() == realButton) {
        config.mode = "realtime";
        // при необходимости здесь можно добавить диалог выбора файла
    } else if (msgBox.clickedButton() == hardwareButton) {
        config.mode = "hardware";
        // Hardware получает измерения напрямую с платы через BoardDriver::readMeasurement().
    } else if (msgBox.clickedButton() == calibrationButton) {
        config.mode = "hardware";
        config.enableCalibrationFeedback = true;
        config.calibrationDriveHardware = true;
        // Режим калибровки использует обратную связь и аппаратное подключение.
    }

    MainWindow mainWindow(config);
    mainWindow.show();

    return app.exec();
}
