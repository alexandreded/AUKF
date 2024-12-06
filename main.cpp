#include <QApplication>
#include <QMessageBox>
#include <QPushButton>
#include "gui/MainWindow.h"
#include "Config.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    Config config;
    // Диалог выбора режима работы
    QMessageBox msgBox;
    msgBox.setText("Выберите режим работы");
    QPushButton *simButton = msgBox.addButton("Симуляция", QMessageBox::ActionRole);
    QPushButton *realButton = msgBox.addButton("Реальные данные", QMessageBox::ActionRole);
    msgBox.exec();

    if (msgBox.clickedButton() == simButton) {
        config.mode = "simulation";
    } else if (msgBox.clickedButton() == realButton) {
        config.mode = "realtime";
        // при необходимости здесь можно добавить диалог выбора файла
    }

    MainWindow mainWindow(config);
    mainWindow.show();

    return app.exec();
}
