// BeamSimulation.h

#ifndef BEAM_SIMULATION_H
#define BEAM_SIMULATION_H

#include <Eigen/Dense>
#include <vector>
#include <functional>

class BeamSimulation {
public:
    BeamSimulation(double P0, double w, double x0, double y0);

    Eigen::VectorXd generateMeasurement(int iteration);

    // Получение текущих координат
    double getXc() const;
    double getYc() const;

private:
    double P0;      // Мощность луча
    double w;       // Ширина луча
    double x_c;     // Положение по x
    double y_c;     // Положение по y
    int k;          // Направление движения по x

    // Дополнительные параметры
    double speed;           // Скорость движения
    double adc_time_step;   // Временной шаг
    double bounds_min;      // Нижняя граница движения
    double bounds_max;      // Верхняя граница движения

    // Функции для генерации данных
    double h_function(double x, double y, double X, double Y);
    double erfinv(double x);
    double g_function(double Ex);
    double lambda_func(double x0);

    // Генерация случайного шума
    double randomNoise(double level);
};

#endif // BEAM_SIMULATION_H
