#ifndef BEAM_SIMULATION_H
#define BEAM_SIMULATION_H

#include <Eigen/Dense>
#include <random>

class BeamSimulation {
public:
    BeamSimulation();

    // Метод для перемещения луча и получения интенсивностей
    Eigen::VectorXd moveBeamAndIntegrate(double P0, double w);

    // Установка уровня шума
    void setNoiseLevel(double noiseLevel);

    // Установка размера щели между детекторами
    void setGapSize(double gapSize);

    // Установка шага времени и скорости луча
    void setTimeStep(double timeStep);
    void setBeamSpeed(double speed);

    // Получение текущих координат
    double getXc() const;
    double getYc() const;

    // Сброс симуляции
    void reset();

private:
    double x_c; // Координата X центра луча
    double y_c; // Координата Y центра луча

    int k_x; // Направление движения по X
    int k_y; // Направление движения по Y

    double noiseLevel; // Уровень шума
    double gapSize;    // Размер щели между детекторами

    double timeStep;   // Шаг времени
    double speed;      // Скорость луча

    // Генератор случайных чисел
    std::default_random_engine generator;
    std::normal_distribution<double> noiseDistribution;

    // Внутренние методы
    double gaussianBeam(double x, double y, double P0, double x_c, double y_c, double w);

    // Метод для интегрирования интенсивности по площади детектора
    double integrateIntensity(double x_min, double x_max, double y_min, double y_max,
                              double P0, double x_c, double y_c, double w);
};

#endif // BEAM_SIMULATION_H
