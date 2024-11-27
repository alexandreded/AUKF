# AUKF
AUKF
Данный проект представляет собой приложение на основе Qt, которое симулирует движение лазерного луча и использует адаптивный нецентральный фильтр Калмана (Adaptive Unscented Kalman Filter) для оценки положения пятна луча на основе измеренных интенсивностей с четырех детекторов.

Приложение позволяет визуализировать:

Нефильтрованные интенсивности от каждого из четырех детекторов.
Фильтрованные интенсивности после обработки фильтром Калмана.
Истинные координаты пятна луча.
Оцененные координаты, полученные с помощью фильтра Калмана.
Ошибку между истинными и оцененными координатами.
Структура проекта
Проект состоит из следующих основных компонентов:

AdaptiveUnscentedKalmanFilter:

AdaptiveUnscentedKalmanFilter.h
AdaptiveUnscentedKalmanFilter.cpp
Класс реализует адаптивный нецентральный фильтр Калмана для оценки состояния системы на основе измерений.
BeamSimulation:

BeamSimulation.h
BeamSimulation.cpp
Класс отвечает за симуляцию движения луча и генерацию интенсивностей на четырех детекторах.
MainWindow:

MainWindow.h
MainWindow.cpp
Основной класс приложения, реализующий пользовательский интерфейс и логику взаимодействия с пользователем.
EigenQDebug.h:

Заголовочный файл для корректного вывода объектов Eigen через QDebug.
main.cpp:

Точка входа в приложение.
Зависимости
Для сборки и запуска проекта необходимы следующие библиотеки и инструменты:

Qt версии 5 или выше:
Для реализации пользовательского интерфейса и общего приложения.
Eigen:
Библиотека для линейной алгебры, используемая в фильтре Калмана и симуляции луча.
Qwt:
Библиотека для построения графиков и визуализации данных.
C++11 или выше:
Проект использует современные возможности языка C++.
Сборка проекта
Установите зависимости:

Qt:
Рекомендуется использовать Qt Creator для удобства разработки.
Eigen:
Скачайте и распакуйте библиотеку Eigen.
Поскольку Eigen состоит только из заголовочных файлов, достаточно указать путь к ним в проекте.
Qwt:
Установите библиотеку Qwt для построения графиков.
Склонируйте репозиторий проекта или скачайте его исходный код.

Откройте проект:

Откройте файл проекта (.pro) в Qt Creator.
Убедитесь, что пути к библиотекам Eigen и Qwt правильно настроены в настройках проекта.
Соберите проект:

Нажмите "Run qmake" для генерации Makefile.
Нажмите "Сборка" для сборки проекта.
Запустите приложение:

После успешной сборки вы можете запустить приложение прямо из Qt Creator или выполнить скомпилированный исполняемый файл из командной строки.
Использование приложения
После запуска приложения вы увидите интерфейс с различными настройками и графиками.

Настройка параметров
Параметры фильтра:

Alpha: Определяет разброс сигма-точек вокруг среднего значения (обычно маленькое значение, например, 1e-3).
Beta: Используется для учета априорной информации о распределении (для гауссовского распределения обычно β=2).
Kappa: Параметр для настройки разброса сигма-точек (часто устанавливается в 0).
Process Noise: Ковариация шума процесса (матрица Q).
Measurement Noise: Ковариация шума измерения (матрица R).
Параметры симуляции:

Noise Level: Уровень шума, добавляемый к измерениям интенсивностей.
Gap Size: Размер щели между детекторами (значение от 0 до 1).
Управление симуляцией
Старт: Начинает симуляцию и обработку данных фильтром Калмана.
Стоп: Останавливает текущую симуляцию.
Сброс: Сбрасывает симуляцию и очищает все данные.
Отображение интенсивностей
Вы можете выбрать, какие интенсивности отображать на графиках, используя соответствующие чекбоксы:

I1, I2, I3, I4: Интенсивности от каждого из четырех детекторов.
Графики
Нефильтрованные интенсивности: Показывает исходные измеренные интенсивности с детекторов.
Фильтрованные интенсивности: Отображает интенсивности после обработки фильтром Калмана.
Координаты (фильтрованные): Показывает оцененные координаты пятна луча.
Исходные координаты: Показывает истинные координаты пятна луча в симуляции.
Ошибка координат: Отображает разницу между истинными и оцененными координатами.

Description
This project is a Qt application that simulates the movement of a laser beam. It uses an Adaptive Unscented Kalman Filter to estimate the position of the beam spot based on intensity measurements from four detectors. The application visualizes both raw and filtered intensity data, as well as compares true and estimated spot coordinates.

Key Features
Beam Movement Simulation: Generates intensity readings on four detectors.
Data Filtering: Applies an adaptive Kalman filter to enhance measurement accuracy.
Visualization:
Raw and filtered intensities.
True and estimated spot coordinates.
Errors between true and estimated positions.
Parameter Adjustment: Tune filter and simulation parameters through the user interface.
Dependencies
To build and run the project, the following libraries are required:

Qt (version 5 or higher)
Eigen (for linear algebra)
Qwt (for plotting graphs)
C++11 or higher
Building the Project
Install Dependencies:

Qt: Recommended to use Qt Creator.
Eigen: Download from Eigen and include it in your project.
Qwt: Install from Qwt.
Clone the Repository or download the source code.

Open the Project in Qt Creator:

Ensure that the paths to Eigen and Qwt libraries are correctly set.
Build the Project:

Click on "Run qmake", then "Build".
Run the Application:

Launch directly from Qt Creator or execute the compiled binary.
