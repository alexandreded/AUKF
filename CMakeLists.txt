cmake_minimum_required(VERSION 3.5)

project(AUKFProject)

# Установка стандарта C++
set(CMAKE_CXX_STANDARD 14)

# Автоматическая генерация и включение заголовочных файлов Qt
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# Включить генерацию compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Поиск пакетов
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Вручную указываем директории include и библиотеки для Qwt
set(QWT_INCLUDE_DIRS "/usr/include/qwt")
set(QWT_LIBRARIES "/usr/lib/libqwt.so")

include_directories(
    ${QWT_INCLUDE_DIRS}
    src
)

# Исходные файлы
set(SOURCES
    src/main.cpp
    src/AdaptiveUnscentedKalmanFilter.cpp
    src/BeamSimulation.cpp
    src/MainWindow.cpp
)

# Заголовочные файлы
set(HEADERS
    src/AdaptiveUnscentedKalmanFilter.h
    src/BeamSimulation.h
    src/MainWindow.h
)

# Создание исполняемого файла
add_executable(AUKFProject ${SOURCES} ${HEADERS})

# Подключение библиотек
target_link_libraries(AUKFProject
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    ${QWT_LIBRARIES}
)
