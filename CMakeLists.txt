cmake_minimum_required(VERSION 3.5)

project(AUKFProject)

# Стандарт C++
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Пути к Qwt могут отличаться в зависимости от системы
set(QWT_INCLUDE_DIRS "/usr/include/qwt")
set(QWT_LIBRARIES "/usr/lib/libqwt.so")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${QWT_INCLUDE_DIRS}
)

# Источники
set(SOURCES
    main.cpp
    gui/MainWindow.cpp
    filter/AdaptiveUnscentedKalmanFilter.cpp
    simulation/BeamSimulation.cpp
    io/DataLogger.cpp
)

set(HEADERS
    Config.h
    gui/MainWindow.h
    filter/AdaptiveUnscentedKalmanFilter.h
    simulation/BeamSimulation.h
    io/DataLogger.h
)

add_executable(AUKFProject ${SOURCES} ${HEADERS})



target_link_libraries(AUKFProject
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    ${QWT_LIBRARIES}
)
