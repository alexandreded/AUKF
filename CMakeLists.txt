cmake_minimum_required(VERSION 3.5)

project(AUKFProject)

# стандарта C++
set(CMAKE_CXX_STANDARD 14)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})


set(QWT_INCLUDE_DIRS "/usr/include/qwt")
set(QWT_LIBRARIES "/usr/lib/libqwt.so")

include_directories(
    ${QWT_INCLUDE_DIRS}
    src
)


set(SOURCES
    src/main.cpp
    src/AdaptiveUnscentedKalmanFilter.cpp
    src/BeamSimulation.cpp
    src/MainWindow.cpp
)


set(HEADERS
    src/AdaptiveUnscentedKalmanFilter.h
    src/BeamSimulation.h
    src/MainWindow.h
)


add_executable(AUKFProject ${SOURCES} ${HEADERS})


target_link_libraries(AUKFProject
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    ${QWT_LIBRARIES}
)
