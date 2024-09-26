cmake_minimum_required(VERSION 3.5)

project(AUKFProject)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_PREFIX_PATH "/path/to/Qt")  # Укажите путь к Qt

find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)

# Путь к библиотеке Qwt
set(QWT_ROOT "/path/to/qwt")  # Укажите путь к Qwt
include_directories(${QWT_ROOT}/include)
link_directories(${QWT_ROOT}/lib)

# Путь к библиотеке Eigen
include_directories("/path/to/eigen")  # Укажите путь к Eigen

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

target_link_libraries(AUKFProject Qt5::Widgets Qt5::Core Qt5::Gui qwt)
