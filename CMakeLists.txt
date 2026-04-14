cmake_minimum_required(VERSION 3.10)

project(AUKFProject)

# Стандарт C++
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_path(QWT_INCLUDE_DIR NAMES qwt_plot.h PATH_SUFFIXES qwt)
find_library(QWT_LIBRARY NAMES qwt qwt-qt5 qwt6)
option(AUKF_ENABLE_AD995X_DRIVER "Enable AD995x USB board driver integration" ON)

if (NOT QWT_INCLUDE_DIR OR NOT QWT_LIBRARY)
    message(FATAL_ERROR "Qwt library not found. Install Qwt and ensure headers/libs are discoverable.")
endif()

set(QWT_INCLUDE_DIRS ${QWT_INCLUDE_DIR})
set(QWT_LIBRARIES ${QWT_LIBRARY})

set(AUKF_AD995X_SOURCES)
set(AUKF_AD995X_LIBRARIES)
set(AUKF_AD995X_INCLUDE_DIRS)

if (AUKF_ENABLE_AD995X_DRIVER)
    find_path(AD995X_DRIVER_INCLUDE_DIR
        NAMES ad995x_usb_aod_driver.h
        PATHS
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/../ad9959_AOD_STM32/include
    )

    find_package(PkgConfig QUIET)
    if (PKG_CONFIG_FOUND)
        pkg_check_modules(LIBUSB QUIET libusb-1.0)
    endif()

    if (AD995X_DRIVER_INCLUDE_DIR AND LIBUSB_FOUND)
        set(AUKF_AD995X_SOURCES ad995x_usb_aod_driver.cpp)
        set(AUKF_AD995X_INCLUDE_DIRS ${AD995X_DRIVER_INCLUDE_DIR} ${LIBUSB_INCLUDE_DIRS})
        set(AUKF_AD995X_LIBRARIES ${LIBUSB_LIBRARIES})
        add_compile_definitions(AUKF_HAS_AD995X_DRIVER=1)
        message(STATUS "AD995x driver enabled")
    else()
        message(STATUS "AD995x driver disabled (header or libusb-1.0 not found)")
    endif()
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${QWT_INCLUDE_DIRS}
)

if (AUKF_AD995X_INCLUDE_DIRS)
    include_directories(SYSTEM ${AUKF_AD995X_INCLUDE_DIRS})
endif()

# Источники
set(SOURCES
    main.cpp
    core/TrackingEngine.cpp
    hardware/BoardDriver.cpp
    gui/MainWindow.cpp
    filter/AdaptiveUnscentedKalmanFilter.cpp
    simulation/BeamSimulation.cpp
    io/DataLogger.cpp
    ${AUKF_AD995X_SOURCES}
)

set(HEADERS
    Config.h
    core/TrackingEngine.h
    hardware/BoardDriver.h
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
    ${AUKF_AD995X_LIBRARIES}
)

include(CTest)
if (BUILD_TESTING)
    add_executable(test_beam_simulation
        tests/test_beam_simulation.cpp
        simulation/BeamSimulation.cpp
    )
    target_include_directories(test_beam_simulation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_beam_simulation Eigen3::Eigen)
    add_test(NAME beam_simulation COMMAND test_beam_simulation)

    add_executable(test_aukf_filter
        tests/test_aukf_filter.cpp
        filter/AdaptiveUnscentedKalmanFilter.cpp
    )
    target_include_directories(test_aukf_filter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_aukf_filter Eigen3::Eigen)
    add_test(NAME aukf_filter COMMAND test_aukf_filter)

    add_executable(test_tracking_engine
        tests/test_tracking_engine.cpp
        core/TrackingEngine.cpp
        hardware/BoardDriver.cpp
        filter/AdaptiveUnscentedKalmanFilter.cpp
        simulation/BeamSimulation.cpp
        ${AUKF_AD995X_SOURCES}
    )
    target_include_directories(test_tracking_engine PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_tracking_engine Eigen3::Eigen ${AUKF_AD995X_LIBRARIES})
    add_test(NAME tracking_engine COMMAND test_tracking_engine)
endif()

if (AUKF_AD995X_SOURCES)
    set_source_files_properties(ad995x_usb_aod_driver.cpp
        PROPERTIES COMPILE_OPTIONS "-Wno-pedantic;-Wno-unused-parameter")
endif()
