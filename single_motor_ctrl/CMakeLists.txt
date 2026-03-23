cmake_minimum_required(VERSION 3.16)

project(single_motor_cli LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(single_motor_core
    src/common/config_loader.cpp
    src/device/board_client.cpp
    src/driver/motor_driver.cpp
    src/control/safety_manager.cpp
    src/control/telemetry_monitor.cpp
    src/domain/single_motor_controller.cpp
    src/app/cli_app.cpp
)

target_include_directories(single_motor_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/linux_sdk/include/c
)

target_link_directories(single_motor_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/linux_sdk/lib/output
)

target_link_libraries(single_motor_core
    PUBLIC
        MotorDrive
        Threads::Threads
)

if(UNIX AND NOT APPLE)
    target_link_options(single_motor_core PUBLIC
        -Wl,-rpath,${PROJECT_SOURCE_DIR}/linux_sdk/lib/output
    )
endif()

add_executable(motor_cli src/main.cpp)
target_link_libraries(motor_cli PRIVATE single_motor_core)
