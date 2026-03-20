cmake_minimum_required(VERSION 3.16)

project(parallel_drive_unit_ctrl VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(PDU_BUILD_TESTS "Build lightweight verification binaries" ON)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/linux_sdk/lib/output/libMotorDrive.so")
    option(PDU_BUILD_VENDOR_BACKEND "Build the vendor MotorDrive backend" ON)
else()
    option(PDU_BUILD_VENDOR_BACKEND "Build the vendor MotorDrive backend" OFF)
endif()

add_library(pdu_core
    src/backend_factory.cpp
    src/config.cpp
    src/kinematics.cpp
    src/mock_motor_backend.cpp
    src/parallel_drive_unit.cpp
)

target_include_directories(pdu_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(pdu_core PUBLIC cxx_std_17)

if(PDU_BUILD_VENDOR_BACKEND)
    find_package(Threads REQUIRED)

    add_library(pdu_vendor_backend
        src/vendor_sdk_backend.cpp
    )

    target_include_directories(pdu_vendor_backend
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/linux_sdk/include/c
    )

    target_link_directories(pdu_vendor_backend
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/linux_sdk/lib/output
    )

    target_link_libraries(pdu_vendor_backend
        PUBLIC
            pdu_core
        PRIVATE
            MotorDrive
            Threads::Threads
    )

    target_compile_definitions(pdu_core PUBLIC PDU_HAS_VENDOR_BACKEND=1)

    set_target_properties(pdu_vendor_backend PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/linux_sdk/lib/output"
    )
else()
    target_compile_definitions(pdu_core PUBLIC PDU_HAS_VENDOR_BACKEND=0)
endif()

add_executable(parallel_drive_app
    src/main.cpp
)

target_link_libraries(parallel_drive_app
    PRIVATE
        pdu_core
)

if(PDU_BUILD_VENDOR_BACKEND)
    target_link_libraries(parallel_drive_app PRIVATE pdu_vendor_backend)
    set_target_properties(parallel_drive_app PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/linux_sdk/lib/output"
    )
endif()

if(PDU_BUILD_TESTS)
    enable_testing()

    add_executable(parallel_drive_tests
        tests/parallel_drive_tests.cpp
    )

    target_link_libraries(parallel_drive_tests
        PRIVATE
            pdu_core
    )

    if(PDU_BUILD_VENDOR_BACKEND)
        target_link_libraries(parallel_drive_tests PRIVATE pdu_vendor_backend)
    endif()

    add_test(NAME parallel_drive_tests COMMAND parallel_drive_tests)
endif()
