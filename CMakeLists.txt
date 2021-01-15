cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(autopilot_manager)

if(NOT MSVC)
    add_definitions("-Wall -Wextra ")
    # This warning is triggered by the MAVLink includes.
    add_definitions("-Wno-address-of-packed-member")
else()
    add_definitions("-WX -W2")
endif()

find_package(MAVSDK REQUIRED)

add_subdirectory(src/modules)

add_executable(
    autopilot-manager
    src/main.cpp
)

target_link_libraries(
    autopilot-manager
    mission-manager
    MAVSDK::mavsdk
)

# Install autopilot-manager
install(TARGETS autopilot-manager RUNTIME DESTINATION bin)
