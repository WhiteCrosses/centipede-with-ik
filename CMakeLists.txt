cmake_minimum_required(VERSION 3.10)

# Set the project name
project(Main)

# Set the path to the SFML directory containing the CMake configuration files
set(SFML_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmake/SFML")

set(CMAKE_BUILD_TYPE Debug)

# Add an executable
set(SOURCE_FILES
    src/main.cpp
    src/Centipede.cpp
    src/Game.cpp
    src/render/Projection.cpp
    src/render/GridRenderer.cpp)
    list(APPEND SOURCE_FILES
        src/render/DrawHelpers.cpp)
    # Newly added modules
    list(APPEND SOURCE_FILES
        src/gait/GaitController.cpp
        src/ik/LegIK.cpp)
    list(APPEND SOURCE_FILES
        src/input/Camera.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Set C++ standard
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)

# Find SFML package (static linking)
set(SFML_STATIC_LIBRARIES TRUE)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link SFML libraries
target_link_libraries(${PROJECT_NAME} PRIVATE sfml-graphics sfml-window sfml-system)

# Add definitions for static SFML linking
target_compile_definitions(${PROJECT_NAME} PRIVATE SFML_STATIC)

# Set additional properties for the linker (like -mwindows if needed)
if(WIN32)
    target_link_options(${PROJECT_NAME} PRIVATE -mconsole)
endif()
