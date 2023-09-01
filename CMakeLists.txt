cmake_minimum_required(VERSION 3.10)
project(MosquitoAirDefence)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

find_package(JetsonGPIO)
find_package(Curses REQUIRED) # For keyboard input
find_package(OpenCV REQUIRED)


# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CURSES_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS} 
)

# List all source and header files
set(SOURCE_FILES
    main.cpp
    src/turret/turret_controller.cpp
    src/turret/hr8825_driver.cpp
    src/utilities/utils.cpp
    src/detection/object_detector.cpp
    src/detection/background_subtractor.cpp
    # src/video_interface.cpp
)

set(HEADER_FILES
    src/turret/turret_controller.hpp
    src/turret/hr8825_driver.hpp
    src/utilities/debug.hpp
    src/utilities/utils.hpp
    src/detection/object_detector.hpp
    src/detection/background_subtractor.hpp
)

# Add executable
add_executable(mosquito-air-defence ${SOURCE_FILES} ${HEADER_FILES})

# Link libraries
target_link_libraries(mosquito-air-defence
    JetsonGPIO::JetsonGPIO
    ${CURSES_LIBRARIES}
    ${OpenCV_LIBS}
)

# ---- Google Test Configuration ----
# Specify the location of Google Test on your system
# set(GTEST_DIR /path/to/google/test) # Replace with the actual path

# Find Google Test and Google Mock
find_package(GTest REQUIRED)

enable_testing()

# Add your unit tests target
add_executable(unit_tests
    tests/test_main.cpp  # A main file for running all tests
    tests/turret/turret_controller_test.cpp  
    tests/turret/hr8825_driver_test.cpp
)

# Link your project's code and Google Test
target_link_libraries(unit_tests
    # PRIVATE
    # build/mosqutio-air-defence  # Link your project's library or source files
    GTest::GTest
    GTest::Main
)

# Register your tests with CTest
include(GoogleTest)
gtest_discover_tests(unit_tests)