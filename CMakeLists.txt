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
    src/motors/controller.cpp
    src/motors/HR8825_driver.cpp
    src/utilities/utilities.cpp
    src/detection/object_detector.cpp
    src/detection/background_subtractor.cpp
    src/video_interface.cpp
)

set(HEADER_FILES
    src/motors/controller.hpp
    src/motors/HR8825_driver.hpp
    src/utilities/debug.hpp
    src/utilities/utilities.hpp
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
