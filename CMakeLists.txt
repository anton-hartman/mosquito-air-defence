cmake_minimum_required(VERSION 3.10)
project(MosquitoAirDefence)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

enable_language(CUDA)

find_package(CUDA REQUIRED)
find_package(JetsonGPIO REQUIRED)
find_package(Curses REQUIRED) # For keyboard input
find_package(OpenCV REQUIRED)


# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CURSES_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS} 
    ${CUDA_INCLUDE_DIRS}
)

# List all source and header files
set(SOURCE_FILES
    # src/camera_calibration.cpp
    main.cpp
    src/frame.cpp
    src/stepper.cpp
    src/turret.cpp
    src/image_processing.cu
    src/mads.cpp
)

set(HEADER_FILES
    include/frame.hpp
    include/mads.hpp
    include/pt.hpp
    include/stepper.hpp
    include/turret.hpp
    include/image_processing.hpp
)

# Add executable
add_executable(mosquito-air-defence ${SOURCE_FILES} ${HEADER_FILES})

# Link libraries
target_link_libraries(mosquito-air-defence
    JetsonGPIO::JetsonGPIO
    ${CURSES_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
)