cmake_minimum_required(VERSION 3.10)
project(MosquitoAirDefence)

# Set the language standards
set(CMAKE_CXX_STANDARD 14)
enable_language(CUDA)

# Find required packages
find_package(CUDA REQUIRED)
find_package(JetsonGPIO REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${CUDA_TOOLKIT_ROOT_DIR}/include
)

# Automatically include all source and header files from specific directories (Optional)
# file(GLOB SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu)
# file(GLOB HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp)

# Explicitly list all source and header files
set(SOURCE_FILES
    main.cpp
    src/frame.cpp
    src/stepper.cpp
    src/turret.cpp
    src/utils.cpp
    src/detection.cpp
    src/image_processing.cu
    src/tracking.cpp
    src/kalman.cpp
    src/hungarian.cpp
)

set(HEADER_FILES
    include/frame.hpp
    include/stepper.hpp
    include/turret.hpp
    include/utils.hpp
    include/detection.hpp
    include/image_processing.hpp
    include/tracking.hpp
    include/kalman.hpp
    include/kalman_interface.hpp
    include/tracking_interface.hpp
    include/hungarian.hpp

    # include/kalman_filter_2d.hpp
    )

# Add executable
add_executable(mosquito_air_defence ${SOURCE_FILES} ${HEADER_FILES})

# Link libraries
target_link_libraries(mosquito_air_defence
    JetsonGPIO::JetsonGPIO
    ${CURSES_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    Eigen3::Eigen
)
