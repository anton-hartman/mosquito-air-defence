# Find the OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Create a library (or executable) for the detection code
# Replace 'detection_files.cpp' with your actual C++ files
add_library(detection_lib detection.cpp)

# Link the OpenCV libraries
target_link_libraries(detection_lib ${OpenCV_LIBS})
