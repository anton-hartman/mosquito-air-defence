# Include the current directory (for header files)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add the detection and steppers subdirectories
add_subdirectory(detection)
add_subdirectory(steppers)

# Create an executable for the main system
add_executable(main_system main.c sysfs_gpio.c)

# If the detection or steppers directories produce libraries, link them here
# target_link_libraries(main_system detection_lib steppers_lib)
