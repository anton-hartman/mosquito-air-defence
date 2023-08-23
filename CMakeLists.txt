# # # cmake_minimum_required(VERSION 3.10)
# # # project(MosquitoAirDefence)

# # # Add subdirectories here
# # add_subdirectory(src)
# # add_subdirectory(tests)

# cmake_minimum_required(VERSION 3.10)
# project(MosquitoAirDefence)

# # Set the C++ standard
# set(CMAKE_CXX_STANDARD 11)

# # Find the JetsonGPIO library
# find_package(JetsonGPIO)

# # Include directories
# include_directories(
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
#     # Add any other necessary include directories here
# )

# # Add executable
# add_executable(mosquito-air-defence main.cpp)

# # Link libraries
# target_link_libraries(mosquito-air-defence
#     JetsonGPIO::JetsonGPIO
#     # Add any other necessary libraries here
# )

cmake_minimum_required(VERSION 3.10)
project(MosquitoAirDefence)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)

# Find the JetsonGPIO library
find_package(JetsonGPIO)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    # Add any other necessary include directories here
)

# List all source and header files
set(SOURCE_FILES
    main.cpp
    src/motors/controller.cpp
    src/motors/HR8825_driver.cpp
    src/utilities/utilities.cpp
)

set(HEADER_FILES
    src/motors/controller.hpp
    src/motors/HR8825_driver.hpp
    src/utilities/debug.hpp
    src/utilities/utilities.hpp
)

# Add executable
add_executable(mosquito-air-defence ${SOURCE_FILES} ${HEADER_FILES})

# Link libraries
target_link_libraries(mosquito-air-defence
    JetsonGPIO::JetsonGPIO
    # Add any other necessary libraries here
)
