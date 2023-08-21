cmake_minimum_required(VERSION 3.10)

# Set the project name and specify that it has both C and C++ languages
project(mosquito-air-defence LANGUAGES C CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Add subdirectories to the build
add_subdirectory(c_cpp_system)
