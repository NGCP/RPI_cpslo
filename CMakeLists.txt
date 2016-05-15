cmake_minimum_required(VERSION 2.8)  

# name of project
project(rpi_cpslo)

# Use glob to get the list of all source files.
file(GLOB_RECURSE SOURCES "src/*.cpp")

# We don't really need to include header and resource files to build, but it's
# nice to have them show up in IDEs.
file(GLOB_RECURSE HEADERS "src/*.h")

# Set the executable.
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES} ${HEADERS})

# Get the MAVlink environment variable.
set(MAVLINK_DIR "$ENV{MAVLINK_DIR}")
if(NOT MAVLINK_DIR)
	MESSAGE(FATAL_ERROR "Please point the environment variable MAVLINK_DIR to the root directory of your mavlink installation.")
endif()
include_directories(${MAVLINK_DIR}/include)

# Get OpenCV library
# TODO: Implement installation of OpenCV via bash or other methods if package not found
find_package(OpenCV REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} ${OpenCV_LIBS})

# Add the pthread library
find_package(Threads REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

# OS Specific Compiler options and libraries
# TODO: Implement Options for Mac & Windows
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -lpthread")