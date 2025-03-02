cmake_minimum_required(VERSION 3.10)
project(turtlebot3_mppi_controller)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add compile options
add_compile_options(-Wall -Wextra -Wpedantic)

# Default to Release build type if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Optimization flags for Release build
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(WEBOTS_HOME "/usr/local/webots")
# compile the mppi controller lib
add_subdirectory(mppi_controller)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/mppi_controller/include
)

# Link with the Webots controller library.
link_directories(${WEBOTS_HOME}/lib/controller)
set (LIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories(
    ${WEBOTS_HOME}/include/controller/c 
    ${WEBOTS_HOME}/include/controller/cpp
)

# Build the example executable
add_executable(turtlebot3_ostacle_avoidance turtlebot3_obstacle_avoidance.cpp)
target_link_libraries(turtlebot3_ostacle_avoidance ${LIBRARIES} mppi_controller)

# Installation
install(TARGETS turtlebot3_ostacle_avoidance)

# Custom command to copy the executable to the parent directory
add_custom_command(TARGET turtlebot3_ostacle_avoidance POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  $<TARGET_FILE:turtlebot3_ostacle_avoidance>
  ${CMAKE_CURRENT_SOURCE_DIR}/../
  COMMENT "Copying turtlebot3_ostacle_avoidance to parent directory"
)


