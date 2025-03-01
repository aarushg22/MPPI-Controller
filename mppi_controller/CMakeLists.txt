cmake_minimum_required(VERSION 3.10)
project(custom_mppi_controller)

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

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Library source files
set(MPPI_SOURCES
  src/mppi_controller.cpp
)

# Create the library
add_library(mppi_controller STATIC ${MPPI_SOURCES})
target_include_directories(mppi_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Build the example executable
add_executable(mppi_example mppi_example.cpp)
target_link_libraries(mppi_example PRIVATE mppi_controller)

# Installation
install(TARGETS mppi_controller
  EXPORT mppi_controller-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS mppi_example
  DESTINATION bin
)
