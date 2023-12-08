cmake_minimum_required(VERSION 3.10)
project(tonioviz)

# Set some compilation options.
set(CMAKE_CXX_STANDARD 17)

# External package dependencies.
find_package(Eigen3 3.3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV 4 REQUIRED)

# Modularize into separate components.
option(BUILD_GTSAM_UTILS "Build the GTSAM utilities module." ON)
option(BUILD_EXAMPLES "Build the examples." OFF)
# Create library.
set(TONIOVIZ_SRCS
  src/DataUtils.cpp
  src/Visualizer.cpp)
set(TONIOVIZ_INCLUDE_DIRS
  include
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})
set(TONIOVIZ_LIBS
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen)

# GTSAM module.
if (BUILD_GTSAM_UTILS)
  find_package(GTSAM REQUIRED)
  # Add GTSAM-related files.
  list(APPEND TONIOVIZ_SRCS
    src/GtsamUtils.cpp)
  list(APPEND TONIOVIZ_HDRS
    include/tonioviz/GtsamUtils.h)
  # Add GTSAM library as dependency.
  list(APPEND TONIOVIZ_LIBS
    gtsam)
endif()

add_library(tonioviz SHARED ${TONIOVIZ_SRCS})
target_include_directories(tonioviz
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Pangolin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

target_link_libraries(tonioviz PUBLIC ${TONIOVIZ_LIBS})

target_compile_options(tonioviz PRIVATE -Wall)

# Make library reachable by other cmake projects.
export(PACKAGE tonioviz)
export(TARGETS tonioviz FILE toniovizConfig.cmake)

install(TARGETS tonioviz
    EXPORT toniovizTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
    PACKAGE DESTINATION lib
)

# Create and install the package configuration file
install(EXPORT toniovizTargets
    FILE toniovizTargets.cmake
    NAMESPACE tonioviz::
    DESTINATION lib/cmake/tonioviz
)

if (BUILD_EXAMPLES)
  # Build a bunch of useful examples.
  add_subdirectory(examples)
endif ()

# Include unit tests directory to the project.
enable_testing()
add_subdirectory(tests)
