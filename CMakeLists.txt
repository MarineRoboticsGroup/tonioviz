cmake_minimum_required(VERSION 3.10)
project(tonioviz)

# Set some compilation options.
set(CMAKE_CXX_STANDARD 17)

# External package dependencies.
find_package(GTSAM REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
# find_package(cpp-toolbox REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(SESync REQUIRED)

# Create library.
set(TONIOVIZ_SRCS
  src/DataUtils.cpp
  src/Visualizer.cpp)

set(TONIOVIZ_HDRS
  include/tonioviz/DataUtils.h
  include/tonioviz/Visualizer.h)

set(TONIOVIZ_LIBS
  gtsam
  ${Pangolin_LIBRARIES}
  # cpp-toolbox
  ${OpenCV_LIBS}
  Eigen3::Eigen)

add_library(tonioviz STATIC ${TONIOVIZ_SRCS} ${TONIOVIZ_HDRS})
target_include_directories(tonioviz PUBLIC include)
target_link_libraries(tonioviz PUBLIC ${TONIOVIZ_LIBS})
target_compile_options(tonioviz PRIVATE -Wall)

# Make library reachable by other cmake projects.
export(PACKAGE tonioviz)
export(TARGETS tonioviz FILE toniovizConfig.cmake)

# Build a bunch of useful examples.
add_subdirectory(examples)

# Include unit tests directory to the project.
enable_testing()
add_subdirectory(tests)
