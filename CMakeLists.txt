cmake_minimum_required(VERSION 3.16)
project(ppmt_fiducial_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(PPMT_FIDUCIAL_BUILD_TESTS "Build unit tests" ON)
option(PPMT_FIDUCIAL_BUILD_TOOLS "Build developer tools (synthetic data generator)" ON)

find_package(OpenCV REQUIRED COMPONENTS core)

add_library(ppmt_fiducial_core
  src/version.cpp
)
target_include_directories(ppmt_fiducial_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(ppmt_fiducial_core PUBLIC ${OpenCV_LIBS})
target_compile_options(ppmt_fiducial_core PRIVATE -Wall -Wextra -Wpedantic -Werror)

if(PPMT_FIDUCIAL_BUILD_TOOLS)
  add_subdirectory(tools)
endif()

if(PPMT_FIDUCIAL_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
