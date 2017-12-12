cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(one)
find_package(PCL 1.6 REQUIRED COMPONENTS common io visualization)

# Output folder
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(INCLUDE_DIR .)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(processing ${PROJECT_SOURCE_DIR}/processing.cpp)
target_link_libraries(processing ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
