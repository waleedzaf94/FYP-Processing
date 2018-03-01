cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(one)
set(CMAKE_CXX_STANDARD 11)

find_package(CGAL REQUIRED)
find_package(Boost REQUIRED )
find_package(Eigen3 REQUIRED)

find_package(PCL 1.8 REQUIRED)
if (EIGEN3_FOUND)
  include( ${EIGEN3_USE_FILE} )
endif()

include( ${CGAL_USE_FILE} )

# Output folder
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
include_directories("${PROJECT_SOURCE_DIR}")
include_directories(${PROJECT_SOURCE_DIR}/vcglib)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
file(GLOB processing_SRC
	"*.h"
	"*.hpp"
	"*.cpp"
)


add_definitions(${PCL_DEFINITIONS})
add_executable(processing ${processing_SRC})
target_link_libraries(processing ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
