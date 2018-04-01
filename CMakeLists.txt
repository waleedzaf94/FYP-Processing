cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(one)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

include( ${CGAL_USE_FILE} )
include( ${EIGEN3_USE_FILE} )
# include ( ${BOOST_USE_FILE} )

# Output folder
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
include_directories("${PROJECT_SOURCE_DIR}")

include_directories(${CGAL_INCLUDE_DIRS} ${BOOST_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})
 file(GLOB processing_SRC	"*.h" "*.hpp"	"*.cpp")

add_executable(processing ${processing_SRC})  # Adds the cpp files in folder to executable
target_link_libraries(processing ${CGAL_LIBRARIES} ${BOOST_LIBRARIES} ${EIGEN_LIBRARIES})