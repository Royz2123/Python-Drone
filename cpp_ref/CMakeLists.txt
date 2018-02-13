#
# Written by Andrey Leshenko and Eli Tarnarutsky, 2017.
# Published under the MIT license.
#

cmake_minimum_required(VERSION 3.2)

project(linetrack)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake/)
find_package(raspicam REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(linetrack linetrack.cpp pid.cpp quad_serial.cpp)
target_link_libraries(linetrack ${OpenCV_LIBS} ${raspicam_CV_LIBS})

if (CMAKE_COMPILER_IS_GNUCC)
	set_property( TARGET linetrack APPEND_STRING PROPERTY COMPILE_FLAGS -Wall )
endif (CMAKE_COMPILER_IS_GNUCC)
