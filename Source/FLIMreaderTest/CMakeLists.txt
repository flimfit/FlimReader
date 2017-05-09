cmake_minimum_required(VERSION 3.3)

PROJECT(FlimReaderTest)

find_package(Boost CONFIG REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release)
find_package(dlib REQUIRED)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

include_directories(${FlimReader_INCLUDE_DIR})

# Source Files
#====================================
set(SOURCE
   FlimReaderTest.cpp
)

set(HEADERS
   FlimReaderTest.h
   catch.hpp
)

add_executable(FlimReaderTest ${SOURCE} ${HEADERS} $<TARGET_OBJECTS:FlimReader>)
target_link_libraries(FlimReaderTest ${FlimReader_LIBRARIES})

set_property(TARGET FlimReaderTest PROPERTY CXX_STANDARD 14)
