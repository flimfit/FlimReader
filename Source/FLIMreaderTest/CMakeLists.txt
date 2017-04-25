cmake_minimum_required(VERSION 3.3)

PROJECT(FLIMreaderTest)

find_package(Boost CONFIG REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release)
find_package(dlib REQUIRED)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

include_directories(${FLIMreader_INCLUDE_DIR})

# Source Files
#====================================
set(SOURCE
   FLIMreaderTest.cpp
)

set(HEADERS
   FLIMreaderTest.h
   catch.hpp
)

add_executable(FLIMreaderTest ${SOURCE} ${HEADERS} $<TARGET_OBJECTS:FLIMreader>)
target_link_libraries(FLIMreaderTest ${FLIMreader_LIBRARIES})

set_property(TARGET FLIMreaderTest PROPERTY CXX_STANDARD 14)
