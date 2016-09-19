cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMreaderTest)

find_package(Boost CONFIG REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED)
find_package("OpenMP" REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0043 NEW)

include_directories(../FLIMreader)

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
