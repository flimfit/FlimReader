cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMreaderTest)

find_package(Boost CONFIG REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED)

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
