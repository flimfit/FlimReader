cmake_minimum_required(VERSION 3.3)

PROJECT(FLIMreaderTest)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

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
