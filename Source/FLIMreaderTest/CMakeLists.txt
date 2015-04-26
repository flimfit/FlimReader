cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMReaderTest)

cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0043 NEW)

include_directories(../FLIMReader)


# Source Files
#====================================
set(SOURCE
   FLIMReaderTest.cpp
)

set(HEADERS
   FLIMReaderTest.h
)

add_executable(FLIMReaderTest ${SOURCE} ${HEADERS} $<TARGET_OBJECTS:FLIMReader>)
