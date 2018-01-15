cmake_minimum_required(VERSION 3.7)

PROJECT(FlimReaderTest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

set(SOURCE
   FlimReaderTest.cpp
)

set(HEADERS
   FlimReaderTest.h
)

add_executable(FlimReaderTest ${SOURCE} ${HEADERS})
target_link_libraries(FlimReaderTest FlimReader)
