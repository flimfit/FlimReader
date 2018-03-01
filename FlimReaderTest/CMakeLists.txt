cmake_minimum_required(VERSION 3.7)

project(FlimReaderTest)


set(SOURCE
   FlimReaderTest.cpp
)

set(HEADERS
   FlimReaderTest.h
)

add_executable(FlimReaderTest ${SOURCE} ${HEADERS})
target_link_libraries(FlimReaderTest FlimReader)
