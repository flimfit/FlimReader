cmake_minimum_required(VERSION 3.0)

project(FLIMreader)

if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(UNIX)

# CMakeLists.txt
add_subdirectory(FLIMReader)
add_subdirectory(FLIMReaderTest)
add_subdirectory(FLIMReaderMex)
