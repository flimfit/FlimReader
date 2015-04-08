cmake_minimum_required(VERSION 3.0)

project(FLIMreader)

# Set output directories
#====================================
set(OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/../)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY                ${OUT_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG          ${OUT_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE        ${OUT_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${OUT_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${OUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${OUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        ${OUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${OUT_DIR})


# CMakeLists.txt
add_subdirectory(FLIMReader)
add_subdirectory(FLIMReaderTest)
add_subdirectory(FLIMReaderMex)
