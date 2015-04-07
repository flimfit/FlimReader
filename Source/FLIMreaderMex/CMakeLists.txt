cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMreaderMex)

cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0043 NEW)

# MATLAB stuff
#====================================
set(MATLAB_EXTERN_DIR "E:/Program Files/extern/")
set(MATLAB_LIB_DIR ${MATLAB_EXTERN_DIR}lib/win64/microsoft/)
set(MATLAB_INCLUDE_DIR ${MATLAB_EXTERN_DIR}include)
set(MATLAB_LIBRARIES ${MATLAB_LIB_DIR}libmat.lib ${MATLAB_LIB_DIR}libmex.lib ${MATLAB_LIB_DIR}libmx.lib)


include_directories(${MATLAB_INCLUDE_DIR})
include_directories(../FLIMreader)

# Add platform specific flags
#===================================================
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PLATFORM_FLAGS_ALL}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${PLATFORM_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${PLATFORM_FLAGS_DEBUG}")

# Source Files
#====================================
set(SOURCE
   FLIMreaderMex.cpp
)

set(HEADERS
   MexUtils.h
)

add_library(FLIMreaderMex MODULE ${SOURCE} ${HEADERS} $<TARGET_OBJECTS:FLIMreader>)
add_definitions(-DMATLAB_MEX_FILE)

# Needed for entry point.
set_target_properties(FLIMreader PROPERTIES LINK_FLAGS "/export:mexFunction" )

# Change the dll extension to a mex extension.
set_target_properties(FLIMreaderMex PROPERTIES SUFFIX ".mexw64")

target_link_libraries(FLIMreaderMex ${MATLAB_LIBRARIES})