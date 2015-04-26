cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMReaderMex)

cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0043 NEW)

# MATLAB stuff
#====================================
set(MATLAB_EXTERN_DIR "E:/Program Files/extern/")
set(MATLAB_LIB_DIR ${MATLAB_EXTERN_DIR}lib/win64/microsoft/)
set(MATLAB_INCLUDE_DIR ${MATLAB_EXTERN_DIR}include)
set(MATLAB_LIBRARIES ${MATLAB_LIB_DIR}libmat.lib ${MATLAB_LIB_DIR}libmex.lib ${MATLAB_LIB_DIR}libmx.lib)


include_directories(${MATLAB_INCLUDE_DIR})
include_directories(../FLIMReader)

# Add platform specific flags
#===================================================
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PLATFORM_FLAGS_ALL}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${PLATFORM_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${PLATFORM_FLAGS_DEBUG}")

# Source Files
#====================================
set(SOURCE
   FLIMReaderMex.cpp
)

set(HEADERS
   MexUtils.h
)

add_library(FLIMReaderMex MODULE ${SOURCE} ${HEADERS} $<TARGET_OBJECTS:FLIMReader>)
add_definitions(-DMATLAB_MEX_FILE)

# Needed for entry point.
set_target_properties(FLIMReaderMex PROPERTIES LINK_FLAGS "/export:mexFunction" )

# Change the dll extension to a mex extension.
set_target_properties(FLIMReaderMex PROPERTIES SUFFIX ".mexw64")

target_link_libraries(FLIMReaderMex ${MATLAB_LIBRARIES})

if (OUT_DIR)
   add_custom_command(TARGET FLIMReaderMex POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:FLIMReaderMex> ${OUT_DIR})
endif (OUT_DIR)