cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMReaderMex)

cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0043 NEW)

# MATLAB stuff
#====================================

if(UNIX)
   set(MATLAB_EXTERN_DIR "/Applications/MATLAB_R2014b.app/extern/")
   set(MATLAB_LIB_DIR ${MATLAB_EXTERN_DIR}lib/maci64/)
   set(MATLAB_LIBRARIES ${MATLAB_LIB_DIR}fexport.map ${MATLAB_LIB_DIR}mexFunction.map ${MATLAB_LIB_DIR}mexLibrary.map)
   set(MEX_EXT ".mexmaci64")
else(UNIX)
   set(MATLAB_EXTERN_DIR "C:/Program Files/MATLAB R2014b/extern/")
   set(MATLAB_LIB_DIR ${MATLAB_EXTERN_DIR}lib/win64/microsoft/)
   set(MATLAB_LIBRARIES ${MATLAB_LIB_DIR}libmat.lib ${MATLAB_LIB_DIR}libmex.lib ${MATLAB_LIB_DIR}libmx.lib)
   set(MEX_EXT ".mexw64")
endif(UNIX)
set(MATLAB_INCLUDE_DIR ${MATLAB_EXTERN_DIR}include)


include_directories(${MATLAB_INCLUDE_DIR})
include_directories(../FLIMReader)

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

if (OUT_DIR)
   add_custom_command(TARGET FLIMReaderMex POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:FLIMReaderMex> ${OUT_DIR})
endif (OUT_DIR)