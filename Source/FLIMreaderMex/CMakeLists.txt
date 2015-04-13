cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMreaderMex)

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
include_directories(../FLIMreader)

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
target_link_libraries(FLIMreaderMex ${MATLAB_LIBRARIES})

# Needed for entry point.
if(MSVC)
set_target_properties(FLIMreaderMex PROPERTIES LINK_FLAGS "/export:mexFunction" )
endif(MSVC)

# Change the dll extension to a mex extension.
set_target_properties(FLIMreaderMex PROPERTIES SUFFIX ${MEX_EXT})

if (OUT_DIR)
   add_custom_command(TARGET FLIMreaderMex POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:FLIMreaderMex> ${OUT_DIR})
endif (OUT_DIR)