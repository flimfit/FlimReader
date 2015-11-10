cmake_minimum_required(VERSION 3.0)

project(FLIMreaderMex)

# Find MATLAB
set(MATLAB_ADDITIONAL_VERSIONS
    "R2015b=8.6"
    "R2015a=8.5"
    "R2014b=8.4")

if(DEFINED ENV{MATLAB_VER})
	if(UNIX)
		set(Matlab_ROOT_DIR "/Applications/Matlab_R$ENV{MATLAB_VER}.app")
	else()
		set(Matlab_ROOT_DIR "C:/Program Files/MATLAB/R$ENV{MATLAB_VER}")
	endif()
endif()

set(MATLAB_FIND_DEBUG TRUE)
find_package(Matlab COMPONENTS MEX_COMPILER MX_LIBRARY REQUIRED)

include_directories(../FLIMreader)

set(SOURCE
   FLIMreaderMex.cpp
)

set(HEADERS
   MexUtils.h
)

matlab_add_mex(NAME FLIMreaderMex SRC ${SOURCE} ${HEADERS} $<TARGET_OBJECTS:FLIMreader>)

target_link_libraries(FLIMreaderMex ${FLIMreader_LIBRARIES})

if(FLIMreaderMEX_OUT_DIR)
   add_custom_command(TARGET FLIMreaderMex POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:FLIMreaderMex> ${CMAKE_BINARY_DIR}/${FLIMreaderMEX_OUT_DIR})
endif()