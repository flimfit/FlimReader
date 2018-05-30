cmake_minimum_required(VERSION 3.9)

project(FlimReaderMex)

if(DEFINED ENV{MATLAB_VER})
	if(UNIX)
		set(Matlab_ROOT_DIR "/Applications/Matlab_$ENV{MATLAB_VER}.app")
	else()
		set(Matlab_ROOT_DIR "C:/Program Files/MATLAB/$ENV{MATLAB_VER}")
	endif()
endif()
message("Set Matlab root: ${Matlab_ROOT_DIR}")

set(MATLAB_FIND_DEBUG TRUE)
find_package(Matlab COMPONENTS MEX_COMPILER MX_LIBRARY REQUIRED)

set(CMAKE_DEBUG_POSTFIX "")

set(SOURCE
   FlimReaderMex.cpp
)

set(HEADERS
   MexUtils.h
)

matlab_add_mex(NAME FlimReaderMex SRC ${SOURCE} ${HEADERS} LINK_TO FlimReader)

if (FlimReaderMEX_OUT_DIR)
   add_custom_command(TARGET FlimReaderMex POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory $<TARGET_FILE_DIR:FlimReaderMex> ${FlimReaderMEX_OUT_DIR})
endif()
