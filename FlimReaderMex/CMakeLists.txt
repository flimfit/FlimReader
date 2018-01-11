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
find_package(OpenCV REQUIRED)
find_package(dlib REQUIRED)
find_package(CUDA QUIET)

set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_DEBUG_POSTFIX "")

include_directories(${FlimReader_INCLUDE_DIR})

set(SOURCE
   FlimReaderMex.cpp
)

set(HEADERS
   MexUtils.h
)

matlab_add_mex(NAME FlimReaderMex SRC ${SOURCE} ${HEADERS} $<TARGET_OBJECTS:FlimReader>)

target_link_libraries(FlimReaderMex ${FlimReader_LIBRARIES})

if(FlimReaderMEX_OUT_DIR)
   add_custom_command(TARGET FlimReaderMex POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:FlimReaderMex> ${FlimReaderMEX_OUT_DIR})
endif()