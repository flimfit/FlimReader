cmake_minimum_required(VERSION 3.3)

project(FlimReaderMex)
set(CMAKE_DEBUG_POSTFIX "")

# Find MATLAB
set(MATLAB_ADDITIONAL_VERSIONS
    "R2016a=9.1"
    "R2016a=9.0"
    "R2015b=8.6"
    "R2015a=8.5"
    "R2014b=8.4")

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
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")


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