cmake_minimum_required(VERSION 3.7)
project(FrameAligner)

find_package(OpenCV 3.1 REQUIRED core imgproc highgui )
find_package(OpenMP REQUIRED)
find_package(dlib CONFIG REQUIRED)

if(APPLE)
    set(EXTRA_LIBRARIES "-framework Accelerate") # for dlib
endif()

include_directories(${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${dlib_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

set(SOURCE
   AbstractFrameAligner.cpp
   RigidFrameAligner.cpp
   FrameWarpAligner.cpp
   WriteMultipageTiff.cpp
)

set(HEADERS
   AbstractFrameAligner.h
   RigidFrameAligner.h
   FrameWarpAligner.h
   LinearInterpolation.h
   WriteMultipageTiff.h
)

set(FrameAligner_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS} PARENT_SCOPE)
set(FrameAligner_LIBRARIES ${OpenCV_LIBS} ${dlib_LIBRARIES} ${EXTRA_LIBRARIES} PARENT_SCOPE)
add_library(FrameAligner OBJECT ${SOURCE} ${HEADERS})

