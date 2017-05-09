cmake_minimum_required(VERSION 3.7)

PROJECT(FlimReader)


if(MSVC)
  set( Boost_USE_STATIC_LIBS ON )
  add_definitions(-DBOOST_ALL_NO_LIB=1)
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED core imgproc highgui)
find_package(OpenMP REQUIRED)
find_package(dlib CONFIG REQUIRED)

include_directories(${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${dlib_INCLUDE_DIRS})

# Enable OpenMP support, disable for XCode
#===================================================
if (OPENMP_FOUND)
   option(USE_OMP "Use OpenMP" ON)
   if (USE_OMP)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DUSE_OMP")
      message("Using OpenMP")
   endif()
endif()


set(SOURCE
   FlimReader.cpp
   AbstractFifoReader.cpp
   PicoquantTTRReader.cpp
   PicoquantPTUReader.cpp
   FfdReader.cpp
   BhSPCReader.cpp
   TextReader.cpp
   AbstractFrameAligner.cpp
   RigidFrameAligner.cpp
   FrameWarpAligner.cpp
   WriteMultipageTiff.cpp
)

set(HEADERS
   FlimReader.h
   AbstractFifoReader.h
   PicoquantTTRReader.h
   PicoquantPTUReader.h
   PicoquantBINReader.h
   PicoquantHelper.h
   FfdReader.h
   FfhReader.h
   FfdHeader.h
   HistogramReader.h
   BhSPCReader.h
   TextReader.h
   PicoquantT3Event.h
   FifoProcessor.h
   AbstractFrameAligner.h
   RigidFrameAligner.h
   FrameWarpAligner.h
   MetadataTag.h
   FlimCube.h
   FlimCubeWriter.h
   LinearInterpolation.h
   WriteMultipageTiff.h
)

set(FlimReader_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS  } PARENT_SCOPE)
set(FlimReader_LIBRARIES ${Boost_LIBRARIES} ${OpenCV_LIBS} ${dlib_LIBRARIES} PARENT_SCOPE)
add_library(FlimReader OBJECT ${SOURCE} ${HEADERS})

