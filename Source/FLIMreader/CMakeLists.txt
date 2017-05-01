cmake_minimum_required(VERSION 3.3)

PROJECT(FLIMreader)


if(MSVC)
  set( Boost_USE_STATIC_LIBS ON )
  add_definitions(-DBOOST_ALL_NO_LIB=1)
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED core imgproc)
find_package(OpenMP REQUIRED)
find_package(dlib REQUIRED)

include_directories(${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

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
   FLIMreader.cpp
   AbstractFifoReader.cpp
   PicoquantTTRReader.cpp
   PicoquantPTUReader.cpp
   FfdReader.cpp
   BhSPCReader.cpp
   TextReader.cpp
   AbstractFrameAligner.cpp
   RigidFrameAligner.cpp
   FrameWarpAligner.cpp
)

set(HEADERS
   FLIMreader.h
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
)

set(FLIMreader_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS} PARENT_SCOPE)
set(FLIMreader_LIBRARIES ${Boost_LIBRARIES} ${OpenCV_LIBS} PARENT_SCOPE)
add_library(FLIMreader OBJECT ${SOURCE} ${HEADERS})

