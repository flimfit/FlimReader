cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMreader)


if(MSVC)
  set( Boost_USE_STATIC_LIBS ON )
  add_definitions(-DBOOST_ALL_NO_LIB=1)
endif()

find_package(Boost CONFIG REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
find_package("OpenMP" REQUIRED)

# Enable OpenMP support, disable for XCode
#===================================================
find_package("OpenMP")
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
   FFDReader.cpp
   BhSPCReader.cpp
   TextReader.cpp
   lz4.c
   AbstractFrameAligner.cpp
   TransformInterpolator.cpp
   FrameWarpAligner.cpp
)

set(HEADERS
   FLIMreader.h
   AbstractFifoReader.h
   PicoquantTTRReader.h
   PicoquantPTUReader.h
   PicoquantBINReader.h
   PicoquantHelper.h
   FFDReader.h
   BhSPCReader.h
   TextReader.h
   PicoquantT3Event.h
   FifoProcessor.h
   lz4.h
   AbstractFrameAligner.h
   TransformInterpolator.h
   FrameWarpAligner.h
)

set(FLIMreader_LIBRARIES ${Boost_LIBRARIES} ${OpenCV_LIBS} PARENT_SCOPE)
add_library(FLIMreader OBJECT ${SOURCE} ${HEADERS})

set_property(TARGET FLIMreader PROPERTY CXX_STANDARD 14)

