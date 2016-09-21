cmake_minimum_required(VERSION 3.3)

PROJECT(FLIMreader)

if(MSVC)
  set( Boost_USE_STATIC_LIBS ON )
  add_definitions(-DBOOST_ALL_NO_LIB=1)
endif()

find_package(Boost CONFIG REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

set(SOURCE
   FLIMreader.cpp
   AbstractFifoReader.cpp
   PicoquantTTRReader.cpp
   PicoquantPTUReader.cpp
   FFDReader.cpp
   BhSPCReader.cpp
   TextReader.cpp
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
)

set(FLIMreader_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
set(FLIMreader_LIBRARIES ${Boost_LIBRARIES} ${OpenCV_LIBS} PARENT_SCOPE)
add_library(FLIMreader OBJECT ${SOURCE} ${HEADERS})

