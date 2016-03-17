cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMreader)

if(WIN32)
   set( Boost_USE_STATIC_LIBS ON )
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIR})

set(SOURCE
   FLIMreader.cpp
   AbstractFifoReader.cpp
   PicoquantTTRReader.cpp
   PicoquantPTUReader.cpp
   FFDReader.cpp
   TextReader.cpp
   lz4.c
)

set(HEADERS
   FLIMreader.h
   AbstractFifoReader.h
   PicoquantTTRReader.h
   PicoquantPTUReader.h
   PicoquantHelper.h
   FFDReader.h
   TextReader.h
   PicoquantT3Event.h
   lz4.h
)

set(FLIMreader_LIBRARIES ${Boost_LIBRARIES} PARENT_SCOPE)
add_library(FLIMreader OBJECT ${SOURCE} ${HEADERS})

