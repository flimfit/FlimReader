cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMreader)

if(WIN32)
   set( Boost_USE_STATIC_LIBS ON )
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIR})

set(SOURCE
   FLIMreader.cpp
   AbstractPicoquantReader.cpp
   PicoquantTTRReader.cpp
   PicoquantPTUReader.cpp
   TextReader.cpp
)

set(HEADERS
   FLIMreader.h
   AbstractPicoquantReader.h
   PicoquantTTRReader.h
   PicoquantPTUReader.h
   PicoquantHelper.h
   TextReader.h
)

set(FLIMreader_LIBRARIES ${Boost_LIBRARIES} PARENT_SCOPE)
add_library(FLIMreader OBJECT ${SOURCE} ${HEADERS})

