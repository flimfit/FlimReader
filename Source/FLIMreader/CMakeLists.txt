cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMReader)

cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0043 NEW)

set( Boost_USE_STATIC_LIBS ON )
set(BOOST_ROOT "C:/local/boost_1_59_0")
set(BOOST_LIBRARYDIR "C:/local/boost_1_59_0/lib64-msvc-12.0/")

find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIR})

set(SOURCE
   FLIMReader.cpp
   AbstractPicoquantReader.cpp
   PicoquantTTRReader.cpp
   PicoquantPTUReader.cpp
   TextReader.cpp
)

set(HEADERS
   FLIMReader.h
   AbstractPicoquantReader.h
   PicoquantTTRReader.h
   PicoquantPTUReader.h
   PicoquantHelper.h
   TextReader.h
)

set(FLIMReader_LIBRARIES ${Boost_LIBRARIES} PARENT_SCOPE)
add_library(FLIMReader OBJECT ${SOURCE} ${HEADERS})

