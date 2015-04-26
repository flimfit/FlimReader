cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMReader)

cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0043 NEW)

set(SOURCE
   FLIMReader.cpp
   PicoquantTTRReader.cpp
   TextReader.cpp
)

set(HEADERS
   FLIMReader.h
   PicoquantTTRReader.h
   TextReader.h
)

add_library(FLIMReader OBJECT ${SOURCE} ${HEADERS})
