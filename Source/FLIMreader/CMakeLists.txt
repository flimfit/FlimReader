cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMreader)

cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0043 NEW)

set(SOURCE
   PicoquantTTRReader.cpp
)

set(HEADERS
   FLIMreader.h
   PicoquantTTRReader.h
)

add_library(FLIMreader OBJECT ${SOURCE} ${HEADERS})
