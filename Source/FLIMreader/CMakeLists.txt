cmake_minimum_required(VERSION 3.0)

PROJECT(FLIMreader)

cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0043 NEW)

# Source Files
#====================================
set(SOURCE
   FLIMreader.cpp
)

set(HEADERS
   FLIMreader.h
)

add_library(FLIMreader OBJECT ${SOURCE} ${HEADERS})
