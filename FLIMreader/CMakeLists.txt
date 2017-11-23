cmake_minimum_required(VERSION 3.7)
project(FlimReader)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV 3.1 REQUIRED core imgproc highgui )

include_directories(${Boost_INCLUDE_DIR} ${FrameAligner_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(SOURCE
   FlimReader.cpp
   AbstractFifoReader.cpp
   PicoquantTTRReader.cpp
   PicoquantPTUReader.cpp
   FfdReader.cpp
   BhFifoReader.cpp
   TextReader.cpp
   AligningReader.cpp
   AbstractEventReader.cpp
   FifoProcessor.cpp
   FfhReader.cpp
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
   BhFifoReader.h
   TextReader.h
   PicoquantT3Event.h
   FifoProcessor.h
   MetadataTag.h
   FlimCube.h
   FlimCubeWriter.h
   AligningReader.h
   AbstractEventReader.h
)

set(FlimReader_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} ${FrameAligner_INCLUDE_DIRS} PARENT_SCOPE)
set(FlimReader_LIBRARIES ${Boost_LIBRARIES} ${OpenCV_LIBS} ${FrameAligner_LIBRARIES} FrameAligner ${EXTRA_LIBRARIES} PARENT_SCOPE)
add_library(FlimReader OBJECT ${SOURCE} ${HEADERS})

