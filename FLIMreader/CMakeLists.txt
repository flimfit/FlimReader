cmake_minimum_required(VERSION 3.7)
project(FlimReader)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(zlib REQUIRED)

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
   FlimCube.cpp
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

add_library(FlimReader STATIC ${SOURCE} ${HEADERS})
target_link_libraries(FlimReader ${Boost_LIBRARIES} FrameAligner)
target_include_directories(FlimReader INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} PUBLIC ${ZLIB_INCLUDE_DIRS})
