cmake_minimum_required(VERSION 3.7)
project(FlimReader)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(ZLIB REQUIRED)
find_package(libics REQUIRED)

set(SOURCE
   FlimReader.cpp
   AbstractFifoReader.cpp
   PicoquantTTRReader.cpp
   PicoquantPTUReader.cpp
   FfdReader.cpp
   BhFifoReader.cpp
   TextReader.cpp
   SdtReader.cpp
   AligningReader.cpp
   AbstractEventReader.cpp
   FifoProcessor.cpp
   FfhReader.cpp
   FlimCube.cpp
   IcsReader.cpp
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
   SdtReader.h
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
   IcsReader.h
)

add_library(FlimReader STATIC ${SOURCE} ${HEADERS})
target_link_libraries(FlimReader ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} FrameAligner libics)
target_include_directories(FlimReader INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} PUBLIC ${ZLIB_INCLUDE_DIRS})
