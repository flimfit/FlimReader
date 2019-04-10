cmake_minimum_required(VERSION 3.0)

option(BUILD_OPENCV "Build OpenCV (will produce static libs)" OFF)
option(GENERATE_FLIMREADER_MEX "Generate FlimReader Mex file" ON)

string(COMPARE EQUAL ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ROOT_PROJECT)
if (BUILD_OPENCV AND ROOT_PROJECT)
   include("cmake/HunterGate.cmake")
   HunterGate(
      URL "https://github.com/ruslo/hunter/archive/v0.20.13.tar.gz"
      SHA1 "d92042d6e22522516183b86da10e56fb1af73f7a"
   )
endif ()

project(FlimReader)
set(CMAKE_CXX_STANDARD 14)

include("cmake/ConfigureMSVCRuntime.cmake")
configure_msvc_runtime("static")

set(CMAKE_MAP_IMPORTED_CONFIG_MINSIZEREL Release)
set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release) 

if (BUILD_OPENCV)
    hunter_add_package(OpenCV)
    hunter_add_package(Tiff)
    # Prefer static libraries so we don't mess with Matlab 
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a .tbd .dylib .so)
endif ()

if(MSVC)
   set(PLATFORM_FLAGS_ALL "/Oi /fp:fast")
   set(PLATFORM_FLAGS_RELEASE "/Ox /Ot /Ob2")
   set(PLATFORM_FLAGS_RELWITHDEBINFO "/Ox /Ot /Ob2")
   set(PLATFORM_FLAGS_DEBUG "/ZI")
   add_definitions(-D_CRT_SECURE_NO_WARNINGS)
   add_definitions(-D_MSVC)
endif(MSVC)

if(UNIX)
   set(PLATFORM_FLAGS_ALL "-mavx2 -mfma")
   set(PLATFORM_FLAGS_RELEASE "-O3 -ffast-math -ftree-vectorize")
   set(PLATFORM_FLAGS_DEBUG "")
endif(UNIX)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PLATFORM_FLAGS_ALL}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${PLATFORM_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${PLATFORM_FLAGS_DEBUG}")

# CMakeLists.txt
add_subdirectory(frame-aligner/FrameAligner)
add_subdirectory(FlimReader)
add_subdirectory(FlimReaderTest)

if(GENERATE_FLIMREADER_MEX)
   add_subdirectory(FlimReaderMex)
endif()