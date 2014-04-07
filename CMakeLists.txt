cmake_minimum_required(VERSION 2.4)

project(voxel-collision-checker-plugin)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

message(STATUS "CMAKE_SOURCE_DIR : " ${CMAKE_SOURCE_DIR})
message(STATUS "CMAKE_MODULE_PATH : " ${CMAKE_MODULE_PATH})

# Default mode is Debug
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

SET(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib/$ENV{HOSTTYPE}")

# Default Flags
ADD_DEFINITIONS(-DKCD_MOVE3D -DPRINT_ERROR -DPRINT_INFO)

IF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  ADD_DEFINITIONS(-g -Wall -Wno-unknown-pragmas -fstack-protector-all)
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  ADD_DEFINITIONS(-O2 -Wall -Wno-unknown-pragmas)
ENDIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")

# Find packages

find_package(OpenRAVE REQUIRED)

if( NOT OPENRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing ")
endif()
add_definitions("-g ")

cmake_minimum_required(VERSION 2.4.6)

file(GLOB source_files src/*.cpp)

add_library( voxelcollcheck SHARED ${source_files} )
set_target_properties( voxelcollcheck PROPERTIES LINK_FLAGS ${OPENRAVE_LINK_FLAGS} COMPILE_FLAGS ${OPENRAVE_CXXFLAGS})
include_directories(${OPENRAVE_INCLUDE_DIRS} src/)
target_link_libraries( voxelcollcheck ${OPENRAVE_LIBRARY} )

set(PLUGIN_INSTALL_DIR "${CMAKE_SOURCE_DIR}/plugins")
install(TARGETS voxelcollcheck DESTINATION ${PLUGIN_INSTALL_DIR} )

