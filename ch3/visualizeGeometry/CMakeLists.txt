cmake_minimum_required( VERSION 2.8 )
project( visualizeGeometry )

set(CMAKE_CXX_FLAGS "-std=c++11")

# add Eigen header
include_directories( "/usr/include/eigen3" )

# add Pangolin include
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( visualizeGeometry visualizeGeometry.cpp )
target_link_libraries( visualizeGeometry ${Pangolin_LIBRARIES} )
