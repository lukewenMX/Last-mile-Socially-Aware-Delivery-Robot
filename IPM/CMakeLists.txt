cmake_minimum_required( VERSION 2.8 )
project( ipm )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )

include_directories( 
    ${OpenCV_DIRS}
    ${EIGEN3_INCLUDE_DIR} 
)


add_executable( ipm IPM.cpp )
target_link_libraries( ipm 
    ${OpenCV_LIBS}
)
