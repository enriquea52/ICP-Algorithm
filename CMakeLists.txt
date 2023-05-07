cmake_minimum_required(VERSION 2.8)
project(ICP)

#find_package(nanoflann)
find_package( PCL 1.10 REQUIRED )
find_package(Eigen3 3.4 REQUIRED)
find_package(OpenMP)


include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

add_executable(ICP src/main.cpp src/visualization.cpp src/scanMatching.cpp nanoflann.hpp)
target_link_libraries(ICP ${PCL_LIBRARIES} )
target_include_directories(ICP PRIVATE ${EIGEN3_INCLUDE_DIR} )


add_executable(kitti src/trajectory.cpp src/visualization.cpp src/scanMatching.cpp nanoflann.hpp)
target_link_libraries(kitti ${PCL_LIBRARIES} )
target_include_directories(kitti PRIVATE ${EIGEN3_INCLUDE_DIR} )

target_compile_features(kitti PRIVATE cxx_std_17)