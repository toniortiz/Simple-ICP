cmake_minimum_required(VERSION 2.8)
project(icp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall   -O3 -march=native -pthread")

#------------------------------------------
#                    Eigen
#------------------------------------------
find_package(Eigen3 3.0 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

#------------------------------------------
#                    PCL
#------------------------------------------
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARY_DIRS})


include_directories(${PROJECT_SOURCE_DIR})

add_library(${PROJECT_NAME} SHARED
    Viewer/Viewer.cpp

    Utils/Random.cpp

    Algorithms/Icp.cpp
    Algorithms/Gicp.cpp
    Algorithms/AbstractIcp.cpp
)

target_link_libraries(${PROJECT_NAME}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
)

add_executable(main main.cpp)
target_link_libraries(main ${PROJECT_NAME})
