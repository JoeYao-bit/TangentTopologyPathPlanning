cmake_minimum_required(VERSION 2.8)

project(TangentTopologyPathPlanning)
enable_testing()
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++14")
    add_definitions(-DCOMPILEDWITHC14)
    message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


add_subdirectory(freeNav-base)

aux_source_directory(algorithm alg_src)
add_library(alg_lib ${alg_src})

aux_source_directory(test/visualization vis_src)
add_library(vis_lib ${vis_src}  ${EIGEN3_LIBS})

aux_source_directory(third_party/dynamicvoronoi third1_src)
aux_source_directory(third_party/DynamicVoronoi2D third2_src)
aux_source_directory(third_party/ENL-SVG/src third3_src)

add_library(external_lib ${third1_src} ${third2_src} ${third3_src})

add_executable(test_tangent_graph_build test/test_2D_path_planning.cpp)
target_link_libraries(test_tangent_graph_build freeNav-base alg_lib vis_lib external_lib)

add_executable(test_massive_comparison test/test_massive_comparison_2d.cpp)
target_link_libraries(test_massive_comparison freeNav-base alg_lib vis_lib external_lib)
