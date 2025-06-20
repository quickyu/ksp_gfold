cmake_minimum_required(VERSION 3.13)
project(gfold VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE OFF)

set(CMAKE_BUILD_TYPE "Release")

find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(matplotlibcpp17 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)

add_subdirectory(scripts/solver/c_code/c/)

add_executable(test_solver
        src/test_solver.cpp
        )

target_include_directories(test_solver PUBLIC
        ${cpg_include}
        ${Python3_INCLUDE_DIRS})

target_link_libraries(test_solver
        cpg
        ${Python3_LIBRARIES} 
        pybind11::embed
        matplotlibcpp17::matplotlibcpp17
        )

add_executable(ksp_gfold
        src/ksp_gfold.cpp
        src/gfold_solver.cpp
        src/draw_trajectory.cpp
        src/MiniPID.cpp
        )        

target_include_directories(ksp_gfold PUBLIC
        ${cpg_include}
        ${ZLIB_INCLUDE_DIRS} 
        ${Protobuf_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR})

target_link_libraries(ksp_gfold
        cpg
        ${ZLIB_LIBRARIES} 
        ${Protobuf_LIBRARIES} 
        krpc
        glog::glog
        ${YAML_CPP_LIBRARIES}
        )