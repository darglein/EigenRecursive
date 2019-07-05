cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

set(LIB_NAME EigenRecursive)
set(EXPORT_NAME EigenRecursiveExport)

project (${LIB_NAME} CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH} )
include(helper_macros)

############# Build Options ###############

message(STATUS "\nBuild Options")


DefaultBuildType(RelWithDebInfo)



if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
else()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0")
endif()


if(MSVC)
    #multiprocessor compilation for visual studio
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
else()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()
############# Required LIBRARIES ###############


#eigen
find_package(Eigen3 REQUIRED)
PackageHelperTarget(Eigen3::Eigen EIGEN3_FOUND)


#eigen
find_package(Sophus QUIET)
PackageHelperTarget(Sophus::Sophus SOPHUS_FOUND)


include_directories(include/)

add_subdirectory(include/)
add_subdirectory(samples)



