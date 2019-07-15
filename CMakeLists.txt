cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

set(LIB_NAME EigenRecursive)
set(EXPORT_NAME EigenRecursiveExport)

project (${LIB_NAME} CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH} )
include(helper_macros)

############# Build Options ###############

message(STATUS "\nBuild Options")
DefaultBuildType(Release)
message(STATUS "CXX Compiler: ${CMAKE_CXX_COMPILER_ID}")


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


find_package(MKL QUIET)
if(MKL_FOUND)
    add_definitions(-DSAIGA_USE_MKL)
endif()
PackageHelper(MKL "${MKL_FOUND}" "${MKL_INCLUDE_DIR}" "${MKL_LIBRARIES}")
include_directories(${PACKAGE_INCLUDES})
include_directories(include/)

add_subdirectory(include/)
add_subdirectory(samples)



