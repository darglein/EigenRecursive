cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

set(LIB_NAME EigenRecursive)

project (${LIB_NAME} CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH} )
include(helper_macros)

############# Build Options ###############

message(STATUS "\nBuild Options")


DefaultBuildType(RelWithDebInfo)


############# Required LIBRARIES ###############


#eigen
find_package(Eigen3 REQUIRED)
PackageHelperTarget(Eigen3::Eigen EIGEN3_FOUND)



include_directories(include/)

add_subdirectory(include/EigenRecursive)
add_subdirectory(samples)



