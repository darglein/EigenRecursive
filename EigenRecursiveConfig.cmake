


set(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_DIR}")

add_library(Eigen::EigenRecursive INTERFACE IMPORTED)

set_target_properties(Eigen::EigenRecursive PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES "Eigen3::Eigen"
)
