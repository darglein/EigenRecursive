
FILE(GLOB_RECURSE HEADERS  "*.h" "*.hpp" "*.inl")

add_library(${LIB_NAME} INTERFACE)
#target_sources(${LIB_NAME} INTERFACE ${HEADERS})
#target_include_directories(${LIB_NAME} INTERFACE .)
target_link_libraries(${LIB_NAME} INTERFACE ${LIB_TARGETS})


set(INSTALL_INCLUDE   "include/EigenRecursive")

target_include_directories(
    ${LIB_NAME}
    INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/>
    )

install(
    TARGETS ${LIB_NAME}
    EXPORT ${LIB_NAME}
    DESTINATION "."
    )

install(
    EXPORT ${LIB_NAME}
    DESTINATION "share/EigenRecursive"
    )


install(
    FILES "${PROJECT_SOURCE_DIR}/cmake/EigenRecursiveConfig.cmake"
    DESTINATION "share/EigenRecursive"
    )


install(
    DIRECTORY ./EigenRecursive/
    DESTINATION ${INSTALL_INCLUDE}
    FILES_MATCHING PATTERN "*.h"
    PATTERN "*.inl"
    PATTERN "*.hpp"
    )
