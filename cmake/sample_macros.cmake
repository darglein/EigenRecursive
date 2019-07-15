macro(make_sample)

    # PREFIX: The directory name of the
    get_filename_component(PARENT_DIR ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
    get_filename_component(PREFIX ${PARENT_DIR} NAME)
    # Create target name from directory name
    get_filename_component(PROG_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
    string(REPLACE " " "_" PROG_NAME ${PROG_NAME})

    set(PROG_NAME "${PREFIX}_${PROG_NAME}")

    message(STATUS "Sample enabled:      ${PROG_NAME}")

    # Collect source and header files
    FILE(GLOB main_SRC  *.cpp)
    FILE(GLOB cuda_SRC  *.cu)
    FILE(GLOB main_HEADER  *.h)
    FILE(GLOB main_HEADER2  *.hpp)
    SET(PROG_SRC ${main_SRC} ${cuda_SRC} ${main_HEADER} ${main_HEADER2})


    add_executable(${PROG_NAME} ${PROG_SRC} )

    target_link_libraries(${PROG_NAME} ${LIB_NAME})
    target_link_libraries(${PROG_NAME} ${LIBS})
    # We only need to link the saiga target
    #message(STATUS "${${_modules}}")
    #target_link_libraries(${PROG_NAME} ${${_modules}})
    #target_link_libraries(${PROG_NAME} saiga_vision saiga_vulkan saiga_opengl)
    #target_link_libraries(${PROG_NAME} saiga_core )

    #set working directory for visual studio so the project can be executed from the ide
    set(OUTPUT_DIR ${CMAKE_BINARY_DIR})
    set_target_properties(${PROG_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${OUTPUT_DIR}")
    set_target_properties(${PROG_NAME} PROPERTIES FOLDER samples/${PREFIX})
    set_target_properties(${PROG_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}")

    target_compile_features(${PROG_NAME} PUBLIC cxx_std_17)
endmacro()
