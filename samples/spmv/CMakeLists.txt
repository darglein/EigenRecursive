
set(PROG_NAME sample_spmv)

FILE(GLOB_RECURSE PROG_SRC  *.cpp)
FILE(GLOB_RECURSE PROG_HEADER  *.h)


#creates 'folders' in the solution explorer of visual studio
GroupSources2(${PROJECT_SOURCE_DIR}/src)

include_directories(.)

message(STATUS "\n libs")
message(STATUS "${LIBS} ${LIB_TARGETS}")


add_executable(${PROG_NAME} ${PROG_SRC} ${PROG_HEADER})
target_link_libraries(${PROG_NAME} ${LIBS} ${LIB_TARGETS})



target_compile_features(${PROG_NAME} PUBLIC cxx_std_17)
