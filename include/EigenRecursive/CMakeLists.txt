
# Collect all files in this directory
FILE(GLOB_RECURSE ALL_SRC  "*.cpp" "*.cu" "*.cc")
FILE(GLOB_RECURSE ALL_HEADER  "*.h" "*.hpp" "*.inl")
set(ALL_FILES ${ALL_SRC} ${ALL_HEADER})

