# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "AUKFProject_autogen"
  "CMakeFiles/AUKFProject_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/AUKFProject_autogen.dir/ParseCache.txt"
  )
endif()
