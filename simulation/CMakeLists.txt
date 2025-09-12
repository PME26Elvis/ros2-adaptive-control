cmake_minimum_required(VERSION 3.16)
project(adaptive_simulation)
find_package(ament_cmake REQUIRED)

# Only install dirs that actually exist
foreach(d launch worlds models resource)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${d}")
    install(DIRECTORY ${d} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
