cmake_minimum_required(VERSION 3.16)
project(adaptive_simulation)

find_package(ament_cmake REQUIRED)

# 只安裝實際存在的資料夾，避免 install 階段報錯
foreach(d launch worlds models resource)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${d}")
    install(DIRECTORY ${d} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
