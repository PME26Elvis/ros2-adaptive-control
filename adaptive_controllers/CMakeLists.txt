cmake_minimum_required(VERSION 3.16)
project(adaptive_controllers)

# ---- dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rcutils REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# ---- library ----
add_library(${PROJECT_NAME} SHARED
  src/adaptive_state_feedback_controller.cpp
  src/observers/luenberger_observer.cpp
  src/adaptive_laws/mrac_law.cpp
  src/param_utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  controller_interface
  hardware_interface
  realtime_tools
  rcutils
)

# ---- plugin exports ----
pluginlib_export_plugin_description_file(controller_interface plugin_description.xml)
pluginlib_export_plugin_description_file(adaptive_controllers observer_plugins.xml)

# ---- install ----
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(FILES
  plugin_description.xml
  observer_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# 可有可無：若有 resource/ 目錄就安裝；沒有也不會報錯
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource")
  install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})
endif()

# ---- testing (no ament_lint_auto here) ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_compile tests/test_compile.cpp)
  target_link_libraries(test_compile ${PROJECT_NAME})

  ament_add_gtest(test_controller_lifecycle tests/test_controller_lifecycle.cpp)
  target_link_libraries(test_controller_lifecycle ${PROJECT_NAME})
endif()

# ---- exports ----
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  pluginlib
  controller_interface
  hardware_interface
  realtime_tools
  rcutils
)

ament_package()
