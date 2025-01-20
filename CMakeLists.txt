cmake_minimum_required(VERSION 3.10)
project(rviz_pac)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_rendering REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES_PAC_STATUS_PANEL include/rviz_pac/pac_status_panel.hpp)

add_library(pac_status_panel src/pac_status_panel.cpp ${MOC_FILES_PAC_STATUS_PANEL})
target_include_directories(pac_status_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ament_export_dependencies(
#   geometry_msgs
#   rclcpp
#   pluginlib
#   rviz_common
#   rviz_rendering
#   sensor_msgs)

ament_target_dependencies(pac_status_panel
  pluginlib
  rviz_common
)

qt5_wrap_cpp(MOC_FILES_TELEOP_PANEL include/rviz_pac/xy_drive_widget.hpp include/rviz_pac/xy_teleop_panel.hpp)
add_library(xy_teleop_panel SHARED src/xy_drive_widget.cpp src/xy_teleop_panel.cpp ${MOC_FILES_TELEOP_PANEL})
target_include_directories(xy_teleop_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(xy_teleop_panel
  pluginlib
  rviz_common
  geometry_msgs
  rviz_rendering
  rclcpp
)
install(TARGETS pac_status_panel
        EXPORT export_rviz_pac_pac_status_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(TARGETS xy_teleop_panel
        EXPORT export_rviz_pac_teleop_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_pac_plugin.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/PACStatusPanel.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_rviz_pac_pac_status_panel)
ament_export_targets(export_rviz_pac_teleop_panel)
pluginlib_export_plugin_description_file(rviz_common rviz_pac_plugin.xml)

ament_package()
