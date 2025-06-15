cmake_minimum_required(VERSION 3.10)
project(rviz_pac)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(async_pac_gnn_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES_PAC_STATUS_PANEL include/rviz_pac/pac_status_panel.hpp)

add_library(pac_status_panel src/pac_status_panel.cpp ${MOC_FILES_PAC_STATUS_PANEL})
target_include_directories(pac_status_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pac_status_panel
  pluginlib
  rviz_common
  async_pac_gnn_interfaces
  rclcpp_action
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

qt5_wrap_cpp(MOC_FILES_TF include/rviz_pac/tf/frame_info.hpp include/rviz_pac/tf/tf_display.hpp)
add_library(tf_display SHARED src/tf/frame_info.cpp src/tf/tf_display.cpp src/tf/frame_selection_handler.cpp ${MOC_FILES_TF})
target_include_directories(tf_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tf_display
  pluginlib
  rviz_common
  rviz_rendering
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  rviz_default_plugins
)

# ament_export_targets(tf_display HAS_LIBRARY_TARGET)

target_compile_definitions(tf_display PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
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

install(TARGETS tf_display
  EXPORT export_rviz_pac_tf_display
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)
install(FILES rviz_pac_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_targets(export_rviz_pac_pac_status_panel)
ament_export_targets(export_rviz_pac_teleop_panel)
ament_export_targets(export_rviz_pac_tf_display)
pluginlib_export_plugin_description_file(rviz_common rviz_pac_plugin.xml)

ament_package()
