cmake_minimum_required(VERSION 3.8)
project(time_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)

# 実行可能ファイルの作成
add_executable(extract_time_node src/extract_timestamp.cpp)
ament_target_dependencies(extract_time_node 
  rclcpp 
  rosbag2_cpp 
  event_camera_msgs 
  event_camera_codecs
)

# インストールターゲットの指定
install(TARGETS
  extract_time_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
