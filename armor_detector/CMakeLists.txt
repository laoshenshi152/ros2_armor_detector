cmake_minimum_required(VERSION 3.8)
project(armor_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)


# 添加可执行文件
add_executable(armor_detector_node
  src/armor_detector_node.cpp  # 你之后需要补上这个文件
  src/detector.cpp
  src/tracker.cpp
  src/kalmanfilter.cpp
)

add_executable(video_publisher_node
  src/video_publisher_node.cpp
)

# 设置 include 目录
target_include_directories(armor_detector_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
)

target_include_directories(video_publisher_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 链接 OpenCV
target_link_libraries(armor_detector_node ${OpenCV_LIBS})
target_link_libraries(video_publisher_node ${OpenCV_LIBS})

# 添加 ROS 依赖
ament_target_dependencies(armor_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
)

ament_target_dependencies(video_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
)

# 安装目标
install(TARGETS
  armor_detector_node
  video_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
