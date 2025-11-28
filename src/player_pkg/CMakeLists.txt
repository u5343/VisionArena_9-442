cmake_minimum_required(VERSION 3.8)
project(player_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++17标准
set(CMAKE_CXX_STANDARD 17)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(pose_msg REQUIRED)
find_package(referee_pkg REQUIRED)

# ONNXRuntime路径（根据你的安装路径修改）
set(ONNXRUNTIME_ROOT_PATH "/home/jiangping/onnxruntime-linux-x64-1.23.2/include")
set(ONNXRUNTIME_LIB_PATH "/home/jiangping/onnxruntime-linux-x64-1.23.2/lib")

# 设置RPATH，使可执行文件能找到库
set(CMAKE_INSTALL_RPATH "${ONNXRUNTIME_LIB_PATH}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 包含目录
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_ROOT_PATH}
)

# 添加可执行文件
add_executable(yolo_detector_node src/camera_detect.cpp)

# 链接库
ament_target_dependencies(yolo_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs
  geometry_msgs
  OpenCV
  pose_msg
  referee_pkg
)

target_link_libraries(yolo_detector_node
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIB_PATH}/libonnxruntime.so
)
target_link_libraries(yolo_detector_node ${typesupport_target})
target_link_libraries(yolo_detector_node ${OpenCV_LIBS})
# 安装
install(TARGETS
  yolo_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()